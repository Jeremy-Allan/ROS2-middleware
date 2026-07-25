import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import math

# Arm Actions and Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, JointConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# Gripper Actions and Messages
from control_msgs.action import GripperCommand

# Services
from std_srvs.srv import Trigger
from example_interfaces.msg import Bool
from example_interfaces.srv import Trigger as ExampleTrigger

# Custom telemetry message
from kinova_interfaces.msg import ExtendedStatus
from kinova_interfaces.srv import HomeArm, MoveArm, MoveGripper, RelativeMove

# Controller Manager Messages
from controller_manager_msgs.srv import ListControllers
from kinova_interface.motion_lease import MotionLease, MotionLeaseError

# TF for Relative Movements
from tf2_ros import Buffer, TransformListener

class HardwareInterfaceClient(Node):
    def __init__(self):
        super().__init__('kinova_hardware_client')
        self.get_logger().info('Kinova Hardware Client Online - Waiting for Service Requests...')
        self.declare_parameter('use_fake_hardware', True)
        self.use_fake_hardware = self.get_parameter(
            'use_fake_hardware'
        ).value
        if not isinstance(self.use_fake_hardware, bool):
            raise ValueError('use_fake_hardware must be true or false')
        
        # Use a ReentrantCallbackGroup to allow service handlers and action callbacks to run concurrently
        self.callback_group = ReentrantCallbackGroup()
        # Legacy services share one result event, so they must never overlap.
        self.service_group = MutuallyExclusiveCallbackGroup()

        # Action Clients (The "Skills")
        self.arm_client = ActionClient(
            self, MoveGroup, 'move_action', 
            callback_group=self.callback_group
        )
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gen3_lite_2f_gripper_controller/gripper_cmd',
            callback_group=self.callback_group
        )

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Fault Monitoring and Recovery
        self.fault_sub = self.create_subscription(
            Bool,
            '/fault_controller/is_faulted',
            self.fault_callback,
            10,
            callback_group=self.callback_group
        )
        self.is_faulted = False
        self.fault_controller_warning_active = False
        # Kortex intentionally does not spawn its internal-bus fault
        # controller with fake hardware.
        self.fault_controller_active = self.use_fake_hardware
        self.interfaces_ready = False
        self.motion_active = False

        # Controller Manager Client for health monitoring of fault_controller
        self.list_controllers_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
            callback_group=self.callback_group
        )
        self.health_timer = self.create_timer(3.0, self.check_fault_controller_health, callback_group=self.callback_group)

        # Synchronous Movement Control
        self.movement_finished = threading.Event()
        self.movement_finished.set() 
        self.last_action_successful = False
        self.active_goal_handle = None
        self.active_gripper_target = None
        self.motion_recovery_required = False
        self.declare_parameter('motion_timeout_sec', 120.0)
        self.declare_parameter(
            'motion_lock_path',
            '/tmp/kinova_gen3_lite_motion.lock',
        )
        self.declare_parameter('gripper_min_position', 0.0)
        self.declare_parameter('gripper_max_position', 0.85)
        self.declare_parameter('gripper_closed_command_min', 0.75)
        self.declare_parameter('gripper_contact_position_min', 0.05)
        self.motion_timeout_sec = self.get_parameter('motion_timeout_sec').value
        self.gripper_min_position = self.get_parameter(
            'gripper_min_position'
        ).value
        self.gripper_max_position = self.get_parameter(
            'gripper_max_position'
        ).value
        self.gripper_closed_command_min = self.get_parameter(
            'gripper_closed_command_min'
        ).value
        self.gripper_contact_position_min = self.get_parameter(
            'gripper_contact_position_min'
        ).value
        if self.motion_timeout_sec <= 0.0:
            raise ValueError('motion_timeout_sec must be positive')
        if not (
            0.0 <= self.gripper_min_position
            < self.gripper_max_position
        ):
            raise ValueError('invalid gripper min/max positions')
        if not (
            self.gripper_min_position
            <= self.gripper_contact_position_min
            <= self.gripper_closed_command_min
            <= self.gripper_max_position
        ):
            raise ValueError('invalid gripper contact/closed thresholds')
        self.motion_lease = MotionLease(
            self.get_parameter('motion_lock_path').value,
            self.get_name(),
        )

        # Telemetry Setup
        self.status_pub = self.create_publisher(ExtendedStatus, '/status/node_report', 10)
        self.status_timer = self.create_timer(0.5, self.publish_status, callback_group=self.callback_group)
        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = (
            "Waiting for arm and gripper"
            if self.use_fake_hardware
            else "Waiting for arm, gripper, and fault controller"
        )
        self.command_success = False

        # ROS 2 Services (The "API")
        # Custom Service
        self.home_arm = self.create_service(HomeArm, '~/home_arm', self.handle_home_arm,callback_group=self.service_group)
        self.move_arm_srv = self.create_service(MoveArm, '~/move_arm', self.handle_move_arm, callback_group=self.service_group )
        self.move_gripper_srv = self.create_service(MoveGripper, '~/move_gripper', self.handle_move_gripper,callback_group=self.service_group)
        self.relative_move_srv = self.create_service(RelativeMove, '~/relative_move', self.handle_relative_move,callback_group=self.service_group)

    # --- Telemetry Status Publisher ---
    def publish_status(self):
        msg = ExtendedStatus()
        msg.node_name = self.get_name()
        msg.state = self.current_state
        msg.status_message = self.status_text
        msg.last_command_valid = self.command_success
        self.status_pub.publish(msg)

    def update_readiness_status(self):
        arm_ready = self.arm_client.server_is_ready()
        gripper_ready = self.gripper_client.server_is_ready()
        self.interfaces_ready = (
            arm_ready
            and gripper_ready
            and (self.use_fake_hardware or self.fault_controller_active)
        )
        if self.motion_active or self.is_faulted or self.motion_recovery_required:
            return
        if self.interfaces_ready:
            self.current_state = ExtendedStatus.STATE_IDLE
            self.status_text = "Hardware motion interfaces ready"
            self.command_success = True
        else:
            missing = []
            if not arm_ready:
                missing.append("MoveGroup action")
            if not gripper_ready:
                missing.append("gripper action")
            if not self.use_fake_hardware and not self.fault_controller_active:
                missing.append("fault_controller")
            self.current_state = ExtendedStatus.STATE_BUSY
            self.status_text = "Waiting for " + ", ".join(missing)
            self.command_success = False
        self.publish_status()

    # --- Fault Controller Health Check & Helper ---
    def check_fault_controller_health(self):
        """Timer callback to check if the fault_controller is active on the controller_manager."""
        self.update_readiness_status()
        if self.use_fake_hardware:
            return
        if not self.list_controllers_client.service_is_ready():
            self.get_logger().warn(
                "Controller manager '/list_controllers' service not ready!",
                throttle_duration_sec=10.0
            )
            return
        
        # Uses srv_type.Request() dynamically to avoid IDE/static analysis unresolved reference warnings
        req = self.list_controllers_client.srv_type.Request()
        future = self.list_controllers_client.call_async(req)
        future.add_done_callback(self.list_controllers_callback)

    def list_controllers_callback(self, future):
        try:
            response = future.result()
            fault_ctrl_active = False
            fault_ctrl_found = False
            for controller in response.controller:
                if controller.name == 'fault_controller':
                    fault_ctrl_found = True
                    if controller.state == 'active':
                        fault_ctrl_active = True
                    break
            
            if fault_ctrl_found and not fault_ctrl_active:
                self.fault_controller_active = False
                if not self.fault_controller_warning_active:
                    self.fault_controller_warning_active = True
                    self.get_logger().warn("fault_controller is present but NOT active!")
                    self.status_text = "SYSTEM CONFIG WARNING: fault_controller is NOT active"
                    self.publish_status()
            elif fault_ctrl_found and fault_ctrl_active:
                self.fault_controller_active = True
                if self.fault_controller_warning_active:
                    self.fault_controller_warning_active = False
                    self.get_logger().info("fault_controller is now active! Clearing config warning status.")
                    if self.status_text == "SYSTEM CONFIG WARNING: fault_controller is NOT active":
                        self.status_text = "Hardware Interface Client Ready"
            else:
                self.fault_controller_active = False
            self.update_readiness_status()
        except Exception as e:
            self.get_logger().error(f"Failed to query controllers health: {e}")

    def finalize_service_status(self, response):
        """Helper to centralize state & status updates after a service completes."""
        self.current_state = (
            ExtendedStatus.STATE_FAULT
            if self.is_faulted or self.motion_recovery_required
            else ExtendedStatus.STATE_IDLE
        )
        self.command_success = response.success
        if self.is_faulted or self.motion_recovery_required:
            # Preserve the more specific hardware fault status message set by the callback/diagnostics
            pass
        else:
            self.status_text = response.message
        self.publish_status()
        return response

    # --- Fault Handling ---
    def fault_callback(self, msg: Bool):
        """Asynchronously updates the internal fault status."""
        was_faulted = self.is_faulted
        self.is_faulted = bool(msg.data)
        if msg.data and not was_faulted:
            self.get_logger().error("Robot entered a hardware FAULT state.")
            if self.motion_active:
                self.motion_recovery_required = True
            if self.active_goal_handle is not None:
                try:
                    self.active_goal_handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().error(
                        f"Could not cancel motion after hardware fault: {exc}"
                    )
            self.current_state = ExtendedStatus.STATE_FAULT
            self.status_text = (
                "HARDWARE FAULT: active motion cancel requested"
                if self.active_goal_handle is not None
                else "HARDWARE FAULT: Robot is faulted"
            )
            self.command_success = False
        elif not msg.data and was_faulted:
            self.get_logger().info("Robot hardware fault has been cleared.")
            if self.motion_recovery_required:
                self.current_state = ExtendedStatus.STATE_FAULT
                self.status_text = (
                    "Hardware fault cleared, but motion state is uncertain; "
                    "reconcile and restart this node"
                )
                self.command_success = False
            else:
                self.update_readiness_status()
                return
        self.publish_status()

    def handle_moveit_failure(self):
        """Called when MoveIt execution fails."""
        self.get_logger().error("MoveIt trajectory execution failed. Inspecting hardware health...")
        # A MoveGroup goal was accepted, so a failed/unknown result may follow
        # partial physical execution. Retain the lease until operator recovery
        # and process restart instead of allowing a competing command.
        self.motion_recovery_required = True
        self.current_state = ExtendedStatus.STATE_FAULT

        if self.is_faulted:
            self.status_text = "MoveIt Failure: Hardware fault confirmed. Awaiting fault-reset command."
            self.publish_status()
            self.get_logger().error(self.status_text)
        else:
            self.status_text = (
                "MoveIt failed after accepting motion; robot state is uncertain. "
                "Reconcile state and restart this node."
            )
            self.publish_status()
            self.get_logger().error(self.status_text)

    # --- Service Handlers ---
    def begin_motion(self, response, label):
        self.update_readiness_status()
        if not self.interfaces_ready:
            response.success = False
            response.message = (
                "Robot motion interfaces are not ready; wait for MoveGroup, "
                "gripper, and the physical fault monitor"
            )
            self.current_state = ExtendedStatus.STATE_BUSY
            self.status_text = response.message
            self.command_success = False
            self.publish_status()
            return False
        if self.is_faulted:
            response.success = False
            response.message = "Robot hardware is faulted"
            self.current_state = ExtendedStatus.STATE_FAULT
            self.status_text = response.message
            self.command_success = False
            self.publish_status()
            return False
        if self.reject_if_motion_recovery_required(response):
            return False
        try:
            acquired = self.motion_lease.acquire(label)
        except MotionLeaseError as exc:
            response.success = False
            response.message = str(exc)
            self.command_success = False
            self.publish_status()
            return False
        if not acquired:
            response.success = False
            response.message = (
                "Robot motion is busy in another middleware process"
            )
            self.command_success = False
            self.publish_status()
            return False
        self.motion_active = True
        return True

    def end_motion(self):
        self.motion_active = False
        # Retain the cross-process lock if cancellation or timeout left the
        # physical state uncertain. Process restart releases it after recovery.
        if not self.motion_recovery_required:
            self.motion_lease.release()

    def reject_if_motion_recovery_required(self, response):
        if not self.motion_recovery_required:
            return False
        response.success = False
        response.message = (
            "A previous legacy motion timed out. Reconcile robot state and "
            "restart the hardware interface before sending another command."
        )
        self.current_state = ExtendedStatus.STATE_FAULT
        self.status_text = response.message
        self.command_success = False
        self.publish_status()
        return True

    def wait_for_movement(self, label):
        if self.movement_finished.wait(timeout=self.motion_timeout_sec):
            return True
        self.get_logger().error(
            f"{label} exceeded the {self.motion_timeout_sec:.1f}s motion deadline"
        )
        self.last_action_successful = False
        self.motion_recovery_required = True
        self.current_state = ExtendedStatus.STATE_FAULT
        self.status_text = (
            f"{label} timed out; robot state is uncertain and restart is required"
        )
        self.command_success = False
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
        self.publish_status()
        return False

    def handle_home_arm(self, request, response):
        if not self.begin_motion(response, "legacy:home"):
            return self.finalize_service_status(response)
        try:
            self.get_logger().info("Service Call: Home Arm")
            self.current_state = ExtendedStatus.STATE_BUSY
            self.status_text = "Sending arm to home position"
            self.publish_status()
            if self.send_home_goal():
                completed = self.wait_for_movement("Home motion")
                response.success = completed and self.last_action_successful
                response.message = (
                    "Arm moved home successfully"
                    if response.success
                    else "Arm movement failed or timed out"
                )
            else:
                response.success = False
                response.message = "Failed to initiate home movement"
        finally:
            self.end_motion()

        return self.finalize_service_status(response)

    def handle_move_arm(self, request, response):
        x = request.x
        y = request.y
        z = request.z
        if not all(math.isfinite(value) for value in (x, y, z)):
            response.success = False
            response.message = "Arm target coordinates must be finite"
            return self.finalize_service_status(response)
        if not self.begin_motion(response, "legacy:move_arm"):
            return self.finalize_service_status(response)
        try:
            self.get_logger().info(f"Service Call: Move Arm to {x}, {y}, {z}")
            self.current_state = ExtendedStatus.STATE_BUSY
            self.status_text = f"Moving arm to {x}, {y}, {z}..."
            self.publish_status()
            if self.send_goal(x, y, z):
                completed = self.wait_for_movement("Arm motion")
                response.success = completed and self.last_action_successful
                response.message = (
                    f"Arm moved to {x}, {y}, {z}"
                    if response.success
                    else "Arm movement failed or timed out"
                )
            else:
                response.success = False
                response.message = "Failed to initiate arm movement"
        finally:
            self.end_motion()

        return self.finalize_service_status(response)

    def handle_relative_move(self, request, response):
        vx = request.vx
        vy = request.vy
        vz = request.vz
        if not all(math.isfinite(value) for value in (vx, vy, vz)):
            response.success = False
            response.message = "Relative movement vector must be finite"
            return self.finalize_service_status(response)
        if not self.begin_motion(response, "legacy:relative_move"):
            return self.finalize_service_status(response)
        self.get_logger().info(f"Service Call: Relative Move by Vector [{vx}, {vy}, {vz}]")
        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = f"Executing relative move by vector [{vx}, {vy}, {vz}]..."
        self.publish_status()

        try:
            # Look up current pose of the tool frame
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'tool_frame', now, timeout=rclpy.duration.Duration(seconds=1.0))
            
            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            curr_z = trans.transform.translation.z
            
            target_x = curr_x + vx
            target_y = curr_y + vy
            target_z = curr_z + vz
            
            self.get_logger().info(f"Calculated target: {target_x}, {target_y}, {target_z}")
            
            if self.send_goal(target_x, target_y, target_z):
                completed = self.wait_for_movement("Relative arm motion")
                response.success = completed and self.last_action_successful
                response.message = "Relative movement complete" if response.success else "Relative movement failed or timed out"
            else:
                response.success = False
                response.message = "Failed to initiate relative movement"
                
        except Exception as e:
            self.get_logger().error(f"Could not calculate relative move: {e}")
            response.success = False
            response.message = str(e)
        finally:
            self.end_motion()
            
        return self.finalize_service_status(response)

    def handle_move_gripper(self, request, response):
        pos = request.position
        if (
            not math.isfinite(pos)
            or not self.gripper_min_position
            <= pos
            <= self.gripper_max_position
        ):
            response.success = False
            response.message = (
                f"Gripper position must be finite and between "
                f"{self.gripper_min_position} and {self.gripper_max_position}"
            )
            return self.finalize_service_status(response)
        if not self.begin_motion(response, "legacy:move_gripper"):
            return self.finalize_service_status(response)
        try:
            self.get_logger().info(f"Service Call: Move Gripper to {pos}")
            self.current_state = ExtendedStatus.STATE_BUSY
            self.status_text = f"Moving gripper to {pos}..."
            self.publish_status()
            if self.move_gripper(pos):
                completed = self.wait_for_movement("Gripper motion")
                response.success = completed and self.last_action_successful
                response.message = (
                    f"Gripper moved to {pos}"
                    if response.success
                    else "Gripper movement failed or timed out"
                )
            else:
                response.success = False
                response.message = "Failed to initiate gripper movement"
        finally:
            self.end_motion()

        return self.finalize_service_status(response)

    # --- Action Client Methods ---
    def send_goal(self, x, y, z):
        if not all(math.isfinite(value) for value in (x, y, z)):
            self.get_logger().error("Arm goal coordinates must be finite")
            return False
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Arm server not available')
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link" 
        pos_constraint.link_name = "tool_frame"      
        
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01] 

        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        target_pose.orientation.w = 1.0

        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        self.last_action_successful = False
        self.movement_finished.clear()
        future = self.arm_client.send_goal_async(
            goal_msg,
            feedback_callback=self.arm_feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        return True

    def send_home_goal(self):
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Arm server not available (Home Goal)')
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joint_positions = [0.0, 0.0, 1.5708, 1.5708, 1.5708, 0.0]
        tolerance = 0.01

        constraints = []
        for name, pos in zip(joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            constraints.append(jc)

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = constraints
        goal_msg.request.goal_constraints.append(goal_constraints)

        self.last_action_successful = False
        self.movement_finished.clear()
        future = self.arm_client.send_goal_async(
            goal_msg,
            feedback_callback=self.arm_feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        return True

    def move_gripper(self, position):
        if (
            not math.isfinite(position)
            or not self.gripper_min_position
            <= position
            <= self.gripper_max_position
        ):
            self.get_logger().error("Gripper goal is outside configured limits")
            return False
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            return False
        
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        
        self.active_gripper_target = float(position)
        self.last_action_successful = False
        self.movement_finished.clear()
        future = self.gripper_client.send_goal_async(
            goal,
            feedback_callback=self.gripper_feedback_callback
        )
        future.add_done_callback(self.gripper_response_callback)
        return True

    # --- Callbacks ---
    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Arm goal request failed: {exc}")
            self.last_action_successful = False
            self.motion_recovery_required = True
            self.movement_finished.set()
            return
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by the Action Server.')
            self.last_action_successful = False
            self.movement_finished.set()
            return
        self.active_goal_handle = goal_handle
        if self.is_faulted or self.motion_recovery_required:
            self.motion_recovery_required = True
            self.get_logger().error(
                "Arm goal was accepted after a hardware fault; canceling it"
            )
            goal_handle.cancel_goal_async()
        
        self.get_logger().info('Goal accepted! Moving...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def arm_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'[Feedback] MoveIt State: {feedback.state}')

    def result_callback(self, future):
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            error_code = result.error_code.val

            if (
                wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                and error_code == MoveItErrorCodes.SUCCESS
            ):
                self.get_logger().info('Movement complete!')
                self.last_action_successful = True
                return

            self.last_action_successful = False

            match error_code:
                case MoveItErrorCodes.NO_IK_SOLUTION:
                    self.get_logger().error("ERROR: Coordinates out of reach! (Arm is too short or pose is physically impossible)")
                case MoveItErrorCodes.PLANNING_FAILED:
                    self.get_logger().error("ERROR: Planning failed! (Path is blocked by an obstacle or self-collision)")
                case MoveItErrorCodes.TIMED_OUT:
                    self.get_logger().error("ERROR: Movement timed out!")
                case MoveItErrorCodes.GOAL_IN_COLLISION:
                    self.get_logger().error("ERROR: Goal is in collision! (Target position is inside an object)")
                case MoveItErrorCodes.START_STATE_IN_COLLISION:
                    self.get_logger().error("ERROR: Start state is in collision! (Robot is already hitting itself or an obstacle)")
                case MoveItErrorCodes.CONTROL_FAILED:
                    self.get_logger().error("ERROR: Control failed during execution! (Path planned successfully, but physical execution failed)")
                case MoveItErrorCodes.PREEMPTED:
                    self.get_logger().error("ERROR: Movement was canceled or preempted!")
                case _:
                    self.get_logger().error(
                        f'MoveIt failed with action status {wrapped_result.status} '
                        f'and error code {error_code}'
                    )

            self.handle_moveit_failure()
        except Exception as exc:
            self.last_action_successful = False
            self.get_logger().error(f"Failed to process MoveIt result: {exc}")
            self.handle_moveit_failure()
        finally:
            self.active_goal_handle = None
            self.movement_finished.set()

    def gripper_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Gripper goal request failed: {exc}")
            self.last_action_successful = False
            self.motion_recovery_required = True
            self.active_gripper_target = None
            self.movement_finished.set()
            return
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected.')
            self.last_action_successful = False
            self.active_gripper_target = None
            self.movement_finished.set()
            return
        self.active_goal_handle = goal_handle
        if self.is_faulted or self.motion_recovery_required:
            self.motion_recovery_required = True
            self.get_logger().error(
                "Gripper goal was accepted after a hardware fault; canceling it"
            )
            goal_handle.cancel_goal_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def gripper_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_width = round(feedback.position, 3)
        self.get_logger().debug(f'[Feedback] Gripper Width: {current_width}')

    def gripper_result_callback(self, future):
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            contact_stop = (
                result.stalled
                and self.active_gripper_target is not None
                and self.active_gripper_target
                >= self.gripper_closed_command_min
                and math.isfinite(result.position)
                and result.position >= self.gripper_contact_position_min
            )
            self.last_action_successful = (
                wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                and (result.reached_goal or contact_stop)
            )
            if self.last_action_successful:
                state = (
                    "reached target"
                    if result.reached_goal
                    else "contact stop while closing"
                )
                self.get_logger().info(f'Gripper movement complete ({state})')
            else:
                self.motion_recovery_required = True
                self.get_logger().error(
                    f"Gripper failed with action status {wrapped_result.status}; "
                    f"reached_goal={result.reached_goal}, stalled={result.stalled}"
                )
        except Exception as exc:
            self.last_action_successful = False
            self.motion_recovery_required = True
            self.get_logger().error(f"Failed to process gripper result: {exc}")
        finally:
            self.active_goal_handle = None
            self.active_gripper_target = None
            self.movement_finished.set()

    def destroy_node(self):
        if self.active_goal_handle is not None:
            try:
                self.active_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().error(
                    f"Could not cancel active motion during shutdown: {exc}"
                )
        self.motion_lease.release()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterfaceClient()
    
    # Use MultiThreadedExecutor to allow concurrent callback execution
    executor = MultiThreadedExecutor(num_threads=10) # TODO (pulkit) change the hardcoded threads numbers
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
