import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

# Arm Actions and Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# Gripper Actions and Messages
from control_msgs.action import GripperCommand

# Services and Messages
from example_interfaces.msg import Bool

# Custom telemetry message & services
from kinova_interfaces.msg import ExtendedStatus
from kinova_interfaces.srv import HomeArm, MoveArm, MoveGripper, RelativeMove

# Controller Manager Messages
from controller_manager_msgs.srv import ListControllers

# TF for Relative Movements
from tf2_ros import Buffer, TransformListener

class HardwareInterfaceClient(Node):
    ACTION_TIMEOUT_SEC = 30.0
    GRIPPER_TIMEOUT_SEC = 10.0

    HOME_JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    HOME_JOINT_POSITIONS = [0.0, 0.0, 1.5708, 1.5708, 1.5708, 0.0]
    HOME_JOINT_TOLERANCE = 0.01

    def __init__(self):
        super().__init__('kinova_hardware_client')
        self.get_logger().info('Kinova Hardware Client Online - Waiting for Service Requests...')
        
        # Use a ReentrantCallbackGroup to allow service handlers and action callbacks to run concurrently
        self.callback_group = ReentrantCallbackGroup()

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

        # Controller Manager Client for health monitoring of fault_controller
        self.list_controllers_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
            callback_group=self.callback_group
        )
        self.health_timer = self.create_timer(3.0, self.check_fault_controller_health, callback_group=self.callback_group)

        # Synchronous Movement Control & State Tracking
        self.arm_movement_finished = threading.Event()
        self.arm_movement_finished.set()
        self.arm_action_successful = False
        self.arm_action_message = "Ready"

        self.gripper_movement_finished = threading.Event()
        self.gripper_movement_finished.set()
        self.gripper_action_successful = False
        self.gripper_action_message = "Ready"

        # Telemetry Setup
        self.status_pub = self.create_publisher(ExtendedStatus, '/status/node_report', 10)
        self.status_timer = self.create_timer(0.5, self.publish_status, callback_group=self.callback_group)
        self.current_state = ExtendedStatus.STATE_IDLE
        self.status_text = "Hardware Interface Client Ready"
        self.command_success = True

        # ROS 2 Services
        self.home_arm = self.create_service(HomeArm, '~/home_arm', self.handle_home_arm, callback_group=self.callback_group)
        self.move_arm_srv = self.create_service(MoveArm, '~/move_arm', self.handle_move_arm, callback_group=self.callback_group)
        self.move_gripper_srv = self.create_service(MoveGripper, '~/move_gripper', self.handle_move_gripper, callback_group=self.callback_group)
        self.relative_move_srv = self.create_service(RelativeMove, '~/relative_move', self.handle_relative_move, callback_group=self.callback_group)

    # --- Telemetry Status Publisher ---
    def publish_status(self):
        msg = ExtendedStatus()
        msg.node_name = self.get_name()
        msg.state = self.current_state
        msg.status_message = self.status_text
        msg.last_command_valid = self.command_success
        self.status_pub.publish(msg)

    # --- Fault Controller Health Check & Helper ---
    def check_fault_controller_health(self):
        """Timer callback to check if the fault_controller is active on the controller_manager."""
        if not self.list_controllers_client.service_is_ready():
            self.get_logger().warn(
                "Controller manager '/list_controllers' service not ready!",
                throttle_duration_sec=10.0
            )
            return
        
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
                if not self.fault_controller_warning_active:
                    self.fault_controller_warning_active = True
                    self.get_logger().warn("fault_controller is present but NOT active!")
                    self.status_text = "SYSTEM CONFIG WARNING: fault_controller is NOT active"
                    self.publish_status()
            elif fault_ctrl_found and fault_ctrl_active:
                if self.fault_controller_warning_active:
                    self.fault_controller_warning_active = False
                    self.get_logger().info("fault_controller is now active! Clearing config warning status.")
                    if self.status_text == "SYSTEM CONFIG WARNING: fault_controller is NOT active":
                        self.status_text = "Hardware Interface Client Ready"
                    self.publish_status()
        except Exception as e:
            self.get_logger().error(f"Failed to query controllers health: {e}")

    def finalize_service_status(self, response):
        """Helper to centralize state & status updates after a service completes."""
        self.current_state = ExtendedStatus.STATE_FAULT if self.is_faulted else ExtendedStatus.STATE_IDLE
        self.command_success = response.success
        if not self.is_faulted and not self.fault_controller_warning_active:
            self.status_text = response.message
        self.publish_status()
        return response

    # --- Fault Handling ---
    def fault_callback(self, msg: Bool):
        """Asynchronously updates the internal fault status."""
        if msg.data and not self.is_faulted:
            self.get_logger().error("Robot entered a hardware FAULT state.")
            self.current_state = ExtendedStatus.STATE_FAULT
            self.status_text = "HARDWARE FAULT: Robot is faulted"
            self.command_success = False
        elif not msg.data and self.is_faulted:
            self.get_logger().info("Robot hardware fault has been cleared.")
            self.current_state = ExtendedStatus.STATE_IDLE
            self.status_text = "Robot Ready (Fault Cleared)"
            self.command_success = True
        self.publish_status()
        self.is_faulted = msg.data

    def handle_moveit_failure(self):
        """Called when MoveIt execution fails."""
        self.get_logger().error("MoveIt trajectory execution failed. Inspecting hardware health...")

        if self.is_faulted:
            self.status_text = "MoveIt Failure: Hardware fault confirmed. Awaiting fault-reset command."
            self.publish_status()
            self.get_logger().error(self.status_text)
        else:
            self.get_logger().info("No hardware fault detected. Failure may be algorithmic (planning timeout).")

    # --- Service Handlers ---
    def handle_home_arm(self, request, response):
        self.get_logger().info("Service Call: Home Arm")
        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = "Sending arm to home position"
        self.publish_status()

        if self.send_home_goal():
            finished = self.arm_movement_finished.wait(timeout=self.ACTION_TIMEOUT_SEC)
            if not finished:
                response.success = False
                response.message = f"Homing movement timed out after {self.ACTION_TIMEOUT_SEC}s"
                self.get_logger().error(response.message)
            else:
                response.success = self.arm_action_successful
                response.message = self.arm_action_message
        else:
            response.success = False
            response.message = "Failed to initiate home movement (action server unavailable)"

        return self.finalize_service_status(response)

    def handle_move_arm(self, request, response):
        x = request.x
        y = request.y
        z = request.z
        self.get_logger().info(f"Service Call: Move Arm to {x}, {y}, {z}")
        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = f"Moving arm to {x}, {y}, {z}..."
        self.publish_status()

        if self.send_goal(x, y, z):
            finished = self.arm_movement_finished.wait(timeout=self.ACTION_TIMEOUT_SEC)
            if not finished:
                response.success = False
                response.message = f"Arm movement to ({x}, {y}, {z}) timed out after {self.ACTION_TIMEOUT_SEC}s"
                self.get_logger().error(response.message)
            else:
                response.success = self.arm_action_successful
                response.message = self.arm_action_message
        else:
            response.success = False
            response.message = "Failed to initiate arm movement (action server unavailable)"

        return self.finalize_service_status(response)

    def handle_relative_move(self, request, response):
        vx = request.vx
        vy = request.vy
        vz = request.vz
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
            
            self.get_logger().info(f"Calculated target: {target_x:.3f}, {target_y:.3f}, {target_z:.3f}")
            
            if self.send_goal(target_x, target_y, target_z):
                finished = self.arm_movement_finished.wait(timeout=self.ACTION_TIMEOUT_SEC)
                if not finished:
                    response.success = False
                    response.message = f"Relative movement timed out after {self.ACTION_TIMEOUT_SEC}s"
                    self.get_logger().error(response.message)
                else:
                    response.success = self.arm_action_successful
                    response.message = self.arm_action_message
            else:
                response.success = False
                response.message = "Failed to initiate relative movement (action server unavailable)"
                
        except Exception as e:
            self.get_logger().error(f"Could not calculate relative move: {e}")
            response.success = False
            response.message = f"Relative move TF lookup failed: {e}"
            
        return self.finalize_service_status(response)

    def handle_move_gripper(self, request, response):
        pos = request.position
        self.get_logger().info(f"Service Call: Move Gripper to {pos}")
        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = f"Moving gripper to {pos}..."
        self.publish_status()

        if self.move_gripper(pos):
            finished = self.gripper_movement_finished.wait(timeout=self.GRIPPER_TIMEOUT_SEC)
            if not finished:
                response.success = False
                response.message = f"Gripper movement to {pos} timed out after {self.GRIPPER_TIMEOUT_SEC}s"
                self.get_logger().error(response.message)
            else:
                response.success = self.gripper_action_successful
                response.message = self.gripper_action_message
        else:
            response.success = False
            response.message = "Failed to initiate gripper movement (action server unavailable)"

        return self.finalize_service_status(response)

    # --- Action Client Methods ---
    def send_goal(self, x, y, z):
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

        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        self.arm_movement_finished.clear()
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
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        constraints = []
        for name, pos in zip(self.HOME_JOINT_NAMES, self.HOME_JOINT_POSITIONS):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = self.HOME_JOINT_TOLERANCE
            jc.tolerance_below = self.HOME_JOINT_TOLERANCE
            jc.weight = 1.0
            constraints.append(jc)

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = constraints
        goal_msg.request.goal_constraints.append(goal_constraints)

        self.arm_movement_finished.clear()
        future = self.arm_client.send_goal_async(
            goal_msg,
            feedback_callback=self.arm_feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        return True

    def move_gripper(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper server not available')
            return False
        
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        
        self.gripper_movement_finished.clear()
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
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected by the Action Server.')
                self.arm_action_successful = False
                self.arm_action_message = 'Goal rejected by MoveIt Action Server'
                self.arm_movement_finished.set()
                return
            
            self.get_logger().info('Goal accepted! Moving...')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)
        except Exception as e:
            self.get_logger().error(f"Error handling arm goal response: {e}")
            self.arm_action_successful = False
            self.arm_action_message = f"Goal dispatch error: {e}"
            self.arm_movement_finished.set()

    def arm_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'[Feedback] MoveIt State: {feedback.state}')

    def result_callback(self, future):
        try:
            result = future.result().result
            error_code = result.error_code.val
            
            if error_code == result.error_code.SUCCESS:
                self.get_logger().info('Movement complete!')
                self.arm_action_successful = True
                self.arm_action_message = 'Movement complete'
            else:
                self.arm_action_successful = False

                match error_code:
                    case result.error_code.NO_IK_SOLUTION:
                        msg = "Coordinates out of reach (no inverse kinematics solution)"
                    case result.error_code.PLANNING_FAILED:
                        msg = "Planning failed (path blocked by obstacle or self-collision)"
                    case result.error_code.TIMED_OUT:
                        msg = "MoveIt planning/movement timed out"
                    case result.error_code.GOAL_IN_COLLISION:
                        msg = "Goal is in collision (target position inside an obstacle)"
                    case result.error_code.START_STATE_IN_COLLISION:
                        msg = "Start state is in collision (robot currently in collision)"
                    case result.error_code.CONTROL_FAILED:
                        msg = "Control failed during execution (hardware error)"
                    case result.error_code.ABORT:
                        msg = "Movement was aborted by MoveIt"
                    case _:
                        msg = f"MoveIt failed with error code: {error_code}"

                self.get_logger().error(f"ERROR: {msg}")
                self.arm_action_message = msg
                self.handle_moveit_failure()
        except Exception as e:
            self.get_logger().error(f"Error processing arm action result: {e}")
            self.arm_action_successful = False
            self.arm_action_message = f"Action result processing error: {e}"
        finally:
            self.arm_movement_finished.set()

    def gripper_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Gripper goal rejected.')
                self.gripper_action_successful = False
                self.gripper_action_message = 'Goal rejected by Gripper Action Server'
                self.gripper_movement_finished.set()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.gripper_result_callback)
        except Exception as e:
            self.get_logger().error(f"Error handling gripper goal response: {e}")
            self.gripper_action_successful = False
            self.gripper_action_message = f"Gripper goal dispatch error: {e}"
            self.gripper_movement_finished.set()

    def gripper_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_width = round(feedback.position, 3)
        self.get_logger().debug(f'[Feedback] Gripper Width: {current_width}')

    def gripper_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(
                f'Gripper movement complete! position={result.position:.3f}, '
                f'effort={result.effort:.3f}, stalled={result.stalled}, '
                f'reached_goal={result.reached_goal}'
            )
            self.gripper_action_successful = result.reached_goal or result.stalled
            if self.gripper_action_successful:
                self.gripper_action_message = f"Gripper moved to position {result.position:.3f}"
            else:
                self.gripper_action_message = f"Gripper failed to reach target (position={result.position:.3f})"
        except Exception as e:
            self.get_logger().error(f"Error processing gripper action result: {e}")
            self.gripper_action_successful = False
            self.gripper_action_message = f"Gripper result processing error: {e}"
        finally:
            self.gripper_movement_finished.set()

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
