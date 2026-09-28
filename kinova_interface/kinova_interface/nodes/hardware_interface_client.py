import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from control_msgs.action import GripperCommand

# Current joint positions, for relative joint moves (e.g. 'pour's tilt)
from sensor_msgs.msg import JointState

from example_interfaces.msg import Bool
from controller_manager_msgs.srv import ListControllers
from tf2_ros import Buffer, TransformListener

from kinova_interfaces.msg import ExtendedStatus
from kinova_interfaces.srv import HomeArm, MoveArm, MoveGripper, RelativeMove, JointMove

from kinova_interface.utils.geometry import euler_to_quaternion, quaternion_to_euler
from kinova_interface.utils.robot import BASE_FRAME, TOOL_FRAME, JOINT_NAMES


class HardwareInterfaceClient(Node):
    ACTION_TIMEOUT_SEC = 30.0
    GRIPPER_TIMEOUT_SEC = 10.0
    SERVER_WAIT_TIMEOUT_SEC = 5.0
    PLANNING_GROUP = 'arm'
    # A tight combined position+orientation constraint reached in one big
    # jump from a very different starting configuration (e.g. 'pickup' going
    # straight from home) is a much harder search problem than the same pose
    # reached through several small incremental moves - bumped from 10/5.0s
    # after exactly this case (a forced pickup orientation) failed with
    # generic error 99999 despite being a confirmed-reachable, collision-free
    # pose (manually verified in RViz).
    NUM_PLANNING_ATTEMPTS = 20
    ALLOWED_PLANNING_TIME_SEC = 10.0
    SPHERE_TOLERANCE_RADIUS = 0.01

    # Home's fixed joint configuration - also the base pose 'throw' starts
    # its wind-up/fling from, reoriented at joint_1 to face the throw
    # direction and offset at joint_3 (the elbow) for the swing.
    HOME_JOINT_POSITIONS = [0.0, 0.0, 1.5708, 1.5708, 1.5708, 0.0]
    HOME_JOINT_TOLERANCE = 0.01
    # TODO: make these ^^ configurable

    # A rejected/invalid goal (e.g. an unreachable combined joint state)
    # typically fails within milliseconds, well before a real motion could
    # possibly finish - so a short bounded wait here can catch that kind of
    # fast rejection in the fire-and-forget path without turning into a
    # real wait for a genuine, slow motion in progress.
    FIRE_AND_FORGET_REJECTION_WINDOW_SEC = 0.5

    def __init__(self):
        """Initialise the Kinova hardware interface node.

        Creates the MoveIt arm and gripper action clients, TF listener, joint-state
        and fault subscriptions, controller health monitoring, telemetry publisher,
        and ROS 2 movement services.

        Args:
            None

        Returns:
            None
        """
        
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

        # Latest joint positions (name -> position), for relative joint
        # moves (e.g. 'pour's tilt, a delta on joint_6 alone) - None until
        # the first /joint_states message arrives.
        self.latest_joint_positions = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            10,
            callback_group=self.callback_group
        )

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

        # ROS 2 Services (The "API")
        self.home_arm = self.create_service(HomeArm, '~/home_arm', self.handle_home_arm, callback_group=self.callback_group)
        self.move_arm_srv = self.create_service(MoveArm, '~/move_arm', self.handle_move_arm, callback_group=self.callback_group)
        self.move_gripper_srv = self.create_service(MoveGripper, '~/move_gripper', self.handle_move_gripper, callback_group=self.callback_group)
        self.relative_move_srv = self.create_service(RelativeMove, '~/relative_move', self.handle_relative_move, callback_group=self.callback_group)
        self.joint_move_srv = self.create_service(JointMove, '~/joint_move', self.handle_joint_move, callback_group=self.callback_group)

    # --- Telemetry Status Publisher ---
    def publish_status(self):
        """Publish the current hardware interface status.

        Constructs an ``ExtendedStatus`` message using the node's current state,
        status text, and most recent command result, then publishes it.

        Args:
            None

        Returns:
            None
        """
        
        msg = ExtendedStatus()
        msg.node_name = self.get_name()
        msg.state = self.current_state
        msg.status_message = self.status_text
        msg.last_command_valid = self.command_success
        self.status_pub.publish(msg)

    def _on_joint_state(self, msg):
        """Update the cached joint positions from a joint-state message.

        Stores the latest positions for the configured robot joints so they can
        be used by relative joint movements and other hardware interface logic.

        Args:
            msg (JointState): ROS 2 joint-state message containing joint names
                and their corresponding positions.

        Returns:
            None
        """     
        self.latest_joint_positions = dict(zip(msg.name, msg.position))

    # --- Fault Controller Health Check & Helper ---
    def check_fault_controller_health(self):
        """Check the health of the fault controller.

        Checks whether the controller-manager service is available and requests
        the current controller list. The response is processed asynchronously by
        ``list_controllers_callback``.

        Returns:
            None
        """

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
        """Process the controller-manager health check response.

        Checks whether ``fault_controller`` exists and whether it is active.
        Updates the node status and warning state when the controller becomes
        inactive or returns to an active state.

        Args:
            future: Future containing the ``ListControllers`` service response.

        Returns:
            None
        """
        
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
        """Finalise a movement service response and update node telemetry.

        Sets the node state, command success flag, and status message based on
        the service response and current hardware fault state.

        Args:
            response: ROS 2 service response object containing ``success`` and
                ``message`` fields.

        Returns:
            The same service response after status information has been updated.
        """

        self.current_state = ExtendedStatus.STATE_FAULT if self.is_faulted else ExtendedStatus.STATE_IDLE
        self.command_success = response.success
        if not self.is_faulted and not self.fault_controller_warning_active:
            self.status_text = response.message
        self.publish_status()
        return response

    # --- Fault Handling ---
    def fault_callback(self, msg: Bool):
        """Update the node state when the hardware fault status changes.

        Args:
            msg (Bool): ROS 2 boolean message indicating whether the robot is
        currently faulted.

        Returns:
            None
        """

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
        """Handle a failed MoveIt trajectory execution.

        Logs the failure and checks the current hardware fault state to
        distinguish a confirmed hardware fault from a planning or execution
        failure without a detected hardware fault.

        Args:
            None

        Returns:
            None
        """
        
        self.get_logger().error("MoveIt trajectory execution failed. Inspecting hardware health...")

        if self.is_faulted:
            self.status_text = "MoveIt Failure: Hardware fault confirmed. Awaiting fault-reset command."
            self.publish_status()
            self.get_logger().error(self.status_text)
        else:
            self.get_logger().info("No hardware fault detected. Failure may be algorithmic (planning timeout).")

    def _await_action(self, event: threading.Event, timeout_sec: float, success_attr: str, message_attr: str, action_desc: str, response):
        """Wait for an action to finish and populate a service response.

        Waits for the supplied event for up to ``timeout_sec`` seconds and reads
        the configured success and message state when the action completes.

        Args:
            event (threading.Event): Event that is set when the action completes.
            timeout_sec (float): Maximum time to wait for the action.
            success_attr (str): Attribute name containing the action success state,
                or a callable returning that state.
            message_attr (str): Attribute name containing the action result message,
                or a callable returning that message.
            action_desc (str): Human-readable description used in timeout messages.
            response: ROS 2 service response object to populate.

        Returns:
            The supplied service response with its ``success`` and ``message``
            fields populated.
        """

        # TODO: create a request/response interface to do type constraints in functions
        finished = event.wait(timeout=timeout_sec)
        if not finished:
            response.success = False
            response.message = f"{action_desc} timed out after {timeout_sec}s"
            self.get_logger().error(response.message)
        else:
            response.success = getattr(self, success_attr) if isinstance(success_attr, str) else success_attr()
            response.message = getattr(self, message_attr) if isinstance(message_attr, str) else message_attr()
        return response

    # --- Service Handlers ---
    def handle_home_arm(self, request, response):
        """Handle a request to move the arm to its predefined home position.

        Starts the home-position action and waits for completion before returning
        the service response.

        Args:
            request (HomeArm.Request): Service request containing optional
                movement parameters.
            response (HomeArm.Response): Service response populated with the
                movement result.

        Returns:
            HomeArm.Response: The populated service response.
        """

        self.get_logger().info("Service Call: Home Arm")
        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = "Sending arm to home position"
        self.publish_status()

        if self.send_home_goal(motion_params=request.motion_params):
            self._await_action(
                self.arm_movement_finished,
                self.ACTION_TIMEOUT_SEC,
                'arm_action_successful',
                'arm_action_message',
                "Arm movement to Home",
                response
            )
        else:
            response.success = False
            response.message = "Failed to initiate home movement (action server unavailable)"

        return self.finalize_service_status(response)

    def handle_joint_move(self, request, response):
        """Handle an absolute or relative joint-space movement request.

        Absolute mode sends the requested six joint positions directly to MoveIt.
        Relative mode adds the requested joint deltas to the latest joint state.

        When ``wait_for_completion`` is false, the service waits only for the
        short fire-and-forget rejection window before returning, allowing the
        caller to continue while the movement remains in progress.

        Args:
            request (JointMove.Request): Service request containing joint target
                positions, relative-mode settings, completion behaviour, and
                movement parameters.
            response (JointMove.Response): Service response populated with the
                movement result.

        Returns:
            JointMove.Response: The populated service response.
        """

        joint_positions = list(request.joint_positions)

        if request.relative:
            if self.latest_joint_positions is None:
                response.success = False
                response.message = "No joint state available for relative joint move"
                return self.finalize_service_status(response)
            try:
                current = [self.latest_joint_positions[name] for name in JOINT_NAMES]
            except KeyError as e:
                response.success = False
                response.message = f"Missing joint {e} in latest joint state"
                return self.finalize_service_status(response)
            joint_positions = [current[i] + joint_positions[i] for i in range(6)]

        self.get_logger().info(f"Service Call: Joint Move to {joint_positions}")
        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = f"Moving to joint targets {joint_positions}..."
        self.publish_status()

        if not self.send_joint_goal(joint_positions, motion_params=request.motion_params):
            response.success = False
            response.message = "Failed to initiate joint move"
            return self.finalize_service_status(response)

        if not request.wait_for_completion:
            if self.arm_movement_finished.wait(timeout=self.FIRE_AND_FORGET_REJECTION_WINDOW_SEC):
                response.success = self.arm_action_successful
                response.message = self.arm_action_message
                return self.finalize_service_status(response)
            response.success = True
            response.message = "Joint move goal accepted (not waiting for completion)"
            return response

        self._await_action(
            self.arm_movement_finished,
            self.ACTION_TIMEOUT_SEC,
            'arm_action_successful',
            'arm_action_message',
            f"Joint move to {joint_positions}",
            response
        )
        return self.finalize_service_status(response)

    def handle_move_arm(self, request, response):
        """Handle a Cartesian arm movement request.

        Sends the requested Cartesian position and optional orientation to MoveIt,
        then waits for the movement to complete.

        Args:
            request (MoveArm.Request): Service request containing the target
                position, optional orientation, and movement parameters.
            response (MoveArm.Response): Service response populated with the
                movement result.

        Returns:
            MoveArm.Response: The populated service response.
        """
        
        x = request.target_position.x
        y = request.target_position.y
        z = request.target_position.z
        self.get_logger().info(f"Service Call: Move Arm to {x}, {y}, {z}")
        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = f"Moving arm to {x}, {y}, {z}..."
        self.publish_status()

        if self.send_goal(
            x, y, z,
            has_orientation=request.has_orientation,
            roll=request.roll,
            pitch=request.pitch,
            yaw=request.yaw,
            motion_params=request.motion_params,
        ):
            self._await_action(
                self.arm_movement_finished,
                self.ACTION_TIMEOUT_SEC,
                'arm_action_successful',
                'arm_action_message',
                f"Arm movement to ({x}, {y}, {z})",
                response
            )
        else:
            response.success = False
            response.message = "Failed to initiate arm movement (action server unavailable)"

        return self.finalize_service_status(response)

    def handle_relative_move(self, request, response):
        """Handle a relative Cartesian movement request.

        Looks up the current tool pose using TF, applies the requested position
        and optional roll, pitch, and yaw offsets, and sends the resulting target
        pose to MoveIt.

        Args:
            request (RelativeMove.Request): Service request containing position
                and optional orientation deltas and movement parameters.
            response (RelativeMove.Response): Service response populated with the
                movement result.

        Returns:
            RelativeMove.Response: The populated service response.
        """
        
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
            trans = self.tf_buffer.lookup_transform(BASE_FRAME, TOOL_FRAME, now, timeout=rclpy.duration.Duration(seconds=1.0))

            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            curr_z = trans.transform.translation.z

            target_x = curr_x + vx
            target_y = curr_y + vy
            target_z = curr_z + vz

            self.get_logger().info(f"Calculated target: {target_x:.3f}, {target_y:.3f}, {target_z:.3f}")

            has_orientation = request.has_orientation
            target_roll = target_pitch = target_yaw = 0.0
            if has_orientation:
                # quaternion_to_euler uses asin() for pitch, capped at +-90deg.
                # If the arm is already near that boundary (e.g. right after
                # tilted_for_pour), roll/yaw become coupled and this delta
                # composition gives unintuitive results. Fine for small nudges
                # from a normal pose, which is the expected use case.
                q = trans.transform.rotation
                curr_roll, curr_pitch, curr_yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
                target_roll = curr_roll + request.roll_delta
                target_pitch = curr_pitch + request.pitch_delta
                target_yaw = curr_yaw + request.yaw_delta
                self.get_logger().info(
                    f"Calculated target orientation (rpy): {target_roll}, {target_pitch}, {target_yaw}"
                )

            if self.send_goal(
                target_x, target_y, target_z,
                has_orientation=has_orientation,
                roll=target_roll,
                pitch=target_pitch,
                yaw=target_yaw,
                motion_params=request.motion_params,
            ):
                self._await_action(
                    self.arm_movement_finished,
                    self.ACTION_TIMEOUT_SEC,
                    'arm_action_successful',
                    'arm_action_message',
                    f"Relative movement to ({target_x}, {target_y}, {target_z})",
                    response
                )
            else:
                response.success = False
                response.message = "Failed to initiate relative movement (action server unavailable)"

        except Exception as e:
            self.get_logger().error(f"Could not calculate relative move: {e}")
            response.success = False
            response.message = f"Relative move TF lookup failed: {e}"

        return self.finalize_service_status(response)

    def handle_move_gripper(self, request, response):
        """Handle a gripper movement request.

        Sends the requested gripper position to the gripper action server and
        waits for the movement to complete.

        Args:
            request (MoveGripper.Request): Service request containing the target
                gripper position.
            response (MoveGripper.Response): Service response populated with the
                movement result.

        Returns:
            MoveGripper.Response: The populated service response.
        """

        pos = request.position
        self.get_logger().info(f"Service Call: Move Gripper to {pos}")
        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = f"Moving gripper to {pos}..."
        self.publish_status()

        if self.move_gripper(pos):
            self._await_action(
                self.gripper_movement_finished,
                self.GRIPPER_TIMEOUT_SEC,
                'gripper_action_successful',
                'gripper_action_message',
                f"Gripper movement to {pos}",
                response
            )
        else:
            response.success = False
            response.message = "Failed to initiate gripper movement (action server unavailable)"

        return self.finalize_service_status(response)

    # --- Orientation / Motion Params Helpers ---
    def clamp_motion_params(self, motion_params):
        """Clamp velocity and acceleration scaling factors to the valid range.

        Values outside the MoveIt scaling range of 0.0 to 1.0 are clamped and
        a warning is logged.

        Args:
            motion_params: Movement parameter object containing
                ``velocity_scale`` and ``acceleration_scale``.

        Returns:
            tuple[float, float]: Clamped velocity and acceleration scaling
                factors.
        """

        velocity_scale = motion_params.velocity_scale
        acceleration_scale = motion_params.acceleration_scale

        if velocity_scale < 0.0 or velocity_scale > 1.0:
            self.get_logger().warn(f"velocity_scale {velocity_scale} out of range, clamping to [0.0, 1.0]")
            velocity_scale = max(0.0, min(1.0, velocity_scale))

        if acceleration_scale < 0.0 or acceleration_scale > 1.0:
            self.get_logger().warn(f"acceleration_scale {acceleration_scale} out of range, clamping to [0.0, 1.0]")
            acceleration_scale = max(0.0, min(1.0, acceleration_scale))

        return velocity_scale, acceleration_scale

    # --- Action Client Methods ---
    def send_goal(self, x, y, z, has_orientation=False, roll=0.0, pitch=0.0, yaw=0.0, motion_params=None):
        """Send a Cartesian movement goal to MoveIt.

        Constructs a position constraint for the robot tool and optionally adds
        an orientation constraint and motion-scaling parameters. The goal is
        submitted asynchronously.

        This method reports whether the goal could be initiated; it does not
        indicate whether the physical movement has completed successfully.
        Completion is reported through the action callbacks.

        Args:
            x (float): Target X position in ``BASE_FRAME``.
            y (float): Target Y position in ``BASE_FRAME``.
            z (float): Target Z position in ``BASE_FRAME``.
            has_orientation (bool): Whether an orientation constraint should be
                included.
            roll (float): Target roll angle in radians.
            pitch (float): Target pitch angle in radians.
            yaw (float): Target yaw angle in radians.
            motion_params: Optional movement parameters containing velocity and
                acceleration scaling factors.

        Returns:
            bool: ``True`` if the MoveIt action server is available and the goal
                was submitted; ``False`` if the action server is unavailable.
        """

        if not self.arm_client.wait_for_server(timeout_sec=self.SERVER_WAIT_TIMEOUT_SEC):
            self.get_logger().error('Arm server not available')
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.PLANNING_GROUP
        goal_msg.request.num_planning_attempts = self.NUM_PLANNING_ATTEMPTS
        goal_msg.request.allowed_planning_time = self.ALLOWED_PLANNING_TIME_SEC

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = BASE_FRAME
        pos_constraint.link_name = TOOL_FRAME

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.SPHERE_TOLERANCE_RADIUS]

        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)

        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos_constraint)

        if has_orientation:
            qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
            orient_constraint = OrientationConstraint()
            orient_constraint.header.frame_id = BASE_FRAME
            orient_constraint.link_name = TOOL_FRAME
            orient_constraint.orientation.x = qx
            orient_constraint.orientation.y = qy
            orient_constraint.orientation.z = qz
            orient_constraint.orientation.w = qw
            orient_constraint.absolute_x_axis_tolerance = 0.1
            orient_constraint.absolute_y_axis_tolerance = 0.1
            orient_constraint.absolute_z_axis_tolerance = 0.1
            orient_constraint.weight = 1.0
            goal_constraints.orientation_constraints.append(orient_constraint)

        goal_msg.request.goal_constraints.append(goal_constraints)

        if motion_params is not None:
            velocity_scale, acceleration_scale = self.clamp_motion_params(motion_params)
            if velocity_scale > 0.0:
                goal_msg.request.max_velocity_scaling_factor = velocity_scale
            if acceleration_scale > 0.0:
                goal_msg.request.max_acceleration_scaling_factor = acceleration_scale

        self.arm_movement_finished.clear()
        future = self.arm_client.send_goal_async(
            goal_msg,
            feedback_callback=self.arm_feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        return True

    def send_home_goal(self, motion_params=None):
        """Send a joint-space goal using the predefined home configuration.

        Args:
            motion_params: Optional movement parameters containing velocity and
                acceleration scaling factors.

        Returns:
            bool: ``True`` if the MoveIt action server is available and the goal
                was submitted; ``False`` otherwise.
        """
        
        return self.send_joint_goal(self.HOME_JOINT_POSITIONS, motion_params=motion_params)

    def send_joint_goal(self, joint_positions, motion_params=None):
        """Send a joint-space movement goal to MoveIt.

        Creates a ``JointConstraint`` for each configured robot joint and submits
        the resulting goal asynchronously.

        Args:
            joint_positions (list[float]): Target positions for the robot joints,
                ordered according to ``JOINT_NAMES``.
            motion_params: Optional movement parameters containing velocity and
                acceleration scaling factors.

        Returns:
            bool: ``True`` if the MoveIt action server is available and the goal
                was submitted; ``False`` otherwise.
        """
        
        if not self.arm_client.wait_for_server(timeout_sec=self.SERVER_WAIT_TIMEOUT_SEC):
            self.get_logger().error('Arm server not available (Joint Move)')
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.PLANNING_GROUP
        goal_msg.request.num_planning_attempts = self.NUM_PLANNING_ATTEMPTS
        goal_msg.request.allowed_planning_time = self.ALLOWED_PLANNING_TIME_SEC

        constraints = []
        for name, pos in zip(JOINT_NAMES, joint_positions):
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

        if motion_params is not None:
            velocity_scale, acceleration_scale = self.clamp_motion_params(motion_params)
            if velocity_scale > 0.0:
                goal_msg.request.max_velocity_scaling_factor = velocity_scale
            if acceleration_scale > 0.0:
                goal_msg.request.max_acceleration_scaling_factor = acceleration_scale

        self.arm_movement_finished.clear()
        future = self.arm_client.send_goal_async(
            goal_msg,
            feedback_callback=self.arm_feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        return True

    def move_gripper(self, position):
        """Send a target position to the gripper action server.

        The gripper action is submitted asynchronously. Completion is reported
        through the gripper action callbacks.

        Args:
            position (float): Target gripper position.

        Returns:
            bool: ``True`` if the gripper action server is available and the goal
                was submitted; ``False`` otherwise.
        """

        if not self.gripper_client.wait_for_server(timeout_sec=self.SERVER_WAIT_TIMEOUT_SEC):
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
        """Process the response received after submitting an arm action goal.

        If the goal is accepted, registers a callback for the eventual action
        result. If the goal is rejected or an exception occurs, records the
        failure and releases the movement wait event.

        Args:
            future: Future containing the MoveIt arm action goal handle.

        Returns:
            None
        """

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
        """Process feedback received while an arm action is executing.

        Logs the current MoveIt execution state at debug level.

        Args:
            feedback_msg: MoveIt action feedback message containing the current
                execution state.

        Returns:
            None
        """

        feedback = feedback_msg.feedback
        self.get_logger().debug(f'[Feedback] MoveIt State: {feedback.state}')

    def result_callback(self, future):
        """Process the final result of a MoveIt arm action.

        Maps MoveIt error codes to human-readable status messages, updates the
        stored movement result, handles execution failures, and releases the
        movement wait event.

        Args:
            future: Future containing the completed MoveIt action result.

        Returns:
            None
        """

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
            # Reset state here (not just in finalize_service_status, which only
            # runs for a caller that waited) so a fire-and-forget joint move
            # (e.g. 'throw's fling) doesn't leave current_state stuck on BUSY
            # once it actually finishes with nobody waiting on it.
            if not self.is_faulted:
                self.current_state = ExtendedStatus.STATE_IDLE
                self.status_text = "Movement complete!" if self.arm_action_successful else "Movement failed"
                self.publish_status()
            self.arm_movement_finished.set()

    def gripper_response_callback(self, future):
        """Process the response received after submitting a gripper goal.

        If the goal is accepted, registers a callback for the eventual gripper
        action result. Rejected goals and dispatch exceptions are recorded as
        failures.

        Args:
            future: Future containing the gripper action goal handle.

        Returns:
            None
        """

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
        """Process feedback received while the gripper is moving.

        Logs the current gripper position at debug level.

        Args:
            feedback_msg: Gripper action feedback message containing the current
                gripper position.

        Returns:
            None
        """

        feedback = feedback_msg.feedback
        current_width = round(feedback.position, 3)
        self.get_logger().debug(f'[Feedback] Gripper Width: {current_width}')

    def gripper_result_callback(self, future):
        """Process the final result of a gripper action.

        Updates the stored success state and result message based on whether the
        gripper reached its target or stalled, then releases the movement wait
        event.

        Args:
            future: Future containing the completed gripper action result.

        Returns:
            None
        """

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
    """Start the Kinova hardware interface ROS 2 node.

    Initialises rclpy, creates the hardware interface node, runs it using a
    multi-threaded executor, and performs node and ROS 2 shutdown cleanup.

    Args:
        args (list[str], optional): Command-line arguments passed to rclpy.

    Returns:
        None
    """
    
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
