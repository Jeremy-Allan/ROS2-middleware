"""ROS 2 node that is the only part of the middleware that touches the robot.

Exposes the arm and gripper movement services, dispatches the underlying
MoveIt and gripper actions, tracks hardware fault state, and reports
telemetry. Other code may query MoveIt (IK, FK, state validity, planning
scene) directly, but every actual movement goes through this node.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
from collections import namedtuple

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from control_msgs.action import GripperCommand

# Current joint positions, for relative joint moves (e.g. 'pour's tilt)
from sensor_msgs.msg import JointState

from example_interfaces.msg import Bool
from controller_manager_msgs.srv import ListControllers
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation

from kinova_interfaces.msg import ExtendedStatus
from kinova_interfaces.srv import HomeArm, MoveArm, MoveGripper, RelativeMove, JointMove

from kinova_interface.utils.robot import BASE_FRAME, TOOL_FRAME, JOINT_NAMES


class HardwareInterfaceClient(Node):
    """The only node that executes motion on the real or simulated robot.

    Exposes the arm and gripper movement services, dispatches the underlying
    MoveIt and gripper actions and waits for them to finish, tracks hardware
    fault state, and reports telemetry. Movement calls block synchronously on
    a ``threading.Event`` until the action's result callback fires, which is
    what makes each recipe step wait for the robot to actually finish.
    """

    ACTION_TIMEOUT_SEC = 30.0
    GRIPPER_TIMEOUT_SEC = 10.0
    SERVER_WAIT_TIMEOUT_SEC = 5.0
    PLANNING_GROUP = 'arm'
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

    # Arm moves try Pilz PTP first (same path every time, but it won't route
    # around obstacles), then fall back to OMPL RRT* if it can't plan.
    Planner = namedtuple('Planner', 'pipeline_id planner_id num_attempts label')
    PILZ_PTP = Planner('pilz_industrial_motion_planner', 'PTP', 4, 'Pilz PTP')
    # 4 attempts = one parallel batch, so about ALLOWED_PLANNING_TIME_SEC total
    RRT_STAR = Planner('ompl', 'RRTstarkConfigDefault', 4, 'OMPL RRT*')
    # Failed before anything moved, so it's safe to re-plan
    REPLANNABLE_ERROR_CODES = {
        MoveItErrorCodes.FAILURE,
        MoveItErrorCodes.PLANNING_FAILED,
        MoveItErrorCodes.INVALID_MOTION_PLAN,
        MoveItErrorCodes.NO_IK_SOLUTION,
        MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS,
    }

    def __init__(self):
        """Initialise the Kinova hardware interface node.

        Creates the MoveIt arm and gripper action clients, the TF listener,
        the joint-state and hardware-fault subscriptions, the fault
        controller health-check timer, the telemetry publisher, and the five
        movement services.
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
        self.arm_action_error_code = None
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
        """Publish the node's current telemetry status.

        The published status includes the node name, current state, status
        message, and whether the most recent command was successful.
        """
        msg = ExtendedStatus()
        msg.node_name = self.get_name()
        msg.state = self.current_state
        msg.status_message = self.status_text
        msg.last_command_valid = self.command_success
        self.status_pub.publish(msg)

    def _on_joint_state(self, msg):
        """Cache the latest joint positions from a ``/joint_states`` message.

        Args:
            msg (JointState): Joint-state message with names and positions.
        """
        self.latest_joint_positions = dict(zip(msg.name, msg.position))

    # --- Fault Controller Health Check & Helper ---
    def check_fault_controller_health(self):
        """Timer callback to check if the fault_controller is active on the controller_manager."""
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

        Checks whether ``fault_controller`` is present and active, and
        raises or clears the fault-controller config warning status
        accordingly.

        Args:
            future: Future containing the ``ListControllers`` response.
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
        """Helper to centralize state & status updates after a service completes.

        Args:
            response: Service response with populated ``success`` and
                ``message`` fields.

        Returns:
            The same response, after node telemetry has been updated from it.
        """
        self.current_state = ExtendedStatus.STATE_FAULT if self.is_faulted else ExtendedStatus.STATE_IDLE
        self.command_success = response.success
        if not self.is_faulted and not self.fault_controller_warning_active:
            self.status_text = response.message
        self.publish_status()
        return response

    # --- Fault Handling ---
    def fault_callback(self, msg: Bool):
        """Asynchronously updates the internal fault status.

        Args:
            msg (Bool): Latest value of ``/fault_controller/is_faulted``.
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
        """Called when MoveIt execution fails.

        Logs the failure and checks current fault state to tell a confirmed
        hardware fault apart from a planning/execution failure with no
        hardware fault behind it.
        """
        self.get_logger().error("MoveIt trajectory execution failed. Inspecting hardware health...")

        if self.is_faulted:
            self.status_text = "MoveIt Failure: Hardware fault confirmed. Awaiting fault-reset command."
            self.publish_status()
            self.get_logger().error(self.status_text)
        else:
            self.get_logger().info("No hardware fault detected. Failure may be algorithmic (planning timeout).")

    def _await_action(self, event: threading.Event, timeout_sec: float, success_attr: str, message_attr: str, action_desc: str, response):
        """Wait for an action event and populate the service response with status and message.

        Args:
            event (threading.Event): Event set when the action completes.
            timeout_sec (float): Maximum time to wait, in seconds.
            success_attr (str): Attribute name (or callable) giving the
                action's success state once ``event`` is set.
            message_attr (str): Attribute name (or callable) giving the
                action's result message once ``event`` is set.
            action_desc (str): Human-readable description, used only in the
                timeout log/message.
            response: Service response to populate.

        Returns:
            The same response, with ``success`` and ``message`` populated.
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

    def _plan_and_move(self, send_goal, planners, action_desc, start_failure_message, response, wait=True):
        """Try each planner in turn until one works, or a failure isn't worth
        re-planning. send_goal(planner) sends the goal. With wait=False, only waits
        long enough to catch a fast failure.

        Args:
            send_goal: Callable taking one ``planner`` and sending the goal,
                returning whether the action server accepted dispatch.
            planners: Ordered sequence of planners to try, most preferred first.
            action_desc (str): Human-readable description used in timeout/log
                messages.
            start_failure_message (str): Message to set if ``send_goal`` itself
                fails (e.g. action server unavailable).
            response: Service response to populate.
            wait (bool): If True, wait the full ``ACTION_TIMEOUT_SEC`` for
                completion. If False, only wait
                ``FIRE_AND_FORGET_REJECTION_WINDOW_SEC`` for a fast rejection.

        Returns:
            bool: True if a goal was left running with nobody waiting on it
                (only possible when ``wait`` is False), otherwise False.
        """
        for i, planner in enumerate(planners):
            if i:
                self.get_logger().warn(
                    f"{planners[i - 1].label} could not plan ({response.message}), re-planning with {planner.label}"
                )
            if not send_goal(planner):
                response.success = False
                response.message = start_failure_message
                return False

            if wait:
                self._await_action(
                    self.arm_movement_finished,
                    self.ACTION_TIMEOUT_SEC,
                    'arm_action_successful',
                    'arm_action_message',
                    action_desc,
                    response
                )
            elif self.arm_movement_finished.wait(timeout=self.FIRE_AND_FORGET_REJECTION_WINDOW_SEC):
                response.success = self.arm_action_successful
                response.message = self.arm_action_message
            else:
                response.success = True
                response.message = "Joint move goal accepted (not waiting for completion)"
                return True

            if response.success:
                response.message += f" (planned with {planner.label})"
                return False
            if self.arm_action_error_code not in self.REPLANNABLE_ERROR_CODES:
                return False
        return False

    # --- Service Handlers ---
    def handle_home_arm(self, request, response):
        """Handle a request to move the arm to its predefined home position.

        Args:
            request (HomeArm.Request): Service request with optional motion
                parameters.
            response (HomeArm.Response): Service response to populate.

        Returns:
            HomeArm.Response: The populated service response.
        """
        self.get_logger().info("Service Call: Home Arm")
        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = "Sending arm to home position"
        self.publish_status()

        self._plan_and_move(
            lambda planner: self.send_home_goal(motion_params=request.motion_params, planner=planner),
            [self.PILZ_PTP, self.RRT_STAR],
            "Arm movement to Home",
            "Failed to initiate home movement (action server unavailable)",
            response,
        )
        return self.finalize_service_status(response)

    def handle_joint_move(self, request, response):
        """Move to a joint-space target - absolute by default, or relative
        to the current joint state (from the latest /joint_states message)
        if request.relative is True. Relative mode is for a delta on a
        single joint (e.g. 'pour's tilt, joint_6 alone) without needing to
        know or recompute the other joints' current values.

        If wait_for_completion is False, returns once either the goal is
        accepted and stays that way for FIRE_AND_FORGET_REJECTION_WINDOW_SEC,
        or it fails/succeeds within that window - whichever comes first.
        This lets a caller (e.g. 'throw's fling) do something else, like
        releasing the gripper, partway through a genuinely still-in-progress
        motion, while still catching a fast rejection instead of treating it
        as a success. A rejection arriving after the window would still be
        missed - this narrows that gap, it doesn't close it entirely.

        Args:
            request (JointMove.Request): Service request with target joint
                positions, relative-mode flag, completion behaviour, and
                motion parameters.
            response (JointMove.Response): Service response to populate.

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

        still_running = self._plan_and_move(
            lambda planner: self.send_joint_goal(joint_positions, motion_params=request.motion_params, planner=planner),
            [self.PILZ_PTP, self.RRT_STAR],
            f"Joint move to {joint_positions}",
            "Failed to initiate joint move",
            response,
            wait=request.wait_for_completion,
        )
        if still_running:
            return response
        return self.finalize_service_status(response)

    def handle_move_arm(self, request, response):
        """Handle a Cartesian arm movement request.

        Pilz PTP is tried first when an orientation is requested (it needs a
        full pose); position-only goals skip straight to OMPL RRT*, since
        Pilz needs a full pose to plan against.

        Args:
            request (MoveArm.Request): Service request with target position,
                optional orientation, and motion parameters.
            response (MoveArm.Response): Service response to populate.

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

        self._plan_and_move(
            lambda planner: self.send_goal(
                x, y, z,
                has_orientation=request.has_orientation,
                roll=request.roll,
                pitch=request.pitch,
                yaw=request.yaw,
                motion_params=request.motion_params,
                planner=planner,
            ),
            # Pilz needs a full pose, so position-only goals skip it
            [self.PILZ_PTP, self.RRT_STAR] if request.has_orientation else [self.RRT_STAR],
            f"Arm movement to ({x}, {y}, {z})",
            "Failed to initiate arm movement (action server unavailable)",
            response,
        )
        return self.finalize_service_status(response)

    def handle_relative_move(self, request, response):
        """Handle a relative Cartesian movement request.

        Looks up the tool frame's current pose via TF, adds the requested
        position and orientation deltas to it, and sends the resulting
        absolute target to MoveIt. Always keeps (or offsets) the current
        orientation, so there's no ``has_orientation`` flag here the way
        there is on :meth:`handle_move_arm`.

        Args:
            request (RelativeMove.Request): Service request with position and
                optional orientation deltas, and motion parameters.
            response (RelativeMove.Response): Service response to populate.

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

            # Keep the current orientation, plus any deltas. Deltas get odd
            # near +-90deg pitch (gimbal lock).
            q = trans.transform.rotation
            curr_roll, curr_pitch, curr_yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
            target_roll = curr_roll + request.roll_delta
            target_pitch = curr_pitch + request.pitch_delta
            target_yaw = curr_yaw + request.yaw_delta
            self.get_logger().info(
                f"Calculated target orientation (rpy): {target_roll}, {target_pitch}, {target_yaw}"
            )

            self._plan_and_move(
                lambda planner: self.send_goal(
                    target_x, target_y, target_z,
                    has_orientation=True,
                    roll=target_roll,
                    pitch=target_pitch,
                    yaw=target_yaw,
                    motion_params=request.motion_params,
                    planner=planner,
                ),
                [self.PILZ_PTP, self.RRT_STAR],
                f"Relative movement to ({target_x}, {target_y}, {target_z})",
                "Failed to initiate relative movement (action server unavailable)",
                response,
            )

        except Exception as e:
            self.get_logger().error(f"Could not calculate relative move: {e}")
            response.success = False
            response.message = f"Relative move TF lookup failed: {e}"

        return self.finalize_service_status(response)

    def handle_move_gripper(self, request, response):
        """Handle a gripper movement request.

        Args:
            request (MoveGripper.Request): Service request with the target
                gripper position.
            response (MoveGripper.Response): Service response to populate.

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
        """Clamp velocity/acceleration scale to [0.0, 1.0], warn if a caller sent something outside that range.

        Args:
            motion_params: Object with ``velocity_scale`` and
                ``acceleration_scale`` fields.

        Returns:
            tuple[float, float]: The clamped ``(velocity_scale,
                acceleration_scale)``.
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
    def _new_arm_goal(self, planner, motion_params):
        """Build an arm MoveGroup goal configured for the given planner, without constraints.

        Sets the planning group, pipeline, planner id, planning attempts, and
        allowed planning time from ``planner``, and applies velocity and
        acceleration scaling from ``motion_params`` if supplied. The caller
        adds position/orientation/joint constraints afterwards.

        Args:
            planner: Planner configuration (pipeline id, planner id, and
                planning-attempt settings) to build this goal for.
            motion_params: Optional movement parameters with velocity and
                acceleration scaling factors.

        Returns:
            MoveGroup.Goal: The constructed goal, without constraints.
        """
        goal_msg = MoveGroup.Goal()
        request = goal_msg.request
        request.group_name = self.PLANNING_GROUP
        request.pipeline_id = planner.pipeline_id
        request.planner_id = planner.planner_id
        request.num_planning_attempts = planner.num_attempts
        request.allowed_planning_time = self.ALLOWED_PLANNING_TIME_SEC

        if motion_params is not None:
            velocity_scale, acceleration_scale = self.clamp_motion_params(motion_params)
            if velocity_scale > 0.0:
                request.max_velocity_scaling_factor = velocity_scale
            if acceleration_scale > 0.0:
                request.max_acceleration_scaling_factor = acceleration_scale

        if planner.pipeline_id == self.PILZ_PTP.pipeline_id:
            # Pilz rejects 0; OMPL treats 0 as full speed, so match that
            request.max_velocity_scaling_factor = request.max_velocity_scaling_factor or 1.0
            request.max_acceleration_scaling_factor = request.max_acceleration_scaling_factor or 1.0
        return goal_msg

    def send_goal(self, x, y, z, planner, has_orientation=False, roll=0.0, pitch=0.0, yaw=0.0, motion_params=None):
        """Move tool_frame to (x, y, z), optionally with a fixed orientation.

        Reports only whether the goal could be dispatched; completion is
        reported asynchronously through the action callbacks.

        Args:
            x (float): Target X position in ``BASE_FRAME``.
            y (float): Target Y position in ``BASE_FRAME``.
            z (float): Target Z position in ``BASE_FRAME``.
            planner: Planner configuration used to build the underlying goal.
            has_orientation (bool): Whether an orientation constraint should
                be included.
            roll (float): Target roll angle in radians.
            pitch (float): Target pitch angle in radians.
            yaw (float): Target yaw angle in radians.
            motion_params: Optional movement parameters with velocity and
                acceleration scaling factors.

        Returns:
            bool: True if the action server is available and the goal was
                dispatched, otherwise False.
        """
        if not self.arm_client.wait_for_server(timeout_sec=self.SERVER_WAIT_TIMEOUT_SEC):
            self.get_logger().error('Arm server not available')
            return False

        goal_msg = self._new_arm_goal(planner, motion_params)

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
            qx, qy, qz, qw = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()
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
        return self._dispatch_arm_goal(goal_msg)

    def send_home_goal(self, planner, motion_params=None):
        """Send a joint-space goal using the predefined home configuration.

        Args:
            planner: Planner configuration used to build the underlying goal.
            motion_params: Optional movement parameters with velocity and
                acceleration scaling factors.

        Returns:
            bool: True if the action server is available and the goal was
                dispatched, otherwise False.
        """
        return self.send_joint_goal(self.HOME_JOINT_POSITIONS, motion_params=motion_params, planner=planner)

    def send_joint_goal(self, joint_positions, planner, motion_params=None):
        """Plan and execute a move to an absolute target for each of
        joint_1..joint_6, the same JointConstraint-based approach send_home_goal
        already used, just parameterized instead of hardcoded to home.

        Args:
            joint_positions (list[float]): Target positions for each robot
                joint, ordered according to ``JOINT_NAMES``.
            planner: Planner configuration used to build the underlying goal.
            motion_params: Optional movement parameters with velocity and
                acceleration scaling factors.

        Returns:
            bool: True if the action server is available and the goal was
                dispatched, otherwise False.
        """
        if not self.arm_client.wait_for_server(timeout_sec=self.SERVER_WAIT_TIMEOUT_SEC):
            self.get_logger().error('Arm server not available (Joint Move)')
            return False

        goal_msg = self._new_arm_goal(planner, motion_params)

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
        return self._dispatch_arm_goal(goal_msg)

    def _dispatch_arm_goal(self, goal_msg):
        """Clear arm movement state and submit an arm goal asynchronously.

        Shared by :meth:`send_goal` and :meth:`send_joint_goal` so both
        reset the same wait event and error-code tracking before dispatch.

        Args:
            goal_msg (MoveGroup.Goal): Fully constructed goal to send.

        Returns:
            bool: Always True; the action server's own acceptance/rejection
                is reported later through :meth:`goal_response_callback`.
        """
        self.arm_movement_finished.clear()
        self.arm_action_error_code = None
        future = self.arm_client.send_goal_async(
            goal_msg,
            feedback_callback=self.arm_feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        return True

    def move_gripper(self, position):
        """Send a target position to the gripper action server.

        Args:
            position (float): Target gripper position.

        Returns:
            bool: True if the gripper action server is available and the
                goal was dispatched, otherwise False.
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
        """Handle MoveIt's accept/reject response to a dispatched arm goal.

        If accepted, registers :meth:`result_callback` for the eventual
        result. If rejected or an exception occurs, records the failure and
        releases the movement wait event immediately.

        Args:
            future: Future containing MoveIt's goal handle.
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
        """Log MoveIt's execution-state feedback for an in-progress arm goal.

        Args:
            feedback_msg: MoveIt action feedback message.
        """
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'[Feedback] MoveIt State: {feedback.state}')

    def result_callback(self, future):
        """Process the final result of a dispatched MoveIt arm action.

        Maps MoveIt's error code to a human-readable message, updates the
        stored movement result, triggers :meth:`handle_moveit_failure` on
        failure, and releases the movement wait event.

        Args:
            future: Future containing MoveIt's action result.
        """
        try:
            result = future.result().result
            error_code = result.error_code.val
            self.arm_action_error_code = error_code

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
        """Handle the gripper action server's accept/reject response.

        If accepted, registers :meth:`gripper_result_callback` for the
        eventual result. If rejected or an exception occurs, records the
        failure and releases the gripper wait event immediately.

        Args:
            future: Future containing the gripper action's goal handle.
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
        """Log the gripper's current width while a gripper goal is in progress.

        Args:
            feedback_msg: Gripper action feedback message.
        """
        feedback = feedback_msg.feedback
        current_width = round(feedback.position, 3)
        self.get_logger().debug(f'[Feedback] Gripper Width: {current_width}')

    def gripper_result_callback(self, future):
        """Process the final result of a dispatched gripper action.

        Treats either reaching the goal or stalling (fingers met resistance,
        as against a real object) as success, updates the stored result, and
        releases the gripper wait event.

        Args:
            future: Future containing the gripper action's result.
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

    Initialises rclpy, creates the node, spins it on a multi-threaded
    executor, and performs node and rclpy shutdown on exit.

    Args:
        args (list[str], optional): Command-line arguments passed to rclpy.
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
