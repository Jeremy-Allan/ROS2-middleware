import math

import pytest

from unittest.mock import MagicMock, patch

from std_srvs.srv import Trigger
from example_interfaces.msg import Bool
from kinova_interfaces.msg import ExtendedStatus
from kinova_interfaces.srv import HomeArm, MoveArm, MoveGripper, RelativeMove, JointMove

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from control_msgs.action import GripperCommand


# Change this import only if your package/module name is different
from kinova_interface.nodes.hardware_interface_client import HardwareInterfaceClient

"""
Test with:
pytest src/ROS2-middleware/kinova_interface/test/test_hardware_interface_client.py -v
"""




@pytest.fixture
def node(ros_context):
    """
    Create the HardwareInterfaceClient without connecting to
    real ROS 2 action/service servers.
    """

    with patch(
        "kinova_interface.nodes.hardware_interface_client.ActionClient"
    ), patch(
        "kinova_interface.nodes.hardware_interface_client.TransformListener"
    ), patch.object(
        HardwareInterfaceClient,
        "create_timer",
        return_value=MagicMock()
    ):

        node = HardwareInterfaceClient()

    # Replace ROS clients with mocks
    node.arm_client = MagicMock()
    node.gripper_client = MagicMock()
    node.list_controllers_client = MagicMock()

    # Prevent real status publishing
    node.status_pub = MagicMock()

    yield node

    node.destroy_node()


# Node / Telemetry
def test_node_initialises(node):
    """Test that the node initialises correctly."""

    assert node.get_name() == "kinova_hardware_client"
    assert node.is_faulted is False
    assert node.fault_controller_warning_active is False
    assert node.current_state == ExtendedStatus.STATE_IDLE
    assert node.command_success is True

# publish_status()
def test_publish_status(node):
    """Test that publish_status publishes the current node status."""

    node.current_state = ExtendedStatus.STATE_BUSY
    node.status_text = "Testing status"
    node.command_success = True

    node.publish_status()

    node.status_pub.publish.assert_called_once()

    msg = node.status_pub.publish.call_args[0][0]

    assert msg.node_name == "kinova_hardware_client"
    assert msg.state == ExtendedStatus.STATE_BUSY
    assert msg.status_message == "Testing status"
    assert msg.last_command_valid is True


# check_fault_controller_health()
def test_check_fault_controller_health_service_unavailable(node):
    """Test health check when controller manager is unavailable."""

    node.list_controllers_client.service_is_ready.return_value = False

    node.check_fault_controller_health()

    node.list_controllers_client.call_async.assert_not_called()


def test_check_fault_controller_health(node):
    """Test health check starts a controller query."""

    node.list_controllers_client.service_is_ready.return_value = True

    fake_future = MagicMock()

    node.list_controllers_client.srv_type.Request.return_value = MagicMock()
    node.list_controllers_client.call_async.return_value = fake_future

    node.check_fault_controller_health()

    node.list_controllers_client.call_async.assert_called_once()
    fake_future.add_done_callback.assert_called_once()

# list_controllers_callback()
def test_list_controllers_callback_inactive(node):
    """Test detection of an inactive fault controller."""

    controller = MagicMock()
    controller.name = "fault_controller"
    controller.state = "inactive"

    response = MagicMock()
    response.controller = [controller]

    future = MagicMock()
    future.result.return_value = response

    node.list_controllers_callback(future)

    assert node.fault_controller_warning_active is True
    assert (
        node.status_text
        == "SYSTEM CONFIG WARNING: fault_controller is NOT active"
    )


def test_list_controllers_callback_active(node):
    """Test detection of an active fault controller."""

    node.fault_controller_warning_active = True
    node.status_text = (
        "SYSTEM CONFIG WARNING: fault_controller is NOT active"
    )

    controller = MagicMock()
    controller.name = "fault_controller"
    controller.state = "active"

    response = MagicMock()
    response.controller = [controller]

    future = MagicMock()
    future.result.return_value = response

    node.list_controllers_callback(future)

    assert node.fault_controller_warning_active is False
    assert node.status_text == "Hardware Interface Client Ready"


# finalize_service_status()
def test_finalize_service_status_success(node):
    """Test final service status after successful command."""

    response = Trigger.Response()
    response.success = True
    response.message = "Command successful"

    result = node.finalize_service_status(response)

    assert result is response
    assert node.current_state == ExtendedStatus.STATE_IDLE
    assert node.command_success is True
    assert node.status_text == "Command successful"


def test_finalize_service_status_fault(node):
    """Test final service status while hardware is faulted."""

    node.is_faulted = True
    node.status_text = "HARDWARE FAULT: Robot is faulted"

    response = Trigger.Response()
    response.success = False
    response.message = "Failed"

    node.finalize_service_status(response)

    assert node.current_state == ExtendedStatus.STATE_FAULT
    assert node.command_success is False

    # Fault-specific message should be preserved
    assert node.status_text == "HARDWARE FAULT: Robot is faulted"

# fault_callback()
def test_fault_callback_faulted(node):
    """Test entering a hardware fault state."""

    msg = Bool()
    msg.data = True

    node.fault_callback(msg)

    assert node.is_faulted is True
    assert node.current_state == ExtendedStatus.STATE_FAULT
    assert node.command_success is False
    assert node.status_text == "HARDWARE FAULT: Robot is faulted"


def test_fault_callback_fault_cleared(node):
    """Test clearing a hardware fault."""

    node.is_faulted = True

    msg = Bool()
    msg.data = False

    node.fault_callback(msg)

    assert node.is_faulted is False
    assert node.current_state == ExtendedStatus.STATE_IDLE
    assert node.command_success is True
    assert node.status_text == "Robot Ready (Fault Cleared)"

# handle_moveit_failure()
def test_handle_moveit_failure_with_fault(node):
    """Test MoveIt failure when hardware is faulted."""

    node.is_faulted = True

    node.handle_moveit_failure()

    assert (
        node.status_text
        == "MoveIt Failure: Hardware fault confirmed. Awaiting fault-reset command."
    )


def test_handle_moveit_failure_without_fault(node):
    """Test MoveIt failure when there is no hardware fault."""

    node.is_faulted = False
    old_status = node.status_text

    node.handle_moveit_failure()

    # Status should not be changed
    assert node.status_text == old_status


# handle_home_arm()
def test_handle_home_arm_success(node):
    """Test successful home arm service."""

    node.send_home_goal = MagicMock(return_value=True)
    node.arm_movement_finished = MagicMock()
    node.arm_movement_finished.wait.return_value = True
    node.arm_action_successful = True
    node.arm_action_message = "Arm moved home successfully"

    request = HomeArm.Request()
    response = HomeArm.Response()

    result = node.handle_home_arm(request, response)

    assert result.success is True
    assert result.message == "Arm moved home successfully (planned with Pilz PTP)"
    node.send_home_goal.assert_called_once()


def test_handle_home_arm_failure_to_start(node):
    """Test home arm when action cannot be started."""

    node.send_home_goal = MagicMock(return_value=False)

    request = HomeArm.Request()
    response = HomeArm.Response()

    result = node.handle_home_arm(request, response)

    assert result.success is False
    assert result.message == "Failed to initiate home movement (action server unavailable)"


# send_joint_goal() / handle_joint_move()
def test_send_joint_goal_custom_positions(node):
    """send_joint_goal should build the same kind of joint-constrained goal
    as send_home_goal, for arbitrary target positions."""

    node.arm_client.wait_for_server.return_value = True
    fake_future = MagicMock()
    node.arm_client.send_goal_async.return_value = fake_future

    positions = [0.5, 0.0, 1.0, 1.5708, 1.5708, 0.0]
    result = node.send_joint_goal(positions, node.PILZ_PTP)

    assert result is True

    goal = node.arm_client.send_goal_async.call_args[0][0]
    constraints = goal.request.goal_constraints[0].joint_constraints

    assert len(constraints) == 6
    assert constraints[0].joint_name == "joint_1"
    assert constraints[0].position == 0.5
    assert constraints[2].joint_name == "joint_3"
    assert constraints[2].position == 1.0


def test_send_home_goal_matches_send_joint_goal_defaults(node):
    """send_home_goal should just be send_joint_goal with home's fixed
    positions - refactored from its own inline copy of the same logic."""

    node.arm_client.wait_for_server.return_value = True
    node.arm_client.send_goal_async.return_value = MagicMock()

    node.send_home_goal(node.PILZ_PTP)

    goal = node.arm_client.send_goal_async.call_args[0][0]
    positions = [jc.position for jc in goal.request.goal_constraints[0].joint_constraints]

    assert positions == node.HOME_JOINT_POSITIONS


def test_handle_joint_move_waits_by_default(node):
    """handle_joint_move should wait for completion when wait_for_completion
    is True, via the shared _await_action helper like the other arm-move
    handlers."""

    node.send_joint_goal = MagicMock(return_value=True)
    node.arm_movement_finished = MagicMock()
    node.arm_movement_finished.wait.return_value = True
    node.arm_action_successful = True
    node.arm_action_message = "Joint move complete"

    request = JointMove.Request()
    request.joint_positions = [0.0, 0.0, 1.0, 1.5708, 1.5708, 0.0]
    request.wait_for_completion = True
    response = JointMove.Response()

    result = node.handle_joint_move(request, response)

    assert result.success is True
    assert result.message == "Joint move complete (planned with Pilz PTP)"
    node.arm_movement_finished.wait.assert_called_once()


def test_handle_joint_move_fire_and_forget_still_in_progress(node):
    """With wait_for_completion False, if the motion is still going after
    the short rejection-catching window, handle_joint_move should return
    without waiting further or touching current_state (the arm is still
    genuinely moving at that point)."""

    node.send_joint_goal = MagicMock(return_value=True)
    node.arm_movement_finished = MagicMock()
    node.arm_movement_finished.wait.return_value = False  # still in progress

    request = JointMove.Request()
    request.joint_positions = [0.0, 0.0, 0.5, 1.5708, 1.5708, 0.0]
    request.wait_for_completion = False
    response = JointMove.Response()

    result = node.handle_joint_move(request, response)

    assert result.success is True
    assert result.message == "Joint move goal accepted (not waiting for completion)"
    node.arm_movement_finished.wait.assert_called_once_with(
        timeout=node.FIRE_AND_FORGET_REJECTION_WINDOW_SEC
    )
    assert node.current_state == ExtendedStatus.STATE_BUSY


def test_handle_joint_move_fire_and_forget_catches_fast_rejection(node):
    """With wait_for_completion False, a rejection that arrives within the
    short catching window (e.g. an invalid combined joint state, which
    fails within milliseconds) should be reported as a real failure -
    not silently treated as accepted."""

    node.send_joint_goal = MagicMock(return_value=True)
    node.arm_movement_finished = MagicMock()
    node.arm_movement_finished.wait.return_value = True  # finished within the window
    node.arm_action_successful = False
    node.arm_action_message = "Arm movement failed"

    request = JointMove.Request()
    request.joint_positions = [0.0, 0.0, 2.3562, 1.5708, 1.5708, 0.0]
    request.wait_for_completion = False
    response = JointMove.Response()

    result = node.handle_joint_move(request, response)

    assert result.success is False
    assert result.message == "Arm movement failed"


def test_handle_joint_move_failure_to_start(node):
    """Test joint move when the action cannot be started at all."""

    node.send_joint_goal = MagicMock(return_value=False)

    request = JointMove.Request()
    request.joint_positions = [0.0] * 6
    request.wait_for_completion = True
    response = JointMove.Response()

    result = node.handle_joint_move(request, response)

    assert result.success is False
    assert result.message == "Failed to initiate joint move"


def test_on_joint_state_caches_latest_positions(node):
    """_on_joint_state should cache the latest name->position mapping,
    for relative joint moves to read current values from."""
    from sensor_msgs.msg import JointState

    msg = JointState()
    msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    msg.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    node._on_joint_state(msg)

    assert node.latest_joint_positions == {
        'joint_1': 0.1, 'joint_2': 0.2, 'joint_3': 0.3,
        'joint_4': 0.4, 'joint_5': 0.5, 'joint_6': 0.6,
    }


def test_handle_joint_move_relative_adds_delta_to_current(node):
    """A relative joint move should resolve to an absolute target of
    current + delta before actually moving, e.g. 'pour's tilt: a delta on
    joint_6 alone, without disturbing the other five joints."""

    node.latest_joint_positions = {
        'joint_1': 0.1, 'joint_2': 0.2, 'joint_3': 0.3,
        'joint_4': 0.4, 'joint_5': 0.5, 'joint_6': 0.6,
    }
    node.send_joint_goal = MagicMock(return_value=True)
    node.arm_movement_finished = MagicMock()
    node.arm_action_successful = True
    node.arm_action_message = "Joint move complete"

    request = JointMove.Request()
    request.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 2.36]
    request.wait_for_completion = True
    request.relative = True
    response = JointMove.Response()

    result = node.handle_joint_move(request, response)

    assert result.success is True
    node.send_joint_goal.assert_called_once()
    resolved_positions = node.send_joint_goal.call_args[0][0]
    assert resolved_positions == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6 + 2.36])


def test_handle_joint_move_relative_fails_without_joint_state(node):
    """A relative joint move should fail cleanly (not crash or silently
    treat deltas as absolutes) if no /joint_states message has arrived yet."""

    node.latest_joint_positions = None
    node.send_joint_goal = MagicMock()

    request = JointMove.Request()
    request.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 2.36]
    request.wait_for_completion = True
    request.relative = True
    response = JointMove.Response()

    result = node.handle_joint_move(request, response)

    assert result.success is False
    assert result.message == "No joint state available for relative joint move"
    node.send_joint_goal.assert_not_called()


# handle_move_arm()
def test_handle_move_arm_success(node):
    """Test successful arm movement service."""

    node.send_goal = MagicMock(return_value=True)
    node.arm_movement_finished = MagicMock()
    node.arm_movement_finished.wait.return_value = True
    node.arm_action_successful = True
    node.arm_action_message = "Arm moved to 0.5, 0.2, 0.3"

    request = MoveArm.Request()
    request.target_position.x = 0.5
    request.target_position.y = 0.2
    request.target_position.z = 0.3

    response = MoveArm.Response()

    result = node.handle_move_arm(request, response)

    assert result.success is True
    assert result.message == "Arm moved to 0.5, 0.2, 0.3 (planned with OMPL RRT*)"

    node.send_goal.assert_called_once_with(
        0.5,
        0.2,
        0.3,
        has_orientation=False,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        motion_params=request.motion_params,
        planner=node.RRT_STAR,
    )


def test_handle_move_arm_with_orientation_and_speed(node):
    """Test arm movement carries orientation and speed through to send_goal."""

    node.send_goal = MagicMock(return_value=True)
    node.arm_movement_finished = MagicMock()
    node.arm_action_successful = True

    request = MoveArm.Request()
    request.target_position.x = 0.5
    request.target_position.y = 0.2
    request.target_position.z = 0.3
    request.has_orientation = True
    request.roll = 0.1
    request.pitch = 1.57
    request.yaw = 0.0
    request.motion_params.velocity_scale = 0.5
    request.motion_params.acceleration_scale = 0.3

    response = MoveArm.Response()

    node.handle_move_arm(request, response)

    node.send_goal.assert_called_once_with(
        0.5,
        0.2,
        0.3,
        has_orientation=True,
        roll=0.1,
        pitch=1.57,
        yaw=0.0,
        motion_params=request.motion_params,
        planner=node.PILZ_PTP,
    )


# Planner selection: Pilz PTP first, OMPL RRT* fallback
def _oriented_request():
    request = MoveArm.Request()
    request.target_position.x = 0.4
    request.has_orientation = True
    request.roll = math.pi
    return request


def _goal_results(node, results):
    """send_goal/send_joint_goal mock whose Nth call leaves result N
    (success, message, error_code) behind, as result_callback would."""
    def send(*args, **kwargs):
        node.arm_action_successful, node.arm_action_message, node.arm_action_error_code = results.pop(0)
        return True
    node.arm_movement_finished = MagicMock()
    node.arm_movement_finished.wait.return_value = True
    return MagicMock(side_effect=send)


def test_handle_move_arm_with_orientation_uses_pilz_ptp(node):
    node.send_goal = _goal_results(node, [(True, "Movement complete", MoveItErrorCodes.SUCCESS)])

    result = node.handle_move_arm(_oriented_request(), MoveArm.Response())

    assert result.success is True
    assert result.message == "Movement complete (planned with Pilz PTP)"
    assert node.send_goal.call_args.kwargs["planner"] == node.PILZ_PTP


def test_handle_move_arm_falls_back_to_rrtstar(node):
    """PTP's direct path collides (INVALID_MOTION_PLAN): the same goal is
    re-planned with OMPL RRT*, which routes around obstacles."""
    node.send_goal = _goal_results(node, [
        (False, "Planning failed", MoveItErrorCodes.INVALID_MOTION_PLAN),
        (True, "Movement complete", MoveItErrorCodes.SUCCESS),
    ])

    result = node.handle_move_arm(_oriented_request(), MoveArm.Response())

    assert result.success is True
    assert result.message == "Movement complete (planned with OMPL RRT*)"
    first, fallback = node.send_goal.call_args_list
    assert (first.kwargs["planner"], fallback.kwargs["planner"]) == (node.PILZ_PTP, node.RRT_STAR)
    # Same target and orientation both times
    assert first.args == fallback.args
    assert fallback.kwargs["roll"] == pytest.approx(math.pi)


def test_handle_move_arm_no_fallback_after_execution_failure(node):
    """A failure during execution (the arm already moved) must not be
    retried with another planner."""
    node.send_goal = _goal_results(node, [(False, "Control failed", MoveItErrorCodes.CONTROL_FAILED)])

    result = node.handle_move_arm(_oriented_request(), MoveArm.Response())

    assert result.success is False
    assert node.send_goal.call_count == 1


def test_handle_move_arm_without_orientation_goes_straight_to_rrtstar(node):
    """Pilz needs a full target pose, so a position-only goal skips it."""
    node.send_goal = _goal_results(node, [(False, "Planning failed", MoveItErrorCodes.PLANNING_FAILED)])
    request = _oriented_request()
    request.has_orientation = False

    result = node.handle_move_arm(request, MoveArm.Response())

    assert result.success is False
    assert node.send_goal.call_count == 1
    assert node.send_goal.call_args.kwargs["planner"] == node.RRT_STAR


def test_handle_home_arm_falls_back_to_rrtstar(node):
    node.send_home_goal = _goal_results(node, [
        (False, "No IK", MoveItErrorCodes.PLANNING_FAILED),
        (True, "Movement complete", MoveItErrorCodes.SUCCESS),
    ])

    result = node.handle_home_arm(HomeArm.Request(), HomeArm.Response())

    assert result.success is True
    planners = [c.kwargs["planner"] for c in node.send_home_goal.call_args_list]
    assert planners == [node.PILZ_PTP, node.RRT_STAR]


def test_fire_and_forget_joint_move_falls_back_after_fast_pilz_failure(node):
    """throw's fling: a Pilz failure inside the rejection window is
    re-planned with RRT*, still without waiting for it to finish."""
    node.send_joint_goal = MagicMock(return_value=True)
    node.arm_movement_finished = MagicMock()
    # Pilz fails within the window; RRT* is still planning when the window ends
    node.arm_movement_finished.wait.side_effect = [True, False]
    node.arm_action_successful = False
    node.arm_action_message = "Planning failed"
    node.arm_action_error_code = MoveItErrorCodes.INVALID_MOTION_PLAN

    request = JointMove.Request()
    request.joint_positions = [0.0] * 6
    request.wait_for_completion = False

    result = node.handle_joint_move(request, JointMove.Response())

    assert result.success is True
    assert result.message == "Joint move goal accepted (not waiting for completion)"
    planners = [c.kwargs["planner"] for c in node.send_joint_goal.call_args_list]
    assert planners == [node.PILZ_PTP, node.RRT_STAR]


def test_send_goal_pilz_defaults_zero_scaling_to_full_speed(node):
    """Pilz rejects a 0 scaling factor; 0 means 'not given', which MoveIt
    treats as 1.0 for OMPL plans."""
    node.arm_client.wait_for_server.return_value = True
    node.arm_client.send_goal_async.return_value = MagicMock()

    node.send_goal(0.4, 0.0, 0.2, node.PILZ_PTP, has_orientation=True, roll=math.pi)

    request = node.arm_client.send_goal_async.call_args[0][0].request
    assert (request.pipeline_id, request.planner_id) == ("pilz_industrial_motion_planner", "PTP")
    assert request.num_planning_attempts == node.PILZ_PTP.num_attempts
    assert request.max_velocity_scaling_factor == 1.0
    assert request.max_acceleration_scaling_factor == 1.0


def test_send_goal_rrtstar_keeps_zero_scaling(node):
    """0 scaling is left for MoveIt to default on OMPL plans."""
    node.arm_client.wait_for_server.return_value = True
    node.arm_client.send_goal_async.return_value = MagicMock()

    node.send_goal(0.4, 0.0, 0.2, node.RRT_STAR)

    request = node.arm_client.send_goal_async.call_args[0][0].request
    assert request.max_velocity_scaling_factor == 0.0


def test_handle_move_arm_failure_to_start(node):
    """Test arm movement when action cannot be started."""

    node.send_goal = MagicMock(return_value=False)

    request = MoveArm.Request()
    request.target_position.x = 0.5
    request.target_position.y = 0.2
    request.target_position.z = 0.3

    response = MoveArm.Response()

    result = node.handle_move_arm(request, response)

    assert result.success is False
    assert result.message == "Failed to initiate arm movement (action server unavailable)"

# handle_relative_move()
def test_handle_relative_move_success(node):
    """Test successful relative movement."""

    transform = MagicMock()
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.translation.z = 3.0
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0

    node.tf_buffer.lookup_transform = MagicMock(
        return_value=transform
    )

    node.send_goal = MagicMock(return_value=True)
    node.arm_movement_finished = MagicMock()
    node.arm_movement_finished.wait.return_value = True
    node.arm_action_successful = True
    node.arm_action_message = "Relative movement complete"

    request = RelativeMove.Request()
    request.vx = 0.5
    request.vy = 0.5
    request.vz = 0.5

    response = RelativeMove.Response()

    result = node.handle_relative_move(request, response)

    assert result.success is True
    assert result.message == "Relative movement complete (planned with Pilz PTP)"

    # Keeps the current orientation (identity here), so Pilz can plan it
    node.send_goal.assert_called_once_with(
        1.5,
        2.5,
        3.5,
        has_orientation=True,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        motion_params=request.motion_params,
        planner=node.PILZ_PTP,
    )


def test_handle_relative_move_with_rotation_delta(node):
    """Test relative_move composes a rotation delta onto the arm's current orientation."""

    import math

    transform = MagicMock()
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.translation.z = 3.0
    # Identity orientation (no current rotation)
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0

    node.tf_buffer.lookup_transform = MagicMock(return_value=transform)
    node.send_goal = MagicMock(return_value=True)
    node.arm_action_successful = True

    request = RelativeMove.Request()
    request.vx = 0.0
    request.vy = 0.0
    request.vz = 0.0
    request.pitch_delta = 1.5708  # 90 degrees, starting from identity

    response = RelativeMove.Response()

    node.handle_relative_move(request, response)

    node.send_goal.assert_called_once()
    call_kwargs = node.send_goal.call_args.kwargs
    assert call_kwargs["has_orientation"] is True
    # Starting orientation is identity (roll=pitch=yaw=0), delta is
    # applied on top, so target pitch should be ~ the delta itself.
    assert math.isclose(call_kwargs["pitch"], 1.5708, abs_tol=1e-4)
    assert math.isclose(call_kwargs["roll"], 0.0, abs_tol=1e-4)
    assert math.isclose(call_kwargs["yaw"], 0.0, abs_tol=1e-4)


def test_handle_relative_move_tf_failure(node):
    """Test relative movement when TF lookup fails."""

    node.tf_buffer.lookup_transform = MagicMock(
        side_effect=Exception("TF unavailable")
    )

    request = RelativeMove.Request()
    request.vx = 0.5
    request.vy = 0.5
    request.vz = 0.5

    response = RelativeMove.Response()

    result = node.handle_relative_move(request, response)

    assert result.success is False
    assert "Relative move TF lookup failed" in result.message

# handle_move_gripper()
def test_handle_move_gripper_success(node):
    """Test successful gripper movement."""

    node.move_gripper = MagicMock(return_value=True)
    node.gripper_movement_finished = MagicMock()
    node.gripper_movement_finished.wait.return_value = True
    node.gripper_action_successful = True
    node.gripper_action_message = "Gripper moved to 1.0"

    request = MoveGripper.Request()
    request.position = 1.0

    response = MoveGripper.Response()

    result = node.handle_move_gripper(request, response)

    assert result.success is True
    assert result.message == "Gripper moved to 1.0"

    node.move_gripper.assert_called_once_with(1.0)


def test_handle_move_gripper_failure_to_start(node):
    """Test gripper movement when action cannot be started."""

    node.move_gripper = MagicMock(return_value=False)

    request = MoveGripper.Request()
    request.position = 1.0

    response = MoveGripper.Response()

    result = node.handle_move_gripper(request, response)

    assert result.success is False
    assert result.message == "Failed to initiate gripper movement (action server unavailable)"


# send_goal()
def test_send_goal_server_unavailable(node):
    """Test arm movement when MoveIt action server is unavailable."""

    node.arm_client.wait_for_server.return_value = False

    result = node.send_goal(1.0, 2.0, 3.0, node.RRT_STAR)

    assert result is False


def test_send_goal(node):
    """Test that send_goal creates and sends an arm goal."""

    node.arm_client.wait_for_server.return_value = True

    fake_future = MagicMock()

    node.arm_client.send_goal_async.return_value = fake_future

    result = node.send_goal(1.0, 2.0, 3.0, node.RRT_STAR)

    assert result is True

    node.arm_client.send_goal_async.assert_called_once()

    goal = node.arm_client.send_goal_async.call_args[0][0]

    assert isinstance(goal, MoveGroup.Goal)
    assert goal.request.group_name == "arm"
    assert (goal.request.pipeline_id, goal.request.planner_id) == ("ompl", "RRTstarkConfigDefault")
    assert goal.request.num_planning_attempts == node.RRT_STAR.num_attempts
    assert goal.request.allowed_planning_time == 10.0

    constraint = goal.request.goal_constraints[0]
    position_constraint = constraint.position_constraints[0]

    assert position_constraint.link_name == "tool_frame"

    pose = position_constraint.constraint_region.primitive_poses[0]

    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0


def test_send_goal_no_orientation_by_default(node):
    """Backward compatibility: default call adds no orientation constraint."""

    node.arm_client.wait_for_server.return_value = True
    node.arm_client.send_goal_async.return_value = MagicMock()

    node.send_goal(1.0, 2.0, 3.0, node.RRT_STAR)

    goal = node.arm_client.send_goal_async.call_args[0][0]
    constraint = goal.request.goal_constraints[0]

    assert len(constraint.orientation_constraints) == 0
    # No motion_params passed, no scaling fields should be touched (stay at default 0.0)
    assert goal.request.max_velocity_scaling_factor == 0.0


def test_send_goal_with_orientation(node):
    """has_orientation=True should add a real OrientationConstraint on tool_frame."""

    import math

    node.arm_client.wait_for_server.return_value = True
    node.arm_client.send_goal_async.return_value = MagicMock()

    node.send_goal(1.0, 2.0, 3.0, node.PILZ_PTP, has_orientation=True, roll=0.0, pitch=math.pi / 2, yaw=0.0)

    goal = node.arm_client.send_goal_async.call_args[0][0]
    constraint = goal.request.goal_constraints[0]

    assert len(constraint.orientation_constraints) == 1
    orient = constraint.orientation_constraints[0]
    assert orient.link_name == "tool_frame"
    # 90 degree pitch, verified against the known reference value from
    # the standalone math check: (0, 0, 0.7071, 0.7071) for 90deg yaw is
    # the analogous known case, this checks pitch instead.
    assert math.isclose(orient.orientation.y, math.sin(math.pi / 4), abs_tol=1e-4)
    assert math.isclose(orient.orientation.w, math.cos(math.pi / 4), abs_tol=1e-4)


def test_send_goal_applies_speed_scaling(node):
    """A valid motion_params should set MoveIt's scaling factors."""

    from kinova_interfaces.msg import MotionParams

    node.arm_client.wait_for_server.return_value = True
    node.arm_client.send_goal_async.return_value = MagicMock()

    params = MotionParams()
    params.velocity_scale = 0.5
    params.acceleration_scale = 0.3

    node.send_goal(1.0, 2.0, 3.0, node.RRT_STAR, motion_params=params)

    goal = node.arm_client.send_goal_async.call_args[0][0]
    assert goal.request.max_velocity_scaling_factor == 0.5
    assert goal.request.max_acceleration_scaling_factor == 0.3


def test_send_goal_clamps_out_of_range_speed(node):
    """Values outside [0.0, 1.0] must be clamped, not passed through raw."""

    from kinova_interfaces.msg import MotionParams

    node.arm_client.wait_for_server.return_value = True
    node.arm_client.send_goal_async.return_value = MagicMock()

    params = MotionParams()
    params.velocity_scale = 5.0
    params.acceleration_scale = -1.0

    node.send_goal(1.0, 2.0, 3.0, node.RRT_STAR, motion_params=params)

    goal = node.arm_client.send_goal_async.call_args[0][0]
    assert goal.request.max_velocity_scaling_factor == 1.0
    # -1.0 clamps to 0.0, which per the "0.0 means use default" contract
    # means the field is left untouched (never set), not set to 0.0 explicitly.
    assert goal.request.max_acceleration_scaling_factor == 0.0


def test_send_home_goal_server_unavailable(node):
    """Test home goal when MoveIt server is unavailable."""

    node.arm_client.wait_for_server.return_value = False

    result = node.send_home_goal(node.PILZ_PTP)

    assert result is False

# send_home_goal()
def test_send_home_goal(node):
    """Test that send_home_goal creates the expected joint constraints."""

    node.arm_client.wait_for_server.return_value = True

    fake_future = MagicMock()
    node.arm_client.send_goal_async.return_value = fake_future

    result = node.send_home_goal(node.PILZ_PTP)

    assert result is True

    node.arm_client.send_goal_async.assert_called_once()

    goal = node.arm_client.send_goal_async.call_args[0][0]

    assert isinstance(goal, MoveGroup.Goal)
    assert goal.request.group_name == "arm"

    constraints = goal.request.goal_constraints[0].joint_constraints

    assert len(constraints) == 6

    assert constraints[0].joint_name == "joint_1"
    assert constraints[0].position == 0.0

    assert constraints[2].joint_name == "joint_3"
    assert constraints[2].position == 1.5708

# move_gripper()
def test_move_gripper_server_unavailable(node):
    """Test gripper movement when action server is unavailable."""

    node.gripper_client.wait_for_server.return_value = False

    result = node.move_gripper(1.0)

    assert result is False


def test_move_gripper(node):
    """Test that move_gripper sends the requested gripper position."""

    node.gripper_client.wait_for_server.return_value = True

    fake_future = MagicMock()

    node.gripper_client.send_goal_async.return_value = fake_future

    result = node.move_gripper(1.0)

    assert result is True

    node.gripper_client.send_goal_async.assert_called_once()

    goal = node.gripper_client.send_goal_async.call_args[0][0]

    assert isinstance(goal, GripperCommand.Goal)
    assert goal.command.position == pytest.approx(1.0)


# goal_response_callback()
def test_goal_response_callback_rejected(node):
    """Test rejected arm goal."""

    goal_handle = MagicMock()
    goal_handle.accepted = False

    future = MagicMock()
    future.result.return_value = goal_handle

    node.goal_response_callback(future)

    assert node.arm_action_successful is False
    assert node.arm_movement_finished.is_set()


def test_goal_response_callback_accepted(node):
    """Test accepted arm goal."""

    goal_handle = MagicMock()
    goal_handle.accepted = True

    result_future = MagicMock()
    goal_handle.get_result_async.return_value = result_future

    future = MagicMock()
    future.result.return_value = goal_handle

    node.goal_response_callback(future)

    goal_handle.get_result_async.assert_called_once()
    result_future.add_done_callback.assert_called_once()

# arm_feedback_callback()
def test_arm_feedback_callback(node):
    """Test arm feedback callback executes without error."""

    feedback = MagicMock()
    feedback.state = 1

    feedback_msg = MagicMock()
    feedback_msg.feedback = feedback

    node.arm_feedback_callback(feedback_msg)

# result_callback()
def test_result_callback_success(node):
    """Test successful MoveIt result."""

    result = MagicMock()

    result.error_code.val = result.error_code.SUCCESS

    future = MagicMock()
    future.result.return_value.result = result

    node.result_callback(future)

    assert node.arm_action_successful is True
    assert node.arm_movement_finished.is_set()
    # current_state must reset even with nobody waiting (a fire-and-forget
    # joint move has no finalize_service_status caller to do this instead)
    assert node.current_state == ExtendedStatus.STATE_IDLE
    node.status_pub.publish.assert_called_once()


def test_result_callback_failure(node):
    """Test failed MoveIt result."""

    result = MagicMock()

    result.error_code.val = result.error_code.PLANNING_FAILED

    future = MagicMock()
    future.result.return_value.result = result

    node.handle_moveit_failure = MagicMock()

    node.result_callback(future)

    assert node.arm_action_successful is False
    assert node.arm_movement_finished.is_set()
    assert node.current_state == ExtendedStatus.STATE_IDLE
    assert node.status_text == "Movement failed"

    node.handle_moveit_failure.assert_called_once()

# gripper_response_callback()
def test_gripper_response_callback_rejected(node):
    """Test rejected gripper goal."""

    goal_handle = MagicMock()
    goal_handle.accepted = False

    future = MagicMock()
    future.result.return_value = goal_handle

    node.gripper_response_callback(future)

    assert node.gripper_action_successful is False
    assert node.gripper_movement_finished.is_set()


def test_gripper_response_callback_accepted(node):
    """Test accepted gripper goal."""

    goal_handle = MagicMock()
    goal_handle.accepted = True

    result_future = MagicMock()
    goal_handle.get_result_async.return_value = result_future

    future = MagicMock()
    future.result.return_value = goal_handle

    node.gripper_response_callback(future)

    goal_handle.get_result_async.assert_called_once()
    result_future.add_done_callback.assert_called_once()

#gripper_feedback_callback()
def test_gripper_feedback_callback(node):
    """Test gripper feedback callback executes without error."""

    feedback = MagicMock()
    feedback.position = 1.0

    feedback_msg = MagicMock()
    feedback_msg.feedback = feedback

    node.gripper_feedback_callback(feedback_msg)

#gripper_result_callback()
def test_gripper_result_callback(node):
    """Test successful gripper result."""

    result = MagicMock()
    result.position = 1.0
    result.effort = 0.0
    result.stalled = False
    result.reached_goal = True

    future = MagicMock()
    future.result.return_value.result = result

    node.gripper_result_callback(future)

    assert node.gripper_action_successful is True
    assert node.gripper_movement_finished.is_set()


# _await_action()
def test_await_action_evaluates_attributes_after_wait(node):
    """Test that _await_action reads attributes after the wait completes, not before."""
    node.arm_action_successful = False
    node.arm_action_message = "Initial stale message"

    def simulate_action_completion(timeout):
        # Simulate action result callback updating the node attributes while wait is blocked
        node.arm_action_successful = True
        node.arm_action_message = "Fresh updated message"
        return True

    mock_event = MagicMock()
    mock_event.wait.side_effect = simulate_action_completion

    response = HomeArm.Response()
    result = node._await_action(
        mock_event,
        10.0,
        'arm_action_successful',
        'arm_action_message',
        "Test action",
        response
    )

    assert result.success is True
    assert result.message == "Fresh updated message"


def test_await_action_timeout(node):
    """Test that _await_action properly handles timeout."""
    mock_event = MagicMock()
    mock_event.wait.return_value = False

    response = HomeArm.Response()
    result = node._await_action(
        mock_event,
        5.0,
        'arm_action_successful',
        'arm_action_message',
        "Test action",
        response
    )

    assert result.success is False
    assert "timed out after 5.0s" in result.message
