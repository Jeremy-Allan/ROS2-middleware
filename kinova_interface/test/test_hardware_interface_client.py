import pytest

from unittest.mock import MagicMock, patch

from std_srvs.srv import Trigger
from example_interfaces.msg import Bool
from kinova_interfaces.msg import ExtendedStatus
from kinova_interfaces.srv import HomeArm, MoveArm, MoveGripper, RelativeMove

from moveit_msgs.action import MoveGroup
from control_msgs.action import GripperCommand


# Change this import only if your package/module name is different
from kinova_interface.hardware_interface_client import HardwareInterfaceClient

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
        "kinova_interface.hardware_interface_client.ActionClient"
    ), patch(
        "kinova_interface.hardware_interface_client.TransformListener"
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
    node.movement_finished = MagicMock()
    node.last_action_successful = True

    request = HomeArm.Request()
    response = HomeArm.Response()

    result = node.handle_home_arm(request, response)

    assert result.success is True
    assert result.message == "Arm moved home successfully"
    node.send_home_goal.assert_called_once()


def test_handle_home_arm_failure_to_start(node):
    """Test home arm when action cannot be started."""

    node.send_home_goal = MagicMock(return_value=False)

    request = HomeArm.Request()
    response = HomeArm.Response()

    result = node.handle_home_arm(request, response)

    assert result.success is False
    assert result.message == "Failed to initiate home movement"

# handle_move_arm()
def test_handle_move_arm_success(node):
    """Test successful arm movement service."""

    node.send_goal = MagicMock(return_value=True)
    node.movement_finished = MagicMock()
    node.last_action_successful = True

    request = MoveArm.Request()
    request.x = 0.5
    request.y = 0.2
    request.z = 0.3

    response = MoveArm.Response()

    result = node.handle_move_arm(request, response)

    assert result.success is True
    assert result.message == "Arm moved to 0.5, 0.2, 0.3"

    node.send_goal.assert_called_once_with(
        0.5,
        0.2,
        0.3
    )


def test_handle_move_arm_failure_to_start(node):
    """Test arm movement when action cannot be started."""

    node.send_goal = MagicMock(return_value=False)

    request = MoveArm.Request()
    request.x = 0.5
    request.y = 0.2
    request.z = 0.3

    response = MoveArm.Response()

    result = node.handle_move_arm(request, response)

    assert result.success is False
    assert result.message == "Failed to initiate arm movement"

# handle_relative_move()
def test_handle_relative_move_success(node):
    """Test successful relative movement."""

    transform = MagicMock()
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.translation.z = 3.0

    node.tf_buffer.lookup_transform = MagicMock(
        return_value=transform
    )

    node.send_goal = MagicMock(return_value=True)
    node.last_action_successful = True

    request = RelativeMove.Request()
    request.vx = 0.5
    request.vy = 0.5
    request.vz = 0.5

    response = RelativeMove.Response()

    result = node.handle_relative_move(request, response)

    assert result.success is True
    assert result.message == "Relative movement complete"

    node.send_goal.assert_called_once_with(
        1.5,
        2.5,
        3.5
    )


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
    assert result.message == "TF unavailable"

# handle_move_gripper()
def test_handle_move_gripper_success(node):
    """Test successful gripper movement."""

    node.move_gripper = MagicMock(return_value=True)
    node.movement_finished = MagicMock()
    node.last_action_successful = True

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
    assert result.message == "Failed to initiate gripper movement"


# send_goal()
def test_send_goal_server_unavailable(node):
    """Test arm movement when MoveIt action server is unavailable."""

    node.arm_client.wait_for_server.return_value = False

    result = node.send_goal(1.0, 2.0, 3.0)

    assert result is False


def test_send_goal(node):
    """Test that send_goal creates and sends an arm goal."""

    node.arm_client.wait_for_server.return_value = True

    fake_future = MagicMock()

    node.arm_client.send_goal_async.return_value = fake_future

    result = node.send_goal(1.0, 2.0, 3.0)

    assert result is True

    node.arm_client.send_goal_async.assert_called_once()

    goal = node.arm_client.send_goal_async.call_args[0][0]

    assert isinstance(goal, MoveGroup.Goal)
    assert goal.request.group_name == "arm"
    assert goal.request.num_planning_attempts == 10
    assert goal.request.allowed_planning_time == 5.0

    constraint = goal.request.goal_constraints[0]
    position_constraint = constraint.position_constraints[0]

    assert position_constraint.link_name == "tool_frame"

    pose = position_constraint.constraint_region.primitive_poses[0]

    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 3.0


def test_send_home_goal_server_unavailable(node):
    """Test home goal when MoveIt server is unavailable."""

    node.arm_client.wait_for_server.return_value = False

    result = node.send_home_goal()

    assert result is False

# send_home_goal()
def test_send_home_goal(node):
    """Test that send_home_goal creates the expected joint constraints."""

    node.arm_client.wait_for_server.return_value = True

    fake_future = MagicMock()
    node.arm_client.send_goal_async.return_value = fake_future

    result = node.send_home_goal()

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

    assert node.last_action_successful is False
    assert node.movement_finished.is_set()


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

    assert node.last_action_successful is True
    assert node.movement_finished.is_set()


def test_result_callback_failure(node):
    """Test failed MoveIt result."""

    result = MagicMock()

    result.error_code.val = result.error_code.PLANNING_FAILED

    future = MagicMock()
    future.result.return_value.result = result

    node.handle_moveit_failure = MagicMock()

    node.result_callback(future)

    assert node.last_action_successful is False
    assert node.movement_finished.is_set()

    node.handle_moveit_failure.assert_called_once()

# gripper_response_callback()
def test_gripper_response_callback_rejected(node):
    """Test rejected gripper goal."""

    goal_handle = MagicMock()
    goal_handle.accepted = False

    future = MagicMock()
    future.result.return_value = goal_handle

    node.gripper_response_callback(future)

    assert node.last_action_successful is False
    assert node.movement_finished.is_set()


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

    future = MagicMock()

    node.gripper_result_callback(future)

    assert node.last_action_successful is True
    assert node.movement_finished.is_set()