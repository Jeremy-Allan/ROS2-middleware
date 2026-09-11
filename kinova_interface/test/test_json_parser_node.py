import json
import pytest

from unittest.mock import MagicMock, patch

from kinova_interface.json_parser_node import JsonParser, JsonParserNode

from kinova_interfaces.srv import (
    GetObjectCoordinates,
    GetRelativeMovement,
    GetObjectInfo,
    GetOrientationPreset,
    HomeArm,
    MoveArm,
    MoveGripper,
    RelativeMove,
)
from kinova_interfaces.msg import ExtendedStatus, MotionParams
from shape_msgs.msg import SolidPrimitive


"""
Test with: 
pytest src/ROS2-middleware/kinova_interface/test/test_json_parser_node.py -v
"""

# Fixtures

@pytest.fixture
def node(ros_context):
    """
    Create the JsonParserNode without running real timers.
    """
    with patch.object(JsonParserNode, "create_timer") as mock_timer:
        mock_timer.return_value = MagicMock()

        node = JsonParserNode()

    yield node
    node.destroy_node()


@pytest.fixture
def parser():
    """Create a standalone JsonParser."""
    return JsonParser(MagicMock())


# JsonParser
def test_parser_load_file(parser, tmp_path):
    """Test loading a recipe from a JSON file."""

    recipe = {
        "recipe_name": "Test",
        "steps": [
            {"action": "home"}
        ]
    }

    file = tmp_path / "recipe.json"
    file.write_text(json.dumps(recipe))

    assert parser.load_recipe_from_file(str(file)) is True
    assert parser.recipe == recipe
    assert parser.get_recipe_steps() == recipe["steps"]


def test_parser_load_service(parser):
    """Test loading a recipe from a JSON string."""

    recipe = {
        "recipe_name": "Test",
        "steps": [
            {"action": "gripper", "parameters": {"position": 0.5}}
        ]
    }

    assert parser.load_recipe_from_service(json.dumps(recipe)) is True
    assert parser.recipe == recipe


def test_parser_invalid_json(parser):
    """Invalid JSON should return False."""

    assert parser.load_recipe_from_service("invalid json") is False


def test_parser_no_recipe(parser):
    """No recipe should return an empty step list."""

    assert parser.get_recipe_steps() == []


# Node Initialisation
def test_node_initialises(node):
    """Test that the node and its clients are created."""

    assert node.parser is not None
    assert node.home_client is not None
    assert node.move_arm_client is not None
    assert node.move_gripper_client is not None
    assert node.relative_move_client is not None
    assert node.coord_client is not None
    assert node.relative_client is not None
    assert node.status_pub is not None
    assert node.execute_srv is not None


# wait_for_future()
def test_wait_for_future(node):
    """Test that wait_for_future returns the service response."""

    future = MagicMock()
    response = MagicMock()

    future.done.return_value = True
    future.result.return_value = response

    result = node.wait_for_future(future, "/test")

    assert result == response


# publish_status()
def test_publish_status(node):
    """Test that node status is published correctly."""

    node.status_pub.publish = MagicMock()

    node.current_state = ExtendedStatus.STATE_BUSY
    node.status_text = "Testing"
    node.command_success = True

    node.publish_status()

    node.status_pub.publish.assert_called_once()

    msg = node.status_pub.publish.call_args[0][0]

    assert msg.node_name == node.get_name()
    assert msg.state == ExtendedStatus.STATE_BUSY
    assert msg.status_message == "Testing"
    assert msg.last_command_valid is True


# get_static_object_coords()
# Note: get_static_object_coords() actually calls get_object_info() (the
# /get_object_info service via info_client), not /get_coordinates via
# coord_client, so these tests mock info_client to match real behaviour.
def test_get_static_object_coords(node):
    """Test getting object coordinates."""

    node.info_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = GetObjectInfo.Response()
    response.success = True
    response.pose.position.x = 1.0
    response.pose.position.y = 2.0
    response.pose.position.z = 3.0

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    node.info_client.call_async = MagicMock(
        return_value=future
    )

    result = node.get_static_object_coords("cube")

    assert result == {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0
    }

    request = node.info_client.call_async.call_args[0][0]
    assert request.object_id == "cube"


def test_get_static_object_coords_unavailable(node):
    """Test coordinate service unavailable."""

    node.info_client.wait_for_service = MagicMock(
        return_value=False
    )

    assert node.get_static_object_coords("cube") is None


#get_relative_movement_vector()
def test_get_relative_movement_vector(node):
    """Test getting a relative movement vector."""

    node.relative_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = GetRelativeMovement.Response()
    response.success = True
    response.x = 0.1
    response.y = 0.2
    response.z = 0.3

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    node.relative_client.call_async = MagicMock(
        return_value=future
    )

    result = node.get_relative_movement_vector("forward")

    assert result == {
        "x": 0.1,
        "y": 0.2,
        "z": 0.3
    }


# call_home_service()
def test_call_home_service(node):
    """Test the home service."""

    node.home_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = HomeArm.Response()
    response.success = True
    response.message = "Homed"

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    node.home_client.call_async = MagicMock(
        return_value=future
    )

    result = node.call_home_service()

    assert result["success"] is True
    assert result["message"] == "Homed"


# call_move_service()
def test_call_move_service(node):
    """Test the move arm service."""

    node.move_arm_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = MoveArm.Response()
    response.success = True
    response.message = "Moved"

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    node.move_arm_client.call_async = MagicMock(
        return_value=future
    )

    result = node.call_move_service(1.0, 2.0, 3.0)

    assert result["success"] is True

    request = node.move_arm_client.call_async.call_args[0][0]

    assert request.target_position.x == 1.0
    assert request.target_position.y == 2.0
    assert request.target_position.z == 3.0
    assert request.has_orientation is False


def test_call_move_service_with_orientation_and_speed(node):
    """Test the move arm service passes orientation and speed through."""

    node.move_arm_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = MoveArm.Response()
    response.success = True
    response.message = "Moved"

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    node.move_arm_client.call_async = MagicMock(
        return_value=future
    )

    motion_params = MotionParams()
    motion_params.velocity_scale = 0.5
    motion_params.acceleration_scale = 0.5

    result = node.call_move_service(
        1.0, 2.0, 3.0,
        has_orientation=True, roll=0.0, pitch=1.57, yaw=0.0,
        motion_params=motion_params
    )

    assert result["success"] is True

    request = node.move_arm_client.call_async.call_args[0][0]

    assert request.has_orientation is True
    assert request.pitch == 1.57
    assert request.motion_params.velocity_scale == 0.5


#call_relative_move_service()
def test_call_relative_move_service(node):
    """Test relative movement service."""

    node.relative_move_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = RelativeMove.Response()
    response.success = True
    response.message = "Moved"

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    node.relative_move_client.call_async = MagicMock(
        return_value=future
    )

    result = node.call_relative_move_service(
        0.1,
        0.2,
        0.3
    )

    assert result["success"] is True

    request = node.relative_move_client.call_async.call_args[0][0]

    assert request.vx == 0.1
    assert request.vy == 0.2
    assert request.vz == 0.3
    assert request.has_orientation is False


#call_move_gripper_service()
def test_call_move_gripper_service(node):
    """Test gripper service."""

    node.move_gripper_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = MoveGripper.Response()
    response.success = True
    response.message = "Gripper moved"

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    node.move_gripper_client.call_async = MagicMock(
        return_value=future
    )

    result = node.call_move_gripper_service(0.5)

    assert result["success"] is True

    request = node.move_gripper_client.call_async.call_args[0][0]

    assert request.position == 0.5


# execute_recipe()
def test_execute_recipe(node):
    """Test executing a recipe containing several action types."""

    node.parser.recipe = {
        "recipe_name": "Test Recipe",
        "steps": [
            {"action": "home"},
            {
                "action": "move_arm",
                "parameters": {
                    "target": "cube"
                }
            },
            {
                "action": "relative_move",
                "parameters": {
                    "vector": "forward"
                }
            },
            {
                "action": "gripper",
                "parameters": {
                    "position": 0.5
                }
            }
        ]
    }

    node.call_home_service = MagicMock(
        return_value={"success": True}
    )

    node.get_static_object_coords = MagicMock(
        return_value={
            "x": 1.0,
            "y": 2.0,
            "z": 3.0
        }
    )

    node.call_move_service = MagicMock(
        return_value={"success": True}
    )

    node.get_relative_movement_vector = MagicMock(
        return_value={
            "x": 0.1,
            "y": 0.2,
            "z": 0.3
        }
    )

    node.call_relative_move_service = MagicMock(
        return_value={"success": True}
    )

    node.call_move_gripper_service = MagicMock(
        return_value={"success": True}
    )

    node.publish_status = MagicMock()

    with patch("kinova_interface.json_parser_node.time.sleep"):
        result = node.execute_recipe()

    assert result is True

    node.call_home_service.assert_called_once()
    node.call_move_service.assert_called_once_with(
        1.0, 2.0, 3.0, False, 0.0, 0.0, 0.0, MotionParams()
    )
    node.call_relative_move_service.assert_called_once_with(
        0.1, 0.2, 0.3, False, 0.0, 0.0, 0.0, MotionParams()
    )
    node.call_move_gripper_service.assert_called_once_with(0.5)


def test_execute_recipe_no_steps(node):
    """Recipe with no steps should fail."""

    node.parser.recipe = None
    node.publish_status = MagicMock()

    assert node.execute_recipe() is False


# execute_recipe_callback()
def test_execute_recipe_callback(node):
    """Test dynamic recipe execution."""

    request = MagicMock()
    response = MagicMock()

    request.recipe_json = json.dumps({
        "recipe_name": "Dynamic Recipe",
        "steps": [
            {"action": "home"}
        ]
    })

    node.execute_recipe = MagicMock(
        return_value=True
    )

    node.publish_status = MagicMock()

    result = node.execute_recipe_callback(
        request,
        response
    )

    assert result == response
    assert response.success is True
    assert response.message == "Recipe executed successfully."


def test_execute_recipe_callback_invalid_json(node):
    """Invalid dynamic recipe should fail."""

    request = MagicMock()
    response = MagicMock()

    request.recipe_json = "invalid json"

    node.publish_status = MagicMock()

    result = node.execute_recipe_callback(
        request,
        response
    )

    assert result == response
    assert response.success is False


#startup_timer_callback()
def test_startup_timer_callback(node):
    """Test startup timer starts recipe execution."""

    node.startup_timer = MagicMock()
    node.execute_recipe = MagicMock()

    node.startup_timer_callback()

    node.startup_timer.cancel.assert_called_once()
    node.execute_recipe.assert_called_once()


# resolve_orientation()
def test_resolve_orientation_none_requested(node):
    """No preset name means no orientation, no service call needed."""

    node.get_orientation_preset = MagicMock()

    result = node.resolve_orientation(None)

    assert result == (False, 0.0, 0.0, 0.0)
    node.get_orientation_preset.assert_not_called()


def test_resolve_orientation_valid_preset(node):
    """A valid preset name resolves to its angles with has_orientation True."""

    node.get_orientation_preset = MagicMock(
        return_value={"roll": 0.0, "pitch": 1.57, "yaw": 0.0}
    )

    result = node.resolve_orientation("tilted_for_pour")

    assert result == (True, 0.0, 1.57, 0.0)


def test_resolve_orientation_unknown_preset(node):
    """An unresolvable preset name returns None, distinct from 'not requested'."""

    node.get_orientation_preset = MagicMock(return_value=None)

    result = node.resolve_orientation("not_a_real_preset")

    assert result is None


# build_motion_params()
def test_build_motion_params_none(node):
    """No speed given means both scales stay at the 0.0 'use arm default'."""

    params = node.build_motion_params(None)

    assert params.velocity_scale == 0.0
    assert params.acceleration_scale == 0.0


def test_build_motion_params_with_speed(node):
    """A speed value sets both velocity and acceleration scale."""

    params = node.build_motion_params(0.4)

    assert params.velocity_scale == 0.4
    assert params.acceleration_scale == 0.4


# object_half_height()
def test_object_half_height_box(node):
    shape = {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.08]}
    assert node.object_half_height(shape) == pytest.approx(0.04)


def test_object_half_height_cylinder(node):
    shape = {"type": SolidPrimitive.CYLINDER, "dimensions": [0.1, 0.02]}
    assert node.object_half_height(shape) == pytest.approx(0.05)


def test_object_half_height_sphere(node):
    shape = {"type": SolidPrimitive.SPHERE, "dimensions": [0.03]}
    assert node.object_half_height(shape) == pytest.approx(0.03)


# _dispatch_step() / execute_recipe() all_success regression
def test_execute_recipe_pickup_failure_sets_all_success_false(node):
    """Regression test: a mid-pickup failure must fail the whole recipe, not
    just break the loop silently (the bug 4.2 in the implementation plan fixes)."""

    node.parser.recipe = {
        "recipe_name": "Test",
        "steps": [
            {"action": "pickup", "parameters": {"target": "red_cube"}}
        ]
    }

    node.get_static_object_coords = MagicMock(
        return_value={"x": 1.0, "y": 2.0, "z": 3.0}
    )
    # Gripper open succeeds, but the move to the object fails
    node.call_move_gripper_service = MagicMock(return_value={"success": True})
    node.call_move_service = MagicMock(return_value={"success": False})
    node.publish_status = MagicMock()

    with patch("kinova_interface.json_parser_node.time.sleep"):
        result = node.execute_recipe()

    assert result is False
    assert node.command_success is False


def test_execute_recipe_unknown_action_fails(node):
    """An action with no registered handler should fail the step, not crash."""

    node.parser.recipe = {
        "recipe_name": "Test",
        "steps": [
            {"action": "not_a_real_action", "parameters": {}}
        ]
    }
    node.publish_status = MagicMock()

    with patch("kinova_interface.json_parser_node.time.sleep"):
        result = node.execute_recipe()

    assert result is False


# _handle_dropoff() two-stage descent and stacking height
def test_handle_dropoff_two_stage_descent_with_stacking(node):
    """dropoff should hover above, then lower to a small clearance above the
    computed stacking height (destination top + target's own half-height),
    not release from the hover height."""

    def object_info_side_effect(name):
        if name == "delivery_tray":
            return {
                "pose": {"position": {"x": 0.5, "y": 0.1, "z": 0.0}, "orientation": {}},
                "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.3, 0.2, 0.02]}
            }
        if name == "red_cube":
            return {
                "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
                "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
            }
        return None

    node.get_object_info = MagicMock(side_effect=object_info_side_effect)
    node.call_move_service = MagicMock(return_value={"success": True})
    node.call_move_gripper_service = MagicMock(return_value={"success": True})
    node.update_object_pose = MagicMock(return_value=True)
    node.detach_object = MagicMock(return_value=True)

    result = node._handle_dropoff({
        "target": "red_cube",
        "destination": "delivery_tray",
        "place_offset": 0.1
    })

    assert result is True
    assert node.call_move_service.call_count == 2

    # dest top = 0.0 + 0.01 (half of 0.02 tray height) = 0.01
    # target half height = 0.025 (half of 0.05 cube)
    # release_z = 0.01 + 0.025 = 0.035
    hover_call = node.call_move_service.call_args_list[0][0]
    release_call = node.call_move_service.call_args_list[1][0]

    assert hover_call[2] == pytest.approx(0.035 + 0.1)
    assert release_call[2] == pytest.approx(0.035 + 0.02)
    # the release move must be lower than the hover move, not the same height
    assert release_call[2] < hover_call[2]

