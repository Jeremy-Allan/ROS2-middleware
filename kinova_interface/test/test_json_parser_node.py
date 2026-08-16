import json
import pytest

from unittest.mock import MagicMock, patch

from kinova_interface.json_parser_node import JsonParser, JsonParserNode

from kinova_interfaces.srv import (
    GetObjectCoordinates,
    GetRelativeMovement,
    HomeArm,
    MoveArm,
    MoveGripper,
    RelativeMove,
)
from kinova_interfaces.msg import ExtendedStatus


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
def test_get_static_object_coords(node):
    """Test getting object coordinates."""

    node.coord_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = GetObjectCoordinates.Response()
    response.success = True
    response.x = 1.0
    response.y = 2.0
    response.z = 3.0

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    node.coord_client.call_async = MagicMock(
        return_value=future
    )

    result = node.get_static_object_coords("cube")

    assert result == {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0
    }

    request = node.coord_client.call_async.call_args[0][0]
    assert request.object_id == "cube"


def test_get_static_object_coords_unavailable(node):
    """Test coordinate service unavailable."""

    node.coord_client.wait_for_service = MagicMock(
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

    assert request.x == 1.0
    assert request.y == 2.0
    assert request.z == 3.0


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
    node.call_move_service.assert_called_once_with(1.0, 2.0, 3.0)
    node.call_relative_move_service.assert_called_once_with(
        0.1, 0.2, 0.3
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

