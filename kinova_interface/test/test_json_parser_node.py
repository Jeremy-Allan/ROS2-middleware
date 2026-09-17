import json
import pytest

from unittest.mock import MagicMock, patch

from kinova_interface.json_parser_node import JsonParser, JsonParserNode

from kinova_interfaces.msg import ExtendedStatus


"""
Test with:
pytest src/ROS2-middleware/kinova_interface/test/test_json_parser_node.py -v

Note: these tests cover JsonParserNode's own responsibility - loading a
recipe and running its steps through whatever handler is registered for
each step's action. What each action actually does (which hardware/
environment services it calls) is ArmActions' responsibility, covered by
test_arm_actions.py; that's why steps here are exercised via
node.arm_actions.handlers rather than mocking individual service calls.
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
    """Test that the node, its arm actions, and its own services are created."""

    assert node.parser is not None
    assert node.status_pub is not None
    assert node.execute_srv is not None

    assert node.arm_actions is not None
    assert set(node.arm_actions.handlers.keys()) == {
        'home', 'move_arm', 'relative_move', 'gripper', 'pickup', 'dropoff',
        'pour', 'thrust', 'push', 'throw'
    }


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


# execute_recipe() / _dispatch_step()
def test_execute_recipe(node):
    """Test executing a recipe runs each step's action through the registered
    handler, in order, with that step's parameters."""

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
                "action": "gripper",
                "parameters": {
                    "position": 0.5
                }
            }
        ]
    }

    node.arm_actions.handlers = {
        'home': MagicMock(return_value=True),
        'move_arm': MagicMock(return_value=True),
        'gripper': MagicMock(return_value=True),
    }

    node.publish_status = MagicMock()

    with patch("kinova_interface.json_parser_node.time.sleep"):
        result = node.execute_recipe()

    assert result is True

    node.arm_actions.handlers['home'].assert_called_once_with({})
    node.arm_actions.handlers['move_arm'].assert_called_once_with({"target": "cube"})
    node.arm_actions.handlers['gripper'].assert_called_once_with({"position": 0.5})


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


# _dispatch_step() / execute_recipe() all_success regression
def test_execute_recipe_step_failure_sets_all_success_false(node):
    """Regression test: a step failure must fail the whole recipe, not
    just break the loop silently (the bug 4.2 in the implementation plan fixes)."""

    node.parser.recipe = {
        "recipe_name": "Test",
        "steps": [
            {"action": "pickup", "parameters": {"target": "red_cube"}}
        ]
    }

    node.arm_actions.handlers = {
        'pickup': MagicMock(return_value=False),
    }
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


def test_execute_recipe_stops_at_first_failure(node):
    """A failing step must stop the recipe - later steps should not run."""

    node.parser.recipe = {
        "recipe_name": "Test",
        "steps": [
            {"action": "home"},
            {"action": "move_arm", "parameters": {"target": "cube"}},
        ]
    }

    node.arm_actions.handlers = {
        'home': MagicMock(return_value=False),
        'move_arm': MagicMock(return_value=True),
    }
    node.publish_status = MagicMock()

    with patch("kinova_interface.json_parser_node.time.sleep"):
        result = node.execute_recipe()

    assert result is False
    node.arm_actions.handlers['move_arm'].assert_not_called()
