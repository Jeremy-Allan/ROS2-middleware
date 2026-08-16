import json

import pytest

from unittest.mock import MagicMock, patch

from kinova_interface.environment_mapping_node import EnvironmentMappingNode

from kinova_interfaces.srv import (
    GetObjectCoordinates,
    GetRelativeMovement,
    GetRobotParameters,
)

from kinova_interfaces.msg import ExtendedStatus

from moveit_msgs.msg import CollisionObject

"""
Test with: 
pytest src/ROS2-middleware/kinova_interface/test/test_environment_mapping_node.py -v
"""

# Fixtures
@pytest.fixture
def node(ros_context, tmp_path):

    config_parameter = MagicMock()
    config_parameter.value = str(tmp_path)

    with patch.object(
        EnvironmentMappingNode,
        "get_parameter",
        return_value=config_parameter
    ), \
    patch.object(
        EnvironmentMappingNode,
        "load_coordinate_dictionary",
        return_value={
            "cube": {
                "x": 1.0,
                "y": 2.0,
                "z": 3.0
            }
        }
    ), \
    patch.object(
        EnvironmentMappingNode,
        "load_relative_movements",
        return_value={
            "forward": {
                "x": 0.1,
                "y": 0.0,
                "z": 0.0
            }
        }
    ), \
    patch.object(
        EnvironmentMappingNode,
        "load_obstacles",
        return_value=[]
    ), \
    patch.object(
        EnvironmentMappingNode,
        "publish_planning_scene"
    ):

        node = EnvironmentMappingNode()

    yield node

    node.destroy_node()


# Node Initialisation

def test_node_initialises(node):

    assert node.status_pub is not None
    assert node.srv_coords is not None
    assert node.srv_move is not None
    assert node.srv_list is not None
    assert node.scene_client is not None

    assert node.static_objects is not None
    assert node.relative_movements is not None
    assert node.obstacles is not None

    assert node.current_state == ExtendedStatus.STATE_IDLE
    assert node.command_success is True


# publish_status()
def test_publish_status(node):

    node.status_pub.publish = MagicMock()

    node.current_state = ExtendedStatus.STATE_IDLE
    node.status_text = "Testing"
    node.command_success = True

    node.publish_status()

    node.status_pub.publish.assert_called_once()

    msg = node.status_pub.publish.call_args[0][0]

    assert msg.node_name == node.get_name()
    assert msg.state == ExtendedStatus.STATE_IDLE
    assert msg.status_message == "Testing"
    assert msg.last_command_valid is True


# load_coordinate_dictionary()
def test_load_coordinate_dictionary(node, tmp_path):

    data = {
        "cube": {
            "x": 1.0,
            "y": 2.0,
            "z": 3.0
        }
    }

    file_path = tmp_path / "coordinate_dictionary.json"
    file_path.write_text(json.dumps(data))

    node.config_dir = str(tmp_path)

    result = node.load_coordinate_dictionary()

    assert result == data


def test_load_coordinate_dictionary_invalid_json(node, tmp_path):

    file_path = tmp_path / "coordinate_dictionary.json"
    file_path.write_text("invalid json")

    node.config_dir = str(tmp_path)

    with pytest.raises(SystemExit):
        node.load_coordinate_dictionary()


# load_relative_movements()
def test_load_relative_movements(node, tmp_path):

    data = {
        "forward": {
            "x": 0.1,
            "y": 0.0,
            "z": 0.0
        }
    }

    file_path = tmp_path / "relative_movement.json"
    file_path.write_text(json.dumps(data))

    node.config_dir = str(tmp_path)

    result = node.load_relative_movements()

    assert result == data


def test_load_relative_movements_invalid_json(node, tmp_path):

    file_path = tmp_path / "relative_movement.json"
    file_path.write_text("invalid json")

    node.config_dir = str(tmp_path)

    with pytest.raises(SystemExit):
        node.load_relative_movements()


# load_obstacles()
def test_load_obstacles(node, tmp_path):

    obstacles = [
        {
            "id": "table",
            "shape": 1,
            "dimensions": [1.0, 2.0, 0.5],
            "position": {
                "x": 1.0,
                "y": 2.0,
                "z": 0.25
            },
            "description": "Test table"
        }
    ]

    with patch(
        "kinova_interface.environment_mapping_node.get_package_share_directory",
        return_value=str(tmp_path)
    ):

        data_dir = tmp_path / "data" / "configs" / "env"
        data_dir.mkdir(parents=True)

        file_path = data_dir / "obstacles.json"
        file_path.write_text(json.dumps(obstacles))

        result = node.load_obstacles()

    assert result == obstacles


def test_load_obstacles_invalid_json(node, tmp_path):

    with patch(
        "kinova_interface.environment_mapping_node.get_package_share_directory",
        return_value=str(tmp_path)
    ):

        data_dir = tmp_path / "data" / "configs" / "env"
        data_dir.mkdir(parents=True)

        file_path = data_dir / "obstacles.json"
        file_path.write_text("invalid json")

        with pytest.raises(SystemExit):
            node.load_obstacles()

# get_coordinates_callback()
def test_get_coordinates_callback_found(node):

    request = GetObjectCoordinates.Request()
    request.object_id = "cube"

    response = GetObjectCoordinates.Response()

    node.publish_status = MagicMock()

    result = node.get_coordinates_callback(request, response)

    assert result.success is True
    assert result.message == "Object Found"

    assert result.x == 1.0
    assert result.y == 2.0
    assert result.z == 3.0

    assert node.command_success is True

    node.publish_status.assert_called_once()


def test_get_coordinates_callback_not_found(node):

    request = GetObjectCoordinates.Request()
    request.object_id = "unknown"

    response = GetObjectCoordinates.Response()

    node.publish_status = MagicMock()

    result = node.get_coordinates_callback(request, response)

    assert result.success is False
    assert result.message == "Object unknown NOT Found"

    assert node.command_success is False

    node.publish_status.assert_called_once()


# get_relative_movement_callback()
def test_get_relative_movement_callback_found(node):

    request = GetRelativeMovement.Request()
    request.move_id = "forward"

    response = GetRelativeMovement.Response()

    node.publish_status = MagicMock()

    result = node.get_relative_movement_callback(request, response)

    assert result.success is True
    assert result.message == "Movement Found"

    assert result.x == 0.1
    assert result.y == 0.0
    assert result.z == 0.0

    assert node.command_success is True

    node.publish_status.assert_called_once()


def test_get_relative_movement_callback_not_found(node):

    request = GetRelativeMovement.Request()
    request.move_id = "unknown"

    response = GetRelativeMovement.Response()

    node.publish_status = MagicMock()

    result = node.get_relative_movement_callback(request, response)

    assert result.success is False
    assert result.message == "Movement unknown NOT Found"

    assert node.command_success is False

    node.publish_status.assert_called_once()


# get_robot_parameters_callback()
def test_get_robot_parameters_callback(node):

    request = GetRobotParameters.Request()
    response = GetRobotParameters.Response()

    node.publish_status = MagicMock()

    result = node.get_robot_parameters_callback(request, response)

    assert result.object_list == ["cube"]
    assert result.movement_names == ["forward"]

    assert node.command_success is True
    assert node.status_text == "Robot Parameters queried"

    node.publish_status.assert_called_once()


# build_collision_object()
def test_build_collision_object(node):

    obstacle = {
        "id": "table",
        "shape": 1,
        "dimensions": [1.0, 2.0, 0.5],
        "position": {
            "x": 1.0,
            "y": 2.0,
            "z": 0.25
        },
        "description": "Test table"
    }

    result = node.build_collision_object(obstacle)

    assert isinstance(result, CollisionObject)

    assert result.id == "table"
    assert result.header.frame_id == "base_link"

    assert len(result.primitives) == 1
    assert len(result.primitive_poses) == 1

    assert result.primitives[0].type == 1
    assert list(result.primitives[0].dimensions) == [
        1.0,
        2.0,
        0.5
    ]

    pose = result.primitive_poses[0]

    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert pose.position.z == 0.25
    assert pose.orientation.w == 1.0

    assert result.operation == CollisionObject.ADD


# publish_planning_scene()
def test_publish_planning_scene_service_unavailable(node):

    node.scene_client.wait_for_service = MagicMock(
        return_value=False
    )

    node.publish_planning_scene()

    node.scene_client.wait_for_service.assert_called_once_with(
        timeout_sec=15.0
    )


def test_publish_planning_scene_success(node):

    obstacle = {
        "id": "table",
        "shape": 1,
        "dimensions": [1.0, 2.0, 0.5],
        "position": {
            "x": 1.0,
            "y": 2.0,
            "z": 0.25
        },
        "description": "Test table"
    }

    node.obstacles = [obstacle]

    node.scene_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = MagicMock()
    response.success = True

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    node.scene_client.call_async = MagicMock(
        return_value=future
    )

    with patch(
        "kinova_interface.environment_mapping_node.time.sleep"
    ):

        node.publish_planning_scene()

    node.scene_client.call_async.assert_called_once()

    request = node.scene_client.call_async.call_args[0][0]

    assert request.scene.is_diff is True

    assert len(
        request.scene.world.collision_objects
    ) == 1

    collision_object = (
        request.scene.world.collision_objects[0]
    )

    assert collision_object.id == "table"