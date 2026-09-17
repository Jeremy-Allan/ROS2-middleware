import pytest

from unittest.mock import MagicMock, patch
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from kinova_interface.arm_actions import ArmActions

from kinova_interfaces.srv import (
    GetObjectInfo,
    GetRelativeMovement,
    HomeArm,
    MoveArm,
    RelativeMove,
    MoveGripper,
)
from kinova_interfaces.msg import MotionParams
from shape_msgs.msg import SolidPrimitive


"""
Test with:
pytest src/ROS2-middleware/kinova_interface/test/test_arm_actions.py -v
"""

# Fixtures

@pytest.fixture
def actions(ros_context):
    """Create ArmActions against a minimal real rclpy node, so its service
    clients are real (but disconnected). Kept separate from JsonParserNode
    so these tests only exercise ArmActions itself."""
    host_node = Node('test_arm_actions_host')
    host_node.cb_group = ReentrantCallbackGroup()

    arm_actions = ArmActions(host_node)

    yield arm_actions
    host_node.destroy_node()


# Construction

def test_actions_creates_clients_and_handlers(actions):
    """Test that ArmActions creates its service clients and the action dictionary."""

    assert actions.home_client is not None
    assert actions.move_arm_client is not None
    assert actions.move_gripper_client is not None
    assert actions.relative_move_client is not None
    assert actions.coord_client is not None
    assert actions.relative_client is not None
    assert actions.info_client is not None
    assert actions.orientation_client is not None
    assert actions.attach_client is not None
    assert actions.detach_client is not None
    assert actions.update_pose_client is not None

    assert set(actions.handlers.keys()) == {
        'home', 'move_arm', 'relative_move', 'gripper', 'pickup', 'dropoff',
        'pour', 'thrust', 'push', 'throw'
    }
    assert actions.held_object is None


# wait_for_future()
def test_wait_for_future(actions):
    """Test that wait_for_future returns the service response."""

    future = MagicMock()
    response = MagicMock()

    future.done.return_value = True
    future.result.return_value = response

    result = actions.wait_for_future(future, "/test")

    assert result == response


# get_static_object_coords()
# Note: get_static_object_coords() actually calls get_object_info() (the
# /get_object_info service via info_client), not /get_coordinates via
# coord_client, so these tests mock info_client to match real behaviour.
def test_get_static_object_coords(actions):
    """Test getting object coordinates."""

    actions.info_client.wait_for_service = MagicMock(
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

    actions.info_client.call_async = MagicMock(
        return_value=future
    )

    result = actions.get_static_object_coords("cube")

    assert result == {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0
    }

    request = actions.info_client.call_async.call_args[0][0]
    assert request.object_id == "cube"


def test_get_static_object_coords_unavailable(actions):
    """Test coordinate service unavailable."""

    actions.info_client.wait_for_service = MagicMock(
        return_value=False
    )

    assert actions.get_static_object_coords("cube") is None


# get_relative_movement_vector()
def test_get_relative_movement_vector(actions):
    """Test getting a relative movement vector."""

    actions.relative_client.wait_for_service = MagicMock(
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

    actions.relative_client.call_async = MagicMock(
        return_value=future
    )

    result = actions.get_relative_movement_vector("forward")

    assert result == {
        "x": 0.1,
        "y": 0.2,
        "z": 0.3
    }


# call_home_service()
def test_call_home_service(actions):
    """Test the home service."""

    actions.home_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = HomeArm.Response()
    response.success = True
    response.message = "Homed"

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    actions.home_client.call_async = MagicMock(
        return_value=future
    )

    result = actions.call_home_service()

    assert result["success"] is True
    assert result["message"] == "Homed"


# call_move_service()
def test_call_move_service(actions):
    """Test the move arm service."""

    actions.move_arm_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = MoveArm.Response()
    response.success = True
    response.message = "Moved"

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    actions.move_arm_client.call_async = MagicMock(
        return_value=future
    )

    result = actions.call_move_service(1.0, 2.0, 3.0)

    assert result["success"] is True

    request = actions.move_arm_client.call_async.call_args[0][0]

    assert request.target_position.x == 1.0
    assert request.target_position.y == 2.0
    assert request.target_position.z == 3.0
    assert request.has_orientation is False


def test_call_move_service_with_orientation_and_speed(actions):
    """Test the move arm service passes orientation and speed through."""

    actions.move_arm_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = MoveArm.Response()
    response.success = True
    response.message = "Moved"

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    actions.move_arm_client.call_async = MagicMock(
        return_value=future
    )

    motion_params = MotionParams()
    motion_params.velocity_scale = 0.5
    motion_params.acceleration_scale = 0.5

    result = actions.call_move_service(
        1.0, 2.0, 3.0,
        has_orientation=True, roll=0.0, pitch=1.57, yaw=0.0,
        motion_params=motion_params
    )

    assert result["success"] is True

    request = actions.move_arm_client.call_async.call_args[0][0]

    assert request.has_orientation is True
    assert request.pitch == 1.57
    assert request.motion_params.velocity_scale == 0.5


#call_relative_move_service()
def test_call_relative_move_service(actions):
    """Test relative movement service."""

    actions.relative_move_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = RelativeMove.Response()
    response.success = True
    response.message = "Moved"

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    actions.relative_move_client.call_async = MagicMock(
        return_value=future
    )

    result = actions.call_relative_move_service(
        0.1,
        0.2,
        0.3
    )

    assert result["success"] is True

    request = actions.relative_move_client.call_async.call_args[0][0]

    assert request.vx == 0.1
    assert request.vy == 0.2
    assert request.vz == 0.3
    assert request.has_orientation is False


#call_move_gripper_service()
def test_call_move_gripper_service(actions):
    """Test gripper service."""

    actions.move_gripper_client.wait_for_service = MagicMock(
        return_value=True
    )

    response = MoveGripper.Response()
    response.success = True
    response.message = "Gripper moved"

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response

    actions.move_gripper_client.call_async = MagicMock(
        return_value=future
    )

    result = actions.call_move_gripper_service(0.5)

    assert result["success"] is True

    request = actions.move_gripper_client.call_async.call_args[0][0]

    assert request.position == 0.5


# resolve_orientation()
def test_resolve_orientation_none_requested(actions):
    """No preset name means no orientation, no service call needed."""

    actions.get_orientation_preset = MagicMock()

    result = actions.resolve_orientation(None)

    assert result == (False, 0.0, 0.0, 0.0)
    actions.get_orientation_preset.assert_not_called()


def test_resolve_orientation_valid_preset(actions):
    """A valid preset name resolves to its angles with has_orientation True."""

    actions.get_orientation_preset = MagicMock(
        return_value={"roll": 0.0, "pitch": 1.57, "yaw": 0.0}
    )

    result = actions.resolve_orientation("tilted_for_pour")

    assert result == (True, 0.0, 1.57, 0.0)


def test_resolve_orientation_unknown_preset(actions):
    """An unresolvable preset name returns None, distinct from 'not requested'."""

    actions.get_orientation_preset = MagicMock(return_value=None)

    result = actions.resolve_orientation("not_a_real_preset")

    assert result is None


# build_motion_params()
def test_build_motion_params_none(actions):
    """No speed given means both scales stay at the 0.0 'use arm default'."""

    params = actions.build_motion_params(None)

    assert params.velocity_scale == 0.0
    assert params.acceleration_scale == 0.0


def test_build_motion_params_with_speed(actions):
    """A speed value sets both velocity and acceleration scale."""

    params = actions.build_motion_params(0.4)

    assert params.velocity_scale == 0.4
    assert params.acceleration_scale == 0.4


# object_half_height()
def test_object_half_height_box(actions):
    shape = {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.08]}
    assert actions.object_half_height(shape) == pytest.approx(0.04)


def test_object_half_height_cylinder(actions):
    shape = {"type": SolidPrimitive.CYLINDER, "dimensions": [0.1, 0.02]}
    assert actions.object_half_height(shape) == pytest.approx(0.05)


def test_object_half_height_sphere(actions):
    shape = {"type": SolidPrimitive.SPHERE, "dimensions": [0.03]}
    assert actions.object_half_height(shape) == pytest.approx(0.03)


# _handle_dropoff() two-stage descent and stacking height
def test_handle_dropoff_two_stage_descent_with_stacking(actions):
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

    actions.get_object_info = MagicMock(side_effect=object_info_side_effect)
    actions.call_move_service = MagicMock(return_value={"success": True})
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.update_object_pose = MagicMock(return_value=True)
    actions.detach_object = MagicMock(return_value=True)

    result = actions._handle_dropoff({
        "target": "red_cube",
        "destination": "delivery_tray",
        "place_offset": 0.1
    })

    assert result is True
    assert actions.call_move_service.call_count == 2

    # dest top = 0.0 + 0.01 (half of 0.02 tray height) = 0.01
    # target half height = 0.025 (half of 0.05 cube)
    # release_z = 0.01 + 0.025 = 0.035
    hover_call = actions.call_move_service.call_args_list[0][0]
    release_call = actions.call_move_service.call_args_list[1][0]

    assert hover_call[2] == pytest.approx(0.035 + 0.1)
    assert release_call[2] == pytest.approx(0.035 + 0.02)
    # the release move must be lower than the hover move, not the same height
    assert release_call[2] < hover_call[2]


# _handle_pickup() / _handle_move_arm() wiring through the handlers dict
def test_handle_pickup_failure_when_coords_missing(actions):
    """pickup should fail cleanly (not raise) when the target can't be resolved."""

    actions.get_static_object_coords = MagicMock(return_value=None)

    result = actions.handlers['pickup']({"target": "unknown_object"})

    assert result is False


def test_handle_move_arm_unknown_orientation_fails(actions):
    """move_arm should fail cleanly when an orientation preset can't be resolved."""

    actions.get_static_object_coords = MagicMock(return_value={"x": 1.0, "y": 2.0, "z": 3.0})
    actions.get_orientation_preset = MagicMock(return_value=None)

    result = actions.handlers['move_arm']({"target": "cube", "orientation": "not_a_real_preset"})

    assert result is False


# held_object tracking: pickup sets it, dropoff clears it / falls back to it
def test_pickup_sets_held_object(actions):
    """A successful pickup should record what's now held."""

    actions.get_static_object_coords = MagicMock(return_value={"x": 1.0, "y": 2.0, "z": 3.0})
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.call_move_service = MagicMock(return_value={"success": True})
    actions.attach_object = MagicMock(return_value=True)

    result = actions.handlers['pickup']({"target": "red_cube"})

    assert result is True
    assert actions.held_object == "red_cube"


def test_dropoff_clears_held_object(actions):
    """A successful dropoff of the held object should clear held_object."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 0.5, "y": 0.1, "z": 0.0}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.3, 0.2, 0.02]}
    })
    actions.call_move_service = MagicMock(return_value={"success": True})
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.update_object_pose = MagicMock(return_value=True)
    actions.detach_object = MagicMock(return_value=True)

    result = actions.handlers['dropoff']({"target": "red_cube", "destination": "delivery_tray"})

    assert result is True
    assert actions.held_object is None


def test_dropoff_falls_back_to_held_object_when_target_omitted(actions):
    """Regression: a dropoff step missing 'target' must still use the actually
    held object's height for the release calculation, not silently default to 0."""

    actions.held_object = "red_cube"

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

    actions.get_object_info = MagicMock(side_effect=object_info_side_effect)
    actions.call_move_service = MagicMock(return_value={"success": True})
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.update_object_pose = MagicMock(return_value=True)
    actions.detach_object = MagicMock(return_value=True)

    result = actions.handlers['dropoff']({"destination": "delivery_tray"})

    assert result is True
    # Same release_z as test_handle_dropoff_two_stage_descent_with_stacking:
    # dest top 0.01 + held cube's half-height 0.025 = 0.035, not 0.01 (which
    # is what a silent target_half_height=0.0 fallback would have produced).
    release_call = actions.call_move_service.call_args_list[1][0]
    assert release_call[2] == pytest.approx(0.035 + 0.02)
    assert actions.held_object is None


# pour()
def test_pour_requires_target(actions):
    """pour without a target should fail cleanly, not assume anything is held."""

    result = actions.handlers['pour']({})

    assert result is False


def test_pour_fails_if_target_not_held(actions):
    """pour should refuse to run if the named target isn't what's actually held."""

    actions.held_object = "blue_cube"

    result = actions.handlers['pour']({"target": "red_cube"})

    assert result is False


def test_pour_tilts_holds_and_returns_upright(actions):
    """pour should tilt in place, dwell, then rotate back to upright."""

    actions.held_object = "red_cube"
    actions.get_orientation_preset = MagicMock(
        return_value={"roll": 0.0, "pitch": 1.57, "yaw": 0.0}
    )
    actions.call_relative_move_service = MagicMock(return_value={"success": True})

    with patch("kinova_interface.arm_actions.time.sleep") as mock_sleep:
        result = actions.handlers['pour']({"target": "red_cube", "duration": 2.0})

    assert result is True
    assert actions.call_relative_move_service.call_count == 2
    mock_sleep.assert_called_once_with(2.0)

    tilt_call = actions.call_relative_move_service.call_args_list[0][0]
    return_call = actions.call_relative_move_service.call_args_list[1][0]

    # tilt_call args: (vx, vy, vz, has_orientation, roll, pitch, yaw, motion_params)
    assert tilt_call[0:3] == (0.0, 0.0, 0.0)
    assert tilt_call[5] == pytest.approx(1.57)
    # returning upright applies the negated delta
    assert return_call[5] == pytest.approx(-1.57)


def test_pour_fails_on_unknown_orientation(actions):
    """pour should fail cleanly if its orientation preset can't be resolved."""

    actions.held_object = "red_cube"
    actions.get_orientation_preset = MagicMock(return_value=None)

    result = actions.handlers['pour']({"target": "red_cube"})

    assert result is False


# thrust()
def test_thrust_requires_target(actions):
    """thrust without a target should fail cleanly, not assume anything is held."""

    result = actions.handlers['thrust']({})

    assert result is False


def test_thrust_fails_if_target_not_held(actions):
    """thrust should refuse to run if the named target isn't what's actually held."""

    actions.held_object = "blue_cube"

    result = actions.handlers['thrust']({"target": "red_cube"})

    assert result is False


def test_thrust_levels_then_thrusts_forward(actions):
    """thrust should level to horizontal, then move by the thrust_forward vector."""

    actions.held_object = "red_cube"
    actions.get_orientation_preset = MagicMock(return_value={"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
    actions.call_relative_move_service = MagicMock(return_value={"success": True})
    actions.get_relative_movement_vector = MagicMock(
        return_value={"x": 0.1, "y": 0.0, "z": 0.0}
    )

    result = actions.handlers['thrust']({"target": "red_cube"})

    assert result is True
    assert actions.call_relative_move_service.call_count == 2

    level_call = actions.call_relative_move_service.call_args_list[0][0]
    thrust_call = actions.call_relative_move_service.call_args_list[1][0]

    # facing_forward is (0, 0, 0) - leveling is a zero-delta orientation set
    assert level_call[3] is True
    assert level_call[4:7] == (0.0, 0.0, 0.0)
    # the actual forward displacement
    assert thrust_call[0:3] == (0.1, 0.0, 0.0)

    actions.get_relative_movement_vector.assert_called_once_with("thrust_forward")


def test_thrust_fails_on_unknown_vector(actions):
    """thrust should fail cleanly if its movement vector can't be resolved."""

    actions.held_object = "red_cube"
    actions.get_orientation_preset = MagicMock(return_value={"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
    actions.call_relative_move_service = MagicMock(return_value={"success": True})
    actions.get_relative_movement_vector = MagicMock(return_value=None)

    result = actions.handlers['thrust']({"target": "red_cube"})

    assert result is False


# pour() with a direct 'amount' override
def test_pour_amount_overrides_preset(actions):
    """An explicit 'amount' should be used as the pitch tilt directly,
    without resolving any orientation preset."""

    actions.held_object = "red_cube"
    actions.get_orientation_preset = MagicMock()
    actions.call_relative_move_service = MagicMock(return_value={"success": True})

    with patch("kinova_interface.arm_actions.time.sleep"):
        result = actions.handlers['pour']({"target": "red_cube", "amount": 0.5})

    assert result is True
    actions.get_orientation_preset.assert_not_called()

    tilt_call = actions.call_relative_move_service.call_args_list[0][0]
    return_call = actions.call_relative_move_service.call_args_list[1][0]

    assert tilt_call[5] == pytest.approx(0.5)
    assert return_call[5] == pytest.approx(-0.5)


# push()
def test_push_requires_target_and_destination(actions):
    """push without both a target and a destination should fail cleanly."""

    assert actions.handlers['push']({"target": "red_cube"}) is False
    assert actions.handlers['push']({"destination": "delivery_tray"}) is False


def test_push_slides_object_to_destination_at_same_height(actions):
    """push should level the gripper flat, approach the object at its
    resting height, then slide across to the destination at that same
    height and orientation without ever lifting it."""

    def object_info_side_effect(name):
        if name == "red_cube":
            return {
                "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.01}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
                "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
            }
        if name == "delivery_tray":
            return {
                "pose": {"position": {"x": 0.5, "y": 0.1, "z": 0.0}, "orientation": {}},
                "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.3, 0.2, 0.02]}
            }
        return None

    actions.get_object_info = MagicMock(side_effect=object_info_side_effect)
    actions.get_orientation_preset = MagicMock(return_value={"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.call_move_service = MagicMock(return_value={"success": True})
    actions.update_object_pose = MagicMock(return_value=True)

    result = actions.handlers['push']({"target": "red_cube", "destination": "delivery_tray"})

    assert result is True
    assert actions.call_move_service.call_count == 2

    approach_call = actions.call_move_service.call_args_list[0][0]
    push_call = actions.call_move_service.call_args_list[1][0]

    # both moves stay at the object's own resting height - never lifted -
    # and gripper leveled flat (has_orientation True, all angles 0)
    assert approach_call[0:3] == (0.0, 0.0, 0.01)
    assert approach_call[3] is True
    assert approach_call[4:7] == (0.0, 0.0, 0.0)
    assert push_call[0:3] == (0.5, 0.1, 0.01)
    assert push_call[3] is True

    actions.update_object_pose.assert_called_once_with(
        "red_cube", 0.5, 0.1, 0.01, {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    )


def test_push_fails_if_target_unresolved(actions):
    """push should fail cleanly if the target object can't be resolved."""

    actions.get_object_info = MagicMock(return_value=None)

    result = actions.handlers['push']({"target": "unknown", "destination": "delivery_tray"})

    assert result is False


# throw()
def test_throw_requires_target(actions):
    """throw without a target should fail cleanly, not assume anything is held."""

    result = actions.handlers['throw']({"destination": "delivery_tray"})

    assert result is False


def test_throw_fails_if_target_not_held(actions):
    """throw should refuse to run if the named target isn't what's actually held."""

    actions.held_object = "blue_cube"

    result = actions.handlers['throw']({"target": "red_cube", "destination": "delivery_tray"})

    assert result is False


def test_throw_requires_destination(actions):
    """throw without a destination should fail cleanly."""

    actions.held_object = "red_cube"

    result = actions.handlers['throw']({"target": "red_cube"})

    assert result is False


def test_throw_winds_up_then_pitches_then_releases(actions):
    """throw should wind up (retreat back and up) then pitch fast to the
    release point and release immediately - not dropoff's careful
    hover-then-lower-then-release staging."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 0.5, "y": 0.1, "z": 0.0}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.3, 0.2, 0.02]}
    })
    actions.call_move_service = MagicMock(return_value={"success": True})
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.detach_object = MagicMock(return_value=True)
    actions.update_object_pose = MagicMock(return_value=True)

    result = actions.handlers['throw']({"target": "red_cube", "destination": "delivery_tray"})

    assert result is True
    assert actions.call_move_service.call_count == 2
    actions.detach_object.assert_called_once_with("red_cube")
    assert actions.held_object is None


# resolve_direction_offset()
def test_resolve_direction_offset_forward_continues_same_bearing(actions):
    """'forward' should extend further out along the same bearing from the
    arm's base (origin) the reference point already had."""

    x, y = actions.resolve_direction_offset(1.0, 0.0, 'forward', 0.2)

    assert x == pytest.approx(1.2)
    assert y == pytest.approx(0.0)


def test_resolve_direction_offset_backward_reverses_bearing(actions):
    x, y = actions.resolve_direction_offset(1.0, 0.0, 'backward', 0.2)

    assert x == pytest.approx(0.8)
    assert y == pytest.approx(0.0)


def test_resolve_direction_offset_left_and_right_are_perpendicular(actions):
    """'left'/'right' should be the bearing rotated +/-90 degrees, not
    along the original bearing at all."""

    left_x, left_y = actions.resolve_direction_offset(1.0, 0.0, 'left', 0.2)
    right_x, right_y = actions.resolve_direction_offset(1.0, 0.0, 'right', 0.2)

    assert left_x == pytest.approx(1.0)
    assert left_y == pytest.approx(0.2)
    assert right_x == pytest.approx(1.0)
    assert right_y == pytest.approx(-0.2)


def test_resolve_direction_offset_unknown_direction_returns_none(actions):
    assert actions.resolve_direction_offset(1.0, 0.0, 'sideways', 0.2) is None


def test_resolve_direction_offset_falls_back_when_at_origin(actions):
    """A reference point essentially at the arm's base has no bearing to
    rotate - fall back to a fixed +X bearing rather than dividing by zero."""

    x, y = actions.resolve_direction_offset(0.0, 0.0, 'forward', 0.2)

    assert x == pytest.approx(0.2)
    assert y == pytest.approx(0.0)


# push() with 'direction' instead of 'destination'
def test_push_with_direction(actions):
    """push should accept 'direction'+'distance' as an alternative to
    'destination', computed from the target's own original bearing."""

    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 1.0, "y": 0.0, "z": 0.01}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    })
    actions.get_orientation_preset = MagicMock(return_value={"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.call_move_service = MagicMock(return_value={"success": True})
    actions.update_object_pose = MagicMock(return_value=True)

    result = actions.handlers['push']({"target": "red_cube", "direction": "forward", "distance": 0.3})

    assert result is True
    push_call = actions.call_move_service.call_args_list[1][0]
    assert push_call[0:3] == (1.3, 0.0, 0.01)


def test_push_fails_on_unknown_direction(actions):
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 1.0, "y": 0.0, "z": 0.01}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    })
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})

    result = actions.handlers['push']({"target": "red_cube", "direction": "sideways"})

    assert result is False


# throw() with 'direction' instead of 'destination'
def test_throw_with_direction(actions):
    """throw should accept 'direction'+'distance' as an alternative to
    'destination', computed from the held object's own original bearing."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 1.0, "y": 0.0, "z": 0.05}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    })
    actions.call_move_service = MagicMock(return_value={"success": True})
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.detach_object = MagicMock(return_value=True)
    actions.update_object_pose = MagicMock(return_value=True)

    result = actions.handlers['throw']({"target": "red_cube", "direction": "left", "distance": 0.3})

    assert result is True
    assert actions.call_move_service.call_count == 2

    windup_call = actions.call_move_service.call_args_list[0][0]
    release_call = actions.call_move_service.call_args_list[1][0]

    # throw direction is (0, 1) (origin (1,0) -> release (1, 0.3)); windup
    # retreats the opposite way (0, -1) by wind_up_distance (default 0.15),
    # and rises by wind_up_height (default 0.1) above the origin's z (0.05)
    assert windup_call[0] == pytest.approx(1.0)
    assert windup_call[1] == pytest.approx(-0.15)
    assert windup_call[2] == pytest.approx(0.15)

    # left = bearing (1,0) rotated +90 degrees -> (0,1), scaled by distance
    assert release_call[0] == pytest.approx(1.0)
    assert release_call[1] == pytest.approx(0.3)
    # height is the object's own original height (0.05) + release_clearance (default 0.15)
    assert release_call[2] == pytest.approx(0.2)


def test_throw_fails_on_unknown_direction(actions):
    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 1.0, "y": 0.0, "z": 0.05}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    })

    result = actions.handlers['throw']({"target": "red_cube", "direction": "sideways"})

    assert result is False
