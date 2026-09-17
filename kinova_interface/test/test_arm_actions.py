import math

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
    assert actions.reset_scene_client is not None
    assert actions.compute_ik_client is not None
    assert actions.apply_planning_scene_client is not None
    assert actions.compute_fk_client is not None
    assert actions.check_state_validity_client is not None
    assert actions.get_planning_scene_client is not None
    assert actions.joint_state_sub is not None
    assert actions.latest_joint_positions is None

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
    # unconstrained by default, same as before 'orientation' support was added
    move_call = actions.call_move_service.call_args[0]
    assert move_call == (1.0, 2.0, 3.0, False, 0.0, 0.0, 0.0)


def test_pickup_applies_orientation_when_explicitly_given(actions):
    """A named orientation preset should be forced on the descend move,
    for actions (like 'pour') that need a known, repeatable grasp."""

    actions.get_static_object_coords = MagicMock(return_value={"x": 1.0, "y": 2.0, "z": 3.0})
    actions.get_orientation_preset = MagicMock(
        return_value={"roll": 1.66, "pitch": -0.04, "yaw": -1.57}
    )
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.call_move_service = MagicMock(return_value={"success": True})
    actions.attach_object = MagicMock(return_value=True)

    result = actions.handlers['pickup']({"target": "red_cube", "orientation": "some_orientation_preset"})

    assert result is True
    move_call = actions.call_move_service.call_args[0]
    assert move_call == (1.0, 2.0, 3.0, True, 1.66, -0.04, -1.57)
    actions.get_orientation_preset.assert_called_once_with("some_orientation_preset")


def test_pickup_applies_grasp_offset_from_object_center(actions):
    """'grasp_offset' should shift the approach target away from the
    object's registered center - needed alongside a forced 'orientation'
    that was only demonstrated/valid at an offset point, not dead-center."""

    actions.get_static_object_coords = MagicMock(return_value={"x": 1.0, "y": 2.0, "z": 3.0})
    actions.get_orientation_preset = MagicMock(
        return_value={"roll": 1.66, "pitch": -0.04, "yaw": -1.57}
    )
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.call_move_service = MagicMock(return_value={"success": True})
    actions.attach_object = MagicMock(return_value=True)

    result = actions.handlers['pickup']({
        "target": "red_cube", "orientation": "some_orientation_preset",
        "grasp_offset": {"x": -0.04, "y": -0.01, "z": 0.007}
    })

    assert result is True
    move_call = actions.call_move_service.call_args[0]
    assert move_call[0:3] == (pytest.approx(0.96), pytest.approx(1.99), pytest.approx(3.007))


def test_pickup_fails_on_unknown_orientation(actions):
    """pickup should fail cleanly if its orientation preset can't be resolved."""

    actions.get_static_object_coords = MagicMock(return_value={"x": 1.0, "y": 2.0, "z": 3.0})
    actions.get_orientation_preset = MagicMock(return_value=None)

    result = actions.handlers['pickup']({"target": "red_cube", "orientation": "not_a_real_preset"})

    assert result is False


# compute_side_grasp_candidates() / verify_grasp_pose()
def test_compute_side_grasp_candidates_for_box(actions):
    """Candidates should be generated from the object's own registered
    shape/pose (not a fixed preset), centered on it, at each of the 4
    candidate yaw offsets, all with the flat 'side grasp' roll."""

    target_info = {
        "pose": {
            "position": {"x": 0.3, "y": -0.1, "z": 0.02},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}  # yaw 0
        },
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.1, 0.07, 0.04]}
    }

    candidates = actions.compute_side_grasp_candidates(target_info)

    assert len(candidates) >= 4
    # first 4 candidates: object's exact center, each yaw offset
    for cand in candidates[:4]:
        assert cand[0:3] == (0.3, -0.1, 0.02)
        assert cand[3] == pytest.approx(math.pi / 2.0)  # flat roll
        assert cand[4] == 0.0
    yaws = [c[5] for c in candidates[:4]]
    assert yaws == pytest.approx([0.0, math.pi / 2.0, -math.pi / 2.0, math.pi])


def test_compute_side_grasp_candidates_rejects_non_box(actions):
    """Only BOX shapes are supported currently - fail cleanly (empty list),
    not guess, for anything else."""

    target_info = {
        "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "shape": {"type": SolidPrimitive.CYLINDER, "dimensions": [0.03, 0.1]}
    }

    assert actions.compute_side_grasp_candidates(target_info) == []


def _mock_ik_response(error_code, joint_positions=None):
    """Build a MagicMock standing in for a GetPositionIK.Response - a
    plain MagicMock() isn't enough since find_ik_solution actually reads
    (not just checks) response.solution.joint_state."""
    response = MagicMock()
    response.error_code.val = error_code
    response.error_code.SUCCESS = 1
    response.solution.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    response.solution.joint_state.position = joint_positions or [0.0] * 6
    return response


def test_verify_grasp_pose_true_on_ik_success(actions):
    """verify_grasp_pose should call /compute_ik with collision-avoidance
    on, and report True only on a genuine SUCCESS error code."""

    actions.compute_ik_client.wait_for_service = MagicMock(return_value=True)
    response = _mock_ik_response(1)
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response
    actions.compute_ik_client.call_async = MagicMock(return_value=future)

    result = actions.verify_grasp_pose(0.3, -0.1, 0.02, math.pi / 2.0, 0.0, 0.0)

    assert result is True
    request = actions.compute_ik_client.call_async.call_args[0][0]
    assert request.ik_request.avoid_collisions is True
    assert request.ik_request.pose_stamped.pose.position.x == pytest.approx(0.3)


def test_find_ik_solution_returns_joint_positions_in_order(actions):
    """find_ik_solution should return [joint_1..joint_6] in that specific
    order, regardless of what order the response lists them in."""

    actions.compute_ik_client.wait_for_service = MagicMock(return_value=True)
    response = MagicMock()
    response.error_code.val = 1
    response.error_code.SUCCESS = 1
    # deliberately out of order, to prove the lookup is by name not position
    response.solution.joint_state.name = ['joint_3', 'joint_1', 'joint_2', 'joint_6', 'joint_5', 'joint_4']
    response.solution.joint_state.position = [0.3, 0.1, 0.2, 0.6, 0.5, 0.4]
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response
    actions.compute_ik_client.call_async = MagicMock(return_value=future)

    result = actions.find_ik_solution(0.3, -0.1, 0.02, 0.0, 0.0, 0.0)

    assert result == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])


def test_find_ik_solution_passes_seed_when_given(actions):
    """A seed should be forwarded as the request's starting robot_state -
    needed so IK stays in the same configuration branch as the seed
    (see the method's own docstring for why this matters)."""

    actions.compute_ik_client.wait_for_service = MagicMock(return_value=True)
    response = _mock_ik_response(1)
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response
    actions.compute_ik_client.call_async = MagicMock(return_value=future)

    seed = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    actions.find_ik_solution(0.3, -0.1, 0.02, 0.0, 0.0, 0.0, seed_joint_positions=seed)

    request = actions.compute_ik_client.call_async.call_args[0][0]
    assert list(request.ik_request.robot_state.joint_state.position) == pytest.approx(seed)


def test_verify_grasp_pose_false_on_ik_failure(actions):
    """A non-SUCCESS error code (e.g. NO_IK_SOLUTION or in-collision)
    should report as not verified, not raise or assume success."""

    actions.compute_ik_client.wait_for_service = MagicMock(return_value=True)
    response = MagicMock()
    response.error_code.val = -31
    response.error_code.SUCCESS = 1
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response
    actions.compute_ik_client.call_async = MagicMock(return_value=future)

    result = actions.verify_grasp_pose(0.3, -0.1, 0.02, math.pi / 2.0, 0.0, 0.0)

    assert result is False


def _mock_planning_scene_response(entry_names, rows):
    """Build a MagicMock GetPlanningScene response with a real ACM -
    entry_names/entry_values need to actually be readable/iterable
    (unlike a bare MagicMock()), since set_collision_allowed reads and
    extends them, it doesn't just check a field exists."""
    response = MagicMock()
    response.scene.allowed_collision_matrix.entry_names = list(entry_names)
    entries = []
    for row in rows:
        entry = MagicMock()
        entry.enabled = list(row)
        entries.append(entry)
    response.scene.allowed_collision_matrix.entry_values = entries
    return response


def test_set_collision_allowed_adds_new_object_row_and_column(actions):
    """set_collision_allowed should query the current ACM, then add an
    explicit row/column for a not-yet-known object, allowed against every
    existing link - not just a 'default' fallback entry (verified not to
    override MoveIt's own auto-populated explicit disallow entries - see
    docs/push-motion-reference.md)."""

    actions.get_planning_scene_client.wait_for_service = MagicMock(return_value=True)
    gps_response = _mock_planning_scene_response(
        ["link_a", "link_b"],
        [[True, False], [False, True]],
    )
    gps_future = MagicMock()
    gps_future.done.return_value = True
    gps_future.result.return_value = gps_response
    actions.get_planning_scene_client.call_async = MagicMock(return_value=gps_future)

    actions.apply_planning_scene_client.wait_for_service = MagicMock(return_value=True)
    aps_response = MagicMock()
    aps_response.success = True
    aps_future = MagicMock()
    aps_future.done.return_value = True
    aps_future.result.return_value = aps_response
    actions.apply_planning_scene_client.call_async = MagicMock(return_value=aps_future)

    result = actions.set_collision_allowed("push_block", True)

    assert result is True
    request = actions.apply_planning_scene_client.call_async.call_args[0][0]
    assert request.scene.is_diff is True
    acm = request.scene.allowed_collision_matrix
    assert list(acm.entry_names) == ["link_a", "link_b", "push_block"]
    # existing rows grew by one column (True for push_block), unrelated
    # entries between existing links preserved exactly
    assert list(acm.entry_values[0].enabled) == [True, False, True]
    assert list(acm.entry_values[1].enabled) == [False, True, True]
    # push_block's own new row: allowed against everything
    assert list(acm.entry_values[2].enabled) == [True, True, True]


def test_set_collision_allowed_reverts_existing_object_row(actions):
    """If the object already has an ACM entry (e.g. reverting after a
    push), set_collision_allowed should update its existing row/column
    rather than adding a duplicate one."""

    actions.get_planning_scene_client.wait_for_service = MagicMock(return_value=True)
    gps_response = _mock_planning_scene_response(
        ["link_a", "push_block"],
        [[True, True], [True, True]],
    )
    gps_future = MagicMock()
    gps_future.done.return_value = True
    gps_future.result.return_value = gps_response
    actions.get_planning_scene_client.call_async = MagicMock(return_value=gps_future)

    actions.apply_planning_scene_client.wait_for_service = MagicMock(return_value=True)
    aps_response = MagicMock()
    aps_response.success = True
    aps_future = MagicMock()
    aps_future.done.return_value = True
    aps_future.result.return_value = aps_response
    actions.apply_planning_scene_client.call_async = MagicMock(return_value=aps_future)

    result = actions.set_collision_allowed("push_block", False)

    assert result is True
    request = actions.apply_planning_scene_client.call_async.call_args[0][0]
    acm = request.scene.allowed_collision_matrix
    assert list(acm.entry_names) == ["link_a", "push_block"]
    assert list(acm.entry_values[0].enabled) == [True, False]
    assert list(acm.entry_values[1].enabled) == [False, False]


def test_pickup_side_grasp_uses_first_verified_candidate(actions):
    """pickup with grasp_style='side' should try candidates in order and
    use the first one that actually verifies, not just the first one
    generated."""

    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 0.3, "y": -0.1, "z": 0.02}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.1, 0.07, 0.04]}
    })
    # first 3 candidates fail verification, the 4th succeeds
    actions.verify_grasp_pose = MagicMock(side_effect=[False, False, False, True])
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.call_move_service = MagicMock(return_value={"success": True})
    actions.attach_object = MagicMock(return_value=True)

    result = actions.handlers['pickup']({"target": "box", "grasp_style": "side"})

    assert result is True
    assert actions.verify_grasp_pose.call_count == 4
    move_call = actions.call_move_service.call_args[0]
    expected = actions.compute_side_grasp_candidates(actions.get_object_info.return_value)[3]
    assert move_call[0:3] == expected[0:3]
    assert move_call[3] is True


def test_pickup_side_grasp_fails_if_no_candidate_verifies(actions):
    """If nothing in the candidate search verifies, pickup should fail
    cleanly rather than attempt an unverified pose."""

    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 0.3, "y": -0.1, "z": 0.02}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.1, 0.07, 0.04]}
    })
    actions.verify_grasp_pose = MagicMock(return_value=False)
    actions.call_move_service = MagicMock(return_value={"success": True})

    result = actions.handlers['pickup']({"target": "box", "grasp_style": "side"})

    assert result is False
    actions.call_move_service.assert_not_called()


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

    result = actions.handlers['pour']({"target": "red_cube", "destination": "delivery_tray"})

    assert result is False


def test_pour_requires_destination_or_direction(actions):
    """pour without either a destination or a direction should fail cleanly."""

    actions.held_object = "red_cube"

    result = actions.handlers['pour']({"target": "red_cube"})

    assert result is False


def test_pour_lifts_transits_tilts_and_returns(actions):
    """pour should lift, move above the destination, tilt via a pure
    joint_6 delta, dwell, then rotate back level via the negated delta -
    all with orientation preserved (zero-delta) on the Cartesian legs."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(side_effect=lambda name: {
        "red_cube": {
            "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {}},
            "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
        },
        "delivery_tray": {
            "pose": {"position": {"x": 0.5, "y": 0.4, "z": 0.0}, "orientation": {}},
            "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.3, 0.2, 0.02]}
        }
    }[name])
    actions.call_relative_move_service = MagicMock(return_value={"success": True})
    actions.call_joint_move_service = MagicMock(return_value={"success": True})

    with patch("kinova_interface.arm_actions.time.sleep") as mock_sleep:
        result = actions.handlers['pour']({
            "target": "red_cube", "destination": "delivery_tray",
            "lift_height": 0.14, "tilt_angle": 2.36, "duration": 2.0
        })

    assert result is True
    mock_sleep.assert_called_once_with(2.0)

    assert actions.call_relative_move_service.call_count == 2
    lift_call = actions.call_relative_move_service.call_args_list[0][0]
    transit_call = actions.call_relative_move_service.call_args_list[1][0]

    # lift_call args: (vx, vy, vz, has_orientation, roll_delta, pitch_delta, yaw_delta, motion_params)
    assert lift_call[0:3] == (0.0, 0.0, pytest.approx(0.14))
    assert lift_call[3] is True
    assert lift_call[4:7] == (0.0, 0.0, 0.0)  # orientation preserved, not changed

    # moves by the vector from the held object to the destination
    assert transit_call[0:3] == (pytest.approx(0.2), pytest.approx(0.3), 0.0)
    assert transit_call[4:7] == (0.0, 0.0, 0.0)

    assert actions.call_joint_move_service.call_count == 2
    tilt_call = actions.call_joint_move_service.call_args_list[0]
    untilt_call = actions.call_joint_move_service.call_args_list[1]

    assert tilt_call[0][0] == [0.0, 0.0, 0.0, 0.0, 0.0, pytest.approx(2.36)]
    assert tilt_call[1]['relative'] is True
    assert untilt_call[0][0] == [0.0, 0.0, 0.0, 0.0, 0.0, pytest.approx(-2.36)]
    assert untilt_call[1]['relative'] is True


def test_pour_fails_if_lift_fails(actions):
    """pour should stop and fail cleanly if the initial lift fails, never
    attempting the tilt."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    })
    actions.call_relative_move_service = MagicMock(return_value={"success": False})
    actions.call_joint_move_service = MagicMock(return_value={"success": True})

    result = actions.handlers['pour']({"target": "red_cube", "direction": "forward"})

    assert result is False
    actions.call_joint_move_service.assert_not_called()


def test_pour_fails_if_tilt_fails(actions):
    """pour should fail cleanly (and never open the gripper - this action
    doesn't release at all) if the joint-space tilt fails."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    })
    actions.call_relative_move_service = MagicMock(return_value={"success": True})
    actions.call_joint_move_service = MagicMock(return_value=None)

    with patch("kinova_interface.arm_actions.time.sleep") as mock_sleep:
        result = actions.handlers['pour']({"target": "red_cube", "direction": "forward"})

    assert result is False
    mock_sleep.assert_not_called()


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


def test_thrust_requires_destination_or_direction(actions):
    """thrust without either a destination or a direction should fail
    cleanly, before ever touching the arm."""

    actions.held_object = "red_cube"

    result = actions.handlers['thrust']({"target": "red_cube"})

    assert result is False


def test_thrust_raises_spins_and_extends(actions):
    """thrust should raise straight up (facing wherever the object was
    originally resting, wrist reset to level), spin to face the thrust
    direction (joint_1 only, shoulder/elbow held from the raise), then
    extend toward the destination (shoulder/elbow only, seeded at the
    spin position) - reusing the exact same single-plane mechanism as
    push (see docs/push-motion-reference.md)."""

    actions.held_object = "red_cube"
    origin_x, origin_y, origin_z = 0.3, 0.1, 0.02
    target_info = {
        "pose": {"position": {"x": origin_x, "y": origin_y, "z": origin_z}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    }
    dest_info = {
        "pose": {"position": {"x": -0.2, "y": 0.4, "z": 0.0}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.1, 0.1, 0.02]}
    }
    actions.get_object_info = MagicMock(side_effect=lambda name: {
        "red_cube": target_info, "target_spot": dest_info
    }[name])

    face_yaw = math.atan2(origin_y, origin_x)
    thrust_yaw = math.atan2(0.4, -0.2)
    raise_z = origin_z + actions._POUR_DEFAULT_LIFT_HEIGHT

    def solve_side_effect(base_yaw, x, y, z, seed_shoulder=None, seed_elbow=None):
        if seed_shoulder is None:
            assert base_yaw == pytest.approx(face_yaw)
            assert (x, y, z) == pytest.approx((origin_x, origin_y, raise_z))
            return (-0.1, 1.8, (x, y, z), 0.0)
        assert base_yaw == pytest.approx(thrust_yaw)
        assert seed_shoulder == pytest.approx(-0.1)
        assert seed_elbow == pytest.approx(1.8)
        return (-0.4, 1.5, (x, y, z), 0.0)

    actions.solve_planar_reach = MagicMock(side_effect=solve_side_effect)
    actions.check_joint_state_validity = MagicMock(return_value=True)
    actions.call_joint_move_service = MagicMock(return_value={"success": True})

    result = actions.handlers['thrust']({"target": "red_cube", "destination": "target_spot"})

    assert result is True
    raise_call, spin_call, extend_call = actions.call_joint_move_service.call_args_list

    assert raise_call[0][0] == [pytest.approx(face_yaw), pytest.approx(-0.1), pytest.approx(1.8), 0.0, 0.0, 0.0]
    assert spin_call[0][0] == [pytest.approx(thrust_yaw), pytest.approx(-0.1), pytest.approx(1.8), 0.0, 0.0, 0.0]
    assert extend_call[0][0] == [pytest.approx(thrust_yaw), pytest.approx(-0.4), pytest.approx(1.5), 0.0, 0.0, 0.0]


def test_thrust_fails_if_raise_unsolvable(actions):
    """If solve_planar_reach can't find the raised pose at all, thrust
    should fail cleanly before ever moving."""

    actions.held_object = "red_cube"
    target_info = {
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    }
    actions.get_object_info = MagicMock(return_value=target_info)
    actions.solve_planar_reach = MagicMock(return_value=None)
    actions.call_joint_move_service = MagicMock()

    result = actions.handlers['thrust']({"target": "red_cube", "direction": "forward"})

    assert result is False
    actions.call_joint_move_service.assert_not_called()


def test_thrust_fails_if_raise_pose_in_collision(actions):
    """A geometrically-solved raise pose that's actually in collision
    should fail cleanly, without ever moving."""

    actions.held_object = "red_cube"
    target_info = {
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    }
    actions.get_object_info = MagicMock(return_value=target_info)
    actions.solve_planar_reach = MagicMock(return_value=(-0.1, 1.8, (0.3, 0.1, 0.16), 0.0))
    actions.check_joint_state_validity = MagicMock(return_value=False)
    actions.call_joint_move_service = MagicMock()

    result = actions.handlers['thrust']({"target": "red_cube", "direction": "forward"})

    assert result is False
    actions.call_joint_move_service.assert_not_called()


def test_thrust_fails_if_spin_pose_in_collision(actions):
    """If the raise succeeds but spinning to face the thrust direction
    would be in collision, thrust should fail cleanly rather than spin
    into it."""

    actions.held_object = "red_cube"
    target_info = {
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    }
    actions.get_object_info = MagicMock(return_value=target_info)
    actions.solve_planar_reach = MagicMock(return_value=(-0.1, 1.8, (0.3, 0.1, 0.16), 0.0))
    # raise pose valid, spin pose (same shoulder/elbow, different yaw) invalid
    actions.check_joint_state_validity = MagicMock(side_effect=[True, False])
    actions.call_joint_move_service = MagicMock(return_value={"success": True})

    result = actions.handlers['thrust']({"target": "red_cube", "direction": "forward"})

    assert result is False
    # the raise itself should still have been attempted (it was valid)
    assert actions.call_joint_move_service.call_count == 1


def test_thrust_fails_if_extend_unsolvable(actions):
    """If the raise and spin succeed but the extend can't be solved,
    thrust should fail cleanly rather than attempt an unresolved move."""

    actions.held_object = "red_cube"
    target_info = {
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    }
    actions.get_object_info = MagicMock(return_value=target_info)

    def solve_side_effect(base_yaw, x, y, z, seed_shoulder=None, seed_elbow=None):
        if seed_shoulder is None:
            return (-0.1, 1.8, (x, y, z), 0.0)
        return None

    actions.solve_planar_reach = MagicMock(side_effect=solve_side_effect)
    actions.check_joint_state_validity = MagicMock(return_value=True)
    actions.call_joint_move_service = MagicMock(return_value={"success": True})

    result = actions.handlers['thrust']({"target": "red_cube", "direction": "forward"})

    assert result is False
    # raise and spin both happen (2 real moves) before the extend fails to solve
    assert actions.call_joint_move_service.call_count == 2


# push()
def test_push_requires_target_and_destination(actions):
    """push without both a target and a destination should fail cleanly."""

    assert actions.handlers['push']({"target": "red_cube"}) is False
    assert actions.handlers['push']({"destination": "delivery_tray"}) is False


def test_push_faces_object_and_extends_shoulder_elbow_only(actions):
    """push should face the object (joint_1 = its bearing from the arm),
    solve a planar reach (solve_planar_reach) for both the contact pose
    and the extended end pose, verify both collision-free
    (check_joint_state_validity), then approach/close/extend - all while
    contact with the object is temporarily permitted."""

    target_info = {
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.06, 0.06, 0.03]}
    }
    dest_info = {
        "pose": {"position": {"x": 0.6, "y": 0.2, "z": 0.0}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.3, 0.2, 0.02]}
    }
    actions.get_object_info = MagicMock(side_effect=lambda name: {
        "push_block": target_info, "delivery_tray": dest_info
    }[name])

    expected_base_yaw = math.atan2(0.1, 0.3)

    def solve_side_effect(base_yaw, x, y, z, seed_shoulder=None, seed_elbow=None):
        assert base_yaw == pytest.approx(expected_base_yaw)
        if (x, y) == (0.3, 0.1):
            return (-0.3, 2.0, (0.3, 0.1, 0.02), 0.0)
        if (x, y) == (0.6, 0.2):
            assert seed_shoulder == pytest.approx(-0.3)
            assert seed_elbow == pytest.approx(2.0)
            return (-0.5, 1.6, (0.6, 0.2, 0.02), 0.0)
        raise AssertionError(f"unexpected target ({x}, {y})")

    actions.solve_planar_reach = MagicMock(side_effect=solve_side_effect)
    actions.check_joint_state_validity = MagicMock(return_value=True)
    actions.set_collision_allowed = MagicMock(return_value=True)
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.call_joint_move_service = MagicMock(return_value={"success": True})
    actions.update_object_pose = MagicMock(return_value=True)

    result = actions.handlers['push']({"target": "push_block", "destination": "delivery_tray"})

    assert result is True

    allow_calls = actions.set_collision_allowed.call_args_list
    assert allow_calls[0][0] == ("push_block", True)
    assert allow_calls[-1][0] == ("push_block", False)

    approach_call, extend_call = actions.call_joint_move_service.call_args_list
    assert approach_call[0][0] == [pytest.approx(expected_base_yaw), pytest.approx(-0.3), pytest.approx(2.0), 0.0, 0.0, 0.0]
    assert extend_call[0][0] == [pytest.approx(expected_base_yaw), pytest.approx(-0.5), pytest.approx(1.6), 0.0, 0.0, 0.0]

    actions.update_object_pose.assert_called_once_with(
        "push_block", 0.6, 0.2, 0.02, {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    )


def test_push_fails_if_contact_reach_unsolvable(actions):
    """If solve_planar_reach can't find the contact pose at all (returns
    None), push should fail cleanly before ever moving, but still revert
    the collision allowance it already granted (contact with the target
    is permitted up front now - see test_push_allows_target_object_contact_before_checking)."""

    target_info = {
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.06, 0.06, 0.03]}
    }
    actions.get_object_info = MagicMock(return_value=target_info)
    actions.solve_planar_reach = MagicMock(return_value=None)
    actions.set_collision_allowed = MagicMock(return_value=True)
    actions.call_joint_move_service = MagicMock()

    result = actions.handlers['push']({"target": "push_block", "direction": "forward"})

    assert result is False
    actions.call_joint_move_service.assert_not_called()
    allow_calls = actions.set_collision_allowed.call_args_list
    assert allow_calls[0][0] == ("push_block", True)
    assert allow_calls[-1][0] == ("push_block", False)


def test_push_fails_if_contact_reach_error_too_large(actions):
    """A solved reach with a large position error should be treated as a
    failure to converge, not trusted just because some result came back."""

    target_info = {
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.06, 0.06, 0.03]}
    }
    actions.get_object_info = MagicMock(return_value=target_info)
    actions.solve_planar_reach = MagicMock(return_value=(-0.3, 2.0, (0.1, 0.1, 0.5), 0.3))
    actions.set_collision_allowed = MagicMock(return_value=True)

    result = actions.handlers['push']({"target": "push_block", "direction": "forward"})

    assert result is False
    allow_calls = actions.set_collision_allowed.call_args_list
    assert allow_calls[-1][0] == ("push_block", False)


def test_push_allows_target_object_contact_before_checking(actions):
    """Contact with the target object is the whole point of a push - it
    should be permitted (set_collision_allowed) *before* the pose
    validity checks, not after, so a geometrically-necessary overlap
    with a bigger object (e.g. reaching a large box's exact center)
    isn't rejected as if it were an unrelated collision. Verified this
    matters for real: a plain box, much bigger than the small test
    block this was first built against, needs exactly this (see
    docs/push-motion-reference.md)."""

    target_info = {
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.1, 0.08, 0.05]}
    }
    actions.get_object_info = MagicMock(return_value=target_info)

    call_order = []
    actions.set_collision_allowed = MagicMock(
        side_effect=lambda *a: call_order.append("set_collision_allowed") or True
    )
    actions.solve_planar_reach = MagicMock(
        side_effect=lambda *a, **k: call_order.append("solve_planar_reach") or (-0.3, 2.0, (0.3, 0.1, 0.02), 0.0)
    )
    actions.check_joint_state_validity = MagicMock(
        side_effect=lambda *a: call_order.append("check_joint_state_validity") or True
    )
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.call_joint_move_service = MagicMock(return_value={"success": True})
    actions.update_object_pose = MagicMock(return_value=True)

    result = actions.handlers['push']({"target": "push_block", "direction": "forward"})

    assert result is True
    assert actions.set_collision_allowed.call_args_list[0][0] == ("push_block", True)
    # allowed BEFORE the geometry/validity checks ever run
    assert call_order.index("set_collision_allowed") < call_order.index("solve_planar_reach")
    assert call_order.index("set_collision_allowed") < call_order.index("check_joint_state_validity")


def test_push_fails_if_contact_pose_in_collision(actions):
    """A geometrically-solved contact pose that's in collision with
    something *other* than the target object should still fail cleanly,
    and still revert the collision allowance."""

    target_info = {
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.06, 0.06, 0.03]}
    }
    actions.get_object_info = MagicMock(return_value=target_info)
    actions.solve_planar_reach = MagicMock(return_value=(-0.3, 2.0, (0.3, 0.1, 0.02), 0.0))
    actions.check_joint_state_validity = MagicMock(return_value=False)
    actions.set_collision_allowed = MagicMock(return_value=True)
    actions.call_joint_move_service = MagicMock()

    result = actions.handlers['push']({"target": "push_block", "direction": "forward"})

    assert result is False
    actions.call_joint_move_service.assert_not_called()
    allow_calls = actions.set_collision_allowed.call_args_list
    assert allow_calls[0][0] == ("push_block", True)
    assert allow_calls[-1][0] == ("push_block", False)


def test_push_fails_if_extend_reach_unsolvable(actions):
    """If the contact pose is fine but the extended end pose can't be
    solved, push should fail cleanly before ever moving, still reverting
    the collision allowance."""

    target_info = {
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.06, 0.06, 0.03]}
    }
    actions.get_object_info = MagicMock(return_value=target_info)

    def solve_side_effect(base_yaw, x, y, z, seed_shoulder=None, seed_elbow=None):
        if seed_shoulder is None:
            return (-0.3, 2.0, (0.3, 0.1, 0.02), 0.0)
        return None  # extend fails to solve

    actions.solve_planar_reach = MagicMock(side_effect=solve_side_effect)
    actions.check_joint_state_validity = MagicMock(return_value=True)
    actions.set_collision_allowed = MagicMock(return_value=True)
    actions.call_joint_move_service = MagicMock()

    result = actions.handlers['push']({"target": "push_block", "direction": "forward"})

    assert result is False
    actions.call_joint_move_service.assert_not_called()
    allow_calls = actions.set_collision_allowed.call_args_list
    assert allow_calls[0][0] == ("push_block", True)
    assert allow_calls[-1][0] == ("push_block", False)


def test_push_reverts_collision_allowance_if_real_move_fails(actions):
    """If the geometry/collision checks all pass but the actual
    joint-space move fails for real, push should still fail cleanly and
    revert the temporary collision allowance."""

    target_info = {
        "pose": {"position": {"x": 0.3, "y": 0.1, "z": 0.02}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.06, 0.06, 0.03]}
    }
    actions.get_object_info = MagicMock(return_value=target_info)
    actions.solve_planar_reach = MagicMock(return_value=(-0.3, 2.0, (0.3, 0.1, 0.02), 0.0))
    actions.check_joint_state_validity = MagicMock(return_value=True)
    actions.set_collision_allowed = MagicMock(return_value=True)
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.call_joint_move_service = MagicMock(return_value={"success": False})

    result = actions.handlers['push']({"target": "push_block", "direction": "forward"})

    assert result is False
    allow_calls = actions.set_collision_allowed.call_args_list
    assert allow_calls[0][0] == ("push_block", True)
    assert allow_calls[-1][0] == ("push_block", False)


# compute_fk() / check_joint_state_validity() / solve_planar_reach()
def test_compute_fk_returns_position(actions):
    """compute_fk should return the (x, y, z) of tool_frame from a
    successful /compute_fk response."""

    actions.compute_fk_client.wait_for_service = MagicMock(return_value=True)
    response = MagicMock()
    response.error_code.val = 1
    response.error_code.SUCCESS = 1
    pose_stamped = MagicMock()
    pose_stamped.pose.position.x = 0.3
    pose_stamped.pose.position.y = 0.1
    pose_stamped.pose.position.z = 0.02
    response.pose_stamped = [pose_stamped]
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response
    actions.compute_fk_client.call_async = MagicMock(return_value=future)

    result = actions.compute_fk([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    assert result == (0.3, 0.1, 0.02)


def test_compute_fk_returns_none_on_failure(actions):
    actions.compute_fk_client.wait_for_service = MagicMock(return_value=True)
    response = MagicMock()
    response.error_code.val = -1
    response.error_code.SUCCESS = 1
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response
    actions.compute_fk_client.call_async = MagicMock(return_value=future)

    assert actions.compute_fk([0.0] * 6) is None


def test_check_joint_state_validity_true_and_false(actions):
    actions.check_state_validity_client.wait_for_service = MagicMock(return_value=True)

    for expected in (True, False):
        response = MagicMock()
        response.valid = expected
        future = MagicMock()
        future.done.return_value = True
        future.result.return_value = response
        actions.check_state_validity_client.call_async = MagicMock(return_value=future)

        assert actions.check_joint_state_validity([0.0] * 6) is expected


def test_solve_planar_reach_converges_to_known_solution(actions):
    """solve_planar_reach's Newton-Raphson search should converge to the
    known-correct (shoulder, elbow) for a synthetic, smoothly-invertible
    stand-in forward-kinematics function - isolating the numerical method
    itself from real robot geometry."""

    def fake_fk(joints):
        _, shoulder, elbow, _, _, _ = joints
        radius = 1.0 + 0.5 * shoulder + 0.3 * elbow
        height = 2.0 - 0.4 * shoulder + 0.6 * elbow
        return (radius, 0.0, height)  # base_yaw = 0, so x = radius, y = 0

    actions.compute_fk = MagicMock(side_effect=fake_fk)

    result = actions.solve_planar_reach(0.0, 1.2, 0.0, 1.9, seed_shoulder=0.0, seed_elbow=0.0)

    assert result is not None
    shoulder, elbow, achieved, error = result
    assert shoulder == pytest.approx(0.357143, abs=1e-4)
    assert elbow == pytest.approx(0.071429, abs=1e-4)
    assert error < 1e-6


def test_solve_planar_reach_returns_none_if_fk_fails(actions):
    actions.compute_fk = MagicMock(return_value=None)

    assert actions.solve_planar_reach(0.0, 1.0, 0.0, 1.0) is None


def test_push_fails_if_target_unresolved(actions):
    """push should fail cleanly if the target object can't be resolved."""

    actions.get_object_info = MagicMock(return_value=None)

    result = actions.handlers['push']({"target": "unknown", "destination": "delivery_tray"})

    assert result is False


# _on_joint_state() / call_joint_move_service_async() / wait_for_joint_crossing()
def test_on_joint_state_caches_latest_positions(actions):
    """_on_joint_state should cache the latest name->position mapping."""

    msg = MagicMock()
    msg.name = ['joint_1', 'joint_2']
    msg.position = [0.1, 0.2]

    actions._on_joint_state(msg)

    assert actions.latest_joint_positions == {'joint_1': 0.1, 'joint_2': 0.2}


def test_call_joint_move_service_async_fires_without_waiting(actions):
    """call_joint_move_service_async should send the request and return
    the raw future immediately, without waiting for any response -
    unlike call_joint_move_service, which always waits at least for the
    server's own bounded fire-and-forget window."""

    actions.joint_move_client.wait_for_service = MagicMock(return_value=True)
    future = MagicMock()
    actions.joint_move_client.call_async = MagicMock(return_value=future)

    result = actions.call_joint_move_service_async([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

    assert result is future
    request = actions.joint_move_client.call_async.call_args[0][0]
    assert list(request.joint_positions) == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert request.wait_for_completion is False
    assert request.relative is False


def test_call_joint_move_service_async_returns_none_if_unavailable(actions):
    actions.joint_move_client.wait_for_service = MagicMock(return_value=False)

    assert actions.call_joint_move_service_async([0.0] * 6) is None


def test_wait_for_joint_crossing_detects_decreasing_crossing(actions):
    """wait_for_joint_crossing should return True as soon as the joint's
    value crosses the threshold while decreasing from starting_value."""

    actions.latest_joint_positions = {'joint_5': 0.4}

    def side_effect(*a, **k):
        actions.latest_joint_positions = {'joint_5': -1.0}

    with patch("kinova_interface.arm_actions.time.sleep", side_effect=side_effect):
        result = actions.wait_for_joint_crossing('joint_5', -0.5, 0.4, timeout=1.0)

    assert result is True


def test_wait_for_joint_crossing_times_out_if_never_crossed(actions):
    """If the joint never actually reaches the threshold, this should
    time out and return False - not hang forever or assume success."""

    actions.latest_joint_positions = {'joint_5': 0.4}

    result = actions.wait_for_joint_crossing('joint_5', -0.5, 0.4, timeout=0.05)

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


def _mock_future(result=None):
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = result
    return future


def test_throw_rotates_winds_up_flings_and_releases_on_joint5_crossing(actions):
    """throw should rotate to face the throw direction (joint_1 only,
    current shoulder/elbow/wrist held exactly as read from
    latest_joint_positions), move straight to the captured wind-up shape,
    fire the fling asynchronously (not waiting for any response), then
    release the instant joint_5 crosses the captured release point -
    not after a fixed delay."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 0.5, "y": 0.1, "z": 0.0}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.3, 0.2, 0.02]}
    })
    actions.latest_joint_positions = {
        'joint_1': 0.0, 'joint_2': 0.1, 'joint_3': 0.2, 'joint_4': 0.3, 'joint_5': 0.4, 'joint_6': 0.5
    }
    actions.call_joint_move_service = MagicMock(return_value={"success": True})
    actions.call_joint_move_service_async = MagicMock(return_value=_mock_future())
    actions.wait_for_joint_crossing = MagicMock(return_value=True)
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.detach_object = MagicMock(return_value=True)
    actions.update_object_pose = MagicMock(return_value=True)

    result = actions.handlers['throw']({"target": "red_cube", "destination": "delivery_tray"})

    assert result is True

    # destination == target here (same mocked get_object_info return), so
    # the release point is (0.5, 0.1) - the bearing from the arm's base
    expected_yaw = math.atan2(0.1, 0.5)

    rotate_args = actions.call_joint_move_service.call_args_list[0][0]
    windup_args = actions.call_joint_move_service.call_args_list[1][0]
    fling_args = actions.call_joint_move_service_async.call_args_list[0][0]

    rotate_joints, windup_joints, fling_joints = rotate_args[0], windup_args[0], fling_args[0]

    # rotate: joint_1 = face yaw, everything else exactly as currently held
    assert rotate_joints[0] == pytest.approx(expected_yaw)
    assert rotate_joints[1:] == [0.1, 0.2, 0.3, 0.4, 0.5]

    # windup/fling: the captured, fixed shapes, joint_1 = the same face yaw
    assert windup_joints[0] == pytest.approx(expected_yaw)
    assert windup_joints[1:] == pytest.approx(actions._THROW_WINDUP_POSE)
    assert fling_joints[0] == pytest.approx(expected_yaw)
    assert fling_joints[1:] == pytest.approx(actions._THROW_FLING_POSE)

    # release trigger: joint_5, moving from the wind-up's own joint_5
    # toward the captured release threshold - not a timed sleep
    crossing_args = actions.wait_for_joint_crossing.call_args[0]
    assert crossing_args[0] == 'joint_5'
    assert crossing_args[1] == pytest.approx(actions._THROW_RELEASE_JOINT5)
    assert crossing_args[2] == pytest.approx(actions._THROW_WINDUP_POSE[3])

    actions.detach_object.assert_called_once_with("red_cube")
    assert actions.held_object is None


def test_throw_fails_if_no_joint_state_available_to_rotate(actions):
    """throw should fail cleanly (not guess or crash) if no /joint_states
    message has arrived yet to read the current shoulder/elbow/wrist from."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 0.5, "y": 0.1, "z": 0.0}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.3, 0.2, 0.02]}
    })
    actions.latest_joint_positions = None
    actions.call_joint_move_service = MagicMock()

    result = actions.handlers['throw']({"target": "red_cube", "destination": "delivery_tray"})

    assert result is False
    actions.call_joint_move_service.assert_not_called()


def test_throw_fails_if_rotate_fails(actions):
    """If rotating to face the throw direction fails, throw should fail
    cleanly before ever attempting the wind-up."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 0.5, "y": 0.1, "z": 0.0}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.3, 0.2, 0.02]}
    })
    actions.latest_joint_positions = {
        'joint_1': 0.0, 'joint_2': 0.1, 'joint_3': 0.2, 'joint_4': 0.3, 'joint_5': 0.4, 'joint_6': 0.5
    }
    actions.call_joint_move_service = MagicMock(return_value=None)
    actions.call_joint_move_service_async = MagicMock()

    result = actions.handlers['throw']({"target": "red_cube", "destination": "delivery_tray"})

    assert result is False
    assert actions.call_joint_move_service.call_count == 1
    actions.call_joint_move_service_async.assert_not_called()


def test_throw_fails_if_windup_fails(actions):
    """If the rotate succeeds but the wind-up move fails, throw should
    fail cleanly before ever attempting the fling."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 0.5, "y": 0.1, "z": 0.0}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.3, 0.2, 0.02]}
    })
    actions.latest_joint_positions = {
        'joint_1': 0.0, 'joint_2': 0.1, 'joint_3': 0.2, 'joint_4': 0.3, 'joint_5': 0.4, 'joint_6': 0.5
    }
    actions.call_joint_move_service = MagicMock(side_effect=[{"success": True}, None])
    actions.call_joint_move_service_async = MagicMock()

    result = actions.handlers['throw']({"target": "red_cube", "destination": "delivery_tray"})

    assert result is False
    actions.call_joint_move_service_async.assert_not_called()


def test_throw_fails_if_release_point_never_crossed(actions):
    """If joint_5 never actually crosses the release threshold (a real
    timeout, not a guess about elapsed time), throw should fail cleanly
    and never open the gripper on an arm that hasn't reached it."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 0.5, "y": 0.1, "z": 0.0}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.3, 0.2, 0.02]}
    })
    actions.latest_joint_positions = {
        'joint_1': 0.0, 'joint_2': 0.1, 'joint_3': 0.2, 'joint_4': 0.3, 'joint_5': 0.4, 'joint_6': 0.5
    }
    actions.call_joint_move_service = MagicMock(return_value={"success": True})
    actions.call_joint_move_service_async = MagicMock(return_value=_mock_future())
    actions.wait_for_joint_crossing = MagicMock(return_value=False)
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})

    result = actions.handlers['throw']({"target": "red_cube", "destination": "delivery_tray"})

    assert result is False
    actions.call_move_gripper_service.assert_not_called()
    assert actions.held_object == "red_cube"


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
    'destination' when deciding which way to face, computed from the held
    object's own original bearing."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 1.0, "y": 0.0, "z": 0.05}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    })
    actions.latest_joint_positions = {
        'joint_1': 0.0, 'joint_2': 0.1, 'joint_3': 0.2, 'joint_4': 0.3, 'joint_5': 0.4, 'joint_6': 0.5
    }
    actions.call_joint_move_service = MagicMock(return_value={"success": True})
    actions.call_joint_move_service_async = MagicMock(return_value=_mock_future())
    actions.wait_for_joint_crossing = MagicMock(return_value=True)
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})
    actions.detach_object = MagicMock(return_value=True)
    actions.update_object_pose = MagicMock(return_value=True)

    result = actions.handlers['throw']({"target": "red_cube", "direction": "left", "distance": 0.3})

    assert result is True

    # left = bearing (1,0) rotated +90 degrees -> (0,1), scaled by distance,
    # giving release point (1.0, 0.3); face yaw points at that from the arm's base
    expected_yaw = math.atan2(0.3, 1.0)
    rotate_joints = actions.call_joint_move_service.call_args_list[0][0][0]
    assert rotate_joints[0] == pytest.approx(expected_yaw)

    actions.update_object_pose.assert_called_once()
    pose_args = actions.update_object_pose.call_args[0]
    assert pose_args[0] == "red_cube"
    assert pose_args[1] == pytest.approx(1.0)
    assert pose_args[2] == pytest.approx(0.3)
    assert pose_args[3] == pytest.approx(0.05)


def test_throw_fails_on_unknown_direction(actions):
    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 1.0, "y": 0.0, "z": 0.05}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    })

    result = actions.handlers['throw']({"target": "red_cube", "direction": "sideways"})

    assert result is False


def test_throw_fails_if_fling_does_not_start(actions):
    """If the fling itself can't even be started (e.g. the joint move
    service is unavailable), throw should fail cleanly rather than
    releasing the gripper on an arm that never actually swung."""

    actions.held_object = "red_cube"
    actions.get_object_info = MagicMock(return_value={
        "pose": {"position": {"x": 0.5, "y": 0.1, "z": 0.0}, "orientation": {}},
        "shape": {"type": SolidPrimitive.BOX, "dimensions": [0.05, 0.05, 0.05]}
    })
    actions.latest_joint_positions = {
        'joint_1': 0.0, 'joint_2': 0.1, 'joint_3': 0.2, 'joint_4': 0.3, 'joint_5': 0.4, 'joint_6': 0.5
    }
    # rotate and windup succeed, but the fling itself fails to start
    actions.call_joint_move_service = MagicMock(return_value={"success": True})
    actions.call_joint_move_service_async = MagicMock(return_value=None)
    actions.call_move_gripper_service = MagicMock(return_value={"success": True})

    result = actions.handlers['throw']({"target": "red_cube", "destination": "delivery_tray"})

    assert result is False
    actions.call_move_gripper_service.assert_not_called()
    assert actions.held_object == "red_cube"


# reset_environment()
def test_reset_environment_success(actions):
    """reset_environment should call /reset_environment_scene and clear
    locally-tracked held-object state."""

    actions.held_object = "red_cube"
    actions.reset_scene_client.wait_for_service = MagicMock(return_value=True)

    response = MagicMock()
    response.success = True
    response.message = "Environment reset: 3 object(s), 1 obstacle(s) restored to configured defaults"
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response
    actions.reset_scene_client.call_async = MagicMock(return_value=future)

    success, message = actions.reset_environment()

    assert success is True
    assert message == response.message
    assert actions.held_object is None


def test_reset_environment_service_unavailable(actions):
    """reset_environment should fail cleanly if the scene-reset service
    isn't up, without raising."""

    actions.reset_scene_client.wait_for_service = MagicMock(return_value=False)

    success, message = actions.reset_environment()

    assert success is False
    assert "not available" in message


def test_reset_environment_clears_held_object_even_on_failure(actions):
    """Even if the scene-side reset fails, local held-object tracking
    should still be cleared - a reset request means 'start over'."""

    actions.held_object = "red_cube"
    actions.reset_scene_client.wait_for_service = MagicMock(return_value=True)

    response = MagicMock()
    response.success = False
    response.message = "Failed to apply the reset planning scene"
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response
    actions.reset_scene_client.call_async = MagicMock(return_value=future)

    success, message = actions.reset_environment()

    assert success is False
    assert actions.held_object is None
