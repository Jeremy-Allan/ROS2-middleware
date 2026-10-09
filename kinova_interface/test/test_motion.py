import numpy as np
import pytest

from unittest.mock import patch
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from moveit_msgs.msg import AllowedCollisionEntry, ContactInformation, MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import (
    ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPlanningScene, GetPositionFK, GetStateValidity,
)
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

from kinova_interfaces.srv import ExecuteTrajectory
from kinova_interface.actions import motion
from kinova_interface.actions.motion import Motion, MotionUnavailable, Plan, slow_down, tilt_between
from kinova_interface.utils.grasping import Waypoint
from kinova_interface.utils.robot import JOINT_NAMES

CONFIG = {'velocity_scale': 0.5, 'acceleration_scale': 0.2, 'fallback_planning_time': 1.5,
          'upright_tolerance': 0.2}
DOWN = Rotation.from_euler('x', 180, degrees=True)
FREE = Waypoint('pre-grasp', np.array([0.4, 0.0, 0.2]), straight=False)
LINE = Waypoint('grasp', np.array([0.4, 0.0, 0.1]), straight=True)


@pytest.fixture
def mover(ros_context):
    node = Node('test_motion_host')
    node.cb_group = ReentrantCallbackGroup()
    yield Motion(node)
    node.destroy_node()


def _trajectory(end, seconds=2.0, names=JOINT_NAMES):
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(names)
    point = JointTrajectoryPoint(positions=list(end), velocities=[1.0] * 6, accelerations=[2.0] * 6)
    point.time_from_start.sec, point.time_from_start.nanosec = int(seconds), int((seconds % 1) * 1e9)
    trajectory.joint_trajectory.points = [point]
    return trajectory


def _planned(end=(0.1,) * 6):
    response = GetMotionPlan.Response()
    response.motion_plan_response.error_code.val = MoveItErrorCodes.SUCCESS
    response.motion_plan_response.trajectory = _trajectory(end)
    return response


def _not_planned(code=MoveItErrorCodes.NO_IK_SOLUTION):
    response = GetMotionPlan.Response()
    response.motion_plan_response.error_code.val = code
    return response


def _line(fraction=1.0, end=(0.2,) * 6):
    response = GetCartesianPath.Response()
    response.error_code.val = MoveItErrorCodes.SUCCESS
    response.fraction = fraction
    response.solution = _trajectory(end)
    return response


def _services(*responses):
    """Patch call_service to answer with `responses` in order, recording each request."""
    requests, queue = [], list(responses)

    def answer(client, request, *args, **kwargs):
        requests.append(request)
        return queue.pop(0)
    return patch.object(motion, 'call_service', side_effect=answer), requests


# Free-space moves
def test_free_move_is_a_pilz_ptp_pose_goal_for_tool_frame(mover):
    patched, requests = _services(_planned())
    with patched:
        plan, reason = mover.plan(FREE, DOWN, CONFIG)

    assert reason is None and plan.planner == 'Pilz PTP'
    motion_request = requests[0].motion_plan_request
    assert motion_request.pipeline_id == 'pilz_industrial_motion_planner'
    assert (motion_request.planner_id, motion_request.group_name) == ('PTP', 'arm')
    assert motion_request.max_velocity_scaling_factor == 0.5
    assert motion_request.max_acceleration_scaling_factor == 0.2
    start_state = motion_request.start_state
    assert start_state.is_diff and not start_state.joint_state.name  # from the current state
    goal = motion_request.goal_constraints[0]
    position, orientation = goal.position_constraints[0], goal.orientation_constraints[0]
    assert position.link_name == orientation.link_name == 'tool_frame'
    assert position.constraint_region.primitives[0].type == SolidPrimitive.SPHERE
    goal_position = position.constraint_region.primitive_poses[0].position
    assert (goal_position.x, goal_position.y, goal_position.z) == pytest.approx(FREE.position)
    quaternion = orientation.orientation
    goal_rotation = Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    assert (goal_rotation * DOWN.inv()).magnitude() == pytest.approx(0.0, abs=1e-9)
    assert not goal.joint_constraints  # MoveIt picks the joints


def test_free_move_falls_back_to_rrtstar_with_its_own_time(mover):
    patched, requests = _services(_not_planned(), _planned())
    with patched:
        plan, _ = mover.plan(FREE, DOWN, CONFIG)

    assert plan.planner == 'RRT*'
    motion_request = requests[1].motion_plan_request
    assert (motion_request.pipeline_id, motion_request.planner_id) == ('ompl', 'RRTstarkConfigDefault')
    assert motion_request.allowed_planning_time == CONFIG['fallback_planning_time']


def test_plan_reports_the_fallback_reason_when_both_fail(mover):
    patched, _ = _services(_not_planned(), _not_planned(MoveItErrorCodes.PLANNING_FAILED))
    with patched:
        plan, reason = mover.plan(FREE, DOWN, CONFIG)

    assert plan is None
    assert reason == ("no path to pre-grasp (Pilz PTP: out of reach (no IK solution); "
                      "RRT*: planning failed (path blocked by an obstacle or self-collision))")


# Straight lines
def test_straight_move_is_pilz_lin(mover):
    patched, requests = _services(_planned())
    with patched:
        plan, _ = mover.plan(LINE, DOWN, CONFIG)

    assert plan.planner == 'Pilz LIN'
    assert requests[0].motion_plan_request.planner_id == 'LIN'


def test_straight_move_falls_back_to_compute_cartesian_path(mover):
    patched, requests = _services(_not_planned(), _line())
    with patched:
        plan, _ = mover.plan(LINE, DOWN, CONFIG, start=[0.3] * 6)

    assert plan.planner == 'Cartesian path'
    cartesian_request = requests[1]
    assert isinstance(cartesian_request, GetCartesianPath.Request)
    assert cartesian_request.group_name == 'arm' and cartesian_request.link_name == 'tool_frame'
    assert cartesian_request.header.frame_id == 'base_link'
    assert cartesian_request.avoid_collisions and cartesian_request.max_step == motion.CARTESIAN_STEP
    assert cartesian_request.revolute_jump_threshold == motion.MAX_JOINT_JUMP
    assert list(cartesian_request.start_state.joint_state.position) == [0.3] * 6
    # Timed at full speed by MoveIt, slowed to velocity_scale
    assert plan.duration == pytest.approx(2.0 / 0.5)


def test_partial_straight_line_is_rejected(mover):
    patched, _ = _services(_not_planned(), _line(fraction=0.6))
    with patched:
        plan, reason = mover.plan(LINE, DOWN, CONFIG)

    assert plan is None
    assert reason == ("no path to grasp (Pilz LIN: out of reach (no IK solution); "
                      "Cartesian path: only 60% of the straight line is clear)")


def test_a_waypoint_after_the_gripper_opens_is_planned_with_it_open(mover):
    back_off = Waypoint('back-off', np.array([0.4, 0.0, 0.2]), straight=True, gripper=0.0)
    patched, requests = _services(_planned())
    with patched:
        mover.plan(back_off, DOWN, CONFIG, start=[0.3] * 6)

    joints = requests[0].motion_plan_request.start_state.joint_state
    assert list(joints.name) == JOINT_NAMES + ['right_finger_bottom_joint']
    assert list(joints.position) == [0.3] * 6 + [0.0]


# Upright carries
HOVER = Waypoint('hover', np.array([0.4, 0.0, 0.2]), straight=False, upright=True)


def _tool_pose(rotation):
    """/compute_fk answer: where the gripper is pointing right now."""
    quaternion = rotation.as_quat()
    pose = PoseStamped()
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion
    response = GetPositionFK.Response(pose_stamped=[pose])
    response.error_code.val = MoveItErrorCodes.SUCCESS
    return response


def _tolerances(constraint):
    return (constraint.absolute_x_axis_tolerance, constraint.absolute_y_axis_tolerance,
            constraint.absolute_z_axis_tolerance)


def test_upright_carry_tries_pilz_lin_first(mover):
    patched, requests = _services(_tool_pose(DOWN), _planned())
    with patched:
        plan, _ = mover.plan(HOVER, DOWN, CONFIG)

    assert plan.planner == 'Pilz LIN'
    assert not requests[1].motion_plan_request.path_constraints.orientation_constraints


def test_upright_carry_falls_back_to_rrtstar_held_to_the_tilt(mover):
    patched, requests = _services(_tool_pose(DOWN), _not_planned(), _planned())
    with patched:
        plan, _ = mover.plan(HOVER, DOWN, CONFIG)

    assert plan.planner == 'RRT* (upright)'
    motion_request = requests[2].motion_plan_request
    assert motion_request.pipeline_id == 'ompl'
    assert motion_request.allowed_planning_time == CONFIG['fallback_planning_time']
    constraint = motion_request.path_constraints.orientation_constraints[0]
    assert constraint.link_name == 'tool_frame' and constraint.parameterization == constraint.ROTATION_VECTOR
    # Pointing down, tool Z is vertical: tilt held to the tolerance, turning about vertical free
    assert _tolerances(constraint) == pytest.approx((0.2, 0.2, np.pi))


def test_upright_carry_is_dropped_if_the_object_is_already_tipped(mover):
    """e.g. a home move while holding it: keeping it upright from there can't work."""
    sideways = Rotation.from_euler('y', 90, degrees=True)
    patched, requests = _services(_tool_pose(sideways), _not_planned(), _planned())
    with patched:
        plan, _ = mover.plan(HOVER, DOWN, CONFIG)

    assert plan.planner == 'RRT*'  # a normal free-space move: Pilz PTP, then RRT*
    assert requests[1].motion_plan_request.planner_id == 'PTP'
    assert not requests[2].motion_plan_request.path_constraints.orientation_constraints


def test_tilt_ignores_turning_about_vertical():
    turned = Rotation.from_euler('z', 70, degrees=True) * DOWN
    assert tilt_between(DOWN, turned) == pytest.approx(0.0, abs=1e-9)
    assert tilt_between(DOWN, Rotation.from_euler('y', 90, degrees=True)) == pytest.approx(np.pi / 2)


def test_upright_tolerance_frees_whichever_tool_axis_is_vertical():
    side = Rotation.from_euler('y', 90, degrees=True)  # tool Z horizontal, tool X pointing down
    constraint = motion._upright(side, 0.2).orientation_constraints[0]
    assert _tolerances(constraint) == pytest.approx((np.pi, 0.2, 0.2))


# Chains
def test_chain_plans_each_waypoint_from_where_the_last_ends(mover):
    patched, requests = _services(_planned(end=[0.1] * 6), _planned(end=[0.2] * 6))
    with patched:
        plans, reason = mover.plan_chain([FREE, LINE], DOWN, CONFIG)

    assert reason is None and list(plans) == ['pre-grasp', 'grasp']
    assert not requests[0].motion_plan_request.start_state.joint_state.name
    assert list(requests[1].motion_plan_request.start_state.joint_state.position) == [0.1] * 6


def test_chain_stops_at_the_first_unplannable_waypoint(mover):
    patched, requests = _services(_not_planned(), _not_planned())
    with patched:
        plans, reason = mover.plan_chain([FREE, LINE], DOWN, CONFIG)

    assert plans is None and reason.startswith("no path to pre-grasp")
    assert len(requests) == 2  # PTP and RRT* for the pre-grasp, nothing for the grasp


def test_unreachable_moveit_raises_instead_of_failing_the_candidate(mover):
    with patch.object(motion, 'call_service', return_value=None):
        with pytest.raises(MotionUnavailable):
            mover.plan(FREE, DOWN, CONFIG)
        assert mover.move(FREE, DOWN, CONFIG) == (False, 'no response from /plan_kinematic_path')


# Scene
def test_touching_lists_what_the_held_object_is_in_contact_with(mover):
    response = GetStateValidity.Response(valid=False, contacts=[
        ContactInformation(contact_body_1='table', contact_body_2='bottle'),
        ContactInformation(contact_body_1='bottle', contact_body_2='tray'),
        ContactInformation(contact_body_1='forearm_link', contact_body_2='wall'),  # not the object
    ])
    patched, requests = _services(response)
    with patched:
        assert mover.touching('bottle') == ['table', 'tray']
    assert requests[0].robot_state.is_diff  # the current state, attached object included


def test_allow_contact_only_changes_that_pair(mover):
    scene = GetPlanningScene.Response()
    acm = scene.scene.allowed_collision_matrix
    acm.entry_names = ['link', 'table']
    acm.entry_values = [AllowedCollisionEntry(enabled=[True, False]), AllowedCollisionEntry(enabled=[False, False])]
    acm.default_entry_names = ['link']
    acm.default_entry_values = [True]
    patched, requests = _services(scene, ApplyPlanningScene.Response(success=True))
    with patched:
        assert mover.allow_contact('bottle', ['table'], True)

    applied = requests[1].scene.allowed_collision_matrix
    rows = {name: list(entry.enabled) for name, entry in zip(applied.entry_names, applied.entry_values)}
    assert rows == {'link': [True, False, False], 'table': [False, False, True], 'bottle': [False, True, False]}
    assert list(applied.default_entry_names) == ['link'] and requests[1].scene.is_diff


# Execution
def test_move_plans_from_the_current_state_then_executes(mover):
    executed = ExecuteTrajectory.Response(success=True, message="Movement complete")
    patched, requests = _services(_planned(), executed)
    with patched:
        assert mover.move(LINE, DOWN, CONFIG) == (True, "Movement complete")

    assert isinstance(requests[1], ExecuteTrajectory.Request)
    assert requests[1].trajectory == _planned().motion_plan_response.trajectory


# Plan / slow_down
def test_end_joints_are_in_joint_order():
    names = list(reversed(JOINT_NAMES))
    plan = Plan(_trajectory(end=[6, 5, 4, 3, 2, 1], names=names), 'Pilz PTP')
    assert plan.end_joints == [1, 2, 3, 4, 5, 6]


def test_slow_down_stretches_time_and_scales_velocity_and_acceleration():
    point = slow_down(_trajectory([0.0] * 6, seconds=1.5), 0.5).joint_trajectory.points[0]

    assert point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 == pytest.approx(3.0)
    assert list(point.velocities) == [0.5] * 6
    assert list(point.accelerations) == [0.5] * 6
