import numpy as np
import pytest

from unittest.mock import patch
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from moveit_msgs.msg import AllowedCollisionEntry, ContactInformation, MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

from kinova_interfaces.srv import ExecuteTrajectory
from kinova_interface.actions import motion
from kinova_interface.actions.motion import Motion, MotionUnavailable, Plan, slow_down
from kinova_interface.utils.grasping import Waypoint
from kinova_interface.utils.robot import JOINT_NAMES

CONFIG = {'velocity_scale': 0.5, 'acceleration_scale': 0.2, 'fallback_planning_time': 1.5}
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
    r = requests[0].motion_plan_request
    assert (r.pipeline_id, r.planner_id, r.group_name) == ('pilz_industrial_motion_planner', 'PTP', 'arm')
    assert (r.max_velocity_scaling_factor, r.max_acceleration_scaling_factor) == (0.5, 0.2)
    assert r.start_state.is_diff and not r.start_state.joint_state.name  # from the current state
    goal = r.goal_constraints[0]
    position, orientation = goal.position_constraints[0], goal.orientation_constraints[0]
    assert position.link_name == orientation.link_name == 'tool_frame'
    assert position.constraint_region.primitives[0].type == SolidPrimitive.SPHERE
    p = position.constraint_region.primitive_poses[0].position
    assert (p.x, p.y, p.z) == pytest.approx(FREE.position)
    q = orientation.orientation
    assert (Rotation.from_quat([q.x, q.y, q.z, q.w]) * DOWN.inv()).magnitude() == pytest.approx(0.0, abs=1e-9)
    assert not goal.joint_constraints  # MoveIt picks the joints


def test_free_move_falls_back_to_rrtstar_with_its_own_time(mover):
    patched, requests = _services(_not_planned(), _planned())
    with patched:
        plan, _ = mover.plan(FREE, DOWN, CONFIG)

    assert plan.planner == 'RRT*'
    r = requests[1].motion_plan_request
    assert (r.pipeline_id, r.planner_id) == ('ompl', 'RRTstarkConfigDefault')
    assert r.allowed_planning_time == CONFIG['fallback_planning_time']


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
    r = requests[1]
    assert isinstance(r, GetCartesianPath.Request)
    assert (r.group_name, r.link_name, r.header.frame_id) == ('arm', 'tool_frame', 'base_link')
    assert r.avoid_collisions and r.max_step == motion.CARTESIAN_STEP
    assert r.revolute_jump_threshold == motion.MAX_JOINT_JUMP
    assert list(r.start_state.joint_state.position) == [0.3] * 6
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
