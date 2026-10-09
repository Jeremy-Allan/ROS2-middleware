"""Plan pickup and dropoff moves with MoveIt's planning services, then run them, the way pymoveit2 does.

Planning only returns a trajectory, so nothing moves until execute(). That lets a whole pick or place be
planned (each move starting where the last one ends) before the arm commits to it.

Each move tries Pilz first and falls back if Pilz can't plan it:
  free-space moves: Pilz PTP, then OMPL RRT*        (/plan_kinematic_path)
  straight lines:   Pilz LIN, then MoveIt's own     (/compute_cartesian_path)
Goals are poses, so the planner picks the joint solution itself.
"""
import math
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry, AllowedCollisionMatrix, Constraints, MoveItErrorCodes, OrientationConstraint,
    PlanningScene, PlanningSceneComponents, PositionConstraint, RobotState, RobotTrajectory,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPlanningScene, GetStateValidity
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive

from kinova_interfaces.srv import ExecuteTrajectory
from kinova_interface.utils.grasping import Waypoint
from kinova_interface.utils.robot import BASE_FRAME, GRIPPER_JOINT, JOINT_NAMES, TOOL_FRAME, moveit_error
from kinova_interface.utils.ros import call_service

PLANNING_GROUP = 'arm'
PILZ = 'pilz_industrial_motion_planner'
PILZ_PLANNING_TIME = 1.0        # s, Pilz answers in milliseconds
FALLBACK_ATTEMPTS = 4           # parallel RRT* runs, kept to one batch so it takes fallback_planning_time

# Goal tolerances for RRT* (pymoveit2's defaults). Pilz always goes to the exact pose.
POSITION_TOLERANCE = 0.001      # m
ORIENTATION_TOLERANCE = 0.001   # rad, per axis

CARTESIAN_STEP = 0.0025         # m between /compute_cartesian_path waypoints
MAX_JOINT_JUMP = 0.5            # rad between those waypoints, more means the wrist would flip
MIN_CARTESIAN_FRACTION = 0.999  # a straight line only counts if all of it was planned

EXECUTE_SERVICE = '/kinova_hardware_client/execute_trajectory'
EXECUTE_TIMEOUT_MARGIN = 70.0   # s on top of the trajectory's own length, the hardware client waits up to 30 s past it


class MotionUnavailable(Exception):
    """A MoveIt planning service didn't answer, so no plan can be made at all."""


@dataclass
class Plan:
    trajectory: RobotTrajectory
    planner: str                # which planner made it, for logs

    @property
    def end_joints(self) -> list[float]:
        jt = self.trajectory.joint_trajectory
        last = jt.points[-1].positions
        return [last[list(jt.joint_names).index(name)] for name in JOINT_NAMES]

    @property
    def duration(self) -> float:
        return _seconds(self.trajectory.joint_trajectory.points[-1].time_from_start)


def _seconds(duration) -> float:
    return duration.sec + duration.nanosec * 1e-9


def _start_state(start: Optional[list[float]], gripper: Optional[float] = None) -> RobotState:
    """The current state, with the arm joints moved to `start` and the gripper to `gripper` if given.
    is_diff keeps the rest of it, like an attached object, so that gets collision-checked too."""
    state = RobotState()
    state.is_diff = True
    if start is not None:
        state.joint_state.name = list(JOINT_NAMES)
        state.joint_state.position = [float(p) for p in start]
    if gripper is not None:
        state.joint_state.name.append(GRIPPER_JOINT)
        state.joint_state.position.append(float(gripper))
    return state


def _pose(position, rotation: Rotation) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
    q = rotation.as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (float(v) for v in q)
    return pose


def _pose_goal(position, rotation: Rotation) -> Constraints:
    """tool_frame at the pose, as MoveIt goal constraints (a small sphere plus an orientation)."""
    pose = _pose(position, rotation)

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = BASE_FRAME
    position_constraint.link_name = TOOL_FRAME
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[POSITION_TOLERANCE])
    position_constraint.constraint_region.primitives.append(sphere)
    position_constraint.constraint_region.primitive_poses.append(Pose(position=pose.position))
    position_constraint.weight = 1.0

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = BASE_FRAME
    orientation_constraint.link_name = TOOL_FRAME
    orientation_constraint.orientation = pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE
    orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE
    orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE
    orientation_constraint.weight = 1.0

    return Constraints(position_constraints=[position_constraint], orientation_constraints=[orientation_constraint])


def slow_down(trajectory: RobotTrajectory, scale: float) -> RobotTrajectory:
    """Stretch a trajectory timed at full speed to `scale` of it. /compute_cartesian_path has no speed setting."""
    if scale >= 1.0:
        return trajectory
    for point in trajectory.joint_trajectory.points:
        t = _seconds(point.time_from_start) / scale
        point.time_from_start.sec, point.time_from_start.nanosec = int(t), int((t % 1) * 1e9)
        point.velocities = [v * scale for v in point.velocities]
        point.accelerations = [a * scale * scale for a in point.accelerations]
    return trajectory


class Motion:
    """Plans and runs tool_frame moves for pickup and dropoff."""

    def __init__(self, node):
        self.logger = node.get_logger()
        self.plan_client = node.create_client(GetMotionPlan, '/plan_kinematic_path', callback_group=node.cb_group)
        self.cartesian_client = node.create_client(GetCartesianPath, '/compute_cartesian_path', callback_group=node.cb_group)
        self.execute_client = node.create_client(ExecuteTrajectory, EXECUTE_SERVICE, callback_group=node.cb_group)
        self.validity_client = node.create_client(GetStateValidity, '/check_state_validity', callback_group=node.cb_group)
        self.get_scene_client = node.create_client(GetPlanningScene, '/get_planning_scene', callback_group=node.cb_group)
        self.apply_scene_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=node.cb_group)

    # --- planning ---

    def plan(self, waypoint: Waypoint, rotation: Rotation, config: dict,
             start: Optional[list[float]] = None) -> tuple[Optional[Plan], Optional[str]]:
        """Plan tool_frame to `waypoint` from `start` joints (the current state if None).
        Returns (plan, None), or (None, reason) if neither planner could.
        Raises MotionUnavailable if MoveIt doesn't answer."""
        state = _start_state(start, waypoint.gripper)
        if waypoint.straight:
            attempts = [('Pilz LIN', lambda: self._plan_pose(PILZ, 'LIN', waypoint.position, rotation, state, config)),
                        ('Cartesian path', lambda: self._plan_cartesian(waypoint.position, rotation, state, config))]
        else:
            attempts = [('Pilz PTP', lambda: self._plan_pose(PILZ, 'PTP', waypoint.position, rotation, state, config)),
                        ('RRT*', lambda: self._plan_pose('ompl', 'RRTstarkConfigDefault', waypoint.position,
                                                         rotation, state, config))]
        reasons = []
        for label, attempt in attempts:
            trajectory, reason = attempt()
            if trajectory is not None:
                plan = Plan(trajectory, label)
                self.logger.debug(f"Planned {waypoint.name} with {label} ({plan.duration:.2f} s)")
                return plan, None
            self.logger.debug(f"{label} could not plan {waypoint.name}: {reason}")
            reasons.append(f"{label}: {reason}")
        return None, f"no path to {waypoint.name} ({'; '.join(reasons)})"

    def plan_chain(self, waypoints: list[Waypoint], rotation: Rotation,
                   config: dict) -> tuple[Optional[dict[str, Plan]], Optional[str]]:
        """Plan each waypoint from where the previous one ends, starting at the current state.
        Returns ({name: plan}, None), or (None, reason) at the first that can't be planned."""
        plans, start = {}, None
        for waypoint in waypoints:
            plan, reason = self.plan(waypoint, rotation, config, start)
            if plan is None:
                return None, reason
            plans[waypoint.name] = plan
            start = plan.end_joints
        return plans, None

    def _plan_pose(self, pipeline: str, planner: str, position, rotation: Rotation,
                   start: RobotState, config: dict) -> tuple[Optional[RobotTrajectory], Optional[str]]:
        fallback = pipeline != PILZ
        request = GetMotionPlan.Request()
        r = request.motion_plan_request
        r.group_name = PLANNING_GROUP
        r.pipeline_id = pipeline
        r.planner_id = planner
        r.num_planning_attempts = FALLBACK_ATTEMPTS if fallback else 1
        r.allowed_planning_time = config['fallback_planning_time'] if fallback else PILZ_PLANNING_TIME
        r.max_velocity_scaling_factor = config['velocity_scale']
        r.max_acceleration_scaling_factor = config['acceleration_scale']
        r.workspace_parameters.header.frame_id = BASE_FRAME
        r.workspace_parameters.min_corner.x = r.workspace_parameters.min_corner.y = r.workspace_parameters.min_corner.z = -1.0
        r.workspace_parameters.max_corner.x = r.workspace_parameters.max_corner.y = r.workspace_parameters.max_corner.z = 1.0
        r.start_state = start
        r.goal_constraints = [_pose_goal(position, rotation)]

        response = call_service(self.plan_client, request, '/plan_kinematic_path', self.logger,
                                timeout_sec=r.allowed_planning_time + 5.0)
        if response is None:
            raise MotionUnavailable('no response from /plan_kinematic_path')
        result = response.motion_plan_response
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, moveit_error(result.error_code.val)
        return result.trajectory, None

    def _plan_cartesian(self, position, rotation: Rotation, start: RobotState,
                        config: dict) -> tuple[Optional[RobotTrajectory], Optional[str]]:
        request = GetCartesianPath.Request()
        request.header.frame_id = BASE_FRAME
        request.start_state = start
        request.group_name = PLANNING_GROUP
        request.link_name = TOOL_FRAME
        request.waypoints = [_pose(position, rotation)]
        request.max_step = CARTESIAN_STEP
        request.jump_threshold = 0.0  # use the absolute one below
        request.revolute_jump_threshold = MAX_JOINT_JUMP
        request.avoid_collisions = True

        response = call_service(self.cartesian_client, request, '/compute_cartesian_path', self.logger)
        if response is None:
            raise MotionUnavailable('no response from /compute_cartesian_path')
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, moveit_error(response.error_code.val)
        if response.fraction < MIN_CARTESIAN_FRACTION:
            return None, f'only {math.floor(response.fraction * 100)}% of the straight line is clear'
        return slow_down(response.solution, config['velocity_scale']), None

    # --- scene ---

    def touching(self, object_id: str) -> list[str]:
        """What the attached `object_id` is in contact with right now, e.g. the table it was standing on."""
        request = GetStateValidity.Request()
        request.group_name = PLANNING_GROUP
        request.robot_state.is_diff = True  # the current state, with the object attached
        response = call_service(self.validity_client, request, '/check_state_validity', self.logger)
        if response is None:
            raise MotionUnavailable('no response from /check_state_validity')
        others = set()
        for contact in response.contacts:
            if object_id == contact.contact_body_1:
                others.add(contact.contact_body_2)
            elif object_id == contact.contact_body_2:
                others.add(contact.contact_body_1)
        return sorted(others)

    def allow_contact(self, object_id: str, others: list[str], allowed: bool) -> bool:
        """Allow (or stop allowing) collisions between `object_id` and each of `others` only, via the ACM."""
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        response = call_service(self.get_scene_client, request, '/get_planning_scene', self.logger)
        if response is None:
            return False
        acm = response.scene.allowed_collision_matrix
        names = list(acm.entry_names)
        rows = [list(entry.enabled) for entry in acm.entry_values]
        for name in [object_id, *others]:
            if name not in names:  # unknown names are checked as normal
                names.append(name)
                for row in rows:
                    row.append(False)
                rows.append([False] * len(names))
        i = names.index(object_id)
        for other in others:
            j = names.index(other)
            rows[i][j] = rows[j][i] = allowed

        scene = PlanningScene(is_diff=True)
        scene.allowed_collision_matrix = AllowedCollisionMatrix(
            entry_names=names, entry_values=[AllowedCollisionEntry(enabled=row) for row in rows],
            default_entry_names=acm.default_entry_names, default_entry_values=acm.default_entry_values)
        applied = call_service(self.apply_scene_client, ApplyPlanningScene.Request(scene=scene),
                               '/apply_planning_scene', self.logger)
        return applied is not None and applied.success

    # --- execution ---

    def execute(self, plan: Plan) -> tuple[bool, str]:
        """Run a plan through the hardware interface. Returns (success, message)."""
        request = ExecuteTrajectory.Request()
        request.trajectory = plan.trajectory
        response = call_service(self.execute_client, request, EXECUTE_SERVICE, self.logger,
                                timeout_sec=plan.duration + EXECUTE_TIMEOUT_MARGIN)
        if response is None:
            return False, 'no response from the hardware interface'
        return response.success, response.message

    def move(self, waypoint: Waypoint, rotation: Rotation, config: dict) -> tuple[bool, str]:
        """Plan from where the arm is now, then run it."""
        try:
            plan, reason = self.plan(waypoint, rotation, config)
        except MotionUnavailable as e:
            return False, str(e)
        if plan is None:
            return False, reason
        self.logger.debug(f"Executing {waypoint.name} ({plan.planner}, {plan.duration:.2f} s)")
        return self.execute(plan)
