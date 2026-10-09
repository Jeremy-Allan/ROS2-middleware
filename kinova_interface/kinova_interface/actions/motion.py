"""Plan pickup and dropoff moves with MoveIt's planning services, then run them, the way pymoveit2 does.

Planning only returns a trajectory, so nothing moves until execute(). That lets a whole pick or place be
planned (each move starting where the last one ends) before the arm commits to it.

Each move tries Pilz first and falls back if Pilz can't plan it:
  free-space moves: Pilz PTP, then OMPL RRT*        (/plan_kinematic_path)
  straight lines:   Pilz LIN, then MoveIt's own     (/compute_cartesian_path)
  upright carries:  Pilz LIN, then OMPL RRT* held to the gripper's tilt
Goals are poses, so the planner picks the joint solution itself.
"""
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry, AllowedCollisionMatrix, Constraints, MoveItErrorCodes, OrientationConstraint,
    PlanningScene, PlanningSceneComponents, PositionConstraint, RobotState, RobotTrajectory,
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPlanningScene, GetPositionFK, GetStateValidity,
)
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive

from kinova_interfaces.srv import ExecuteTrajectory
from kinova_interface.utils.grasping import Waypoint
from kinova_interface.utils.robot import BASE_FRAME, GRIPPER_JOINT, JOINT_NAMES, TOOL_FRAME, moveit_error
from kinova_interface.utils.ros import call_service

PLANNING_GROUP = 'arm'
PILZ = 'pilz_industrial_motion_planner'
RRT_STAR = 'RRTstarkConfigDefault'
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
        joint_trajectory = self.trajectory.joint_trajectory
        last_positions = joint_trajectory.points[-1].positions
        return [last_positions[list(joint_trajectory.joint_names).index(name)] for name in JOINT_NAMES]

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
        state.joint_state.position = [float(position) for position in start]
    if gripper is not None:
        state.joint_state.name.append(GRIPPER_JOINT)
        state.joint_state.position.append(float(gripper))
    return state


def _pose(position, rotation: Rotation) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(value) for value in position)
    quaternion = rotation.as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (float(value) for value in quaternion)
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


def _upright(rotation: Rotation, tolerance: float) -> Constraints:
    """Keep tool_frame's tilt as in `rotation`, within `tolerance` rad, but let it turn freely about vertical.
    The rotation-vector error is in tool axes, and turning about vertical moves it along world-up in those axes,
    so each axis gets extra room in proportion to how much it lines up with world-up."""
    world_up_in_tool_axes = rotation.inv().apply([0.0, 0.0, 1.0])
    x_tolerance, y_tolerance, z_tolerance = (
        min(math.pi, tolerance + math.pi * abs(component)) for component in world_up_in_tool_axes)
    constraint = OrientationConstraint(
        link_name=TOOL_FRAME, orientation=_pose([0, 0, 0], rotation).orientation,
        absolute_x_axis_tolerance=x_tolerance, absolute_y_axis_tolerance=y_tolerance,
        absolute_z_axis_tolerance=z_tolerance,
        parameterization=OrientationConstraint.ROTATION_VECTOR, weight=1.0)
    constraint.header.frame_id = BASE_FRAME
    return Constraints(orientation_constraints=[constraint])


def tilt_between(first: Rotation, second: Rotation) -> float:
    """How differently two tool orientations tip whatever the gripper holds, in rad. Turning about vertical doesn't count."""
    world_up = [0.0, 0.0, 1.0]
    first_up, second_up = first.inv().apply(world_up), second.inv().apply(world_up)
    return math.acos(np.clip(np.dot(first_up, second_up), -1.0, 1.0))


def slow_down(trajectory: RobotTrajectory, scale: float) -> RobotTrajectory:
    """Stretch a trajectory timed at full speed to `scale` of it. /compute_cartesian_path has no speed setting."""
    if scale >= 1.0:
        return trajectory
    for point in trajectory.joint_trajectory.points:
        seconds = _seconds(point.time_from_start) / scale
        point.time_from_start.sec, point.time_from_start.nanosec = int(seconds), int((seconds % 1) * 1e9)
        point.velocities = [velocity * scale for velocity in point.velocities]
        point.accelerations = [acceleration * scale * scale for acceleration in point.accelerations]
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
        self.forward_kinematics_client = node.create_client(GetPositionFK, '/compute_fk', callback_group=node.cb_group)

    # --- planning ---

    def plan(self, waypoint: Waypoint, rotation: Rotation, config: dict,
             start: Optional[list[float]] = None) -> tuple[Optional[Plan], Optional[str]]:
        """Plan tool_frame to `waypoint` from `start` joints (the current state if None).
        Returns (plan, None), or (None, reason) if neither planner could.
        Raises MotionUnavailable if MoveIt doesn't answer."""
        start_state = _start_state(start, waypoint.gripper)

        def plan_to_pose(pipeline, planner, planning_time, path_constraints=None):
            return lambda: self._plan_pose(pipeline, planner, waypoint.position, rotation, start_state, config,
                                           planning_time, path_constraints)

        keep_upright = waypoint.upright and self._still_upright(start, rotation, config['upright_tolerance'], waypoint)
        if keep_upright:
            attempts = [('Pilz LIN', plan_to_pose(PILZ, 'LIN', PILZ_PLANNING_TIME)),
                        ('RRT* (upright)', plan_to_pose('ompl', RRT_STAR, config['fallback_planning_time'],
                                                        _upright(rotation, config['upright_tolerance'])))]
        elif waypoint.straight:
            attempts = [('Pilz LIN', plan_to_pose(PILZ, 'LIN', PILZ_PLANNING_TIME)),
                        ('Cartesian path',
                         lambda: self._plan_cartesian(waypoint.position, rotation, start_state, config))]
        else:
            attempts = [('Pilz PTP', plan_to_pose(PILZ, 'PTP', PILZ_PLANNING_TIME)),
                        ('RRT*', plan_to_pose('ompl', RRT_STAR, config['fallback_planning_time']))]
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

    def _plan_pose(self, pipeline: str, planner: str, position, rotation: Rotation, start_state: RobotState,
                   config: dict, planning_time: float, path_constraints: Optional[Constraints] = None
                   ) -> tuple[Optional[RobotTrajectory], Optional[str]]:
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = PLANNING_GROUP
        motion_request.pipeline_id = pipeline
        motion_request.planner_id = planner
        motion_request.num_planning_attempts = 1 if pipeline == PILZ else FALLBACK_ATTEMPTS
        motion_request.allowed_planning_time = float(planning_time)
        motion_request.max_velocity_scaling_factor = config['velocity_scale']
        motion_request.max_acceleration_scaling_factor = config['acceleration_scale']
        workspace = motion_request.workspace_parameters
        workspace.header.frame_id = BASE_FRAME
        workspace.min_corner.x = workspace.min_corner.y = workspace.min_corner.z = -1.0
        workspace.max_corner.x = workspace.max_corner.y = workspace.max_corner.z = 1.0
        motion_request.start_state = start_state
        motion_request.goal_constraints = [_pose_goal(position, rotation)]
        if path_constraints is not None:
            motion_request.path_constraints = path_constraints

        # MoveIt's own pre-planning steps (e.g. fixing the start state) can run past the planning time
        response = call_service(self.plan_client, request, '/plan_kinematic_path', self.logger,
                                timeout_sec=2 * motion_request.allowed_planning_time + 5.0)
        if response is None:
            raise MotionUnavailable('no response from /plan_kinematic_path')
        result = response.motion_plan_response
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, moveit_error(result.error_code.val)
        return result.trajectory, None

    def _plan_cartesian(self, position, rotation: Rotation, start_state: RobotState,
                        config: dict) -> tuple[Optional[RobotTrajectory], Optional[str]]:
        request = GetCartesianPath.Request()
        request.header.frame_id = BASE_FRAME
        request.start_state = start_state
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

    def _still_upright(self, start: Optional[list[float]], rotation: Rotation, tolerance: float,
                       waypoint: Waypoint) -> bool:
        """Whether the gripper at `start` is within `tolerance` of the tilt the carry should keep.
        If something already tipped the held object (e.g. a home move), keeping it upright can't work."""
        request = GetPositionFK.Request()
        request.header.frame_id = BASE_FRAME
        request.fk_link_names = [TOOL_FRAME]
        request.robot_state = _start_state(start)
        response = call_service(self.forward_kinematics_client, request, '/compute_fk', self.logger)
        if response is None:
            raise MotionUnavailable('no response from /compute_fk')
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.logger.warn(f"Couldn't check the held object's tilt ({moveit_error(response.error_code.val)}), "
                             f"carrying it to the {waypoint.name} without keeping it upright")
            return False
        orientation = response.pose_stamped[0].pose.orientation
        current = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        tilt = tilt_between(current, rotation)
        if tilt > tolerance:
            self.logger.warn(f"The held object is already tipped {math.degrees(tilt):.0f} deg from how it was "
                             f"picked up, carrying it to the {waypoint.name} without keeping it upright")
            return False
        return True

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
        collision_matrix = response.scene.allowed_collision_matrix
        names = list(collision_matrix.entry_names)
        rows = [list(entry.enabled) for entry in collision_matrix.entry_values]
        for name in [object_id, *others]:
            if name not in names:  # unknown names are checked as normal
                names.append(name)
                for row in rows:
                    row.append(False)
                rows.append([False] * len(names))
        object_index = names.index(object_id)
        for other in others:
            other_index = names.index(other)
            rows[object_index][other_index] = rows[other_index][object_index] = allowed

        scene = PlanningScene(is_diff=True)
        scene.allowed_collision_matrix = AllowedCollisionMatrix(
            entry_names=names, entry_values=[AllowedCollisionEntry(enabled=row) for row in rows],
            default_entry_names=collision_matrix.default_entry_names,
            default_entry_values=collision_matrix.default_entry_values)
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
        except MotionUnavailable as error:
            return False, str(error)
        if plan is None:
            return False, reason
        self.logger.debug(f"Executing {waypoint.name} ({plan.planner}, {plan.duration:.2f} s)")
        return self.execute(plan)
