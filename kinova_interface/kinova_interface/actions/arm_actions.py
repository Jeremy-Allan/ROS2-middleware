import math
import time
from functools import partial

import rclpy
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene, GetPositionFK, GetStateValidity, GetPlanningScene
from moveit_msgs.msg import RobotState, PlanningScene, AllowedCollisionMatrix, AllowedCollisionEntry, PlanningSceneComponents

from kinova_interfaces.srv import (
    GetObjectCoordinates,
    GetRelativeMovement,
    GetObjectInfo,
    GetOrientationPreset,
    HomeArm,
    MoveArm,
    MoveGripper,
    RelativeMove,
    JointMove,
    AttachObject,
    DetachObject,
    UpdateObjectPose,
)
from kinova_interfaces.msg import MotionParams
from std_srvs.srv import Trigger

from kinova_interface.actions import basic, pickup, dropoff, pour, thrust, push, throw
from kinova_interface.utils.geometry import euler_to_quaternion, resolve_direction_offset
from kinova_interface.utils.frames import BASE_FRAME, TOOL_FRAME


class ArmActions:
    """Shared context for every recipe action: owns the hardware/environment
    service clients and their call_* wrappers, the MoveIt query helpers
    (IK/FK/state validity/ACM), the planar-reach solver, and held_object
    state. The actions themselves live one per module in this package
    (basic.py, pickup.py, ...), each as a function taking this object as
    `ctx`. Exposes `handlers`, a dict of action name -> callable(params) ->
    bool, for JsonParserNode to dispatch recipe steps against."""

    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger

        cb_group = node.cb_group

        # Hardware Interface Services
        self.home_client = node.create_client(HomeArm, '/kinova_hardware_client/home_arm', callback_group=cb_group)
        self.move_arm_client = node.create_client(MoveArm, '/kinova_hardware_client/move_arm', callback_group=cb_group)
        self.move_gripper_client = node.create_client(MoveGripper, '/kinova_hardware_client/move_gripper', callback_group=cb_group)
        self.joint_move_client = node.create_client(JointMove, '/kinova_hardware_client/joint_move', callback_group=cb_group)
        self.relative_move_client = node.create_client(RelativeMove, '/kinova_hardware_client/relative_move', callback_group=cb_group)

        # Service clients for coordinate/info fetching
        self.coord_client = node.create_client(GetObjectCoordinates, '/get_coordinates', callback_group=cb_group)
        self.relative_client = node.create_client(GetRelativeMovement, '/get_relative_movement', callback_group=cb_group)
        self.info_client = node.create_client(GetObjectInfo, '/get_object_info', callback_group=cb_group)
        self.orientation_client = node.create_client(GetOrientationPreset, '/get_orientation_preset', callback_group=cb_group)

        # Service clients for attach/detach/update-pose, provided by the environment mapping node
        self.attach_client = node.create_client(AttachObject, '/attach_object', callback_group=cb_group)
        self.detach_client = node.create_client(DetachObject, '/detach_object', callback_group=cb_group)
        self.update_pose_client = node.create_client(UpdateObjectPose, '/update_object_pose', callback_group=cb_group)
        self.reset_scene_client = node.create_client(Trigger, '/reset_environment_scene', callback_group=cb_group)

        # For verifying a computed grasp candidate is actually reachable and
        # collision-free before trusting it (see compute_side_grasp_candidates/
        # verify_grasp_pose) - not exposed to MoveIt's own move_group action,
        # a plain service so it can be checked cheaply before ever moving.
        self.compute_ik_client = node.create_client(GetPositionIK, '/compute_ik', callback_group=cb_group)

        # For temporarily permitting deliberate gripper/object contact
        # during 'push' (see set_collision_allowed) - a normal
        # collision-aware plan would otherwise reject, or silently route
        # around, the sustained contact a push actually requires.
        self.apply_planning_scene_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=cb_group)
        self.get_planning_scene_client = node.create_client(GetPlanningScene, '/get_planning_scene', callback_group=cb_group)

        # Latest joint positions (name -> position), for throw's
        # closed-loop release trigger (see wait_for_joint_crossing) and
        # for reading the arm's current shoulder/elbow/wrist before
        # rotating to face a new direction without disturbing them.
        # None until the first /joint_states message arrives.
        self.latest_joint_positions = None
        self.joint_state_sub = node.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10, callback_group=cb_group
        )

        # For solving a single-plane reach (fixed base + wrist, only
        # shoulder/elbow move) via forward kinematics rather than a
        # general 6-DOF IK search - see solve_planar_reach.
        self.compute_fk_client = node.create_client(GetPositionFK, '/compute_fk', callback_group=cb_group)
        self.check_state_validity_client = node.create_client(GetStateValidity, '/check_state_validity', callback_group=cb_group)

        # Dictionary of arm actions, keyed by recipe step 'action' name
        self.handlers = {
            'home': partial(basic.home, self),
            'move_arm': partial(basic.move_arm, self),
            'relative_move': partial(basic.relative_move, self),
            'gripper': partial(basic.gripper, self),
            'pickup': partial(pickup.run, self),
            'dropoff': partial(dropoff.run, self),
            'pour': partial(pour.run, self),
            'thrust': partial(thrust.run, self),
            'push': partial(push.run, self),
            'throw': partial(throw.run, self),
        }

        # Name of the object currently grasped by the gripper, or None.
        # Set by a successful pickup, cleared by a successful dropoff; used
        # to verify 'pour'/'thrust' aren't invoked on nothing, and as a
        # fallback for dropoff's own release-height calculation.
        self.held_object = None

    # Client-side wait_for_future timeouts for the real arm-moving calls
    # below (home/move/relative_move/joint_move) - must exceed
    # HardwareInterfaceClient.ACTION_TIMEOUT_SEC (30.0s), since that's how
    # long the server itself can legitimately block before responding
    # while a real trajectory executes. wait_for_future's own 10.0s
    # default is far too short for this on real hardware (fine on fake
    # hardware, where moves complete near-instantly) - using it here
    # was reporting a false "timed out/no response" failure for any real
    # move that legitimately took longer than 10s, even ones that would
    # have gone on to succeed a few seconds later.
    _ARM_ACTION_TIMEOUT_SEC = 35.0
    # Same idea, matching HardwareInterfaceClient.GRIPPER_TIMEOUT_SEC (10.0s).
    _GRIPPER_ACTION_TIMEOUT_SEC = 15.0

    def wait_for_future(self, future, service_name, timeout_sec=10.0):
        """Safely wait for an async service call future to complete without deadlocking the executor."""
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > timeout_sec:
                self.get_logger().error(f"Timed out waiting for {service_name}")
                return None
            time.sleep(0.01)
        return future.result() if future.done() else None

    def _on_joint_state(self, msg):
        self.latest_joint_positions = dict(zip(msg.name, msg.position))

    def wait_for_joint_crossing(self, joint_name, threshold, starting_value, timeout=5.0, poll_interval=0.005):
        """Block until self.latest_joint_positions[joint_name] crosses
        'threshold' - moving in whichever direction 'starting_value'
        implies (decreasing if threshold < starting_value, increasing
        otherwise) - or 'timeout' elapses without it. Returns True/False
        for whether the crossing was actually observed.

        This is throw's release trigger: tied to the arm's real, live
        joint state, not a computed time.sleep() delay. A timed sleep was
        tried first and found unreliable - the fire-and-forget fling call
        itself was blocking for up to
        HardwareInterfaceClient.FIRE_AND_FORGET_REJECTION_WINDOW_SEC
        before this code could even start timing, which for a short,
        fast fling could consume the whole motion before any release
        logic ran at all (see docs/throw-motion-reference.md). Polling
        the genuinely-async fling's real position sidesteps that
        entirely - it doesn't matter how long the fling call took to
        return (see call_joint_move_service_async), because this checks
        where the arm actually is, not how much time has passed."""
        decreasing = threshold < starting_value
        deadline = time.time() + timeout
        while time.time() < deadline:
            current = self.latest_joint_positions.get(joint_name) if self.latest_joint_positions else None
            if current is not None:
                if decreasing and current <= threshold:
                    return True
                if not decreasing and current >= threshold:
                    return True
            time.sleep(poll_interval)
        return False

    def get_static_object_coords(self, target_name):
        """Return a dict with x,y,z for the object, or None on failure."""
        info = self.get_object_info(target_name)
        if info is None:
            return None
        pos = info['pose']['position']
        return {'x': pos['x'], 'y': pos['y'], 'z': pos['z']}

    def get_object_info(self, target_name):
        """Query the environment mapping node for full object info (pose + shape)."""
        if not self.info_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Get Object Info service not available")
            return None
        req = GetObjectInfo.Request()
        req.object_id = target_name

        future = self.info_client.call_async(req)
        response = self.wait_for_future(future, '/get_object_info')

        if response and response.success:
            pos = response.pose.position
            orient = response.pose.orientation
            shape = response.shape
            return {
                'pose': {
                    'position': {'x': pos.x, 'y': pos.y, 'z': pos.z},
                    'orientation': {'x': orient.x, 'y': orient.y, 'z': orient.z, 'w': orient.w}
                },
                'shape': {
                    'type': shape.type,
                    'dimensions': list(shape.dimensions)
                }
            }
        else:
            self.get_logger().error(f"Failed to get object info for {target_name}: {response.message if response else 'no response'}")
            return None

    def get_relative_movement_vector(self, movement_name):
        if not self.relative_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Get Relative Movement Service not available")
            return None
        req = GetRelativeMovement.Request()
        req.move_id = movement_name

        # Async call + safe wait loop
        future = self.relative_client.call_async(req)
        response = self.wait_for_future(future, '/get_relative_movement')

        if response and response.success:
            return {'x': response.x, 'y': response.y, 'z': response.z}
        else:
            self.get_logger().error(f"Failed to get movement vector for {movement_name}: {response.message if response else 'no response'}")
            return None

    def get_orientation_preset(self, preset_name):
        if not self.orientation_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Get Orientation Preset service not available")
            return None
        req = GetOrientationPreset.Request()
        req.preset_name = preset_name

        future = self.orientation_client.call_async(req)
        response = self.wait_for_future(future, '/get_orientation_preset')

        if response and response.success:
            return {'roll': response.roll, 'pitch': response.pitch, 'yaw': response.yaw}
        else:
            self.get_logger().error(f"Failed to get orientation preset '{preset_name}': {response.message if response else 'no response'}")
            return None

    def resolve_orientation(self, preset_name):
        """Returns (has_orientation, roll, pitch, yaw). If preset_name is
        given but can't be resolved, returns None so the caller can tell
        that apart from 'no orientation requested'."""
        if not preset_name:
            return (False, 0.0, 0.0, 0.0)
        preset = self.get_orientation_preset(preset_name)
        if preset is None:
            return None
        return (True, preset['roll'], preset['pitch'], preset['yaw'])

    def build_motion_params(self, speed):
        """speed is an optional 0.0-1.0 float from a recipe step, used for
        both velocity and acceleration scale. None or not given means
        MotionParams() with its 0.0 defaults, which hardware_interface_client
        treats as 'use the arm's configured default'."""
        params = MotionParams()
        if speed is not None:
            params.velocity_scale = float(speed)
            params.acceleration_scale = float(speed)
        return params

    def find_ik_solution(self, x, y, z, roll, pitch, yaw, seed_joint_positions=None):
        """Check whether a Cartesian pose is actually reachable and
        collision-free via IK (with collision-avoidance on), rather than
        trusting a computed/geometric candidate blindly - this is exactly
        what caught a plausible-looking but actually-in-collision pose
        during testing (see docs/pour-motion-reference.md), a manually
        demonstrated pose that turned out to have never really been
        validated at all. Returns the solved [joint_1..joint_6] positions,
        or None if unreachable/in collision.

        'seed_joint_positions', if given, is used as the IK search's
        starting point instead of the current robot state - KDL (the
        default IK plugin here) is a local numerical solver, so seeding it
        near an already-known-good configuration (e.g. 'push's contact
        pose, when solving for the pose it extends to) reliably converges
        to a nearby solution differing only in the joints that actually
        need to move, rather than jumping to an unrelated configuration
        branch. Seeding from a very different configuration (e.g. home)
        is what caused a real, physically-reachable pose to report
        NO_IK_SOLUTION during testing - see docs/pour-motion-reference.md."""
        if not self.compute_ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Compute IK service not available")
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = BASE_FRAME
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        pose_stamped.pose = pose
        req.ik_request.pose_stamped = pose_stamped

        if seed_joint_positions is not None:
            seed_state = RobotState()
            seed_js = JointState()
            seed_js.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            seed_js.position = [float(p) for p in seed_joint_positions]
            seed_state.joint_state = seed_js
            req.ik_request.robot_state = seed_state

        future = self.compute_ik_client.call_async(req)
        response = self.wait_for_future(future, '/compute_ik')
        if response is None or response.error_code.val != response.error_code.SUCCESS:
            return None

        names = list(response.solution.joint_state.name)
        positions = list(response.solution.joint_state.position)
        return [positions[names.index(f'joint_{i}')] for i in range(1, 7)]

    def verify_grasp_pose(self, x, y, z, roll, pitch, yaw):
        """True/False convenience wrapper around find_ik_solution, for
        callers that only need to know whether a candidate is valid, not
        its actual joint solution (e.g. pickup's grasp_style='side')."""
        return self.find_ik_solution(x, y, z, roll, pitch, yaw) is not None

    def set_collision_allowed(self, object_id, allowed):
        """Temporarily allow (or restore disallowing) collision between
        'object_id' and every robot link, via an explicit update to the
        planning scene's allowed collision matrix (ACM). Needed because
        'push' (unlike every other action) deliberately keeps the gripper
        in sustained contact with the object - a normal collision-aware
        plan would otherwise reject that contact outright, or worse,
        silently find a path that avoids the object entirely (planning
        around it) rather than actually pushing it.

        This must set an *explicit* entry for object_id against every
        already-known link name, not just the ACM's 'default_entry'
        fallback - verified directly: MoveIt auto-populates explicit
        disallow entries for a collision object against nearby links as
        soon as it's added to the scene, and those explicit entries take
        precedence over a blanket default, so setting only the default
        (an earlier version of this method) silently had no effect and
        let a real push fail with INVALID_MOTION_PLAN (see
        docs/push-motion-reference.md).

        Scoped to just this one object, not the whole matrix, so the
        table and every other object are still checked normally in the
        meantime. Returns True/False for whether the scene update itself
        succeeded - callers are responsible for reverting (allowed=False)
        once done, in a 'finally' block, so a mid-push failure doesn't
        leave it permanently collision-exempt."""
        if not self.get_planning_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Get Planning Scene service not available")
            return False
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.get_planning_scene_client.call_async(req)
        response = self.wait_for_future(future, '/get_planning_scene')
        if response is None:
            self.get_logger().error("Failed to fetch current planning scene ACM")
            return False
        current_acm = response.scene.allowed_collision_matrix

        names = list(current_acm.entry_names)
        rows = [list(entry.enabled) for entry in current_acm.entry_values]
        if object_id in names:
            idx = names.index(object_id)
            for i in range(len(names)):
                rows[i][idx] = bool(allowed)
                rows[idx][i] = bool(allowed)
        else:
            names.append(object_id)
            for row in rows:
                row.append(bool(allowed))
            rows.append([bool(allowed)] * len(names))

        new_acm = AllowedCollisionMatrix()
        new_acm.entry_names = names
        new_entry_values = []
        for row in rows:
            entry = AllowedCollisionEntry()
            entry.enabled = row
            new_entry_values.append(entry)
        new_acm.entry_values = new_entry_values

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = new_acm

        if not self.apply_planning_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Apply Planning Scene service not available")
            return False
        apply_req = ApplyPlanningScene.Request()
        apply_req.scene = scene
        apply_future = self.apply_planning_scene_client.call_async(apply_req)
        apply_response = self.wait_for_future(apply_future, '/apply_planning_scene')
        return apply_response is not None and apply_response.success

    def compute_fk(self, joint_positions):
        """Forward kinematics: [joint_1..joint_6] -> tool_frame's (x, y, z)
        in base_link, or None on failure. Used by solve_planar_reach to
        numerically search a 2-DOF reach, rather than relying on a
        general 6-DOF IK search (see that method's docstring for why)."""
        if not self.compute_fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Compute FK service not available")
            return None
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_FRAME
        req.fk_link_names = [TOOL_FRAME]
        state = RobotState()
        js = JointState()
        js.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        js.position = [float(p) for p in joint_positions]
        state.joint_state = js
        req.robot_state = state

        future = self.compute_fk_client.call_async(req)
        response = self.wait_for_future(future, '/compute_fk')
        if response is None or response.error_code.val != response.error_code.SUCCESS:
            return None
        p = response.pose_stamped[0].pose.position
        return (p.x, p.y, p.z)

    def check_joint_state_validity(self, joint_positions):
        """True if [joint_1..joint_6] is a collision-free, valid state,
        via /check_state_validity - a direct check against a known joint
        state, with none of the ambiguity find_ik_solution has (that
        searches for *some* joint state satisfying a Cartesian pose;
        this checks one specific, already-known state)."""
        if not self.check_state_validity_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Check State Validity service not available")
            return False
        req = GetStateValidity.Request()
        req.group_name = 'arm'
        state = RobotState()
        js = JointState()
        js.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        js.position = [float(p) for p in joint_positions]
        state.joint_state = js
        req.robot_state = state

        future = self.check_state_validity_client.call_async(req)
        response = self.wait_for_future(future, '/check_state_validity')
        return response is not None and response.valid

    # Wrist held level/neutral for a single-plane reach (push, and
    # designed to be reused for thrust) - matches what was actually
    # demonstrated (see docs/push-motion-reference.md). Combined with a
    # fixed joint_1 (facing the target), the whole reach stays within one
    # vertical plane - only the shoulder/elbow move.
    _PLANAR_REACH_WRIST = (0.0, 0.0, 0.0)
    # A reasonable default starting guess for solve_planar_reach's Newton
    # search - an arbitrary "bent forward and down" shoulder/elbow shape,
    # not tied to any specific object. A seed near (0, 0) was found to
    # converge to the wrong side (behind the arm) instead of forward, so
    # this needs to already be a genuinely bent posture.
    _PLANAR_REACH_SEED = (math.radians(-20), math.radians(140))

    def solve_planar_reach(self, base_yaw, target_x, target_y, target_z, seed_shoulder=None, seed_elbow=None, iterations=15):
        """Solve for (joint_2, joint_3) reaching (target_x, target_y,
        target_z), with joint_1 fixed at base_yaw and the wrist fixed
        level (_PLANAR_REACH_WRIST) - the whole reach stays in a single
        vertical plane containing that bearing, matching a manually
        demonstrated push (see docs/push-motion-reference.md) and
        designed to generalize to 'thrust' too.

        Uses Newton-Raphson on (radial distance from the base, height)
        via compute_fk, not a general 6-DOF IK search: with only 2
        unknowns and a smooth forward-kinematics function, this converges
        to near-exact precision from a reasonable seed - a general IK
        search was tried first and found unreliable for exactly this kind
        of small planar reach (see docs/push-motion-reference.md for the
        verified failures that ruled it out).

        Returns (joint_2, joint_3, achieved_xyz, position_error), or None
        if compute_fk itself fails. The caller is responsible for
        checking position_error is small enough and the resulting full
        joint state ([base_yaw, joint_2, joint_3, *_PLANAR_REACH_WRIST])
        is actually collision-free (check_joint_state_validity) before
        trusting it - this only solves the geometry, it doesn't verify
        collision-freeness itself."""
        shoulder = seed_shoulder if seed_shoulder is not None else self._PLANAR_REACH_SEED[0]
        elbow = seed_elbow if seed_elbow is not None else self._PLANAR_REACH_SEED[1]
        target_radius = math.hypot(target_x, target_y)
        eps = 1e-3

        def radius_and_height(s, e):
            pos = self.compute_fk([base_yaw, s, e, *self._PLANAR_REACH_WRIST])
            if pos is None:
                return None, None
            return (math.hypot(pos[0], pos[1]), pos[2]), pos

        for _ in range(iterations):
            rz0, _ = radius_and_height(shoulder, elbow)
            if rz0 is None:
                return None
            rz_ds, _ = radius_and_height(shoulder + eps, elbow)
            rz_de, _ = radius_and_height(shoulder, elbow + eps)
            if rz_ds is None or rz_de is None:
                return None

            jac = [
                [(rz_ds[0] - rz0[0]) / eps, (rz_de[0] - rz0[0]) / eps],
                [(rz_ds[1] - rz0[1]) / eps, (rz_de[1] - rz0[1]) / eps],
            ]
            det = jac[0][0] * jac[1][1] - jac[0][1] * jac[1][0]
            if abs(det) < 1e-9:
                break
            f0 = (rz0[0] - target_radius, rz0[1] - target_z)
            shoulder += (-jac[1][1] * f0[0] + jac[0][1] * f0[1]) / det
            elbow += (jac[1][0] * f0[0] - jac[0][0] * f0[1]) / det

        rz_final, achieved = radius_and_height(shoulder, elbow)
        if rz_final is None:
            return None
        error = math.hypot(rz_final[0] - target_radius, rz_final[1] - target_z)
        return shoulder, elbow, achieved, error

    def call_home_service(self, motion_params=None):
        if not self.home_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Home Arm Service not available")
            return None

        req = HomeArm.Request()
        req.motion_params = motion_params if motion_params is not None else MotionParams()
        # Async call + safe wait loop
        future = self.home_client.call_async(req)
        response = self.wait_for_future(future, '/kinova_hardware_client/home_arm', self._ARM_ACTION_TIMEOUT_SEC)

        if response and response.success:
            return {'success': response.success, 'message': response.message}
        else:
            self.get_logger().error(f"Failed to move Home: {response.message if response else 'no response'}")
            return None

    def call_move_service(self, x, y, z, has_orientation=False, roll=0.0, pitch=0.0, yaw=0.0, motion_params=None):
        if not self.move_arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Move Arm Service not available")
            return None
        req = MoveArm.Request()
        req.target_position.x = x
        req.target_position.y = y
        req.target_position.z = z
        req.has_orientation = has_orientation
        req.roll = roll
        req.pitch = pitch
        req.yaw = yaw
        req.motion_params = motion_params if motion_params is not None else MotionParams()

        # Async call + safe wait loop
        future = self.move_arm_client.call_async(req)
        response = self.wait_for_future(future, '/kinova_hardware_client/move_arm', self._ARM_ACTION_TIMEOUT_SEC)

        if response and response.success:
            return {'success': response.success, 'message': response.message}
        else:
            self.get_logger().error(f"Failed to perform Move to:{x},{y},{z}: {response.message if response else 'no response'}")
            return None

    def call_relative_move_service(self, vx, vy, vz, has_orientation=False, roll_delta=0.0, pitch_delta=0.0, yaw_delta=0.0, motion_params=None):
        if not self.relative_move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Relative Move Service not available")
            return None
        req = RelativeMove.Request()
        req.vx = vx
        req.vy = vy
        req.vz = vz
        req.has_orientation = has_orientation
        req.roll_delta = roll_delta
        req.pitch_delta = pitch_delta
        req.yaw_delta = yaw_delta
        req.motion_params = motion_params if motion_params is not None else MotionParams()

        # Async call + safe wait loop
        future = self.relative_move_client.call_async(req)
        response = self.wait_for_future(future, '/kinova_hardware_client/relative_move', self._ARM_ACTION_TIMEOUT_SEC)

        if response and response.success:
            return {'success': response.success, 'message': response.message}
        else:
            self.get_logger().error(f"Failed to perform relative move:{vx},{vy},{vz}: {response.message if response else 'no response'}")
            return None

    def call_move_gripper_service(self, position):
        if not self.move_gripper_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Move Gripper service not available")
            return None
        req = MoveGripper.Request()
        req.position = position
        # Async call + safe wait loop
        future = self.move_gripper_client.call_async(req)
        response = self.wait_for_future(future, '/kinova_hardware_client/move_gripper', self._GRIPPER_ACTION_TIMEOUT_SEC)

        if response and response.success:
            return {'success': response.success, 'message': response.message}
        else:
            self.get_logger().error(f"Failed to Move Gripper to: {position}: {response.message if response else 'no response'}")
            return None

    def call_joint_move_service(self, joint_positions, motion_params=None, wait_for_completion=True, relative=False):
        """Move to a joint-space target - absolute by default, or a delta
        from whatever the current joint state actually is if relative=True
        (e.g. 'pour's tilt: a delta on joint_6 alone, without needing to
        know or recompute the other five joints' current values).

        With wait_for_completion False, this still blocks the caller for
        up to HardwareInterfaceClient.FIRE_AND_FORGET_REJECTION_WINDOW_SEC
        (the server's own bounded wait to catch a fast rejection) before
        returning - fine for a caller that only cares the motion started
        without also needing precise timing of what happens next. A
        caller that needs to react to the motion in real time as it
        happens (e.g. throw's release trigger) should use
        call_joint_move_service_async instead, which doesn't wait for
        anything at all - see that method's docstring for why this
        distinction turned out to matter in practice."""
        if not self.joint_move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Joint Move service not available")
            return None
        req = JointMove.Request()
        req.joint_positions = [float(p) for p in joint_positions]
        req.wait_for_completion = wait_for_completion
        req.relative = relative
        req.motion_params = motion_params if motion_params is not None else MotionParams()

        future = self.joint_move_client.call_async(req)
        response = self.wait_for_future(future, '/kinova_hardware_client/joint_move', self._ARM_ACTION_TIMEOUT_SEC)

        if response and response.success:
            return {'success': response.success, 'message': response.message}
        else:
            self.get_logger().error(f"Failed to move to joint positions {joint_positions}: {response.message if response else 'no response'}")
            return None

    def call_joint_move_service_async(self, joint_positions, motion_params=None):
        """Fire a joint-space move without waiting for any response at
        all - not even call_joint_move_service's own bounded
        fire-and-forget wait. Returns the raw future (which the caller
        can inspect later if it cares about eventual success/failure) or
        None if the service isn't even available to call.

        This exists specifically because that bounded wait
        (HardwareInterfaceClient.FIRE_AND_FORGET_REJECTION_WINDOW_SEC,
        0.5s) was found to break 'throw's release timing: it blocks the
        *client* for up to that long before call_joint_move_service
        returns at all, so for a short, fast fling, the entire motion
        could finish before any release-timing code even started
        running - the object would still be gripped when the arm had
        already stopped (see docs/throw-motion-reference.md). Not
        waiting for any response at all means the caller can start
        reacting to the arm's real, live position (see
        wait_for_joint_crossing) immediately after sending the goal,
        regardless of how long the server takes to eventually reply."""
        if not self.joint_move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Joint Move service not available")
            return None
        req = JointMove.Request()
        req.joint_positions = [float(p) for p in joint_positions]
        req.wait_for_completion = False
        req.relative = False
        req.motion_params = motion_params if motion_params is not None else MotionParams()
        return self.joint_move_client.call_async(req)

    def attach_object(self, obj_id):
        """Remove object from planning scene (allow collision) via the environment mapping node."""
        if not self.attach_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Attach service not available")
            return False
        req = AttachObject.Request()
        req.object_id = obj_id
        future = self.attach_client.call_async(req)
        response = self.wait_for_future(future, '/attach_object')
        if response and response.success:
            self.get_logger().info(f"Attached object '{obj_id}' (removed from scene)")
            return True
        else:
            self.get_logger().error(f"Failed to attach '{obj_id}'")
            return False

    def detach_object(self, obj_id):
        """Add object back to planning scene via the environment mapping node."""
        if not self.detach_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Detach service not available")
            return False
        req = DetachObject.Request()
        req.object_id = obj_id
        future = self.detach_client.call_async(req)
        response = self.wait_for_future(future, '/detach_object')
        if response and response.success:
            self.get_logger().info(f"Detached object '{obj_id}' (added back to scene)")
            return True
        else:
            self.get_logger().error(f"Failed to detach '{obj_id}'")
            return False

    def update_object_pose(self, obj_id, x, y, z, orientation=None):
        if not self.update_pose_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Update Object Pose service not available")
            return False

        req = UpdateObjectPose.Request()
        req.object_id = obj_id
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z

        if orientation:
            # Convert dict to Quaternion message
            q = Quaternion()
            q.x = orientation['x']
            q.y = orientation['y']
            q.z = orientation['z']
            q.w = orientation['w']
            req.pose.orientation = q
        else:
            req.pose.orientation.x = 0.0
            req.pose.orientation.y = 0.0
            req.pose.orientation.z = 0.0
            req.pose.orientation.w = 1.0

        future = self.update_pose_client.call_async(req)
        response = self.wait_for_future(future, '/update_object_pose')
        if response and response.success:
            return True
        else:
            self.get_logger().error(f"Failed to update pose for {obj_id}: {response.message if response else 'no response'}")
            return False

    def reset_environment(self):
        """Reset the environment_mapping_node's objects/obstacles/scene back
        to their configured defaults (undoing any pose drift from earlier
        pickup/dropoff/push/throw actions), and clear locally-tracked
        held-object state to match, without needing a full middleware
        restart. Returns (success, message)."""
        if not self.reset_scene_client.wait_for_service(timeout_sec=5.0):
            return False, "Environment reset service not available"

        future = self.reset_scene_client.call_async(Trigger.Request())
        response = self.wait_for_future(future, '/reset_environment_scene')

        self.held_object = None

        if response and response.success:
            return True, response.message
        return False, response.message if response else "No response from /reset_environment_scene"

    # Above this, a solved planar reach is considered to have failed to
    # converge (meters) - a sanity bound, not a tolerance to plan within.
    _PLANAR_REACH_MAX_ERROR = 0.01

    # Search bounds for the dynamic default push/thrust distance (see
    # _find_max_planar_reach_distance): 0.4m (thrust) / 0.2m (push) were
    # each previously-verified reach limits at their respective typical
    # heights, but only for the specific object position they were
    # measured against - for others they can already be past what's
    # solvable, which is what made a single flat default unreliable. 0.5m
    # is a search ceiling comfortably past either, not a claim anything
    # past it is generally reachable.
    _PLANAR_DISTANCE_SEARCH_MIN = 0.05
    _PLANAR_DISTANCE_SEARCH_MAX = 0.5
    _PLANAR_DISTANCE_SEARCH_ITERATIONS = 10
    # Back off from the furthest distance the search found solvable -
    # right at that edge is numerically marginal (small enough real-world
    # deviations could tip it back into failure), so this trades a little
    # reach for headroom.
    _PLANAR_DISTANCE_SAFETY_MARGIN = 0.03

    def _find_max_planar_reach_distance(self, origin, direction, height, seed_shoulder, seed_elbow, fixed_yaw=None):
        """Binary-search the furthest distance (up to
        _PLANAR_DISTANCE_SEARCH_MAX) in 'direction' from an object's
        original position that a single-plane reach (push's extend, or
        thrust's spin+extend) can actually reach at 'height' - used as
        the default distance instead of a fixed constant, since a flat
        default is sometimes already past what's reachable for a given
        object position/direction. That gap previously only showed up as
        a failed push/thrust after the object had already been approached
        (push) or raised and spun into place (thrust) - this finds it up
        front instead, as pure geometry (compute_fk/collision checks),
        before the arm actually moves.

        fixed_yaw covers the two callers' different geometry:
        - thrust (fixed_yaw=None, the default): spins to face each
          candidate distance's own point, so a new yaw is computed per
          distance and that spin pose's validity is checked too, exactly
          mirroring the real spin+extend steps.
        - push (fixed_yaw=<push's one unchanging base_yaw>): push never
          reorients - the whole action stays in the single plane it faced
          the object in - so every candidate is checked at that same
          fixed yaw, with no separate spin-pose check (there's no spin
          move to check).

        Either way this mirrors exactly what the real extend step does at
        a given distance, so 'reachable per this search' really does mean
        'the real move will reach it'. Returns a distance backed off by
        _PLANAR_DISTANCE_SAFETY_MARGIN from the furthest point found
        solvable, or None if not even the minimum search distance is
        reachable."""
        def feasible(distance):
            offset = resolve_direction_offset(origin['x'], origin['y'], direction, distance)
            if offset is None:
                return False
            x, y = offset
            if fixed_yaw is not None:
                yaw = fixed_yaw
            else:
                yaw = math.atan2(y, x)
                spin_joints = [yaw, seed_shoulder, seed_elbow, *self._PLANAR_REACH_WRIST]
                if not self.check_joint_state_validity(spin_joints):
                    return False
            solved = self.solve_planar_reach(yaw, x, y, height, seed_shoulder=seed_shoulder, seed_elbow=seed_elbow)
            if solved is None or solved[3] > self._PLANAR_REACH_MAX_ERROR:
                return False
            extend_joints = [yaw, solved[0], solved[1], *self._PLANAR_REACH_WRIST]
            return self.check_joint_state_validity(extend_joints)

        if not feasible(self._PLANAR_DISTANCE_SEARCH_MIN):
            return None

        low, high = self._PLANAR_DISTANCE_SEARCH_MIN, self._PLANAR_DISTANCE_SEARCH_MAX
        if feasible(high):
            low = high
        else:
            for _ in range(self._PLANAR_DISTANCE_SEARCH_ITERATIONS):
                mid = (low + high) / 2.0
                if feasible(mid):
                    low = mid
                else:
                    high = mid

        return max(self._PLANAR_DISTANCE_SEARCH_MIN, low - self._PLANAR_DISTANCE_SAFETY_MARGIN)
