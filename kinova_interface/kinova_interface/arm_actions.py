import math
import time

import rclpy
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
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

from kinova_interface.geometry_utils import euler_to_quaternion, quaternion_to_euler


class ArmActions:
    """Owns the hardware/environment service clients and the concrete
    implementation of every named recipe action (home, move_arm,
    relative_move, gripper, pickup, dropoff). Exposes `handlers`, a dict of
    action name -> callable(params) -> bool, for JsonParserNode to dispatch
    recipe steps against without needing to know how each action works."""

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
            'home': self._handle_home,
            'move_arm': self._handle_move_arm,
            'relative_move': self._handle_relative_move,
            'gripper': self._handle_gripper,
            'pickup': self._handle_pickup,
            'dropoff': self._handle_dropoff,
            'pour': self._handle_pour,
            'thrust': self._handle_thrust,
            'push': self._handle_push,
            'throw': self._handle_throw,
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

    def object_half_height(self, shape):
        """Half the object's extent along Z, from its shape type/dimensions,
        used to place one object on top of another from their center poses."""
        stype = shape['type']
        dims = shape['dimensions']
        if stype == SolidPrimitive.BOX:
            return dims[2] / 2.0
        if stype in (SolidPrimitive.CYLINDER, SolidPrimitive.CONE):
            return dims[0] / 2.0
        if stype == SolidPrimitive.SPHERE:
            return dims[0]
        return 0.0

    # A flat, level wrist (not pointing down) - the one part of a side
    # grasp that's genuinely independent of which object it is.
    _SIDE_GRASP_ROLL = math.pi / 2.0
    # Candidate yaw rotations relative to the object's own registered yaw -
    # covers approaching aligned with, or perpendicular to, its own frame,
    # from either side. Which of these is actually correct depends on the
    # gripper's own closing-axis convention, which isn't assumed here -
    # every candidate is verified for real (see verify_grasp_pose) rather
    # than trusted from geometry alone.
    _SIDE_GRASP_YAW_OFFSETS = [0.0, math.pi / 2.0, -math.pi / 2.0, math.pi]
    # A CYLINDER is radially symmetric - there's no "face" to align with the
    # way BOX's 4 offsets (above) align with its own registered frame, every
    # angle around its circumference is an equally valid grasp. 8 evenly
    # spaced absolute yaws (not relative to the object's own yaw, which has
    # no real meaning for a symmetric shape) gives verify_grasp_pose a much
    # wider net to find one the arm can actually reach - important since
    # cylinders (bottles, cups) need this to work reliably for pouring.
    _SIDE_GRASP_CYLINDER_YAWS = [math.radians(a) for a in range(0, 360, 45)]
    # Small position nudges tried only if the object's exact center doesn't
    # verify at any of the yaw offsets above - meters, along each world axis.
    _SIDE_GRASP_POSITION_OFFSETS = [0.02, -0.02, 0.04, -0.04]

    def compute_side_grasp_candidates(self, target_info):
        """Generate candidate flat, side-on grasp poses for a BOX or
        CYLINDER object from its own registered shape and pose - not a
        fixed preset calibrated to one specific object/position (see
        docs/pour-motion-reference.md for why that didn't generalize).
        Each candidate still needs verifying (verify_grasp_pose) before
        being trusted - this generates plausible options, it doesn't
        guarantee any one of them is actually reachable/collision-free.
        Returns a list of (x, y, z, roll, pitch, yaw) tuples, cheapest/most
        likely first; [] if the shape isn't supported."""
        shape = target_info['shape']
        if shape['type'] not in (SolidPrimitive.BOX, SolidPrimitive.CYLINDER):
            self.get_logger().error(f"Side grasp only supports BOX/CYLINDER shapes currently (got shape type {shape['type']})")
            return []

        pos = target_info['pose']['position']

        if shape['type'] == SolidPrimitive.CYLINDER:
            yaw_candidates = self._SIDE_GRASP_CYLINDER_YAWS
        else:
            orient = target_info['pose']['orientation']
            _, _, object_yaw = quaternion_to_euler(orient['x'], orient['y'], orient['z'], orient['w'])
            yaw_candidates = [object_yaw + offset for offset in self._SIDE_GRASP_YAW_OFFSETS]

        candidates = []
        # Pass 1: the object's exact center, at each candidate yaw - the
        # cheapest and most likely to work.
        for yaw in yaw_candidates:
            candidates.append((pos['x'], pos['y'], pos['z'], self._SIDE_GRASP_ROLL, 0.0, yaw))

        # Pass 2: only if none of those verify - small offsets along each
        # world axis, at every candidate yaw again.
        for offset in self._SIDE_GRASP_POSITION_OFFSETS:
            for yaw in yaw_candidates:
                candidates.append((pos['x'] + offset, pos['y'], pos['z'], self._SIDE_GRASP_ROLL, 0.0, yaw))
                candidates.append((pos['x'], pos['y'] + offset, pos['z'], self._SIDE_GRASP_ROLL, 0.0, yaw))

        return candidates

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
        pose_stamped.header.frame_id = 'base_link'
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
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['tool_frame']
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

    # Direction keywords for 'push'/'throw' as an alternative to a named
    # destination, as an angle to rotate the reference bearing by. The
    # workspace frame's origin is the arm's own base, so a held/pushed
    # object's own (x, y) position doubles as the bearing vector from the
    # arm out to it. 'left' is a +90 degree (counter-clockwise, viewed from
    # above) rotation of that bearing, 'right' is -90 degrees - this matches
    # the object's original position, not the arm's own facing, and should
    # be validated against the real arm before relying on it.
    _DIRECTION_ROTATIONS = {
        'forward': 0.0,
        'left': math.pi / 2.0,
        'right': -math.pi / 2.0,
        'backward': math.pi,
    }

    def resolve_direction_offset(self, reference_x, reference_y, direction, distance):
        """Given the point a held/pushed object originally rested at
        (reference_x, reference_y), return an (x, y) point further out
        along that same bearing from the arm's base, rotated by the named
        direction and displaced by 'distance'. Returns None for an
        unrecognised direction."""
        if direction not in self._DIRECTION_ROTATIONS:
            return None

        bearing_len = math.hypot(reference_x, reference_y)
        if bearing_len < 1e-6:
            # No meaningful bearing to rotate (object rests essentially at
            # the arm's base) - fall back to a fixed +X bearing.
            bx, by = 1.0, 0.0
        else:
            bx, by = reference_x / bearing_len, reference_y / bearing_len

        angle = self._DIRECTION_ROTATIONS[direction]
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        dx = bx * cos_a - by * sin_a
        dy = bx * sin_a + by * cos_a

        return reference_x + dx * distance, reference_y + dy * distance

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

    def _handle_home(self, params):
        motion_params = self.build_motion_params(params.get('speed'))
        result = self.call_home_service(motion_params)
        return result is not None and result['success']

    def _handle_move_arm(self, params):
        target_name = params['target']
        coords = self.get_static_object_coords(target_name)
        if not coords:
            return False

        orientation = self.resolve_orientation(params.get('orientation'))
        if orientation is None:
            self.get_logger().error(f"Unknown orientation preset '{params.get('orientation')}'")
            return False
        has_orientation, roll, pitch, yaw = orientation

        motion_params = self.build_motion_params(params.get('speed'))
        result = self.call_move_service(coords['x'], coords['y'], coords['z'], has_orientation, roll, pitch, yaw, motion_params)
        return result is not None and result['success']

    def _handle_relative_move(self, params):
        vector_name = params['vector']
        vector = self.get_relative_movement_vector(vector_name)
        if not vector:
            return False

        orientation = self.resolve_orientation(params.get('orientation'))
        if orientation is None:
            self.get_logger().error(f"Unknown orientation preset '{params.get('orientation')}'")
            return False
        has_orientation, roll_delta, pitch_delta, yaw_delta = orientation

        motion_params = self.build_motion_params(params.get('speed'))
        result = self.call_relative_move_service(vector['x'], vector['y'], vector['z'], has_orientation, roll_delta, pitch_delta, yaw_delta, motion_params)
        return result is not None and result['success']

    def _handle_gripper(self, params):
        gripper = float(params['position'])
        result = self.call_move_gripper_service(gripper)
        return result is not None and result['success']

    def _handle_pickup(self, params):
        """Default behaviour is unchanged: unconstrained orientation at the
        object's registered center (forcing one can make an
        otherwise-reachable approach infeasible for the planner, same
        caveat as 'push').

        'grasp_style': 'side' switches to a flat, side-on grasp instead -
        computed from the object's own shape/pose (BOX or CYLINDER
        currently), not a fixed preset for one specific object, and each candidate
        pose is verified reachable/collision-free (via /compute_ik) before
        being trusted, rather than assumed correct from geometry alone.
        See compute_side_grasp_candidates/verify_grasp_pose and
        docs/pour-motion-reference.md for why that verification step
        matters - a manually-demonstrated pose used earlier turned out to
        have never actually been validated this way.

        'orientation'/'grasp_offset' remain available for a fully manual
        override (an explicit preset name, plus an x/y/z shift off the
        object's center) when 'grasp_style' isn't given."""
        target_name = params['target']
        open_pos = float(params.get('open_position', 0.0))
        close_pos = float(params.get('close_position', 0.8))

        if params.get('grasp_style') == 'side':
            target_info = self.get_object_info(target_name)
            if not target_info:
                return False
            candidates = self.compute_side_grasp_candidates(target_info)
            if not candidates:
                return False
            chosen = next((c for c in candidates if self.verify_grasp_pose(*c)), None)
            if chosen is None:
                self.get_logger().error(f"No valid side-grasp pose found for '{target_name}'")
                return False
            target_x, target_y, target_z, roll, pitch, yaw = chosen
            has_orientation = True
        else:
            coords = self.get_static_object_coords(target_name)
            if not coords:
                return False

            orientation = self.resolve_orientation(params.get('orientation'))
            if orientation is None:
                self.get_logger().error(f"Unknown orientation preset '{params.get('orientation')}' for pickup")
                return False
            has_orientation, roll, pitch, yaw = orientation

            grasp_offset = params.get('grasp_offset') or {}
            target_x = coords['x'] + float(grasp_offset.get('x', 0.0))
            target_y = coords['y'] + float(grasp_offset.get('y', 0.0))
            target_z = coords['z'] + float(grasp_offset.get('z', 0.0))

        # 1. Open gripper before moving
        rg = self.call_move_gripper_service(open_pos)
        if not (rg and rg['success']):
            self.get_logger().error('Failed to open gripper for pickup')
            return False

        # 2. Descend to the chosen approach pose
        r = self.call_move_service(target_x, target_y, target_z, has_orientation, roll, pitch, yaw)
        if not (r and r['success']):
            self.get_logger().error('Failed to move to object position')
            return False

        # 3. Close gripper
        rg = self.call_move_gripper_service(close_pos)
        if not (rg and rg['success']):
            return False

        # 4. Remove object from planning scene (attach)
        if not self.attach_object(target_name):
            self.get_logger().error("Failed to attach object after pickup")
            return False

        self.held_object = target_name
        self.get_logger().info(f"Picked up '{target_name}'")
        return True

    def _handle_dropoff(self, params):
        # Fall back to the object we actually know is held if the recipe
        # step didn't name one - the release-height math below needs the
        # held object's height to avoid releasing into the destination.
        target_name = params.get('target') or self.held_object
        destination_name = params.get('destination')
        open_pos = float(params.get('open_position', 0.0))
        hover_clearance = float(params.get('place_offset', 0.1))
        release_clearance = 0.02

        if not destination_name:
            self.get_logger().error("dropoff action requires 'destination' object name")
            return False

        dest_info = self.get_object_info(destination_name)
        if not dest_info:
            self.get_logger().error(f"Could not resolve destination '{destination_name}'")
            return False

        dest_pos = dest_info['pose']['position']
        dest_top_z = dest_pos['z'] + self.object_half_height(dest_info['shape'])

        target_info = self.get_object_info(target_name) if target_name else None
        target_half_height = self.object_half_height(target_info['shape']) if target_info else 0.0

        # release_z is where the target object's center should end up, resting
        # on top of the destination rather than at the destination's own center
        release_z = dest_top_z + target_half_height
        px, py = dest_pos['x'], dest_pos['y']

        # 1. Move to a hover position above the destination, collision-safe approach
        r = self.call_move_service(px, py, release_z + hover_clearance)
        if not (r and r['success']):
            self.get_logger().error('Failed to move to hover position above destination')
            return False

        # 2. Lower to a small clearance above the release height before opening,
        # so the object isn't dropped from the hover height
        r = self.call_move_service(px, py, release_z + release_clearance)
        if not (r and r['success']):
            self.get_logger().error('Failed to lower to release position')
            return False

        # 3. Open gripper to release
        rg = self.call_move_gripper_service(open_pos)
        if not (rg and rg['success']):
            self.get_logger().error('Failed to open gripper during place')
            return False

        if target_name:
            orient = target_info['pose']['orientation'] if target_info else None
            # Update pose to the actual release position, not the hover offset
            if not self.update_object_pose(target_name, px, py, release_z, orient):
                self.get_logger().error(f"Failed to update pose for {target_name}, but continuing...")

            # 4. Add object back to planning scene (detach)
            self.detach_object(target_name)
            self.get_logger().info(f"Placed '{target_name}' at '{destination_name}'")

        if target_name == self.held_object:
            self.held_object = None
        return True

    # Default tilt: matches the ~135 degree joint_6 delta captured in the
    # manual RViz demo this was built from - see docs/pour-motion-reference.md.
    _POUR_DEFAULT_TILT_ANGLE = math.radians(135)
    _POUR_DEFAULT_LIFT_HEIGHT = 0.14  # meters; demo measured ~0.137m

    def _handle_pour(self, params):
        """Lift a held object and tip it to pour, then return level -
        directly above wherever it currently is by default, or above a
        'destination'/'direction' first if one is given. Assumes 'target'
        is already grasped - fails cleanly rather than guessing if it
        isn't - and, importantly, assumes it was grasped in a known,
        level orientation (e.g. via pickup's 'side_grasp_flat' preset),
        which lift/transit below then preserve exactly rather than assume
        or recompute.

        'destination'/'direction' are optional, unlike push/thrust/throw:
        a pour doesn't inherently need to go anywhere - pouring is the
        point, not the travel - so with neither given this skips the
        horizontal move entirely and pours right where the object already
        was. Only when a destination/direction actually is given does it
        first move to hover above that point before tilting, exactly the
        same as before.

        This replaces the previous design, which applied a relative
        orientation delta on top of whatever arbitrary orientation an
        unconstrained pickup happened to produce - workable in principle,
        but only if the starting orientation is actually known, which it
        wasn't. See docs/pour-motion-reference.md for the manually-driven
        RViz demonstration this sequence is built from, and why.

        The actual tilt is a pure joint-space delta on joint_6 alone (like
        'throw's wind-up/fling), not a Cartesian orientation change -
        deliberately: 'side_grasp_flat' holds the wrist at roughly a 90
        degree roll, and composing a Cartesian relative orientation delta
        from there hits exactly the asin()-based gimbal-lock-adjacent
        coupling that handle_relative_move's own docstring already warns
        about (see hardware_interface_client.py) - which is what produced
        the unreachable target that made the original design fail."""
        target_name = params.get('target')
        if not target_name:
            self.get_logger().error("pour action requires 'target' naming the held object")
            return False
        if target_name != self.held_object:
            self.get_logger().error(f"Cannot pour '{target_name}': held object is '{self.held_object}'")
            return False

        destination_name = params.get('destination')
        direction = params.get('direction')

        target_info = self.get_object_info(target_name)
        if not target_info:
            self.get_logger().error(f"Could not resolve held object '{target_name}' for pour")
            return False
        origin = target_info['pose']['position']

        if destination_name:
            dest_info = self.get_object_info(destination_name)
            if not dest_info:
                self.get_logger().error(f"Could not resolve pour destination '{destination_name}'")
                return False
            release_x, release_y = dest_info['pose']['position']['x'], dest_info['pose']['position']['y']
        elif direction:
            distance = float(params.get('distance', 0.3))
            offset = self.resolve_direction_offset(origin['x'], origin['y'], direction, distance)
            if offset is None:
                self.get_logger().error(f"Unknown pour direction '{direction}'")
                return False
            release_x, release_y = offset
        else:
            # No destination/direction - pour stays exactly where the
            # object already was, so there's nothing to move sideways to.
            release_x, release_y = origin['x'], origin['y']

        motion_params = self.build_motion_params(params.get('speed'))

        # 1. Lift straight up from the grasp height, holding whatever
        # orientation it was grasped in exactly unchanged (a zero-delta
        # relative move - orientation is preserved, never recomputed)
        lift_height = float(params.get('lift_height', self._POUR_DEFAULT_LIFT_HEIGHT))
        r = self.call_relative_move_service(0.0, 0.0, lift_height, True, 0.0, 0.0, 0.0, motion_params)
        if not (r and r['success']):
            self.get_logger().error('Failed to lift for pour')
            return False

        # 2. Move horizontally to hover above the destination, at that same
        # lifted height - orientation still untouched. Skipped entirely
        # (no real move issued) when neither destination nor direction was
        # given, since release_x/release_y then equal origin exactly.
        dx, dy = release_x - origin['x'], release_y - origin['y']
        if dx != 0.0 or dy != 0.0:
            r = self.call_relative_move_service(dx, dy, 0.0, True, 0.0, 0.0, 0.0, motion_params)
            if not (r and r['success']):
                self.get_logger().error('Failed to move above pour destination')
                return False

        # 3. Tilt: a pure joint-space delta on joint_6 alone (see docstring)
        tilt_angle = float(params.get('tilt_angle', self._POUR_DEFAULT_TILT_ANGLE))
        tilt_delta = [0.0, 0.0, 0.0, 0.0, 0.0, tilt_angle]
        r = self.call_joint_move_service(tilt_delta, motion_params=motion_params, relative=True)
        if not (r and r['success']):
            self.get_logger().error('Failed to tilt for pour')
            return False

        # 4. Hold the tilt so contents can pour out
        dwell = float(params.get('duration', 1.5))
        time.sleep(dwell)

        # 5. Rotate back level - the exact negated delta
        untilt_delta = [0.0, 0.0, 0.0, 0.0, 0.0, -tilt_angle]
        r = self.call_joint_move_service(untilt_delta, motion_params=motion_params, relative=True)
        if not (r and r['success']):
            self.get_logger().error('Failed to return to level after pour')
            return False

        if destination_name:
            where = f"toward '{destination_name}'"
        elif direction:
            where = direction
        else:
            where = "in place"
        self.get_logger().info(f"Poured '{target_name}' {where}")
        return True

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
            offset = self.resolve_direction_offset(origin['x'], origin['y'], direction, distance)
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

    def _handle_thrust(self, params):
        """Raise a held object and thrust it toward a destination -
        reuses the exact same single-plane mechanism as 'push' (see
        docs/push-motion-reference.md): joint_1 fixed per stage, wrist
        held level at _PLANAR_REACH_WRIST, only joint_2/joint_3
        (shoulder/elbow) ever change to reach a position.

        Assumes 'target' is already grasped - fails cleanly rather than
        guessing if it isn't - and expects it to have been grasped via
        pickup's grasp_style='side' (the same verified, level side grasp
        'pour' uses), so it starts out level and pointing outward.

        Sequence:
        1. Raise straight up from the grasp position, facing wherever it
           was originally resting (before pickup), to 'lift_height'
           above it. This also resets the wrist to _PLANAR_REACH_WRIST
           regardless of whatever specific orientation pickup's
           grasp_style='side' search happened to land on, so the result
           is level and facing outward the same way every time - not
           dependent on which grasp candidate was chosen.
        2. Spin to face the thrust direction, if it differs from where
           it's currently facing - joint_1 only; joint_2/joint_3/wrist
           stay exactly where the raise left them, so nothing about the
           object's orientation changes, only which way the arm points.
           There's no separate 'spin angle' parameter - an arbitrarily
           large turn (e.g. facing a destination on the opposite side of
           where it started) falls directly out of whichever
           destination/direction is given, the same way push's own
           facing does.
        3. Extend toward the destination/direction at that same height,
           joint_2/joint_3 only - exactly like push's own extend.

        Where to thrust it is either a named 'destination' object, or a
        'direction' ('forward'/'backward'/'left'/'right', relative to
        the object's own original resting bearing from the arm) plus a
        'distance' - both resolved from the object's pre-pickup
        registered position, the same as 'push'."""
        target_name = params.get('target')
        if not target_name:
            self.get_logger().error("thrust action requires 'target' naming the held object")
            return False
        if target_name != self.held_object:
            self.get_logger().error(f"Cannot thrust '{target_name}': held object is '{self.held_object}'")
            return False

        destination_name = params.get('destination')
        direction = params.get('direction')
        if not destination_name and not direction:
            self.get_logger().error("thrust action requires either 'destination' or 'direction'")
            return False

        target_info = self.get_object_info(target_name)
        if not target_info:
            self.get_logger().error(f"Could not resolve held object '{target_name}' for thrust")
            return False
        origin = target_info['pose']['position']

        # Fast by default (unlike push/pour's more careful pace) - a
        # thrust is meant to be a forceful, punchy motion; still
        # overridable via an explicit 'speed' if a slower one is wanted.
        motion_params = self.build_motion_params(params.get('speed', 1.0))
        lift_height = float(params.get('lift_height', self._POUR_DEFAULT_LIFT_HEIGHT))
        raise_z = origin['z'] + lift_height

        # 1. Solve + validate the raise pose - facing wherever the object
        # was originally resting, wrist reset to level regardless of
        # pickup's own grasp orientation. Solved (but not yet executed)
        # before the destination/direction is resolved below, since its
        # shoulder/elbow double as the seed for the dynamic thrust-distance
        # search, and are reused unchanged for the real raise move further
        # down - this is geometry only so far, the arm hasn't moved yet.
        face_yaw = math.atan2(origin['y'], origin['x'])
        raise_solved = self.solve_planar_reach(face_yaw, origin['x'], origin['y'], raise_z)
        if raise_solved is None or raise_solved[3] > self._PLANAR_REACH_MAX_ERROR:
            self.get_logger().error(f"Could not solve a planar reach to raise '{target_name}'")
            return False
        raise_shoulder, raise_elbow, _, _ = raise_solved
        raise_joints = [face_yaw, raise_shoulder, raise_elbow, *self._PLANAR_REACH_WRIST]
        if not self.check_joint_state_validity(raise_joints):
            self.get_logger().error(f"Raised pose for '{target_name}' is in collision")
            return False

        if destination_name:
            dest_info = self.get_object_info(destination_name)
            if not dest_info:
                self.get_logger().error(f"Could not resolve thrust destination '{destination_name}'")
                return False
            release_x, release_y = dest_info['pose']['position']['x'], dest_info['pose']['position']['y']
        else:
            requested_distance = params.get('distance')
            if requested_distance is not None:
                distance = float(requested_distance)
            else:
                # No explicit distance - find how far this specific object
                # position/direction/lift_height combination can actually
                # reach, rather than assuming a fixed default is reachable
                # (see _find_max_planar_reach_distance).
                distance = self._find_max_planar_reach_distance(origin, direction, raise_z, raise_shoulder, raise_elbow)
                if distance is None:
                    self.get_logger().error(
                        f"No reachable thrust distance found for '{target_name}' in direction '{direction}'"
                    )
                    return False
            offset = self.resolve_direction_offset(origin['x'], origin['y'], direction, distance)
            if offset is None:
                self.get_logger().error(f"Unknown thrust direction '{direction}'")
                return False
            release_x, release_y = offset

        r = self.call_joint_move_service(raise_joints, motion_params=motion_params)
        if not (r and r['success']):
            self.get_logger().error(f"Failed to raise '{target_name}'")
            return False

        # 2. Spin to face the thrust direction - joint_1 only,
        # shoulder/elbow held exactly where the raise left them
        thrust_yaw = math.atan2(release_y, release_x)
        spin_joints = [thrust_yaw, raise_shoulder, raise_elbow, *self._PLANAR_REACH_WRIST]
        if not self.check_joint_state_validity(spin_joints):
            self.get_logger().error(f"Spin pose for '{target_name}' is in collision")
            return False

        r = self.call_joint_move_service(spin_joints, motion_params=motion_params)
        if not (r and r['success']):
            self.get_logger().error(f"Failed to spin to face the thrust direction for '{target_name}'")
            return False

        # 3. Extend toward the destination - same height, seeded at the
        # spin position so it stays a small, local adjustment
        extend_solved = self.solve_planar_reach(
            thrust_yaw, release_x, release_y, raise_z,
            seed_shoulder=raise_shoulder, seed_elbow=raise_elbow
        )
        if extend_solved is None or extend_solved[3] > self._PLANAR_REACH_MAX_ERROR:
            self.get_logger().error(f"Could not solve a planar reach to thrust '{target_name}' to its destination")
            return False
        extend_shoulder, extend_elbow, _, _ = extend_solved
        extend_joints = [thrust_yaw, extend_shoulder, extend_elbow, *self._PLANAR_REACH_WRIST]
        if not self.check_joint_state_validity(extend_joints):
            self.get_logger().error(f"Thrust end pose for '{target_name}' is in collision")
            return False

        r = self.call_joint_move_service(extend_joints, motion_params=motion_params)
        if not (r and r['success']):
            self.get_logger().error(f"Failed to thrust '{target_name}' to its destination")
            return False

        where = f"toward '{destination_name}'" if destination_name else direction
        self.get_logger().info(f"Thrust '{target_name}' {where}")
        return True

    def _handle_push(self, params):
        """Slide an object to a destination by sustained contact, without
        ever grasping or lifting it - captured from a manual RViz
        demonstration (see docs/push-motion-reference.md): face the
        object (joint_1 only), then move only the shoulder/elbow
        (joint_2/joint_3) - the wrist and base stay fixed the whole time,
        so the entire push happens in a single vertical plane, not a
        general 6-DOF reach.

        Both the contact pose and the extended end pose are solved with
        solve_planar_reach (forward-kinematics-based, not IK) and checked
        collision-free with check_joint_state_validity *before* any real
        motion happens - unlike an earlier version of this, which
        searched a handful of differently-rotated 6-DOF grasp-style
        candidates via general IK and had to actually attempt (and
        sometimes retreat from) each one in turn. That approach worked
        but visibly produced arbitrary, sideways-looking approaches whose
        IK solutions happened to be reachable in some direction unrelated
        to the object's own bearing; this one only ever considers the
        single, deliberate plane facing the object, matching the manual
        demonstration this was built from (see
        docs/push-motion-reference.md for the full history, including why
        a general IK/6-DOF search and a plan-only pre-check were both
        tried and ruled out first).

        Push only ever extends an object further from the arm's own base
        - it's not designed to drag one back in.

        Sustained contact is deliberate here (unlike every other action),
        so a normal collision-aware plan would otherwise reject it, or
        silently route around the object instead of actually pushing it -
        set_collision_allowed temporarily exempts just this one object for
        the duration, and is always reverted in a 'finally' block even if
        the push fails partway through.

        Where to push it is either a named 'destination' object, or a
        'direction' ('forward'/'backward'/'left'/'right', relative to the
        object's own original bearing from the arm) plus a 'distance'."""
        target_name = params.get('target')
        destination_name = params.get('destination')
        direction = params.get('direction')
        if not target_name:
            self.get_logger().error("push action requires 'target'")
            return False
        if not destination_name and not direction:
            self.get_logger().error("push action requires either 'destination' or 'direction'")
            return False

        target_info = self.get_object_info(target_name)
        if not target_info:
            self.get_logger().error(f"Could not resolve push target '{target_name}'")
            return False
        origin = target_info['pose']['position']

        # Face the object - joint_1 fixed for the whole push, the single
        # plane every subsequent move (including the dynamic distance
        # search below) stays within.
        base_yaw = math.atan2(origin['y'], origin['x'])

        # Contact with the target object itself is the entire point of a
        # push, not a collision to avoid - permitted up front, before the
        # validity checks below, not just before the real motion. A
        # bigger object (e.g. 'box', ~107x75mm, vs. push_block's small
        # ~60x60mm) genuinely needs the gripper to overlap it to reach its
        # exact registered center, which the pre-check would otherwise
        # (correctly) reject - verified directly that the only contact at
        # that pose is (gripper, target object), nothing else (see
        # docs/push-motion-reference.md), so exempting just this object
        # doesn't hide a real problem with anything else in the scene.
        # This intentionally doesn't try to approach a bigger object's
        # near face instead - see docs/push-motion-reference.md for why
        # that's a known, accepted limitation for now: it may push a
        # larger object slightly off from how a human would naturally
        # grip it, favoring keeping the single-plane motion simple.
        if not self.set_collision_allowed(target_name, True):
            self.get_logger().error(f"Failed to allow contact with '{target_name}' for push")
            return False

        try:
            contact = self.solve_planar_reach(base_yaw, origin['x'], origin['y'], origin['z'])
            if contact is None or contact[3] > self._PLANAR_REACH_MAX_ERROR:
                self.get_logger().error(f"Could not solve a planar reach to '{target_name}'")
                return False
            contact_shoulder, contact_elbow, _, _ = contact
            contact_joints = [base_yaw, contact_shoulder, contact_elbow, *self._PLANAR_REACH_WRIST]
            if not self.check_joint_state_validity(contact_joints):
                self.get_logger().error(f"Push contact pose for '{target_name}' is in collision")
                return False

            if destination_name:
                dest_info = self.get_object_info(destination_name)
                if not dest_info:
                    self.get_logger().error(f"Could not resolve push destination '{destination_name}'")
                    return False
                release_x, release_y = dest_info['pose']['position']['x'], dest_info['pose']['position']['y']
            else:
                requested_distance = params.get('distance')
                if requested_distance is not None:
                    distance = float(requested_distance)
                else:
                    # No explicit distance - find how far this specific
                    # object position/direction combination can actually
                    # reach, rather than assuming a fixed default is
                    # reachable (see _find_max_planar_reach_distance).
                    # Push never reorients (fixed_yaw=base_yaw) - unlike
                    # thrust, there's no separate spin step to search over.
                    distance = self._find_max_planar_reach_distance(
                        origin, direction, origin['z'], contact_shoulder, contact_elbow, fixed_yaw=base_yaw
                    )
                    if distance is None:
                        self.get_logger().error(
                            f"No reachable push distance found for '{target_name}' in direction '{direction}'"
                        )
                        return False
                offset = self.resolve_direction_offset(origin['x'], origin['y'], direction, distance)
                if offset is None:
                    self.get_logger().error(f"Unknown push direction '{direction}'")
                    return False
                release_x, release_y = offset

            # Same height, same plane, seeded at the contact solution so
            # the extend stays a small, local adjustment rather than
            # jumping to an unrelated configuration.
            extend = self.solve_planar_reach(
                base_yaw, release_x, release_y, origin['z'],
                seed_shoulder=contact_shoulder, seed_elbow=contact_elbow
            )
            if extend is None or extend[3] > self._PLANAR_REACH_MAX_ERROR:
                self.get_logger().error(f"Could not solve a planar reach to the push destination for '{target_name}'")
                return False
            extend_shoulder, extend_elbow, _, _ = extend
            extend_joints = [base_yaw, extend_shoulder, extend_elbow, *self._PLANAR_REACH_WRIST]
            if not self.check_joint_state_validity(extend_joints):
                self.get_logger().error(f"Push end pose for '{target_name}' is in collision")
                return False

            motion_params = self.build_motion_params(params.get('speed'))
            close_pos = float(params.get('close_position', 0.75))

            # 1. Open the gripper before approaching
            rg = self.call_move_gripper_service(0.0)
            if not (rg and rg['success']):
                self.get_logger().error('Failed to open gripper before push approach')
                return False

            # 2. Move to the contact pose
            r = self.call_joint_move_service(contact_joints, motion_params=motion_params)
            if not (r and r['success']):
                self.get_logger().error('Failed to approach push target')
                return False

            # 3. Close the gripper onto it
            rg = self.call_move_gripper_service(close_pos)
            if not (rg and rg['success']):
                self.get_logger().error('Failed to set gripper for push')
                return False

            # 4. Extend - shoulder/elbow only, same plane
            r = self.call_joint_move_service(extend_joints, motion_params=motion_params)
            if not (r and r['success']):
                self.get_logger().error('Failed to push to destination')
                return False
        finally:
            self.set_collision_allowed(target_name, False)

        # It moved by contact, not attachment - update its known position.
        orient = target_info['pose']['orientation']
        if not self.update_object_pose(target_name, release_x, release_y, origin['z'], orient):
            self.get_logger().error(f"Failed to update pose for {target_name}, but continuing...")

        where = f"to '{destination_name}'" if destination_name else f"{direction}"
        self.get_logger().info(f"Pushed '{target_name}' {where}")
        return True

    # A captured, fixed joint_2..6 shape (see docs/throw-motion-reference.md)
    # for a real, manually-demonstrated wind-up - not derived from home by
    # a wind-up angle like an earlier version of this; joint_1 (base
    # facing) is the only thing that varies per throw.
    _THROW_WINDUP_POSE = [math.radians(v) for v in [-22.84, 56.01, 80.71, 50.6, 0.0]]
    # The fling's own end pose - a single continuous swing straight from
    # the wind-up to here, joint_1 fixed the whole way.
    _THROW_FLING_POSE = [math.radians(v) for v in [32.63, -34.26, 80.71, -63.25, 0.0]]
    # joint_5 barely moves during the early part of the fling, then swings
    # hard right at the release moment - captured directly from the demo,
    # this is where the gripper should open.
    _THROW_RELEASE_JOINT5 = math.radians(-42.7)

    def _handle_throw(self, params):
        """A genuine joint-space throw, captured from a manual RViz
        demonstration (see docs/throw-motion-reference.md): rotate to
        face the throw direction (joint_1 only, current shoulder/elbow/
        wrist held exactly where pickup left them), move straight to a
        captured wind-up shape (_THROW_WINDUP_POSE), then one continuous
        fling straight to a captured end pose (_THROW_FLING_POSE) -
        releasing the gripper the instant joint_5 crosses
        _THROW_RELEASE_JOINT5, not after a fixed delay or once the arm
        has stopped.

        Assumes 'target' is already grasped - fails cleanly rather than
        guessing if it isn't.

        The release is a closed-loop trigger on the arm's real, live
        joint_5 position (wait_for_joint_crossing), not a computed
        time.sleep(). A timed sleep - even one scaled to an estimated
        fling duration - was tried first and found unreliable: the
        fire-and-forget fling call itself blocked the client for up to
        HardwareInterfaceClient.FIRE_AND_FORGET_REJECTION_WINDOW_SEC
        (0.5s) before any release-timing code could even start running,
        which for a short, fast fling could consume the entire motion -
        the object would still be gripped once the arm had already
        stopped. call_joint_move_service_async (no wait at all, not even
        that bounded one) plus polling the real joint state fixes this by
        not depending on timing at all - see docs/throw-motion-reference.md
        for the full diagnosis.

        Because release timing is now driven by the arm's real position
        rather than a fixed delay, the landing position recorded
        afterward is still a rough approximation - it isn't computing
        where the object will actually land physically, just recording
        the intended target."""
        target_name = params.get('target')
        destination_name = params.get('destination')
        direction = params.get('direction')
        if not target_name:
            self.get_logger().error("throw action requires 'target' naming the held object")
            return False
        if target_name != self.held_object:
            self.get_logger().error(f"Cannot throw '{target_name}': held object is '{self.held_object}'")
            return False
        if not destination_name and not direction:
            self.get_logger().error("throw action requires either 'destination' or 'direction'")
            return False

        target_info = self.get_object_info(target_name)
        if not target_info:
            self.get_logger().error(f"Could not resolve held object '{target_name}' for throw")
            return False
        origin = target_info['pose']['position']

        if destination_name:
            dest_info = self.get_object_info(destination_name)
            if not dest_info:
                self.get_logger().error(f"Could not resolve throw destination '{destination_name}'")
                return False
            release_x, release_y = dest_info['pose']['position']['x'], dest_info['pose']['position']['y']
        else:
            distance = float(params.get('distance', 0.3))
            offset = self.resolve_direction_offset(origin['x'], origin['y'], direction, distance)
            if offset is None:
                self.get_logger().error(f"Unknown throw direction '{direction}'")
                return False
            release_x, release_y = offset

        base_yaw = math.atan2(release_y, release_x)
        motion_params = self.build_motion_params(params.get('speed'))

        # 1. Rotate to face the throw direction - joint_1 only, current
        # shoulder/elbow/wrist (wherever pickup left them) held exactly
        if self.latest_joint_positions is None:
            self.get_logger().error("No joint state available to rotate for throw")
            return False
        try:
            current_arm_joints = [self.latest_joint_positions[f'joint_{i}'] for i in range(2, 7)]
        except KeyError as e:
            self.get_logger().error(f"Missing joint {e} in latest joint state")
            return False
        rotate_joints = [base_yaw, *current_arm_joints]
        r = self.call_joint_move_service(rotate_joints, motion_params=motion_params)
        if not (r and r['success']):
            self.get_logger().error(f"Failed to rotate to face the throw direction for '{target_name}'")
            return False

        # 2. Wind-up - go straight to the captured wind-up shape
        windup_joints = [base_yaw, *self._THROW_WINDUP_POSE]
        r = self.call_joint_move_service(windup_joints, motion_params=motion_params)
        if not (r and r['success']):
            self.get_logger().error(f"Failed to wind up for throw of '{target_name}'")
            return False

        # 3. Fling - one continuous swing straight to the end pose, fired
        # without waiting for any response at all (see
        # call_joint_move_service_async's docstring for why)
        fling_joints = [base_yaw, *self._THROW_FLING_POSE]
        fling_motion = self.build_motion_params(params.get('speed', 1.0))
        fling_future = self.call_joint_move_service_async(fling_joints, motion_params=fling_motion)
        if fling_future is None:
            self.get_logger().error(f"Failed to start throw fling for '{target_name}'")
            return False

        # 4. Release the instant joint_5 crosses the captured release
        # point - a closed-loop trigger on the arm's real position
        windup_joint5 = self._THROW_WINDUP_POSE[3]
        crossed = self.wait_for_joint_crossing('joint_5', self._THROW_RELEASE_JOINT5, windup_joint5)
        if not crossed:
            self.get_logger().error(f"Timed out waiting for the release point during throw of '{target_name}'")
            return False

        open_pos = float(params.get('open_position', 0.0))
        rg = self.call_move_gripper_service(open_pos)
        if not (rg and rg['success']):
            self.get_logger().error('Failed to release gripper during throw')
            return False

        self.detach_object(target_name)
        self.update_object_pose(target_name, release_x, release_y, origin['z'], None)
        where = f"toward '{destination_name}'" if destination_name else direction
        self.get_logger().info(f"Threw '{target_name}' {where}")

        if target_name == self.held_object:
            self.held_object = None
        return True
