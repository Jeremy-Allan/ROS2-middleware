import math
import time

import rclpy
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetPositionIK

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

    def wait_for_future(self, future, service_name, timeout_sec=10.0):
        """Safely wait for an async service call future to complete without deadlocking the executor."""
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > timeout_sec:
                self.get_logger().error(f"Timed out waiting for {service_name}")
                return None
            time.sleep(0.01)
        return future.result() if future.done() else None

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

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Same conversion as hardware_interface_client.py/environment_mapping_node.py,
        # kept local rather than shared - both of those already duplicate
        # this same small helper, not introducing a new pattern here.
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def quaternion_to_euler(self, x, y, z, w):
        # Same conversion as hardware_interface_client.py, kept local for
        # the same reason as euler_to_quaternion above.
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

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
    # Small position nudges tried only if the object's exact center doesn't
    # verify at any of the yaw offsets above - meters, along each world axis.
    _SIDE_GRASP_POSITION_OFFSETS = [0.02, -0.02, 0.04, -0.04]

    def compute_side_grasp_candidates(self, target_info):
        """Generate candidate flat, side-on grasp poses for a BOX-shaped
        object from its own registered shape and pose - not a fixed preset
        calibrated to one specific object/position (see
        docs/pour-motion-reference.md for why that didn't generalize).
        Each candidate still needs verifying (verify_grasp_pose) before
        being trusted - this generates plausible options, it doesn't
        guarantee any one of them is actually reachable/collision-free.
        Returns a list of (x, y, z, roll, pitch, yaw) tuples, cheapest/most
        likely first; [] if the shape isn't supported."""
        shape = target_info['shape']
        if shape['type'] != SolidPrimitive.BOX:
            self.get_logger().error(f"Side grasp only supports BOX shapes currently (got shape type {shape['type']})")
            return []

        pos = target_info['pose']['position']
        orient = target_info['pose']['orientation']
        _, _, object_yaw = self.quaternion_to_euler(orient['x'], orient['y'], orient['z'], orient['w'])

        candidates = []
        # Pass 1: the object's exact center, at each candidate yaw - the
        # cheapest and most likely to work.
        for yaw_offset in self._SIDE_GRASP_YAW_OFFSETS:
            candidates.append((pos['x'], pos['y'], pos['z'], self._SIDE_GRASP_ROLL, 0.0, object_yaw + yaw_offset))

        # Pass 2: only if none of those verify - small offsets along each
        # world axis, at every candidate yaw again.
        for offset in self._SIDE_GRASP_POSITION_OFFSETS:
            for yaw_offset in self._SIDE_GRASP_YAW_OFFSETS:
                candidates.append((pos['x'] + offset, pos['y'], pos['z'], self._SIDE_GRASP_ROLL, 0.0, object_yaw + yaw_offset))
                candidates.append((pos['x'], pos['y'] + offset, pos['z'], self._SIDE_GRASP_ROLL, 0.0, object_yaw + yaw_offset))

        return candidates

    def verify_grasp_pose(self, x, y, z, roll, pitch, yaw):
        """Check whether a Cartesian pose is actually reachable and
        collision-free via IK (with collision-avoidance on), rather than
        trusting a computed/geometric candidate blindly. This is exactly
        what caught a plausible-looking but actually-in-collision pose
        during testing (see docs/pour-motion-reference.md) - a manually
        demonstrated pose that turned out to have never really been
        validated at all."""
        if not self.compute_ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Compute IK service not available")
            return False

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        pose_stamped.pose = pose
        req.ik_request.pose_stamped = pose_stamped

        future = self.compute_ik_client.call_async(req)
        response = self.wait_for_future(future, '/compute_ik')
        if response is None:
            return False
        return response.error_code.val == response.error_code.SUCCESS

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
        response = self.wait_for_future(future, '/kinova_hardware_client/home_arm')

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
        response = self.wait_for_future(future, '/kinova_hardware_client/move_arm')

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
        response = self.wait_for_future(future, '/kinova_hardware_client/relative_move')

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
        response = self.wait_for_future(future, '/kinova_hardware_client/move_gripper')

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

        With wait_for_completion False, this returns as soon as the goal is
        accepted rather than once the motion finishes - the caller (e.g.
        'throw's fling) is then responsible for whatever timing it needs,
        and won't know whether the motion itself ultimately succeeded, only
        that it started."""
        if not self.joint_move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Joint Move service not available")
            return None
        req = JointMove.Request()
        req.joint_positions = [float(p) for p in joint_positions]
        req.wait_for_completion = wait_for_completion
        req.relative = relative
        req.motion_params = motion_params if motion_params is not None else MotionParams()

        future = self.joint_move_client.call_async(req)
        response = self.wait_for_future(future, '/kinova_hardware_client/joint_move')

        if response and response.success:
            return {'success': response.success, 'message': response.message}
        else:
            self.get_logger().error(f"Failed to move to joint positions {joint_positions}: {response.message if response else 'no response'}")
            return None

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
        computed from the object's own shape/pose (BOX only currently),
        not a fixed preset for one specific object, and each candidate
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
        """Carry a held object above a destination and tip it to pour, then
        return level. Assumes 'target' is already grasped - fails cleanly
        rather than guessing if it isn't - and, importantly, assumes it was
        grasped in a known, level orientation (e.g. via pickup's
        'side_grasp_flat' preset), which lift/transit below then preserve
        exactly rather than assume or recompute.

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
        if not destination_name and not direction:
            self.get_logger().error("pour action requires either 'destination' or 'direction'")
            return False

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
        else:
            distance = float(params.get('distance', 0.3))
            offset = self.resolve_direction_offset(origin['x'], origin['y'], direction, distance)
            if offset is None:
                self.get_logger().error(f"Unknown pour direction '{direction}'")
                return False
            release_x, release_y = offset

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
        # lifted height - orientation still untouched
        dx, dy = release_x - origin['x'], release_y - origin['y']
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

        self.get_logger().info(f"Poured '{target_name}' toward '{destination_name}'" if destination_name else f"Poured '{target_name}' {direction}")
        return True

    def _handle_thrust(self, params):
        """Level the held object horizontally, then thrust it forward.
        Assumes the object named by 'target' is already grasped - fails
        cleanly rather than guessing if it isn't."""
        target_name = params.get('target')
        if not target_name:
            self.get_logger().error("thrust action requires 'target' naming the held object")
            return False
        if target_name != self.held_object:
            self.get_logger().error(f"Cannot thrust '{target_name}': held object is '{self.held_object}'")
            return False

        orientation_name = params.get('orientation', 'facing_forward')
        orientation = self.resolve_orientation(orientation_name)
        if orientation is None:
            self.get_logger().error(f"Unknown orientation preset '{orientation_name}' for thrust")
            return False
        has_orientation, roll, pitch, yaw = orientation

        motion_params = self.build_motion_params(params.get('speed', 0.8))

        # 1. Level the held object horizontally before thrusting
        r = self.call_relative_move_service(0.0, 0.0, 0.0, has_orientation, roll, pitch, yaw, motion_params)
        if not (r and r['success']):
            self.get_logger().error('Failed to level for thrust')
            return False

        vector_name = params.get('vector', 'thrust_forward')
        vector = self.get_relative_movement_vector(vector_name)
        if not vector:
            self.get_logger().error(f"Unknown movement vector '{vector_name}' for thrust")
            return False

        # 2. Thrust forward
        r = self.call_relative_move_service(vector['x'], vector['y'], vector['z'], motion_params=motion_params)
        if not (r and r['success']):
            self.get_logger().error('Failed to thrust forward')
            return False

        self.get_logger().info(f"Thrust '{target_name}' forward")
        return True

    def _handle_push(self, params):
        """Slide an object to a destination by contact, without ever
        grasping or lifting it - approaches the object at its resting
        height (plus an optional 'height_offset') with the gripper open,
        partially closes it to act as a flat pusher once already in
        position, then slides across to the destination at that same
        height. Optionally holds a named 'orientation' throughout for a
        level, consistent pushing face - not forced by default, since
        constraining orientation can make an otherwise-reachable approach
        point infeasible for the planner (see the note below).

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

        tx, ty = target_info['pose']['position']['x'], target_info['pose']['position']['y']

        if destination_name:
            dest_info = self.get_object_info(destination_name)
            if not dest_info:
                self.get_logger().error(f"Could not resolve push destination '{destination_name}'")
                return False
            dx, dy = dest_info['pose']['position']['x'], dest_info['pose']['position']['y']
        else:
            distance = float(params.get('distance', 0.2))
            offset = self.resolve_direction_offset(tx, ty, direction, distance)
            if offset is None:
                self.get_logger().error(f"Unknown push direction '{direction}'")
                return False
            dx, dy = offset

        # No default orientation - forcing one removes an entire degree of
        # freedom from the planner, and testing showed 'facing_forward'
        # specifically is not reachable at some real approach points near
        # the table (pickup succeeds at the same points precisely because
        # it leaves orientation unconstrained). Only apply one if the
        # caller explicitly asks for it.
        orientation = self.resolve_orientation(params.get('orientation'))
        if orientation is None:
            self.get_logger().error(f"Unknown orientation preset '{params.get('orientation')}' for push")
            return False
        has_orientation, roll, pitch, yaw = orientation

        close_pos = float(params.get('close_position', 0.5))
        motion_params = self.build_motion_params(params.get('speed'))
        # height_offset defaults to 0.0 (unchanged from before) rather than
        # a lower value - this exact height, close to the real table
        # surface, is what caused push's collision failure during testing;
        # tune it down incrementally on hardware rather than guess a new
        # default blind.
        push_z = target_info['pose']['position']['z'] + float(params.get('height_offset', 0.0))

        # 1. Open the gripper before approaching - closing it first risks
        # the fingers colliding with the table at this low, near-surface
        # height (this broke push during testing: a half-closed gripper
        # made an otherwise-reachable goal pose collide with the table)
        rg = self.call_move_gripper_service(0.0)
        if not (rg and rg['success']):
            self.get_logger().error('Failed to open gripper before push approach')
            return False

        # 2. Move to the object at its resting height, holding the given
        # orientation if one was requested (unconstrained by default)
        r = self.call_move_service(tx, ty, push_z, has_orientation, roll, pitch, yaw, motion_params)
        if not (r and r['success']):
            self.get_logger().error('Failed to approach push target')
            return False

        # 3. Now in position - partially close the gripper to act as a
        # flat pushing surface
        rg = self.call_move_gripper_service(close_pos)
        if not (rg and rg['success']):
            self.get_logger().error('Failed to set gripper for push')
            return False

        # 4. Slide it to the destination, staying at the same height and
        # orientation - it's pushed by contact the whole way, never
        # grasped or lifted
        r = self.call_move_service(dx, dy, push_z, has_orientation, roll, pitch, yaw, motion_params)
        if not (r and r['success']):
            self.get_logger().error('Failed to push to destination')
            return False

        # 5. It moved by contact, not attachment - update its known position.
        # Recorded at its real resting height, not push_z (which may include
        # 'height_offset' - an approach-height tweak, not the object's
        # actual height off the table).
        orient = target_info['pose']['orientation']
        if not self.update_object_pose(target_name, dx, dy, target_info['pose']['position']['z'], orient):
            self.get_logger().error(f"Failed to update pose for {target_name}, but continuing...")

        where = f"to '{destination_name}'" if destination_name else f"{direction}"
        self.get_logger().info(f"Pushed '{target_name}' {where}")
        return True

    # Mirrors HardwareInterfaceClient.HOME_JOINT_POSITIONS - duplicated here
    # rather than shared across nodes/processes, the same way small
    # per-node constants/helpers already are elsewhere in this codebase
    # (e.g. the quaternion math duplicated in environment_mapping_node.py
    # and hardware_interface_client.py). Update both if home's pose changes.
    _HOME_JOINT_POSITIONS = [0.0, 0.0, 1.5708, 1.5708, 1.5708, 0.0]
    _SHOULDER_JOINT_INDEX = 1  # joint_2
    _ELBOW_JOINT_INDEX = 2  # joint_3

    # From kortex_description's gen3_lite_macro.xacro joint limits: used
    # only to estimate roughly how long the fling will take, to time the
    # mid-swing release - not an exact figure, real trajectories ramp
    # velocity up/down rather than moving at a constant rate throughout.
    _JOINT_MAX_VELOCITY_RAD_S = 0.5

    def _handle_throw(self, params):
        """A genuine joint-space throw: face the throw direction from
        home's pose (rotating only joint_1, the base), wind the elbow
        (joint_3) back past facing the opposite way, then fling it forward
        fast back to the faced pose - releasing the gripper mid-swing
        rather than waiting for the fling to finish. joint_2 (the shoulder)
        rocks back and forward in step with the elbow, to exaggerate the
        motion. Assumes the object named by 'target' is already grasped -
        fails cleanly rather than guessing if it isn't.

        The whole swing happens in a single vertical plane (only joint_1,
        joint_2, and joint_3 move), aimed by 'destination' or 'direction'
        the same way as other actions, but the actual distance thrown is
        governed by 'wind_up_angle'/'fling_angle'/'speed', not by
        'distance' alone - those just decide which way the arm faces
        before swinging. The fling is fired without waiting for it to
        finish; a rejection arriving within about half a second is still
        caught and fails the whole action before the gripper ever opens
        (see HardwareInterfaceClient.handle_joint_move), but the object
        leaves the gripper mid-swing rather than at a controlled position,
        so the landing position recorded afterward is a rough
        approximation at best."""
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

        # Face the throw direction: home's pose, rotated at the base
        # (joint_1) to point along the release bearing from the arm's own
        # origin - the single plane the whole swing happens in.
        base_yaw = math.atan2(release_y, release_x)
        face_pose = list(self._HOME_JOINT_POSITIONS)
        face_pose[0] = base_yaw

        face_motion = self.build_motion_params(0.6)
        r = self.call_joint_move_service(face_pose, motion_params=face_motion)
        if not (r and r['success']):
            self.get_logger().error('Failed to face throw direction')
            return False

        # Wind-up: rotate the elbow back past facing the opposite way from
        # the throw direction (225 degrees past the faced pose by default -
        # 180 to face backward, plus 45 further), and rock the shoulder
        # back too, like cocking the whole arm before a pitch
        wind_up_angle = float(params.get('wind_up_angle', math.radians(225)))
        shoulder_rock_angle = float(params.get('joint_2_rock_angle', math.radians(15)))
        windup_pose = list(face_pose)
        windup_pose[self._ELBOW_JOINT_INDEX] -= wind_up_angle
        windup_pose[self._SHOULDER_JOINT_INDEX] -= shoulder_rock_angle

        windup_motion = self.build_motion_params(0.5)
        r = self.call_joint_move_service(windup_pose, motion_params=windup_motion)
        if not (r and r['success']):
            self.get_logger().error('Failed to wind up for throw')
            return False

        # Fling: swing the elbow (and shoulder) forward fast, resetting
        # effectively to the faced pose ('fling_angle' 0.0 by default) -
        # fired without waiting for it to finish, so the release below
        # happens mid-swing rather than only once the arm has stopped
        fling_angle = float(params.get('fling_angle', 0.0))
        fling_pose = list(face_pose)
        fling_pose[self._ELBOW_JOINT_INDEX] += fling_angle
        fling_pose[self._SHOULDER_JOINT_INDEX] += shoulder_rock_angle

        fling_speed = float(params.get('speed', 1.0))
        fling_motion = self.build_motion_params(fling_speed)
        if not self.call_joint_move_service(fling_pose, motion_params=fling_motion, wait_for_completion=False):
            self.get_logger().error('Failed to start throw fling')
            return False

        # Release roughly midway through the fling, not after a fixed
        # delay - scaled to the actual size of the swing and its speed, so
        # a much bigger wind-up/fling still releases mid-swing rather than
        # either before the arm has really started moving or after it's
        # already stopped.
        fling_sweep = abs(fling_pose[self._ELBOW_JOINT_INDEX] - windup_pose[self._ELBOW_JOINT_INDEX])
        estimated_fling_duration = fling_sweep / (self._JOINT_MAX_VELOCITY_RAD_S * max(fling_speed, 0.1))
        default_release_delay = estimated_fling_duration * 0.5
        release_delay = float(params.get('release_delay', default_release_delay))
        time.sleep(release_delay)

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
