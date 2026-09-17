import math
import time

import rclpy
from geometry_msgs.msg import Quaternion
from shape_msgs.msg import SolidPrimitive

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

    def call_joint_move_service(self, joint_positions, motion_params=None, wait_for_completion=True):
        """Move to an absolute joint-space target. With wait_for_completion
        False, this returns as soon as the goal is accepted rather than
        once the motion finishes - the caller (e.g. 'throw's fling) is then
        responsible for whatever timing it needs, and won't know whether
        the motion itself ultimately succeeded, only that it started."""
        if not self.joint_move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Joint Move service not available")
            return None
        req = JointMove.Request()
        req.joint_positions = [float(p) for p in joint_positions]
        req.wait_for_completion = wait_for_completion
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
        target_name = params['target']
        open_pos = float(params.get('open_position', 0.0))
        close_pos = float(params.get('close_position', 0.8))
        coords = self.get_static_object_coords(target_name)
        if not coords:
            return False

        # 1. Open gripper before moving
        rg = self.call_move_gripper_service(open_pos)
        if not (rg and rg['success']):
            self.get_logger().error('Failed to open gripper for pickup')
            return False

        # 2. Descend to the actual object position
        r = self.call_move_service(coords['x'], coords['y'], coords['z'])
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

    def _handle_pour(self, params):
        """Tilt the held object to pour, hold briefly, then return upright.
        Assumes the object named by 'target' is already grasped - fails
        cleanly rather than guessing if it isn't.

        The tilt itself is either a direct 'amount' (radians of pitch,
        overriding everything else) or a named 'orientation' preset,
        defaulting to 'tilted_for_pour'."""
        target_name = params.get('target')
        if not target_name:
            self.get_logger().error("pour action requires 'target' naming the held object")
            return False
        if target_name != self.held_object:
            self.get_logger().error(f"Cannot pour '{target_name}': held object is '{self.held_object}'")
            return False

        amount = params.get('amount')
        if amount is not None:
            roll, pitch, yaw = 0.0, float(amount), 0.0
        else:
            preset_name = params.get('orientation', 'tilted_for_pour')
            preset = self.get_orientation_preset(preset_name)
            if preset is None:
                self.get_logger().error(f"Unknown orientation preset '{preset_name}' for pour")
                return False
            roll, pitch, yaw = preset['roll'], preset['pitch'], preset['yaw']

        motion_params = self.build_motion_params(params.get('speed'))
        dwell = float(params.get('duration', 1.5))

        # 1. Tilt to pour, in place (zero displacement, orientation delta only)
        r = self.call_relative_move_service(0.0, 0.0, 0.0, True, roll, pitch, yaw, motion_params)
        if not (r and r['success']):
            self.get_logger().error('Failed to tilt for pour')
            return False

        # 2. Hold the tilt so contents can pour out
        time.sleep(dwell)

        # 3. Rotate back to upright
        r = self.call_relative_move_service(0.0, 0.0, 0.0, True, -roll, -pitch, -yaw, motion_params)
        if not (r and r['success']):
            self.get_logger().error('Failed to return to upright after pour')
            return False

        self.get_logger().info(f"Poured '{target_name}'")
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
    _ELBOW_JOINT_INDEX = 2  # joint_3

    def _handle_throw(self, params):
        """A genuine joint-space throw: face the throw direction from
        home's pose (rotating only joint_1, the base), wind up by rotating
        the elbow (joint_3) back away from that direction, then fling it
        forward past the faced pose, fast - releasing the gripper mid-swing
        rather than waiting for the fling to finish. Assumes the object
        named by 'target' is already grasped - fails cleanly rather than
        guessing if it isn't.

        The whole swing happens in a single vertical plane (only joint_1
        and joint_3 move), aimed by 'destination' or 'direction' the same
        way as other actions, but the actual distance thrown is governed
        by 'wind_up_angle'/'fling_angle'/'speed', not by 'distance' alone -
        those just decide which way the arm faces before swinging. Since
        firing the fling without waiting for it to finish means we can't
        confirm it actually succeeded, and the object leaves the gripper
        mid-swing rather than at a controlled position, the landing
        position recorded afterward is a rough approximation at best."""
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

        # Wind-up: rotate the elbow back, away from the throw direction,
        # like cocking the arm before a pitch
        wind_up_angle = float(params.get('wind_up_angle', 0.7854))  # 45 degrees
        windup_pose = list(face_pose)
        windup_pose[self._ELBOW_JOINT_INDEX] -= wind_up_angle

        windup_motion = self.build_motion_params(0.5)
        r = self.call_joint_move_service(windup_pose, motion_params=windup_motion)
        if not (r and r['success']):
            self.get_logger().error('Failed to wind up for throw')
            return False

        # Fling: swing the elbow forward past the faced pose, fast - fire
        # the motion without waiting for it to finish, so the release
        # below happens mid-swing rather than only once the arm has stopped
        fling_angle = float(params.get('fling_angle', 0.7854))
        fling_pose = list(face_pose)
        fling_pose[self._ELBOW_JOINT_INDEX] += fling_angle

        fling_motion = self.build_motion_params(params.get('speed', 1.0))
        if not self.call_joint_move_service(fling_pose, motion_params=fling_motion, wait_for_completion=False):
            self.get_logger().error('Failed to start throw fling')
            return False

        release_delay = float(params.get('release_delay', 0.25))
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
