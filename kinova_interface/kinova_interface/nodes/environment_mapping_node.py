import json
import os
import time
import threading
import rclpy
from pathlib import Path
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from kinova_interfaces.srv import GetObjectCoordinates, GetRobotParameters, GetRelativeMovement, GetOrientationPreset, GetObjectInfo, AttachObject, DetachObject, UpdateObjectPose
from std_srvs.srv import Trigger
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from kinova_interfaces.msg import ExtendedStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener

from kinova_interface.utils.geometry import euler_to_quaternion, pose_in_new_frame
from kinova_interface.utils.robot import BASE_FRAME, TOOL_FRAME, GRIPPER_TOUCH_LINKS


class EnvironmentMappingNode(Node):
    def __init__(self):
        super().__init__("environment_mapping_node")
        self.declare_parameter('config_dir', '')
        self.config_dir = self.get_parameter('config_dir').value

        if not self.config_dir:
            self.get_logger().fatal("Parameter 'config_dir' not set!")
            raise SystemExit(1)

        self.get_logger().info('Environment Mapping Node started')

        # TF Buffer and Listener, needed to attach objects at the gripper's
        # actual current pose rather than a guessed offset
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Telemetry Setup
        self.status_pub = self.create_publisher(ExtendedStatus, '/status/node_report', 10)
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.current_state = ExtendedStatus.STATE_IDLE
        self.status_text = "Environment Mapper Active"
        self.command_success = True
        
        self.static_objects = self.load_object_dictionary()
        self.relative_movements = self.load_relative_movements()
        self.orientation_presets = self.load_orientation_presets()
        self.obstacles = self.load_obstacles_dictionary()

        self.srv_coords = self.create_service(GetObjectCoordinates, '/get_coordinates', self.get_coordinates_callback)
        self.srv_move = self.create_service(GetRelativeMovement, '/get_relative_movement', self.get_relative_movement_callback)
        self.srv_orientation = self.create_service(GetOrientationPreset, '/get_orientation_preset', self.get_orientation_preset_callback)
        self.srv_list = self.create_service(GetRobotParameters, '/get_robot_parameters', self.get_robot_parameters_callback)
        self.srv_info = self.create_service(GetObjectInfo, '/get_object_info', self.get_object_info_callback)
        self.update_pose_srv = self.create_service(UpdateObjectPose, '/update_object_pose', self.update_object_pose_callback)
        self.scene_cb_group = ReentrantCallbackGroup()
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=self.scene_cb_group)
        self.attach_srv = self.create_service(AttachObject, '/attach_object', self.attach_object_callback, callback_group=self.scene_cb_group)
        self.detach_srv = self.create_service(DetachObject, '/detach_object', self.detach_object_callback, callback_group=self.scene_cb_group)
        self.reset_srv = self.create_service(Trigger, '/reset_environment_scene', self.reset_environment_callback, callback_group=self.scene_cb_group)

        self.attached_objects = set() 


        self.scene_thread = threading.Thread(target=self.publish_planning_scene, daemon=True)
        self.scene_thread.start()


    def publish_status(self):
        """Publish the node's current telemetry status.

        The published status includes the node name, current state, status
        message, and whether the most recent command was successful.

        Returns:
            None
        """
        
        msg = ExtendedStatus()
        msg.node_name = self.get_name()
        msg.state = self.current_state
        msg.status_message = self.status_text
        msg.last_command_valid = self.command_success
        self.status_pub.publish(msg)

    def load_object_dictionary(self):
        """Load and normalise the configured static object dictionary.

        The object dictionary is read from ``config_dir/object_dictionary.json``.
        Each object's pose and collision shape are normalised using
        : meth:`parse_object_data`.

        Returns:
            dict: Mapping of object identifiers to their parsed configuration.

        Raises:
            SystemExit: If the configuration file is missing or contains invalid
            JSON.
        """
        
        json_path = Path(self.config_dir) / 'object_dictionary.json'
        try:
            with open(json_path, 'r') as file:
                objects = json.load(file)
            self.get_logger().info('Loaded object dictionary JSON file')
        except FileNotFoundError:
            self.get_logger().fatal(f'Object Dictionary file not found at: {json_path}')
            raise SystemExit(1)
        except json.JSONDecodeError:
            self.get_logger().fatal('Failed to decode JSON from the object dictionary file')
            raise SystemExit(1)
        
        # Process each object using the helper
        for obj_id, obj_data in objects.items():
            objects[obj_id] = self.parse_object_data(obj_id, obj_data)
        
        self.get_logger().info(f'Processed {len(objects)} objects')
        return objects
    
    def load_obstacles_dictionary(self):
        """Load and normalise the configured obstacle dictionary.

        The obstacle configuration is loaded from the installed
        ``kinova_interface`` package share directory and each obstacle is
        processed using :meth:`parse_object_data`.

        Returns:
            dict: Mapping of obstacle identifiers to their parsed configuration.

        Raises:
            SystemExit: If the configuration file is missing or contains invalid
            JSON.
        """
        
        pkg_share = get_package_share_directory('kinova_interface')
        json_path = os.path.join(pkg_share, 'data', 'configs', 'env', 'obstacles.json')
        try:
            with open(json_path, 'r') as file:
                obstacles = json.load(file)
            self.get_logger().info('Loaded obstacles JSON file')
        except FileNotFoundError:
            self.get_logger().fatal(f'Obstacles file not found at: {json_path}')
            raise SystemExit(1)
        except json.JSONDecodeError:
            self.get_logger().fatal('Failed to decode JSON from obstacles file')
            raise SystemExit(1)
        
        # Process each obstacle using the helper function
        for obs_id, obs_data in obstacles.items():
            obstacles[obs_id] = self.parse_object_data(obs_id, obs_data)
        
        self.get_logger().info(f'Processed {len(obstacles)} obstacles')
        return obstacles
    
    def normalize_shape(self, obj, obj_id="unknown"):
        """Validate and normalise an object's collision shape configuration.

        Supported shape types are BOX, SPHERE, CYLINDER, and CONE. Unknown
        types and incorrectly sized dimension lists are replaced with safe
        default BOX or shape-specific values.

        Args:
            obj (dict): Object configuration containing a ``shape`` entry.
            obj_id (str): Object identifier used when reporting validation
            warnings.

        Returns:
            dict: The object configuration with a normalised ``shape`` entry.
        """
        
        shape = obj.get("shape", {})
        stype = shape.get("type", "BOX").upper()
        
        shape_info = { 
        "BOX": {"count": 3, "default": [0.05, 0.05, 0.05]},
        "SPHERE": {"count": 1, "default": [0.05]},
        "CYLINDER": {"count": 2, "default": [0.05, 0.02]},
        "CONE": {"count": 2, "default": [0.05, 0.02]}
        }
        
        if stype not in shape_info: #default to BOX if unknown shape type
            self.get_logger().warn(f"Object '{obj_id}': unknown shape type '{stype}', defaulting to BOX")
            stype = "BOX"

        info = shape_info[stype]
        dims = shape.get("dimensions", [])
        
        if len(dims) != info["count"]:
            self.get_logger().warn(
                f"Object '{obj_id}': expected {info['count']} dimensions for '{stype}', recieved {len(dims)}. "
                f"Using defaults: {info['default']}"
            )
            dims = info["default"].copy()
        
        # Store shape
        obj['shape'] = {
            'type': stype,
            'dimensions': dims
        }
        return obj

    def parse_object_data(self, obj_id, obj_data):
        """Normalise an object's pose and collision shape configuration.

        Orientations supplied as roll, pitch, and yaw are converted to
        quaternion form. Missing or invalid orientations are replaced with the
        identity quaternion. Shape information is normalised using
        :meth:`normalize_shape`.

        Args:
            obj_id (str): Identifier of the object being parsed.
            obj_data (dict): Raw object configuration loaded from JSON.

        Returns:
            dict: The normalised object configuration.
        """
        pose = obj_data.get('pose', {})
        orientation = pose.get('orientation', {})
        
        if not orientation:
            pose['orientation'] = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        elif any(k in orientation for k in ('roll', 'pitch', 'yaw')):
            roll = orientation.get('roll', 0.0)
            pitch = orientation.get('pitch', 0.0)
            yaw = orientation.get('yaw', 0.0)
            qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
            obj_data['pose']['orientation'] = {'x': qx, 'y': qy, 'z': qz, 'w': qw}
        else:
            # Already in quaternion format or invalid
            if not all(k in orientation for k in ('x', 'y', 'z', 'w')):
                self.get_logger().warn(f"Object '{obj_id}': invalid orientation format, defaulting to quaternion")
                pose['orientation'] = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        
        # Parse SHAPE (type & dimensions)
        obj_data = self.normalize_shape(obj_data, obj_id)
        
        return obj_data


    def load_relative_movements(self):
        """Load configured relative movement definitions.

        The definitions are read from ``config_dir/relative_movement.json``.

        Returns:
            dict: Mapping of movement identifiers to their X, Y, and Z offsets.

        Raises:
            SystemExit: If the configuration file is missing or contains invalid
                JSON.
        """
        
        json_path = Path(self.config_dir) / 'relative_movement.json'
        try:
            with open(json_path, 'r') as file:
                movements = json.load(file)
                self.get_logger().info('Loaded Relative Movement File')
            return movements
        except FileNotFoundError:
            self.get_logger().fatal(f'Relative Movement file not found at: {json_path}')
            raise SystemExit(1)
        except json.JSONDecodeError:
            self.get_logger().fatal('Failed to decode JSON from Relative Movement File')
            raise SystemExit(1)


    def load_orientation_presets(self):
        """Load configured orientation presets.

        The definitions are read from ``config_dir/orientation_presets.json``.

        Returns:
            dict: Mapping of preset names to roll, pitch, and yaw values.

        Raises:
            SystemExit: If the configuration file is missing or contains invalid
                JSON.
        """
        
        json_path = Path(self.config_dir) / 'orientation_presets.json'
        try:
            with open(json_path, 'r') as file:
                presets = json.load(file)
                self.get_logger().info('Loaded Orientation Presets File')
            return presets
        except FileNotFoundError:
            self.get_logger().fatal(f'Orientation Presets file not found at: {json_path}')
            raise SystemExit(1)
        except json.JSONDecodeError:
            self.get_logger().fatal('Failed to decode JSON from Orientation Presets File')
            raise SystemExit(1)

    def get_coordinates_callback(self, request, response):
        """Resolve an object identifier to its configured position.

        Args:
            request (GetObjectCoordinates.Request): Service request containing
                the object identifier.
            response (GetObjectCoordinates.Response): Service response populated
                with the object's X, Y, and Z coordinates or an error message.

        Returns:
            GetObjectCoordinates.Response: The populated service response.
        """
        
        obj_id = request.object_id
        if obj_id in self.static_objects:
            obj_data = self.static_objects[obj_id]
            pos = obj_data['pose']['position']
            response.x = pos['x']
            response.y = pos['y']
            response.z = pos['z']
            response.success = True
            response.message = "Object Found"
            self.command_success = True
            self.status_text = f"Resolved object: {obj_id}"
        else:
            response.success = False
            response.message = f"Object {obj_id} NOT Found"
            self.command_success = False
            self.status_text = f"Failed to resolve object: {obj_id}"
        self.publish_status()
        return response
        
    def get_relative_movement_callback(self, request, response):
        """Resolve a relative movement identifier to its configured offset.

        Args:
            request (GetRelativeMovement.Request): Service request containing
                the movement identifier.
            response (GetRelativeMovement.Response): Service response populated
                with the movement's X, Y, and Z offsets or an error message.

        Returns:
            GetRelativeMovement.Response: The populated service response.
        """
        
        move_id = request.move_id
        if move_id in self.relative_movements:
            move = self.relative_movements[move_id]
            response.x = move['x']
            response.y = move['y']
            response.z = move['z']
            response.success = True
            response.message = "Movement Found"
            self.command_success = True
            self.status_text = f"Resolved movement: {move_id}"
        else:
            response.success = False
            response.message = f"Movement {move_id} NOT Found"
            self.command_success = False
            self.status_text = f"Failed to resolve movement: {move_id}"
        self.publish_status()
        return response

    def get_orientation_preset_callback(self, request, response):
        """Resolve an orientation preset name to its configured Euler angles.

        Args:
            request (GetOrientationPreset.Request): Service request containing
                the preset name.
            response (GetOrientationPreset.Response): Service response populated
                with roll, pitch, and yaw values or an error message.

        Returns:
            GetOrientationPreset.Response: The populated service response.
        """
        
        preset_name = request.preset_name
        if preset_name in self.orientation_presets:
            preset = self.orientation_presets[preset_name]
            response.roll = preset['roll']
            response.pitch = preset['pitch']
            response.yaw = preset['yaw']
            response.success = True
            response.message = "Orientation preset found"
            self.command_success = True
            self.status_text = f"Resolved orientation preset: {preset_name}"
        else:
            response.success = False
            response.message = f"Orientation preset {preset_name} NOT Found"
            self.command_success = False
            self.status_text = f"Failed to resolve orientation preset: {preset_name}"
        self.publish_status()
        return response

    def get_robot_parameters_callback(self, request, response):
        """Return the configured environment parameters available to clients.

        The response contains available object identifiers, relative movement
        names, orientation preset names, and table bounds when a BOX-shaped
        ``table`` obstacle is configured.

        Args:
            request (GetRobotParameters.Request): Service request.
            response (GetRobotParameters.Response): Service response populated
                with the available environment parameters.

        Returns:
            GetRobotParameters.Response: The populated service response.
        """
        
        response.object_list = list(self.static_objects.keys())
        response.movement_names = list(self.relative_movements.keys())
        response.orientation_names = list(self.orientation_presets.keys())

        table = self.obstacles.get('table')
        if table and table.get('shape', {}).get('type') == 'BOX':
            pos = table['pose']['position']
            dims = table['shape']['dimensions']
            response.has_table_bounds = True
            response.table_x_min = pos['x'] - dims[0] / 2.0
            response.table_x_max = pos['x'] + dims[0] / 2.0
            response.table_y_min = pos['y'] - dims[1] / 2.0
            response.table_y_max = pos['y'] + dims[1] / 2.0
        else:
            response.has_table_bounds = False

        self.command_success = True
        self.status_text = "Robot Parameters queried"
        self.publish_status()
        return response

    def get_object_info_callback(self, request, response):
        """Return the configured pose and collision shape for an object.

        Args:
            request (GetObjectInfo.Request): Service request containing the
                object identifier.
            response (GetObjectInfo.Response): Service response populated with
                the object's pose, shape type, dimensions, and lookup status.

        Returns:
            GetObjectInfo.Response: The populated service response.
        """
        
        obj_id = request.object_id
        if obj_id in self.static_objects:
            obj_data = self.static_objects[obj_id]
            pos = obj_data['pose']['position']
            orient = obj_data['pose']['orientation']

            response.pose.position.x = pos['x']
            response.pose.position.y = pos['y']
            response.pose.position.z = pos['z']
            response.pose.orientation.x = orient['x']
            response.pose.orientation.y = orient['y']
            response.pose.orientation.z = orient['z']
            response.pose.orientation.w = orient['w']

            # Map stored shape type (string) to SolidPrimitive enum
            shape_type_map = {
                "BOX": SolidPrimitive.BOX,
                "SPHERE": SolidPrimitive.SPHERE,
                "CYLINDER": SolidPrimitive.CYLINDER,
                "CONE": SolidPrimitive.CONE
            }
            stype = obj_data['shape'].get('type', 'BOX')
            response.shape.type = shape_type_map.get(stype, SolidPrimitive.BOX)
            response.shape.dimensions = obj_data['shape'].get('dimensions', [])

            response.success = True
            response.message = "Object Info Found"
            self.command_success = True
            self.status_text = f"Resolved object info: {obj_id}"
        else:
            response.success = False
            response.message = f"Object {obj_id} NOT Found"
            self.command_success = False
            self.status_text = f"Failed to resolve object info: {obj_id}"

        self.publish_status()
        return response

    def attach_object_callback(self, request, response):
        """Attach an environment object to the robot's tool frame.

        Looks up the requested object, obtains its current transform relative
        to the robot tool, and applies a MoveIt planning-scene update to attach
        the object to the tool frame.

        Args:
            request (AttachObject.Request): Service request containing the
                object identifier.
            response (AttachObject.Response): Service response populated with
                the result of the attachment operation.

        Returns:
            AttachObject.Response: The populated service response indicating
                whether the object was successfully attached.
        """
        
        obj_id = request.object_id
        if obj_id in self.attached_objects:
            self.get_logger().warn(f"Object '{obj_id}' is already attached.")
            response.success = True
            response.message = "Object already attached"
            return response

        if obj_id not in self.static_objects and obj_id not in self.obstacles:
            response.success = False
            response.message = f"Object '{obj_id}' not found in environment"
            self.get_logger().error(response.message)
            return response

        obj_data = self.static_objects.get(obj_id) or self.obstacles.get(obj_id)

        try:
            transform = self.tf_buffer.lookup_transform(
                TOOL_FRAME, BASE_FRAME, rclpy.time.Time(), timeout=Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().error(f"Could not look up tool_frame to attach '{obj_id}': {e}")
            response.success = False
            response.message = f"Failed to attach '{obj_id}': tool_frame transform unavailable"
            return response

        relative_pose = pose_in_new_frame(obj_data['pose'], transform)
        success = self.apply_attach_diff(obj_id, obj_data, relative_pose)
        if success:
            self.attached_objects.add(obj_id)
            response.success = True
            response.message = f"Object '{obj_id}' attached to gripper"
        else:
            response.success = False
            response.message = f"Failed to attach '{obj_id}'"
        return response

    def detach_object_callback(self, request, response):
        """Detach an object currently tracked as attached to the gripper.

        The object is removed from MoveIt's attached collision objects and is
        returned to the world planning scene according to MoveIt's attachment
        semantics.

        Args:
            request (DetachObject.Request): Service request containing the object
                identifier.
            response (DetachObject.Response): Service response indicating
                whether the detachment succeeded.

        Returns:
            DetachObject.Response: The populated service response.
        """
        
        obj_id = request.object_id
        if obj_id not in self.attached_objects:
            self.get_logger().warn(f"Object '{obj_id}' is not currently attached.")
            response.success = False
            response.message = f"Object '{obj_id}' not attached"
            return response

        success = self.apply_detach_diff(obj_id)
        if success:
            self.attached_objects.remove(obj_id)
            response.success = True
            response.message = f"Object '{obj_id}' detached (added back to scene)"
        else:
            response.success = False
            response.message = f"Failed to detach '{obj_id}'"
        return response

    def reset_environment_callback(self, request, response):
        """Reset the environment to the configured default state.

        Reloads the object and obstacle configuration files and rebuilds the
        MoveIt planning scene. This clears pose changes and attachment tracking
        without requiring the middleware node to restart.

        Args:
            request (Trigger.Request): Empty trigger request.
            response (Trigger.Response): Service response indicating whether
                the environment reset succeeded.

        Returns:
            Trigger.Response: The populated service response.
        """
        
        self.get_logger().info("Resetting environment to configured defaults...")
        self.static_objects = self.load_object_dictionary()
        self.obstacles = self.load_obstacles_dictionary()

        if self.apply_full_scene():
            response.success = True
            response.message = f"Environment reset: {len(self.static_objects)} object(s), {len(self.obstacles)} obstacle(s) restored to configured defaults"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "Failed to apply the reset planning scene"
            self.get_logger().error(response.message)

        self.command_success = response.success
        self.status_text = response.message
        self.publish_status()
        return response

    def apply_attach_diff(self, obj_id, obj_data, relative_pose):
        """Attach a collision object to the robot tool frame in MoveIt.

        Creates an AttachedCollisionObject using the object's collision geometry
        and pose relative to the tool frame, then submits the planning-scene
        difference to MoveIt.

        Args:
            obj_id (str): Identifier of the object to attach.
            obj_data (dict): Object configuration containing pose and shape data.
            relative_pose (dict): Object pose expressed relative to the tool
                frame.

        Returns:
            bool: True if MoveIt successfully applied the planning-scene change,
                otherwise False.
        """
        
        if not self.scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/apply_planning_scene not available')
            return False

        attached_obj_data = {'shape': obj_data['shape'], 'pose': relative_pose}
        collision_obj = self.build_collision_object(obj_id, attached_obj_data)
        if collision_obj is None:
            return False
        collision_obj.header.frame_id = TOOL_FRAME

        attached = AttachedCollisionObject()
        attached.link_name = TOOL_FRAME
        attached.object = collision_obj
        attached.touch_links = GRIPPER_TOUCH_LINKS

        # MoveIt moves a same-ID object from world to attached automatically
        # when it sees an AttachedCollisionObject with operation=ADD, no
        # separate world REMOVE needed, that was fighting this and getting
        # rejected as "object does not exist" (attach already removed it).
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)

        return self.send_apply_planning_scene(scene, obj_id, "attach")

    def apply_detach_diff(self, obj_id):
        """Detach an object from the robot tool frame in MoveIt.

        Creates a planning-scene diff that removes the specified attached
        collision object. MoveIt then returns the object to the world scene
        according to its attached-object semantics.

        Args:
            obj_id (str): Identifier of the attached object.

        Returns:
            bool: True if MoveIt successfully applied the planning-scene change,
                otherwise False.
        """
        
        if not self.scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/apply_planning_scene not available')
            return False

        detach_marker = AttachedCollisionObject()
        detach_marker.link_name = TOOL_FRAME
        detach_marker.object.id = obj_id
        detach_marker.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(detach_marker)

        return self.send_apply_planning_scene(scene, obj_id, "detach")

    def send_apply_planning_scene(self, scene, obj_id, label):
        """Submit a planning-scene update and wait for its result.

        The asynchronous MoveIt service call is given a maximum five-second
        wait period.

        Args:
            scene (PlanningScene): Planning-scene diff to apply.
            obj_id (str): Object identifier used for diagnostic logging.
            label (str): Operation label used for diagnostic logging.

        Returns:
            bool: True if the service completed successfully, otherwise False.
        """
        
        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.scene_client.call_async(request)
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > 5.0:
                self.get_logger().error(f"Timed out waiting for apply_planning_scene ({label} {obj_id})")
                return False
            time.sleep(0.01)

        if future.result() is not None and future.result().success:
            return True
        else:
            self.get_logger().error(f"apply_planning_scene failed for '{obj_id}' ({label})")
            return False
    
    def update_object_pose_callback(self, request, response):
        """Update the configured pose of a static environment object.

        The position is always replaced. The orientation is replaced only when
        the request contains a non-zero quaternion.

        Args:
            request (UpdateObjectPose.Request): Service request containing the
                object identifier and new pose.
            response (UpdateObjectPose.Response): Service response indicating
                whether the update succeeded.

        Returns:
            UpdateObjectPose.Response: The populated service response.
        """
        
        obj_id = request.object_id
        if obj_id not in self.static_objects:
            response.success = False
            response.message = f"Object '{obj_id}' not found"
            return response

        # Update position
        self.static_objects[obj_id]['pose']['position']['x'] = request.pose.position.x
        self.static_objects[obj_id]['pose']['position']['y'] = request.pose.position.y
        self.static_objects[obj_id]['pose']['position']['z'] = request.pose.position.z

        # Update orientation (if provided, else keep existing)
        if request.pose.orientation.x != 0.0 or request.pose.orientation.y != 0.0 or \
        request.pose.orientation.z != 0.0 or request.pose.orientation.w != 0.0:
            self.static_objects[obj_id]['pose']['orientation']['x'] = request.pose.orientation.x
            self.static_objects[obj_id]['pose']['orientation']['y'] = request.pose.orientation.y
            self.static_objects[obj_id]['pose']['orientation']['z'] = request.pose.orientation.z
            self.static_objects[obj_id]['pose']['orientation']['w'] = request.pose.orientation.w

        response.success = True
        response.message = f"Updated pose for '{obj_id}'"
        return response

    def build_collision_object(self, obj_id, obj_data):
        """Construct a MoveIt collision object from environment configuration.

        The configured primitive shape, dimensions, position, and orientation
        are converted into a ``CollisionObject`` using ``BASE_FRAME`` as the
        reference frame.

        Args:
            obj_id (str): Identifier assigned to the collision object.
            obj_data (dict): Object configuration containing pose and shape data.

        Returns:
            CollisionObject or None: The constructed collision object, or
                ``None`` if the configured shape type is unsupported.
        """   
        
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = BASE_FRAME
        collision_obj.id = obj_id

        #Shape
        shape = SolidPrimitive()
        shape_type_map = {
            "BOX": SolidPrimitive.BOX,
            "SPHERE": SolidPrimitive.SPHERE,
            "CYLINDER": SolidPrimitive.CYLINDER,
            "CONE": SolidPrimitive.CONE
        }
        stype = obj_data['shape']['type']
        if stype not in shape_type_map:
            self.get_logger().error(f"Object '{obj_id}': unknown shape type '{stype}'")
            return None
        shape.type = shape_type_map[stype]
        shape.dimensions = obj_data['shape']['dimensions']

        # Pose
        pose = Pose()
        # Position
        pos = obj_data['pose']['position']
        pose.position.x = pos['x']
        pose.position.y = pos['y']
        pose.position.z = pos['z']
        # Orientation
        orient = obj_data['pose']['orientation']
        pose.orientation.x = orient['x']
        pose.orientation.y = orient['y']
        pose.orientation.z = orient['z']
        pose.orientation.w = orient['w']

        # Build collision object
        collision_obj.primitives.append(shape)
        collision_obj.primitive_poses.append(pose)
        collision_obj.operation = CollisionObject.ADD

        return collision_obj

    def publish_planning_scene(self):
        """Wait for MoveIt and publish the initial planning scene.

        This method is executed by the node's background scene thread. It waits
        for the ``/apply_planning_scene`` service to become available before
        calling :meth:`apply_full_scene`.
        """
        
        self.get_logger().info('Waiting for /apply_planning_scene service...')
        if not self.scene_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().warn('/apply_planning_scene not available. MoveIt may not be running. Skipping planning scene setup.')
            return

        self.get_logger().info('MoveIt ready. Publishing collision objects to planning scene...')
        time.sleep(1.0)
        self.apply_full_scene()

    def apply_full_scene(self):
        """Rebuild and apply the complete MoveIt planning scene.

        The scene is reconstructed from the current obstacle and static-object
        dictionaries. Any tracked attached objects are first removed from the
        attached-object state. On successful application, attachment tracking
        is cleared.

        Returns:
            bool: True if the complete planning scene was successfully applied,
                otherwise False.
        """
        
        scene = PlanningScene()
        scene.is_diff = True

        if self.attached_objects:
            scene.robot_state.is_diff = True
            for obj_id in self.attached_objects:
                detach_marker = AttachedCollisionObject()
                detach_marker.link_name = TOOL_FRAME
                detach_marker.object.id = obj_id
                detach_marker.object.operation = CollisionObject.REMOVE
                scene.robot_state.attached_collision_objects.append(detach_marker)

        # Add obstacles from obstacles dictionary
        for obs_id, obs_data in self.obstacles.items():
            obj = self.build_collision_object(obs_id, obs_data)
            if obj is not None:
                scene.world.collision_objects.append(obj)
                self.get_logger().info(f"Adding obstacle: '{obs_id}'")

        # Add objects from object dictionary
        for obj_id, obj_data in self.static_objects.items():
            obj = self.build_collision_object(obj_id, obj_data)
            if obj is not None:
                scene.world.collision_objects.append(obj)
                self.get_logger().info(f"Adding object: '{obj_id}'")

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.scene_client.call_async(request)
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > 5.0:
                self.get_logger().error('Timed out waiting for apply_planning_scene (full scene)')
                return False
            time.sleep(0.1)

        if future.result() is not None and future.result().success:
            self.attached_objects.clear()
            total = len(self.obstacles) + len(self.static_objects)
            self.get_logger().info(f'Planning scene updated with {total} collision objects.')
            for obs_id, obs_data in self.obstacles.items():
                pos = obs_data['pose']['position']
                self.get_logger().info(f"  Obstacle '{obs_id}' at x={pos['x']}, y={pos['y']}, z={pos['z']}")
            for obj_id, obj_data in self.static_objects.items():
                pos = obj_data['pose']['position']
                self.get_logger().info(f"  Object '{obj_id}' at x={pos['x']}, y={pos['y']}, z={pos['z']}")
            return True
        else:
            self.get_logger().error('Failed to apply planning scene.')
            return False


    def main(args=None):
        """Start the environment mapping ROS 2 node.

        Initialises rclpy, creates the environment mapping node, runs it using
        a four-threaded executor, and performs node and ROS 2 shutdown cleanup.

        Args:
            args (list[str], optional): Command-line arguments passed to rclpy.

        Returns:
            None
        """
    
        rclpy.init(args=args)
        node = EnvironmentMappingNode()
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
        node.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()
