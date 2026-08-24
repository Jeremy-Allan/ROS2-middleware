import json
import os
import time
import threading
import rclpy
import math
from pathlib import Path
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from kinova_interfaces.srv import GetObjectCoordinates, GetRobotParameters, GetRelativeMovement, GetObjectInfo, AttachObject, DetachObject, UpdateObjectPose
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Quaternion
from kinova_interfaces.msg import ExtendedStatus
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class EnvironmentMappingNode(Node):
    def __init__(self):
        super().__init__("environment_mapping_node")
        self.declare_parameter('config_dir', '')
        self.config_dir = self.get_parameter('config_dir').value

        if not self.config_dir:
            self.get_logger().fatal("Parameter 'config_dir' not set!")
            raise SystemExit(1)

        self.get_logger().info('Environment Mapping Node started')

        # Telemetry Setup
        self.status_pub = self.create_publisher(ExtendedStatus, '/status/node_report', 10)
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.current_state = ExtendedStatus.STATE_IDLE
        self.status_text = "Environment Mapper Active"
        self.command_success = True
        
        self.static_objects = self.load_object_dictionary()
        self.relative_movements = self.load_relative_movements()
        self.obstacles = self.load_obstacles_dictionary()

        self.srv_coords = self.create_service(GetObjectCoordinates, '/get_coordinates', self.get_coordinates_callback)
        self.srv_move = self.create_service(GetRelativeMovement, '/get_relative_movement', self.get_relative_movement_callback)
        self.srv_list = self.create_service(GetRobotParameters, '/get_robot_parameters', self.get_robot_parameters_callback)
        self.srv_info = self.create_service(GetObjectInfo, '/get_object_info', self.get_object_info_callback)
        self.update_pose_srv = self.create_service(UpdateObjectPose, '/update_object_pose', self.update_object_pose_callback)
        self.scene_cb_group = ReentrantCallbackGroup()
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=self.scene_cb_group)
        self.attach_srv = self.create_service(AttachObject, '/attach_object', self.attach_object_callback, callback_group=self.scene_cb_group)
        self.detach_srv = self.create_service(DetachObject, '/detach_object', self.detach_object_callback, callback_group=self.scene_cb_group)
        
        self.attached_objects = set() 


        self.scene_thread = threading.Thread(target=self.publish_planning_scene, daemon=True)
        self.scene_thread.start()


    def publish_status(self):
        msg = ExtendedStatus()
        msg.node_name = self.get_name()
        msg.state = self.current_state
        msg.status_message = self.status_text
        msg.last_command_valid = self.command_success
        self.status_pub.publish(msg)

    def load_object_dictionary(self):
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

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        return {'x': qx, 'y': qy, 'z': qz, 'w': qw}
    
    def parse_object_data(self, obj_id, obj_data):
        # Parse POSE (position & orientation)
        pose = obj_data.get('pose', {})
        orientation = pose.get('orientation', {})
        
        if not orientation:
            pose['orientation'] = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        elif any(k in orientation for k in ('roll', 'pitch', 'yaw')):
            roll = orientation.get('roll', 0.0)
            pitch = orientation.get('pitch', 0.0)
            yaw = orientation.get('yaw', 0.0)
            quat = self.euler_to_quaternion(roll, pitch, yaw)
            obj_data['pose']['orientation'] = quat
        else:
            # Already in quaternion format or invalid
            if not all(k in orientation for k in ('x', 'y', 'z', 'w')):
                self.get_logger().warn(f"Object '{obj_id}': invalid orientation format, defaulting to quaternion")
                pose['orientation'] = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        
        # Parse SHAPE (type & dimensions)
        obj_data = self.normalize_shape(obj_data, obj_id)
        
        return obj_data


    def load_relative_movements(self):
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


    def get_coordinates_callback(self, request, response):
        #request is the obj_id, the response will be coordinates of obj
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

    def get_robot_parameters_callback(self, request, response):
        # Service for the LLM Proxy || send list of objects + relative movements
        response.object_list = list(self.static_objects.keys())
        response.movement_names = list(self.relative_movements.keys())
        self.command_success = True
        self.status_text = f"Robot Parameters queried"
        self.publish_status()
        return response

    def get_object_info_callback(self, request, response):
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

        # Remove from planning scene
        success = self.update_planning_scene_for_object(obj_id, CollisionObject.REMOVE)
        if success:
            self.attached_objects.add(obj_id)
            response.success = True
            response.message = f"Object '{obj_id}' attached (removed from scene)"
        else:
            response.success = False
            response.message = f"Failed to attach '{obj_id}'"
        return response

    def detach_object_callback(self, request, response):
        obj_id = request.object_id
        if obj_id not in self.attached_objects:
            self.get_logger().warn(f"Object '{obj_id}' is not currently attached.")
            response.success = False
            response.message = f"Object '{obj_id}' not attached"
            return response

        # Add back to planning scene
        success = self.update_planning_scene_for_object(obj_id, CollisionObject.ADD)
        if success:
            self.attached_objects.remove(obj_id)
            response.success = True
            response.message = f"Object '{obj_id}' detached (added back to scene)"
        else:
            response.success = False
            response.message = f"Failed to detach '{obj_id}'"
        return response
    
    def update_object_pose_callback(self, request, response):
        obj_id = request.object_id
        if obj_id not in self.static_objects:
            response.success = False
            response.message = f"Object '{obj_id}' not found"
            return response

        # Get existing pose for comparison
        old_pos = self.static_objects[obj_id]['pose']['position']
        old_orient = self.static_objects[obj_id]['pose']['orientation']

        # Check if change is significant (Threshold: 5mm or ~2 degrees)
        dist = math.sqrt(
            (request.pose.position.x - old_pos['x'])**2 +
            (request.pose.position.y - old_pos['y'])**2 +
            (request.pose.position.z - old_pos['z'])**2
        )
        
        # Simple quat check (dot product)
        dot = abs(request.pose.orientation.x * old_orient['x'] +
                  request.pose.orientation.y * old_orient['y'] +
                  request.pose.orientation.z * old_orient['z'] +
                  request.pose.orientation.w * old_orient['w'])

        if dist < 0.005 and dot > 0.999:
            response.success = True
            response.message = "Pose change below threshold, skipping update"
            return response

        # Update position
        self.static_objects[obj_id]['pose']['position']['x'] = request.pose.position.x
        self.static_objects[obj_id]['pose']['position']['y'] = request.pose.position.y
        self.static_objects[obj_id]['pose']['position']['z'] = request.pose.position.z

        # Update orientation
        self.static_objects[obj_id]['pose']['orientation']['x'] = request.pose.orientation.x
        self.static_objects[obj_id]['pose']['orientation']['y'] = request.pose.orientation.y
        self.static_objects[obj_id]['pose']['orientation']['z'] = request.pose.orientation.z
        self.static_objects[obj_id]['pose']['orientation']['w'] = request.pose.orientation.w

        # Trigger MoveIt Update
        success = self.update_planning_scene_for_object(obj_id, CollisionObject.ADD)
        
        if success:
            response.success = True
            response.message = f"Updated pose and MoveIt scene for '{obj_id}'"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"Failed to update MoveIt scene for '{obj_id}'"
            self.get_logger().error(response.message)

        return response
    def update_planning_scene_for_object(self, obj_id, operation):
        """Apply a REMOVE or ADD diff to the planning scene for a single object."""
        if not self.scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/apply_planning_scene not available')
            return False

        collision_obj = CollisionObject()
        collision_obj.header.frame_id = "base_link"
        collision_obj.id = obj_id
        collision_obj.operation = operation  # REMOVE / ADD

        if operation == CollisionObject.ADD:
            # Need to re-supply the full geometry when adding back
            obj_data = self.static_objects.get(obj_id) or self.obstacles.get(obj_id)
            if obj_data is None:
                self.get_logger().error(f"No stored data for '{obj_id}' to re-add to scene")
                return False
            full_obj = self.build_collision_object(obj_id, obj_data)
            if full_obj is None:
                return False
            collision_obj = full_obj
            collision_obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision_obj)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        
        future = self.scene_client.call_async(request)
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > 5.0:
                self.get_logger().error(f"Timed out waiting for apply_planning_scene ({obj_id})")
                return False
            time.sleep(0.01)

        if future.result() is not None and future.result().success:
            return True
        else:
            self.get_logger().error(f"apply_planning_scene failed for '{obj_id}' (op={operation})")
            return False

    def build_collision_object(self, obj_id, obj_data):
        
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = "base_link"
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
        self.get_logger().info('Waiting for /apply_planning_scene service...')
        if not self.scene_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().warn('/apply_planning_scene not available. MoveIt may not be running. Skipping planning scene setup.')
            return

        self.get_logger().info('MoveIt ready. Publishing collision objects to planning scene...')
        time.sleep(1.0)

        scene = PlanningScene()
        scene.is_diff = True

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
        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        if future.result() is not None and future.result().success:
            total = len(self.obstacles) + len(self.static_objects)
            self.get_logger().info(f'Planning scene updated with {total} collision objects.')
            for obs_id, obs_data in self.obstacles.items():
                pos = obs_data['pose']['position']
                self.get_logger().info(f"  Obstacle '{obs_id}' at x={pos['x']}, y={pos['y']}, z={pos['z']}")
            for obj_id, obj_data in self.static_objects.items():
                pos = obj_data['pose']['position']
                self.get_logger().info(f"  Object '{obj_id}' at x={pos['x']}, y={pos['y']}, z={pos['z']}")
            
            self.get_logger().info(f'Planning scene updated with {len(self.obstacles)} obstacle(s).')
        else:
            self.get_logger().error('Failed to apply planning scene.')


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentMappingNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
