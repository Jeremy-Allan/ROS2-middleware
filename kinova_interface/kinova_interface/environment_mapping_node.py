import json
import time
import threading
import rclpy
from pathlib import Path
from rclpy.node import Node
from kinova_interfaces.srv import GetObjectCoordinates, GetRobotParameters, GetRelativeMovement
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from kinova_interfaces.msg import ExtendedStatus
from kinova_interface.manipulation_config import load_manipulation_config, ConfigurationError

SHAPE_TYPE_MAP = {
    "box": SolidPrimitive.BOX,
    "sphere": SolidPrimitive.SPHERE,
    "cylinder": SolidPrimitive.CYLINDER,
    "cone": SolidPrimitive.CONE,
}


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

        self.static_objects = self.load_coordinate_dictionary()
        self.relative_movements = self.load_relative_movements()
        self.obstacles = self.load_obstacles()
        self.manipulation_config, self.raw_manipulation_json = self.load_manipulation_config()

        self.srv_coords = self.create_service(GetObjectCoordinates, '/get_coordinates', self.get_coordinates_callback)
        self.srv_move = self.create_service(GetRelativeMovement, '/get_relative_movement', self.get_relative_movement_callback)
        self.srv_list = self.create_service(GetRobotParameters, '/get_robot_parameters', self.get_robot_parameters_callback)

        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = "Waiting to apply the static collision scene"
        self.scene_thread = threading.Thread(target=self.publish_planning_scene, daemon=True)
        self.scene_thread.start()


    def publish_status(self):
        msg = ExtendedStatus()
        msg.node_name = self.get_name()
        msg.state = self.current_state
        msg.status_message = self.status_text
        msg.last_command_valid = self.command_success
        self.status_pub.publish(msg)

    def load_coordinate_dictionary(self):
        json_path = Path(self.config_dir) / 'coordinate_dictionary.json'
        try:
            with open(json_path, 'r') as file:
                objects = json.load(file)
                self.get_logger().info('Loaded static objects')
            return objects
        except FileNotFoundError:
            self.get_logger().fatal(f'Coordinate Dictionary file not found at: {json_path}')
            raise SystemExit(1)
        except json.JSONDecodeError:
            self.get_logger().fatal('Failed to decode JSON from the file')
            raise SystemExit(1)

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

    def load_obstacles(self):
        json_path = Path(self.config_dir) / 'obstacles.json'
        try:
            with open(json_path, 'r') as file:
                obstacles = json.load(file)
                self.get_logger().info('Loaded obstacles')
            return obstacles
        except FileNotFoundError:
            self.get_logger().fatal(f'Obstacles file not found at: {json_path}')
            raise SystemExit(1)
        except json.JSONDecodeError:
            self.get_logger().fatal('Failed to decode JSON from obstacles file')
            raise SystemExit(1)

    def load_manipulation_config(self):
        json_path = Path(self.config_dir) / 'manipulation_objects.json'
        raw_json = {}
        config = None
        if json_path.exists():
            try:
                config = load_manipulation_config(json_path)
                with open(json_path, 'r') as file:
                    raw_json = json.load(file)
                self.get_logger().info('Loaded manipulation_objects.json for planning scene object shapes')
            except ConfigurationError as exc:
                self.get_logger().warning(f'Validation warning in manipulation_objects.json: {exc}')
            except Exception as exc:
                self.get_logger().warning(f'Could not load manipulation_objects.json: {exc}')
        else:
            self.get_logger().info('manipulation_objects.json not found; using default object shapes')
        return config, raw_json

    def get_coordinates_callback(self, request, response):
        obj_id = request.object_id
        if obj_id in self.static_objects:
            response.x = self.static_objects[obj_id]['x']
            response.y = self.static_objects[obj_id]['y']
            response.z = self.static_objects[obj_id]['z']
            response.success = True
            self.command_success = True
            self.status_text = f"Coordinates found for object: {obj_id}"
        else:
            response.x = 0.0
            response.y = 0.0
            response.z = 0.0
            response.success = False
            self.command_success = False
            self.status_text = f"Coordinates NOT found for object: {obj_id}"

        self.publish_status()
        return response
        
    def get_relative_movement_callback(self, request, response):
        movement_name = request.movement_name
        if movement_name in self.relative_movements:
            response.dx = self.relative_movements[movement_name]['dx']
            response.dy = self.relative_movements[movement_name]['dy']
            response.dz = self.relative_movements[movement_name]['dz']
            response.success = True
            self.command_success = True
            self.status_text = f"Relative movement found for object: {movement_name}"
        else:
            response.dx = 0.0
            response.dy = 0.0
            response.dz = 0.0
            response.success = False
            self.command_success = False
            self.status_text = f"Relative movement NOT found for object: {movement_name}"

        self.publish_status()
        return response

    def get_robot_parameters_callback(self, request, response):
        response.object_list = list(self.static_objects.keys())
        response.movement_names = list(self.relative_movements.keys())
        self.command_success = True
        self.status_text = f"Robot Parameters queried"
        self.publish_status()
        return response

    def build_collision_object(self, obstacle):
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = obstacle["id"]

        shape = SolidPrimitive()
        shape.type = obstacle["shape"]
        shape.dimensions = obstacle["dimensions"]

        pose = Pose()
        pose.position.x = obstacle["position"]["x"]
        pose.position.y = obstacle["position"]["y"]
        pose.position.z = obstacle["position"]["z"]
        pose.orientation.w = 1.0

        obj.primitives.append(shape)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        return obj

    def extract_shape_and_dimensions(self, obj_id, coords):
        """
        Extract shape primitive type, dimensions, and optional orientation quaternion from configuration.
        """
        # 1. Check loaded ManipulationConfig objects
        if self.manipulation_config and obj_id in self.manipulation_config.objects:
            obj_spec = self.manipulation_config.objects[obj_id]
            shape_type = SHAPE_TYPE_MAP.get(obj_spec.shape.kind, SolidPrimitive.BOX)
            dimensions = list(obj_spec.shape.dimensions)
            orientation = list(obj_spec.collision_pose.orientation)
            return shape_type, dimensions, orientation

        # 2. Check coordinate_dictionary.json entry
        if isinstance(coords, dict):
            shape_kind = coords.get("shape", "box")
            shape_type = SHAPE_TYPE_MAP.get(shape_kind, SolidPrimitive.BOX)
            if "dimensions" in coords and isinstance(coords["dimensions"], (list, tuple)):
                return shape_type, [float(d) for d in coords["dimensions"]], coords.get("orientation")

        # 3. Check raw manipulation_objects.json dict
        raw_objects = self.raw_manipulation_json.get("objects", {})
        raw_dests = self.raw_manipulation_json.get("destinations", {})
        target_dict = raw_objects.get(obj_id) or raw_dests.get(obj_id)

        if isinstance(target_dict, dict) and "shape" in target_dict:
            shape_info = target_dict["shape"]
            shape_type = SHAPE_TYPE_MAP.get(shape_info.get("type", "box"), SolidPrimitive.BOX)
            dimensions = [float(d) for d in shape_info.get("dimensions", [])]
            if dimensions:
                return shape_type, dimensions, None

        self.get_logger().warning(
            f"No dimensions specified in config for '{obj_id}'; defaulting to 2.5cm cube"
        )
        return SolidPrimitive.BOX, [0.025, 0.025, 0.025], None

    def build_coordinate_collision_object(self, obj_id, coords):
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = obj_id

        shape_type, dimensions, orientation = self.extract_shape_and_dimensions(obj_id, coords)

        shape = SolidPrimitive()
        shape.type = shape_type
        shape.dimensions = dimensions

        pose = Pose()
        pose.position.x = float(coords['x'])
        pose.position.y = float(coords['y'])
        pose.position.z = float(coords['z'])

        if orientation and len(orientation) == 4:
            pose.orientation.x = float(orientation[0])
            pose.orientation.y = float(orientation[1])
            pose.orientation.z = float(orientation[2])
            pose.orientation.w = float(orientation[3])
        else:
            pose.orientation.w = 1.0

        obj.primitives.append(shape)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        return obj

    def publish_planning_scene(self):
        self.get_logger().info('Waiting for /apply_planning_scene service...')
        while rclpy.ok():
            if not self.scene_client.wait_for_service(timeout_sec=2.0):
                self.status_text = "Waiting for MoveIt planning-scene service"
                self.command_success = False
                continue

            scene = PlanningScene()
            scene.is_diff = True

            obstacle_ids = set()
            for obstacle in self.obstacles:
                scene.world.collision_objects.append(
                    self.build_collision_object(obstacle)
                )
                obstacle_ids.add(obstacle.get("id"))

            for obj_id, coords in self.static_objects.items():
                if obj_id not in obstacle_ids:
                    scene.world.collision_objects.append(
                        self.build_coordinate_collision_object(obj_id, coords)
                    )

            request = ApplyPlanningScene.Request()
            request.scene = scene
            future = self.scene_client.call_async(request)
            deadline = time.monotonic() + 15.0
            while (
                rclpy.ok()
                and not future.done()
                and time.monotonic() < deadline
            ):
                time.sleep(0.1)

            try:
                result = future.result() if future.done() else None
            except Exception as exc:
                self.get_logger().error(
                    f"Planning-scene request failed: {exc}"
                )
                result = None

            if result is not None and result.success:
                self.current_state = ExtendedStatus.STATE_IDLE
                total_objects = len(scene.world.collision_objects)
                self.status_text = (
                    f"Planning scene ready with {total_objects} object(s)"
                )
                self.command_success = True
                self.publish_status()
                self.get_logger().info(self.status_text)
                return

            if not future.done():
                future.cancel()
                self.get_logger().error(
                    "Timed out applying the planning scene; retrying"
                )
            else:
                self.get_logger().error(
                    "MoveIt rejected the planning scene; retrying"
                )
            self.current_state = ExtendedStatus.STATE_FAULT
            self.status_text = "Static planning scene is not ready; retrying"
            self.command_success = False
            self.publish_status()
            time.sleep(2.0)


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
