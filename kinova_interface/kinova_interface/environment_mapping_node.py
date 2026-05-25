import os
import json
import time
import threading
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from kinova_interfaces.srv import GetObjectCoordinates, GetRobotParameters, GetRelativeMovement
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


OBSTACLES = [
    {
        "id": "table",
        "description": "Table the robot arm is mounted on",
        "shape": SolidPrimitive.BOX,
        "dimensions": [1.2, 0.8, 0.05],
        "position": {"x": 0.0, "y": 0.0, "z": -0.03}
    },
    {
        "id": "computer",
        "description": "Computer monitor on the table near the robot",
        "shape": SolidPrimitive.BOX,
        "dimensions": [0.05, 0.35, 0.5],
        "position": {"x": -0.3, "y": 0.3, "z": 0.25}
    },
]


class EnvironmentMappingNode(Node):
    def __init__(self):
        super().__init__("environment_mapping_node")
        self.get_logger().info('Environment Mapping Node started')

        self.static_objects = self.load_coordinate_dictionary()
        self.relative_movements = self.load_relative_movements()

        self.srv_coords = self.create_service(GetObjectCoordinates, '/get_coordinates', self.get_coordinates_callback)
        self.srv_move = self.create_service(GetRelativeMovement, '/get_relative_movement', self.get_relative_movement_callback)
        self.srv_list = self.create_service(GetRobotParameters, '/get_robot_parameters', self.get_robot_parameters_callback)

        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        self.scene_thread = threading.Thread(target=self.publish_planning_scene, daemon=True)
        self.scene_thread.start()

    def load_coordinate_dictionary(self):
        pkg_share = get_package_share_directory('kinova_interface')
        json_path = os.path.join(pkg_share, 'data', 'coordinate_dictionary.json')
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
        pkg_share = get_package_share_directory('kinova_interface')
        json_path = os.path.join(pkg_share, 'data', 'relative_movement.json')
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
        obj_id = request.object_id
        if obj_id in self.static_objects:
            coords = self.static_objects[obj_id]
            response.x = coords['x']
            response.y = coords['y']
            response.z = coords['z']
            response.success = True
            response.message = "Object Found"
        else:
            response.success = False
            response.message = f"Object {obj_id} NOT Found"
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
        else:
            response.success = False
            response.message = f"Movement {move_id} NOT Found"
        return response

    def get_robot_parameters_callback(self, request, response):
        response.object_list = list(self.static_objects.keys()) + list(self.relative_movements.keys())
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

    def publish_planning_scene(self):
        self.get_logger().info('Waiting for /apply_planning_scene service...')
        if not self.scene_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().warn('/apply_planning_scene not available. MoveIt may not be running. Skipping planning scene setup.')
            return

        self.get_logger().info('MoveIt ready. Publishing collision objects to planning scene...')
        time.sleep(1.0)

        scene = PlanningScene()
        scene.is_diff = True

        for obstacle in OBSTACLES:
            obj = self.build_collision_object(obstacle)
            scene.world.collision_objects.append(obj)
            self.get_logger().info(f"Adding obstacle: '{obstacle['id']}' - {obstacle['description']}")

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.scene_client.call_async(request)

        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Planning scene updated with {len(OBSTACLES)} obstacle(s).')
            for obstacle in OBSTACLES:
                self.get_logger().info(f"  '{obstacle['id']}' at x={obstacle['position']['x']}, y={obstacle['position']['y']}, z={obstacle['position']['z']}")
        else:
            self.get_logger().error('Failed to apply planning scene.')


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
