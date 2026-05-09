import os
import json
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from kinova_interfaces.srv import GetObjectCoordinates, GetRobotParameters, GetRelativeMovement


class EnvironmentMappingNode(Node):
    def __init__(self):
        super().__init__("environment_mapping_node")
        self.get_logger().info('Environment Mapping Node started')

        self.static_objects = self.load_coordinate_dictionary()
        self.relative_movements = self.load_relative_movements()

        self.srv_coords = self.create_service(GetObjectCoordinates, 'get_coordinates', self.get_coordinates_callback)
        self.srv_move = self.create_service(GetRelativeMovement, 'get_relative_movement', self.get_relative_movement_callback)
        self.srv_list = self.create_service(GetRobotParameters, 'get_robot_parameters', self.get_robot_parameters_callback)

    def load_coordinate_dictionary(self):
        pkg_share = get_package_share_directory('kinova_interface')
        json_path = os.path.join(pkg_share, 'recipes', 'coordinate_dictionary.json')
        try:
            with open(json_path, 'r') as file:
                data = json.load(file)
            self.get_logger().info('Loaded static objects')
            return data.get('static_targets', {})
        except FileNotFoundError:
            self.get_logger().fatal(f'Coordinate Dictionary file not found at: {json_path}')
            raise SystemExit(1)
        except json.JSONDecodeError:
            self.get_logger().fatal('Failed to decode JSON from the file')
            raise SystemExit(1)

    def load_relative_movements(self):
        pkg_share = get_package_share_directory('kinova_interface')
        json_path = os.path.join(pkg_share, 'recipes', 'relative_movement.json')
        try:
            with open(json_path, 'r') as file:
                data = json.load(file)
            self.get_logger().info('Loaded Relative Movement File')
            return data
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


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
