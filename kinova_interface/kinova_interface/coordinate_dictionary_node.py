import rclpy
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory

class CoordinateDictionaryNode(Node):
    def __init__(self):
        super().__init__('coordinate_dictionary_node')
        
        # 1. Load the initial data from the JSON file to populate parameters
        try:
            package_share_directory = get_package_share_directory('kinova_interface')
            dict_path = os.path.join(package_share_directory, 'recipes', 'coordinate_dictionary.json')
        except Exception:
            # Fallback for local development if not installed
            base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            dict_path = os.path.join(base_path, 'recipes', 'coordinate_dictionary.json')
        
        try:
            with open(dict_path, 'r') as f:
                data = json.load(f)
                
            # 2. Register each target as a ROS 2 Parameter
            # Parameters in ROS 2 are usually flat, so we'll use a prefix
            for name, coords in data.get('static_targets', {}).items():
                param_name = f"targets.{name}"
                # We store as a double array [x, y, z]
                self.declare_parameter(param_name, [float(coords['x']), float(coords['y']), float(coords['z'])])
                self.get_logger().info(f"Registered target: {param_name}")

            for name, vec in data.get('relative_vectors', {}).items():
                param_name = f"vectors.{name}"
                self.declare_parameter(param_name, [float(vec['x']), float(vec['y']), float(vec['z'])])
                self.get_logger().info(f"Registered vector: {param_name}")

        except Exception as e:
            self.get_logger().error(f"Failed to load dictionary into parameters: {e}")

        self.get_logger().info("Coordinate Dictionary Node is ONLINE and serving parameters.")

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateDictionaryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
