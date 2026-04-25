import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
import threading
import time
import os
import json
import argparse
from ament_index_python.packages import get_package_share_directory
from .hardware_interface_client import HardwareInterfaceServer

class JsonParser:
    """Helper class to handle JSON loading and coordinate substitution."""
    def __init__(self, recipe_path, node_context):
        self.recipe_path = recipe_path
        self.recipe = None
        self.node = node_context # Reference to the ROS 2 node for service calls

    def load_recipe(self):
        try:
            with open(self.recipe_path, 'r') as f:
                self.recipe = json.load(f)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error loading Recipe JSON: {e}")
            return False

    def get_coords_from_dict_node(self, target_name):
        """Fetches XYZ from the CoordinateDictionaryNode via ROS 2 Service."""
        client = self.node.create_client(GetParameters, '/coordinate_dictionary_node/get_parameters')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("Coordinate Dictionary Node service not available!")
            return None

        request = GetParameters.Request()
        request.names = [f"targets.{target_name}"]
        
        future = client.call_async(request)
        # Using a simple wait loop since we are in a background thread
        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        if future.result() is not None:
            values = future.result().values
            if values and values[0].double_array_value:
                coords = values[0].double_array_value
                return {'x': coords[0], 'y': coords[1], 'z': coords[2]}
        
        return None

    def get_executable_steps(self):
        executable_list = []
        if not self.recipe:
            return executable_list

        for step in self.recipe.get('steps', []):
            action = step['action']
            params = step.get('parameters', {})
            cmd = {'action': action, 'description': step.get('description', '')}

            if action == 'move_arm':
                target_name = params['target']
                coords = self.get_coords_from_dict_node(target_name)
                if coords:
                    cmd['values'] = coords
                else:
                    self.node.get_logger().error(f"Could not resolve target: {target_name}")
                    continue
            
            elif action == 'gripper':
                cmd['values'] = params['position']
            elif action == 'home':
                cmd['values'] = None
            elif action == 'relative_move':
                cmd['values'] = params

            executable_list.append(cmd)
        return executable_list

class JsonParserNode(Node):
    """ROS 2 Node that orchestrates tasks based on a JSON recipe."""
    def __init__(self, recipe_path):
        super().__init__('json_parser_node')
        
        # 1. Reference the Hardware Logic (Always initialize to avoid AttributeError)
        self.hw_interface = HardwareInterfaceServer()

        # 2. Initialize the Parser
        self.parser = JsonParser(recipe_path, self)
        if not self.parser.load_recipe():
            self.get_logger().error(f"Failed to load recipe from {recipe_path}")
            return

        self.get_logger().info(f"JSON Parser Node Online.")

        # Start the execution thread after a short delay to allow dict node to start
        self.thread = threading.Thread(target=self.execute_recipe)
        self.thread.start()

    def execute_recipe(self):
        self.get_logger().info("Waiting for Coordinate Dictionary to be ready...")
        time.sleep(2.0) 

        # We resolve steps here so we can ask the Dict Node for coordinates
        self.steps = self.parser.get_executable_steps()
        if not self.steps:
            self.get_logger().error("No executable steps found or recipe failed to load.")
            return

        self.get_logger().info(f"--- Starting Automated Sequence ({len(self.steps)} steps) ---")
        
        for i, step in enumerate(self.steps):
            self.get_logger().info(f"[Step {i+1}] {step['description']}")
            
            action = step['action']
            values = step['values']

            success = False
            if action == 'home':
                success = self.hw_interface.send_home_goal()
            elif action == 'move_arm':
                success = self.hw_interface.send_goal(values['x'], values['y'], values['z'])
            elif action == 'gripper':
                success = self.hw_interface.move_gripper(values)

            if success:
                self.hw_interface.movement_finished.wait()
                time.sleep(0.5)
            else:
                self.get_logger().error(f"Failed to initiate action: {action}")
                break

        self.get_logger().info("--- All Tasks Completed ---")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--recipe', type=str, required=True)
    args, unknown = parser.parse_known_args()

    rclpy.init()
    
    recipe_file = args.recipe
    if os.path.isabs(recipe_file):
        recipe_path = recipe_file
    else:
        try:
            package_share_directory = get_package_share_directory('kinova_interface')
            recipe_path = os.path.join(package_share_directory, 'recipes', recipe_file)
        except Exception:
            # Fallback for local development
            base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            recipe_path = os.path.join(base_path, 'recipes', recipe_file)

    node = JsonParserNode(recipe_path)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.hw_interface)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
