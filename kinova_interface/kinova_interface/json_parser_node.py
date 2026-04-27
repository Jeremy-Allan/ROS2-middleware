import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_srvs.srv import Trigger
import threading
import time
import os
import json
import argparse
from ament_index_python.packages import get_package_share_directory

class JsonParser:
    """Helper class to handle JSON loading and coordinate substitution."""
    def __init__(self, recipe_path, node_context):
        self.recipe_path = recipe_path
        self.recipe = None
        self.node = node_context # Reference to the ROS 2 node for logging/utils
        
        # Create the client once and reuse it
        self.dict_client = self.node.create_client(GetParameters, '/coordinate_dictionary_node/get_parameters')

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
        if not self.dict_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("Coordinate Dictionary Node service not available!")
            return None

        request = GetParameters.Request()
        request.names = [f"targets.{target_name}"]
        
        future = self.dict_client.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        if future.result() is not None:
            values = future.result().values
            if values and values[0].type == ParameterType.PARAMETER_DOUBLE_ARRAY:
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
        
        # 1. Reuseable ROS 2 Service Clients
        self.param_client = self.create_client(SetParameters, '/kinova_hardware_client/set_parameters')
        self.home_client = self.create_client(Trigger, '/kinova_hardware_client/home_arm')
        self.move_arm_client = self.create_client(Trigger, '/kinova_hardware_client/move_arm')
        self.move_gripper_client = self.create_client(Trigger, '/kinova_hardware_client/move_gripper')

        # 2. Initialize the Parser (it creates its own internal client)
        self.parser = JsonParser(recipe_path, self)
        if not self.parser.load_recipe():
            self.get_logger().error(f"Failed to load recipe from {recipe_path}")
            return

        self.get_logger().info(f"JSON Parser Node Online.")

        # Start the execution thread
        self.thread = threading.Thread(target=self.execute_recipe)
        self.thread.start()

    def call_trigger_service(self, client):
        """Helper to call a Trigger service and wait for response."""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service {client.srv_name} not available!")
            return False
        
        request = Trigger.Request()
        future = client.call_async(request)
        
        # In a background thread, we can block
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        
        if future.result() is not None:
            return future.result().success
        return False

    def set_hw_parameters(self, params_dict):
        """Helper to update parameters on the hardware node."""
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Hardware parameter service not available!")
            return False
        
        request = SetParameters.Request()
        for name, value in params_dict.items():
            param = Parameter()
            param.name = name
            if isinstance(value, float):
                param.value.type = ParameterType.PARAMETER_DOUBLE
                param.value.double_value = value
            request.parameters.append(param)
        
        future = self.param_client.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        
        return future.result() is not None

    def execute_recipe(self):
        self.get_logger().info("Waiting for other nodes to be ready...")
        time.sleep(2.0) 

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
                success = self.call_trigger_service(self.home_client)
            elif action == 'move_arm':
                if self.set_hw_parameters({
                    'target_x': values['x'],
                    'target_y': values['y'],
                    'target_z': values['z']
                }):
                    success = self.call_trigger_service(self.move_arm_client)
            elif action == 'gripper':
                if self.set_hw_parameters({'gripper_position': float(values)}):
                    success = self.call_trigger_service(self.move_gripper_client)

            if success:
                self.get_logger().info(f"Step {i+1} completed successfully.")
                time.sleep(0.5)
            else:
                self.get_logger().error(f"Failed at step {i+1}: {action}")
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
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
