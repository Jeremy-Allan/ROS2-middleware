import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from std_srvs.srv import Trigger
import threading
import time
import os
import json
from ament_index_python.packages import get_package_share_directory
from kinova_interfaces.srv import GetObjectCoordinates, GetRelativeMovement, ExecuteRecipe

class JsonParser:
    """Helper class to handle JSON loading and coordinate substitution."""
    def __init__(self, node_context, cb_group=None):
        self.recipe = None
        self.node = node_context # Reference to the ROS 2 node for logging/utils
        
        self.coord_client = self.node.create_client(GetObjectCoordinates, '/get_coordinates', callback_group=cb_group)
        self.relative_client = self.node.create_client(GetRelativeMovement, '/get_relative_movement', callback_group=cb_group)
    
    def load_recipe_from_file(self, recipe_path):
        try:
            with open(recipe_path, 'r') as f:
                self.recipe = json.load(f)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error loading Recipe JSON file: {e}")
            return False

    def load_recipe_from_service(self, recipe_str):
        try:
            self.recipe = json.loads(recipe_str)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error loading Recipe JSON string: {e}")
            return False
       
    def get_static_object_coords(self, target_name):
        if not self.coord_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("Coordinate service not available")
            return None
        req = GetObjectCoordinates.Request()
        req.object_id = target_name
        future = self.coord_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        if future.result() and future.result().success:
            return {'x': future.result().x, 'y': future.result().y, 'z': future.result().z}
        else:
            self.node.get_logger().error(f"Failed to get coordinates for {target_name}: {future.result().message if future.result() else 'no response'}")
            return None

    def get_relative_movement_vector(self, movement_name):
        if not self.relative_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("Relative movement service not available")
            return None
        req = GetRelativeMovement.Request()
        req.move_id = movement_name
        future = self.relative_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        if future.result() and future.result().success:
            return {'x': future.result().x, 'y': future.result().y, 'z': future.result().z}
        else:
            self.node.get_logger().error(f"Failed to get movement vector for {movement_name}: {future.result().message if future.result() else 'no response'}")
            return None

    def get_executable_steps(self):
        executable_list = []
        if not self.recipe:
            return executable_list

        self.node.get_logger().debug("Resolving steps and fetching coordinates...")
        for step in self.recipe.get('steps', []):
            action = step['action']
            params = step.get('parameters', {})
            cmd = {'action': action, 'description': step.get('description', '')}
            self.node.get_logger().debug(f"Processing step: {action}")

            if action == 'move_arm':
                target_name = params['target']
                self.node.get_logger().debug(f"Fetching coordinates for target: {target_name}")
                coords = self.get_static_object_coords(target_name)
                if coords:
                    cmd['values'] = coords
                else:
                    self.node.get_logger().error(f"Could not resolve target: {target_name}")
                    continue
            elif action == 'relative_move':
                vector_name = params['vector']
                self.node.get_logger().debug(f"Fetching vector for: {vector_name}")
                vector = self.get_relative_movement_vector(vector_name)
                if vector:
                    cmd['values'] = vector
                else:
                    self.node.get_logger().error(f"Could not resolve vector: {vector_name}")
                    continue
            elif action == 'gripper':
                cmd['values'] = params['position']
            elif action == 'home':
                cmd['values'] = None

            executable_list.append(cmd)
        self.node.get_logger().debug(f"Resolved {len(executable_list)} executable steps.")
        return executable_list

class JsonParserNode(Node):
    """ROS 2 Node that orchestrates tasks based on a JSON recipe."""
    def __init__(self):
        super().__init__('json_parser_node')
        
        # Create a reentrant callback group to avoid deadlocks when a service calls another service
        self.cb_group = ReentrantCallbackGroup()

        # 1. Reuseable ROS 2 Service Clients
        self.param_client = self.create_client(SetParameters, '/kinova_hardware_client/set_parameters', callback_group=self.cb_group)
        self.home_client = self.create_client(Trigger, '/kinova_hardware_client/home_arm', callback_group=self.cb_group)
        self.move_arm_client = self.create_client(Trigger, '/kinova_hardware_client/move_arm', callback_group=self.cb_group)
        self.move_gripper_client = self.create_client(Trigger, '/kinova_hardware_client/move_gripper', callback_group=self.cb_group)
        self.relative_move_client = self.create_client(Trigger, '/kinova_hardware_client/relative_move', callback_group=self.cb_group)

        # 2. Initialize the Parser (it creates its own internal client)
        self.parser = JsonParser(self, cb_group=self.cb_group)
        
        # 3. Create Service to execute recipes dynamically
        self.execute_srv = self.create_service(ExecuteRecipe, '/execute_recipe', self.execute_recipe_callback, callback_group=self.cb_group)

        self.get_logger().info(f"JSON Parser Node Online.")

        # 4. Declare and get the recipe parameter
        self.declare_parameter('recipe', 'none')
        recipe_file = self.get_parameter('recipe').get_parameter_value().string_value
        
        recipe_path = None
        if recipe_file and recipe_file.lower() != 'none':
            if os.path.isabs(recipe_file):
                recipe_path = recipe_file
            else:
                try:
                    package_share_directory = get_package_share_directory('kinova_interface')
                    recipe_path = os.path.join(package_share_directory, 'recipes', recipe_file)
                except Exception as e:
                    # Fallback for local development
                    self.get_logger().warning(f"Could not find package share directory, falling back to local path: {e}")
                    base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                    recipe_path = os.path.join(base_path, 'recipes', recipe_file)

        # 5. If a static recipe was provided, execute it on startup
        if recipe_path:
            self.get_logger().info(f"Loading static recipe from {recipe_path}")
            if self.parser.load_recipe_from_file(recipe_path):
                # Start the execution thread
                self.thread = threading.Thread(target=self.execute_recipe_thread)
                self.thread.start()
            else:
                self.get_logger().error(f"Failed to load recipe from {recipe_path}")

    def execute_recipe_callback(self, request, response):
        """Callback for the dynamic execution service."""
        self.get_logger().info("Received dynamic recipe execution request.")
        self.get_logger().debug(f"Payload recipe received: {request.recipe_json}")

        if not self.parser.load_recipe_from_service(request.recipe_json):
            self.get_logger().error("Failed to parse JSON recipe string.")
            response.success = False
            response.message = "Failed to parse JSON recipe string."
            return response

        self.get_logger().info("Successfully parsed JSON recipe. Getting executable steps...")
        success = self.execute_recipe()

        response.success = success
        if success:
            self.get_logger().info("Returning Success to client.")
            response.message = "Recipe executed successfully."
        else:
            self.get_logger().error("Returning Failure to client.")
            response.message = "Recipe execution failed. Check logs."

        return response

    def call_trigger_service(self, client):
        """Helper to call a Trigger service and wait for response."""
        self.get_logger().info(f"Calling trigger service: {client.srv_name}")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service {client.srv_name} not available!")
            return False
        
        request = Trigger.Request()
        future = client.call_async(request)
        
        self.get_logger().info(f"Waiting for {client.srv_name} to complete...")
        # In a background thread, we can block
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        
        if future.result() is not None:
            self.get_logger().info(f"{client.srv_name} completed with success={future.result().success}")
            return future.result().success
        self.get_logger().error(f"{client.srv_name} failed to return a valid result.")
        return False

    def set_hw_parameters(self, params_dict):
        """Helper to update parameters on the hardware node."""
        self.get_logger().info(f"Setting HW parameters: {params_dict}")
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
        self.get_logger().info("Waiting for parameter update to complete...")
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        
        success = future.result() is not None
        self.get_logger().info(f"Parameter update completed. Success: {success}")
        return success

    def execute_recipe_thread(self):
        """Wrapper for thread execution."""
        self.get_logger().info("Waiting for other nodes to be ready...")
        time.sleep(2.0) 
        self.execute_recipe()

    def execute_recipe(self):
        """Core execution logic."""
        steps = self.parser.get_executable_steps()
        if not steps:
            self.get_logger().error("No executable steps found or recipe failed to load.")
            return False

        self.get_logger().info(f"--- Starting Automated Sequence ({len(steps)} steps) ---")
        
        all_success = True
        for i, step in enumerate(steps):
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
            elif action == 'relative_move':
                if self.set_hw_parameters({
                    'vector_x': values['x'],
                    'vector_y': values['y'],
                    'vector_z': values['z']
                }):
                    success = self.call_trigger_service(self.relative_move_client)
            elif action == 'gripper':
                if self.set_hw_parameters({'gripper_position': float(values)}):
                    success = self.call_trigger_service(self.move_gripper_client)

            if success:
                self.get_logger().info(f"Step {i+1} completed successfully.")
                time.sleep(0.5)
            else:
                self.get_logger().error(f"Failed at step {i+1}: {action}")
                all_success = False
                break

        self.get_logger().info("--- All Tasks Completed ---")
        return all_success

def main():
    rclpy.init()
    node = JsonParserNode()
    
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=10) # TODO (pulkit) change the hardcoded threads numbers
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
