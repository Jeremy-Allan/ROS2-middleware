import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
import os
import json
from ament_index_python.packages import get_package_share_directory
#Services
from kinova_interfaces.srv import GetObjectCoordinates, GetRelativeMovement, GetObjectInfo, ExecuteRecipe, HomeArm, MoveArm, MoveGripper, RelativeMove, AttachObject, DetachObject
from kinova_interfaces.msg import ExtendedStatus

class JsonParser:
    """Helper class to handle JSON loading."""
    def __init__(self, node_context):
        self.recipe = None
        self.node = node_context # Reference to the ROS 2 node for logging
    
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

    def get_recipe_steps(self):
        if not self.recipe:
            return []
        return self.recipe.get('steps', [])

class JsonParserNode(Node):
    """ROS 2 Node that orchestrates tasks based on a JSON recipe."""
    def __init__(self):
        super().__init__('json_parser_node')
        
        # 1. Callback Groups
        # Reentrant group for general service clients to allow multiple responses
        self.cb_group = ReentrantCallbackGroup()
        # Mutually exclusive group for the execution sequence to ensure one recipe at a time
        self.exec_cb_group = MutuallyExclusiveCallbackGroup()

        # Hardware Interface Services
        self.home_client = self.create_client(HomeArm, '/kinova_hardware_client/home_arm', callback_group=self.cb_group)
        self.move_arm_client = self.create_client(MoveArm, '/kinova_hardware_client/move_arm', callback_group=self.cb_group)
        self.move_gripper_client = self.create_client(MoveGripper, '/kinova_hardware_client/move_gripper', callback_group=self.cb_group)
        self.relative_move_client = self.create_client(RelativeMove, '/kinova_hardware_client/relative_move', callback_group=self.cb_group)

        # Service clients for coordinate fetching
        self.coord_client = self.create_client(GetObjectCoordinates, '/get_coordinates', callback_group=self.cb_group)
        self.relative_client = self.create_client(GetRelativeMovement, '/get_relative_movement', callback_group=self.cb_group)
        self.info_client = self.create_client(GetObjectInfo, '/get_object_info', callback_group=self.cb_group)

        # 3. Initialize the Parser
        self.parser = JsonParser(self)

        # Telemetry Setup
        self.status_pub = self.create_publisher(ExtendedStatus, '/status/node_report', 10)
        self.status_timer = self.create_timer(0.5, self.publish_status, callback_group=self.cb_group)
        self.current_state = ExtendedStatus.STATE_IDLE
        self.status_text = "JSON Parser Online & Ready"
        self.command_success = True

        # 4. Create Service to execute recipes dynamically
        # Put this in the exec_cb_group so dynamic recipes don't overlap with static ones
        self.execute_srv = self.create_service(ExecuteRecipe, '/execute_recipe', self.execute_recipe_callback, callback_group=self.exec_cb_group)

        self.get_logger().info(f"JSON Parser Node Online.")

        # 5. Declare and get the recipe parameter
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

        # 6. If a static recipe was provided, execute it on startup using a one-shot Timer
        if recipe_path:
            self.get_logger().info(f"Loading static recipe from {recipe_path}")
            if self.parser.load_recipe_from_file(recipe_path):
                self.startup_timer = self.create_timer(2.0, self.startup_timer_callback, callback_group=self.exec_cb_group)
            else:
                self.get_logger().error(f"Failed to load recipe from {recipe_path}")

        # Service clients for attach/detach provided by environment mapping node
        self.attach_client = self.create_client(AttachObject, '/attach_object', callback_group=self.cb_group)
        self.detach_client = self.create_client(DetachObject, '/detach_object', callback_group=self.cb_group)

    def wait_for_future(self, future, service_name, timeout_sec=10.0):
        """Safely wait for an async service call future to complete without deadlocking the executor."""
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > timeout_sec:
                self.get_logger().error(f"Timed out waiting for {service_name}")
                return None
            time.sleep(0.01)
        return future.result() if future.done() else None

    def publish_status(self):
        msg = ExtendedStatus()
        msg.node_name = self.get_name()
        msg.state = self.current_state
        msg.status_message = self.status_text
        msg.last_command_valid = self.command_success
        self.status_pub.publish(msg)

    def startup_timer_callback(self):
        """One-shot timer callback to start the initial recipe."""
        self.startup_timer.cancel()
        self.get_logger().info("Starting initial recipe sequence...")
        self.execute_recipe()

    def execute_recipe_callback(self, request, response):
        """Callback for the dynamic execution service."""
        self.get_logger().info("Received dynamic recipe execution request.")
        self.get_logger().debug(f"Payload recipe received: {request.recipe_json}")

        if not self.parser.load_recipe_from_service(request.recipe_json):
            self.get_logger().error("Failed to parse JSON recipe string.")
            response.success = False
            response.message = "Failed to parse JSON recipe string."
            self.command_success = False
            self.status_text = "Failed to parse dynamic JSON recipe"
            self.publish_status()
            return response

        self.get_logger().info("Successfully parsed JSON recipe. Executing...")
        success = self.execute_recipe()

        response.success = success
        if success:
            self.get_logger().info("Returning Success to client.")
            response.message = "Recipe executed successfully."
            self.command_success = True
            self.status_text = "Recipe execution complete (Success)"
            self.publish_status()
        else:
            self.get_logger().error("Returning Failure to client.")
            response.message = "Recipe execution failed. Check logs."
            self.command_success = False
            self.status_text = "Recipe execution failed"
            self.publish_status()

        return response

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


    def getObjectInfo(self, target_name):
        return self.get_object_info(target_name)

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

    def call_home_service(self):
        if not self.home_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Home Arm Service not available")
            return None

        req = HomeArm.Request()
        # Async call + safe wait loop
        future = self.home_client.call_async(req)
        response = self.wait_for_future(future, '/kinova_hardware_client/home_arm')
        
        if response and response.success:
            return {'success': response.success, 'message': response.message}
        else:
            self.get_logger().error(f"Failed to move Home: {response.message if response else 'no response'}")
            return None

    def call_move_service(self, x, y, z):
        if not self.move_arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Move Arm Service not available")
            return None
        req = MoveArm.Request()
        req.x = x
        req.y = y
        req.z = z

        # Async call + safe wait loop
        future = self.move_arm_client.call_async(req)
        response = self.wait_for_future(future, '/kinova_hardware_client/move_arm')
        
        if response and response.success:
            return {'success': response.success, 'message': response.message}
        else:
            self.get_logger().error(f"Failed to perform Move to:{x},{y},{z}: {response.message if response else 'no response'}")
            return None

    def call_relative_move_service(self, vx, vy, vz):
        if not self.relative_move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Relative Move Service not available")
            return None
        req = RelativeMove.Request()
        req.vx = vx
        req.vy = vy
        req.vz = vz

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


    def execute_recipe(self):
        """Core execution logic."""
        steps = self.parser.get_recipe_steps()
        if not steps:
            self.get_logger().error("No executable steps found or recipe failed to load.")
            self.command_success = False
            self.status_text = "No executable steps in recipe"
            self.publish_status()
            return False

        self.get_logger().info(f"--- Starting Automated Sequence ({len(steps)} steps) ---")
        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = f"Executing recipe: {self.parser.recipe.get('recipe_name', 'Unnamed')}"
        self.publish_status()

        all_success = True
        for i, step in enumerate(steps):
            self.get_logger().info(f"[Step {i+1}] {step.get('description', '')}")
            self.status_text = f"Step {i+1}/{len(steps)}: {step.get('description', '')}"
            self.publish_status()

            action = step['action']
            params = step.get('parameters', {})
            success = False
            if action == 'home':
                result = self.call_home_service()
                success = result is not None and result['success']
            elif action == 'move_arm':
                target_name = params['target']
                coords = self.get_static_object_coords(target_name)
                if coords:
                    result = self.call_move_service(coords['x'], coords['y'], coords['z'])
                    success = result is not None and result['success']
            elif action == 'relative_move':
                vector_name = params['vector']
                vector = self.get_relative_movement_vector(vector_name)
                if vector:
                    result = self.call_relative_move_service(vector['x'], vector['y'], vector['z'])
                    success = result is not None and result['success']
            elif action == 'gripper':
                gripper = float(params['position'])
                result = self.call_move_gripper_service(gripper)
                success = result is not None and result['success']
            elif action == 'pickup':
                target_name = params['target']
                open_pos = float(params.get('open_position', 0.0))
                close_pos = float(params.get('close_position', 0.8))
                coords = self.get_static_object_coords(target_name)
                if coords:
                    # 1. Open gripper before moving
                    rg = self.call_move_gripper_service(open_pos)
                    success = rg is not None and rg['success']
                    if not success:
                        self.get_logger().error('Failed to open gripper for pickup')
                        break

                    # 3. Descend to the actual object position
                    r = self.call_move_service(coords['x'], coords['y'], coords['z'])
                    success = r is not None and r['success']
                    if not success:
                        self.get_logger().error('Failed to move to object position')
                        break

                    # 4. Close gripper
                    rg = self.call_move_gripper_service(close_pos)
                    success = rg is not None and rg['success']
                    if success:
                        # 5. Remove object from planning scene (attach)
                        if not self.attach_object(target_name):
                            self.get_logger().error("Failed to attach object after pickup")
                            success = False
                        else:
                            self.get_logger().info(f"Picked up '{target_name}'")

            elif action == 'dropoff':
                target_name = params.get('target')
                destination_name = params.get('destination')
                open_pos = float(params.get('open_position', 0.0))
                place_offset = float(params.get('place_offset', 0.1))

                if not destination_name:
                    self.get_logger().error("dropoff action requires 'destination' object name")
                    success = False
                    break

                dest_coords = self.get_static_object_coords(destination_name)
                if not dest_coords:
                    self.get_logger().error(f"Could not resolve destination '{destination_name}'")
                    success = False
                    break

                px = dest_coords['x']
                py = dest_coords['y']
                pz = dest_coords['z'] + place_offset

                # 1. Move to a position offset above the destination
                r = self.call_move_service(px, py, pz)
                success = r is not None and r['success']
                if not success:
                    self.get_logger().error('Failed to move to place position')
                    break

                # 2. Open gripper to release
                rg = self.call_move_gripper_service(open_pos)
                success = rg is not None and rg['success']
                if not success:
                    self.get_logger().error('Failed to open gripper during place')
                    break

                if target_name:
                    # 3. Add object back to planning scene (detach)
                    self.detach_object(target_name)
                    self.get_logger().info(f"Placed '{target_name}' at '{destination_name}'")


            if success:
                self.get_logger().info(f"Step {i+1} completed successfully.")
                time.sleep(0.5)
            else:
                self.get_logger().error(f"Failed at step {i+1}: {action}")
                all_success = False
                break

        self.current_state = ExtendedStatus.STATE_IDLE
        self.command_success = all_success
        if all_success:
            self.status_text = "Recipe execution complete (Success)"
        else:
            self.status_text = f"Recipe failed at step {i+1}"
        self.publish_status()
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
