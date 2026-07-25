import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
import os
import json
from ament_index_python.packages import get_package_share_directory
#Services
from kinova_interfaces.action import PickPlace
from kinova_interfaces.srv import GetObjectCoordinates, GetRelativeMovement, ExecuteRecipe, HomeArm, MoveArm, MoveGripper, RelativeMove
from kinova_interfaces.msg import ExtendedStatus

class JsonParser:
    """Helper class to handle JSON loading."""
    def __init__(self, node_context):
        self.recipe = None
        self.node = node_context # Reference to the ROS 2 node for logging
    
    def load_recipe_from_file(self, recipe_path):
        try:
            with open(recipe_path, 'r') as f:
                recipe = json.load(f)
            self._set_recipe(recipe)
            return True
        except Exception as e:
            self.recipe = None
            self.node.get_logger().error(f"Error loading Recipe JSON file: {e}")
            return False

    def load_recipe_from_service(self, recipe_str):
        try:
            recipe = json.loads(recipe_str)
            self._set_recipe(recipe)
            return True
        except Exception as e:
            self.recipe = None
            self.node.get_logger().error(f"Error loading Recipe JSON string: {e}")
            return False

    def _set_recipe(self, recipe):
        if not isinstance(recipe, dict):
            raise ValueError("recipe root must be a JSON object")
        if 'steps' not in recipe:
            raise ValueError("recipe root must contain a 'steps' array")
        steps = recipe['steps']
        if not isinstance(steps, list):
            raise ValueError("recipe.steps must be a JSON array")
        for index, step in enumerate(steps):
            if not isinstance(step, dict):
                raise ValueError(
                    f"recipe.steps[{index}] must be a JSON object"
                )
        recipe_name = recipe.get('recipe_name')
        if recipe_name is not None and (
            not isinstance(recipe_name, str) or not recipe_name.strip()
        ):
            raise ValueError("recipe.recipe_name must be a non-empty string")
        self.recipe = recipe

    def get_recipe_steps(self):
        if not isinstance(self.recipe, dict):
            return []
        steps = self.recipe.get('steps')
        return steps if isinstance(steps, list) else []

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
        self.pick_place_client = ActionClient(
            self,
            PickPlace,
            '/mtc_task_node/pick_place',
            callback_group=self.cb_group,
        )

        # Service clients for coordinate fetching
        self.coord_client = self.create_client(GetObjectCoordinates, '/get_coordinates', callback_group=self.cb_group)
        self.relative_client = self.create_client(GetRelativeMovement, '/get_relative_movement', callback_group=self.cb_group)

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
        self.declare_parameter('service_timeout_sec', 120.0)
        self.declare_parameter('pick_place_timeout_sec', 600.0)
        self.service_timeout_sec = self.get_parameter('service_timeout_sec').value
        self.pick_place_timeout_sec = self.get_parameter('pick_place_timeout_sec').value
        if self.service_timeout_sec <= 0.0 or self.pick_place_timeout_sec <= 0.0:
            raise ValueError("Recipe service/action timeouts must be positive")
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

    def wait_for_future(self, future, service_name, timeout_sec=None):
        """Wait for an async result with a finite deadline."""
        timeout = self.service_timeout_sec if timeout_sec is None else timeout_sec
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                future.cancel()
                self.get_logger().error(
                    f"Timed out after {timeout:.1f}s waiting for {service_name}"
                )
                return None
            time.sleep(0.01)

        if future.done():
            try:
                return future.result()
            except Exception as exc:
                self.get_logger().error(
                    f"{service_name} completed with an exception: {exc}"
                )
                return None
        else:
            self.get_logger().error(f"Future for {service_name} was cleared or canceled.")
            return None

    def call_pick_place_action(self, object_id, destination_id, plan_only=False):
        if not self.pick_place_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MTC pick/place action server is not available")
            return None

        goal = PickPlace.Goal()
        goal.object_id = object_id
        goal.destination_id = destination_id
        goal.plan_only = plan_only

        self.get_logger().info(
            f"Requesting MTC pick/place: {object_id} -> {destination_id} "
            f"(plan_only={plan_only})"
        )
        goal_future = self.pick_place_client.send_goal_async(
            goal,
            feedback_callback=self.pick_place_feedback,
        )
        goal_handle = self.wait_for_future(
            goal_future,
            '/mtc_task_node/pick_place goal',
            timeout_sec=15.0,
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                "MTC pick/place goal was rejected (invalid configuration or server busy)"
            )
            return None

        result_future = goal_handle.get_result_async()
        result_response = self.wait_for_future(
            result_future,
            '/mtc_task_node/pick_place result',
            timeout_sec=self.pick_place_timeout_sec,
        )
        if result_response is None:
            cancel_future = goal_handle.cancel_goal_async()
            self.wait_for_future(
                cancel_future,
                '/mtc_task_node/pick_place cancellation',
                timeout_sec=5.0,
            )
            return None

        result = result_response.result
        if (
            result_response.status == GoalStatus.STATUS_SUCCEEDED
            and result.success
        ):
            return {
                'success': True,
                'message': result.message,
                'solutions_found': result.solutions_found,
            }

        self.get_logger().error(
            f"MTC pick/place failed at '{result.failed_stage or 'unknown'}' "
            f"(MoveIt code {result.error_code}): {result.message}"
        )
        return {
            'success': False,
            'message': result.message,
            'failed_stage': result.failed_stage,
            'error_code': result.error_code,
        }

    def pick_place_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        self.status_text = (
            f"MTC: {feedback.current_stage} "
            f"({feedback.solutions_found} solution(s))"
        )
        self.publish_status()

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
        if not self.coord_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Get Coordinate service not available")
            return None
        req = GetObjectCoordinates.Request()
        req.object_id = target_name
        
        # Async call + safe wait loop
        future = self.coord_client.call_async(req)
        response = self.wait_for_future(future, '/get_coordinates')

        if response and response.success:
            return {'x': response.x, 'y': response.y, 'z': response.z}
        else:
            self.get_logger().error(f"Failed to get coordinates for {target_name}: {response.message if response else 'no response'}")
            return None

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

            action = step.get('action')
            params = step.get('parameters', {})
            success = False
            if not isinstance(action, str) or not action:
                self.get_logger().error(f"Step {i+1} has no valid action")
            elif not isinstance(params, dict):
                self.get_logger().error(
                    f"Step {i+1} parameters must be a JSON object"
                )
            elif action == 'home':
                result = self.call_home_service()
                success = result is not None and result['success']
            elif action == 'move_arm':
                target_name = params.get('target')
                if not isinstance(target_name, str) or not target_name:
                    self.get_logger().error("move_arm requires a string 'target'")
                else:
                    coords = self.get_static_object_coords(target_name)
                    if coords:
                        result = self.call_move_service(coords['x'], coords['y'], coords['z'])
                        success = result is not None and result['success']
            elif action == 'relative_move':
                vector_name = params.get('vector')
                if not isinstance(vector_name, str) or not vector_name:
                    self.get_logger().error(
                        "relative_move requires a string 'vector'"
                    )
                else:
                    vector = self.get_relative_movement_vector(vector_name)
                    if vector:
                        result = self.call_relative_move_service(vector['x'], vector['y'], vector['z'])
                        success = result is not None and result['success']
            elif action == 'gripper':
                try:
                    gripper = float(params['position'])
                except (KeyError, TypeError, ValueError):
                    self.get_logger().error(
                        "gripper requires a numeric 'position'"
                    )
                else:
                    if not 0.0 <= gripper <= 0.85:
                        self.get_logger().error(
                            "gripper position must be between 0.0 and 0.85 "
                            "for physical Gen3 Lite hardware"
                        )
                    else:
                        result = self.call_move_gripper_service(gripper)
                        success = result is not None and result['success']
            elif action == 'pick_and_place':
                object_id = params.get('object')
                destination_id = params.get('destination')
                plan_only = params.get('plan_only', False)
                if not isinstance(object_id, str) or not object_id:
                    self.get_logger().error(
                        "pick_and_place requires a string 'object'"
                    )
                elif not isinstance(destination_id, str) or not destination_id:
                    self.get_logger().error(
                        "pick_and_place requires a string 'destination'"
                    )
                elif not isinstance(plan_only, bool):
                    self.get_logger().error(
                        "pick_and_place 'plan_only' must be true or false"
                    )
                else:
                    result = self.call_pick_place_action(
                        object_id,
                        destination_id,
                        plan_only,
                    )
                    success = result is not None and result['success']
            else:
                self.get_logger().error(f"Unknown recipe action: {action!r}")

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
