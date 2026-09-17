import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
import os
import json
from ament_index_python.packages import get_package_share_directory
#Services
from kinova_interfaces.srv import ExecuteRecipe
from kinova_interfaces.msg import ExtendedStatus
from std_srvs.srv import Trigger

from kinova_interface.arm_actions import ArmActions

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
    """ROS 2 Node that orchestrates tasks based on a JSON recipe.

    The actual arm actions (what 'home', 'pickup', etc. do, and which
    hardware/environment services they call) live in ArmActions
    (arm_actions.py). This node only owns recipe loading, the step-by-step
    execution loop, and telemetry/service plumbing."""
    def __init__(self):
        super().__init__('json_parser_node')

        # 1. Callback Groups
        # Reentrant group for general service clients to allow multiple responses
        self.cb_group = ReentrantCallbackGroup()
        # Mutually exclusive group for the execution sequence to ensure one recipe at a time
        self.exec_cb_group = MutuallyExclusiveCallbackGroup()

        # 2. Arm actions: owns the hardware/environment service clients and
        # the dictionary of named action handlers ('home', 'pickup', etc.)
        self.arm_actions = ArmActions(self)

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
        self.reset_srv = self.create_service(Trigger, '/reset_environment', self.reset_environment_callback, callback_group=self.exec_cb_group)

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

    def reset_environment_callback(self, request, response):
        """Reset objects/obstacles back to their configured defaults and
        clear held-object tracking, without a full middleware restart. In
        the same exec_cb_group as recipe execution, so it can't run
        concurrently with (or interrupt) an in-progress recipe."""
        self.get_logger().info("Received environment reset request.")
        success, message = self.arm_actions.reset_environment()
        response.success = success
        response.message = message
        if success:
            self.get_logger().info(f"Environment reset: {message}")
        else:
            self.get_logger().error(f"Environment reset failed: {message}")
        return response

    def _dispatch_step(self, index, step):
        """Look up and run the handler for one recipe step, logging enough to
        reconstruct what was attempted and what happened for the LLM safety research."""
        action = step.get('action')
        params = step.get('parameters', {})
        timestamp = time.time()

        handler = self.arm_actions.handlers.get(action)
        if handler is None:
            self.get_logger().error(
                f"[recipe_log] step={index+1} timestamp={timestamp:.3f} action={action} "
                f"parameters={params} accepted=False result=unknown_action"
            )
            return False

        self.get_logger().info(
            f"[recipe_log] step={index+1} timestamp={timestamp:.3f} action={action} "
            f"parameters={params} accepted=True"
        )
        success = handler(params)
        self.get_logger().info(
            f"[recipe_log] step={index+1} action={action} result={'success' if success else 'failure'}"
        )
        return success

    def execute_recipe(self):
        """Core execution logic."""
        steps = self.parser.get_recipe_steps()
        if not steps:
            self.get_logger().error("No executable steps found or recipe failed to load.")
            self.command_success = False
            self.status_text = "No executable steps in recipe"
            self.publish_status()
            return False

        recipe_name = self.parser.recipe.get('recipe_name', 'Unnamed')
        self.get_logger().info(
            f"[recipe_log] event=start timestamp={time.time():.3f} recipe={recipe_name} steps={len(steps)}"
        )
        self.get_logger().info(f"--- Starting Automated Sequence ({len(steps)} steps) ---")
        self.current_state = ExtendedStatus.STATE_BUSY
        self.status_text = f"Executing recipe: {recipe_name}"
        self.publish_status()

        all_success = True
        i = 0
        for i, step in enumerate(steps):
            self.get_logger().info(f"[Step {i+1}] {step.get('description', '')}")
            self.status_text = f"Step {i+1}/{len(steps)}: {step.get('description', '')}"
            self.publish_status()

            success = self._dispatch_step(i, step)

            if success:
                self.get_logger().info(f"Step {i+1} completed successfully.")
                time.sleep(0.5)
            else:
                self.get_logger().error(f"Failed at step {i+1}: {step.get('action')}")
                all_success = False
                break

        self.current_state = ExtendedStatus.STATE_IDLE
        self.command_success = all_success
        if all_success:
            self.status_text = "Recipe execution complete (Success)"
        else:
            self.status_text = f"Recipe failed at step {i+1}"
        self.publish_status()
        self.get_logger().info(
            f"[recipe_log] event=end timestamp={time.time():.3f} recipe={recipe_name} "
            f"result={'success' if all_success else 'failure'}"
        )
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
