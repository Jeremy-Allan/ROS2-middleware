import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
import os
import json
import traceback
from ament_index_python.packages import get_package_share_directory
#Services
from kinova_interfaces.srv import ExecuteRecipe
from kinova_interfaces.msg import ExtendedStatus
from std_srvs.srv import Trigger

from kinova_interface.actions.arm_actions import ArmActions

class JsonParser:
    """Helper class to handle JSON loading."""
    def __init__(self, node_context):
        """Initialise the JSON recipe parser.

        Stores the ROS 2 node context used for logging and initialises the
        currently loaded recipe.

        Args:
            node_context (Node): ROS 2 node providing logging and node context.

        Returns:
            None
        """
        self.recipe = None
        self.node = node_context # Reference to the ROS 2 node for logging

    def load_recipe_from_file(self, recipe_path):
        """Load a recipe from a JSON file.

        Reads and parses the specified JSON file and stores the resulting recipe
        for later execution.

        Args:
            recipe_path (str): Path to the JSON recipe file.

        Returns:
            bool: ``True`` if the recipe was loaded successfully, otherwise
                ``False``.
        """
        try:
            with open(recipe_path, 'r') as f:
                self.recipe = json.load(f)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error loading Recipe JSON file: {e}")
            return False

    def load_recipe_from_service(self, recipe_str):
        """Load a recipe from a JSON string.

        Parses the supplied JSON string and stores the resulting recipe for
        later execution.

        Args:
            recipe_str (str): JSON-encoded recipe.

        Returns:
            bool: ``True`` if the recipe was parsed successfully, otherwise
                ``False``.
        """
        try:
            self.recipe = json.loads(recipe_str)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error loading Recipe JSON string: {e}")
            return False

    def get_recipe_steps(self):
        """Return the steps from the currently loaded recipe.

        If no recipe is loaded, an empty list is returned.

        Returns:
            list: Recipe steps, or an empty list when no recipe is loaded.
        """
        if not self.recipe:
            return []
        return self.recipe.get('steps', [])

class JsonParserNode(Node):
    """ROS 2 Node that orchestrates tasks based on a JSON recipe.

    The actual arm actions (what 'home', 'pickup', etc. do, and which
    hardware/environment services they call) live in ArmActions
    (arm_actions.py). This node only owns recipe loading, the step-by-step
    execution loop, and telemetry/service plumbing."""

    STEP_SETTLE_DELAY_SEC = 0.5

    def __init__(self):
        """Initialise the JSON parser ROS 2 node.

        Creates the recipe parser, arm action interface, telemetry handling,
        service interfaces, and optional startup recipe loading.

        Returns:
            None
        """
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

        self.get_logger().info("JSON Parser Node Online.")

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
                    # nodes/ -> kinova_interface/ (python pkg) -> kinova_interface/ (ROS pkg, holds recipes/)
                    base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                    recipe_path = os.path.join(base_path, 'recipes', recipe_file)

        # 6. If a static recipe was provided, execute it on startup using a one-shot Timer
        if recipe_path:
            self.get_logger().info(f"Loading static recipe from {recipe_path}")
            if self.parser.load_recipe_from_file(recipe_path):
                self.startup_timer = self.create_timer(2.0, self.startup_timer_callback, callback_group=self.exec_cb_group)
            else:
                self.get_logger().error(f"Failed to load recipe from {recipe_path}")

    def _update_node_status(self, state=None, status_text=None, success=None):
        """Update the node's internal telemetry state and publish it immediately.

        Only the supplied state, status text, or success value is changed. Any
        values omitted by the caller retain their current values.

        Args:
            state (int, optional): ExtendedStatus state value to assign.
            status_text (str, optional): Status message to publish.
            success (bool, optional): Whether the most recent command succeeded.

        Returns:
            None
        """
        if state is not None:
            self.current_state = state
        if status_text is not None:
            self.status_text = status_text
        if success is not None:
            self.command_success = success
        self.publish_status()

    def publish_status(self):
        """Publish the current recipe execution status.

        Publishes the node's current state, status message, and most recent
        command result through the telemetry status interface.

        Returns:
            None
        """
        msg = ExtendedStatus()
        msg.node_name = self.get_name()
        msg.state = self.current_state
        msg.status_message = self.status_text
        msg.last_command_valid = self.command_success
        self.status_pub.publish(msg)

    def startup_timer_callback(self):
        """Start the initial recipe after the startup delay.

        Cancels the one-shot startup timer and begins execution of the recipe
        that was loaded during node initialisation.

        Returns:
            None
        """
        self.startup_timer.cancel()
        self.get_logger().info("Starting initial recipe sequence...")
        self.execute_recipe()

    def execute_recipe_callback(self, request, response):
        """Handle a dynamic recipe execution service request.

        Loads the JSON recipe supplied by the service request, executes the
        recipe, and populates the service response with the parsing and
        execution result.

        Args:
            request (ExecuteRecipe.Request): Service request containing the JSON
                recipe string.
            response (ExecuteRecipe.Response): Service response populated with
                the parsing and execution result.

        Returns:
            ExecuteRecipe.Response: The populated service response.
        """
        self.get_logger().info("Received dynamic recipe execution request.")
        self.get_logger().debug(f"Payload recipe received: {request.recipe_json}")

        if not self.parser.load_recipe_from_service(request.recipe_json):
            self.get_logger().error("Failed to parse JSON recipe string.")
            response.success = False
            response.message = "Failed to parse JSON recipe string."
            self._update_node_status(ExtendedStatus.STATE_IDLE, "Failed to parse dynamic JSON recipe", success=False)
            return response

        self.get_logger().info("Successfully parsed JSON recipe. Executing...")
        success = self.execute_recipe()

        response.success = success
        if success:
            self.get_logger().info("Returning Success to client.")
            response.message = "Recipe executed successfully."
            self._update_node_status(status_text="Recipe execution complete (Success)", success=True)
        else:
            self.get_logger().error(f"Returning Failure to client: {self.status_text}")
            response.message = self.status_text
            self._update_node_status(success=False)

        return response

    def reset_environment_callback(self, request, response):
        """Reset the configured environment to its default state.

        Requests the arm action interface to restore configured objects and
        obstacles and clear held-object tracking. The operation runs in the same
        mutually exclusive callback group as recipe execution.

        Args:
            request (Trigger.Request): Service request for the environment reset.
            response (Trigger.Response): Service response populated with the reset
                result and message.

        Returns:
            Trigger.Response: The populated service response.
        """
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
        """Dispatch and execute one recipe step.

        Looks up the action handler associated with the requested action, executes
        it with the supplied parameters, and records the attempted action and
        result in the recipe log.

        Args:
            index (int): Zero-based index of the recipe step.
            step (dict): Recipe step containing an ``action`` field and optional
                ``parameters`` dictionary.

        Returns:
            bool: ``True`` if the action handler executes successfully, otherwise
                ``False``.
        """
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

    def execute_recipe(self) -> bool:
        """Execute the currently loaded recipe with exception-safe cleanup.

        Retrieves the loaded recipe steps and executes them sequentially. Any
        unhandled exception from a step handler is caught and reported as a failed
        recipe. The node state is returned to IDLE after execution regardless of
        whether the recipe succeeds or fails.

        Returns:
            bool: ``True`` if all recipe steps execute successfully, otherwise
                ``False``.
        """
        steps = self.parser.get_recipe_steps()
        if not steps:
            self.get_logger().error("No executable steps found or recipe failed to load.")
            self._update_node_status(ExtendedStatus.STATE_IDLE, "No executable steps in recipe", success=False)
            return False

        try:
            return self._run_steps(steps)
        except Exception as e:
            tb = traceback.format_exc()
            self.get_logger().error(f"Unhandled exception during recipe execution: {e}\n{tb}")
            self.status_text = f"Recipe aborted due to exception: {e}"
            self.command_success = False
            return False
        finally:
            self.current_state = ExtendedStatus.STATE_IDLE
            self.publish_status()

    def _run_steps(self, steps: list) -> bool:
        """Execute recipe steps sequentially.

        Updates telemetry before and after each step, dispatches each action through
        the configured arm action handler, and stops execution when a step fails.

        Args:
            steps (list): Ordered list of recipe step dictionaries to execute.

        Returns:
            bool: ``True`` if every recipe step succeeds, otherwise ``False``.
        """
        recipe_name = self.parser.recipe.get('recipe_name', 'Unnamed')
        self.get_logger().info(
            f"[recipe_log] event=start timestamp={time.time():.3f} recipe={recipe_name} steps={len(steps)}"
        )
        self.get_logger().info(f"--- Starting Automated Sequence ({len(steps)} steps) ---")
        self._update_node_status(ExtendedStatus.STATE_BUSY, f"Executing recipe: {recipe_name}", success=True)

        for i, step in enumerate(steps):
            self.get_logger().info(f"[Step {i+1}] {step.get('description', '')}")
            self._update_node_status(status_text=f"Step {i+1}/{len(steps)}: {step.get('description', '')}")

            success = self._dispatch_step(i, step)

            if not success:
                self.get_logger().error(f"Failed at step {i+1}: {step.get('action')}")
                self.status_text = f"Recipe failed at step {i+1} ({step.get('action')})"
                self.command_success = False
                self.get_logger().info(
                    f"[recipe_log] event=end timestamp={time.time():.3f} recipe={recipe_name} result=failure"
                )
                return False

            self.get_logger().info(f"Step {i+1} completed successfully.")
            if i < len(steps) - 1:
                time.sleep(self.STEP_SETTLE_DELAY_SEC)

        self.status_text = "Recipe execution complete (Success)"
        self.command_success = True
        self.get_logger().info(
            f"[recipe_log] event=end timestamp={time.time():.3f} recipe={recipe_name} result=success"
        )
        self.get_logger().info("--- All Tasks Completed ---")
        return True

def main():
    """Start the JSON parser ROS 2 node.

    Initialises rclpy, creates the JSON parser node, spins the node, and
    performs node and ROS 2 shutdown cleanup.

    Returns:
        None
    """
    
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
