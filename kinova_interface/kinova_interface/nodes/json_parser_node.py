"""ROS 2 node that orchestrates recipe execution.

Loads a recipe (from a file at startup, or dynamically via a service call),
runs its steps one at a time through ArmActions' registered handlers, and
reports progress and failures through telemetry and the recipe log.
"""

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
        """Initialise the parser.

        Args:
            node_context (Node): ROS 2 node used for logging.
        """
        self.recipe = None
        self.node = node_context # Reference to the ROS 2 node for logging

    def load_recipe_from_file(self, recipe_path):
        """Load a recipe from a JSON file on disk.

        Args:
            recipe_path (str): Path to the recipe JSON file.

        Returns:
            bool: True if the file was read and parsed successfully,
                otherwise False.
        """
        try:
            with open(recipe_path, 'r') as f:
                self.recipe = json.load(f)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error loading Recipe JSON file: {e}")
            return False

    def load_recipe_from_service(self, recipe_str):
        """Load a recipe from a JSON string, as received by a service call.

        Args:
            recipe_str (str): JSON-encoded recipe.

        Returns:
            bool: True if the string was parsed successfully, otherwise False.
        """
        try:
            self.recipe = json.loads(recipe_str)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error loading Recipe JSON string: {e}")
            return False

    def get_recipe_steps(self):
        """Return the currently loaded recipe's steps.

        Returns:
            list: The recipe's steps, or an empty list if no recipe is
                loaded.
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
        """Initialise the JSON parser node.

        Creates the recipe parser and ArmActions, sets up telemetry and the
        ``/execute_recipe``/``/reset_environment`` services, and, if a static
        ``recipe`` launch parameter was provided, loads it and schedules its
        execution on a one-shot startup timer.
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
        """Helper to update internal telemetry state and publish immediately.

        Any argument left as None keeps its current value.

        Args:
            state (int, optional): ExtendedStatus state value to set.
            status_text (str, optional): Status message to set.
            success (bool, optional): Most-recent-command-success flag to set.
        """
        if state is not None:
            self.current_state = state
        if status_text is not None:
            self.status_text = status_text
        if success is not None:
            self.command_success = success
        self.publish_status()

    def publish_status(self):
        """Publish the node's current telemetry status.

        The published status includes the node name, current state, status
        message, and whether the most recent command was successful.
        """
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
        """Callback for the dynamic execution service.

        Args:
            request (ExecuteRecipe.Request): Service request containing the
                JSON recipe string.
            response (ExecuteRecipe.Response): Service response to populate.

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
        """Reset objects/obstacles back to their configured defaults and
        clear held-object tracking, without a full middleware restart. In
        the same exec_cb_group as recipe execution, so it can't run
        concurrently with (or interrupt) an in-progress recipe.

        Args:
            request (Trigger.Request): Empty trigger request.
            response (Trigger.Response): Service response to populate.

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
        """Look up and run the handler for one recipe step, logging enough to
        reconstruct what was attempted and what happened for the LLM safety research.

        Args:
            index (int): Zero-based index of this step in the recipe.
            step (dict): The step, with an ``action`` key and optional
                ``parameters`` dict.

        Returns:
            bool: True if the step's handler ran and succeeded, otherwise
                False (including when ``action`` has no registered handler).
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
        # Cleared before every dispatch, so a step that fails without going
        # through ctx.fail() (rather than a genuine bug in the failure
        # path itself) surfaces as "no further detail", not a stale reason
        # left over from an earlier, unrelated failure.
        self.arm_actions.last_error = None
        success = handler(params)
        self.get_logger().info(
            f"[recipe_log] step={index+1} action={action} result={'success' if success else 'failure'}"
        )
        return success

    def execute_recipe(self) -> bool:
        """Entry point for recipe execution with guaranteed exception safety
        and IDLE cleanup - an unhandled exception from a step handler is
        caught here so current_state can never get stuck on BUSY, and is
        reported as a failed recipe rather than crashing the callback.

        Returns:
            bool: True if every step in the currently loaded recipe
                succeeded, otherwise False.
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
        """Sequential step execution loop.

        Runs each step through :meth:`_dispatch_step` in order, updating
        telemetry before and after each one, and stops at the first failure.

        Args:
            steps (list): The recipe's steps, in execution order.

        Returns:
            bool: True if every step succeeded, otherwise False.
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
                if self.arm_actions.last_error:
                    self.status_text += f": {self.arm_actions.last_error}"
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

    Initialises rclpy, creates the node, spins it on a multi-threaded
    executor, and performs node and rclpy shutdown on exit.
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
