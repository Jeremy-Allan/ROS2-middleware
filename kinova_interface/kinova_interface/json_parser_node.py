import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
import os
import json
from geometry_msgs.msg import Quaternion
from ament_index_python.packages import get_package_share_directory
#Services
from kinova_interfaces.srv import GetRelativeMovement, GetObjectInfo, ExecuteRecipe, HomeArm, MoveArm, MoveGripper, RelativeMove, AttachObject, DetachObject, UpdateObjectPose
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

        # 2. Hardware Interface Service Clients
        self.home_client = self.create_client(HomeArm, '/kinova_hardware_client/home_arm', callback_group=self.cb_group)
        self.move_arm_client = self.create_client(MoveArm, '/kinova_hardware_client/move_arm', callback_group=self.cb_group)
        self.move_gripper_client = self.create_client(MoveGripper, '/kinova_hardware_client/move_gripper', callback_group=self.cb_group)
        self.relative_move_client = self.create_client(RelativeMove, '/kinova_hardware_client/relative_move', callback_group=self.cb_group)

        # 3. Environment Mapping Service Clients
        self.relative_client = self.create_client(GetRelativeMovement, '/get_relative_movement', callback_group=self.cb_group)
        self.info_client = self.create_client(GetObjectInfo, '/get_object_info', callback_group=self.cb_group)
        self.attach_client = self.create_client(AttachObject, '/attach_object', callback_group=self.cb_group)
        self.detach_client = self.create_client(DetachObject, '/detach_object', callback_group=self.cb_group)
        self.update_pose_client = self.create_client(UpdateObjectPose, '/update_object_pose', callback_group=self.cb_group)

        # 4. Initialize Parser & Action Dispatch Map
        self.parser = JsonParser(self)
        self._action_handlers = {
            'home': self._handle_home,
            'move_arm': self._handle_move_arm,
            'relative_move': self._handle_relative_move,
            'gripper': self._handle_gripper,
            'pickup': self._handle_pickup,
            'dropoff': self._handle_dropoff,
        }

        # 5. Telemetry Setup
        self.status_pub = self.create_publisher(ExtendedStatus, '/status/node_report', 10)
        self.status_timer = self.create_timer(0.5, self.publish_status, callback_group=self.cb_group)
        self.current_state = ExtendedStatus.STATE_IDLE
        self.status_text = "JSON Parser Online & Ready"
        self.command_success = True

        # 6. Service to execute recipes dynamically
        self.execute_srv = self.create_service(ExecuteRecipe, '/execute_recipe', self.execute_recipe_callback, callback_group=self.exec_cb_group)

        # 7. Static recipe parameter & startup timer (only after everything is fully constructed)
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

        if recipe_path:
            self.get_logger().info(f"Loading static recipe from {recipe_path}")
            if self.parser.load_recipe_from_file(recipe_path):
                self.startup_timer = self.create_timer(2.0, self.startup_timer_callback, callback_group=self.exec_cb_group)
            else:
                self.get_logger().error(f"Failed to load recipe from {recipe_path}")

        self.get_logger().info("JSON Parser Node Online.")

    def wait_for_future(self, future, service_name, timeout_sec=10.0):
        """Safely wait for an async service call future to complete without deadlocking the executor."""
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > timeout_sec:
                self.get_logger().error(f"Timed out waiting for {service_name}")
                return None
            time.sleep(0.01)
        return future.result() if future.done() else None

    def _update_node_status(self, state=None, status_text=None, success=None):
        """Helper to update internal telemetry state and publish immediately."""
        if state is not None:
            self.current_state = state
        if status_text is not None:
            self.status_text = status_text
        if success is not None:
            self.command_success = success
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
            self._update_node_status(ExtendedStatus.STATE_IDLE, "Failed to parse dynamic JSON recipe", success=False)
            return response

        self.get_logger().info("Successfully parsed JSON recipe. Executing...")
        success = self.execute_recipe()

        response.success = success
        response.message = self.status_text
        if success:
            self.get_logger().info("Returning Success to client.")
        else:
            self.get_logger().error(f"Returning Failure to client: {self.status_text}")

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

    def call_home_service(self) -> tuple[bool, str]:
        if not self.home_client.wait_for_service(timeout_sec=5.0):
            msg = "Home Arm service not available"
            self.get_logger().error(msg)
            return False, msg

        req = HomeArm.Request()
        future = self.home_client.call_async(req)
        response = self.wait_for_future(future, '/kinova_hardware_client/home_arm')

        if response is None:
            msg = "Timed out waiting for Home Arm service response"
            self.get_logger().error(msg)
            return False, msg
        if response.success:
            return True, response.message or "Homed arm successfully"
        self.get_logger().error(f"Failed to move Home: {response.message}")
        return False, response.message or "Home Arm returned failure"

    def call_move_service(self, x: float, y: float, z: float) -> tuple[bool, str]:
        if not self.move_arm_client.wait_for_service(timeout_sec=5.0):
            msg = "Move Arm service not available"
            self.get_logger().error(msg)
            return False, msg

        req = MoveArm.Request()
        req.x = x
        req.y = y
        req.z = z

        future = self.move_arm_client.call_async(req)
        response = self.wait_for_future(future, '/kinova_hardware_client/move_arm')

        if response is None:
            msg = f"Timed out waiting for Move Arm service to ({x}, {y}, {z})"
            self.get_logger().error(msg)
            return False, msg
        if response.success:
            return True, response.message or f"Moved to ({x}, {y}, {z})"
        self.get_logger().error(f"Failed to move to ({x}, {y}, {z}): {response.message}")
        return False, response.message or f"Move to ({x}, {y}, {z}) failed"

    def call_relative_move_service(self, vx: float, vy: float, vz: float) -> tuple[bool, str]:
        if not self.relative_move_client.wait_for_service(timeout_sec=5.0):
            msg = "Relative Move service not available"
            self.get_logger().error(msg)
            return False, msg

        req = RelativeMove.Request()
        req.vx = vx
        req.vy = vy
        req.vz = vz

        future = self.relative_move_client.call_async(req)
        response = self.wait_for_future(future, '/kinova_hardware_client/relative_move')

        if response is None:
            msg = f"Timed out waiting for Relative Move service to ({vx}, {vy}, {vz})"
            self.get_logger().error(msg)
            return False, msg
        if response.success:
            return True, response.message or f"Moved relative ({vx}, {vy}, {vz})"
        self.get_logger().error(f"Failed relative move ({vx}, {vy}, {vz}): {response.message}")
        return False, response.message or f"Relative move ({vx}, {vy}, {vz}) failed"

    def call_move_gripper_service(self, position: float) -> tuple[bool, str]:
        if not self.move_gripper_client.wait_for_service(timeout_sec=5.0):
            msg = "Move Gripper service not available"
            self.get_logger().error(msg)
            return False, msg

        req = MoveGripper.Request()
        req.position = position

        future = self.move_gripper_client.call_async(req)
        response = self.wait_for_future(future, '/kinova_hardware_client/move_gripper')

        if response is None:
            msg = f"Timed out waiting for Move Gripper service to {position}"
            self.get_logger().error(msg)
            return False, msg
        if response.success:
            return True, response.message or f"Moved gripper to {position}"
        self.get_logger().error(f"Failed to move gripper to {position}: {response.message}")
        return False, response.message or f"Move gripper to {position} failed"

    def attach_object(self, obj_id: str) -> tuple[bool, str]:
        """Remove object from planning scene (allow collision) via the environment mapping node."""
        if not self.attach_client.wait_for_service(timeout_sec=5.0):
            msg = "Attach service not available"
            self.get_logger().error(msg)
            return False, msg

        req = AttachObject.Request()
        req.object_id = obj_id
        future = self.attach_client.call_async(req)
        response = self.wait_for_future(future, '/attach_object')

        if response is None:
            msg = f"Timed out waiting for attach service for '{obj_id}'"
            self.get_logger().error(msg)
            return False, msg
        if response.success:
            self.get_logger().info(f"Attached object '{obj_id}' (removed from scene)")
            return True, response.message or f"Attached '{obj_id}'"
        self.get_logger().error(f"Failed to attach '{obj_id}': {response.message}")
        return False, response.message or f"Failed to attach '{obj_id}'"

    def detach_object(self, obj_id: str) -> tuple[bool, str]:
        """Add object back to planning scene via the environment mapping node."""
        if not self.detach_client.wait_for_service(timeout_sec=5.0):
            msg = "Detach service not available"
            self.get_logger().error(msg)
            return False, msg

        req = DetachObject.Request()
        req.object_id = obj_id
        future = self.detach_client.call_async(req)
        response = self.wait_for_future(future, '/detach_object')

        if response is None:
            msg = f"Timed out waiting for detach service for '{obj_id}'"
            self.get_logger().error(msg)
            return False, msg
        if response.success:
            self.get_logger().info(f"Detached object '{obj_id}' (added back to scene)")
            return True, response.message or f"Detached '{obj_id}'"
        self.get_logger().error(f"Failed to detach '{obj_id}': {response.message}")
        return False, response.message or f"Failed to detach '{obj_id}'"

    def update_object_pose(self, obj_id: str, x: float, y: float, z: float, orientation=None) -> tuple[bool, str]:
        if not self.update_pose_client.wait_for_service(timeout_sec=5.0):
            msg = "Update Object Pose service not available"
            self.get_logger().error(msg)
            return False, msg

        req = UpdateObjectPose.Request()
        req.object_id = obj_id
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z

        if orientation:
            q = Quaternion()
            q.x = orientation['x']
            q.y = orientation['y']
            q.z = orientation['z']
            q.w = orientation['w']
            req.pose.orientation = q
        else:
            req.pose.orientation.x = 0.0
            req.pose.orientation.y = 0.0
            req.pose.orientation.z = 0.0
            req.pose.orientation.w = 1.0

        future = self.update_pose_client.call_async(req)
        response = self.wait_for_future(future, '/update_object_pose')

        if response is None:
            msg = f"Timed out waiting for update pose service for '{obj_id}'"
            self.get_logger().error(msg)
            return False, msg
        if response.success:
            return True, response.message or f"Updated pose for '{obj_id}'"
        self.get_logger().error(f"Failed to update pose for {obj_id}: {response.message}")
        return False, response.message or f"Failed to update pose for '{obj_id}'"

    STEP_SETTLE_DELAY_SEC = 0.5

    # Action Handlers (Dispatch Table)
    
    def _handle_home(self, params: dict) -> tuple[bool, str]:
        """
        Execute the 'home' action to move the robot arm to its predefined home pose.

        JSON parameters:
            None required.
        """
        return self.call_home_service()

    def _handle_move_arm(self, params: dict) -> tuple[bool, str]:
        """
        Execute the 'move_arm' action to move the end-effector to the coordinates of a named target.

        JSON parameters:
            target (str, required): Name of the target object/landmark in the environment.
        """
        target_name = params.get('target')
        if not target_name:
            return False, "Missing 'target' parameter for move_arm"
        coords = self.get_static_object_coords(target_name)
        if not coords:
            return False, f"Could not resolve coordinates for target '{target_name}'"
        ok, msg = self.call_move_service(coords['x'], coords['y'], coords['z'])
        if not ok:
            return False, f"Move to '{target_name}' failed: {msg}"
        return True, f"Moved arm to '{target_name}'"

    def _handle_relative_move(self, params: dict) -> tuple[bool, str]:
        """
        Execute the 'relative_move' action using a named 3D translation vector.

        JSON parameters:
            vector (str, required): Name of the relative vector registered in the environment.
        """
        vector_name = params.get('vector')
        if not vector_name:
            return False, "Missing 'vector' parameter for relative_move"
        vector = self.get_relative_movement_vector(vector_name)
        if not vector:
            return False, f"Could not resolve movement vector for '{vector_name}'"
        ok, msg = self.call_relative_move_service(vector['x'], vector['y'], vector['z'])
        if not ok:
            return False, f"Relative move '{vector_name}' failed: {msg}"
        return True, f"Executed relative move '{vector_name}'"

    def _handle_gripper(self, params: dict) -> tuple[bool, str]:
        """
        Execute the 'gripper' action to move the gripper to a specified position.

        JSON parameters:
            position (float, required): Target gripper position (0.0 = fully open, ~0.8-1.0 = fully closed).
        """
        if 'position' not in params:
            return False, "Missing 'position' parameter for gripper"
        try:
            position = float(params['position'])
        except (ValueError, TypeError) as e:
            return False, f"Invalid gripper position '{params.get('position')}': {e}"
        ok, msg = self.call_move_gripper_service(position)
        if not ok:
            return False, f"Move gripper to {position} failed: {msg}"
        return True, f"Moved gripper to {position}"

    def _handle_pickup(self, params: dict) -> tuple[bool, str]:
        """
        Execute the composite 'pickup' action sequence:
          1. Open gripper to `open_position`.
          2. (Optional) Hover at `pre_offset` above target.
          3. Descend to target coordinates.
          4. Close gripper to `close_position`.
          5. Attach object in planning scene (allow collision).

        JSON parameters:
            target (str, required): Name of the target object to pick up.
            open_position (float, optional, default=0.0): Gripper open position.
            close_position (float, optional, default=0.8): Gripper grasp position.
            pre_offset (float, optional, default=0.0): Z-axis hover offset before descending.
        """
        target_name = params.get('target')
        if not target_name:
            return False, "Missing 'target' parameter for pickup"

        try:
            open_pos = float(params.get('open_position', 0.0))
            close_pos = float(params.get('close_position', 0.8))
            pre_offset = float(params.get('pre_offset', 0.0))
        except (ValueError, TypeError) as e:
            return False, f"Invalid numeric parameter in pickup: {e}"

        coords = self.get_static_object_coords(target_name)
        if not coords:
            return False, f"Could not resolve coordinates for '{target_name}'"

        # 1. Open gripper
        ok, msg = self.call_move_gripper_service(open_pos)
        if not ok:
            return False, f"Failed to open gripper for pickup: {msg}"

        # 2. Optional pre-approach hover above target
        if pre_offset > 0.0:
            ok, msg = self.call_move_service(coords['x'], coords['y'], coords['z'] + pre_offset)
            if not ok:
                return False, f"Failed pre-approach move for '{target_name}': {msg}"

        # 3. Descend to object position
        ok, msg = self.call_move_service(coords['x'], coords['y'], coords['z'])
        if not ok:
            return False, f"Failed to move to '{target_name}' position: {msg}"

        # 4. Close gripper
        ok, msg = self.call_move_gripper_service(close_pos)
        if not ok:
            return False, f"Failed to close gripper on '{target_name}': {msg}"

        # 5. Attach object in planning scene
        ok, msg = self.attach_object(target_name)
        if not ok:
            return False, f"Failed to attach '{target_name}' in planning scene: {msg}"

        return True, f"Picked up '{target_name}' successfully"

    def _handle_dropoff(self, params: dict) -> tuple[bool, str]:
        """
        Execute the composite 'dropoff' action sequence:
          1. Move end-effector to destination coordinates + `place_offset` in Z.
          2. Open gripper to `open_position`.
          3. If `target` specified: update object pose in environment and detach in planning scene.

        JSON parameters:
            destination (str, required): Name of destination location/surface.
            target (str, optional): Name of object being placed to update pose and restore in planning scene.
            open_position (float, optional, default=0.0): Gripper open position to release object.
            place_offset (float, optional, default=0.1): Z-axis hover offset above destination.
        """
        target_name = params.get('target')
        destination_name = params.get('destination')
        if not destination_name:
            return False, "dropoff action requires 'destination' parameter"

        try:
            open_pos = float(params.get('open_position', 0.0))
            place_offset = float(params.get('place_offset', 0.1))
        except (ValueError, TypeError) as e:
            return False, f"Invalid numeric parameter in dropoff: {e}"

        dest_coords = self.get_static_object_coords(destination_name)
        if not dest_coords:
            return False, f"Could not resolve destination '{destination_name}'"

        px = dest_coords['x']
        py = dest_coords['y']
        pz = dest_coords['z'] + place_offset

        # 1. Move to offset position above destination
        ok, msg = self.call_move_service(px, py, pz)
        if not ok:
            return False, f"Failed to move above '{destination_name}': {msg}"

        # 2. Open gripper to release
        ok, msg = self.call_move_gripper_service(open_pos)
        if not ok:
            return False, f"Failed to open gripper at '{destination_name}': {msg}"

        # 3. Update pose and detach object if target was specified
        if target_name:
            obj_info = self.get_object_info(target_name)
            orient = obj_info['pose']['orientation'] if obj_info else None
            ok, msg = self.update_object_pose(target_name, dest_coords['x'], dest_coords['y'], dest_coords['z'], orient)
            if not ok:
                return False, f"Failed to update pose for '{target_name}' at '{destination_name}': {msg}. Detach aborted to prevent planning scene corruption."

            ok, msg = self.detach_object(target_name)
            if not ok:
                return False, f"Failed to detach '{target_name}' in planning scene: {msg}"

            return True, f"Placed '{target_name}' at '{destination_name}' successfully"

        return True, f"Executed dropoff move and opened gripper at '{destination_name}' successfully"

    def execute_recipe(self) -> bool:
        """Entry point for recipe execution with guaranteed exception safety and IDLE cleanup."""
        steps = self.parser.get_recipe_steps()
        if not steps:
            self.get_logger().error("No executable steps found or recipe failed to load.")
            self._update_node_status(ExtendedStatus.STATE_IDLE, "No executable steps in recipe", success=False)
            return False

        try:
            return self._run_steps(steps)
        except Exception as e:
            self.get_logger().error(f"Unhandled exception during recipe execution: {e}", exc_info=True)
            self.status_text = f"Recipe aborted due to exception: {e}"
            self.command_success = False
            return False
        finally:
            self.current_state = ExtendedStatus.STATE_IDLE
            self.publish_status()

    def _run_steps(self, steps: list) -> bool:
        """Sequential step execution loop."""
        recipe_name = self.parser.recipe.get('recipe_name', 'Unnamed')
        self.get_logger().info(f"--- Starting Automated Sequence: '{recipe_name}' ({len(steps)} steps) ---")
        self._update_node_status(ExtendedStatus.STATE_BUSY, f"Executing recipe: {recipe_name}", success=True)

        for i, step in enumerate(steps, start=1):
            action = step.get('action')
            desc = step.get('description', f"Step {i}")
            params = step.get('parameters', {})

            self.get_logger().info(f"[Step {i}/{len(steps)}] {desc} (action: '{action}')")
            self._update_node_status(ExtendedStatus.STATE_BUSY, f"Step {i}/{len(steps)}: {desc}")

            handler = self._action_handlers.get(action)
            if not handler:
                err = f"Unknown/unsupported action '{action}' at step {i}"
                self.get_logger().error(err)
                self.status_text = err
                self.command_success = False
                return False

            step_success, msg = handler(params)
            if not step_success:
                self.get_logger().error(f"Failed at step {i} ({action}): {msg}")
                self.status_text = f"Recipe failed at step {i}: {msg}"
                self.command_success = False
                return False

            self.get_logger().info(f"Step {i} completed successfully: {msg}")
            if i < len(steps):
                time.sleep(self.STEP_SETTLE_DELAY_SEC)

        self.status_text = "Recipe execution complete (Success)"
        self.command_success = True
        self.get_logger().info("--- All Tasks Completed Successfully ---")
        return True

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
