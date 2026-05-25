#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from kinova_interfaces.srv import GetObjectCoordinates, GetRelativeMovement
import threading
import time
import json
import os
from ament_index_python.packages import get_package_share_directory

VALID_ACTIONS = ["move_arm", "relative_move", "gripper", "home"]


class EnvironmentServiceClient:

    def __init__(self, node_context):
        self.node = node_context
        self.coord_client = self.node.create_client(GetObjectCoordinates, '/get_coordinates')
        self.movement_client = self.node.create_client(GetRelativeMovement, '/get_relative_movement')

    def wait_for_services(self, timeout_sec=10.0):
        self.node.get_logger().info('Waiting for environment mapping node...')
        if not self.coord_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error('get_coordinates service not available. Is environment_mapping_node running?')
            return False
        if not self.movement_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error('get_relative_movement service not available. Is environment_mapping_node running?')
            return False
        self.node.get_logger().info('Environment mapping node ready.')
        return True

    def resolve_target(self, object_id):
        request = GetObjectCoordinates.Request()
        request.object_id = object_id
        future = self.coord_client.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        if future.result() is not None:
            result = future.result()
            if result.success:
                return {'x': result.x, 'y': result.y, 'z': result.z}
            self.node.get_logger().error(f'Coordinate lookup failed: {result.message}')
        return None

    def resolve_vector(self, move_id):
        request = GetRelativeMovement.Request()
        request.move_id = move_id
        future = self.movement_client.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        if future.result() is not None:
            result = future.result()
            if result.success:
                return {'x': result.x, 'y': result.y, 'z': result.z}
            self.node.get_logger().error(f'Vector lookup failed: {result.message}')
        return None


class RecipeValidator:

    def __init__(self, service_client, node_context):
        self.client = service_client
        self.node = node_context

    def validate(self, steps):
        if not steps:
            return False, "Empty steps array - nothing to execute"

        for i, step in enumerate(steps):
            action = step.get('action')
            step_num = i + 1

            if action not in VALID_ACTIONS:
                return False, f"Step {step_num}: unsupported action '{action}'"

            if action == 'move_arm':
                target = step.get('parameters', {}).get('target')
                if not target:
                    return False, f"Step {step_num}: move_arm missing 'target' parameter"
                coords = self.client.resolve_target(target)
                if coords is None:
                    return False, f"Step {step_num}: target '{target}' not found in coordinate dictionary"
                self.node.get_logger().info(f"  Step {step_num}: '{target}' resolved to x={coords['x']}, y={coords['y']}, z={coords['z']}")

            elif action == 'relative_move':
                vector = step.get('parameters', {}).get('vector')
                if not vector:
                    return False, f"Step {step_num}: relative_move missing 'vector' parameter"
                vec = self.client.resolve_vector(vector)
                if vec is None:
                    return False, f"Step {step_num}: vector '{vector}' not found in movement dictionary"
                self.node.get_logger().info(f"  Step {step_num}: vector '{vector}' resolved to x={vec['x']}, y={vec['y']}, z={vec['z']}")

            elif action == 'gripper':
                position = step.get('parameters', {}).get('position')
                if position is None:
                    return False, f"Step {step_num}: gripper missing 'position' parameter"
                if not (0.0 <= float(position) <= 1.0):
                    return False, f"Step {step_num}: gripper position {position} out of range (0.0 to 1.0)"
                self.node.get_logger().info(f"  Step {step_num}: gripper position {position} valid")

            elif action == 'home':
                self.node.get_logger().info(f"  Step {step_num}: home acknowledged")

        return True, "All steps validated successfully"


class TestScriptInterface(Node):

    def __init__(self):
        super().__init__('test_script_interface')
        self.get_logger().info('Test Script Interface Node Started')

        self.service_client = EnvironmentServiceClient(self)
        self.validator = RecipeValidator(self.service_client, self)
        self.passed = 0
        self.failed = 0

        self.test_thread = threading.Thread(target=self.run_tests, daemon=True)
        self.test_thread.start()

    def load_recipes(self):
        recipes = []
        try:
            pkg_share = get_package_share_directory('kinova_interface')
            data_path = os.path.join(pkg_share, 'data')
            for filename in sorted(os.listdir(data_path)):
                if filename.startswith('recipe_') and filename.endswith('.json'):
                    filepath = os.path.join(data_path, filename)
                    with open(filepath, 'r') as f:
                        data = json.load(f)
                    if 'test_cases' in data:
                        for case in data.get('test_cases', []):
                            recipes.append({
                                'name': f"{data.get('recipe_name', filename)} - {case.get('test_name')}",
                                'steps': case.get('steps', []),
                                'expect_success': case.get('expect_success', True)
                            })
                    else:
                        recipes.append({
                            'name': data.get('recipe_name', filename),
                            'steps': data.get('steps', []),
                            'expect_success': True
                        })
        except Exception as e:
            self.get_logger().error(f'Failed to load recipes: {e}')
        return recipes

    def run_tests(self):
        if not self.service_client.wait_for_services():
            self.get_logger().error('Aborting - services unavailable.')
            return

        recipes = self.load_recipes()
        if not recipes:
            self.get_logger().error('No recipe files found.')
            return

        self.get_logger().info(f'Starting test suite with {len(recipes)} tests...')

        for i, recipe in enumerate(recipes):
            self.get_logger().info(f'Test {i + 1} of {len(recipes)}: {recipe["name"]}')
            success, reason = self.validator.validate(recipe['steps'])

            if success == recipe['expect_success']:
                self.get_logger().info(f'PASS | {reason}')
                self.passed += 1
            else:
                self.get_logger().error(f'FAIL | Expected={recipe["expect_success"]}, Got={success} | {reason}')
                self.failed += 1

            time.sleep(0.3)

        self.print_summary()

    def print_summary(self):
        total = self.passed + self.failed
        self.get_logger().info(f'Test Summary: Total={total} | Passed={self.passed} | Failed={self.failed}')
        self.get_logger().info(f'Result: {"ALL PASSED" if self.failed == 0 else "SOME FAILED"}')


def main(args=None):
    rclpy.init(args=args)
    node = TestScriptInterface()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Test Script Interface...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
