#!/usr/bin/env python3
"""
DEV-215: Hardcoded JSON Testing Script
DEV-217: Validate Parser Node Behaviour
Loads recipe files of increasing complexity and validates each step
against the environment mapping node services.
"""

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
        self.coord_client = self.node.create_client(
            GetObjectCoordinates, 'get_coordinates'
        )
        self.movement_client = self.node.create_client(
            GetRelativeMovement, 'get_relative_movement'
        )

    def wait_for_services(self, timeout_sec=10.0):
        self.node.get_logger().info('Waiting for environment mapping node...')
        if not self.coord_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error(
                'get_coordinates service not available. Is environment_mapping_node running?'
            )
            return False
        if not self.movement_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error(
                'get_relative_movement service not available. Is environment_mapping_node running?'
            )
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
            else:
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
            else:
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
                self.node.get_logger().info(
                    f"  Step {step_num}: '{target}' resolved to x={coords['x']}, y={coords['y']}, z={coords['z']}"
                )

            elif action == 'relative_move':
                vector = step.get('parameters', {}).get('vector')
                if not vector:
                    return False, f"Step {step_num}: relative_move missing 'vector' parameter"
                vec = self.client.resolve_vector(vector)
                if vec is None:
                    return False, f"Step {step_num}: vector '{vector}' not found in movement dictionary"
                self.node.get_logger().info(
                    f"  Step {step_num}: vector '{vector}' resolved to x={vec['x']}, y={vec['y']}, z={vec['z']}"
                )

            elif action == 'gripper':
                position = step.get('parameters', {}).get('position')
                if position is None:
                    return False, f"Step {step_num}: gripper missing 'position' parameter"
                if not (0.0 <= float(position) <= 1.0):
                    return False, f"Step {step_num}: gripper position {position} out of range (0.0 to 1.0)"
                self.node.get_logger().info(
                    f"  Step {step_num}: gripper position {position} valid"
                )

            elif action == 'home':
                self.node.get_logger().info(f"  Step {step_num}: home acknowledged")

        return True, "All steps validated successfully"


class TestScriptInterface(Node):

    def __init__(self):
        super().__init__('test_script_interface')
        self.get_logger().info('Test Script Interface Node Started')

        self.service_client = EnvironmentServiceClient(self)
        self.validator = RecipeValidator(self.service_client, self)

        self.results = []

        self.test_thread = threading.Thread(target=self.run_tests, daemon=True)
        self.test_thread.start()

    def load_recipe(self, filename):
        try:
            pkg_share = get_package_share_directory('kinova_interface')
            path = os.path.join(pkg_share, 'recipes', filename)
            with open(path, 'r') as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load recipe {filename}: {e}")
            return None

    def run_single_recipe(self, recipe_file, expect_success=True):
        recipe = self.load_recipe(recipe_file)
        if recipe is None:
            return False, "Failed to load recipe file"
        steps = recipe.get('steps', [])
        return self.validator.validate(steps)

    def run_invalid_cases(self, recipe_file):
        recipe = self.load_recipe(recipe_file)
        if recipe is None:
            return []
        results = []
        for case in recipe.get('test_cases', []):
            success, reason = self.validator.validate(case.get('steps', []))
            results.append({
                'name': case.get('test_name'),
                'expect_success': case.get('expect_success', False),
                'success': success,
                'reason': reason
            })
        return results

    def run_tests(self):
        if not self.service_client.wait_for_services():
            self.get_logger().error('Aborting - services unavailable.')
            return

        self.get_logger().info('Starting DEV-215 test suite...')

        standard_recipes = [
            ('recipe_l1_single_move.json',       True,  'L1 - Single Move'),
            ('recipe_l2_move_and_gripper.json',   True,  'L2 - Move and Gripper'),
            ('recipe_l3_home_move_gripper.json',  True,  'L3 - Home, Move and Gripper'),
            ('recipe_l4_pick_and_place.json',     True,  'L4 - Full Pick and Place'),
            ('recipe_l5_multi_object.json',       True,  'L5 - Multi Object Sequence'),
        ]

        passed = 0
        failed = 0

        for recipe_file, expect_success, label in standard_recipes:
            self.get_logger().info(f'Running: {label}')
            success, reason = self.run_single_recipe(recipe_file, expect_success)

            if success == expect_success:
                self.get_logger().info(f'PASS | {reason}')
                passed += 1
                result = 'PASS'
            else:
                self.get_logger().error(
                    f'FAIL | Expected={expect_success}, Got={success} | {reason}'
                )
                failed += 1
                result = 'FAIL'

            self.results.append({
                'recipe': label,
                'steps': len(self.load_recipe(recipe_file).get('steps', [])),
                'expect_success': expect_success,
                'result': result,
                'reason': reason
            })
            time.sleep(0.3)

        self.get_logger().info('Running: L6 - Invalid Recipes')
        invalid_results = self.run_invalid_cases('recipe_l6_invalid.json')
        for r in invalid_results:
            if r['success'] == r['expect_success']:
                self.get_logger().info(f"PASS | {r['name']} | {r['reason']}")
                passed += 1
                result = 'PASS'
            else:
                self.get_logger().error(
                    f"FAIL | {r['name']} | Expected={r['expect_success']}, Got={r['success']} | {r['reason']}"
                )
                failed += 1
                result = 'FAIL'

            self.results.append({
                'recipe': f"L6 - {r['name']}",
                'steps': len([]),
                'expect_success': r['expect_success'],
                'result': result,
                'reason': r['reason']
            })

        self.print_summary(passed, failed)

    def print_summary(self, passed, failed):
        total = passed + failed
        self.get_logger().info('Test Summary:')
        self.get_logger().info(f'Total: {total} | Passed: {passed} | Failed: {failed}')
        self.get_logger().info(
            f'Result: {"ALL PASSED" if failed == 0 else "SOME FAILED"}'
        )
        self.get_logger().info('Full results for spreadsheet:')
        self.get_logger().info('Recipe | Steps | Expected | Result | Reason')
        for r in self.results:
            self.get_logger().info(
                f"{r['recipe']} | {r['steps']} | {r['expect_success']} | {r['result']} | {r['reason']}"
            )


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
