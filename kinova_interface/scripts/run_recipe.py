#!/usr/bin/env python3
"""Trigger /execute_recipe directly from a recipe JSON file's path.

Exists specifically to avoid `ros2 service call`'s CLI YAML-quoting
pitfalls: that command wraps the recipe JSON in a single-quoted YAML
string ("{recipe_json: '<json>'}"), and YAML's single-quote syntax treats
a bare apostrophe as the end of the string - so any recipe whose
name/description contains one (e.g. "joint_5's real crossing") breaks the
parse entirely. This script reads the file and sends it as a service
request field directly via rclpy, so the recipe's actual content never
passes through any string-quoting layer at all.

Usage (installed with the package; the middleware must already be running):
    ros2 run kinova_interface run_recipe.py <path/to/recipe.json>
"""
import sys

import rclpy
from rclpy.node import Node
from kinova_interfaces.srv import ExecuteRecipe


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <recipe_json_path>")
        sys.exit(1)

    recipe_path = sys.argv[1]
    with open(recipe_path, 'r') as f:
        recipe_json = f.read()

    rclpy.init()
    node = Node('run_recipe_cli')
    client = node.create_client(ExecuteRecipe, '/execute_recipe')

    if not client.wait_for_service(timeout_sec=10.0):
        print("ERROR: /execute_recipe service not available - is the middleware running?")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    request = ExecuteRecipe.Request()
    request.recipe_json = recipe_json

    print(f"Executing {recipe_path} ...")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()

    node.destroy_node()
    rclpy.shutdown()

    if response is None:
        print("ERROR: no response from /execute_recipe")
        sys.exit(1)

    print(f"success={response.success}")
    print(f"message={response.message}")
    sys.exit(0 if response.success else 1)


if __name__ == '__main__':
    main()
