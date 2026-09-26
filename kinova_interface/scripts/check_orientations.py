#!/usr/bin/env python3
"""Check that each orientation preset points the gripper where it should.

Also shows which spots above the table each orientation can reach.

Moves to a grid of points with every orientation and prints one line per move:
  'ok 0.8 P'  got there, 0.8 deg off, planned by Pilz ('R' = RRT* fallback)
  'BAD 14.3 P' moved, but ended up in the wrong orientation or position
  'FAIL (..)' couldn't get there, with the reason (often just out of reach)
--ik-only only checks reachability ('ik ok' / 'no IK') and never moves.
See docs/testing.md for how to read the results.

Usage (the middleware must already be running):
    ros2 run kinova_interface check_orientations.py [--ik-only] [--speed 0.2] [--tolerance-deg 6]
"""
import argparse
import math
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from kinova_interface.actions.arm_actions import ArmActions
from kinova_interface.utils.geometry import orientation_from_axes
from kinova_interface.utils.robot import BASE_FRAME, TOOL_FRAME

# Positions above the table (top at z = -0.05), in base_link, meters.
GRID_X = [0.25, 0.35, 0.45]
GRID_Y = [-0.2, 0.0, 0.2]
GRID_Z = [0.10]

POSITION_TOLERANCE_M = 0.02


def _orientations():
    """name -> Rotation, with side_level at a few headings."""
    orientations = {
        'top_down': orientation_from_axes((0, 0, -1), (0, 1, 0)),
        'top_down_90': orientation_from_axes((0, 0, -1), (1, 0, 0)),
    }
    for deg in (-90, -45, 0, 45, 90):
        a = math.radians(deg)
        orientations[f'side_level@{deg}'] = orientation_from_axes(
            (math.cos(a), math.sin(a), 0.0), (-math.sin(a), math.cos(a), 0.0)
        )
    return orientations


def _check(actions, tf_buffer, x, y, z, target, args):
    roll, pitch, yaw = target.as_euler('xyz')
    if args.ik_only:
        if actions.find_ik_solution(x, y, z, roll, pitch, yaw) is None:
            return 'no IK', False
        return 'ik ok', True

    motion_params = actions.build_motion_params(args.speed)
    result = actions.call_move_service(x, y, z, True, roll, pitch, yaw, motion_params)
    if not result['success']:
        return f"FAIL ({result['message']})", False
    planner = 'P' if 'Pilz PTP' in result['message'] else 'R'

    tf = tf_buffer.lookup_transform(BASE_FRAME, TOOL_FRAME, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
    t, r = tf.transform.translation, tf.transform.rotation
    reached = Rotation.from_quat([r.x, r.y, r.z, r.w])
    angle_deg = math.degrees((target.inv() * reached).magnitude())
    position_err = math.dist((x, y, z), (t.x, t.y, t.z))
    ok = angle_deg <= args.tolerance_deg and position_err <= POSITION_TOLERANCE_M
    return f"{'ok' if ok else 'BAD'} {angle_deg:.1f} {planner}", ok


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--ik-only', action='store_true', help='only check IK reachability, never move the arm')
    parser.add_argument('--speed', type=float, default=0.2, help='velocity/acceleration scale, 0.0-1.0')
    parser.add_argument('--tolerance-deg', type=float, default=6.0, help='max angular error to count as reached')
    args = parser.parse_args()

    rclpy.init()
    node = Node('check_orientations')
    node.cb_group = ReentrantCallbackGroup()
    actions = ArmActions(node)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    orientations = _orientations()
    positions = [(x, y, z) for z in GRID_Z for x in GRID_X for y in GRID_Y]
    all_ok = True
    try:
        for name, target in orientations.items():
            if not args.ik_only:
                actions.call_home_service(actions.build_motion_params(args.speed))
            for pos in positions:
                cell, ok = _check(actions, tf_buffer, *pos, target, args)
                all_ok = all_ok and ok
                print(f"{name:>15} at ({pos[0]:.2f}, {pos[1]:+.2f}, {pos[2]:.2f}): {cell}")
        if not args.ik_only:
            actions.call_home_service(actions.build_motion_params(args.speed))
    finally:
        executor.shutdown()
        spin_thread.join()
        node.destroy_node()
        rclpy.shutdown()

    # Exit 0 only if every cell reached
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
