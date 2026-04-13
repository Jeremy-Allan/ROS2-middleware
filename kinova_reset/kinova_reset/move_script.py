#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import GripperCommand

class ResetRobot(Node):
    def __init__(self):
        super().__init__('reset_robot')
        self.arm_client = ActionClient(self, MoveGroup, 'move_action')
        self.gripper_client = ActionClient(self, GripperCommand, '/gen3_lite_2f_gripper_controller/gripper_cmd')
        self.get_logger().info("Waiting for move_action server...")

    def close_gripper(self):
        self.gripper_client.wait_for_server()
        goal = GripperCommand.Goal()
        goal.command.position = 0.0
        self.gripper_client.send_goal_async(goal)
        print("Closing gripper...")

    def send_home_goal(self):
        self.arm_client.wait_for_server()
        self.get_logger().info("Connected.")

    
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        # 90degree in radians is 1.5708 
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joint_positions = [0.0, 0.0, 1.5708, 1.5708, 1.5708, 0.0]
        tolerance = 0.01

        constraints = []
        for name, pos in zip(joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            constraints.append(jc)

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = constraints
        goal_msg.request.goal_constraints.append(goal_constraints)

        self.get_logger().info(f"Sending joint goal: {joint_positions}")
        future = self.arm_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return
        self.get_logger().info("Goal accepted, moving...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info("Reset succeeded!")
        else:
            self.get_logger().error(f"Reset failed with error code: {result.error_code.val}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ResetRobot()
    node.close_gripper()
    node.send_home_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()