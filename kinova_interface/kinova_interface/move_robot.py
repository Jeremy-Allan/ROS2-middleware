import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading

# Arm Actions and Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# Gripper Actions and Messages
from control_msgs.action import GripperCommand

class KinovaMoveClient(Node):
    def __init__(self):
        super().__init__('kinova_move_client')
        self.get_logger().info('Kinova Interface Node Started - Ready for Console Input')
        
        # Action Client 1: The Arm (MoveIt)
        self.arm_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Action Client 2: The Gripper (Direct Controller)
        self.gripper_client = ActionClient(self, GripperCommand, '/gen3_lite_2f_gripper_controller/gripper_cmd')

    def send_goal(self, x, y, z):
        self.get_logger().info('Waiting for /move_action server...')
        self.arm_client.wait_for_server()

        # 1. Initialize the Goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        # 2. Define the Target Position Constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link" 
        pos_constraint.link_name = "tool_frame"      
        
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01] 

        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)

        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)

        self.get_logger().info(f'Sending MoveIt goal: X={x}, Y={y}, Z={z}')
        self.arm_client.send_goal_async(goal_msg)

    def send_home_goal(self):
        """Resets the arm using explicit Joint Constraints (Radians)"""
        self.get_logger().info('Waiting for /move_action server to reset...')
        self.arm_client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        # Hardcoded Safe Home Angles
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

        self.get_logger().info("Sending joint goal to reset to HOME position...")
        self.arm_client.send_goal_async(goal_msg)

    def move_gripper(self, position):
        """Commands the 2-Finger Gripper controller directly"""
        self.get_logger().info('Waiting for gripper action server...')
        self.gripper_client.wait_for_server()
        
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        
        action_text = "Closing" if position == 0.0 else "Opening"
        self.get_logger().info(f"{action_text} gripper to position {position}...")
        
        self.gripper_client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = KinovaMoveClient()
    
    # Start a background thread for ROS 2 to "spin"
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # The Main Thread handles the continuous user input menu
    try:
        while rclpy.ok():
            print("\n-------------------------------------------------")
            user_input = input("Enter coords 'X,Y,Z' | 'r' (reset) | 'o' (open) | 'c' (close) | 'q' (quit): ").strip().lower()
            
            if user_input == 'q':
                node.get_logger().info('Quit command received.')
                break
            elif user_input == 'r':
                node.send_home_goal()
            elif user_input == 'c':
                node.move_gripper(0.0) # 0.0 is fully closed
            elif user_input == 'o':
                node.move_gripper(1.0) # 1.0 is fully open
            else:
                try:
                    coords = user_input.split(',')
                    if len(coords) != 3:
                        raise ValueError("Provide exactly three numbers separated by commas, or a valid letter command.")
                        
                    x = float(coords[0].strip())
                    y = float(coords[1].strip())
                    z = float(coords[2].strip())
                    
                    node.send_goal(x, y, z)
                    
                except ValueError as e:
                    print(f"Invalid input: {e}. Please try again.")

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected.')
    finally:
        node.get_logger().info('Shutting down Kinova Interface...')
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()