import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading  # Required to run ROS 2 and input() simultaneously

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class KinovaMoveClient(Node):
    def __init__(self):
        super().__init__('kinova_move_client')
        self.get_logger().info('Kinova Interface Node Started - Ready for Console Input')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def send_goal(self, x, y, z):
        # We removed the timeout error return here so the loop doesn't crash 
        # if you type a coordinate before MoveIt fully boots up.
        self.get_logger().info('Waiting for /move_action server...')
        self._action_client.wait_for_server()

        # 1. Initialize the Goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        # 2. Define the Target Position Constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link" 
        pos_constraint.link_name = "tool_frame"      
        
        # 3. Create a 1cm "Target Bubble" (Sphere)
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01] 

        # 4. Set the exact X, Y, Z 
        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)

        # 5. Pack everything together
        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)

        self.get_logger().info(f'Sending MoveIt goal: X={x}, Y={y}, Z={z}')
        
        # 6. Fire the command
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinovaMoveClient()
    
    # 1. Start a background thread for ROS 2 to "spin" (listen to the network)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # 2. The Main Thread handles the continuous user input
    try:
        while rclpy.ok():
            print("\n-------------------------------------------------")
            user_input = input("Enter coordinates (e.g., '0.3, 0.1, 0.4') or 'q' to quit: ")
            
            if user_input.lower().strip() == 'q':
                node.get_logger().info('Quit command received.')
                break
                
            try:
                # Parse the string into three distinct float values
                coords = user_input.split(',')
                if len(coords) != 3:
                    raise ValueError("You must provide exactly three numbers separated by commas.")
                    
                x = float(coords[0].strip())
                y = float(coords[1].strip())
                z = float(coords[2].strip())
                
                # Fire the command to the background node
                node.send_goal(x, y, z)
                
            except ValueError as e:
                print(f"Invalid input: {e}. Please try again.")

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected.')
    finally:
        # Clean up properly when exiting
        node.get_logger().info('Shutting down Kinova Interface...')
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()