import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class KinovaMoveClient(Node):
    def __init__(self):
        super().__init__('kinova_move_client')
        self.get_logger().info('Kinova Interface Node Started - Awaiting Coordinates')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def send_goal(self, x, y, z):
        self.get_logger().info('Waiting for /move_action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server /move_action NOT available!')
            return

        # 1. Initialize the Goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        # 2. Define the Target Position Constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link" # The coordinate origin
        pos_constraint.link_name = "tool_frame"      # The tip of the Kinova arm
        
        # 3. Create a 1cm "Target Bubble" (Sphere)
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01] 

        # 4. Set the exact X, Y, Z for the center of the bubble
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
    
    # Test Coordinate: Moving Forward and Up
    node.send_goal(0.3, 0.1, 0.4) 
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Kinova Interface...')
    finally:
        node.destroy_node()
        rclpy.shutdown()