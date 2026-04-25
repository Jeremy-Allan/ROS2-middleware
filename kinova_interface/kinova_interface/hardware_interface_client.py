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

# We'll use a standard String service for the POC "POST" API
from std_srvs.srv import Trigger

class HardwareInterfaceServer(Node):
    def __init__(self):
        super().__init__('kinova_hardware_server')
        self.get_logger().info('Kinova Hardware Server Online - Waiting for Service Requests...')
        
        # Action Clients (The "Skills")
        self.arm_client = ActionClient(self, MoveGroup, 'move_action')
        self.gripper_client = ActionClient(self, GripperCommand, '/gen3_lite_2f_gripper_controller/gripper_cmd')

        self.movement_finished = threading.Event()
        self.movement_finished.set() 

        # ROS 2 Services (The "API")
        # In a full build, we'd use custom .srv files for XYZ. 
        # For this POC, we'll keep the internal methods accessible for other nodes.

    def send_goal(self, x, y, z):
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Arm server not available')
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

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
        
        self.movement_finished.clear()
        future = self.arm_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        return True

    def send_home_goal(self):
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        
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

        self.movement_finished.clear()
        future = self.arm_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        return True

    def move_gripper(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            return False
        
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        
        self.movement_finished.clear()
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self.gripper_response_callback)
        return True

    # --- Callbacks ---
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.movement_finished.set()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.movement_finished.set()

    def gripper_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.movement_finished.set()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future):
        self.movement_finished.set()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterfaceServer()
    # In this new architecture, this node just spins and waits
    # It doesn't have a menu anymore.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
