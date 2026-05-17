import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

# Arm Actions and Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# Gripper Actions and Messages
from control_msgs.action import GripperCommand

# Services
from std_srvs.srv import Trigger

# TF for Relative Movements
from tf2_ros import Buffer, TransformListener

class HardwareInterfaceClient(Node):
    def __init__(self):
        super().__init__('kinova_hardware_client')
        self.get_logger().info('Kinova Hardware Client Online - Waiting for Service Requests...')
        
        # Use a ReentrantCallbackGroup to allow service handlers and action callbacks to run concurrently
        self.callback_group = ReentrantCallbackGroup()

        # Action Clients (The "Skills")
        self.arm_client = ActionClient(
            self, MoveGroup, 'move_action', 
            callback_group=self.callback_group
        )
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gen3_lite_2f_gripper_controller/gripper_cmd',
            callback_group=self.callback_group
        )

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Synchronous Movement Control
        self.movement_finished = threading.Event()
        self.movement_finished.set() 
        self.last_action_successful = False
        # Parameters for movement goals
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.0)
        self.declare_parameter('gripper_position', 0.0)
        
        # Parameters for relative vectors
        self.declare_parameter('vector_x', 0.0)
        self.declare_parameter('vector_y', 0.0)
        self.declare_parameter('vector_z', 0.0)

        # ROS 2 Services (The "API")
        self.create_service(Trigger, '~/home_arm', self.handle_home_arm, callback_group=self.callback_group)
        self.create_service(Trigger, '~/move_arm', self.handle_move_arm, callback_group=self.callback_group)
        self.create_service(Trigger, '~/move_gripper', self.handle_move_gripper, callback_group=self.callback_group)
        self.create_service(Trigger, '~/relative_move', self.handle_relative_move, callback_group=self.callback_group)

    # --- Service Handlers ---
    def handle_home_arm(self, request, response):
        self.get_logger().info("Service Call: Home Arm")
        if self.send_home_goal():
            self.movement_finished.wait()
            response.success = self.last_action_successful
            response.message = "Arm moved home successfully" if response.success else "Arm movement failed"
        else:
            response.success = False
            response.message = "Failed to initiate home movement"
        return response

    def handle_move_arm(self, request, response):
        x = self.get_parameter('target_x').get_parameter_value().double_value
        y = self.get_parameter('target_y').get_parameter_value().double_value
        z = self.get_parameter('target_z').get_parameter_value().double_value
        
        self.get_logger().info(f"Service Call: Move Arm to {x}, {y}, {z}")
        if self.send_goal(x, y, z):
            self.movement_finished.wait()
            response.success = self.last_action_successful
            response.message = f"Arm moved to {x}, {y}, {z}" if response.success else "Arm movement failed"
        else:
            response.success = False
            response.message = "Failed to initiate arm movement"
        return response

    def handle_relative_move(self, request, response):
        vx = self.get_parameter('vector_x').get_parameter_value().double_value
        vy = self.get_parameter('vector_y').get_parameter_value().double_value
        vz = self.get_parameter('vector_z').get_parameter_value().double_value
        
        self.get_logger().info(f"Service Call: Relative Move by Vector [{vx}, {vy}, {vz}]")
        
        try:
            # Look up current pose of the tool frame
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'tool_frame', now, timeout=rclpy.duration.Duration(seconds=1.0))
            
            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            curr_z = trans.transform.translation.z
            
            target_x = curr_x + vx
            target_y = curr_y + vy
            target_z = curr_z + vz
            
            self.get_logger().info(f"Calculated target: {target_x}, {target_y}, {target_z}")
            
            if self.send_goal(target_x, target_y, target_z):
                self.movement_finished.wait()
                response.success = self.last_action_successful
                response.message = "Relative movement complete" if response.success else "Relative movement failed"
            else:
                response.success = False
                response.message = "Failed to initiate relative movement"
                
        except Exception as e:
            self.get_logger().error(f"Could not calculate relative move: {e}")
            response.success = False
            response.message = str(e)
            
        return response

    def handle_move_gripper(self, request, response):
        pos = self.get_parameter('gripper_position').get_parameter_value().double_value
        self.get_logger().info(f"Service Call: Move Gripper to {pos}")
        if self.move_gripper(pos):
            self.movement_finished.wait()
            response.success = self.last_action_successful
            response.message = f"Gripper moved to {pos}" if response.success else "Gripper movement failed"
        else:
            response.success = False
            response.message = "Failed to initiate gripper movement"
        return response

    # --- Action Client Methods ---
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
        future = self.arm_client.send_goal_async(
            goal_msg,
            feedback_callback=self.arm_feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        return True

    def send_home_goal(self):
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Arm server not available (Home Goal)')
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
        future = self.arm_client.send_goal_async(
            goal_msg,
            feedback_callback=self.arm_feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        return True

    def move_gripper(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            return False
        
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        
        self.movement_finished.clear()
        future = self.gripper_client.send_goal_async(
            goal,
            feedback_callback=self.gripper_feedback_callback
        )
        future.add_done_callback(self.gripper_response_callback)
        return True

    # --- Callbacks ---
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by the Action Server.')
            self.last_action_successful = False
            self.movement_finished.set()
            return
        
        self.get_logger().info('Goal accepted! Moving...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def arm_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'[Feedback] MoveIt State: {feedback.state}')

    def result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val
        
        if error_code == result.error_code.SUCCESS:
            self.get_logger().info('Movement complete!')
            self.last_action_successful = True
        elif error_code == result.error_code.NO_IK_SOLUTION:
            self.get_logger().error("ERROR: Coordinates out of reach! (Arm is too short)")
            self.last_action_successful = False
        elif error_code == result.error_code.PLANNING_FAILED:
            self.get_logger().error("ERROR: Planning failed! (Likely trying to move through a table)")
            self.last_action_successful = False
        else:
            self.get_logger().error(f'MoveIt failed with error code: {error_code}')
            self.last_action_successful = False

        self.movement_finished.set()

    def gripper_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected.')
            self.last_action_successful = False
            self.movement_finished.set()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def gripper_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_width = round(feedback.position, 3)
        self.get_logger().debug(f'[Feedback] Gripper Width: {current_width}')

    def gripper_result_callback(self, future):
        self.get_logger().info('Gripper movement complete!')
        self.last_action_successful = True
        self.movement_finished.set()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterfaceClient()
    
    # Use MultiThreadedExecutor to allow concurrent callback execution
    executor = MultiThreadedExecutor(num_threads=10) # TODO (pulkit) change the hardcoded threads numbers
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
