import sys
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import launch.logging

def check_hardware_args(context, *args, **kwargs):
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    
    if use_fake_hardware.lower() == 'false':
        # Check if the user explicitly provided 'robot_ip' in the command line arguments
        robot_ip_provided = any(arg.startswith('robot_ip:=') for arg in sys.argv)
        if not robot_ip_provided:
            raise RuntimeError(
                "\n" + "="*62 + "\n"
                "ERROR: 'use_fake_hardware' is set to false, meaning you intend to\n"
                "connect to physical hardware. However, 'robot_ip' was not provided.\n\n"
                "You MUST explicitly provide the robot's IP address. For example:\n"
                "ros2 launch kinova_interface robot.launch.py use_fake_hardware:=false robot_ip:=192.168.1.10\n"
                + "="*62 + "\n"
            )

def launch_setup(context, *args, **kwargs):
    debug_mode = LaunchConfiguration('debug_mode').perform(context).lower() == 'true'
    core_debug = LaunchConfiguration('core_debug').perform(context).lower() == 'true'
    enable_individual_logs = LaunchConfiguration('enable_individual_logs').perform(context).lower() == 'true'

    base_ros_args = []
    if not enable_individual_logs:
        base_ros_args.extend(['--disable-external-lib-logs'])

    def get_ros_args(node_name):
        args = list(base_ros_args)
        if core_debug:
            args.extend(['--log-level', 'debug'])
        elif debug_mode:
            args.extend(['--log-level', f'{node_name}:=debug'])
        return args

    recipe = LaunchConfiguration('recipe')
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    gripper_max_velocity = LaunchConfiguration('gripper_max_velocity')
    gripper_max_force = LaunchConfiguration('gripper_max_force')
    use_sim_time = LaunchConfiguration('use_sim_time')
    allow_mtc_execution = LaunchConfiguration('allow_mtc_execution')
    mtc_execution_timeout_sec = LaunchConfiguration('mtc_execution_timeout_sec')
    mtc_planning_timeout_sec = LaunchConfiguration('mtc_planning_timeout_sec')
    env_dir = PathJoinSubstitution([FindPackageShare('kinova_interface'), 'data', 'configs', 'env'])
    manipulation_config = PathJoinSubstitution([env_dir, 'manipulation_objects.json'])

    moveit_config = (
        MoveItConfigsBuilder(
            'gen3_lite_gen3_lite_2f',
            package_name='kinova_gen3_lite_moveit_config'
        )
        .robot_description(mappings={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'gripper': 'gen3_lite_2f',
            'gripper_joint_name': 'right_finger_bottom_joint',
            'dof': '6',
            'gripper_max_velocity': gripper_max_velocity,
            'gripper_max_force': gripper_max_force,
        })
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'])
        .to_moveit_configs()
    )
    moveit_config.moveit_cpp.update({
        'use_sim_time': use_sim_time.perform(context).lower() == 'true'
    })

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'capabilities': 'move_group/ExecuteTaskSolutionCapability',
                'use_sim_time': use_sim_time,
            },
        ],
    )

    environment_mapping_node = Node(
        package='kinova_interface',
        executable='environment_mapping_node',
        name='environment_mapping_node',
        output='log',
        parameters=[{'config_dir': env_dir}],
        ros_arguments=get_ros_args('environment_mapping_node')
    )

    hardware_interface_client = Node(
        package='kinova_interface',
        executable='hardware_interface_client',
        name='kinova_hardware_client',
        output='log',
        parameters=[{'use_fake_hardware': use_fake_hardware}],
        ros_arguments=get_ros_args('kinova_hardware_client')
    )

    json_parser_node = Node(
        package='kinova_interface',
        executable='json_parser_node',
        name='json_parser_node',
        output='log',
        parameters=[{'recipe': recipe}],
        ros_arguments=get_ros_args('json_parser_node')
    )

    telemetry_node = Node(
        package='kinova_interface',
        executable='telemetry_node',
        name='telemetry_node',
        output='log',
        ros_arguments=get_ros_args('telemetry_node')
    )

    # Keep ``name`` unset: on Humble launch_ros then writes dictionary
    # parameters under /**. This process contains both the rclpy action node
    # and MTC's embedded rclcpp planning node, and both require these values.
    mtc_task_node = Node(
        package='kinova_interface',
        executable='mtc_task_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'manipulation_config': manipulation_config,
                'use_sim_time': use_sim_time,
                'allow_mtc_execution': allow_mtc_execution,
                'mtc_execution_timeout_sec': mtc_execution_timeout_sec,
                'mtc_planning_timeout_sec': mtc_planning_timeout_sec,
            },
        ],
        ros_arguments=get_ros_args('mtc_task_node')
    )

    return [
        move_group_node,
        environment_mapping_node,
        hardware_interface_client,
        json_parser_node,
        telemetry_node,
        mtc_task_node,
    ]

def generate_launch_description():
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable targeted debug mode logging for middleware nodes'
    )
    
    core_debug_arg = DeclareLaunchArgument(
        'core_debug',
        default_value='false',
        description='Enable global debug logging, including rcl and dds_cpp'
    )
    
    enable_individual_logs_arg = DeclareLaunchArgument(
        'enable_individual_logs',
        default_value='false',
        description='Enable individual node logs'
    )

    recipe_arg = DeclareLaunchArgument(
        'recipe',
        default_value='none',
        description='The JSON recipe file to execute. Default is "none", meaning it will wait for the service.'
    )

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.10',
        description='IP address of the robot. Must be provided if use_fake_hardware is false. Default value is 192.168.1.10'
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Whether to use fake hardware (simulation) or physical hardware. Default value is true'
    )

    gripper_max_velocity_arg = DeclareLaunchArgument(
        'gripper_max_velocity',
        default_value='100.0',
        description='Maximum Gen3 Lite gripper velocity percentage'
    )

    gripper_max_force_arg = DeclareLaunchArgument(
        'gripper_max_force',
        default_value='100.0',
        description='Maximum Gen3 Lite gripper force percentage'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated ROS time'
    )

    allow_mtc_execution_arg = DeclareLaunchArgument(
        'allow_mtc_execution',
        default_value='true',
        description=(
            'Permit MTC execution goals. Defaults to true; set false to '
            'disable execution for diagnostics.'
        )
    )

    mtc_execution_timeout_arg = DeclareLaunchArgument(
        'mtc_execution_timeout_sec',
        default_value='300.0',
        description='Deadline for complete MTC solution execution'
    )

    mtc_planning_timeout_arg = DeclareLaunchArgument(
        'mtc_planning_timeout_sec',
        default_value='120.0',
        description='Overall wall-time deadline for MTC task planning'
    )

    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    # 1. Start the main controller and RViz
    kortex_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kortex_bringup'),
                'launch',
                'kortex_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': robot_ip, 
            'use_fake_hardware': use_fake_hardware, 
            'robot_type': 'gen3_lite',
            'dof': '6',
            'gripper': 'gen3_lite_2f',
            'controllers_file': 'ros2_controllers.yaml',
            'gripper_joint_name': 'right_finger_bottom_joint',
            'robot_hand_controller': 'gen3_lite_2f_gripper_controller',
            'gripper_max_velocity': LaunchConfiguration('gripper_max_velocity'),
            'gripper_max_force': LaunchConfiguration('gripper_max_force')
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', launch.logging.launch_config.log_dir),
        debug_mode_arg,
        core_debug_arg,
        enable_individual_logs_arg,
        recipe_arg,
        robot_ip_arg,
        use_fake_hardware_arg,
        gripper_max_velocity_arg,
        gripper_max_force_arg,
        use_sim_time_arg,
        allow_mtc_execution_arg,
        mtc_execution_timeout_arg,
        mtc_planning_timeout_arg,
        OpaqueFunction(function=check_hardware_args),
        kortex_control_launch,
        OpaqueFunction(function=launch_setup)
    ])
