import sys
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

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

def generate_launch_description():
    # Declare arguments
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

    # Launch configurations
    recipe = LaunchConfiguration('recipe')
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
            'robot_hand_controller': 'gen3_lite_2f_gripper_controller'
        }.items()
    )

    # 2. Start MoveIt 2
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kinova_gen3_lite_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware
        }.items()
    )


    # 3. Middleware Nodes (The new "Brain")
    environment_mapping_node = Node(
        package='kinova_interface',
        executable='environment_mapping_node',
        name='environment_mapping_node',
        output='screen'
    )

    hardware_interface_client = Node(
        package='kinova_interface',
        executable='hardware_interface_client',
        name='kinova_hardware_client',
        output='screen'
    )

    json_parser_node = Node(
        package='kinova_interface',
        executable='json_parser_node',
        name='json_parser_node',
        output='screen',
        parameters=[{'recipe': recipe}]
    )

    return LaunchDescription([
        recipe_arg,
        robot_ip_arg,
        use_fake_hardware_arg,
        OpaqueFunction(function=check_hardware_args),
        kortex_control_launch,
        move_group_launch,
        environment_mapping_node,
        hardware_interface_client,
        json_parser_node
    ])
