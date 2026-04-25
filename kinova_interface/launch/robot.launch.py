from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    recipe_arg = DeclareLaunchArgument(
        'recipe',
        default_value='task_recipe.json',
        description='The JSON recipe file to execute.'
    )

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
            'robot_ip': '192.168.1.10', 
            'use_fake_hardware': 'true', 
            'robot_type': 'gen3_lite',
            'dof': '6',
            'gripper': 'gen3_lite_2f',
            'controllers_file': 'ros2_controllers.yaml'
        }.items()
    )

    # 2. Spawn the gripper controller
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gen3_lite_2f_gripper_controller'],
        output='screen',
    )

    # 3. Start MoveIt 2
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kinova_gen3_lite_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'use_fake_hardware': 'true'
        }.items()
    )

    # 4. Middleware Nodes (The new "Brain")
    coordinate_dict_node = Node(
        package='kinova_interface',
        executable='coordinate_dictionary_node',
        name='coordinate_dictionary_node',
        output='screen'
    )

    json_parser_node = Node(
        package='kinova_interface',
        executable='json_parser_node',
        name='json_parser_node',
        output='screen',
        arguments=['--recipe', LaunchConfiguration('recipe')]
    )

    return LaunchDescription([
        recipe_arg,
        kortex_control_launch,
        gripper_spawner,
        move_group_launch,
        coordinate_dict_node,
        json_parser_node
    ])
