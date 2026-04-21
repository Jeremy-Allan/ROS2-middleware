from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Start the main controller and RViz
    kortex_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kortex_bringup'),
                'launch',
                'kortex_control.launch.py'
            ])
        ]),
        # TODO: to easily test on a real robot, take in the robots ip address as a command line argument and set use_fake_harwdware to false
        launch_arguments={
            'robot_ip': '192.168.1.10', # this gets completely ignored by the system when use_fake_hardware is true.
            'use_fake_hardware': 'true', # to test on the real robot the ip address needs to be the correct ip of the robot and use_fake_hardware = false
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

    # 4. Start the HIC in a new GNOME terminal
    cli_node = Node(
        package='kinova_interface',
        executable='hardware_interface_client',
        output='screen',
        emulate_tty=True,
        prefix=['gnome-terminal -- ']
    )

    return LaunchDescription([
        kortex_control_launch,
        gripper_spawner,
        move_group_launch,
        cli_node
    ])
