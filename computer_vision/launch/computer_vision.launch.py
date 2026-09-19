from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('computer_vision')
    config_path = package_dir + '/config/camera_info.yaml'

    return [
        Node(
            package='computer_vision',
            executable='XXXXX',
            name='XXXXXXX',
            output='screen',
            parameters=[config_path],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
    ])
