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
            executable='vision_node',
            name='computer_vision',
            parameters=[{
            'color_topic': '/kinect2/sd/image_color_rect',
            'depth_topic': '/kinect2/sd/image_depth_rect',
            'info_topic':  '/kinect2/sd/camera_info',
            'table_z': 0.0,          # ADD Later
            'workspace': [-0.6, 0.6, -0.5, 0.5], #check later as well
    }],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
    ])
