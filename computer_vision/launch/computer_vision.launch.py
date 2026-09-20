import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('computer_vision'), 'config')
    return LaunchDescription([
        Node(
            package='computer_vision',
            executable='vision_node',
            name='vision_snapshot_node',       # must match the top-level key in the YAML files
            output='screen',
            # Later files override earlier ones for duplicate keys; -p on the command line beats all.
            parameters=[os.path.join(cfg, 'camera.yaml'),
                        os.path.join(cfg, 'vision.yaml'),
                        os.path.join(cfg, 'yolo.yaml')],
        ),
    ])