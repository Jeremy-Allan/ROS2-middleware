import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _load_calib():
    path = os.path.join(
        get_package_share_directory('computer_vision'),
        'config', 'calibration.yaml')
    with open(path) as f:
        raw = yaml.safe_load(f)['calibration']
    return {k: str(v) for k, v in raw.items()}


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('computer_vision'), 'config')
    eh2 = get_package_share_directory('easy_handeye2')
    cal = _load_calib()

    calibrating = IfCondition(LaunchConfiguration('extrinsic_calibrate'))

    publish_saved = IfCondition(PythonExpression([
        "'", LaunchConfiguration('extrinsic_calibrate'), "'.lower() == 'false' and '",
        LaunchConfiguration('use_static_transform'), "'.lower() == 'false'",
    ]))

    publish_manual = IfCondition(PythonExpression([
        "'", LaunchConfiguration('extrinsic_calibrate'), "'.lower() == 'false' and '",
        LaunchConfiguration('use_static_transform'), "'.lower() == 'true'",
    ]))

    args = [
        DeclareLaunchArgument(
            'extrinsic_calibrate', default_value='false',
            description='true: run ArUco + easy_handeye2 GUI to compute a calibration.'),
        DeclareLaunchArgument(
            'use_static_transform', default_value='false',
            description='true: publish a manual static transform from calibration.yaml. '
                        'false: publish the saved easy_handeye2 calibration.'),
        DeclareLaunchArgument('calibration_name', default_value=cal['calibration_name']),
        DeclareLaunchArgument('robot_base_frame', default_value=cal['robot_base_frame']),
        DeclareLaunchArgument('robot_effector_frame', default_value=cal['robot_effector_frame']),
        DeclareLaunchArgument('tracking_base_frame', default_value=cal['tracking_base_frame']),
        DeclareLaunchArgument('tracking_marker_frame', default_value=cal['tracking_marker_frame']),
        DeclareLaunchArgument('cam_base_topic', default_value=cal['cam_base_topic']),
        DeclareLaunchArgument('marker_dict', default_value=cal['marker_dict']),
        DeclareLaunchArgument('marker_size', default_value=cal['marker_size']),
        DeclareLaunchArgument('static_tf_parent_frame',
                              default_value=cal['static_tf_parent_frame']),
        DeclareLaunchArgument('static_tf_child_frame',
                              default_value=cal['static_tf_child_frame']),
        DeclareLaunchArgument('static_tf_x', default_value=cal['static_tf_x']),
        DeclareLaunchArgument('static_tf_y', default_value=cal['static_tf_y']),
        DeclareLaunchArgument('static_tf_z', default_value=cal['static_tf_z']),
        DeclareLaunchArgument('static_tf_roll', default_value=cal['static_tf_roll']),
        DeclareLaunchArgument('static_tf_pitch', default_value=cal['static_tf_pitch']),
        DeclareLaunchArgument('static_tf_yaw', default_value=cal['static_tf_yaw']),
    ]

    # --- Kinect v2 driver ---
    kinect = Node(
        package='kinect2_bridge',
        executable='kinect2_bridge_node',
        name='kinect2_bridge_node',
        output='log',
        parameters=[{
            'base_name':    'kinect2',
            'base_name_tf': 'kinect2',
            'publish_tf':   True,
            'sensor':       '',
            'fps_limit':    -1.0,
            'use_png':      False,
            'depth_method': 'default',
            'reg_method':   'default',
        }],
    )

    # --- Vision node (delayed so the bridge has published its first frames) ---
    vision_node = Node(
        package='computer_vision', executable='vision_node',
        name='vision_snapshot_node', output='screen',
        parameters=[os.path.join(cfg, 'camera.yaml'),
                    os.path.join(cfg, 'vision.yaml'),
                    os.path.join(cfg, 'yolo.yaml')],
    )
    vision = TimerAction(period=5.0, actions=[vision_node])

    # --- Manual static transform, gated on the vision node being ready ---
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='kinect_static_tf',
        output='screen',
        condition=publish_manual,
        arguments=[
            '--x', LaunchConfiguration('static_tf_x'),
            '--y', LaunchConfiguration('static_tf_y'),
            '--z', LaunchConfiguration('static_tf_z'),
            '--roll',  LaunchConfiguration('static_tf_roll'),
            '--pitch', LaunchConfiguration('static_tf_pitch'),
            '--yaw',   LaunchConfiguration('static_tf_yaw'),
            '--frame-id',       LaunchConfiguration('static_tf_parent_frame'),
            '--child-frame-id', LaunchConfiguration('static_tf_child_frame'),
        ],
    )

    fired = {'done': False}

    # NOTE: OnProcessIO handlers are called as handler(event) — *not*
    # handler(event, context). Extra args must be optional.
    def _on_vision_ready(event, *_):
        if fired['done']:
            return None
        text = event.text
        if isinstance(text, bytes):
            text = text.decode('utf-8', 'replace')
        if 'ready | workspace=' not in text:
            return None
        fired['done'] = True
        # 1.5 s > one period of the vision node's 1 Hz /vision/workspace timer,
        # so the static TF cannot start before the workspace has been broadcast.
        return [TimerAction(period=1.5, actions=[static_tf])]

    wait_for_vision = RegisterEventHandler(
        OnProcessIO(
            target_action=vision_node,
            on_stdout=_on_vision_ready,
            on_stderr=_on_vision_ready,   # rclpy logs go to stderr by default
        )
    )

    # --- Calibration mode: ArUco + easy_handeye2 GUI ---
    aruco = TimerAction(period=7.0, actions=[Node(
        package='aruco_opencv', executable='aruco_tracker_autostart',
        name='aruco_tracker', output='screen',
        condition=calibrating,
        parameters=[{
            'cam_base_topic': LaunchConfiguration('cam_base_topic'),
            'marker_dict':    LaunchConfiguration('marker_dict'),
            'marker_size':    LaunchConfiguration('marker_size'),
            'publish_tf':     True,
            'output_frame':   LaunchConfiguration('tracking_base_frame'),
        }],
    )])

    calibrate = TimerAction(period=10.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(eh2, 'launch', 'calibrate.launch.py')),
        condition=calibrating,
        launch_arguments={
            'calibration_type':      'eye_on_base',
            'name':                  LaunchConfiguration('calibration_name'),
            'robot_base_frame':      LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame':  LaunchConfiguration('robot_effector_frame'),
            'tracking_base_frame':   LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
        }.items(),
    )])

    # --- Normal mode A: publish saved easy_handeye2 calibration ---
    publish_calib = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(eh2, 'launch', 'publish.launch.py')),
        condition=publish_saved,
        launch_arguments={
            'name': LaunchConfiguration('calibration_name'),
        }.items(),
    )

    return LaunchDescription(args + [
        kinect,
        vision,
        wait_for_vision,
        aruco,
        calibrate,
        publish_calib,
    ])