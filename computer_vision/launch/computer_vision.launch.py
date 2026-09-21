import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PythonExpression,
)
from launch_ros.actions import Node


def _load_calib():
    path = os.path.join(
        get_package_share_directory('computer_vision'),
        'config', 'calibration.yaml')
    with open(path) as f:
        raw = yaml.safe_load(f)['calibration']
    return {k: str(v) for k, v in raw.items()}


def _static_publisher_when_ready(context, *args, **kwargs):
    """Event-driven: wait for the bridge's TF via tf2_ros, then broadcast
    the static transform in-process (no subprocess, same DDS graph as
    every other node in the launch)."""
    ext_cal = LaunchConfiguration('extrinsic_calibrate').perform(context)
    use_static = LaunchConfiguration('use_static_transform').perform(context)
    if ext_cal.lower() == 'true' or use_static.lower() != 'true':
        return []

    parent = LaunchConfiguration('static_tf_parent_frame').perform(context)
    child  = LaunchConfiguration('static_tf_child_frame').perform(context)
    x      = LaunchConfiguration('static_tf_x').perform(context)
    y      = LaunchConfiguration('static_tf_y').perform(context)
    z      = LaunchConfiguration('static_tf_z').perform(context)
    roll   = LaunchConfiguration('static_tf_roll').perform(context)
    pitch  = LaunchConfiguration('static_tf_pitch').perform(context)
    yaw    = LaunchConfiguration('static_tf_yaw').perform(context)

    waiter = f'''
import math, sys
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


def quat_from_euler(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        sr * cp * cy - cr * sp * sy,   # qx
        cr * sp * cy + sr * cp * sy,   # qy
        cr * cp * sy - sr * sp * cy,   # qz
        cr * cp * cy + sr * sp * sy,   # qw
    )


rclpy.init()
node = rclpy.create_node("static_tf_waiter")

buffer = Buffer()
listener = TransformListener(buffer, node)   # noqa: F841

print("[static_tf] waiting for bridge TF kinect2_link -> kinect2_rgb_optical_frame ...", flush=True)

# Event-driven: spin_once returns as soon as a message arrives on /tf or /tf_static.
# The moment the bridge publishes its first frame, can_transform flips true.
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.5)
    if buffer.can_transform("kinect2_link", "kinect2_rgb_optical_frame", Time()):
        print("[static_tf] bridge TF ready, broadcasting {parent} -> {child}", flush=True)
        break
else:
    sys.exit(1)

broadcaster = StaticTransformBroadcaster(node)
t = TransformStamped()
t.header.stamp = node.get_clock().now().to_msg()
t.header.frame_id = {parent!r}
t.child_frame_id  = {child!r}
t.transform.translation.x = float({x!r})
t.transform.translation.y = float({y!r})
t.transform.translation.z = float({z!r})
qx, qy, qz, qw = quat_from_euler(float({roll!r}), float({pitch!r}), float({yaw!r}))
t.transform.rotation.x = qx
t.transform.rotation.y = qy
t.transform.rotation.z = qz
t.transform.rotation.w = qw
broadcaster.sendTransform(t)

rclpy.spin(node)
'''

    return [ExecuteProcess(
        cmd=['python3', '-c', waiter],
        output='screen',
    )]


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('computer_vision'), 'config')
    eh2 = get_package_share_directory('easy_handeye2')
    cal = _load_calib()

    calibrating = IfCondition(LaunchConfiguration('extrinsic_calibrate'))

    publish_saved = IfCondition(PythonExpression([
        "'", LaunchConfiguration('extrinsic_calibrate'), "'.lower() == 'false' and '",
        LaunchConfiguration('use_static_transform'), "'.lower() == 'false'",
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
    vision = TimerAction(period=5.0, actions=[
        Node(
            package='computer_vision', executable='vision_node',
            name='vision_snapshot_node', output='screen',
            parameters=[os.path.join(cfg, 'camera.yaml'),
                        os.path.join(cfg, 'vision.yaml'),
                        os.path.join(cfg, 'yolo.yaml')],
        ),
    ])

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

    # --- Normal mode B: event-driven static transform ---
    publish_static_node = OpaqueFunction(function=_static_publisher_when_ready)

    return LaunchDescription(args + [
        kinect, vision, aruco, calibrate,
        publish_calib, publish_static_node,
    ])