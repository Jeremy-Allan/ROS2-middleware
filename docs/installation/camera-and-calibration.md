# Camera and calibration

Computer vision is an alpha release. See [Computer vision](../technical/computer-vision.md). This page installs the camera software, connects the Kinect v2 and calibrates it to the arm.

## Install the software

`enable_CV:=true` needs these packages. `rosdep` does not install them.

| Software | Provides | Installation |
|---|---|---|
| libfreenect2 | The driver of the Kinect v2. | Build and install it as described in the [libfreenect2 README](https://github.com/OpenKinect/libfreenect2). |
| `kinect2_bridge` | The node `kinect2_bridge_node` that publishes the images. | Build a ROS 2 port of `iai_kinect2`, for example [kinect2_ros2](https://github.com/YuLiHN/kinect2_ros2), in the workspace. |
| `aruco_opencv` | The tracking of the ArUco marker. | `sudo apt install ros-humble-aruco-opencv` |
| `easy_handeye2` | The calibration of the camera to the arm. | Clone [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2) into the workspace. |
| `ultralytics` | YOLO. | `pip install ultralytics` |

Build the packages that are in the workspace:

```bash
cd ~/workspace/ros2_kortex_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
source install/setup.bash
```

The model file `yolov8n.pt` is loaded by `ultralytics` when the vision node starts. A different model is set with `yolo_model`. See [Parameters](../technical/computer-vision.md#parameters).

## Connect the camera

1. Connect the Kinect v2 to a USB 3.0 port and to its power supply.
2. Run `Protonect`, which libfreenect2 builds. It shows the colour, depth and infrared images when the camera works.

## Calibrate the camera

The calibration computes the transform from `base_link` to `kinect2_link`, the base frame of the camera. The method is eye on base: the camera is fixed and a marker on the arm moves in front of it.

1. Print an ArUco marker with the ID 0 from the dictionary `6X6_250`, with an edge of 0.095 m, and fix it to the gripper. `aruco_opencv` names the frame of a marker `marker_<id>`, and the setting `tracking_marker_frame` is `marker_0`.
2. Start the middleware with the arm:

    ```bash
    ros2 launch kinova_interface robot.launch.py use_fake_hardware:=false robot_ip:=192.168.1.10
    ```

3. In a second terminal, start the calibration:

    ```bash
    ros2 launch computer_vision computer_vision.launch.py extrinsic_calibrate:=true
    ```

    The launch file starts `kinect2_bridge_node` and `vision_snapshot_node`. After 7 s it starts `aruco_tracker`, and after 10 s the calibration of `easy_handeye2`.

4. Move the arm so that the camera sees the marker. In the calibration window, take samples at several poses. Rotate the gripper about every axis between the samples. Compute the calibration and save it. See the [easy_handeye2 README](https://github.com/marcoesposito1988/easy_handeye2) for the window.

The calibration is saved under the name `kinect_eye_on_base`, the setting `calibration_name` of `calibration.yaml`. Stop the launch with `Ctrl+C`.

## Check the calibration

1. Start the middleware with the camera. The saved calibration is published at start.

    ```bash
    ros2 launch kinova_interface robot.launch.py enable_CV:=true
    ```

2. Check that the transform exists:

    ```bash
    ros2 run tf2_ros tf2_echo base_link kinect2_link
    ```

3. Take a snapshot:

    ```bash
    ros2 service call /vision/snapshot kinova_interfaces/srv/Snapshot "{num_frames: 0, remove_missing: true}"
    ```

    The response has `success=True` and a `message` such as `Scene updated: 2 added, 0 updated, 0 removed, 7 total`. In RViz, add an `Image` display for `/vision/debug_image` to see the detected objects.

If the table of the scene looks tilted or at the wrong height, the calibration is wrong. The node logs a warning when the fitted table plane tilts by more than 1°.

## Use a fixed transform

To use a transform that is measured by hand, set `static_tf_x`, `static_tf_y`, `static_tf_z`, `static_tf_roll`, `static_tf_pitch` and `static_tf_yaw` in `calibration.yaml`, then start the camera with the argument `use_static_transform:=true`:

```bash
ros2 launch computer_vision computer_vision.launch.py use_static_transform:=true
```

`robot.launch.py` starts the vision stack with the saved calibration. Run `robot.launch.py` without `enable_CV` and the command above in a second terminal to use the fixed transform.
