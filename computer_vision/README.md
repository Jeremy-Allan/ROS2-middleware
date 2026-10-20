# computer_vision

Optional ROS 2 package with the node `vision_snapshot_node`. The node detects the objects on the table with a Kinect v2 and adds them to the scene of the middleware, so that recipes can name them.

Computer vision is an alpha release.

Start it with the middleware:

```bash
ros2 launch kinova_interface robot.launch.py enable_CV:=true
```

The manual describes the node, its service `/vision/snapshot` and its parameters in [Computer vision](../docs/technical/computer-vision.md). [Camera and calibration](../docs/installation/camera-and-calibration.md) describes the installation and the calibration.
