# Troubleshooting

Each table lists a symptom, its cause and the resolution. The messages are those of the software.

## Build and launch

| Symptom | Cause | Resolution |
|---|---|---|
| `Package 'kinova_interface' not found` | The workspace is not loaded, or the build failed. | Run `source ~/workspace/ros2_kortex_ws/install/setup.bash`. Check the output of `colcon build` for errors. |
| `package 'kortex_bringup' not found` or `package 'kinova_gen3_lite_moveit_config' not found` | `ros2_kortex` is not in `src`, or it did not build. | Follow [Kinova drivers](../installation/kinova-drivers.md), then build the workspace. |
| `Caught exception in launch ... package 'rosbridge_server' not found` after `json_parser_node` starts. All processes stop. | `rosbridge_server` is not installed. | Run `rosdep install --from-paths src --ignore-src -r -y` in the workspace, or `sudo apt install ros-humble-rosbridge-server`. |
| `ERROR: 'use_fake_hardware' is set to false ... 'robot_ip' was not provided.` | `use_fake_hardware:=false` without `robot_ip`. | Add `robot_ip:=<address of the arm>`. |
| `AttributeError: ... object has no attribute 'held_objects'`, or another error about a field of `kinova_interfaces` | The build of `kinova_interfaces` is older than the source. | Run `colcon build --symlink-install` in the workspace and load it again. |
| `rosdep` prints `Cannot locate rosdep definition for [ament_python]`. | `ament_python` has no rosdep key. | None. `-r` makes `rosdep` continue. |
| A change to a configuration file has no effect. | The file is read at start, or the build copied it. | Restart the launch. Build with `--symlink-install`, or build again. See [Configuration](../installation/configuration.md#files). |
| RViz does not open. | The environment has no display. | Check `echo $DISPLAY`. See [Non-native Ubuntu](../installation/non-native-ubuntu.md). |

## Recipes

| Message | Cause | Resolution |
|---|---|---|
| `Failed to parse JSON recipe string.` | The recipe is not valid JSON. | Check the JSON. |
| `No executable steps in recipe` | `steps` is missing or empty. | Add the steps at the top level of the recipe. |
| `Unknown action '<name>'` | The action is not one of the ten. | Use an [action](command-reference.md#actions). |
| `Recipe aborted due to exception: '<parameter>'` | A required parameter is missing. | Add the parameter. See the [Command reference](command-reference.md). |
| `Could not resolve pickup target '<name>'`, `Could not resolve target '<name>'`, `Could not resolve destination '<name>'` | The name is not an object of the scene. Names are case-sensitive. | Check `workspace_objects.yaml` and `ros2 service call /get_scene_objects kinova_interfaces/srv/GetSceneObjects`. |
| `Could not resolve movement '<name>'` | The name is not in `relative_movements`. | Check `movements_and_orientations.yaml`. |
| `Unknown orientation preset '<name>'` | The name is not in `orientations`. | Check `movements_and_orientations.yaml`. |
| `Unknown grasp_style '<value>', expected one of auto, top, side` | The value is not a grasp style. | Use `auto`, `top` or `side`. |
| `Can't pick up '<target>' with a top grasp: 'pour' (step <n>) needs a side grasp` | A later step needs a side grasp. | Use `auto` or `side`. |
| `pour action requires a held object, but nothing is currently held`, and the same for `thrust` and `throw` | No object is in the gripper. | Add a `pickup` before the step. |
| `Cannot pour '<target>': held object is '<held>'`, and the same for `thrust` and `throw` | `target` is not the held object. | Use the held object, or leave out `target`. |
| `Dropoff needs a held object, but nothing is held` | No object is in the gripper. | Add a `pickup` before the step. |
| `push action requires either 'destination' or 'direction'`, and the same for `thrust` and `throw` | The step has neither. | Add one of them. |
| `Release point is off '<destination>', try a smaller distance` | The shift of `dropoff` leaves the top of the destination. | Use a smaller `distance`. |

## Motion

The MoveIt errors, such as `planning failed (path blocked by an obstacle or self-collision)` and `out of reach (no IK solution)`, are in [Errors when a move fails](../technical/kinova-drivers-and-apis.md#errors-when-a-move-fails).

| Message | Cause | Resolution |
|---|---|---|
| `No reachable grasp for '<target>': <summary>` | No grasp has a complete plan. The summary counts the grasps by style and reason. | Check that the object is in reach and that nothing blocks the approach. Try the other `grasp_style`. |
| `No reachable place pose for '<target>': <n> tried (<reasons>)` | No placement has a complete plan. | Choose another `destination`, a smaller `distance` or a lower `place_offset`. Another object on the destination can block the placement. |
| `Failed to move ...: invalid motion plan` at the first step after a reset of the scene | A restored object overlaps the gripper, and the start state is in collision. | Run `home` before the reset. See [Reset the scene](recipes.md#reset-the-scene). |
| `Failed to move to '<target>': ...` from `move_arm` | The gripper cannot surround the object at the target. | Open the gripper and give an `orientation` that lets the fingers straddle the object. |
| `<action> timed out after <n>s`, for example `Arm movement to Home timed out after 30.0s` | A move exceeded its timeout. | Retry. If it repeats, check `hardware.action_timeout` and the reach of the target. |

## Fault state

| Symptom | Cause | Resolution |
|---|---|---|
| `/system/status` is `SYSTEM_FAULT` | A node reports a fault, or it sent no heartbeat for 1.5 s. The system is also `SYSTEM_FAULT` until the first heartbeat. | Run `ros2 topic echo /status/node_report` to find the node. |
| `CRITICAL: Node heartbeat timed out (Stale for <n>s)` | A node stopped publishing on `/status/node_report`. | Check that the node runs. Restart the launch if it died. |
| `HARDWARE FAULT: Robot is faulted` | The arm reports a fault, for example a protective stop. | Clear the cause, then run `ros2 service call /system/reset_fault std_srvs/srv/Trigger {}`. |
| `SYSTEM CONFIG WARNING: fault_controller is NOT active` | The driver is still connecting to the arm. `telemetry_node` configures and activates the controller. | Wait. If the warning remains, read the log of the Kortex driver. |

## Camera

| Message | Cause | Resolution |
|---|---|---|
| `Snapshot failed: No data yet on: [...]` | The node receives no images. | Check that `kinect2_bridge_node` runs and that the topics of `camera.yaml` exist. |
| `Snapshot failed: Timed out waiting for <n> depth frames (stalled?)` | The depth stream stopped. | Check the USB connection and `kinect2_bridge_node`. |
| `Snapshot failed: colour (...) vs depth (...) sizes differ - use the registered (sd) topics` | The colour and depth topics have different sizes. | Use the `sd` topics for `color_topic` and `depth_topic`. |
| `/update_scene_objects not available` | `environment_mapping_node` does not run. | Start the middleware. |
| `Snapshot failed:` with a transform error | The transform from `base_link` to the camera frame is missing. | Check the calibration. See [Camera and calibration](../installation/camera-and-calibration.md#check-the-calibration). |
| Warning `table tilt <n> deg: likely camera extrinsic error` | The calibration is wrong. | Calibrate again. |
| A snapshot finds no objects, or the table as an object. | The filters do not fit the scene. | Adjust `workspace`, `min_height` and `cluster_min_points`. See [Parameters](../technical/computer-vision.md#parameters). |

## Proxy

| Symptom | Cause | Resolution |
|---|---|---|
| `bridge: disconnected` in the status line | The proxy cannot reach `rosbridge_server`. | Check that the middleware launch runs. Check that the `rosbridge_url` of the proxy uses the `bridge_port` of the launch. |
| `cannot execute commands: no ROS 2 bridge connected` | Same. | Same. |
| `disconnected - reconnecting...` | The terminal UI lost the backend. | Restart `./run.sh`. Read `data/logs/server.log` in the proxy folder. |
| `robot recipe execution failed: ...` | The middleware rejected the recipe, or a step failed. | Read the message. It names the step. Check the terminal of the middleware. |
| No answer, or an error with `upstream provider` | The language model is not reachable, or the key is wrong. | Check `data/config/config.json`. For Ollama, check `curl http://localhost:11434`. Read `data/logs/proxy.log`. |
