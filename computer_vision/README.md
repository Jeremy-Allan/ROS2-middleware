# computer_vision

Optional Kinect v2 vision node. It detects objects on the table and adds them to the middleware's scene, so recipes can name them.

## Running

Install the extra dependencies listed in [Installation](../docs/installation.md#optional-computer-vision), then:

```bash
ros2 launch kinova_interface robot.launch.py enable_CV:=true
```

This starts `kinect2_bridge`, `vision_snapshot_node` and the camera-to-robot transform.

## Taking a snapshot

The node does nothing until asked:

```bash
ros2 service call /vision/snapshot kinova_interfaces/srv/Snapshot "{num_frames: 0, remove_missing: true}"
```

- `num_frames`: depth frames to average. `0` uses the configured default.
- `remove_missing`: remove vision objects no longer seen.

Detections are pushed to `environment_mapping_node` through `/update_scene_objects`. Objects YOLO recognises get their class name. Unrecognised clusters are named `object`. The fitted table is added as `table_fit`. The proxy's `/scan` command calls this service.

## Calibration

`robot.launch.py` uses the defaults below. To change them, run `ros2 launch computer_vision computer_vision.launch.py <argument>:=<value>` instead of `enable_CV:=true`.

| Launch argument | Default | Effect |
|---|---|---|
| `extrinsic_calibrate` | `false` | `true` runs ArUco and the `easy_handeye2` GUI to compute a calibration |
| `use_static_transform` | `false` | `true` publishes the manual transform in `config/calibration.yaml`; `false` publishes the saved `easy_handeye2` calibration |

## Configuration

| File | Contents |
|---|---|
| `config/camera.yaml` | Topics, frames and the arm self-mask |
| `config/vision.yaml` | Workspace bounds, table plane and clustering |
| `config/yolo.yaml` | YOLO model, thresholds and per-class settings |
| `config/calibration.yaml` | Calibration frames, marker and the manual static transform |
