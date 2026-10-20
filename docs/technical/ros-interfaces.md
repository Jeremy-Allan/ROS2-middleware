# ROS interfaces

This page lists the interfaces of the middleware: the services and messages defined in `kinova_interfaces`, and the topics, actions and external services the nodes use.

## Messages

| Message | Fields |
|---|---|
| `ExtendedStatus` | `node_name` (string), `state` (uint8: `STATE_IDLE` 0, `STATE_BUSY` 1, `STATE_FAULT` 2), `status_message` (string), `last_command_valid` (bool) |
| `SystemSummary` | `summary_state` (uint8: `SYSTEM_READY` 0, `SYSTEM_BUSY` 1, `SYSTEM_FAULT` 2), `individual_states` (`ExtendedStatus[]`) |
| `MotionParams` | `velocity_scale` (float64), `acceleration_scale` (float64). Each is 0.0 to 1.0. `0.0` leaves the value unset. |

## Services

Most services return `success` (bool) and `message` (string). Rows note the exceptions.

### Recipes

| Service | Request | Response |
|---|---|---|
| `ExecuteRecipe` | `recipe_json` (string) | `message` |

### Hardware

These services are served by `kinova_hardware_client`.

| Service | Request | Response |
|---|---|---|
| `HomeArm` | `motion_params` | |
| `MoveArm` | `target_position` (`geometry_msgs/Point`), `has_orientation` (bool), `roll`, `pitch`, `yaw` (float64), `motion_params` | |
| `RelativeMove` | `vx`, `vy`, `vz` (float64), `roll_delta`, `pitch_delta`, `yaw_delta` (float64), `motion_params` | |
| `JointMove` | `joint_positions` (float64[], six values for joint 1 to 6), `wait_for_completion` (bool), `relative` (bool), `motion_params` | `error_code` (`moveit_msgs/MoveItErrorCodes`). `val` is 0 when MoveIt never answered. |
| `ExecuteTrajectory` | `trajectory` (`moveit_msgs/RobotTrajectory`) | `error_code` |
| `MoveGripper` | `position` (float64) | |

### Environment

These services are served by `environment_mapping_node`.

| Service | Request | Response |
|---|---|---|
| `GetObjectCoordinates` | `object_id` | `x`, `y`, `z` |
| `GetObjectInfo` | `object_id` | `pose` (`geometry_msgs/Pose`), `shape` (`shape_msgs/SolidPrimitive`) |
| `GetRelativeMovement` | `move_id` | `x`, `y`, `z` |
| `GetOrientationPreset` | `preset_name` | `roll`, `pitch`, `yaw` |
| `GetRobotParameters` | none | `object_list`, `movement_names`, `orientation_names`, `held_objects` (string[]), `has_table_bounds` (bool), `table_x_min`, `table_x_max`, `table_y_min`, `table_y_max` (float64). This service has no `success` or `message`. |
| `GetSceneObjects` | none | `objects_json` (string). This service has no `message`. |
| `UpdateSceneObjects` | `objects_json` (string), `remove_missing` (bool) | `added`, `updated`, `removed`, `skipped` (string[]) |
| `UpdateObjectPose` | `object_id`, `pose` | |
| `AttachObject`, `DetachObject`, `RemoveObject` | `object_id` | |

`GetSceneObjects` returns `{"objects": {"<id>": {"pose": ..., "shape": ...}}, "attached": ["<id>"]}`. `UpdateSceneObjects` expects `{"objects": {"<id>": {"pose": ..., "shape": ...}}}`. A pose has `position` (`x`, `y`, `z`) and `orientation`. Input accepts the orientation as `roll`, `pitch`, `yaw` or as a quaternion `x`, `y`, `z`, `w`. Output uses a quaternion.

### Vision

| Service | Request | Response |
|---|---|---|
| `Snapshot` | `num_frames` (uint32), `remove_missing` (bool) | `message`, `summary_json` (string) |

## Topics

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/status/node_report` | `ExtendedStatus` | `json_parser_node`, `environment_mapping_node`, `kinova_hardware_client` | `telemetry_node` |
| `/system/status` | `SystemSummary` | `telemetry_node` | rosbridge clients, such as the proxy |
| `/joint_states` | `sensor_msgs/JointState` | Kinova driver stack | `kinova_hardware_client`, `json_parser_node` |
| `/fault_controller/is_faulted` | `example_interfaces/Bool` | `fault_controller` | `kinova_hardware_client` |
| `/vision/debug_image` | `sensor_msgs/Image` | `vision_snapshot_node` | none |
| `/vision/workspace` | `visualization_msgs/MarkerArray` | `vision_snapshot_node` | none |

## Other services and actions used

| Name | Type | Provided by | Used by |
|---|---|---|---|
| `move_action` | `moveit_msgs/MoveGroup` (action) | MoveIt 2 | `kinova_hardware_client` |
| `execute_trajectory` | `moveit_msgs/ExecuteTrajectory` (action) | MoveIt 2 | `kinova_hardware_client` |
| `/gen3_lite_2f_gripper_controller/gripper_cmd` | `control_msgs/GripperCommand` (action) | Gripper controller of the Kinova stack | `kinova_hardware_client` |
| `/plan_kinematic_path` | `moveit_msgs/GetMotionPlan` | MoveIt 2 | `json_parser_node` |
| `/compute_cartesian_path` | `moveit_msgs/GetCartesianPath` | MoveIt 2 | `json_parser_node` |
| `/compute_ik`, `/compute_fk` | `moveit_msgs/GetPositionIK`, `GetPositionFK` | MoveIt 2 | `json_parser_node` |
| `/check_state_validity` | `moveit_msgs/GetStateValidity` | MoveIt 2 | `json_parser_node` |
| `/get_planning_scene`, `/apply_planning_scene` | `moveit_msgs/GetPlanningScene`, `ApplyPlanningScene` | MoveIt 2 | `json_parser_node`, `environment_mapping_node` |
| `/fault_controller/reset_fault` | `example_interfaces/Trigger` | `fault_controller` | `telemetry_node` |
| `/controller_manager/list_controllers` | `controller_manager_msgs/ListControllers` | Controller manager | `kinova_hardware_client` |
| `/controller_manager/configure_controller`, `/controller_manager/switch_controller` | `controller_manager_msgs` | Controller manager | `telemetry_node` |

The nodes also read the `base_link` to `tool_frame` transform from TF.
