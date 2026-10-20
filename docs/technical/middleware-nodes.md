# Middleware nodes

The `kinova_interface` package contains four nodes. See [ROS interfaces](ros-interfaces.md) for interface definitions.

![Services and actions between the nodes: rosbridge_server calls json_parser_node, which calls environment_mapping_node, kinova_hardware_client and MoveIt 2. kinova_hardware_client calls MoveIt 2 and the gripper controller.](../assets/diagrams/node-services.svg)

## json_parser_node

Receives a recipe, runs its steps in order and reports the result.

**Services**

| Service | Type | Description |
|---|---|---|
| `/execute_recipe` | `ExecuteRecipe` | Runs a recipe passed as a JSON string. Returns when the whole recipe has finished. |
| `/reset_environment` | `std_srvs/Trigger` | Restores the objects and obstacles to the configured state and clears the held object. |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `recipe` | `none` | Recipe file to run once at start-up, two seconds after the node starts. An absolute path, or a path relative to the installed `recipes` folder. `none` waits for the service. |
| `grasping.*` | see [Configuration](../installation/configuration.md) | Settings for pickup, dropoff and the planner. Read from `motion_settings.yaml`. |
| `hardware.action_timeout`, `hardware.gripper_timeout` | `30.0`, `10.0` | Used to size the timeouts of the calls it makes to `kinova_hardware_client`. |

**Behaviour**

- Executes one recipe at a time, stopping on the first failed step.
- Steps use `{"action": ..., "parameters": {...}}`. Optional `description` is logged; unknown keys are ignored.
- Waits 0.5 s between steps and logs each result as `[recipe_log]`.
- Tracks held objects: `pickup` sets the object; `dropoff` and `throw` clear it.
- Passes subsequent steps to actions for grasp planning, including `pour` and `thrust`.

## environment_mapping_node

Holds the object dictionary, the obstacles, the named movements and the named orientations. It copies the objects and obstacles into MoveIt's planning scene.

**Services**

| Service | Type | Purpose |
|---|---|---|
| `/get_coordinates` | `GetObjectCoordinates` | Object position |
| `/get_object_info` | `GetObjectInfo` | Object pose and shape |
| `/get_relative_movement` | `GetRelativeMovement` | Named movement offset |
| `/get_orientation_preset` | `GetOrientationPreset` | Named orientation |
| `/get_robot_parameters` | `GetRobotParameters` | Available objects, presets and workspace bounds |
| `/get_scene_objects` | `GetSceneObjects` | Scene objects and attachments |
| `/update_scene_objects` | `UpdateSceneObjects` | Bulk scene updates |
| `/update_object_pose` | `UpdateObjectPose` | Dictionary pose update |
| `/attach_object` | `AttachObject` | Attach object |
| `/detach_object` | `DetachObject` | Detach object |
| `/remove_object` | `RemoveObject` | Remove unattached object |
| `/reset_environment_scene` | `std_srvs/Trigger` | Restore configured scene |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `config_dir` | none | Folder that holds `workspace_objects.yaml` and `movements_and_orientations.yaml`. Required. The launch file sets it. |

**Behaviour**

- Loads workspace objects, obstacles and movement presets at startup. Invalid configuration prevents startup.
- Supports Euler or quaternion orientations and `BOX`, `CYLINDER`, `CONE` and `SPHERE` shapes. Invalid shape types or dimensions fall back to defaults with warnings.
- Waits up to 15 s for MoveIt's planning scene service before publishing objects.
- `/update_object_pose` modifies the dictionary only, not the planning scene.
- Bulk updates skip attached objects; `remove_missing` only removes dynamically added objects.
- Scene reset clears attachments and reloads the configured objects and obstacles.

## kinova_hardware_client

The only node that sends motion commands to the arm and gripper. The node is named `kinova_hardware_client`, and its services use that name as a prefix.

**Services**

| Service | Type | Description |
|---|---|---|
| `/kinova_hardware_client/home_arm` | `HomeArm` | Moves to `hardware.home_joint_positions`. |
| `/kinova_hardware_client/move_arm` | `MoveArm` | Moves `tool_frame` to a position, with an optional orientation. |
| `/kinova_hardware_client/relative_move` | `RelativeMove` | Moves `tool_frame` by an offset from its current pose. |
| `/kinova_hardware_client/joint_move` | `JointMove` | Moves to joint positions, absolute or relative to the current joints. |
| `/kinova_hardware_client/execute_trajectory` | `ExecuteTrajectory` | Runs a trajectory that was planned elsewhere. |
| `/kinova_hardware_client/move_gripper` | `MoveGripper` | Moves the gripper to a position. |

**Parameters**

Configured through the `hardware` section of `motion_settings.yaml`. See [Configuration](../installation/configuration.md) for defaults and descriptions.

Key parameters include:

- `hardware.action_timeout`, `hardware.gripper_timeout`
- `hardware.planning_time`, `hardware.goal_tolerance`
- `hardware.home_joint_positions`, `hardware.home_joint_tolerance`
  
**Behaviour**

- Motion services report completion, failure or timeout.
- Arm moves use MoveIt 2, attempting Pilz PTP before OMPL RRT* when planning fails before execution. Position-only moves use RRT* directly.
- `motion_params` controls speed. A value of `0.0` uses planner defaults; other values are clamped to 0–1.
- Gripper movement succeeds when the target position is reached or the gripper stalls on an object.
- `joint_move` with `wait_for_completion=false` returns after goal acceptance (within 0.5 s), without waiting for completion.
- Monitors `/fault_controller/is_faulted` and checks controller activation every 3 s.

## telemetry_node

Collects the node heartbeats into one system status.

![Status flow: three nodes publish heartbeats to telemetry_node, which publishes the system status and talks to the fault controller.](../assets/diagrams/status-flow.svg)

**Interfaces:** Subscribes to `/status/node_report`, publishes `/system/status` every 0.5 s and provides `/system/reset_fault`.

**Behaviour**

- Publishes `SYSTEM_FAULT`, `SYSTEM_BUSY` or `SYSTEM_READY` based on node states.
- Missing heartbeats (>1.5 s), including before initial reporting, cause `SYSTEM_FAULT`.
- Automatically configures and activates an inactive `fault_controller`.
- `/system/reset_fault` forwards fault resets to the driver, returning an error if unavailable after 2 s.
