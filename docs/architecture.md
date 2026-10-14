[Docs Home](README.md)

# Architecture

## Full pipeline

```mermaid
flowchart LR
    U["You"] --> P["embodied-ai-proxy"]
    P --> L(("LLM"))
    L --> P
    P -->|"/execute_recipe via websocket"| B["rosbridge_server"]
    B --> M["ROS2-middleware<br/>four nodes (+ optional vision)"]
    M --> R["robot arm"]
    M -.->|"/system/status"| B -.-> P
```

This is the quick-glance version. For the full detailed diagram, node by node with every service call labeled:

![Full architecture diagram, node by node with every service call labeled](diagrams/architecture.png)

The proxy never calls a ROS 2 service directly. Every request goes out over the websocket to `rosbridge_server`, which translates it into a real service call against this middleware. Telemetry flows back the same way (the dashed path above), `rosbridge` subscribes to `/system/status` on the proxy's behalf and forwards messages back over the websocket.

## ROS 2 concepts you need before you start

You don't need to be a ROS 2 expert, but these four words will come up constantly, so here's what they mean in plain English:

- **Node**: a single running program. The middleware ships four of them, plus an optional vision node, see [Overview](overview.md).
- **Topic**: a one-way broadcast channel. Publishers shout messages onto a topic; subscribers listen. Nobody waits for a reply. (Used here for the heartbeat/telemetry messages.)
- **Service**: a request/response call, like a function call over the network. You call it, it does something, it replies. (Used here for almost everything: "give me coordinates for X," "move the arm," "execute this recipe.")
- **Action**: like a service, but for things that take a while and report progress along the way. For example, "move the arm": MoveIt reports planning and monitoring progress before finally succeeding or failing. Only `hardware_interface_client.py` uses actions directly; everything else in the system only ever sees simple services.
  
## Middleware node deep dive

**`hardware_interface_client.py`, the only node that touches the robot:**
- Exposes six services under its private namespace: `/kinova_hardware_client/home_arm`, `move_arm`, `move_gripper`, `relative_move`, `joint_move`, `execute_trajectory`.
- The only node that *executes* motion. Other code may query MoveIt (IK, FK, state validity, planning scene), but every arm or gripper movement goes through this node.
- Internally a client of three ROS 2 actions: `move_action` (MoveIt 2's `MoveGroup`, plans and moves in one go, for `move_arm`, `relative_move`, `joint_move` and `home`), `execute_trajectory` (MoveIt 2's `ExecuteTrajectory`, runs a trajectory that pickup or dropoff already planned) and `/gen3_lite_2f_gripper_controller/gripper_cmd` (direct gripper control, bypassing MoveIt entirely).
- Movement calls block synchronously on a `threading.Event()` until the action's result callback fires, which is what makes each recipe step wait for the robot to actually finish.
- Subscribes to `/fault_controller/is_faulted` to know instantly if the arm enters a hardware fault state.

**`environment_mapping_node.py`, the read-mostly knowledge base:**
- Loads `workspace_objects.yaml` (objects and obstacles) and `movements_and_orientations.yaml` (relative movements and orientations) at startup. `workspace_objects.yaml` is re-read on `/reset_environment_scene`. Objects also change at runtime as actions move them and when the vision node pushes detections.
- Lookups: `/get_coordinates`, `/get_object_info`, `/get_relative_movement`, `/get_orientation_preset`, `/get_robot_parameters`, `/get_scene_objects`.
- Updates: `/update_object_pose`, `/attach_object`, `/detach_object`, `/update_scene_objects`, `/reset_environment_scene`.
- Pushes every object and obstacle into MoveIt's planning scene once `/apply_planning_scene` becomes available.

**`json_parser_node.py`, orchestration:**
- Loads a recipe two ways: a static file via the `recipe` launch parameter, or a JSON string via the `/execute_recipe` service, which is what the proxy calls at runtime.
- `/reset_environment` (`std_srvs/Trigger`) resets objects to their configured poses and clears the held object.
- Iterates recipe steps in order, stopping at the first failure.
- Dispatches each step to an action in `kinova_interface/actions/` (see the layout below). Actions call the hardware and environment nodes' services, and query MoveIt services directly (`/compute_ik`, `/compute_fk`, `/check_state_validity`, `/get_planning_scene`, `/apply_planning_scene`), but never execute motion themselves. Pickup and dropoff plan their moves with `/plan_kinematic_path` (Pilz PTP/LIN, OMPL RRT\* fallback) and `/compute_cartesian_path` (straight-line fallback), checking a whole pick or place before the arm moves, then run each plan through `execute_trajectory`. See `actions/motion.py`.
- Each action returns `(success, message)`. On failure, `/execute_recipe` returns the failing step, its action and the reason, for example `Recipe failed at step 2 (pickup): Could not resolve pickup target 'red_cube'`.

**`telemetry_node.py`, pure aggregation:**
- Subscribes to `/status/node_report`, publishes an aggregated `/system/status` every 0.5s using a worst-case-wins rule across all tracked nodes.
- Hosts `/system/reset_fault`, forwarding to the real Kortex driver's own reset service.
- Automatically reconfigures and reactivates the hardware fault controller in the background if it detects the known driver startup race condition (the physical driver takes several seconds to establish its connection, which otherwise leaves `fault_controller` stuck unconfigured).

## Proxy internals

The proxy's own components (the Go backend, the LLM proxy and the terminal UI) are documented in the embodied-ai-proxy repository itself: [github.com/paul-isit/embodied-ai-proxy](https://github.com/paul-isit/embodied-ai-proxy). This page covers this middleware's own internals in depth; the pipeline diagram above is the shared context between the two.

## Custom interface types (middleware)

**Services** (request fields above the separator, response fields below):

| Service | Request | Response |
|---|---|---|
| `HomeArm` | `motion_params` (`MotionParams`) | `success`, `message` |
| `MoveArm` | `target_position` (`Point`), `has_orientation` (bool), `roll`, `pitch`, `yaw` (float64), `motion_params` (`MotionParams`) | `success`, `message` |
| `MoveGripper` | `position` (float64) | `success`, `message` |
| `RelativeMove` | `vx`, `vy`, `vz` (float64), `roll_delta`, `pitch_delta`, `yaw_delta` (float64), `motion_params` (`MotionParams`) | `success`, `message` |
| `GetObjectCoordinates` | `object_id` (string) | `x`, `y`, `z`, `success`, `message` |
| `GetObjectInfo` | `object_id` (string) | `pose` (`Pose`), `shape` (`SolidPrimitive`), `success`, `message` |
| `GetRelativeMovement` | `move_id` (string) | `x`, `y`, `z`, `success`, `message` |
| `GetOrientationPreset` | `preset_name` (string) | `roll`, `pitch`, `yaw`, `success`, `message` |
| `GetRobotParameters` | (none) | `object_list[]`, `movement_names[]`, `orientation_names[]`, `has_table_bounds`, `table_x_min`, `table_x_max`, `table_y_min`, `table_y_max` |
| `JointMove` | `joint_positions[]` (float64), `wait_for_completion` (bool), `relative` (bool), `motion_params` (`MotionParams`) | `success`, `message`, `error_code` (`moveit_msgs/MoveItErrorCodes`, 0 if MoveIt never answered) |
| `ExecuteTrajectory` | `trajectory` (`moveit_msgs/RobotTrajectory`, already planned) | `success`, `message`, `error_code` (`moveit_msgs/MoveItErrorCodes`, 0 if MoveIt never answered) |
| `ExecuteRecipe` | `recipe_json` (string) | `success`, `message` (the failing step, action and reason on failure) |
| `AttachObject`, `DetachObject` | `object_id` (string) | `success`, `message` |
| `UpdateObjectPose` | `object_id` (string), `pose` (`Pose`) | `success`, `message` |
| `GetSceneObjects` | (none) | `success`, `objects_json` |
| `UpdateSceneObjects` | `objects_json` (string), `remove_missing` (bool) | `success`, `message`, `added[]`, `updated[]`, `removed[]`, `skipped[]` |
| `Snapshot` (vision node, `/vision/snapshot`) | `num_frames` (uint32), `remove_missing` (bool) | `success`, `message`, `summary_json` |

`MoveArm`'s `has_orientation` defaults to `false`: no orientation constraint, MoveIt picks the orientation, and the move is planned with OMPL RRT* since Pilz needs a full pose. `RelativeMove` always keeps the current orientation (plus any deltas), so it has no such flag.

**Messages** (published continuously, not request/response):

- `ExtendedStatus`: one node's heartbeat: `node_name`, `state` (0 = IDLE, 1 = BUSY, 2 = FAULT), `status_message`, `last_command_valid`.
- `SystemSummary`: the aggregated view: `summary_state` plus `individual_states[]`.
- `MotionParams`: not published on its own, a shared field type embedded in `HomeArm`, `MoveArm`, `RelativeMove` and `JointMove` requests: `velocity_scale`, `acceleration_scale` (float64), each `0.0` to `1.0`, clamped on the receiving end. `0.0` (the default) leaves it unset: Pilz moves then run at full speed, and OMPL moves use MoveIt's default scaling.

## Where everything lives on disk

The middleware doesn't stand alone, it builds inside a workspace alongside Kinova's own driver stack, while the proxy lives entirely separately:

```mermaid
flowchart TD
    WS["~/workspace/"] --> RKW["ros2_kortex_ws/"]
    WS --> PROXY["embodied-ai-proxy/"]

    RKW --> SRC["src/"]
    RKW --> BUILD["install/, build/, log/<br/>created by colcon build"]

    SRC --> RK["ros2_kortex/<br/>Kinova's official driver + MoveIt config"]
    SRC --> MID["ROS2-middleware/<br/>this repo"]

    PROXY --> PSRC["backend/, llm-proxy/, tui/, data/config/"]
```

See [Installation](installation.md) for how this layout gets built up step by step.

## Package layout reference

```
ROS2-middleware/
  docs/                           this documentation
  kinova_interface/
    data/configs/
      env/                        workspace_objects.yaml, movements_and_orientations.yaml
      motion_settings.yaml        hardware and grasping parameters
      moveit/joint_limits.yaml    joint limits loaded by move_group
    kinova_interface/
      nodes/                      the four ROS nodes (entry points)
      actions/                    one module per recipe action + arm_actions.py (shared clients) and motion.py (planning)
      utils/                      robot.py, geometry.py, grasping.py, ros.py
    scripts/run_recipe.py         send a recipe file to /execute_recipe
    scripts/check_orientations.py check orientation presets on the arm (see testing.md)
    launch/robot.launch.py        the whole stack, including rosbridge_server
    recipes/                      task_recipe.json, test_suite/, test_suite/qa_middleware/
    test/                         unit tests
  kinova_interfaces/
    srv/, msg/                    custom service and message definitions
  computer_vision/                optional Kinect vision node and its launch/config
```

For the proxy's layout, see its README.

Next: [Safety and Contracts](safety-and-contracts.md)
