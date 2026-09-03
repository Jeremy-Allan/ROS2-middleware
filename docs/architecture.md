[Docs Home](README.md)

# Architecture

## Full pipeline

```mermaid
flowchart LR
    U["You"] --> P["embodied-ai-proxy"]
    P --> L(("LLM"))
    L --> P
    P -->|"/execute_recipe via websocket"| B["rosbridge_server"]
    B --> M["ROS2-middleware<br/>four nodes"]
    M --> R["robot arm"]
    M -.->|"/system/status"| B -.-> P
```

This is the quick-glance version. For the full detailed diagram, node by node with every service call labeled:

![Full architecture diagram, node by node with every service call labeled](diagrams/architecture.png)

The proxy never calls a ROS 2 service directly. Every request goes out over the websocket to `rosbridge_server`, which translates it into a real service call against this middleware. Telemetry flows back the same way (the dashed path above), `rosbridge` subscribes to `/system/status` on the proxy's behalf and forwards messages back over the websocket.

## Middleware node deep dive

**`hardware_interface_client.py`, the only node that touches the robot:**
- Exposes four services under its private namespace: `/kinova_hardware_client/home_arm`, `move_arm`, `move_gripper`, `relative_move`.
- Internally a client of two ROS 2 actions: `move_action` (MoveIt 2's `MoveGroup`, for Cartesian moves and the fixed joint-space `home` pose) and `/gen3_lite_2f_gripper_controller/gripper_cmd` (direct gripper control, bypassing MoveIt entirely).
- Movement calls block synchronously on a `threading.Event()` until the action's result callback fires, which is what makes each recipe step wait for the robot to actually finish.
- Subscribes to `/fault_controller/is_faulted` to know instantly if the arm enters a hardware fault state.

**`environment_mapping_node.py`, the read-mostly knowledge base:**
- Loads `object_dictionary.json`, `relative_movement.json`, `orientation_presets.json`, and `obstacles.json` once at startup. No live file-watching; a config change needs a restart.
- Exposes `/get_coordinates`, `/get_object_info`, `/get_relative_movement`, `/get_orientation_preset`, `/get_robot_parameters`, all pure lookups against the in-memory data.
- Separately pushes every entry in `obstacles.json` into MoveIt's planning scene once `/apply_planning_scene` becomes available.

**`json_parser_node.py`, orchestration only:**
- Never talks to MoveIt or the gripper directly, only ever calls the services the other two nodes expose.
- Can get a recipe two ways: a static file via the `recipe` launch parameter, or a dynamic JSON string via the `/execute_recipe` service, which is exactly what the proxy calls at runtime.
- Iterates recipe steps in order, dispatching on `action`, stopping at the first failure.

**`telemetry_node.py`, pure aggregation:**
- Subscribes to `/status/node_report`, publishes an aggregated `/system/status` every 0.5s using a worst-case-wins rule across all tracked nodes.
- Hosts `/system/reset_fault`, forwarding to the real Kortex driver's own reset service.
- Automatically reconfigures and reactivates the hardware fault controller in the background if it detects the known driver startup race condition (the physical driver takes several seconds to establish its connection, which otherwise leaves `fault_controller` stuck unconfigured).

## Proxy internals

The proxy's own components (`llm_proxy.py`, the LLM adapters, `ros2_bridge_ws`, the Textual terminal interface) are documented in the embodied-ai-proxy repository itself: [github.com/paul-isit/embodied-ai-proxy](https://github.com/paul-isit/embodied-ai-proxy). This page covers this middleware's own internals in depth; the pipeline diagram above is the shared context between the two.

## Custom interface types (middleware)

**Services** (request fields above the separator, response fields below):

| Service | Request | Response |
|---|---|---|
| `HomeArm` | `motion_params` (`MotionParams`) | `success`, `message` |
| `MoveArm` | `target_position` (`Point`), `has_orientation` (bool), `roll`, `pitch`, `yaw` (float64), `motion_params` (`MotionParams`) | `success`, `message` |
| `MoveGripper` | `position` (float64) | `success`, `message` |
| `RelativeMove` | `vx`, `vy`, `vz` (float64), `has_orientation` (bool), `roll_delta`, `pitch_delta`, `yaw_delta` (float64), `motion_params` (`MotionParams`) | `success`, `message` |
| `GetObjectCoordinates` | `object_id` (string) | `x`, `y`, `z`, `success`, `message` |
| `GetObjectInfo` | `object_id` (string) | `pose` (`Pose`), `shape` (`SolidPrimitive`), `success`, `message` |
| `GetRelativeMovement` | `move_id` (string) | `x`, `y`, `z`, `success`, `message` |
| `GetOrientationPreset` | `preset_name` (string) | `roll`, `pitch`, `yaw`, `success`, `message` |
| `GetRobotParameters` | (none) | `object_list[]`, `movement_names[]`, `orientation_names[]` |
| `ExecuteRecipe` | `recipe_json` (string) | `success`, `message` |

`has_orientation` defaults to `false`, so existing callers that never set it get identical behavior to before this field existed, no orientation constraint applied, MoveIt picks whatever orientation it wants. `MoveArm`'s request fields changed shape (`x`/`y`/`z` became `target_position`), a breaking change to the wire format, not additive.

**Messages** (published continuously, not request/response):

- `ExtendedStatus`: one node's heartbeat: `node_name`, `state` (0 = IDLE, 1 = BUSY, 2 = FAULT), `status_message`, `last_command_valid`.
- `SystemSummary`: the aggregated view: `summary_state` plus `individual_states[]`.
- `MotionParams`: not published on its own, a shared field type embedded in `HomeArm`, `MoveArm`, and `RelativeMove` requests: `velocity_scale`, `acceleration_scale` (float64), each `0.0` to `1.0`. `0.0` (the default) means "use the arm's configured default," values are clamped into range on the receiving end.

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

    PROXY --> BRIDGE["ros2_bridge_ws/<br/>src/custom_bridge_pkg/"]
    PROXY --> PSRC["src/, configs/, tests/"]
```

See [Installation](installation.md) for how this layout gets built up step by step.

## Package layout reference

```
ROS2-middleware/
  README.md
  docs/                          this documentation
  kinova_interface/
    data/configs/env/            object_dictionary.json, relative_movement.json, orientation_presets.json, obstacles.json
    kinova_interface/             the four node source files
    launch/robot.launch.py
    recipes/                      task_recipe.json, test_suite/
  kinova_interfaces/
    srv/, msg/                    custom service and message definitions

embodied-ai-proxy/
  main.py                         entry point, launches the TUI
  evaluate_proxy.py                YAML-based LLM evaluation runner
  configs/
    llm_config.json                LLM provider configuration
    system_prompt.md               LLM behavior rules and few-shot examples
    json_schema.json               strict output schema
  ros2_bridge_ws/
    src/custom_bridge_pkg/         launches rosbridge_server
  src/
    backend/
      llm_proxy.py                 main orchestrator
      defaults.py                  dead fallback prompt
      llm_adapters/                one file per provider
    frontend/
      tui_app.py                   terminal interface
      components/                  input bar, log panel, sidebar, status panel
  tests/                           basic_tests.yaml, test_cases.yaml
```

Next: [Safety and Contracts](safety-and-contracts.md)
