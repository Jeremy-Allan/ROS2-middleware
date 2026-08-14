[Docs Home](README.md)

# Architecture

## Full pipeline

```
  You (typed command)
        |
        v
  +----------------------------+       +------------------------------------+
  |   embodied-ai-proxy          |       |   ROS2-middleware                    |
  |   inference domain (src/)    |       |                                      |
  |                              |       |   json_parser_node.py "The Brain"   |
  |   llm_proxy.py               |       |         |                          |
  |     builds prompt      ------+------>|         v                          |
  |     calls LLM adapter        |       |   environment_mapping_node.py       |
  |     validates JSON schema    |       |   "The Database"                    |
  |     (Ollama / OpenAI /       |       |         |                          |
  |      Anthropic / Gemini)     |       |         v                          |
  |                              |       |   hardware_interface_client.py      |
  |   Terminal UI (Textual)      |<------+------   "The Muscle"                |
  |                              | ws:9090 |         |                          |
  +----------------------------+       |         v  (MoveIt 2, then robot)   |
        ^                              |                                      |
        |                              |   telemetry_node.py "The Monitor"    |
        |     ros2_bridge_ws/           |         ^                          |
        +-----rosbridge_server----------+---------+                          |
              (websocket <-> ROS 2)     +------------------------------------+
```

The proxy never calls a ROS 2 service directly. Every request goes out over the websocket to `rosbridge_server`, which translates it into a real service call against this middleware. Telemetry flows back the same way, `rosbridge` subscribes to `/system/status` on the proxy's behalf and forwards messages back over the websocket.

## Middleware node deep dive

**`hardware_interface_client.py`, the only node that touches the robot:**
- Exposes four services under its private namespace: `/kinova_hardware_client/home_arm`, `move_arm`, `move_gripper`, `relative_move`.
- Internally a client of two ROS 2 actions: `move_action` (MoveIt 2's `MoveGroup`, for Cartesian moves and the fixed joint-space `home` pose) and `/gen3_lite_2f_gripper_controller/gripper_cmd` (direct gripper control, bypassing MoveIt entirely).
- Movement calls block synchronously on a `threading.Event()` until the action's result callback fires, which is what makes each recipe step wait for the robot to actually finish.
- Subscribes to `/fault_controller/is_faulted` to know instantly if the arm enters a hardware fault state.

**`environment_mapping_node.py`, the read-mostly knowledge base:**
- Loads `coordinate_dictionary.json`, `relative_movement.json`, and `obstacles.json` once at startup. No live file-watching; a config change needs a restart.
- Exposes `/get_coordinates`, `/get_relative_movement`, `/get_robot_parameters`, all pure lookups against the in-memory data.
- Separately pushes every entry in `obstacles.json` into MoveIt's planning scene once `/apply_planning_scene` becomes available.

**`json_parser_node.py`, orchestration only:**
- Never talks to MoveIt or the gripper directly, only ever calls the services the other two nodes expose.
- Can get a recipe two ways: a static file via the `recipe` launch parameter, or a dynamic JSON string via the `/execute_recipe` service, which is exactly what the proxy calls at runtime.
- Iterates recipe steps in order, dispatching on `action`, stopping at the first failure.

**`telemetry_node.py`, pure aggregation:**
- Subscribes to `/status/node_report`, publishes an aggregated `/system/status` every 0.5s using a worst-case-wins rule across all tracked nodes.
- Hosts `/system/reset_fault`, forwarding to the real Kortex driver's own reset service.
- Automatically reconfigures and reactivates the hardware fault controller in the background if it detects the known driver startup race condition (the physical driver takes several seconds to establish its connection, which otherwise leaves `fault_controller` stuck unconfigured).

## Proxy component deep dive

**`llm_proxy.py`, the orchestrator:** loads `llm_config.json`, `json_schema.json`, and `system_prompt.md` at startup, maintains a persistent websocket connection to the bridge, and exposes the main `process_user_request()` method the TUI calls per command: fetch known objects, build the prompt, call the LLM, validate the response, forward it to `/execute_recipe`.

**`llm_adapters/`, one file per provider:** `ollama.py`, `openai.py`, `anthropic.py`, `gemini.py`, all implementing the same `generate(prompt) -> str` interface against a shared `base.py` that handles the HTTP request and common error cases. Adding a new provider means writing a new adapter here and registering it in `__init__.py`'s `LLM_REGISTRY`.

**`ros2_bridge_ws/`, the proxy's own tiny ROS 2 workspace:** contains a single package, `custom_bridge_pkg`, whose only job is launching `rosbridge_server` on port 9090. It has no custom nodes or logic of its own.

**`src/frontend/`, the terminal interface:** built with Textual. `tui_app.py` is the main application; `components/` holds the input bar, log panel, sidebar (which renders the live telemetry from `/system/status`), and status panel (bridge connection indicator).

## Custom interface types (middleware)

**Services** (request fields above the separator, response fields below):

| Service | Request | Response |
|---|---|---|
| `HomeArm` | (none) | `success`, `message` |
| `MoveArm` | `x`, `y`, `z` (float64) | `success`, `message` |
| `MoveGripper` | `position` (float64) | `success`, `message` |
| `RelativeMove` | `vx`, `vy`, `vz` (float64) | `success`, `message` |
| `GetObjectCoordinates` | `object_id` (string) | `x`, `y`, `z`, `success`, `message` |
| `GetRelativeMovement` | `move_id` (string) | `x`, `y`, `z`, `success`, `message` |
| `GetRobotParameters` | (none) | `object_list[]`, `movement_names[]` |
| `ExecuteRecipe` | `recipe_json` (string) | `success`, `message` |

**Messages** (published continuously, not request/response):

- `ExtendedStatus`: one node's heartbeat: `node_name`, `state` (0 = IDLE, 1 = BUSY, 2 = FAULT), `status_message`, `last_command_valid`.
- `SystemSummary`: the aggregated view: `summary_state` plus `individual_states[]`.

## Package layout reference

```
ROS2-middleware/
  README.md
  docs/                          this documentation
  kinova_interface/
    data/configs/env/            coordinate_dictionary.json, relative_movement.json, obstacles.json
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
