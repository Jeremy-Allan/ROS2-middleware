# Swinburne Lab: Kinova Gen3 Lite - ROS 2 Unified Controller

This repository contains modular ROS 2 middleware for controlling the Kinova Gen3 Lite robotic arm. It includes an atomic MoveIt Task Constructor (MTC) pick-and-place path so that grasp approach, lift, lower, and retreat motions are planned as constrained Cartesian stages instead of unrelated position-only goals.

Read [the MTC integration and commissioning guide](docs/MTC_INTEGRATION.md) before changing manipulation geometry or executing on physical hardware.

---

## Prerequistites

Ensure your host machine meets the following requirements prior to building the workspace:
```text
- Operating System:    Ubuntu 22.04 LTS
- ROS2 Distribution:   Humble
- Python:              3.10+
```
External Requisite: LLM Proxy repository (not in this repo)

The ROS workspace must also provide the Humble versions of
`moveit_task_constructor_core`, `moveit_task_constructor_capabilities`,
`py_binding_tools`, `kortex_bringup`, and
`kinova_gen3_lite_moveit_config`. The Python implementation was checked
against MTC `humble` commit
`756634951326ae17ae099882f7110c6f1d0a98c0`; build that source in the same
MoveIt overlay. The executor intentionally refuses to start unless the
repository patch
`patches/moveit_task_constructor_humble_release_gil.patch` has been applied.
Use the ROS 2 branch of `py_binding_tools` pinned at
`8918f0ece3d7977a4963c54472531906e01e40c2`, then run the ABI smoke test in
the integration guide. Do not assume an older Humble binary exposes the same
bindings, and do not mix Rolling bindings with Humble libraries.

---

## Distributed Architecture

The middleware is split into five specialized nodes that communicate over the ROS 2 network:

1.  **Hardware Interface Server (`hardware_interface_client.py`):** Acts as the "Muscle." It provides low-level Action Clients for MoveIt 2 (Arm) and the Gripper Controller. It also monitors physical hardware faults and reports execution failures (like planning errors or timeouts).
2.  **Environment Mapping Node (`environment_mapping_node.py`):** Acts as the database node mapping coordinate positions to semantic names. It provides coordinate lookup, relative movement vectors, and robot parameters via custom services.
3.  **JSON Parser Node (`json_parser_node.py`):** Acts as the "Brain" or "Supervisor." It reads a high-level `task_recipe.json`, fetches target coordinates, and orchestrates the step-by-step movement sequence.
4.  **Telemetry & Diagnostics Node (`telemetry_node.py`):** Acts as the "Monitor." It aggregates status heartbeats from all nodes, flags timeouts if any node crashes, and handles physical robot fault resets.
5.  **MTC Task Node (`mtc_task_node.py`):** Serves the cancellable `/mtc_task_node/pick_place` action. It validates object geometry and calibrated TCP poses, builds a complete MTC task, publishes the best solution for inspection, and optionally executes it.

---

## System Execution Contract

The Kinova Gen3 Lite middleware enforces a strict execution contract to ensure safe and deterministic robotic behaviour.

### Input Contract (External Systems < Middleware)
- Input is strictly received via ROS 2 services or JSON-parsed execution nodes.
- No direct natural language input is accepted by this repository.
- All commands must originate from validated upstream systems (e.g LLM proxy).

### Execution Contract (Middleware Behaviour Rules)
- All tasks are decomposed into discrete, ordered steps.
- Each step MUST map to a known action type:
  - move_arm
  - move_gripper
  - relative_move
  - home_arm
  - pick_and_place

- Execution is strictly sequential unless explicitly parallelised via executor policy.

### Output Contract
- Middleware returns execution status per step:
  - SUCCESS
  - FAILED
  - TIMEOUT
  - INVALID_REQUEST

This contract defines the boundary between task intent and physical actuation.

---

## Features & Capabilities

*   **Automated Recipes:** Accept only atomic MTC `pick_and_place` steps; homing and gripper control are internal task stages.
*   **Object-Based Targeting:** Instead of raw coordinates, request configured IDs such as `"red_cube"` and `"delivery_tray"`.
*   **Atomic Pick and Place:** MTC plans collision-object insertion, calibrated grasp IK, gripper contact, attachment, transfer, detachment, and retreat as one coherent task.
*   **Deterministic Near-Object Motion:** Cartesian approach/lift/lower/retreat stages enforce the configured axes and full path fraction; OMPL remains responsible for collision-aware free-space connections.
*   **Smart Threading:** Uses `MultiThreadedExecutor` and `threading.Event()` to ensure one action completes before the next begins.
*   **MoveIt 2 Integration:** Full path planning and self-collision avoidance are handled automatically.
*   **Centralized Telemetry & Heartbeats (New):** Nodes report their state (IDLE, BUSY, FAULT) at 2Hz to `/status/node_report`. If any node stops responding for more than 1.5 seconds, the system flags a heartbeat timeout and transitions to a global fault state.
*   **Hardware Fault Monitoring & Recovery (New):** The hardware interface subscribes directly to the robot's hardware fault status (`/fault_controller/is_faulted`). If a protective stop or hardware fault triggers, the telemetry node transitions the system to `SYSTEM_FAULT`. You can clear hardware faults on the physical arm by calling the `/system/reset_fault` service.
*   **Improved Error Diagnostics (New):** When a movement fails, the hardware client checks the active hardware fault status to distinguish between algorithmic planning failures (like an unreachable position or planning timeout) and real physical stops.

---

## Centralized Telemetry & System Recovery

The **Telemetry & Diagnostics Node** (`telemetry_node.py`) acts as the safety monitor for the entire system. It collects individual node heartbeats, aggregates the overall system status, and handles physical hardware fault recoveries.

### 1. Monitoring Overall System Status (`/system/status`)
The telemetry node listens to status messages from all other nodes at 2Hz on `/status/node_report` and aggregates them. Any active subscriber of `/system/status` can expect to receive a `kinova_interfaces/msg/SystemSummary` message with the following structure:

*   `uint8 summary_state` — The consolidated system state based on the worst-case scenario:
    *   `0` (`SYSTEM_READY`): All nodes are healthy and idle.
    *   `1` (`SYSTEM_BUSY`): The robot is actively planning or executing a task.
    *   `2` (`SYSTEM_FAULT`): A node has crashed (heartbeat timed out after 1.5s) or a physical hardware fault was detected.
*   `kinova_interfaces/ExtendedStatus[] individual_states` — An array showing the specific states of each reporting node. Each entry includes:
    *   `node_name`: Name of the node (e.g. `json_parser_node`).
    *   `state`: Node operational state (`0 = STATE_IDLE`, `1 = STATE_BUSY`, `2 = STATE_FAULT`).
    *   `status_message`: Human-readable context (e.g., current recipe step or error description).
    *   `last_command_valid`: Boolean indicating if the last transaction was successful.

#### How to view system telemetry from the terminal:
```bash
ros2 topic echo /system/status
```

### 2. Clearing Hardware Faults (`/system/reset_fault`)
When the physical robot triggers a hardware-level safety stop or fault, the system transitions to `SYSTEM_FAULT`. The telemetry node hosts a `/system/reset_fault` service (type `std_srvs/srv/Trigger`) to recover from these faults safely.

When called, this service forwards a recovery command down to the physical Kortex hardware driver (`/fault_controller/reset_fault`), resetting the arm and returning the system state to `SYSTEM_READY`.

#### How to trigger a fault reset from the terminal:
```bash
ros2 service call /system/reset_fault std_srvs/srv/Trigger {}
```

#### Physical Hardware Startup Note: Fault Controller Delayed Initialization
On the physical robot, the C++ hardware interface (`kortex_driver`) requires several seconds to successfully establish its network communication session with the physical Kinova Gen3 Lite over TCP/IP.

In standard launch sequences, starting the `fault_controller_spawner` instantly alongside `control_node` causes it to attempt configuration before the state interfaces (like `reset_fault/internal_fault`) are fully exported by the active physical connection. This would leave the `fault_controller` in an **`unconfigured`** state, rendering it inoperable and leaving the middleware blind to hardware faults.

**Our Fix (Self-Healing Workaround):** We implemented an automated, self-healing workaround within our ROS 2 middleware:
1. **Health Checking:** The `hardware_interface_client` node runs a periodic 2Hz health timer that queries `/controller_manager/list_controllers`. If the `fault_controller` is loaded but in an inactive or `unconfigured` state, it publishes a warning alert inside its telemetry status message (`"SYSTEM CONFIG WARNING: fault_controller is NOT active"`). This check is safely skipped in simulation where the controller is not loaded.
2. **Automated Activation:** The `telemetry_node` monitors heartbeats. Upon seeing this warning status, it automatically spins up a background thread that makes native ROS 2 service calls to the controller manager (`/controller_manager/configure_controller` and `/controller_manager/switch_controller`) to configure and activate the `fault_controller` on the fly.

This allows the entire system to boot up, automatically self-heal the driver controllers within seconds, and handle faults seamlessly without editing a single line of the official `ros2_kortex` package!

---

## Failure Mode Summary

The following table defines known system failure modes and their resolution pathways.

| Failure Mode              | Origin Layer            | Cause                                      | System Behaviour                     | Resolution Strategy                  |
|--------------------------|------------------------|------------------------------------------|-------------------------------------|--------------------------------------|
| Invalid JSON Recipe      | JSON Parser Node       | Malformed or incomplete structure        | Execution rejected                  | Schema validation + log output       |
| Unknown Action Type      | Supervisor Node        | Action not in whitelist                  | Step skipped / task aborted         | Update action registry              |
| Missing Object Mapping   | Environment Node       | Undefined object key                     | Service returns error               | Update coordinate dictionary         |
| ROS2 Service Timeout     | Middleware Layer       | Node overload or planning delay          | Step timeout triggered              | Retry or abort sequence             |
| Gripper Command Failure  | Hardware Interface     | Actuator limit or communication loss     | Emergency stop / retry              | Hardware reinitialization            |
| Collision Detection Stop | MoveIt Planning Layer  | Unsafe trajectory detected               | Movement aborted                    | Replan trajectory                    |
| Node Desynchronisation   | ROS2 Executor          | Race condition or threading delay        | Partial execution state             | Reset executor + restart pipeline    |

---
## Project Stucture
```text

ros2-middleware/
├── readme.md                                             # JSON Parser + sequential action execution design notes
├── test.txt                                              # Sprint 1 validation notes and temporary feature tracking
├── .gitignore                                            # Git ignore rules (build artifacts, ROS2 temp files, Python cache)
│
├── kinova_interface/                                     # Primary ROS2 middleware package (runtime execution layer)
│   	│
│   	├── package.xml				                      # ROS2 package manifest (dependencies + build configuration)
│   	├── setup.cfg                                     # Python packaging configuration (ROS2 entry points)
│   	├── setup.py                                      # Python package installation and module definition
│   	├── data/                                         # Runtime environment knowledge base
│   	│     │
│   	│  	  ├── coordinate_dictionary.json              # Object → absolute coordinate mapping for motion planning
│   	│	  ├── obstacle.json                           # Environment collision constraints and obstacle definitions
│   	│	  └──  relative_movement.json                 # Predefined relative motion vectors for controlled movement
│   	│
│   	│
│   	├── kinova_interface/                             # ROS2 Python node implementation package
│   	│		│
│   	│		├── __init__.py                           # Package initializer (ROS2 Python module entry)
│   	│		├── environment_mapping_node.py           # Maps object names to coordinates + exposes ROS2 services
│   	│		├── hardware_interface_client.py          # MoveIt2 + gripper action client abstraction layer
│   	│		└── json_parser_node.py                   # Parses structured JSON recipes into executable action sequences
│   	│
│   	│
│   	├── launch/                                       # ROS2 launch system entry points
│   	│		│
│   	│		└── robot.launch.py                       # Address JSON Parser PR comments
│   	│
│   	├── recipes/                                      # Launches middleware stack (nodes + optional simulation + MoveIt)
│   	│		│
│   	│		├── task_recipe.json                                 # Primary runtime execution recipe format (LLM-generated or static)
│   	│		└── test_suite/                                      # Deterministic validation and regression test recipes
│   	│					├── recipe_l1_single_move.json           # Red-cube MTC execution validation
│   	│					├── recipe_l2_move_and_gripper.json      # Blue-cube MTC execution validation
│   	│					├── recipe_l3_home_move_gripper.json     # Multi-object MTC execution validation
│   	│					├── recipe_l4_pick_and_place.json        # Full manipulation pipeline test
│   	│					├── recipe_l5_multi_object.json          # Multi-object sequencing validation
│   	│					└── recipe_l6_invalid.json               # Negative test case for validation failure handling
│   	│
│   	│
│   	└── resource/                                    # Package metadata and internal documentation
│   			│
│   			├── PROJECT-OVERVIEW.md                  # High-level system design and middleware architecture notes
│   			└── kinova_interface                     # ROS2 marker file for package registration
│
│
└── kinova_interfaces/
    │
   	├── CMakeLists.txt                                   # ROS2 build configuration
   	├── LICENSE                                          # License declaration for middleware distribution
   	└── package.xml                                      # Secondary ROS2 package manifest
        └── srv/                                         # ROS2 service interface definitions (middleware API contract)
               ├── ExecuteRecipe.srv                     # Executes full structured task recipe
               ├── GetObjectCoordinates.srv              # Retrieves absolute coordinates for named objects
               ├── GetRelativeMovement.srv               # Returns predefined relative motion vectors
               ├── GetRobotParamters.srv                 # Queries robot configuration and runtime parameters
               ├── HomeArm.srv                           # Sends robot to calibrated home position
               ├── MoveArm.srv                           # Executes absolute end-effector motion command
               ├── MoveGripper.srv                       # Controls gripper open/close state
               └── RelativeMove.srv                      # Executes relative spatial movement command
```

## Building the Workspace

Ensure your ROS 2 workspace is built and sourced before running. Execute the following from the root of your ROS 2 workspace:

```bash
cd ~/workspace/ros2_kortex_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --packages-up-to kinova_interface
source install/setup.bash
```

---

## Running the Automated System

Before running the launch commands, make sure you set the custom log directory so all the logs go to the `middleware_logs/` folder instead of the default ROS hidden directories:

```bash
export ROS_LOG_DIR=$(pwd)/middleware_logs
```

Once that's exported, you can launch the entire stack (Hardware, MoveIt, RViz, and the Automation Supervisor) with a single command. By default, the system boots in simulation (`use_fake_hardware:=true`) and waits for recipes via a ROS service.

```bash
# Launch in simulation mode waiting for recipe service calls
ros2 launch kinova_interface robot.launch.py

# Launch on physical hardware; MTC execution remains disabled by default
ros2 launch kinova_interface robot.launch.py use_fake_hardware:=false robot_ip:=192.168.1.10

# Launch a static recipe (it is rejected until execution gates are enabled)
ros2 launch kinova_interface robot.launch.py recipe:=task_recipe.json

# Physical execution is enabled by default; robot_ip remains required
ros2 launch kinova_interface robot.launch.py use_fake_hardware:=false \
  robot_ip:=192.168.1.10 recipe:=task_recipe.json
```

### New Launch Parameters for Logging & Debugging
On this branch, we added arguments to give you better control over node outputs and logging verbosity:
*   `debug_mode:=true` — Sets logging to `DEBUG` for just our middleware nodes, keeping external libraries quiet.
*   `core_debug:=true` — Turns on global `DEBUG` logging across the whole ROS 2 system (including low-level rcl/DDS components).
*   `enable_individual_logs:=true` — Disables the default library-wide log filtering so you can see full, unsuppressed output from external plugins.

For example, to debug only the custom middleware nodes in simulation:
```bash
ros2 launch kinova_interface robot.launch.py debug_mode:=true
```

### 📂 Understanding the Log Directory (`middleware_logs/`)
When you set `export ROS_LOG_DIR=$(pwd)/middleware_logs`, every launch session creates a timestamped folder inside `middleware_logs/` (e.g., `middleware_logs/2026-05-24-16-59-52-.../`).

Inside this folder, you will find:
*   `launch.log` — The primary launcher log containing start/stop and orchestrating details about the launch process itself.
*   `python3_<pid>_<timestamp>.log` — Logs for our Python-based middleware nodes (since ROS 2 runs them using the `python3` interpreter).
*   `move_group_*.log` and `ros2_control_node_*.log` — Logs specific to MoveIt 2 and standard C++ ROS 2 controller nodes.

> **Note on `enable_individual_logs`:**
> When `enable_individual_logs:=true` is passed, the system actively splits and writes the individual log outputs of each custom node into distinct `python3_*` log files. When set to `false`, these individual log files are suppressed to save disk space and keep your workspace clean.

---

** Important Hardware Note:**
If you set `use_fake_hardware:=false` to connect to the physical robot, you **MUST** explicitly provide the `robot_ip` argument. Failure to provide the IP will result in a pre-launch `RuntimeError` to prevent unintended connections.

---
## The JSON Recipe Contract

Automated recipes accept only atomic `pick_and_place` steps. The middleware
automatically opens the gripper, moves to configured home, follows the exact
configured pre-grasp/grasp approach, and completes placement:

```json
{
  "recipe_name": "Object collection",
  "steps": [
    {
      "step_id": 1,
      "action": "pick_and_place",
      "parameters": {
        "object": "red_cube",
        "destination": "delivery_tray"
      },
      "description": "Execute the calibrated home-started MTC task"
    }
  ]
}
```

AI-generated recipes always request execution; the model controls only the
object and destination. The committed configuration has `commissioned=true`
and `allow_mtc_execution` defaults to `true`. Pass
`allow_mtc_execution:=false` when execution must be disabled for diagnostics.
Legacy low-level services remain externally available for supervised
maintenance, but the recipe supervisor rejects them.

The lower-level action retains `plan_only` for engineering diagnostics, but
that field is not exposed to AI-generated recipes:

```bash
ros2 action send_goal /mtc_task_node/pick_place \
  kinova_interfaces/action/PickPlace \
  "{object_id: red_cube, destination_id: delivery_tray, plan_only: true}" \
  --feedback
```

Dynamic execution recipes still use `/execute_recipe`:

```json
{
  "recipe_name": "Object collection",
  "steps": [
    {
      "step_id": 1,
      "action": "pick_and_place",
      "parameters": {
        "object": "red_cube",
        "destination": "delivery_tray"
      },
      "description": "Execute the commissioned complete task"
    }
  ]
}
```
---

## Runtime Execution Guarantees

The recipe supervisor executes steps in order and stops at the first failed
step. Service/action waits have finite deadlines. The MTC server accepts only
one task at a time, supports cancellation, and returns the exact task failure
it can identify. Every automated recipe requests execution, but motion still
requires the commissioning and launch gates.

For MTC, this guarantee applies to the configured Cartesian near-object
segments. Sampling-based free-space `Connect` stages may still find different
collision-free transfers between runs.

The MTC configuration loader rejects unsafe/missing geometry, zero or
non-normalized quaternions, zero direction vectors, out-of-range gripper
positions, and unknown IDs before task construction. MoveIt checks the full
task against the robot model and planning scene. Hardware limits, the Kortex
fault controller, lab procedures, and an accessible emergency stop remain
independent safety layers.

The MTC server and legacy low-level services share one host-level OS motion
lease, so these middleware processes cannot command the robot concurrently.
All command-producing nodes must run on the same host and use the same
`motion_lock_path`; direct controller clients outside this middleware still
require ROS/DDS access control. After cancellation or execution failure while
an object may be attached, the lease is deliberately retained. Do not restart
or retry until the real robot state and planning-scene attachment have been
reconciled.

---

## Configuration Files

*   **Recipes:** Found in `recipes/task_recipe.json`. Defines the steps.
*   **Coordinates:** Found in `data/configs/env/coordinate_dictionary.json`. Maps object names to `X, Y, Z`.
*   **Relative Movements:** Found in `data/configs/env/relative_movement.json`. Maps movement names to `X, Y, Z`.
*   **Obstacles:** Found in `data/configs/env/obstacles.json`. Defines static collision objects.
*   **MTC Manipulation Objects:** Found in `data/configs/env/manipulation_objects.json`. Defines strict, versioned robot names, object collision geometry, normalized grasp/place quaternions, Cartesian directions/distances, touch links, planner settings, and gripper limits.

---

## System Readiness Definition

The middleware is considered operational only if all conditions are satisfied:

- ROS2 core is running and responsive
- All middleware nodes are active and registered
- MoveIt planning pipeline is operational
- `/execute_task_solution` is available from `move_group/ExecuteTaskSolutionCapability`
- The MTC Python ABI smoke test passes and `mtc_task_node` is reporting a heartbeat
- The selected object/destination configuration and calibration are valid
- Environment mapping service responds correctly
- JSON parser node successfully executes test recipe
- No unresolved ROS2 service exceptions exist

If any condition fails, the system is considered **NON-OPERATIONAL.**
