# Swinburne Lab: Kinova Gen3 Lite - ROS 2 Unified Controller

This repository contains a modular ROS 2 middleware for controlling the Kinova Gen3 Lite robotic arm. It has evolved from a simple manual CLI to a distributed "Supervisor" architecture that uses JSON recipes for automated task execution.

---

## 🏗️ Distributed Architecture

The middleware is split into four specialized nodes that communicate over the ROS 2 network:

1.  **Hardware Interface Server (`hardware_interface_client.py`):** Acts as the "Muscle." It provides low-level Action Clients for MoveIt 2 (Arm) and the Gripper Controller. It also monitors physical hardware faults and reports execution failures (like planning errors or timeouts).
2.  **Environment Mapping Node (`environment_mapping_node.py`):** Acts as the database node mapping coordinate positions to semantic names. It provides coordinate lookup, relative movement vectors, and robot parameters via custom services.
3.  **JSON Parser Node (`json_parser_node.py`):** Acts as the "Brain" or "Supervisor." It reads a high-level `task_recipe.json`, fetches target coordinates, and orchestrates the step-by-step movement sequence.
4.  **Telemetry & Diagnostics Node (`telemetry_node.py`):** Acts as the "Monitor." It aggregates status heartbeats from all nodes, flags timeouts if any node crashes, and handles physical robot fault resets.

---

## 📖 Features & Capabilities

*   **Automated Recipes:** Define a sequence of tasks (pick, place, home) in a standard JSON format.
*   **Object-Based Targeting:** Instead of raw coordinates, tell the robot to move to `"red_cube_pickup"`. The system resolves the physical location via the Environment Mapping Node.
*   **Real-time Parameter Updates:** Update object coordinates in the Environment Mapping Node without restarting the supervisor.
*   **Smart Threading:** Uses `MultiThreadedExecutor` and `threading.Event()` to ensure one action completes before the next begins.
*   **MoveIt 2 Integration:** Full path planning and self-collision avoidance are handled automatically.
*   **Centralized Telemetry & Heartbeats (New):** Nodes report their state (IDLE, BUSY, FAULT) at 2Hz to `/status/node_report`. If any node stops responding for more than 1.5 seconds, the system flags a heartbeat timeout and transitions to a global fault state.
*   **Hardware Fault Monitoring & Recovery (New):** The hardware interface subscribes directly to the robot's hardware fault status (`/fault_controller/is_faulted`). If a protective stop or hardware fault triggers, the telemetry node transitions the system to `SYSTEM_FAULT`. You can clear hardware faults on the physical arm by calling the `/system/reset_fault` service.
*   **Improved Error Diagnostics (New):** When a movement fails, the hardware client checks the active hardware fault status to distinguish between algorithmic planning failures (like an unreachable position or planning timeout) and real physical stops.

---

## 🛰️ Centralized Telemetry & System Recovery

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

---

## 🛠️ Building the Workspace

Ensure your ROS 2 workspace is built and sourced before running:

```bash
cd ~/workspace/ros2_kortex_ws
colcon build --packages-select kinova_interface kinova_interfaces
source install/setup.bash
```

---

## 🚀 Running the Automated System

Before running the launch commands, make sure you set the custom log directory so all the logs go to the `middleware_logs/` folder instead of the default ROS hidden directories:

```bash
export ROS_LOG_DIR=$(pwd)/middleware_logs
```

Once that's exported, you can launch the entire stack (Hardware, MoveIt, RViz, and the Automation Supervisor) with a single command. By default, the system boots in simulation (`use_fake_hardware:=true`) and waits for recipes via a ROS service.

```bash
# Launch in simulation mode waiting for recipe service calls
ros2 launch kinova_interface robot.launch.py

# Launch on physical hardware (REQUIRES robot_ip)
ros2 launch kinova_interface robot.launch.py use_fake_hardware:=false robot_ip:=192.168.1.10

# Launch and immediately execute a specific static recipe
ros2 launch kinova_interface robot.launch.py recipe:=task_recipe.json

# Execute recipe on the physical robot
ros2 launch kinova_interface robot.launch.py use_fake_hardware:=false robot_ip:=192.168.1.10 recipe:=task_recipe.json
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

> 📝 **Note on `enable_individual_logs`:**
> When `enable_individual_logs:=true` is passed, the system actively splits and writes the individual log outputs of each custom node into distinct `python3_*` log files. When set to `false`, these individual log files are suppressed to save disk space and keep your workspace clean.

**⚠️ Important Hardware Note:**
If you set `use_fake_hardware:=false` to connect to the physical robot, you **MUST** explicitly provide the `robot_ip` argument. Failure to provide the IP will result in a pre-launch `RuntimeError` to prevent unintended connections.

---

## 📂 Configuration Files

*   **Recipes:** Found in `recipes/task_recipe.json`. Defines the steps.
*   **Coordinates:** Found in `data/configs/env/coordinate_dictionary.json`. Maps object names to `X, Y, Z`.
*   **Relative Movements:** Found in `data/configs/env/relative_movement.json`. Maps movement names to `X, Y, Z`.
*   **Obstacles:** Found in `data/obstacles.json`. Defines the physical obstacles spawned in MoveIt's planning scene.
