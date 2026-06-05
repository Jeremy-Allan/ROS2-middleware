# Swinburne Lab: Kinova Gen3 Lite - ROS 2 Unified Controller

This repository contains a modular ROS 2 middleware for controlling the Kinova Gen3 Lite robotic arm. It has evolved from a simple manual CLI to a distributed "Supervisor" architecture that uses JSON recipes for automated task execution.

---

## 💻 Prerequistites

Ensure your host machine meets the following requirements prior to building the workspace:
```text
- Operating System:    Ubuntu 22.04 LTS
- ROS2 Distribution:   Humble
- Python:              3.10+
```
External Requisite: LLM Proxy repository (not in this repo)

---

## 🏗️ Distributed Architecture

The middleware is now split into three specialized nodes that communicate over the ROS 2 network:

1.  **Hardware Interface Server (`hardware_interface_client.py`):** Acts as the "Muscle." It provides low-level Action Clients for MoveIt 2 (Arm) and the Gripper Controller.
2.  **Environment Mapping Node (`environment_mapping_node.py`):** Acts as the connecting Node between stored coordinate positions and other nodes. Provides services upon request, using custom services: get_coordinates, get_relative_movement, get_robot_parameters. of which structure is stored in the `"kinova_interfaces"` package. 
3.  **JSON Parser Node (`json_parser_node.py`):** Acts as the "Brain" or "Supervisor." It reads a high-level `task_recipe.json`, fetches coordinates from the dictionary node, and orchestrates the sequence of movements.

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

- Execution is strictly sequential unless explicitly parallelised via executor policy.

### Output Contract
- Middleware returns execution status per step:
  - SUCCESS
  - FAILED
  - TIMEOUT
  - INVALID_REQUEST

This contract defines the boundary between task intent and physical actuation.

---

## 📖 Features & Capabilities

*   **Automated Recipes:** Define a sequence of tasks (pick, place, home) in a standard JSON format.
*   **Object-Based Targeting:** Instead of raw coordinates, tell the robot to move to `"red_cube_pickup"`. The system resolves the physical location via the Environment Mapping Node.
*   **Real-time Parameter Updates:** Update object coordinates in the Environment Mapping Node without restarting the supervisor
*   .
*   **Smart Threading:** Uses `MultiThreadedExecutor` and `threading.Event()` to ensure one action completes before the next begins.
*   **MoveIt 2 Integration:** Full path planning and self-collision avoidance are handled automatically.

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
│   	│					├── recipe_l1_single_move.json           # Minimal single-action movement validation
│   	│					├── recipe_l2_move_and_gripper.json      # Combined motion + gripper coordination test
│   	│					├── recipe_l3_home_move_gripper.json     # Home reset + sequential motion validation
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

## 🛠️ Building the Workspace

Ensure your ROS 2 workspace is built and sourced before running. Execute the following from the root of your ROS 2 workspace:

```bash
cd ~/workspace/ros2_kortex_ws
colcon build --packages-select kinova_interface kinova_interfaces
source install/setup.bash
```

---

## 🚀 Running the Automated System

You can launch the entire stack (Hardware, MoveIt, RViz, and the Automation Supervisor) with a single command. By default, the system boots in simulation (`use_fake_hardware:=true`) and waits for recipes via a ROS service.

```bash
# Launch in simulation mode waiting for recipe service calls
ros2 launch kinova_interface robot.launch.py

# Launch on physical hardware (REQUIRES robot_ip)
ros2 launch kinova_interface robot.launch.py use_fake_hardware:=false robot_ip:=192.168.1.10

# Launch and immediately execute a specific static recipe
ros2 launch kinova_interface robot.launch.py recipe:=task_recipe.json
```

---

**⚠️ Important Hardware Note:**
If you set `use_fake_hardware:=false` to connect to the physical robot, you **MUST** explicitly provide the `robot_ip` argument. Failure to provide the IP will result in a pre-launch `RuntimeError` to prevent unintended connections.

---
📜 The JSON Recipe Contract

Tasks are defined in a strict JSON format. The JSON Parser reads these steps sequentially.
Simple Example 'recipe_l1_single_move.json':
```json
{
  "recipe_name": "L1 - Single Move",
  "description": "Moves the arm to a single known object position",
  "steps": [
    {
      "step_id": 1,
      "action": "move_arm",
      "parameters": { "target": "red_cube_pickup" },
      "description": "Move to red cube pickup position"
    }
  ]
}
```
Complex Routine Example 'task_recipe.json' snippet:
```json
{
  "recipe_name": "Object Collection Routine",
  "steps": [
    {
      "step_id": 3,
      "action": "move_arm",
      "parameters": {
        "target": "red_cube_pickup"
      },
      "description": "Navigate to the red cube"
    },
    {
      "step_id": 4,
      "action": "gripper",
      "parameters": {
        "position": 0.0
      },
      "description": "Close fingers"
    },
    {
      "step_id": 5,
      "action": "relative_move",
      "parameters": {
        "vector": "move_upwards"
      },
      "description": "Lift the cube slightly"
    }
  ]
}
```
---

## Runtime Execution Guarantees

The middleware enforces deterministic execution semantics.

### Guarantees
- Steps execute strictly in declared order
- Each step must complete before the next begins
- Failed steps do NOT propagate silently
- Partial execution states are explicitly tracked

### Execution State Model
Each task maintains the following states:
- PENDING
- RUNNING
- COMPLETED
- FAILED
- ABORTED

### Determinism Constraint
Given identical input recipes and environment state, execution behaviour remains consistent unless:
- Physical hardware constraints intervene
- ROS2 planning failure occurs
- External interruption is triggered

---

## Safety Model

This middleware enforces a multi-layer safety system across planning and execution.

### Safety Layers

#### 1. Schema Validation Layer
- Rejects malformed or incomplete JSON structures

#### 2. Action Whitelist Layer
- Only pre-approved actions are executable

#### 3. Motion Planning Layer 
- Enforces collision avoidance
- Validates movement feasibility

#### 4. Execution Guard Layer
- Prevents simultaneous conflicting commands
- Ensures single active execution thread per robot

#### 5. Hardware Protection Layer
- Enforces joint limits
- Prevents unsafe velocity or torque commands

### Emergency Behaviour
If a safety violation is detected:
- Execution is immediately halted
- Robot transitions to safe idle state
- Error is propagated upstream via ROS diagnostics

### Safety Contract
- No action bypasses MoveIt planning layer.
- No raw joint commands are accepted from external sources.
- All movements are collision-checked before execution.
- Execution halts immediately on invalid or missing parameters.

---

## 📂 Configuration Files

*   **Recipes:** Found in `recipes/task_recipe.json`. Defines the steps.
*   **Coordinates:** Found in `data/coordinate_dictionary.json`. Maps object names to `X, Y, Z`.
*   **Relative Movements:** Found in `data/relative_movement.json`. Maps movement names to `X, Y, Z`.

---

## System Readiness Definition

The middleware is considered operational only if all conditions are satisfied:

- ROS2 core is running and responsive
- All middleware nodes are active and registered
- MoveIt planning pipeline is operational
- Environment mapping service responds correctly
- JSON parser node successfully executes test recipe
- No unresolved ROS2 service exceptions exist

If any condition fails, the system is considered **NON-OPERATIONAL.**
