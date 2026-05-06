# Swinburne Lab: Kinova Gen3 Lite - ROS 2 Unified Controller

This repository contains a modular ROS 2 middleware for controlling the Kinova Gen3 Lite robotic arm. It has evolved from a simple manual CLI to a distributed "Supervisor" architecture that uses JSON recipes for automated task execution.

---

## 🏗️ Distributed Architecture

The middleware is now split into three specialized nodes that communicate over the ROS 2 network:

1.  **Hardware Interface Server (`hardware_interface_client.py`):** Acts as the "Muscle." It provides low-level Action Clients for MoveIt 2 (Arm) and the Gripper Controller.
2.  **Environment Mapping Node (`environment_mapping_node.py`):** Acts as the connecting Node between stored coordinate positions and other nodes. Provides services upon request, using custom services: get_coordinates, get_relative_movement, get_robot_parameters. of which structure is stored in the `"kinova_interfaces"` package. 
3.  **JSON Parser Node (`json_parser_node.py`):** Acts as the "Brain" or "Supervisor." It reads a high-level `task_recipe.json`, fetches coordinates from the dictionary node, and orchestrates the sequence of movements.

---

## 📖 Features & Capabilities

*   **Automated Recipes:** Define a sequence of tasks (pick, place, home) in a standard JSON format.
*   **Object-Based Targeting:** Instead of raw coordinates, tell the robot to move to `"red_cube_pickup"`. The system resolves the physical location via the Environment Mapping Node.
*   **Real-time Parameter Updates:** Update object coordinates in the Environment Mapping Node without restarting the supervisor.
*   **Smart Threading:** Uses `MultiThreadedExecutor` and `threading.Event()` to ensure one action completes before the next begins.
*   **MoveIt 2 Integration:** Full path planning and self-collision avoidance are handled automatically.

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

You can launch the entire stack (Hardware, MoveIt, RViz, and the Automation Supervisor) with a single command. By default, the system boots in simulation (`use_fake_hardware:=true`) and waits for recipes via a ROS service.

```bash
# Launch in simulation mode waiting for recipe service calls
ros2 launch kinova_interface robot.launch.py

# Launch on physical hardware (REQUIRES robot_ip)
ros2 launch kinova_interface robot.launch.py use_fake_hardware:=false robot_ip:=192.168.1.10

# Launch and immediately execute a specific static recipe
ros2 launch kinova_interface robot.launch.py recipe:=task_recipe.json
```

**⚠️ Important Hardware Note:**
If you set `use_fake_hardware:=false` to connect to the physical robot, you **MUST** explicitly provide the `robot_ip` argument. Failure to provide the IP will result in a pre-launch `RuntimeError` to prevent unintended connections.

---

## 📂 Configuration Files

*   **Recipes:** Found in `recipes/task_recipe.json`. Defines the steps.
*   **Coordinates:** Found in `data/coordinate_dictionary.json`. Maps object names to `X, Y, Z`.
*   **Relative Movements:** Found in `data/relative_movement.json`. Maps movement names to `X, Y, Z`.
