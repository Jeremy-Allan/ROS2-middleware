# Swinburne Lab: Kinova Gen3 Lite - ROS 2 Unified Controller

This repository contains a modular ROS 2 middleware for controlling the Kinova Gen3 Lite robotic arm. It has evolved from a simple manual CLI to a distributed "Supervisor" architecture that uses JSON recipes for automated task execution.

---

## 🏗️ Distributed Architecture

The middleware is now split into three specialized nodes that communicate over the ROS 2 network:

1.  **Hardware Interface Server (`hardware_interface_client.py`):** Acts as the "Muscle." It provides low-level Action Clients for MoveIt 2 (Arm) and the Gripper Controller.
2.  **Coordinate Dictionary Node (`coordinate_dictionary_node.py`):** Acts as the "Database." It stores physical object locations as ROS 2 parameters and serves them to other nodes upon request.
3.  **JSON Parser Node (`json_parser_node.py`):** Acts as the "Brain" or "Supervisor." It reads a high-level `task_recipe.json`, fetches coordinates from the dictionary node, and orchestrates the sequence of movements.

---

## 📖 Features & Capabilities

*   **Automated Recipes:** Define a sequence of tasks (pick, place, home) in a standard JSON format.
*   **Object-Based Targeting:** Instead of raw coordinates, tell the robot to move to `"red_cube_pickup"`. The system resolves the physical location via the Dictionary Node.
*   **Real-time Parameter Updates:** Update object coordinates in the Dictionary Node without restarting the supervisor.
*   **Smart Threading:** Uses `MultiThreadedExecutor` and `threading.Event()` to ensure one action completes before the next begins.
*   **MoveIt 2 Integration:** Full path planning and self-collision avoidance are handled automatically.

---

## 🛠️ Building the Workspace

Ensure your ROS 2 workspace is built and sourced before running:

```bash
cd ~/workspace/ros2_kortex_ws
colcon build --packages-select kinova_interface
source install/setup.bash
```

---

## 🚀 Running the Automated System

You can launch the entire stack (Hardware, MoveIt, RViz, and the Automation Supervisor) with a single command:

```bash
# Launch with the default recipe (task_recipe.json)
ros2 launch kinova_interface robot.launch.py

# Launch with a custom recipe
ros2 launch kinova_interface robot.launch.py recipe:=my_custom_task.json
```

---

## 📂 Configuration Files

*   **Recipes:** Found in `recipes/task_recipe.json`. Defines the steps.
*   **Coordinates:** Found in `recipes/coordinate_dictionary.json`. Maps object names to `X, Y, Z`.
