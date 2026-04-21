# Swinburne Lab: Kinova Gen3 Lite - ROS 2 Unified Controller

This repository contains a unified ROS 2 command-line interface (CLI) for controlling the Kinova Gen3 Lite robotic arm and its custom 2-finger gripper. It leverages MoveIt 2 for complex path planning (Inverse Kinematics) and direct action clients for precise joint and gripper control.

## Features & Capabilities (`hardware_interface_client.py`)

The `hardware_interface_client` node (`hardware_interface_client.py`) acts as the Action Client talking to multiple Action Servers. 

* **Cartesian Movement:** Input `X, Y, Z` coordinates to move the robot's end-effector in 3D space. MoveIt calculates safe trajectories to avoid self-collision.
* **Gripper Control:** Direct command of the `gen3_lite_2f_gripper_controller` to open (`o`) or close (`c`) the fingers.
* **Safe Home Reset:** Instant recovery command (`r`) using exact Joint Constraints (Radians) to return the arm to a safe, predictable default posture.
* **Smart Threading:** Uses Python `threading.Event()` to implement a "traffic light" system. The command-line menu patiently waits for the robot to finish its physical movement before prompting for the next input.
* **Error Catching:** Built-in callbacks that translate MoveIt error codes into human-readable alerts (e.g., warning the user if coordinates are physically out of reach).

---

## Building the Workspace

Before running the node, ensure your ROS 2 workspace is built and sourced. Run these commands from the root of your workspace (e.g., `~/workspace/ros2_kortex_ws`):

```bash
cd ~/workspace/ros2_kortex_ws
colcon build --packages-select kinova_interface
source install/setup.bash
```

## Running the Unified Controller

The entire system (Hardware Interface, RViz, Gripper Controller, MoveIt 2, and the CLI) can now be launched simultaneously with a single command:

```bash
ros2 launch kinova_interface robot.launch.py
```

This will boot all necessary background services and automatically open a new, dedicated **GNOME Terminal** window for the interactive command-line interface.

*(Note: If you send a command immediately upon the terminal opening, it may gracefully abort if the MoveIt 2 server is still booting up in the background. Just wait a few seconds and try again!)*