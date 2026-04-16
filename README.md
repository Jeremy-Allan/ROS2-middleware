# Swinburne Lab: Kinova Gen3 Lite - ROS 2 Unified Controller

This repository contains a unified ROS 2 command-line interface (CLI) for controlling the Kinova Gen3 Lite robotic arm and its custom 2-finger gripper. It leverages MoveIt 2 for complex path planning (Inverse Kinematics) and direct action clients for precise joint and gripper control.

## 🚀 Features & Capabilities (`move_robot.py`)

The `mover` node (`move_robot.py`) acts as the (Action Client) talking to multiple (Action Servers). 

* **Cartesian Movement:** Input `X, Y, Z` coordinates to move the robot's end-effector in 3D space. MoveIt calculates safe trajectories to avoid self-collision.
* **Gripper Control:** Direct command of the `gen3_lite_2f_gripper_controller` to open (`o`) or close (`c`) the fingers.
* **Safe Home Reset:** Instant recovery command (`r`) using exact Joint Constraints (Radians) to return the arm to a safe, predictable default posture.
* **Smart Threading:** Uses Python `threading.Event()` to implement a "traffic light" system. The command-line menu patiently waits for the robot to finish its physical movement before prompting for the next input.
* **Error Catching:** Built-in callbacks that translate MoveIt error codes into human-readable alerts (e.g., warning the user if coordinates are physically out of reach).

---

## 🛠️ Building the Workspace

Before running the node, ensure your ROS 2 workspace is built and sourced. Run these commands from the root of your workspace (e.g., `~/workspace/ros2_kortex_ws`):

**Terminal 1: The Muscles (Hardware Interface & RViz)**
Starts the primary controller_manager, loads the robot description, and boots up RViz.

```bash
cd ~/workspace/ros2_kortex_ws
source install/setup.bash
ros2 launch kortex_bringup kortex_control.launch.py \
    robot_ip:=192.168.1.10 \
    use_fake_hardware:=true \
    robot_type:=gen3_lite \
    dof:=6 \
    gripper:=gen3_lite_2f \
    controllers_file:=ros2_controllers.yaml
```

(Expect a [FATAL] error regarding the robotiq_gripper_controller. Leave this terminal running and proceed to Terminal 2).

**Terminal 2: The Gripper Override**
Manually injects the correct Gen3 Lite 2-finger gripper driver into the active controller manager.

```bash
cd ~/workspace/ros2_kortex_ws
source install/setup.bash
ros2 run controller_manager spawner gen3_lite_2f_gripper_controller
```

**Terminal 3: The Brain (MoveIt 2)**
Starts the MoveIt path planning pipeline and opens the /move_action server. It will automatically connect to both the arm and gripper controllers spawned in Terminals 1 & 2.

```bash
cd ~/workspace/ros2_kortex_ws
source install/setup.bash
ros2 launch kinova_gen3_lite_moveit_config move_group.launch.py \
    use_fake_hardware:=true
```

**Terminal 4: The Unified Controller (Your Script)**
Run the interactive CLI to send commands to the robot.

```bash
cd ~/workspace/ros2_kortex_ws
colcon build --packages-select kinova_interface
source install/setup.bash
ros2 run kinova_interface hardware_interface_client
```