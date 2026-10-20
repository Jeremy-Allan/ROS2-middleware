[Docs Home](../README.md)


# Kinova Gen3 Lite — ROS 2 Kortex Driver

This document covers the Kinova Gen3 Lite driver for **ROS 2 Humble on Ubuntu 22.04**. It outlines the driver's capabilities, ROS 2 interfaces, configuration options and basic usage.

The driver is maintained by Kinova and provides the connection between ROS 2 and the physical robot. Our middleware builds on this interface to support LLM-generated commands, computer vision and automated manipulation.

**Source:** [Kinova ROS 2 Kortex — Humble branch](https://github.com/Kinovarobotics/ros2_kortex/tree/humble)

## 1. Driver Packages

The ROS 2 Kortex repository provides the following packages:

| Package | Purpose |
|---|---|
| `kortex_driver` | Communicates with physical Gen3 and Gen3 Lite robots. |
| `kortex_description` | Provides robot models, URDF/Xacro files, meshes and joint definitions. |
| `kortex_bringup` | Launches the robot and its ROS 2 controllers. |
| `kortex_moveit_config` | Provides MoveIt 2 configurations for motion planning and execution. |

## 2. Driver Capabilities

### 2.1 Joint Trajectory Control

The driver supports joint trajectory commands through ROS 2 controllers.

**Topic:** `/joint_trajectory_controller/joint_trajectory`

**Message:** `trajectory_msgs/msg/JointTrajectory`

The Gen3 Lite has six arm joints. Its integrated gripper can also be included in the trajectory, as demonstrated in Kinova's Humble documentation.

Example:

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, right_finger_bottom_joint],
  points: [
    {
      positions: [0, 0, 0, 0, 0, 0, 1],
      time_from_start: {sec: 10}
    }
  ]
}" -1
```

Joint positions are specified in radians. In the documented example, the gripper uses `0.0` for open and `1.0` for closed.

### 2.2 Cartesian Velocity Control

The driver supports Cartesian velocity commands using Twist messages.

**Topic:** `/twist_controller/commands`

Before sending Twist commands, the trajectory controller must be deactivated and the Twist controller activated.

```bash
ros2 service call /controller_manager/switch_controller \
controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [twist_controller],
  deactivate_controllers: [joint_trajectory_controller],
  strictness: 1,
  activate_asap: true
}"
```

The arm can then be controlled using a compatible Twist publisher, such as:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap /cmd_vel:=/twist_controller/commands
```

**Limitations:**
- Twist control is documented as functional only on physical Kinova hardware.
- Mock hardware and simulation do not provide the required Twist interface.
- Collision checking, including self-collision avoidance, is the operator's responsibility when using this control mode.

To return to trajectory control:

```bash
ros2 service call /controller_manager/switch_controller \
controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [joint_trajectory_controller],
  deactivate_controllers: [twist_controller],
  strictness: 1,
  activate_asap: true
}"
```

### 2.3 Gripper Control

The Gen3 Lite includes an integrated two-finger gripper.

The Humble documentation identifies the controlled gripper joint as:

`right_finger_bottom_joint`

The gripper can be commanded as part of the documented joint trajectory interface.

Relevant configuration parameters include:

| Parameter | Description |
|---|---|
| `gripper` | Gripper model, `gen3_lite_2f`. |
| `gripper_joint_name` | Controlled joint, `right_finger_bottom_joint`. |
| `use_internal_bus_gripper_comm` | Enables communication through Kinova's internal bus. |
| `gripper_max_velocity` | Requested gripper velocity as a percentage. |
| `gripper_max_force` | Deprecated force configuration parameter. |

The availability of a separate gripper action should be verified against the installed Humble driver.

### 2.4 Robot State Feedback

The ROS 2 control framework provides access to robot state information.

Joint states can be inspected using:

```bash
ros2 topic echo /joint_states --once
```

The available feedback fields and hardware interfaces should be checked against the deployed driver.

Useful commands:

```bash
ros2 topic list -t
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

These commands help identify the interfaces exposed by the current robot configuration.

### 2.5 MoveIt 2 Integration

Kinova provides MoveIt 2 configurations for motion planning and execution.

For the physical Gen3 Lite:

```bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
  robot_ip:=192.168.1.10
```

MoveIt 2 provides a motion-planning layer above the robot controllers.

This allows applications to generate planned movements rather than manually specifying every joint trajectory.

### 2.6 Robot Description and Reference Frames

The `kortex_description` package contains:

- URDF and Xacro robot descriptions.
- Robot meshes and visual geometry.
- Joint and link definitions.
- Robot configuration files.
- Simulation-related configuration.

The Gen3 Lite description is provided through:

`gen3_lite_gen3_lite_2f.xacro`

An important frame is:

**`tool_frame`** — The tool reference frame used by the arm when reporting end-effector position feedback.

This frame is relevant to our calibration and computer-vision integration, where detected object positions must be related to the robot's coordinate system.

### 2.7 Simulation and Fake Hardware

The driver supports testing without a physical robot.

**Fake hardware:**

```bash
ros2 launch kortex_bringup gen3_lite.launch.py \
  robot_ip:=192.168.1.10 \
  use_fake_hardware:=true
```

Fake hardware mirrors commanded positions into simulated joint states. It does not reproduce physical robot behaviour.

**Simulation:**

The Humble branch also provides:

`kortex_sim_control.launch.py`

It includes configuration options for Ignition and Gazebo Classic.

Simulation requires the appropriate simulator and supporting dependencies.

## 3. Starting the Gen3 Lite

To connect to a physical Gen3 Lite:

```bash
ros2 launch kortex_bringup gen3_lite.launch.py \
  robot_ip:=192.168.1.10
```

The robot IP depends on the connection arrangement.

Kinova's Humble documentation identifies:

| Address | Connection |
|---|---|
| `192.168.1.10` | Factory-default robot address. |
| `192.168.2.10` | Address documented for the specified USB-to-Ethernet connection arrangement. |

Use the address configured for the actual robot.

## 4. Important Configuration Parameters

The Gen3 Lite launch file supports the following arguments:

| Parameter | Description | Default |
|---|---|---|
| `robot_ip` | Robot connection address. | Required |
| `robot_type` | Robot model. | `gen3_lite` |
| `gripper` | Integrated gripper model. | `gen3_lite_2f` |
| `gripper_joint_name` | Controlled gripper joint. | `right_finger_bottom_joint` |
| `use_internal_bus_gripper_comm` | Communicate with the gripper through Kinova's internal bus. | `true` |
| `use_fake_hardware` | Enable mocked hardware. | `false` |
| `fake_sensor_commands` | Enable fake sensor commands when using mock hardware. | `false` |
| `robot_controller` | Controller to start. | `joint_trajectory_controller` |
| `controllers_file` | ROS 2 controller configuration. | `ros2_controllers.yaml` |
| `description_file` | Robot description. | `gen3_lite_gen3_lite_2f.xacro` |
| `launch_rviz` | Launch RViz. | `true` |

Additional configuration is available through the robot's Xacro description, including communication ports, session timeouts, initial joint positions and simulation settings.

See the [Kortex Description package](https://github.com/Kinovarobotics/ros2_kortex/tree/humble/kortex_description).

## 5. Inspecting Available ROS 2 Interfaces

The interfaces available at runtime depend on the driver version, active controllers and launch configuration.

With the robot running, use:

```bash
# List topics and message types
ros2 topic list -t

# List services
ros2 service list -t

# List actions
ros2 action list -t

# Check active controllers
ros2 control list_controllers

# Check command and state interfaces
ros2 control list_hardware_interfaces

# Read current joint states
ros2 topic echo /joint_states --once
```

These commands are useful when troubleshooting or documenting the exact interfaces available in the deployed system.

## 6. Integration With Our Middleware

The Kinova driver handles communication with the robot and exposes the ROS 2 interfaces used for movement and feedback.

Our middleware implements additional functionality on top of these interfaces.

| Component | Responsibility |
|---|---|
| Kinova Kortex driver | Robot communication, control interfaces and hardware feedback. |
| MoveIt 2 | Motion planning and trajectory execution. |
| LLM Proxy | Converts natural-language instructions into structured commands. |
| JSON Parser | Processes structured commands for the middleware. |
| Hardware Interface | Connects middleware commands to robot control functionality. |
| Computer Vision | Detects and localises objects using external camera data. |
| Calibration | Establishes the coordinate transforms needed to relate camera observations to robot coordinates. |

The Kinova driver does not independently provide our LLM command interpretation, Kinect-based object detection or application-specific workspace mapping.

## 7. Limitations and Considerations

- **Collision safety:** Direct Twist control does not automatically perform collision checking.
- **Simulation:** Mock hardware does not represent real-world robot behaviour.
- **Controller selection:** Trajectory and Twist controllers must be switched appropriately.
- **Gripper interfaces:** Confirm the available gripper control interfaces for the installed Humble version.
- **Vision:** Kinova's optional Vision Module is separate from our external Kinect-based computer-vision system.
- **Driver versions:** Interfaces may differ between Humble, Jazzy and the main development branch.

## 8. References

- [Kinova ROS 2 Kortex — Humble](https://github.com/Kinovarobotics/ros2_kortex/tree/humble)
- [Kortex Driver](https://github.com/Kinovarobotics/ros2_kortex/tree/humble/kortex_driver)
- [Kortex Description](https://github.com/Kinovarobotics/ros2_kortex/tree/humble/kortex_description)
- [Kortex MoveIt Configuration](https://github.com/Kinovarobotics/ros2_kortex/tree/humble/kortex_moveit_config)

---

**Documentation scope:** This page covers functionality described in the supplied Kinova Humble documentation. It is not yet an exhaustive inventory of every driver service, action, state interface or fault-recovery operation. Those require verification against the `kortex_driver` implementation and the installed ROS 2 interfaces.
