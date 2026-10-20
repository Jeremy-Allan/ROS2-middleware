# Kinova Gen3 Lite — ROS 2 Kortex Driver Reference

## Overview

The Kinova Kortex ROS 2 driver connects the Gen3 Lite robotic arm to ROS 2 through `ros2_control`. It implements a `hardware_interface::SystemInterface`, translating controller commands into Kinova Kortex API calls and returning actuator and gripper feedback to ROS 2.

This reference describes the **ROS 2 Humble** implementation inspected in the project's `ros2_kortex` checkout. It separates hardware interfaces, ROS 2 controllers and higher-level planning capabilities. Features marked **configured** have been identified in the supplied YAML or launch files but have not been validated against a running robot in this review.

**Target:** Kinova Gen3 Lite, six arm joints, Gen3 Lite two-finger gripper  
**ROS distribution:** Humble  
**Driver plugin:** `kortex_driver/KortexMultiInterfaceHardware`  
**Upstream:** [Kinovarobotics/ros2_kortex — humble](https://github.com/Kinovarobotics/ros2_kortex/tree/humble)

## Packages and responsibilities

| Package | Responsibility |
| --- | --- |
| `kortex_driver` | `ros2_control` hardware plugin, Kortex API sessions, command translation, state feedback and fault handling. |
| `kortex_description` | Robot description, joint and gripper definitions, and controller configuration. |
| `kortex_bringup` | Launches `ros2_control_node`, state publisher, controller spawners and optionally RViz. |
| `kinova_gen3_lite_moveit_config` | MoveIt 2 configuration and launch integration for the Gen3 Lite. |
| `picknik_twist_controller` | External ROS 2 controller plugin configured for Cartesian Twist commands. |
| `picknik_reset_fault_controller` | External ROS 2 controller plugin configured for fault management. |

The two PickNik packages were found installed under `/opt/ros/humble` in the development environment. Their exact ROS-facing runtime topics, services or actions were **not** established by the inspected source.

## Robot description and coordinate frames

The `kortex_description` package provides the Gen3 Lite URDF/Xacro, joint and link definitions, meshes, and associated configuration. The standard Gen3 Lite launch uses `gen3_lite_gen3_lite_2f.xacro`.

The `tool_frame` is an important end-effector frame for TF, calibration and the project's camera-to-robot coordinate transformations. **Do not confuse it with the `tcp` resource name used for the six Twist command interfaces.** The Kortex Twist implementation explicitly requests `CARTESIAN_REFERENCE_FRAME_TOOL`.

## Hardware command interfaces

The driver exports the following `ros2_control` **command interfaces**. These are resource names claimed by controllers, **not ROS topics or services**.

| Resource | Command interface | Implementation status |
| --- | --- | --- |
| Each arm joint (`joint_1`–`joint_6` in the supplied configuration) | `position` | **Implemented**; positions are sent using the Kortex cyclic API. |
| Each arm joint | `velocity` | **Exported, not independently implemented**; velocity values are prepared, but the Kortex velocity setter is commented out. |
| Each arm joint | `effort` | **Exported, unsupported for control**; controller-switching logic explicitly skips effort requests. |
| `tcp` | `twist.linear.x`, `.y`, `.z` | **Implemented**; sent through the Kortex Twist API. |
| `tcp` | `twist.angular.x`, `.y`, `.z` | **Implemented**; sent through the Kortex Twist API. |
| `right_finger_bottom_joint` | `position` | **Implemented** when the gripper controller is active and internal-bus communication is enabled. |
| `right_finger_bottom_joint` | `set_gripper_max_velocity` | **Exported**; used for gripper speed in low-level servoing. |
| `right_finger_bottom_joint` | `set_gripper_max_effort` | **Exported**; used for the gripper force threshold in low-level servoing. |
| `reset_fault` | `command` | **Implemented** as a fault-reset trigger when the fault controller is active. |
| `reset_fault` | `async_success` | **Exported** as a command interface carrying the reset result; it is not a separate ROS service. |

**Important:** Do not describe the arm as supporting operational velocity or torque control merely because `velocity` and `effort` command interfaces are exported. The inspected implementation transmits joint **position** targets, not independent joint velocity or effort commands.

## Hardware state interfaces

| Resource | State interface | Source of data |
| --- | --- | --- |
| Each arm joint | `position` | Actuator position, converted from degrees to radians and wrapped to `[-π, π]`. |
| Each arm joint | `velocity` | Actuator velocity, converted to radians per second. |
| Each arm joint | `effort` | Actuator torque feedback in N·m. |
| Gripper joint | `position` | Gripper motor position, scaled to the driver's joint representation when internal-bus communication is enabled. |
| Gripper joint | `velocity` | Interface exported; the inspected `readGripperPosition()` implementation does not populate a measured velocity. |
| `reset_fault` | `internal_fault` | Derived from arm state and actuator/base fault banks; additional servoing-state condition is also included. |

The fault state is an **aggregated numerical indicator**, not a complete structured diagnostic report. The source contains TODOs for separately exposing warning banks.

## Configured ROS 2 controllers

Both supplied Gen3 Lite controller YAML files declare the same controllers:

| Controller name | Plugin type | Function |
| --- | --- | --- |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | Publishes joint states. |
| `joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | Executes six-joint position trajectories. |
| `twist_controller` | `picknik_twist_controller/PicknikTwistController` | Commands Cartesian tool velocity. |
| `gen3_lite_2f_gripper_controller` | `position_controllers/GripperActionController` | Commands the two-finger gripper position. |
| `fault_controller` | `picknik_reset_fault_controller/PicknikResetFaultController` | Connects fault-management controls to the hardware interfaces. |

### Joint trajectory controller

- **Joints:** `joint_1`, `joint_2`, `joint_3`, `joint_4`, `joint_5`, `joint_6`.
- **Command interface:** `position`.
- **State interfaces:** `position`, `velocity`.
- **Partial joint goals:** disabled (`allow_partial_joints_goal: false`).
- **State publish rate:** configured to 100 Hz.
- **Action monitor rate:** configured to 20 Hz.
- **Stopped velocity tolerance:** configured to `0.0`.
- **Goal time:** configured to `0.0`.

The controller uses the standard `joint_trajectory_controller` interface. With the default unprefixed configuration, its expected ROS action name is:

```text
/joint_trajectory_controller/follow_joint_trajectory
```

The action name is **expected from the controller type**, not observed in the supplied runtime output. Check the live action graph before relying on it.

### Example joint trajectory command

With the configured six-joint controller active, a ROS topic command can be published as follows. **Use only after checking that the positions and path are safe for the physical robot**:

```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 10}}]
  }"
```

The expected command topic is `/joint_trajectory_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`). The example deliberately **does not include the gripper**: the supplied Gen3 Lite YAML configures only six joints in the arm trajectory controller. The original upstream example that included `right_finger_bottom_joint` must not be used unchanged with this configuration. Gripper motion is handled separately by `gen3_lite_2f_gripper_controller`.

### Cartesian Twist controller

The `twist_controller` claims the six command interfaces under `tcp`:

```text
tcp/twist.linear.x
tcp/twist.linear.y
tcp/twist.linear.z
tcp/twist.angular.x
tcp/twist.angular.y
tcp/twist.angular.z
```

The driver builds a Kortex `TwistCommand` and sends it using `BaseClient::SendTwistCommand`. The source explicitly sets `CARTESIAN_REFERENCE_FRAME_TOOL`, so Twist values are interpreted relative to the **tool frame**, not automatically the robot base frame.

The controller's exact ROS command topic and message type require inspection of the installed `picknik_twist_controller` plugin or a running system. They are **not established by the YAML alone**.

#### Switching between trajectory and Twist control

A controller switch is required before sending Twist commands. The following uses the standard `controller_manager` service and should be verified against the active controller manager:

```bash
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController "{
    activate_controllers: [twist_controller],
    deactivate_controllers: [joint_trajectory_controller],
    strictness: 1,
    activate_asap: true
  }"
```

The original Humble usage documentation identifies `/twist_controller/commands` as the command topic, and shows keyboard teleoperation with:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap /cmd_vel:=/twist_controller/commands
```

The installed PickNik Twist controller's **actual topic and accepted message type remain to be verified**; they cannot be established from the supplied YAML alone. Direct Twist control is intended for real hardware in the upstream usage notes; mock/simulation support is not established. It does not automatically provide collision checking or self-collision avoidance.

To return to joint trajectories:

```bash
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController "{
    activate_controllers: [joint_trajectory_controller],
    deactivate_controllers: [twist_controller],
    strictness: 1,
    activate_asap: true
  }"
```

### Gripper controller

The configured controller is `gen3_lite_2f_gripper_controller`, with joint:

```text
right_finger_bottom_joint
```

Its plugin type is `position_controllers/GripperActionController`, and `allow_stalling` is enabled. The expected standard action endpoint is:

```text
/gen3_lite_2f_gripper_controller/gripper_cmd
```

This endpoint is inferred from the configured controller type and name; it was not observed in a running ROS graph.

The hardware implementation converts between the joint position and a Kortex gripper motor percentage using a **hard-coded `0.81`-radian scale**. This value is accompanied by a source comment referring to a Robotiq 2F-85 gripper. It should **not** be treated as a verified physical Gen3 Lite gripper limit without checking the robot model and actual behaviour.

For low-level servoing, the driver sends gripper position, speed and force-threshold values as percentages. For single-level servoing, it sends a Kortex gripper-position command. Gripper commands require an active gripper controller and `use_internal_bus_gripper_comm=true`.

### Fault controller

The configuration declares:

```yaml
fault_controller:
  type: picknik_reset_fault_controller/PicknikResetFaultController
```

The driver exports `reset_fault/command`, `reset_fault/async_success` and `reset_fault/internal_fault` to support this controller. When a reset is requested and the controller is active, the driver switches to single-level servoing, invokes `ApplyEmergencyStop` twice, calls `ClearFaults`, and attempts to restore the previous servoing mode. The reset-result value is set to `1.0` on success or `0.0` on a caught failure.

**Operational caution:** A fault reset is not an ordinary motion command. The implementation's use of emergency-stop and fault-clear operations warrants supervised testing and adherence to the robot's safety procedures. Do not assume it is safe to trigger automatically after every fault.

**Launch distinction:** The standard `kortex_control.launch.py` includes a fault-controller spawner, conditionally enabled by `use_internal_bus_gripper_comm`. In the supplied Gen3 Lite MoveIt launch, `fault_controller_spawner` is commented out of the returned launch actions. Therefore, the MoveIt launch does **not appear to spawn it through that code path**, even though the controller is declared in YAML.

## Controller switching and servoing modes

The driver implements `prepare_command_mode_switch()` and `perform_command_mode_switch()` to coordinate `ros2_control` controller changes.

| Arm control mode | Kortex servoing mode | Behaviour |
| --- | --- | --- |
| Joint position control | `LOW_LEVEL_SERVOING` | Sends cyclic actuator position commands. |
| Cartesian Twist control | `SINGLE_LEVEL_SERVOING` | Sends Kortex Twist commands. |

**Joint-based and Twist controllers are mutually exclusive.** The driver rejects a request to start one while the other remains active. A valid switch must stop the current arm controller and start the replacement controller.

The gripper and fault controller have separate active-state flags. When arm control modes change, the driver resets pending position or Twist commands to current/zero values as appropriate.

The Gen3 Lite bringup starts the joint trajectory controller by default and spawns the Twist controller **inactive**. Switching controllers is therefore required before using Cartesian Twist commands.

## Feedback and update cycle

The controller manager is configured for an **update rate of 1,000 Hz**. This is a target configuration value, **not a measured communication rate**.

The driver:

1. Reads cyclic feedback for actuator positions, velocities, torques and fault information.
2. Converts actuator positions and velocities into ROS-compatible angular units.
3. Processes the active controller's commands.
4. Sends position commands through `BaseCyclicClient::Refresh()` in low-level mode, or Twist commands through `BaseClient::SendTwistCommand()` in single-level mode.
5. Refreshes feedback when no arm controller is active or when writes are blocked during a controller switch.

The source comments state that feedback synchronization is coordinated with the `write()` cycle to reduce bandwidth.

## Communication and lifecycle

The driver opens two Kortex API transports to the robot:

| Connection | Default port | Purpose |
| --- | --- | --- |
| TCP | `10000` | Standard Kortex API calls, including session and base operations. |
| UDP | `10001` | Cyclic feedback and low-level actuator communication. |

The inspected launch file also declares these default connection settings:

| Parameter | Default |
| --- | --- |
| `username` | `admin` |
| `password` | `admin` |
| `session_inactivity_timeout_ms` | `60000` |
| `connection_inactivity_timeout_ms` | `2000` |

**Security note:** These are upstream launch defaults, not recommended deployment credentials. Restrict robot-network access and replace default credentials where supported.

The hardware lifecycle includes:

- **Initialization:** validates key connection parameters, opens TCP/UDP transports, creates sessions, attempts to clear faults, configures servoing mode and determines actuator count.
- **Activation:** obtains initial feedback and seeds commands from the current actuator and gripper positions.
- **Read/write:** exchanges commands and feedback during the controller-manager update loop.
- **Deactivation:** returns to single-level servoing, closes sessions and disconnects transports.

Initialization attempts a fault clear automatically; this is distinct from a later explicit fault-reset command.

## Launch configuration

### Physical Gen3 Lite

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/ros2_kortex_ws/install/setup.bash

ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=<ROBOT_IP>
```

Replace `<ROBOT_IP>` with the address reachable from the host. Physical movement requires a correctly connected and configured robot, active controllers and an appropriate safety setup.

### MoveIt 2

The Gen3 Lite MoveIt package provides a higher-level planning and execution interface over the configured robot controllers. The original project documentation gives this physical-hardware launch example:

```bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
  robot_ip:=192.168.1.10
```

MoveIt 2 planning and collision checking are **not built into the low-level Kortex driver**, and do not make raw Twist commands collision-aware.

### Fake hardware

```bash
ros2 launch kortex_bringup gen3_lite.launch.py \
  robot_ip:=<ROBOT_IP> \
  use_fake_hardware:=true
```

The fake-hardware option mirrors commands to states for development and basic integration tests; it is **not equivalent to a physics-based robot simulation**. The shared launch file disables internal-bus gripper communication when fake hardware is enabled.

### Relevant Gen3 Lite launch arguments

| Argument | Default | Purpose |
| --- | --- | --- |
| `robot_type` | `gen3_lite` | Robot model. |
| `robot_ip` | Required | Robot network address. |
| `use_fake_hardware` | `false` | Use mock hardware rather than the physical Kortex driver. |
| `fake_sensor_commands` | `false` | Enable fake sensor command interfaces in fake-hardware mode. |
| `robot_controller` | `joint_trajectory_controller` | Initial arm controller. |
| `robot_hand_controller` | `gen3_lite_2f_gripper_controller` | Gripper controller. |
| `controllers_file` | `ros2_controllers.yaml` | Controller configuration. |
| `gripper` | `gen3_lite_2f` | Attached gripper model. |
| `gripper_joint_name` | `right_finger_bottom_joint` | Gripper command joint. |
| `use_internal_bus_gripper_comm` | `true` | Enables hardware gripper communication. |
| `gripper_max_velocity` | `100.0` | Default maximum gripper speed setting. |
| `gripper_max_force` | `100.0` | Default maximum gripper force setting. |
| `description_file` | `gen3_lite_gen3_lite_2f.xacro` | Robot description. |
| `launch_rviz` | `true` | Launch RViz. |

The shared bringup file additionally declares session credentials, TCP/UDP ports, timeouts, joint prefix and controller names.

### Simulation

The Humble repository also contains `kortex_bringup/launch/kortex_sim_control.launch.py`, with simulation-related support described in the original project document for Gazebo Classic and Ignition. This launch file and simulator dependencies were **not inspected in the source review**, so supported simulator versions, arguments and runtime behaviour should be checked before documenting a working simulation procedure. Fake hardware is not equivalent to physics simulation.

### Network addresses

The original Humble documentation lists `192.168.1.10` as a common factory-default robot address and `192.168.2.10` for a specified USB-to-Ethernet connection arrangement. These are **examples, not discovery results**; use the address configured on the actual robot/network. The driver requires `robot_ip` to be supplied.

## Runtime inspection

The source and YAML confirm the implementation and configuration, but do not prove which ROS interfaces are active in a particular session.

Once a robot or fake-hardware bringup is running, inspect the graph:

```bash
ros2 action list -t
ros2 topic list -t
ros2 service list -t
```

If the ROS 2 control CLI is installed, inspect controller resources:

```bash
ros2 control list_controllers
ros2 control list_controller_types
ros2 control list_hardware_interfaces
```

On the supplied development environment, the PickNik controller packages were installed, but `ros2 control` was unavailable. The corresponding CLI package is typically installed on Humble using:

```bash
sudo apt install ros-humble-ros2controlcli
```

The previous ROS graph inspection returned only `/parameter_events` and `/rosout`, with no robot actions or services discovered. This means **runtime functionality was not verified in that session**; it does not establish that the configured interfaces are absent.

## Integration boundaries

The Kortex driver supplies **hardware communication and control resources**. Higher-level functions belong to other components:

| Capability | Primary owner |
| --- | --- |
| Read actuator state and send position/Twist commands | Kortex driver and `ros2_control` controllers |
| Publish joint states and execute joint trajectories | ROS 2 controller plugins |
| Plan collision-aware paths and execute planned motions | MoveIt 2 and its configured planning scene/controllers |
| Convert natural-language requests into structured commands | Project LLM proxy/middleware |
| Detect objects and estimate their positions | Project computer-vision pipeline |
| Transform camera coordinates into robot coordinates | ROS TF/calibration and project integration |
| Validate task intent, workspace constraints and execution outcomes | Project middleware and application logic |

A working driver does **not** by itself provide object recognition, autonomous grasp planning, collision avoidance for raw Twist commands, or natural-language control.

## Known limitations and cautions

1. **Velocity and effort commands:** The driver exports these arm interfaces but does not implement independent velocity or effort actuation in the inspected source.
2. **Controller exclusivity:** Joint position and Cartesian Twist controllers cannot operate simultaneously on the arm.
3. **Twist reference frame:** Kortex Twist commands are configured in the tool frame; consumers must account for this when converting commands from base or camera frames.
4. **Gripper scaling:** The `0.81`-radian conversion is hard-coded and should be validated for the actual Gen3 Lite gripper.
5. **Gripper velocity feedback:** The interface is exported, but measured gripper velocity is not populated in the inspected read function.
6. **Fault feedback:** The exported fault state is aggregated; separate warning diagnostics are not implemented in the inspected source.
7. **Fault recovery:** Reset logic includes emergency-stop and fault-clear calls. It should be tested under controlled conditions, not treated as a generic automatic retry mechanism.
8. **MoveIt fault controller:** The inspected Gen3 Lite MoveIt launch comments out its fault-controller spawner.
9. **Fake hardware:** Mock feedback does not establish physical motion accuracy, safety behaviour or hardware-specific controller performance.
10. **Runtime verification:** Exact live ROS topic/action/service names, controller activation and robot performance still require an active ROS graph and, where applicable, physical testing.

## Upstream references

- [Kinova ROS 2 Kortex — Humble branch](https://github.com/Kinovarobotics/ros2_kortex/tree/humble)
- [Kortex driver](https://github.com/Kinovarobotics/ros2_kortex/tree/humble/kortex_driver)
- [Kortex description](https://github.com/Kinovarobotics/ros2_kortex/tree/humble/kortex_description)
- [Kortex MoveIt configuration](https://github.com/Kinovarobotics/ros2_kortex/tree/humble/kortex_moveit_config)

## Source files reviewed

The implementation findings above are grounded in these files from the Humble checkout:

```text
kortex_driver/README.md
kortex_driver/include/kortex_driver/hardware_interface.hpp
kortex_driver/src/hardware_interface.cpp
kortex_driver/hardware_interface_plugin.xml
kortex_bringup/launch/gen3_lite.launch.py
kortex_bringup/launch/kortex_control.launch.py
kortex_description/arms/gen3_lite/6dof/config/ros2_controllers.yaml
kortex_moveit_config/kinova_gen3_lite_moveit_config/config/ros2_controllers.yaml
```
