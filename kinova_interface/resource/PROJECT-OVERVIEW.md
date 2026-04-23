# Swinburne Lab: Kinova Gen3 Lite Middleware Project Overview

## 📖 Executive Summary
This document outlines the architecture, progress, and roadmap for the ROS 2 middleware layer designed to control the **Kinova Gen3 Lite** robotic arm within the Swinburne Lab. The goal of this project is to build a robust, modular, and error-resistant interface that bridges high-level user commands (and eventually automated task files) with the low-level ROS 2 hardware controllers and MoveIt 2 path planning pipeline.

---

## 🏗️ System Architecture

Our middleware is built on standard ROS 2 (Robot Operating System) philosophies, specifically leveraging **Action Clients** and **Action Servers** to handle long-running robotic movements.

### The "Unified Controller" Node
At the core of the middleware is `hardware_interface_client.py`. This single node acts as the central dispatcher, managing two distinct Action Clients simultaneously:
1. **The Arm Client (`move_action`):** Communicates with the MoveIt 2 Action Server. It handles complex 3D Cartesian coordinates ($X, Y, Z$) using `PositionConstraints`, as well as precise Joint-level resets using `JointConstraints`. MoveIt calculates the Inverse Kinematics (IK) and avoids self-collision.
2. **The Gripper Client (`gripper_cmd`):** Communicates directly with the `gen3_lite_2f_gripper_controller`. It bypasses MoveIt entirely to provide fast, direct control over the physical fingers (open/close).

### Smart Threading & Asynchronous Callbacks
To provide a seamless User Experience (UX), the system utilizes:
* **Background Spinning:** ROS 2 node execution runs on a dedicated daemon thread, allowing continuous network communication.
* **Traffic Light Synchronization:** A `threading.Event()` flag ensures the command-line interface gracefully pauses while the physical robot is moving, preventing race conditions and console spam.
* **Real-time Feedback Loops:** Asynchronous callbacks listen to the MoveIt pipeline (e.g., `PLANNING`, `MONITORING`) and translate numerical error codes (e.g., `NO_IK_SOLUTION`) into human-readable alerts if a target is out of reach.

---

## 🚀 Development Roadmap & Progress

### Sprint 1: Manual Hardware Interface (Completed)
The focus of Sprint 1 was establishing direct, reliable control over the robot via an interactive Command Line Interface (CLI).
* ✅ Established native ROS 2 development environment (Ubuntu VM via VS Code SSH).
* ✅ Developed the `HardwareInterfaceClient` node.
* ✅ Implemented Cartesian target reaching ($X, Y, Z$).
* ✅ Implemented Joint-based safe home resetting (Radians).
* ✅ Implemented direct gripper actuation (Open/Close).
* ✅ Added real-time asynchronous feedback callbacks.
* ✅ Diagnosed and documented a workaround for the `kortex_bringup` simulation bug (the "Robotiq" misconfiguration).

### Sprint 2: Automated JSON Task Parser (Upcoming)
The focus of Sprint 2 is to move from manual CLI inputs to automated, sequence-based task execution.
* ⏳ **Develop JSON Parser:** Create a module capable of reading predefined JSON recipe files containing sequential arrays of robotic tasks (e.g., move to coordinate, close gripper, move to home).
* ⏳ **Integration with Hardware Client:** Feed the parsed JSON arrays into the existing `hardware_interface_client.py` logic, allowing the robot to execute complex, multi-step routines autonomously.
* ⏳ **Error Handling during Automation:** Ensure that if MoveIt fails step 2 of a 5-step JSON recipe, the system safely halts and reports the exact failure point rather than blindly continuing.

---

## 🔬 Simulation vs. Hardware Considerations

A critical discovery during Sprint 1 was a discrepancy in the Kinova `kortex_bringup` fake hardware simulator. When launching the `gen3_lite` with `use_fake_hardware:=true`, the launch file incorrectly attempts to spawn a `robotiq_gripper_controller` instead of the native `gen3_lite_2f_gripper_controller`.

**The Mitigation Strategy:**
To maintain a 1:1 parity with the physical Swinburne Lab hardware, we implemented a 4-terminal architecture. We allow the initial hardware launch to fail on the gripper step, and then use a manual `ros2 run controller_manager spawner` command to forcefully inject the correct Lite driver into the active nervous system before starting MoveIt. 

This ensures that the exact same `hardware_interface_client.py` script works flawlessly in both the simulation and the real world without requiring any code changes.
