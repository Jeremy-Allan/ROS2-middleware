# Requirements

A simulation needs only a computer. The arm and the camera are optional.

## Computer

| Item | Requirement |
|---|---|
| Operating system | Ubuntu 22.04. For another system, see [Non-native Ubuntu](non-native-ubuntu.md). |
| ROS 2 | Humble. |
| Python | 3.10 or later. Ubuntu 22.04 includes it. |
| Go | 1.26.5 or later, to build the proxy. |
| Build tools | `colcon`, `rosdep` and `git`. The package `ros-dev-tools` provides the first two. |
| Memory | 8 GB or more. A local language model needs more. |

## Language model

The proxy sends each instruction to one provider. Choose one.

| Provider | Requirement |
|---|---|
| Ollama | [Ollama](https://ollama.com) on the computer or on a reachable host. No key is needed. |
| Google Gemini, OpenAI, Anthropic | An API key and internet access. |

## Robot

| Item | Requirement |
|---|---|
| Arm | Kinova Gen3 Lite with the two-finger gripper (`gen3_lite_2f`). |
| Connection | An Ethernet cable to the arm. The arm has the address `192.168.1.10` unless it was changed. |

See [Kinova drivers](kinova-drivers.md) for the connection and [Simulation vs real arm](simulation-vs-real.md) for the launch arguments.

## Camera

Computer vision is optional and in alpha. See [Computer vision](../technical/computer-vision.md).

| Item | Requirement |
|---|---|
| Camera | Kinect v2 on a USB 3.0 port. |
| Marker | An ArUco marker of the dictionary `6X6_250` with an edge of 0.095 m, to calibrate the camera. |
| Detection | `ultralytics` and a model file. A CUDA GPU makes the detection faster. Without one, YOLO runs on the CPU. |

See [Camera and calibration](camera-and-calibration.md).

## Network ports

| Port | Service |
|---|---|
| 9090 | `rosbridge_server`, started by the middleware launch. |
| 8080 | Backend of the proxy. |
| 8081 | LLM proxy. |
| 11434 | Ollama. |
| 10000 TCP, 10001 UDP | Kinova arm. |
