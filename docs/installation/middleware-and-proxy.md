[Docs Home](../README.md)

# Install the middleware and proxy

## What you need on your machine

| Requirement | Version | Why |
|---|---|---|
| Operating System | Ubuntu 22.04 LTS | ROS 2 Humble targets this release |
| ROS 2 distribution | Humble Hawksbill | This is the ROS 2 version both repositories are written against |
| Python | 3.10+ | Ships with Ubuntu 22.04; used by ROS 2 Humble and the middleware |
| Go | 1.26.5+ | Builds the proxy |
| colcon | (installed with ROS 2 dev tools) | The build tool used to compile/install ROS 2 packages |
| RAM | 8GB minimum, 16GB+ recommended | More if you run a larger local LLM through Ollama |
| Ollama | Latest, only if running a local LLM | Not needed if you're only using a cloud provider (OpenAI, Anthropic, Gemini) |

If you're on Windows or macOS, you'll need a Ubuntu 22.04 VM or WSL2 with Ubuntu 22.04. ROS 2 Humble is not natively supported elsewhere. For other operating systems, see [Non-native Ubuntu](./non-native-ubuntu.md).

## 1. Install ROS 2 Humble

If you don't already have ROS 2 Humble installed, follow the official installation guide for your [Ubuntu Setup](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

After installing, every new terminal you use for this project needs the ROS 2 environment loaded:

```bash
source /opt/ros/humble/setup.bash
```

Verify that ROS Humble has been installed:

```bash
echo $ROS_DISTRO
# it should say `humble`
```
![ROS 2 Humble Verification](../diagrams/ros_humble_verify.png)

Tip: add this line to your `~/.bashrc` so you don't have to type it in every terminal.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## 2. Set up your ROS 2 workspace and clone this repo

A ROS 2 "workspace" is just a folder with a specific structure that `colcon` knows how to build. Create one and put all the required source packages inside `src/`:

```bash
mkdir -p ~/workspace/ros2_kortex_ws/src
cd ~/workspace/ros2_kortex_ws/src

# The official Kinova driver + MoveIt config packages
git clone -b humble https://github.com/Kinovarobotics/ros2_kortex.git

# This middleware repository
git clone https://github.com/Jeremy-Allan/ROS2-middleware.git
```
You can access the official [ROS 2 Kortex Repository](https://github.com/Kinovarobotics/ros2_kortex) here. After completing these steps, the expected project structure is as follows:

```bash
├── /workspace
    ├── /embodied-ai-proxy        (cloned in step 6)
    └── /ros2_kortex_ws
        └── /src
           ├── /ROS2-middleware
           └── /ros2_kortex
```

This repo has three packages: `kinova_interface` (the Python nodes, launch file, recipes, config), `kinova_interfaces` (the custom service/message type definitions) and `computer_vision` (the optional Kinect vision node).

> Note the naming: `kinova_interface` (singular, the nodes) and `kinova_interfaces` (plural, the custom `.srv`/`.msg` type definitions) are two separate ROS 2 packages that live side by side in this one Git repository. Don't confuse them: the launch file, entry points, and every `import` in the Python code depend on getting the singular/plural right.

## 3. Install ROS 2 dependencies with rosdep

From the root of your workspace, let ROS 2's dependency resolver install anything missing (MoveIt 2, `ros2_control`, `tf2_ros`, etc.):

```bash
cd ~/workspace/ros2_kortex_ws
sudo rosdep init      # only needed once per machine, ever
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 4. Build the workspace

```bash
cd ~/workspace/ros2_kortex_ws
# if this is the very first build of the whole workspace run just colcon build
colcon build
# after that, only rebuild the packages you changed
colcon build --packages-select kinova_interface kinova_interfaces
```

`kinova_interfaces` must build successfully before `kinova_interface`, because the Python nodes import the custom service/message types it generates (`HomeArm`, `MoveArm`, `ExtendedStatus`, etc.). Colcon handles this ordering automatically as long as both packages are in `src/`.

## 5. Source the workspace

Every terminal you use to run or interact with the middleware needs:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/ros2_kortex_ws/install/setup.bash
```

"Sourcing" loads environment variables that tell your shell where to find the compiled packages, message types, and executables. If you skip this, you'll get "package not found" or "unknown message type" errors.

## 6. Clone and set up the proxy

The proxy is a separate Go project with no ROS 2 dependency. It does not live inside `ros2_kortex_ws`:

```bash
cd ~/workspace
git clone https://github.com/paul-isit/embodied-ai-proxy.git
cd embodied-ai-proxy
```

Build it (needs Go 1.26.5+):

```bash
./run.sh --build-only
```

`rosbridge_server`, which the proxy connects to, is installed by the rosdep step above and started by the middleware launch.

## 7. Set up an LLM provider

The proxy needs a large language model to talk to. The default configuration expects a local Ollama model, which is free and needs no API key, but you can point it at OpenAI, Anthropic, or Gemini instead. Provider configuration is covered in the [proxy README](https://github.com/paul-isit/embodied-ai-proxy#configuring-the-llm). The short version for the free local path:

```bash
ollama run gemma3:1b
```

Once it's running, exit the chat session by typing `/bye`. Then host it so the proxy can reach it:

```bash
OLLAMA_HOST=0.0.0.0 ollama serve
```

Leave that terminal running. Verify it's reachable:

```bash
curl http://localhost:11434
```

It should respond `Ollama is running`.

## Optional: computer vision

`enable_CV:=true` needs a Kinect v2 and these packages, which rosdep does not install: `kinect2_bridge`, `aruco_opencv` and `easy_handeye2`. The vision node also needs `pip install ultralytics` for YOLO. Build it with `colcon build --packages-select computer_vision`.

Next: [Quickstart](../user-guide/quickstart.md)
