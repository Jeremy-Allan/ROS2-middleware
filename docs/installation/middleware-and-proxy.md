# Middleware and proxy

This page installs the middleware in the ROS 2 workspace of the [Kinova drivers](kinova-drivers.md), and builds the proxy.

## Prerequisites

- The Kinova packages are built in `~/workspace/ros2_kortex_ws`. See [Kinova drivers](kinova-drivers.md).
- Go 1.26.5 or later, to build the proxy. See the [Go installation guide](https://go.dev/doc/install).

## Install the middleware

1. Clone the repository into the workspace:

    ```bash
    cd ~/workspace/ros2_kortex_ws/src
    git clone https://github.com/Jeremy-Allan/ROS2-middleware.git
    ```

    The repository holds three packages:

    | Package | Contents |
    |---|---|
    | `kinova_interface` | The four nodes, the actions, the launch file, the configuration and the recipes. |
    | `kinova_interfaces` | The custom services and messages. |
    | `computer_vision` | The optional vision node. |

2. Install the dependencies. This includes `rosbridge_server`. `rosdep` prints an error for `ament_python`. `-r` lets it continue, and the error has no effect.

    ```bash
    cd ~/workspace/ros2_kortex_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the workspace. Use the same options as for the Kinova packages.

    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
    ```

    `--symlink-install` links the installed configuration and recipe files to the source files. A change to an existing file in `kinova_interface/data` or `kinova_interface/recipes` applies at the next launch. A new file needs a build. Without `--symlink-install`, build the workspace again after each change.

4. Load the workspace in every terminal that uses it:

    ```bash
    source /opt/ros/humble/setup.bash
    source ~/workspace/ros2_kortex_ws/install/setup.bash
    ```

5. Check the installation:

    ```bash
    ros2 interface show kinova_interfaces/srv/ExecuteRecipe
    ```

    The output is the definition of the service:

    ```text
    string recipe_json
    ---
    bool success
    string message
    ```

The build of `kinova_interfaces` generates the Python code that the nodes import. After a change to a `.srv` or `.msg` file, build the workspace again.

## Install the proxy

The proxy is a Go program with no dependency on ROS 2. It does not belong in the ROS 2 workspace.

```bash
cd ~/workspace
git clone https://github.com/paul-isit/embodied-ai-proxy.git
cd embodied-ai-proxy
./run.sh --build-only
```

The directories are:

```text
~/workspace
├── embodied-ai-proxy
└── ros2_kortex_ws
    └── src
        ├── ROS2-middleware
        └── ros2_kortex
```

The requirements of the proxy and the check of the build are in [Installation](../proxy/installation/installation.md) of the proxy.

## Set up a language model

The proxy sends each instruction to one language model. The default is the Ollama model `gemma3:1b`. See [Set up a language model](../proxy/installation/installation.md#set-up-a-language-model) for Ollama, and [Providers](../proxy/installation/configuration.md#providers) for Google Gemini, OpenAI and Anthropic.

## Optional: computer vision

`enable_CV:=true` needs more software. See [Camera and calibration](camera-and-calibration.md).

Next: [Quickstart](../user-guide/quickstart.md).
