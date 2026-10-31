# Quickstart

This page starts the middleware in simulation, runs a recipe and sends a command through the proxy. Install the software first. See [Setup script](../installation/setup-script.md) or [Middleware and proxy](../installation/middleware-and-proxy.md).

## Start the middleware

1. In the first terminal, load the workspace and start the launch file:

    ```bash
    source /opt/ros/humble/setup.bash
    source ~/workspace/ros2_kortex_ws/install/setup.bash
    ros2 launch kinova_interface robot.launch.py
    ```

    The launch file starts the Kinova driver with simulated hardware, MoveIt 2, RViz and the four middleware nodes. After `json_parser_node` starts, it also starts `rosbridge_websocket` on port 9090. RViz shows the arm.

    Leave the terminal open.

2. In a second terminal, load the workspace and read the system status:

    ```bash
    source /opt/ros/humble/setup.bash
    source ~/workspace/ros2_kortex_ws/install/setup.bash
    ros2 topic echo /system/status --once
    ```

    The system is ready when `summary_state` is `0`:

    ```text
    summary_state: 0
    individual_states:
    - node_name: json_parser_node
      state: 0
      status_message: JSON Parser Online & Ready
      last_command_valid: true
    - node_name: environment_mapping_node
      state: 0
      status_message: Environment Mapper Active
      last_command_valid: true
    - node_name: kinova_hardware_client
      state: 0
      status_message: Hardware Interface Client Ready
      last_command_valid: true
    ---
    ```

## Run a recipe

In the second terminal, run a recipe that picks up the water bottle and puts it on the tray:

```bash
ros2 run kinova_interface run_recipe.py ~/workspace/ros2_kortex_ws/src/ROS2-middleware/kinova_interface/recipes/test_suite/recipe_dropoff.json
```

The arm moves in RViz. The command returns after the last step:

```text
Executing <path>/recipe_dropoff.json ...
success=True
message=Recipe executed successfully.
```

A failed recipe prints `success=False` and a message that names the step. See [Recipes](recipes.md) and [Troubleshooting](troubleshooting.md).

## Send a command through the proxy

1. In a third terminal, start the proxy:

    ```bash
    cd ~/workspace/embodied-ai-proxy
    ./run.sh
    ```

    The script starts the LLM proxy and the backend in the background and opens the terminal UI. The language model must be reachable. See [Set up a language model](../installation/middleware-and-proxy.md#set-up-a-language-model).

2. Type a command in the terminal UI and press Enter:

    ```text
    Pick up the blue cube and put it on the delivery tray.
    ```

    The UI shows the recipe and the result. Type `/help` for the commands of the UI. See [Terminal UI](../proxy/user-guide/terminal-ui.md).

The proxy reads the objects from the middleware, builds a prompt, asks the language model for a recipe, checks it and sends it to the middleware. See [Architecture](../technical/architecture.md#request-flow) and [Command pipeline](../proxy/technical/command-pipeline.md).

## Use the real arm

Start the launch file with the address of the arm:

```bash
ros2 launch kinova_interface robot.launch.py use_fake_hardware:=false robot_ip:=192.168.1.10
```

See [Simulation vs real arm](../installation/simulation-vs-real.md) before the first run.

## Use the camera

Start the launch file with `enable_CV:=true`:

```bash
ros2 launch kinova_interface robot.launch.py enable_CV:=true
```

The `/scan` command of the terminal UI takes a snapshot and adds the detected objects. See [Camera and calibration](../installation/camera-and-calibration.md).

## Watch the system

| Command | Shows |
|---|---|
| `ros2 topic echo /system/status` | The state of the system and of each node: ready, busy or fault. |
| `ros2 topic echo /status/node_report` | The heartbeat of each node. |
| `ros2 node list` | The running nodes. |

The sidebar of the terminal UI shows the same status.

## Stop

Press `Ctrl+C` in each terminal. Stop the proxy first. The proxy then does not try to reconnect to a bridge that is gone.
