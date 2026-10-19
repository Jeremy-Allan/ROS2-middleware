[Docs Home](../README.md)

# Quickstart

Running the full pipeline needs two terminals: the middleware, then the proxy. If you use a local LLM, `ollama serve` runs in a third (see [Installation](../installation/middleware-and-proxy.md)).

## Terminal 1: start the middleware

By default ROS 2 writes logs to a hidden folder (`~/.ros/log`). Redirecting them somewhere visible first is recommended, not required:

```bash
export ROS_LOG_DIR=$(pwd)/middleware_logs
```

Then, source and launch:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/ros2_kortex_ws/install/setup.bash
ros2 launch kinova_interface robot.launch.py
```

This single command starts all four middleware nodes plus the Kinova driver stack plus MoveIt 2 plus RViz (the 3D visualizer), all wired together. Once the `json_parser_node` process starts, the launch also starts `rosbridge_server`, the websocket the proxy connects to, on `ws://localhost:9090` (change it with `bridge_port:=<port>`). By default it boots in simulation (`use_fake_hardware:=true`), no physical robot required.

To run against the real arm instead, once you've verified everything in simulation first:

```bash
ros2 launch kinova_interface robot.launch.py use_fake_hardware:=false robot_ip:=192.168.1.10
```

Replace `192.168.1.10` with your arm's actual IP. You must provide `robot_ip` explicitly whenever `use_fake_hardware:=false`; the launch file has a built-in safety check that refuses to launch anything if you forget it, specifically to stop you from accidentally connecting to hardware with a default or wrong IP.

To also start the computer vision stack (Kinect driver, vision node and camera calibration transform), add `enable_CV:=true` (default `false`):

```bash
ros2 launch kinova_interface robot.launch.py enable_CV:=true
```

This starts the vision node, which serves `/vision/snapshot`. Each call to it adds the detected objects to the scene. See the [computer_vision README](https://github.com/Jeremy-Allan/ROS2-middleware/blob/main/computer_vision/README.md).

Leave this terminal running.

## Terminal 2: start the proxy and give it a command

```bash
cd ~/workspace/embodied-ai-proxy
./run.sh
```

This starts the LLM proxy and backend in the background and opens the terminal UI. Type a command like `pick up the blue cube and place it on the delivery tray` and press enter. Type `/help` for the UI's commands.

The proxy fetches the known objects from the middleware, builds a prompt, sends it to the configured LLM, validates the JSON, and forwards the recipe to the middleware. The UI shows the recipe and whether it succeeded once execution finishes.

## Running the middleware alone, without the proxy

Useful for testing or debugging just the robot side, without involving an LLM at all. With the middleware already running (Terminal 1 above), you can execute a specific recipe file automatically from launch:

```bash
ros2 launch kinova_interface robot.launch.py recipe:=task_recipe.json
```

The `recipe:=` argument is a path relative to the installed `recipes/` folder, or an absolute path. Two seconds after `json_parser_node` starts, it loads that file and runs it automatically.

Or, with the stack already running and no `recipe:=` given, trigger a recipe from a second terminal without restarting anything, using the same `/execute_recipe` service the proxy itself calls:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/ros2_kortex_ws/install/setup.bash

ros2 run kinova_interface run_recipe.py ~/workspace/ros2_kortex_ws/src/ROS2-middleware/kinova_interface/recipes/test_suite/recipe_pickup.json
```

It prints `success=...` and `message=...`; on failure the message names the failing step, its action and why it failed (for example `Recipe failed at step 2 (pickup): Could not resolve pickup target 'red_cube'`).

The service returns once the whole recipe has finished executing, it blocks for the full sequence, not just the first step. For a long recipe this call can take a while to return; that's expected, not a hang.

Extra logging controls for the middleware launch, all default to `false`:

```bash
# Debug-level logs for just the middleware's four nodes
ros2 launch kinova_interface robot.launch.py debug_mode:=true

# Debug-level logs for absolutely everything, including low-level ROS 2 internals
ros2 launch kinova_interface robot.launch.py core_debug:=true

# Stop suppressing noisy third-party library logs
ros2 launch kinova_interface robot.launch.py enable_individual_logs:=true
```

## Watching the system while it runs

The proxy's sidebar shows live middleware telemetry automatically once connected. To watch it directly from a terminal instead:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/ros2_kortex_ws/install/setup.bash

# Live aggregated system health (READY / BUSY / FAULT)
ros2 topic echo /system/status

# Raw individual node heartbeats
ros2 topic echo /status/node_report

# Every node currently running
ros2 node list

# Every active service
ros2 service list
```

## Shutting down

`Ctrl+C` each terminal. Stopping the proxy first avoids reconnect messages in its UI.

Next: [Configuration](../installation/configuration.md)
