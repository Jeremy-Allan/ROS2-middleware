[Docs Home](README.md)

# Running the System

Running the full pipeline needs three terminals running at once: the bridge, the middleware, and the proxy. Start them in that order.

## Terminal 1: start the ROS bridge

The bridge needs to see the middleware's custom message and service types, so source the middleware workspace first, then launch the bridge:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/ros2_kortex_ws/install/setup.bash

cd ~/workspace/embodied-ai-proxy/ros2_bridge_ws
source install/setup.bash
ros2 launch custom_bridge_pkg proxy_bridge.launch.py
```

The bridge is now listening on `ws://localhost:9090`. Leave this terminal running.

## Terminal 2: start the middleware

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

This single command starts all four middleware nodes plus the Kinova driver stack plus MoveIt 2 plus RViz (the 3D visualizer), all wired together. By default it boots in simulation (`use_fake_hardware:=true`), no physical robot required.

To run against the real arm instead, once you've verified everything in simulation first:

```bash
ros2 launch kinova_interface robot.launch.py use_fake_hardware:=false robot_ip:=192.168.1.10
```

Replace `192.168.1.10` with your arm's actual IP. You must provide `robot_ip` explicitly whenever `use_fake_hardware:=false`; the launch file has a built-in safety check that refuses to launch anything if you forget it, specifically to stop you from accidentally connecting to hardware with a default or wrong IP.

Leave this terminal running.

## Terminal 3: start the proxy and give it a command

```bash
cd ~/workspace/embodied-ai-proxy
python3 main.py
```

This opens a terminal interface with a status bar, a log panel, an input bar at the bottom, and a sidebar showing live middleware telemetry once it connects. Type a command like `pick up the red cube and place it on the delivery tray` and press enter.

What happens next: the proxy asks the middleware for the current list of known objects, builds a prompt around your command, sends it to whichever LLM provider is configured, validates the JSON that comes back, shows you the resulting recipe in the log panel, and forwards it to the middleware for execution. The log panel will show whether execution succeeded.

Toolbar buttons across the top let you cycle how much detail is shown (recipe only, recipe plus the full prompt, recipe plus prompt plus latency/CPU metadata), check system status, check the current LLM configuration, and copy the active system prompt to your clipboard. Up and down arrows in the input bar cycle through your command history.

## Running the middleware alone, without the proxy

Useful for testing or debugging just the robot side, without involving an LLM at all. With the middleware already running (Terminal 2 above), you can execute a specific recipe file automatically from launch:

```bash
ros2 launch kinova_interface robot.launch.py recipe:=task_recipe.json
```

The `recipe:=` argument is a filename resolved against the middleware's `recipes/` folder. Two seconds after `json_parser_node` starts, it loads that file and runs it automatically.

Or, with the stack already running and no `recipe:=` given, trigger a recipe from a second terminal without restarting anything, using the same `/execute_recipe` service the proxy itself calls:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/ros2_kortex_ws/install/setup.bash

RECIPE=$(cat ~/workspace/ros2_kortex_ws/src/ROS2-middleware/kinova_interface/recipes/task_recipe.json)
ros2 service call /execute_recipe kinova_interfaces/srv/ExecuteRecipe "{recipe_json: '$RECIPE'}"
```

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

`Ctrl+C` each terminal. Order doesn't matter much, but shutting the proxy down first avoids a few seconds of "reconnecting" noise in its log panel. If anything hangs, `Ctrl+C` a second time will force-kill it.

Next: [Configuration](configuration.md)
