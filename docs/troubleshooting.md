[Docs Home](README.md)

# Troubleshooting

## Middleware startup / build errors

| Symptom | Likely cause | Fix |
|---|---|---|
| `Package 'kinova_interface' not found` | Workspace not sourced, or build failed | Re-run `source ~/workspace/ros2_kortex_ws/install/setup.bash`; check `colcon build` output for errors above the summary line |
| `Package 'kortex_bringup' not found` / `'kinova_gen3_lite_moveit_config' not found` | You didn't clone `ros2_kortex` into `src/`, or it failed to build | Re-check [Installation](installation.md); run `colcon build` (no `--packages-select`) from the workspace root |
| Unresolved import errors for `kinova_interfaces.srv`/`.msg` | `kinova_interfaces` wasn't built before `kinova_interface`, or wasn't built at all | `colcon build --packages-select kinova_interfaces` first, then `kinova_interface`, then re-source |
| `RuntimeError` on launch mentioning `robot_ip` | You set `use_fake_hardware:=false` without `robot_ip:=` | Add `robot_ip:=<your arm's IP>`, this is an intentional safety check, not a bug |

## Middleware recipe / config errors

| Symptom | Likely cause | Fix |
|---|---|---|
| `Object <name> NOT Found` | The `target` doesn't match a key in `coordinate_dictionary.json` exactly (case-sensitive) | Check spelling/casing; see the `_pickup` mismatch in [Testing](testing.md) |
| `Movement <name> NOT Found` | Same issue, for `relative_move` and `relative_movement.json` | See the known gap in [Configuration](configuration.md) |
| `No executable steps found or recipe failed to load` | Recipe JSON is malformed, missing a top-level `steps` key, or `steps` is empty | Validate the JSON; confirm `steps` exists at the top level |
| Edited a JSON config but nothing changed at runtime | `colcon build` copies configs into `install/`; you edited the `src/` copy after the last build | Rebuild, or use `colcon build --symlink-install` |
| A step silently fails with no clear message | You used an action name that isn't in the real whitelist | See [Configuration](configuration.md), use `home` and `gripper` |

## Middleware motion / MoveIt errors

| Log message | Meaning | Fix |
|---|---|---|
| `Coordinates out of reach!` (`NO_IK_SOLUTION`) | No valid joint configuration reaches that position | Double-check the coordinate against known-reachable ones already in `coordinate_dictionary.json` |
| `Planning failed!` (`PLANNING_FAILED`) | MoveIt couldn't find a collision-free path | Check `obstacles.json` for a box blocking the route |
| `Goal is in collision!` (`GOAL_IN_COLLISION`) | The target pose overlaps a known obstacle | Adjust the target coordinate or the obstacle definition |
| `Movement timed out!` (`TIMED_OUT`) | Planning or execution exceeded the allowed time | Usually transient, retry; if consistent, the target may be near the edge of reachability |

## Middleware fault state / telemetry

| Symptom | Likely cause | Fix |
|---|---|---|
| `/system/status` stuck at FAULT | A real hardware fault, or a node's heartbeat went stale for more than 1.5s | Check `ros2 topic echo /status/node_report` to see which node and why |
| Physical robot triggered a protective stop | A real hardware condition on the arm | `ros2 service call /system/reset_fault std_srvs/srv/Trigger {}` |
| `fault_controller is NOT active` warning | On real hardware, the driver takes a few seconds to establish its connection before this controller can configure | Usually self-healing within a few seconds; if it persists beyond 15 to 20 seconds, check the driver's own logs |

## Proxy connection and LLM errors

| Symptom | Likely cause | Fix |
|---|---|---|
| Status bar shows "DISCONNECTED" | `rosbridge` isn't running, or died | Confirm Terminal 1 in [Running the System](running.md) is still up and listening on port 9090 |
| "ROS service /execute_recipe not found" | `json_parser_node` isn't running | Confirm Terminal 2 launched successfully with no errors |
| "ROS service /get_robot_parameters not found" | `environment_mapping_node` isn't running | Same as above; the proxy will still proceed with an empty object list rather than crash |
| LLM not responding, or times out | Wrong `base_url`, missing/invalid API key, or Ollama not running | Check `llm_config.json` (see [Configuration](configuration.md)); for Ollama, confirm `ollama serve` is running and `curl http://localhost:11434` responds |
| "Failed to parse LLM output as JSON" | The model produced malformed or non-JSON output | Usually a `temperature` that's too high, or a local model too small to reliably follow the schema; try a larger model or lower temperature |
| Import errors running `main.py` or `evaluate_proxy.py` | `PYTHONPATH` not set, or not running from the repo root | Confirm `PYTHONPATH=.` is set (see [Installation](installation.md)) and you're in the `embodied-ai-proxy` directory |
| Recipe looks correct in the log panel but the robot doesn't move | Middleware not fully initialized, or the recipe failed partway through | Check Terminal 2's logs; the log panel shows "Dispatched to middleware successfully" only on real success |

## Simulation-specific quirks

- **Gripper controller mismatch on fake hardware:** a known bug in `kortex_bringup`'s fake-hardware path can cause it to spawn the wrong gripper controller. If you hit this in simulation only, manually spawn the correct one: `ros2 run controller_manager spawner gen3_lite_2f_gripper_controller`.
- **`relative_move` fails right after launch:** it looks up the arm's current pose via TF before computing the target. If the TF tree hasn't fully come up yet, this can fail. Wait a few seconds, or run `home` first.

Next: [Architecture](architecture.md)
