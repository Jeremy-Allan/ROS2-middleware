[Docs Home](../README.md)

# Troubleshooting

## Middleware startup / build errors

| Symptom | Likely cause | Fix |
|---|---|---|
| `Package 'kinova_interface' not found` | Workspace not sourced, or build failed | Re-run `source ~/workspace/ros2_kortex_ws/install/setup.bash`; check `colcon build` output for errors above the summary line |
| `Package 'kortex_bringup' not found` / `'kinova_gen3_lite_moveit_config' not found` | You didn't clone `ros2_kortex` into `src/`, or it failed to build | Re-check [Installation](../installation/middleware-and-proxy.md); run `colcon build` (no `--packages-select`) from the workspace root |
| Unresolved import errors for `kinova_interfaces.srv`/`.msg` | `kinova_interfaces` wasn't built before `kinova_interface`, or wasn't built at all | `colcon build --packages-select kinova_interfaces` first, then `kinova_interface`, then re-source |
| `RuntimeError` on launch mentioning `robot_ip` | You set `use_fake_hardware:=false` without `robot_ip:=` | Add `robot_ip:=<your arm's IP>`, this is an intentional safety check, not a bug |

## Middleware recipe / config errors

| Symptom | Likely cause | Fix |
|---|---|---|
| `Object <name> NOT Found` | The `target` doesn't match an object in `workspace_objects.yaml` exactly (case-sensitive) | Check spelling and casing |
| `Movement <name> NOT Found` | Same issue, for `relative_move` and `movements_and_orientations.yaml` | Check spelling and casing |
| `Failed to parse JSON recipe string.` | Recipe JSON sent to `/execute_recipe` is malformed | Validate the JSON |
| `No executable steps found or recipe failed to load` | Missing top-level `steps` key, or `steps` is empty | Confirm `steps` exists at the top level |
| Edited a JSON config but nothing changed at runtime | `colcon build` copies configs into `install/`; you edited the `src/` copy after the last build | Rebuild, or use `colcon build --symlink-install` |
| `Unknown action '<name>'` | The action isn't one of the ten in [Configuration](../installation/configuration.md) | Use a listed action |

## Middleware motion / MoveIt errors

| Log message | Meaning | Fix |
|---|---|---|
| `out of reach (no IK solution)` | No valid joint configuration reaches that position | Compare the coordinate against reachable ones in `workspace_objects.yaml` |
| `planning failed (path blocked by an obstacle or self-collision)` | MoveIt couldn't find a collision-free path | Check `workspace_objects.yaml`'s obstacles for a box blocking the route |
| `goal in collision (target inside an obstacle)` | The target pose overlaps a known obstacle | Adjust the target coordinate or the obstacle definition |
| `timed out` / `<call> timed out after Ns` | Planning or execution exceeded `hardware.action_timeout` | Retry; if consistent, the target may be near the edge of reachability |

The full list of MoveIt messages is in `utils/robot.py`.

## Middleware fault state / telemetry

| Symptom | Likely cause | Fix |
|---|---|---|
| `/system/status` stuck at FAULT | A real hardware fault, or a node's heartbeat went stale for more than 1.5s | Check `ros2 topic echo /status/node_report` to see which node and why |
| Physical robot triggered a protective stop | A real hardware condition on the arm | `ros2 service call /system/reset_fault std_srvs/srv/Trigger {}` |
| `fault_controller is NOT active` warning | The fault controller didn't start on real hardware, so hardware faults aren't detected | Check the driver's logs, then configure and activate it: `ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: fault_controller}"` and `ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [fault_controller], strictness: 2}"` |

## Proxy connection and LLM errors

| Symptom | Likely cause | Fix |
|---|---|---|
| UI status shows `bridge: disconnected` | `rosbridge` isn't running | It starts with the middleware launch; check that's still up and listening on port 9090 |
| UI shows `disconnected - reconnecting...` | The proxy backend isn't running | Restart with `./run.sh` and check `data/logs/server.log` |
| `rosbridge is not connected` | Same as `bridge: disconnected` | Same as above |
| `robot failed to execute the recipe` | The middleware rejected or failed a step | Check the middleware terminal for the failing step |
| LLM not responding, or times out | Wrong `base_url`, missing/invalid API key, or Ollama not running | Check the proxy's `data/config/config.json`; for Ollama, confirm `curl http://localhost:11434` responds |
| Schema validation errors | The model produced malformed or off-schema output | Lower `temperature` or use a larger model |

## Simulation-specific quirks

- **Gripper controller mismatch on fake hardware:** a known bug in `kortex_bringup`'s fake-hardware path can cause it to spawn the wrong gripper controller. If you hit this in simulation only, manually spawn the correct one: `ros2 run controller_manager spawner gen3_lite_2f_gripper_controller`.
- **`relative_move` fails right after launch:** it looks up the arm's current pose via TF before computing the target. If the TF tree hasn't fully come up yet, this can fail. Wait a few seconds, or run `home` first.

Next: [Architecture](../technical/architecture.md)
