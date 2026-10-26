# Testing

The middleware has unit tests, test recipes and a script that checks the gripper orientations. None of them needs the proxy. The proxy has its own unit tests and an evaluation of language models. See [Architecture](../proxy/technical/architecture.md#build-and-test) and [Evaluate a language model](../proxy/user-guide/evaluation.md) of the proxy.

!!! warning
    The test recipes and the orientation check move the arm. Run them in simulation first. On a real arm, clear the workspace before each run.

## Unit tests

Unit tests mock MoveIt, controllers and other nodes. They require ROS 2 and a built workspace, but not running middleware or a physical arm.

Build the workspace, then run the tests from the `kinova_interface` package:

```bash
cd ~/workspace/ros2_kortex_ws
colcon build --packages-select kinova_interface kinova_interfaces
source install/setup.bash
cd src/ROS2-middleware/kinova_interface
python3 -m pytest test -q
```

To run one file, give its path, for example `python3 -m pytest test/test_motion.py -v`.

Tests depend on generated `kinova_interfaces` code. Rebuild after modifying `.srv` or `.msg` files to avoid stale-interface errors such as `'GetRobotParameters_Response' object has no attribute 'held_objects'`.

The fixture `ros_context` in `conftest.py` starts `rclpy` with the parameters of `motion_settings.yaml`, as the launch file does.

| File | Covers |
|---|---|
| `test_arm_actions.py` | `ArmActions` and the ten actions in `actions/`. |
| `test_motion.py` | `actions/motion.py`: planning chains, planner fallback, straight-line moves and execution. |
| `test_utils_grasping.py` | `utils/grasping.py`: grasp and placement candidates and their ranking. |
| `test_utils_geometry.py` | `utils/geometry.py`, including the orientation presets. |
| `test_utils_ros.py` | `utils/ros.py`. |
| `test_json_parser_node.py` | `json_parser_node`: recipe loading, step dispatch and status. |
| `test_environment_mapping_node.py` | `environment_mapping_node`: objects, scene updates, attach and detach. |
| `test_hardware_interface_client.py` | `kinova_hardware_client`: services, goals, faults and status. |
| `test_telemetry_node.py` | `telemetry_node`: status aggregation and fault controller recovery. |

The `computer_vision` package has no tests.

## Test recipes

`recipes/test_suite/` has one recipe for each of the composite actions. `recipes/test_suite/qa_middleware/` has the same seven recipes and 18 more. Both folders are installed with the package.

Launch the middleware with a recipe (path relative to the installed recipes folder). It executes once, two seconds after startup:

```bash
ros2 launch kinova_interface robot.launch.py recipe:=test_suite/recipe_pickup.json
```

Or run a recipe file against a middleware that is already running:

```bash
ros2 run kinova_interface run_recipe.py <path-to-recipe.json>
```

A successful recipe logs `--- All Tasks Completed ---`. `run_recipe.py` prints `success=` and `message=`, for example `message=Recipe executed successfully.`, and exits with 0 when the recipe succeeds.

### test_suite

| Recipe | Steps |
|---|---|
| `recipe_pickup.json` | Picks up `blue_cube` and then `water_bottle` with the default grasp. The second pickup puts the cube back first. Then puts the bottle back. |
| `recipe_grasp_styles.json` | Picks up two objects with the grasp style that `auto` does not choose, and puts each back. |
| `recipe_dropoff.json` | Picks up `water_bottle` and stands it on the centre of `delivery_tray`. |
| `recipe_pour.json` | Picks up `water_bottle` with a side grasp and pours toward `delivery_tray`. |
| `recipe_push.json` | Pushes `water_bottle` forward without grasping it. |
| `recipe_throw.json` | Picks up `water_bottle` and throws it forward. |
| `recipe_thrust.json` | Picks up `water_bottle` with a side grasp and thrusts it forward. |

### qa_middleware

| Group | Recipes |
|---|---|
| Home and gripper | `recipe_home_only.json`. `recipe_gripper_open.json` sets the gripper position to 0.0 and `recipe_gripper_close.json` sets it to 1.0. |
| Moves | `recipe_single_move.json` moves to `water_bottle`. `recipe_relative_up.json`, `recipe_relative_forward.json`, `recipe_relative_retreat.json` and `recipe_relative_down_left_right.json` use the named vectors of `movements_and_orientations.yaml`. |
| Pick and place | `recipe_pickup_isolated.json` is one pickup. `recipe_pick_and_place_blue_cube.json` and `recipe_pick_and_place_water_bottle.json` pick and place one object on `delivery_tray`. `recipe_multi_object.json` places the cube and then the bottle on the tray. |
| Variations of composite actions | `recipe_pour_in_place.json` pours without `destination` or `direction`. `recipe_throw_held_object.json` throws without `target`. |
| Invalid input | See the next table. |

| Recipe | Input | `message` of `run_recipe.py` |
|---|---|---|
| `recipe_invalid_empty_steps.json` | A recipe without steps. | `No executable steps in recipe` |
| `recipe_invalid_unsupported_action.json` | The action `fly`. | `Recipe failed at step 1 (fly): Unknown action 'fly'` |
| `recipe_invalid_unknown_object.json` | A pickup of `nonexistent_object`. | `Recipe failed at step 1 (pickup): Could not resolve pickup target 'nonexistent_object'` |
| `recipe_invalid_gripper_range.json` | A gripper position of 1.5. | `Recipe executed successfully.` The middleware does not check the range of the position. The proxy schema accepts 0.0 to 1.0. |

## Orientation check

`check_orientations.py` validates orientation presets and reachability. Run it after changing `movements_and_orientations.yaml`, the gripper convention, or the robot/table position. The fingers point along `tool_frame` +Z and close along its X axis.

Requires running middleware (`move_arm` service). Tests 9 grid points in `base_link` (x: 0.25, 0.35, 0.45 m; y: -0.2, 0.0, 0.2 m; z: 0.10 m) against 7 orientations: `top_down`, `top_down_90` and `side_level` at headings -90°, -45°, 0°, 45° and 90°. Each move is checked against `tool_frame` TF. The arm returns home before each orientation and after the final test. Edit `GRID_X`, `GRID_Y` and `GRID_Z` to change the grid.

```bash
SETTINGS=$(ros2 pkg prefix kinova_interface)/share/kinova_interface/data/configs/motion_settings.yaml
ros2 run kinova_interface check_orientations.py --ik-only --ros-args --params-file $SETTINGS
ros2 run kinova_interface check_orientations.py --ros-args --params-file $SETTINGS
```

The first command only checks that an IK solution exists and does not move the arm. The second command moves the arm.

| Option | Default | Description |
|---|---|---|
| `--ik-only` | off | Check the reachability of each pose and do not move. |
| `--speed` | `0.2` | Velocity and acceleration scale of the moves, 0.0 to 1.0. |
| `--tolerance-deg` | `6.0` | Largest angle between the requested and the reached orientation that counts as reached. |

The script prints one line for each pose.

```text
       top_down at (0.25, -0.20, 0.10): ok 0.8 P
    top_down_90 at (0.45, +0.20, 0.10): ok 2.1 R
 side_level@-90 at (0.25, -0.20, 0.10): FAIL (<message>)
```

| Result | Meaning |
|---|---|
| `ok <angle> P` | The arm reached the pose. `<angle>` is the angle between the requested and the reached orientation, in degrees. Pilz PTP planned the move. |
| `ok <angle> R` | The arm reached the pose. The OMPL RRT\* fallback planned the move. |
| `BAD <angle> P` or `R` | The move succeeded, but the angle is above `--tolerance-deg` or the position is more than 0.02 m from the request. Check the preset and the gripper convention. |
| `FAIL (<message>)` | The move failed. The message is the one of the `move_arm` service. |
| `ik ok` or `no IK` | With `--ik-only`: an IK solution exists or does not exist. |

`FAIL` and `no IK` are expected for poses that the arm cannot reach, for example pointing straight down far from the base. `BAD` is not. The script exits with 0 only when every pose is `ok` or `ik ok`.

## Full pipeline

There is no automated test for the path from the proxy to the arm. To check it, follow the [Quickstart](../user-guide/quickstart.md), type a command in the terminal UI, compare the recipe it shows with the command, and check the motion of the arm.
