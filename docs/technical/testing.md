# Testing

The middleware provides unit tests, test recipes and gripper orientation checks, all independent of the proxy. See the proxy [README](https://github.com/paul-isit/embodied-ai-proxy#evaluating-an-llm-with-test-suites) for its tests.

!!! warning
    Test recipes and the orientation check can move the arm. Run them in simulation first. On real hardware, clear the workspace before each run.

## Unit tests

Unit tests mock MoveIt, controllers and other nodes. They require ROS 2 and a built workspace, but not running middleware or a physical arm.

```bash
cd ~/workspace/ros2_kortex_ws
colcon build --packages-select kinova_interface kinova_interfaces
source install/setup.bash
cd src/ROS2-middleware/kinova_interface
python3 -m pytest test -q
```

Run a single file with, for example, `python3 -m pytest test/test_motion.py -v`.

Tests depend on generated `kinova_interfaces` code. Rebuild after changing `.srv` or `.msg` files; stale builds can produce errors such as `'GetRobotParameters_Response' object has no attribute 'held_objects'`. The `ros_context` fixture in `conftest.py` initialises `rclpy` using `motion_settings.yaml`, as the launch file does.

| File | Coverage |
|---|---|
| `test_arm_actions.py` | `ArmActions` and the ten actions in `actions/` |
| `test_motion.py` | `actions/motion.py`: planning chains, planner fallback, straight-line moves and execution |
| `test_utils_grasping.py` | `utils/grasping.py`: grasp and placement candidates and ranking |
| `test_utils_geometry.py` | `utils/geometry.py`, including orientation presets |
| `test_utils_ros.py` | `utils/ros.py` |
| `test_json_parser_node.py` | Recipe loading, step dispatch and status |
| `test_environment_mapping_node.py` | Objects, scene updates, attachment and detachment |
| `test_hardware_interface_client.py` | Hardware services, goals, faults and status |
| `test_telemetry_node.py` | Status aggregation and fault-controller recovery |

The `computer_vision` package has no tests.

## Test recipes

`recipes/test_suite/` contains seven composite-action recipes. `recipes/test_suite/qa_middleware/` contains the same seven plus 18 additional recipes. Both directories are installed with the package.

Launch the middleware with a recipe (path relative to the installed `recipes` directory). It runs once, two seconds after startup:

```bash
ros2 launch kinova_interface robot.launch.py recipe:=test_suite/recipe_pickup.json
```

Alternatively, execute a recipe against running middleware:

```bash
ros2 run kinova_interface run_recipe.py <path-to-recipe.json>
```

Successful execution logs `--- All Tasks Completed ---`. `run_recipe.py` prints `success=` and `message=` (for example, `message=Recipe executed successfully.`) and exits with code 0 on success.

### test_suite

| Recipe | Test behaviour |
|---|---|
| `recipe_pickup.json` | Picks up `blue_cube`, then `water_bottle`; returns the cube before the second pickup and returns the bottle afterward |
| `recipe_grasp_styles.json` | Tests non-default grasp styles on two objects, returning each |
| `recipe_dropoff.json` | Places `water_bottle` upright at the centre of `delivery_tray` |
| `recipe_pour.json` | Side-grasps `water_bottle` and pours toward `delivery_tray` |
| `recipe_push.json` | Pushes `water_bottle` forward without grasping |
| `recipe_throw.json` | Picks up and throws `water_bottle` forward |
| `recipe_thrust.json` | Side-grasps and thrusts `water_bottle` forward |

### qa_middleware

| Group | Recipes and purpose |
|---|---|
| Home and gripper | `recipe_home_only.json`; `recipe_gripper_open.json` (0.0); `recipe_gripper_close.json` (1.0) |
| Moves | `recipe_single_move.json` (to `water_bottle`); `recipe_relative_up.json`, `recipe_relative_forward.json`, `recipe_relative_retreat.json`, `recipe_relative_down_left_right.json` (named vectors in `movements_and_orientations.yaml`) |
| Pick and place | `recipe_pickup_isolated.json`; `recipe_pick_and_place_blue_cube.json`, `recipe_pick_and_place_water_bottle.json` (place on `delivery_tray`); `recipe_multi_object.json` (cube, then bottle on tray) |
| Composite variations | `recipe_pour_in_place.json` (no `destination` or `direction`); `recipe_throw_held_object.json` (no `target`) |

Invalid-input tests:

| Recipe | Input | `run_recipe.py` message |
|---|---|---|
| `recipe_invalid_empty_steps.json` | No steps | `No executable steps in recipe` |
| `recipe_invalid_unsupported_action.json` | Unsupported action `fly` | `Recipe failed at step 1 (fly): Unknown action 'fly'` |
| `recipe_invalid_unknown_object.json` | Pickup of `nonexistent_object` | `Recipe failed at step 1 (pickup): Could not resolve pickup target 'nonexistent_object'` |
| `recipe_invalid_gripper_range.json` | Gripper position 1.5 | `Recipe executed successfully.` The middleware does not validate the position range; the proxy schema accepts 0.0–1.0. |

## Orientation check

`check_orientations.py` tests whether orientation presets point the gripper correctly and which positions are reachable. Run it after changing `movements_and_orientations.yaml`, the gripper convention, or the robot/table placement. The fingers point along +Z of `tool_frame` and close along its X axis.

The middleware must be running. The script tests a 3×3 grid in `base_link`: X = 0.25, 0.35, 0.45 m; Y = -0.2, 0.0, 0.2 m; Z = 0.10 m. At each point it tests seven orientations: `top_down`, `top_down_90`, and `side_level` at headings -90°, -45°, 0°, 45° and 90°. After each move, it compares the achieved `tool_frame` TF pose with the request. The arm goes home before each orientation and after the final test. Edit `GRID_X`, `GRID_Y` and `GRID_Z` to change the grid.

```bash
SETTINGS=$(ros2 pkg prefix kinova_interface)/share/kinova_interface/data/configs/motion_settings.yaml
ros2 run kinova_interface check_orientations.py --ik-only --ros-args --params-file $SETTINGS
ros2 run kinova_interface check_orientations.py --ros-args --params-file $SETTINGS
```

The first command checks IK solutions **without moving the arm**; the second performs physical or simulated movements.

| Option | Default | Description |
|---|---|---|
| `--ik-only` | off | Test IK reachability without moving |
| `--speed` | `0.2` | Velocity and acceleration scaling, 0.0–1.0 |
| `--tolerance-deg` | `6.0` | Maximum permitted orientation error, in degrees |

Example output:

```text
      top_down at (0.25, -0.20, 0.10): ok 0.8 P
   top_down_90 at (0.45, +0.20, 0.10): ok 2.1 R
side_level@-90 at (0.25, -0.20, 0.10): FAIL (<message>)
```

| Result | Meaning |
|---|---|
| `ok <angle> P` | Pose reached; `<angle>` is orientation error in degrees; planned with Pilz PTP |
| `ok <angle> R` | Pose reached; planned with OMPL RRT* fallback |
| `BAD <angle> P` or `R` | Movement succeeded, but orientation exceeds `--tolerance-deg` or position error exceeds 0.02 m; check preset and gripper convention |
| `FAIL (<message>)` | Move failed; message comes from the `move_arm` service |
| `ik ok` / `no IK` | IK solution exists / does not exist in `--ik-only` mode |

`FAIL` and `no IK` can be expected for unreachable poses, such as pointing straight down far from the base; `BAD` should be investigated. The script exits with code 0 only if every pose reports `ok` or `ik ok`.

## Full pipeline

There is no automated end-to-end test from the proxy to the arm. Follow the [Quickstart](../user-guide/quickstart.md), enter a command in the terminal UI, compare the generated recipe with the instruction, and verify the arm's motion.
