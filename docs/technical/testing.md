# Testing

The middleware provides unit tests, test recipes and gripper orientation checks, all independent of the proxy. See the proxy [README](https://github.com/paul-isit/embodied-ai-proxy#evaluating-an-llm-with-test-suites) for its tests.

!!! warning
    The test recipes and the orientation check move the arm. Run them in simulation first. On a real arm, clear the workspace before each run.

## Unit tests

The unit tests replace MoveIt, the controllers and the other nodes with mocks. They need ROS 2 and a built workspace. They do not need a running middleware or an arm.

Build the workspace, then run the tests from the `kinova_interface` package:

```bash
cd ~/workspace/ros2_kortex_ws
colcon build --packages-select kinova_interface kinova_interfaces
source install/setup.bash
cd src/ROS2-middleware/kinova_interface
python3 -m pytest test -q
```

To run one file, give its path, for example `python3 -m pytest test/test_motion.py -v`.

Rebuild `kinova_interfaces` after modifying `.srv` or `.msg` files to avoid failures caused by outdated generated interfaces.

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

Start the middleware with a recipe. The path is relative to the installed `recipes` folder, and the recipe runs once, two seconds after start-up:

```bash
ros2 launch kinova_interface robot.launch.py recipe:=test_suite/recipe_pickup.json
```

Or run a recipe file against a middleware that is already running:

```bash
ros2 run kinova_interface run_recipe.py <path-to-recipe.json>
```

Successful execution logs `--- All Tasks Completed ---`. `run_recipe.py` reports `success` and `message`, exiting with code 0 on success.

### test_suite

| Recipe | Purpose |
|---|---|
| `recipe_pickup.json` | Multiple pickups and returns |
| `recipe_grasp_styles.json` | Alternative grasp styles |
| `recipe_dropoff.json` | Object placement |
| `recipe_pour.json` | Pouring |
| `recipe_push.json` | Pushing |
| `recipe_throw.json` | Throwing |
| `recipe_thrust.json` | Thrusting |

### qa_middleware

| Group | Recipes |
|---|---|
| Home and gripper | `recipe_home_only.json`, `recipe_gripper_open.json`, `recipe_gripper_close.json` |
| Moves | `recipe_single_move.json`, `recipe_relative_up.json`, `recipe_relative_forward.json`, `recipe_relative_retreat.json`, `recipe_relative_down_left_right.json` |
| Pick and place | `recipe_pickup_isolated.json`, `recipe_pick_and_place_blue_cube.json`, `recipe_pick_and_place_water_bottle.json`, `recipe_multi_object.json` |
| Composite variations | `recipe_pour_in_place.json`, `recipe_throw_held_object.json` |

| Recipe | Input | `message` of `run_recipe.py` |
|---|---|---|
| `recipe_invalid_empty_steps.json` | A recipe without steps. | `No executable steps in recipe` |
| `recipe_invalid_unsupported_action.json` | The action `fly`. | `Recipe failed at step 1 (fly): Unknown action 'fly'` |
| `recipe_invalid_unknown_object.json` | A pickup of `nonexistent_object`. | `Recipe failed at step 1 (pickup): Could not resolve pickup target 'nonexistent_object'` |
| `recipe_invalid_gripper_range.json` | A gripper position of 1.5. | `Recipe executed successfully.` The middleware does not check the range of the position. The proxy schema accepts 0.0 to 1.0. |

## Orientation check

`check_orientations.py` checks that each orientation preset points the gripper where it should, and shows which positions each orientation can reach. Use it after a change to `movements_and_orientations.yaml` or to the gripper convention, and when the table or the robot moves. The convention is that the fingers point along +Z of `tool_frame` and close along its X axis.

The middleware must be running. The script checks seven orientations across a configurable 3×3 grid in `base_link`, using IK or physical movements with TF pose verification. Modify `GRID_X`, `GRID_Y` and `GRID_Z` in the script to change the grid.

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

| Result | Meaning |
|---|---|
| `ok <angle> P/R` | Pose reached using Pilz PTP (`P`) or OMPL RRT* (`R`). Angle is the orientation error in degrees. |
| `BAD <angle> P/R` | Orientation error exceeds `--tolerance-deg` or position error exceeds 0.02 m. |
| `FAIL (<message>)` | Movement failed; displays the service error. |
| `ik ok` / `no IK` | IK solution available / unavailable. |

`FAIL` and `no IK` are expected for poses that the arm cannot reach, for example pointing straight down far from the base. `BAD` is not. The script exits with 0 only when every pose is `ok` or `ik ok`.

## Full pipeline

There is no automated test for the path from the proxy to the arm. To check it, follow the [Quickstart](../user-guide/quickstart.md), type a command in the terminal UI, compare the recipe it shows with the command, and check the motion of the arm.
