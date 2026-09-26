[Docs Home](README.md)

# Testing

Testing is split the same way the system is: the middleware has its own recipe-based test suite, and the proxy has its own LLM-output test suite. Neither currently exercises the other, so passing both is not the same as proving the full pipeline works end to end, see the last section below.

## The middleware's unit tests

The unit tests mock every ROS service and action, so they need neither MoveIt nor the robot. From the workspace root, after `colcon build`:

```bash
source install/setup.bash
cd src/ROS2-middleware/kinova_interface
python3 -m pytest test -q
```

They mirror the package layout: `test_arm_actions.py` covers `actions/`, one file per node, and `test_utils_ros.py` covers `utils/ros.py`.

## The middleware's test recipes

Alongside the automated unit tests, there's a set of hand-crafted recipe files under `kinova_interface/recipes/test_suite/`, run manually against a live launch while you watch the console output and RViz.

Test in simulation first, always.

| File | What it validates |
|---|---|
| `recipe_pickup.json`, `recipe_dropoff.json` | Pickup of `water_bottle`, then place it on `delivery_tray` |
| `recipe_pour.json`, `recipe_push.json`, `recipe_thrust.json`, `recipe_throw.json` | One composite action each (see the `*-motion-reference.md` docs) |

Run them like any other recipe (see [Running the System](running.md)):

```bash
ros2 launch kinova_interface robot.launch.py recipe:=test_suite/recipe_pickup.json
```

Watch for `[Step N] <description>` lines and a final `--- All Tasks Completed ---` message.

## Checking gripper orientations

**What it's for:** When a recipe asks for an orientation like `top_down`, you want to know two things: does the gripper really end up pointing that way, and from which spots on the table can it get there at all? `scripts/check_orientations.py` answers both in one run. Use it when:

- you change `orientation_presets.json`, or the gripper convention behind it (fingers point along `tool_frame` +Z, close along X). If the presets or the convention are wrong, every move shows `BAD`.
- you move to a new table or robot setup and want to see which orientations still reach which positions.
- you're choosing grasp orientations for an object and need to know what's reachable where it sits.

The unit test `test_orientation_presets_match_their_axes` only checks the maths. This script checks what the arm actually does.

**What it does:** It sends the arm to a 3 x 3 grid of points 10 cm above the table. At each point it tries every orientation: `top_down`, `top_down_90`, and `side_level` facing five headings (-90, -45, 0, 45 and 90 degrees). After each move it reads where `tool_frame` really ended up (from TF) and compares it with what was asked for. It goes home before starting each orientation. To change the grid, edit `GRID_X`, `GRID_Y` and `GRID_Z` at the top of the script.

**Running it:** Start the middleware first (no recipe needed), then:

```bash
ros2 run kinova_interface check_orientations.py --ik-only      # only asks "is this reachable?", never moves
ros2 run kinova_interface check_orientations.py --speed 0.2     # actually moves the arm
```

`--tolerance-deg` (default 6) sets how far off an orientation can be and still count as reached.

**Reading the output:** One line per move:

```
       top_down at (0.35, +0.20, 0.10): ok 0.8 P
    top_down_90 at (0.45, -0.20, 0.10): ok 2.1 R
 side_level@-90 at (0.25, -0.20, 0.10): FAIL (<planner error>)
```

| You see | It means |
|---|---|
| `ok 0.8 P` | Got there. The orientation was 0.8 degrees off what was asked for. Pilz PTP planned it. |
| `ok 2.1 R` | Got there, but Pilz couldn't plan it, so the RRT* fallback did. Common near obstacles or awkward poses. |
| `BAD 14.3 P` | The move "succeeded" but the gripper ended up 14.3 degrees off, or more than 2 cm from the point. Something is wrong with the preset or the convention. |
| `FAIL (...)` | Neither planner could get there, with the reason in brackets. Usually just out of reach for that orientation. |
| `ik ok` / `no IK` | (`--ik-only`) The pose is reachable / not reachable. Nothing moved. |

`FAIL` or `no IK` is normal for some spots: pointing straight down far from the base, for example, may simply be out of reach. That's the map you're after. `BAD` is never normal. The script exits with 0 only if every move was `ok`.

**Safety:** On fake hardware this checks planning, IK and the gripper convention, not how accurate the real arm is. On the real arm, run `--ik-only` first, clear the workspace, and keep the speed low.

## A basic manual smoke test for the middleware alone

1. Launch with no recipe: `ros2 launch kinova_interface robot.launch.py`. RViz opens, all four nodes log ready with no errors.
2. `ros2 topic echo /system/status` shows `summary_state: 0` (READY) once everything's settled.
3. Run `recipe_pickup.json`: arm visibly moves in RViz, terminal ends with `--- All Tasks Completed ---`.
4. Run a recipe with a deliberately bad object name: recipe aborts cleanly, no crash, `/system/status` doesn't get stuck in FAULT.

## Testing the proxy's LLM output

**Script:** `evaluate_proxy.py` in the proxy repo. This validates that the LLM produces correctly shaped recipes for a batch of prompts, without touching ROS 2 at all, no bridge, no middleware, nothing gets executed.

```bash
ollama serve
ollama pull gemma3:1b
```

```bash
cd ~/workspace/embodied-ai-proxy
python3 evaluate_proxy.py --config-dir ./configs --tests ./tests/basic_tests.yaml
```

Test cases are YAML, checking things like which action comes first, which comes last, which actions and target objects must appear somewhere in the recipe:

```yaml
test_cases:
  - name: Pick apple
    prompt: "pick up the apple"
    available_objects:
      - apple
      - banana
      - tray
    expected:
      first_action: "gripper"
      last_action: "relative_move"
      must_contain_actions:
        - "move_arm"
        - "gripper"
      must_contain_targets:
        - "apple"
```

Place new test files in the proxy's `tests/` directory. Note that `tests/` currently only contains `basic_tests.yaml` and `test_cases.yaml`; if you see a README example referencing `extended_tests.yaml`, that file doesn't exist yet.

## Testing the full pipeline end to end

There is currently no automated test for this. The practical way to check it: run all three terminals from [Running the System](running.md), type a real command into the proxy, and compare the recipe shown in the log panel against what you'd expect, then confirm the arm actually did it in RViz or on the physical robot. If you're specifically testing a `relative_move` other than straight up, expect it to fail right now, see the known gap in [Configuration](configuration.md).

Next: [Troubleshooting](troubleshooting.md)
