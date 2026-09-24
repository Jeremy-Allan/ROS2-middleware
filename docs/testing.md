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
