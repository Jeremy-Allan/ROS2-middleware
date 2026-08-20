[Docs Home](README.md)

# Testing

Testing is split the same way the system is: the middleware has its own recipe-based test suite, and the proxy has its own LLM-output test suite. Neither currently exercises the other, so passing both is not the same as proving the full pipeline works end to end, see the last section below.

## The middleware's test recipes

Alongside the automated unit tests, there's a set of hand-crafted recipe files under `kinova_interface/recipes/test_suite/`, run manually against a live launch while you watch the console output and RViz.

Test in simulation first, always.

| File | What it validates |
|---|---|
| `recipe_l1_single_move.json` | A single `move_arm` step, the most basic possible sanity check |
| `recipe_l2_move_and_gripper.json` | A move followed by a gripper command |
| `recipe_l3_home_move_gripper.json` | Home, gripper, move, gripper, a slightly longer chain |
| `recipe_l4_pick_and_place.json` | A full pick-and-place: home, open, move to pickup, close, lift, move to drop-off, open, home |
| `recipe_l5_multi_object.json` | Same as L4 but for two objects back-to-back |
| `recipe_l6_invalid.json` | Negative-path validation, deliberately broken inputs the system should reject |

Run L1 through L5 exactly like any other recipe (see [Running the System](running.md)):

```bash
ros2 launch kinova_interface robot.launch.py recipe:=test_suite/recipe_l1_single_move.json
```

Watch for `[Step N] <description>` lines and a final `--- All Tasks Completed ---` message.

> Before running these, check your coordinate dictionary. L1 through L5 reference `red_cube_pickup` and `blue_cube_pickup`. The shipped coordinate dictionary only defines `red_cube`, `blue_cube`, and `delivery_tray`, no `_pickup` suffix. As shipped, these will fail at step one with `Object NOT Found`. Either add the `_pickup` variants to the dictionary, or edit the recipe files' targets to match what's already there.

`recipe_l6_invalid.json` uses a different shape, a top-level `test_cases` array with nested `steps`, that the parser can't read directly (it only looks for a top-level `steps` key). Pointing `recipe:=` at it just returns "no executable steps." To actually exercise one of its four negative cases, extract that case's `steps` array into its own minimal recipe file and run that instead. The four cases are: an unknown object, an unsupported action, a gripper position outside the normal range (the middleware itself doesn't actually validate this, so it depends on the controller), and an empty steps array.

## A basic manual smoke test for the middleware alone

1. Launch with no recipe: `ros2 launch kinova_interface robot.launch.py`. RViz opens, all four nodes log ready with no errors.
2. `ros2 topic echo /system/status` shows `summary_state: 0` (READY) once everything's settled.
3. Run L1: arm visibly moves in RViz, terminal ends with `--- All Tasks Completed ---`.
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
