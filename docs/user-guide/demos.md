# Demos

The `recipes` folder of `kinova_interface` has recipes that demonstrate the actions. Run one against a running middleware, in simulation or with the arm. See [Recipes](recipes.md#run-a-recipe) for the commands.

| Recipe | What it does | Demo |
|---|---|---|
| `test_suite/recipe_pickup.json` | Picks up `blue_cube` with a top grasp. Picks up `water_bottle` with a side grasp, which puts the cube back first. Puts the bottle back. | [Watch video](assets/videos/recipe_pickup.mp4) |
| `test_suite/recipe_dropoff.json` | Picks up `water_bottle` and stands it on the centre of `delivery_tray`. | [Watch video](assets/videos/recipe_dropoff.mp4) |
| `test_suite/recipe_pour.json` | Picks up `water_bottle` with a side grasp, carries it over `delivery_tray`, tilts it, holds the tilt and returns it level. | [Watch video](assets/videos/recipe_pour.mp4) |
| `test_suite/recipe_push.json` | Slides `water_bottle` forward with the open gripper, without grasping it. | [Watch video](assets/videos/recipe_push.mp4) |
| `test_suite/recipe_thrust.json` | Picks up `water_bottle` with a side grasp, pulls it back and extends the arm forward to its reach limit. | [Watch video](assets/videos/recipe_thrust.mp4) |
| `test_suite/recipe_throw.json` | Picks up `water_bottle` and throws it forward. | [Watch video](assets/videos/recipe_throw.mp4) |
| `orientation_and_speed_demo.json` | Opens the gripper, moves to `blue_cube` pointing straight down at 30 % speed, lifts and returns home at 50 % speed. | — |

For example:

```bash
ros2 launch kinova_interface robot.launch.py recipe:=test_suite/recipe_pour.json
```

`recipe_push.json` moves `water_bottle`, and `recipe_throw.json` removes it from the scene. Restore the objects before the next recipe. See [Reset the scene](recipes.md#reset-the-scene).

The proxy runs the same actions from a sentence. Start the proxy as in the [Quickstart](quickstart.md#send-a-command-through-the-proxy) and type, for example:

```text
Pick up the water bottle and pour it over the delivery tray.
```
