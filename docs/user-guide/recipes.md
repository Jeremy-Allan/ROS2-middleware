# Recipes

A recipe is a JSON document with an ordered list of steps. The proxy builds a recipe for each command. A recipe can also be written by hand and run without the proxy.

## Format

```json
{
  "recipe_name": "Dropoff - Water Bottle to Tray",
  "description": "Pick up the water bottle and stand it on the centre of the delivery tray",
  "steps": [
    { "step_id": 1, "action": "home", "description": "Start at home" },
    { "step_id": 2, "action": "pickup", "parameters": { "target": "water_bottle" }, "description": "Pick up the water bottle" },
    { "step_id": 3, "action": "dropoff", "parameters": { "destination": "delivery_tray" }, "description": "Place it on the tray" },
    { "step_id": 4, "action": "home", "description": "Return home" }
  ]
}
```

| Field | Level | Description |
|---|---|---|
| `steps` | Recipe | Required. The list of steps. A recipe without steps fails with `No executable steps in recipe`. |
| `recipe_name` | Recipe | The name that the log shows. The default is `Unnamed`. |
| `description` | Recipe | Text for the reader. The middleware does not use it. |
| `action` | Step | Required. One of the ten [actions](command-reference.md#actions). |
| `parameters` | Step | The parameters of the action. A step without parameters uses an empty object. |
| `description` | Step | The log shows `[Step <n>] <description>`, and the status text of the node shows it while the step runs. |
| `step_id` | Step | The middleware does not use it. The recipe schema of the proxy requires it. |

The proxy also writes `"status": "success"` in a recipe. The middleware ignores it.

## Execution

- The steps run in order, one at a time. The recipe stops at the first step that fails, and the later steps do not run. The result names the step: `Recipe failed at step <n> (<action>): <message>`.
- The middleware waits 0.5 s between two steps.
- `json_parser_node` runs one recipe at a time.
- The held object and the positions of the objects stay as they are between recipes. A recipe that fails after a `pickup` leaves the object in the gripper. The next `pickup` puts it back first.
- The middleware does not check a recipe against a schema. A step that lacks a required parameter stops the recipe with `Recipe aborted due to exception: '<parameter>'`. The proxy checks the recipes that it builds.

## Run a recipe

At start-up, with the argument `recipe`. The path is relative to the installed `recipes` folder, or absolute. The recipe runs once, two seconds after `json_parser_node` starts:

```bash
ros2 launch kinova_interface robot.launch.py recipe:=test_suite/recipe_dropoff.json
```

With the middleware running, with `run_recipe.py`. It reads the file, calls `/execute_recipe`, prints the result and exits with 0 when the recipe succeeds:

```bash
ros2 run kinova_interface run_recipe.py ~/workspace/ros2_kortex_ws/src/ROS2-middleware/kinova_interface/recipes/test_suite/recipe_dropoff.json
```

```text
Executing <path>/recipe_dropoff.json ...
success=True
message=Recipe executed successfully.
```

The service `/execute_recipe` has the type `kinova_interfaces/srv/ExecuteRecipe` with the field `recipe_json`. It returns after the last step has finished, which can take a minute. `run_recipe.py` reads the file itself, so a quote character in a recipe cannot break the call, as it can with `ros2 service call`.

## Write a recipe

1. Copy a recipe from `kinova_interface/recipes/` and change the steps.
2. Use names from the [configuration](../installation/configuration.md).
3. Run the recipe in simulation with `run_recipe.py` and its path.

A new file in `kinova_interface/recipes/` reaches the installed `recipes` folder at the next build. `run_recipe.py` with a path does not need that.

## Reset the scene

`/reset_environment` restores the objects and obstacles to their positions in `workspace_objects.yaml`, removes the objects that the camera added, and clears the held object:

```bash
ros2 service call /reset_environment std_srvs/srv/Trigger {}
```

The result is `success=True, message='Environment reset: 4 object(s), 2 obstacle(s) restored to configured defaults'`. A restored object can overlap the gripper when the arm is next to the object. The next move then fails. Run `home` before the reset in that case. The `/reset-env` command of the terminal UI calls the same service.
