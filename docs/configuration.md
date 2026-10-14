[Docs Home](README.md)

# Configuration

Middleware configuration is plain YAML and JSON.

> Important: `colcon build` copies the middleware's config files into the `install/` folder rather than reading them live from `src/`. If you edit a middleware config or recipe file after building, you generally need to rebuild (`colcon build --packages-select kinova_interface`) or re-source, before the change takes effect on the next launch. If you're actively iterating, consider `colcon build --symlink-install` instead, it links the install folder back to your source files so edits show up immediately.

## Workspace objects: objects and obstacles (middleware)

**File:** `kinova_interface/data/configs/env/workspace_objects.yaml`

Everything around the arm, as poses and shapes in metres relative to the robot's `base_link` frame. It has two sections:

- `objects`: things recipes can name. `move_arm`, `pickup`, `dropoff` and the other actions resolve their targets here, and this is the list the proxy hands the LLM as "objects you're allowed to reference".
- `obstacles`: collision only. MoveIt plans around them, but recipes can't name them. An obstacle called `table` also gives the proxy the table's bounds.

```yaml
objects:
  water_bottle:
    pose:
      position: {x: -0.3255, y: -0.1235, z: 0.065}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
    shape: {type: CYLINDER, dimensions: [0.22, 0.035]}

obstacles:
  table:
    pose:
      position: {x: 0.0, y: 0.0, z: -0.075}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
    shape: {type: BOX, dimensions: [1.4, 1.4, 0.05]}
```

- `position` is the shape's centre point.
- `orientation` is required for every entry: either all of `roll`/`pitch`/`yaw` (radians) or all of `x`/`y`/`z`/`w`. If it's missing or incomplete, the environment mapping node logs a fatal error naming the entry and exits. This also applies to objects sent to `/update_scene_objects` at runtime.
- `shape.dimensions` follows the ROS 2 `SolidPrimitive` convention: `[x, y, z]` for a `BOX`, `[height, radius]` for a `CYLINDER` or `CONE`, `[radius]` for a `SPHERE`. Pickup and dropoff work from the shape, so keep it accurate, not just the position.

Names are looked up exactly as written (case-sensitive). With the vision node running, each `/vision/snapshot` call adds its detections to `objects`, including a `table_fit` object for the fitted table plane. Leave anything it detects out of this file, or the configured copy will sit where it was measured. `/reset_environment` removes vision objects.

If you get `PLANNING_FAILED` or `GOAL_IN_COLLISION` on a target that should be reachable, check whether an obstacle overlaps it.

## Movements and orientations (middleware)

**File:** `kinova_interface/data/configs/env/movements_and_orientations.yaml`

Named moves, so the LLM picks a name instead of making up numbers.

```yaml
relative_movements:
  move_upwards:   {x: 0.0, y: 0.0, z: 0.1}
  move_downwards: {x: 0.0, y: 0.0, z: -0.1}
  move_left:      {x: 0.0, y: 0.1, z: 0.0}
  move_right:     {x: 0.0, y: -0.1, z: 0.0}
  thrust_forward: {x: 0.1, y: 0.0, z: 0.0}
  retreat:        {x: -0.1, y: 0.0, z: 0.0}

orientations:
  top_down:    {roll: 3.141593, pitch: 0.0, yaw: 1.570796}
  top_down_90: {roll: 3.141593, pitch: 0.0, yaw: 0.0}
  side_level:  {roll: 1.570796, pitch: 0.0, yaw: 1.570796}
```

`relative_movements` are offsets in metres, in `base_link`, added to the gripper's current position (read live from TF) by `relative_move` steps. Directions are the robot's own: forward is +X, left is +Y, up is +Z. This differs from `dropoff`'s `direction`, which is relative to the spot's bearing from the base.

`orientations` are `tool_frame` roll/pitch/yaw in radians, in `base_link`, used by `move_arm`'s `orientation` parameter:

| Preset | Gripper points | Fingers close along |
|---|---|---|
| `top_down` | straight down | base Y (side to side, seen from the base) |
| `top_down_90` | straight down | base X (toward/away from the base) |
| `side_level` | forward, level (base +X) | base Y (horizontal) |

The gripper convention behind these: the fingers point along `tool_frame` +Z and close along `tool_frame` X. The values are derived from that with `utils/geometry.orientation_from_axes`, and `test_orientation_presets_match_their_axes` fails if the file drifts from what the names say. When adding a preset, add its axes to that test too.

Both files are read at startup (`workspace_objects.yaml` again on `/reset_environment`), so restart after editing them.

## Motion settings: arm, pickup and dropoff (middleware)

**File:** `kinova_interface/data/configs/motion_settings.yaml`

ROS parameters that launch passes to `kinova_hardware_client` and `json_parser_node`. Every value is required; a missing one stops the node at startup. Write decimals (`30.0`, not `30`).

- `hardware.*`: the hardware interface client, used by `home`, `move_arm`, `relative_move`, `joint_move`, the gripper and running planned trajectories. Timeouts, RRT\* planning time, goal tolerance and the home joint positions. Read at startup. `json_parser_node` also reads the two timeouts, so its own calls to the client always wait longer than the client does.
- `grasping.*`: `pickup` and `dropoff`. Speed, planner fallbacks, goal tolerances, the Cartesian-path fallback, grasp geometry, and `dropoff`'s default and maximum `place_offset`. Read at each pickup or dropoff, so `ros2 param set /json_parser_node grasping.<name> <value>` applies from the next one.

Arm moves are planned with Pilz first (PTP for free moves, LIN for straight lines), which gives the same motion every time and collision-checks it. If Pilz can't plan, free moves fall back to OMPL RRT\*, and straight lines fall back to MoveIt's Cartesian path. `move_arm` without an `orientation` goes straight to RRT\*, since Pilz needs a full pose. The logs say which planner was used.

## MoveIt joint limits (middleware)

**File:** `kinova_interface/data/configs/moveit/joint_limits.yaml`

`launch/robot.launch.py` starts `move_group` the same way `kinova_gen3_lite_moveit_config`'s own `move_group.launch.py` does, except that it loads this file instead of that package's `joint_limits.yaml`. It differs in two ways. Acceleration limits are enabled (1.0 rad/s² per joint, which MoveIt already assumed), because Pilz refuses to plan without them. Kinova's `default_velocity_scaling_factor` and `default_acceleration_scaling_factor` (0.1) are not included.

## Recipes and the action contract (middleware)

**Files:** `kinova_interface/recipes/task_recipe.json` (the default working recipe) and anything you add alongside it. This is also exactly the format the LLM is instructed to produce, so understanding this contract is understanding what the LLM is actually allowed to say.

A recipe is:

```json
{
  "recipe_name": "Human-readable name",
  "description": "Optional, informational only",
  "steps": [
    {
      "step_id": 1,
      "action": "move_arm",
      "parameters": { "target": "blue_cube" },
      "description": "Optional, shown in logs"
    }
  ]
}
```

Steps execute strictly in array order, one at a time, and execution stops immediately the moment any step fails. Later steps never run.

**The ten valid `action` values.** When adding one, also add it to the proxy's `json_schema.json` and `system_prompt.md`, or the LLM can't produce it.

| `action` | Required `parameters` | Optional `parameters` | What happens |
|---|---|---|---|
| `"home"` | (none) | `speed` | Sends the arm to a fixed joint-space home pose |
| `"move_arm"` | `"target": "<object_name>"` | `orientation`, `speed` | Looks up `<object_name>` in the scene's objects, moves there |
| `"relative_move"` | `"vector": "<movement_name>"` | `speed` | Looks up `<movement_name>` in `movements_and_orientations.yaml`, moves the arm by that offset from wherever it currently is |
| `"gripper"` | `"position": <number>` | (none) | Sends the gripper to that position |
| `"pickup"` | `"target": "<object_name>"` | `grasp_style` (`auto`, `top`, `side`) | Puts back any other held object first, plans the whole approach, grasp and lift before moving, then closes the gripper and attaches the object. `auto` picks top or side from the object's shape. A later `pour` or `thrust` forces a side grasp, and a conflicting `grasp_style` fails the step |
| `"dropoff"` | (none) | `target`, `destination`, `direction` + `distance`, `place_offset` | Places the held object with the grasp it was picked up with: hover above the spot, straight down to just above the surface, open, then back away along the approach. The spot is the centre of `destination`'s top, or where the object was picked up if there's no `destination`. See below. |
| `"pour"` | (none) | `target` (defaults to the held object), `destination`, `direction`, `distance`, `lift_height`, `tilt_angle`, `duration`, `speed` | Lifts the held object, optionally moves above a destination/direction, tilts `joint_6` to pour, then returns level. See [pour-motion-reference](pour-motion-reference.md) |
| `"push"` | `"target": "<object_name>"` | `destination` or `direction` (one required), `distance`, `close_position`, `speed` | Slides an object by sustained contact in a single vertical plane. See [push-motion-reference](push-motion-reference.md) |
| `"thrust"` | (none) | `target` (defaults to the held object), `destination` or `direction` (one required), `distance`, `lift_height`, `speed` | Raises the held object, faces the destination and extends toward it |
| `"throw"` | (none) | `target` (defaults to the held object), `destination` or `direction` (one required), `distance`, `open_position`, `speed` | Winds up and flings the held object, releasing on a live `joint_5` trigger. See [throw-motion-reference](throw-motion-reference.md) |

**`orientation` (optional, `move_arm`):** a preset name from `movements_and_orientations.yaml`'s `orientations`, resolved via `/get_orientation_preset`, never raw angles, that's a deliberate anti-hallucination choice so the LLM never has to produce numeric roll/pitch/yaw itself. It's the absolute target orientation. If omitted, the move happens with no orientation constraint and MoveIt picks whatever orientation it wants. `relative_move` is a pure translation and ignores `orientation` (with a warning in the log).

**`speed` (optional):** a float from `0.0` to `1.0`, used as both MoveIt's velocity and acceleration scaling factor. Omit it, or use `0.0`, to leave it unset. `thrust` and the `throw` fling default to `1.0`. `pickup` and `dropoff` ignore it and use `grasping.*` instead.

**`dropoff`'s spot and hover:** `direction` + `distance` shift the spot relative to its bearing from the arm's base: `forward` is further from the arm, `backward` closer, `left`/`right` sideways. With a `destination`, the shifted spot must still be on top of it. `place_offset` (default `grasping.default_place_offset`, at most `grasping.max_place_offset` in `motion_settings.yaml`) is how high above the release pose the arm hovers before the straight descent; the object is always released `grasping.place_clearance` above the surface. If `target` is omitted, it defaults to the held object.

## Proxy configuration

The LLM provider, system prompt and output schema live in the proxy repo under `data/config/`. See its [README](https://github.com/paul-isit/embodied-ai-proxy#configuring-the-llm).

Next: [Testing](testing.md)
