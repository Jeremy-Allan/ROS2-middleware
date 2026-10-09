[Docs Home](README.md)

# Configuration

All runtime configuration on both sides is plain YAML, JSON or Markdown, no rebuilding required for most middleware config changes (rebuild only if you add or remove files that need to be installed, or after code changes). The proxy's config files are read directly at startup, no build step at all.

> Important: `colcon build` copies the middleware's config files into the `install/` folder rather than reading them live from `src/`. If you edit a middleware config or recipe file after building, you generally need to rebuild (`colcon build --packages-select kinova_interface`) or re-source, before the change takes effect on the next launch. If you're actively iterating, consider `colcon build --symlink-install` instead, it links the install folder back to your source files so edits show up immediately.

## Workspace objects: objects and obstacles (middleware)

**File:** `kinova_interface/data/configs/env/workspace_objects.yaml`

Everything around the arm, as poses and shapes in metres relative to the robot's `base_link` frame. It has two sections:

- `objects`: things recipes can name. `move_arm`, `pickup`, `dropoff` and the other actions resolve their targets here, and this is the list the proxy hands the LLM as "objects you're allowed to reference".
- `obstacles`: collision only. MoveIt plans around them, but recipes can't name them. An obstacle called `table` also gives the proxy the table's bounds.

```yaml
objects:
  red_cube:
    pose:
      position: {x: -0.3255, y: -0.1235, z: 0.01}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
    shape: {type: BOX, dimensions: [0.05, 0.05, 0.05]}

obstacles:
  table:
    pose:
      position: {x: 0.0, y: 0.0, z: -0.075}
      orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
    shape: {type: BOX, dimensions: [1.4, 1.4, 0.05]}
```

- `position` is the shape's centre point.
- `orientation` is required for every entry: either all of `roll`/`pitch`/`yaw` (radians) or all of `x`/`y`/`z`/`w`. If it's missing or incomplete, the environment mapping node logs a fatal error naming the entry and exits at startup, rather than quietly assuming an orientation.
- `shape.dimensions` follows the ROS 2 `SolidPrimitive` convention: `[x, y, z]` for a `BOX`, `[height, radius]` for a `CYLINDER` or `CONE`, `[radius]` for a `SPHERE`. Pickup and dropoff work from the shape, so keep it accurate, not just the position.

Names are looked up exactly as written (case-sensitive). With the vision node running, its detections are added to `objects` live; leave anything it detects out of this file, or the configured copy will sit where it was measured.

If you get `PLANNING_FAILED` or `GOAL_IN_COLLISION` on a target that should be reachable, check whether an obstacle overlaps it.

## Movements and orientations (middleware)

**File:** `kinova_interface/data/configs/env/movements_and_orientations.yaml`

Named moves, so the LLM picks a name instead of making up numbers.

```yaml
relative_movements:
  move_upwards:   {x: 0.0, y: 0.0, z: 0.1}
  move_left:      {x: 0.0, y: 0.1, z: 0.0}
  move_right:     {x: 0.0, y: -0.1, z: 0.0}

orientations:
  top_down:    {roll: 3.141593, pitch: 0.0, yaw: 1.570796}
  top_down_90: {roll: 3.141593, pitch: 0.0, yaw: 0.0}
  side_level:  {roll: 1.570796, pitch: 0.0, yaw: 1.570796}
```

`relative_movements` are offsets in metres, in `base_link`, added to the gripper's current position (read live from TF) by `relative_move` steps. Directions are the robot's own: forward is +X, left is +Y, up is +Z. This differs from `dropoff`'s `direction`, which is relative to the spot's bearing from the base.

`orientations` are `tool_frame` roll/pitch/yaw in radians, in `base_link`, used by the `orientation` parameter of `move_arm` and `pickup`:

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

- `hardware.*`: the hardware interface client, used by `home`, `move_arm`, `relative_move`, the gripper and running planned trajectories. Timeouts, RRT\* planning time, goal tolerance and the home joint positions. Read at startup. `json_parser_node` also reads the two timeouts, so its own calls to the client always wait longer than the client does.
- `grasping.*`: `pickup` and `dropoff`. Speed, planner fallbacks, goal tolerances, the Cartesian-path fallback, grasp geometry, and `dropoff`'s default and maximum `place_offset`. Read at each pickup or dropoff, so `ros2 param set /json_parser_node grasping.<name> <value>` applies from the next one.

Every arm move is planned with Pilz first (PTP for free moves, LIN for straight lines), which gives the same motion every time and collision-checks it. If Pilz can't plan, free moves fall back to OMPL RRT\*, which routes around obstacles, and straight lines fall back to MoveIt's Cartesian path. The logs say which planner was used.

## MoveIt joint limits (middleware)

**File:** `kinova_interface/data/configs/moveit/joint_limits.yaml`

`launch/robot.launch.py` starts `move_group` the same way `kinova_gen3_lite_moveit_config`'s own `move_group.launch.py` does, except that it loads this file instead of that package's `joint_limits.yaml`. The only difference is that acceleration limits are enabled (1.0 rad/s² per joint, which MoveIt already assumed for joints without a limit), because the Pilz planner refuses to plan without them.

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
      "parameters": { "target": "red_cube" },
      "description": "Optional, shown in logs"
    }
  ]
}
```

Steps execute strictly in array order, one at a time, and execution stops immediately the moment any step fails. Later steps never run.

**The ten valid `action` values.** The proxy's own schema must list an action too before the LLM can produce it, so keep the two in sync when adding one.

| `action` | Required `parameters` | Optional `parameters` | What happens |
|---|---|---|---|
| `"home"` | (none) | `speed` | Sends the arm to a fixed joint-space home pose |
| `"move_arm"` | `"target": "<object_name>"` | `orientation`, `speed` | Looks up `<object_name>` in the scene's objects, moves there |
| `"relative_move"` | `"vector": "<movement_name>"` | `speed` | Looks up `<movement_name>` in `movements_and_orientations.yaml`, moves the arm by that offset from wherever it currently is |
| `"gripper"` | `"position": <number>` | (none) | Sends the gripper to that position |
| `"pickup"` | `"target": "<object_name>"` | `open_position`, `close_position`, `grasp_style`, `orientation`, `grasp_offset` | Puts back any other held object first, then opens the gripper, moves to the object, closes the gripper, attaches the object in the planning scene. `grasp_style: "side"` computes and IK-verifies a level side grasp (needed by `pour`/`thrust`) |
| `"dropoff"` | (none) | `target`, `destination`, `direction` + `distance`, `place_offset` | Places the held object with the grasp it was picked up with: hover above the spot, straight down to just above the surface, open, then back away along the approach. The spot is the centre of `destination`'s top, or where the object was picked up if there's no `destination`. See below. |
| `"pour"` | (none) | `target` (defaults to the held object), `destination`, `direction`, `distance`, `lift_height`, `tilt_angle`, `duration`, `speed` | Lifts the held object, optionally moves above a destination/direction, tilts `joint_6` to pour, then returns level. See [pour-motion-reference](pour-motion-reference.md) |
| `"push"` | `"target": "<object_name>"` | `destination` or `direction` (one required), `distance`, `close_position`, `speed` | Slides an object by sustained contact in a single vertical plane. See [push-motion-reference](push-motion-reference.md) |
| `"thrust"` | (none) | `target` (defaults to the held object), `destination` or `direction` (one required), `distance`, `lift_height`, `speed` | Raises the held object, faces the destination and extends toward it |
| `"throw"` | (none) | `target` (defaults to the held object), `destination` or `direction` (one required), `distance`, `open_position`, `speed` | Winds up and flings the held object, releasing on a live `joint_5` trigger. See [throw-motion-reference](throw-motion-reference.md) |

> This repo's older top-level docs described the whitelist as `move_arm`, `move_gripper`, `relative_move`, `home_arm`, and missed `pickup`/`dropoff` entirely. Those old names do not appear anywhere in the actual parsing code, or in the proxy's schema. Use the ten values above.

**`orientation` (optional, `move_arm` and `pickup`):** a preset name from `movements_and_orientations.yaml`'s `orientations`, resolved via `/get_orientation_preset`, never raw angles, that's a deliberate anti-hallucination choice so the LLM never has to produce numeric roll/pitch/yaw itself. It's the absolute target orientation. If omitted, the move happens with no orientation constraint and MoveIt picks whatever orientation it wants. `relative_move` is a pure translation and ignores `orientation` (with a warning in the log).

**`speed` (optional, `home`, `move_arm`, `relative_move`):** a float from `0.0` to `1.0`, used as both MoveIt's velocity and acceleration scaling factor for that move. Omit it, or use `0.0`, for full speed.

**`dropoff`'s spot and hover:** `direction` + `distance` shift the spot relative to its bearing from the arm's base: `forward` is further from the arm, `backward` closer, `left`/`right` sideways. With a `destination`, the shifted spot must still be on top of it. `place_offset` (default `grasping.default_place_offset`, at most `grasping.max_place_offset` in `motion_settings.yaml`) is how high above the release pose the arm hovers before the straight descent; the object is always released `grasping.place_clearance` above the surface. If `target` is omitted, it defaults to the held object.

## Configuring the LLM provider (proxy)

**File:** `configs/llm_config.json` in the proxy repo.

Controls which LLM the proxy talks to. No code changes are needed to switch providers, just edit this file:

```json
{
    "provider": "ollama",
    "model": "gemma3:1b",
    "base_url": "http://localhost:11434/api/generate",
    "api_key": "",
    "max_tokens": 1024,
    "temperature": 0.1,
    "timeout_seconds": 30
}
```

Four providers are supported. Copy the block that matches what you want into `llm_config.json`:

**Ollama (local, free, default):** [ollama.com](https://ollama.com)
```json
{
    "provider": "ollama",
    "model": "gemma3:1b",
    "base_url": "http://localhost:11434/api/generate",
    "api_key": "",
    "max_tokens": 1024,
    "temperature": 0.1,
    "timeout_seconds": 30
}
```

**Google Gemini:** [ai.google.dev](https://ai.google.dev)
```json
{
    "provider": "gemini",
    "model": "gemini-1.5-flash",
    "base_url": "https://generativelanguage.googleapis.com/v1beta/models",
    "api_key": "YOUR_GEMINI_API_KEY",
    "max_tokens": 1024,
    "temperature": 0.1,
    "timeout_seconds": 30
}
```

**OpenAI:** [platform.openai.com](https://platform.openai.com)
```json
{
    "provider": "openai",
    "model": "gpt-4o-mini",
    "base_url": "https://api.openai.com/v1/chat/completions",
    "api_key": "YOUR_OPENAI_API_KEY",
    "max_tokens": 1024,
    "temperature": 0.1,
    "timeout_seconds": 30
}
```

**Anthropic:** [platform.claude.com](https://platform.claude.com)
```json
{
    "provider": "anthropic",
    "model": "claude-3-5-sonnet-20241022",
    "base_url": "https://api.anthropic.com/v1/messages",
    "api_key": "YOUR_ANTHROPIC_API_KEY",
    "max_tokens": 1024,
    "temperature": 0.1,
    "timeout_seconds": 30
}
```

> Note: that Anthropic model string is what's currently in the proxy's own README. It's an older model name; consider updating it to a current one when you actually configure this provider, the adapter itself doesn't care which valid model string you use.

`temperature` controls how deterministic the LLM's output is; keep it low (the default `0.1`) for this kind of structured output task, higher values make the model more likely to produce invalid JSON. `timeout_seconds` is how long the proxy waits for a response before giving up.

## The system prompt and output schema (proxy)

**Files:** `configs/system_prompt.md` and `configs/json_schema.json` in the proxy repo.

These two files together are what actually constrains what the LLM is allowed to say. `json_schema.json` is the strict machine-checked contract (every response is validated against it with Pydantic before anything is trusted). `system_prompt.md` is the natural-language instructions and few-shot examples that steer the LLM toward producing that shape in the first place, plus the physical-reasoning rules (open the gripper before approaching an object, close it to grasp, lift before moving to a drop-off, etc.).

You generally shouldn't need to touch `json_schema.json` unless you're adding a genuinely new action type (which also requires updating this repo's `json_parser_node.py` to match, see above). `system_prompt.md` is more likely to need tuning: if the LLM keeps inventing object names, hedge harder on the "only use objects from this exact list" instruction; if it keeps producing malformed JSON, check `temperature` in `llm_config.json` before rewriting the prompt.

A dead fallback prompt also exists at `src/backend/defaults.py` in the proxy repo, only used if `system_prompt.md` fails to load. It describes `relative_move` differently (`direction` and `distance` fields) than the real schema (`vector`), so if you ever see the LLM asking for those fields instead of `vector`, check that `configs/system_prompt.md` still exists and is readable.

Next: [Testing](testing.md)
