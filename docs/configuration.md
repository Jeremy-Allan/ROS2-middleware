[Docs Home](README.md)

# Configuration

All runtime configuration on both sides is plain JSON or Markdown, no rebuilding required for most middleware config changes (rebuild only if you add or remove files that need to be installed, or after code changes). The proxy's config files are read directly at startup, no build step at all.

> Important: `colcon build` copies the middleware's JSON files into the `install/` folder rather than reading them live from `src/`. If you edit a middleware config or recipe file after building, you generally need to rebuild (`colcon build --packages-select kinova_interface`) or re-source, before the change takes effect on the next launch. If you're actively iterating, consider `colcon build --symlink-install` instead, it links the install folder back to your source files so edits show up immediately.

## The object dictionary (middleware)

**File:** `kinova_interface/data/configs/env/object_dictionary.json`

This maps a plain-English object name to a full pose and shape, in meters, relative to the robot's `base_link` frame, that `move_arm`, `pickup`, and `dropoff` steps resolve against. `position` is the object's center point. `shape.dimensions` follows the ROS 2 `SolidPrimitive` convention: `[x, y, z]` for a `BOX`, `[height, radius]` for a `CYLINDER` or `CONE`, `[radius]` for a `SPHERE`.

```json
{
  "red_cube": {
    "pose": {
      "position": { "x": -0.3255, "y": -0.1235, "z": 0.01 },
      "orientation": { "roll": 0.0, "pitch": 0.0, "yaw": 0.0 }
    },
    "shape": { "type": "BOX", "dimensions": [0.05, 0.05, 0.05] }
  },
  "delivery_tray": {
    "pose": {
      "position": { "x": -0.235, "y": -0.425, "z": 0.001 },
      "orientation": { "roll": 0.0, "pitch": 0.0, "yaw": 1.57 }
    },
    "shape": { "type": "BOX", "dimensions": [0.30, 0.20, 0.03] }
  }
}
```

To add a new named object, add a new top-level key with `pose` and `shape`. Keys are looked up exactly as written (case-sensitive, no fuzzy matching), and this is also the exact list the proxy asks for and hands to the LLM as "objects you're allowed to reference," so anything you add here becomes something you can immediately ask the robot for in plain English. `dropoff`'s stacking-height calculation (see below) depends on `shape` being accurate, not just `position`.

Right now this file is maintained entirely by hand. Every object pose and shape the arm can reach for has to be measured and typed in here yourself. Automatically populating this file (or feeding coordinates in some equivalent way) using computer vision, so the system can detect where an object actually is instead of relying on a pre-typed coordinate, is the direction planned for later this semester, but it is not implemented anywhere in either repository yet.

## Orientation presets (middleware)

**File:** `kinova_interface/data/configs/env/orientation_presets.json`

Named end-effector orientations, roll/pitch/yaw in radians, that `move_arm` and `relative_move` steps can reference by name via the optional `orientation` parameter instead of an LLM ever having to produce raw angles:

```json
{
  "facing_forward": { "roll": 0.0, "pitch": 0.0, "yaw": 0.0 },
  "tilted_for_pour": { "roll": 0.0, "pitch": 1.57, "yaw": 0.0 },
  "facing_audience": { "roll": 0.0, "pitch": 0.0, "yaw": 3.14 }
}
```

For `move_arm`, a resolved preset becomes the absolute target orientation. For `relative_move`, the resolved roll/pitch/yaw are applied as a delta on top of the arm's current orientation, not absolute angles, so use small values there.

The values shipped in this file are placeholders and have not been calibrated against the physical arm; whoever tunes this on real hardware should replace them with real measured values before relying on them.

## Relative movements (middleware)

**File:** `kinova_interface/data/configs/env/relative_movement.json`

Named vectors used by `relative_move` steps, added to the arm's current position (read live via TF, not the coordinate dictionary) at execution time:

```json
{
  "move_upwards": { "x": 0.0, "y": 0.0, "z": 0.1 },
  "thrust_forward": { "x": 0.1, "y": 0.0, "z": 0.0 },
  "retreat": { "x": -0.1, "y": 0.0, "z": 0.0 }
}
```

Add new named vectors the same way: new top-level key, `x`/`y`/`z` in meters. These are offsets, not absolute positions.

## Obstacles (middleware)

**File:** `kinova_interface/data/configs/env/obstacles.json`

Defines static collision boxes that get published into MoveIt's planning scene at startup, so MoveIt refuses to plan a path through them. Same `pose`/`shape` structure as the object dictionary above, keyed by id:

```json
{
  "table": {
    "pose": {
      "position": {"x": 0.0, "y": 0.0, "z": -0.05},
      "orientation": { "roll": 0.0, "pitch": 0.0, "yaw": 0.0 }
    },
    "shape": { "type": "BOX", "dimensions": [1.2, 0.8, 0.05] }
  },
  "small_wall": {
    "pose": {
      "position": {"x": -0.1665, "y": -0.0855, "z": 0.052},
      "orientation": { "roll": 0.0, "pitch": 0.0, "yaw": 0.0 }
    },
    "shape": { "type": "BOX", "dimensions": [0.03, 0.3, 0.2] }
  }
}
```

- `shape.type` uses the ROS 2 `SolidPrimitive` type names (`BOX` is what this file currently uses; dimensions for a box are `[length_x, width_y, height_z]` in meters).
- `pose.position` is the box's center point, in the `base_link` frame.
- Obstacles in this file are currently always axis-aligned (identity orientation).

If you get mysterious `PLANNING_FAILED` or `GOAL_IN_COLLISION` errors on a target that should be reachable, check whether an obstacle box overlaps your target coordinate.

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

**The six valid `action` values**, enforced both by `json_parser_node.py` and by the proxy's own schema, so the two sides do agree on this part:

| `action` | Required `parameters` | Optional `parameters` | What happens |
|---|---|---|---|
| `"home"` | (none) | `speed` | Sends the arm to a fixed joint-space home pose |
| `"move_arm"` | `"target": "<object_name>"` | `orientation`, `speed` | Looks up `<object_name>` in the object dictionary, moves there |
| `"relative_move"` | `"vector": "<movement_name>"` | `orientation`, `speed` | Looks up `<movement_name>` in the relative-movements file, moves the arm by that offset from wherever it currently is |
| `"gripper"` | `"position": <number>` | (none) | Sends the gripper to that position |
| `"pickup"` | `"target": "<object_name>"` | `open_position`, `close_position` | Opens the gripper, moves to the object, closes the gripper, attaches the object in the planning scene |
| `"dropoff"` | `"destination": "<object_name>"` | `target`, `open_position`, `place_offset` | Two-stage release onto `destination`: hover above, then lower to a small clearance before opening the gripper, so the object doesn't fall from hover height. See below. |

> This repo's older top-level docs described the whitelist as `move_arm`, `move_gripper`, `relative_move`, `home_arm`, and missed `pickup`/`dropoff` entirely. Those old names do not appear anywhere in the actual parsing code, or in the proxy's schema. Use the six values above.

**`orientation` (optional, `move_arm` and `relative_move`):** a preset name from `orientation_presets.json`, resolved via `/get_orientation_preset`, never raw angles, that's a deliberate anti-hallucination choice so the LLM never has to produce numeric roll/pitch/yaw itself. For `move_arm` this is the absolute target orientation; for `relative_move` it's applied as a delta on top of the arm's current orientation. If omitted, the move happens with no orientation constraint, current behavior, MoveIt picks whatever orientation it wants.

**`speed` (optional, `home`, `move_arm`, `relative_move`):** a float from `0.0` to `1.0`, used as both MoveIt's velocity and acceleration scaling factor for that move. Omit it, or use `0.0`, to fall back to the arm's configured default (10%, per `joint_limits.yaml`).

**`dropoff`'s two-stage release:** `place_offset` (default `0.1` m) is now the hover clearance for the first move, not the release height. The second move lowers to 2 cm above the computed release height before the gripper opens. If `target` is given and resolves to a tracked object, the release height is computed as the destination's top surface plus the target object's own height (from `object_dictionary.json`'s `shape`), so `dropoff` can stack one object on another, not just place onto a flat surface. If `target` is omitted, `dropoff` still performs the two-stage descent onto `destination`, it just can't update a tracked object's pose afterward.

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
