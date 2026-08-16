[Docs Home](README.md)

# Configuration

All runtime configuration on both sides is plain JSON or Markdown, no rebuilding required for most middleware config changes (rebuild only if you add or remove files that need to be installed, or after code changes). The proxy's config files are read directly at startup, no build step at all.

> Important: `colcon build` copies the middleware's JSON files into the `install/` folder rather than reading them live from `src/`. If you edit a middleware config or recipe file after building, you generally need to rebuild (`colcon build --packages-select kinova_interface`) or re-source, before the change takes effect on the next launch. If you're actively iterating, consider `colcon build --symlink-install` instead, it links the install folder back to your source files so edits show up immediately.

## The coordinate dictionary (middleware)

**File:** `kinova_interface/data/configs/env/coordinate_dictionary.json`

This maps a plain-English object name to an absolute X, Y, Z position (in meters, relative to the robot's `base_link` frame) that `move_arm` steps resolve against.

```json
{
  "red_cube": { "x": -0.3255, "y": -0.1235, "z": 0.01 },
  "blue_cube": { "x": 0.45, "y": 0.2, "z": 0.12 },
  "delivery_tray": { "x": -0.235, "y": -0.425, "z": 0.05 }
}
```

To add a new named location, add a new top-level key with `x`, `y`, `z` fields. Keys are looked up exactly as written (case-sensitive, no fuzzy matching), and this is also the exact list the proxy asks for and hands to the LLM as "objects you're allowed to reference," so anything you add here becomes something you can immediately ask the robot for in plain English.

Right now this file is maintained entirely by hand. Every object position the arm can reach for has to be measured and typed in here yourself. Automatically populating this file (or feeding coordinates in some equivalent way) using computer vision, so the system can detect where an object actually is instead of relying on a pre-typed coordinate, is the direction planned for later this semester, but it is not implemented anywhere in either repository yet.

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

Defines static collision boxes that get published into MoveIt's planning scene at startup, so MoveIt refuses to plan a path through them:

```json
[
  {
    "id": "table",
    "description": "Table the robot arm is mounted on",
    "shape": 1,
    "dimensions": [1.2, 0.8, 0.05],
    "position": {"x": 0.0, "y": 0.0, "z": -0.05}
  },
  {
    "id": "small_wall",
    "description": "Small wall that the arm can reach over",
    "shape": 1,
    "dimensions": [0.082, 0.276, 0.205],
    "position": {"x": -0.1665, "y": -0.0855, "z": 0.052}
  }
]
```

- `shape` uses the ROS 2 `SolidPrimitive` type codes. `1` is a `BOX` (the only shape currently used in this file). `dimensions` for a box are `[length_x, width_y, height_z]` in meters.
- `position` is the box's center point, in the `base_link` frame.
- Orientation is always identity (no rotation), obstacles are always axis-aligned boxes.

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

**The four valid `action` values**, enforced both by `json_parser_node.py` and by the proxy's own schema, so the two sides do agree on this part:

| `action` | Required `parameters` | What happens |
|---|---|---|
| `"home"` | (none) | Sends the arm to a fixed joint-space home pose |
| `"move_arm"` | `"target": "<object_name>"` | Looks up `<object_name>` in the coordinate dictionary, moves there |
| `"relative_move"` | `"vector": "<movement_name>"` | Looks up `<movement_name>` in the relative-movements file, moves the arm by that offset from wherever it currently is |
| `"gripper"` | `"position": <number>` | Sends the gripper to that position |

> This repo's older top-level docs described the whitelist as `move_arm`, `move_gripper`, `relative_move`, `home_arm`. Those names do not appear anywhere in the actual parsing code, or in the proxy's schema. The real values, confirmed independently by both repositories, are `home`, `move_arm`, `relative_move`, `gripper`. Use those.

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
