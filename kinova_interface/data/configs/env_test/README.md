# QA Test-Object Fixture (`env_test/`)

A dedicated config directory, separate from `data/configs/env/` (the production config), used for repeatable QA test runs so every session starts with the same known set of objects at the same coordinates in the RViz planning scene.

## What's in here

- `object_dictionary.json` — the real production objects (`red_cube`, `blue_cube`, `delivery_tray`, same coordinates as `env/object_dictionary.json`) **plus two new objects added for test variety**:
  - `green_cylinder` — CYLINDER, dimensions `[height, radius]` = `[0.08, 0.02]`, at `x=-0.30, y=0.05, z=0.01`
  - `yellow_sphere` — SPHERE, dimensions `[radius]` = `[0.03]`, at `x=0.30, y=-0.05, z=0.01`

  **These two are new and provisional.** Coordinates were chosen to sit within the same safe, reachable range as the existing objects (same table height, similar offsets), but they have not been measured or validated against the physical lab setup. Verify them in RViz simulation first, and re-measure before ever relying on them for physical-hardware testing.

- `relative_movement.json`, `obstacles.json` — unchanged copies of the production files, included so this directory is a complete, drop-in config set.

## How to use it

The default launch (`robot.launch.py`) currently reads config from `env/` via a `config_dir` launch parameter, now exposed as an overridable launch argument:

```bash
ros2 launch kinova_interface robot.launch.py config_dir:=/absolute/path/to/kinova_interface/data/configs/env_test
```

If you're running from an installed (`colcon build`) workspace rather than an absolute source path, make sure this directory is picked up by the build — see the `data_files` entry added in `setup.py` for `env_test`, and rebuild with `colcon build --packages-select kinova_interface` (or `--symlink-install` while iterating) before the new objects show up.

## Why this exists

`env/object_dictionary.json` is the production config — hand-maintained, and every entry represents a real, measured position in the lab. Testers needing more variety (e.g. for language/prompt coverage or a reliability soak test) previously had two bad options: edit the production dictionary directly (risking breaking real test recipes), or invent objects with no fixed home. This directory gives QA a stable, separate sandbox instead.
