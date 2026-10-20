# Command reference

A command is a sentence that the operator types in the terminal UI. The language model turns it into a recipe. Each step of a recipe is one of ten actions. This page lists the actions, their parameters and the names that a command can use. See [Recipes](recipes.md) for the file format.

## Actions

| Action | Does | Example command |
|---|---|---|
| `home` | Moves the arm to the home pose. | Go to the home position. |
| `move_arm` | Moves the gripper to the position of an object. | Move the gripper to the blue cube, pointing down. |
| `relative_move` | Moves the gripper by a named offset. | Move up. |
| `gripper` | Sets the position of the gripper. | Open the gripper. |
| `pickup` | Grasps an object and lifts it. | Pick up the blue cube. |
| `dropoff` | Puts the held object down. | Put it on the delivery tray. |
| `pour` | Carries the held object over a destination and tilts it. | Pour the water bottle over the delivery tray. |
| `push` | Slides an object by contact, without grasping it. | Push the water bottle forward. |
| `thrust` | Raises the held object and extends it. | Thrust the water bottle forward. |
| `throw` | Swings the held object and releases it. | Throw the water bottle forward. |

A command that moves an object onto or into another object becomes a `pickup` followed by a `dropoff`. The verbs pour, tip and empty select `pour`. The verbs push, slide, shove and nudge select `push`. The verbs throw, toss, fling and launch select `throw`. The verbs thrust, jab, strike and poke select `thrust`.

The steps run in order, and the recipe stops at the first step that fails.

## Names

A command refers to objects, movements and orientations by name. A name that is not in the configuration is rejected.

| Names | Source | Default values |
|---|---|---|
| Objects | `workspace_objects.yaml` | `water_bottle`, `tea_pot`, `blue_cube`, `delivery_tray` |
| Movements | `movements_and_orientations.yaml` | `move_upwards`, `move_downwards`, `move_left`, `move_right`, `thrust_forward`, `retreat` |
| Orientations | `movements_and_orientations.yaml` | `top_down`, `top_down_90`, `side_level` |

See [Configuration](../installation/configuration.md) to add or change names. Objects that the camera detects are added to the objects. The `/scan` command of the terminal UI takes a snapshot.

## Common parameters

| Parameter | Type | Description |
|---|---|---|
| `speed` | float, 0.0 to 1.0 | Velocity and acceleration scale of the move. Without it, or with 0.0, the scale is not set. A value outside the range is limited to it. |
| `target` | string | The name of an object. Actions that use a held object take it from the held object when `target` is missing. |
| `destination` | string | The name of an object that the action works toward. |
| `direction` | `forward`, `backward`, `left`, `right` | A direction instead of a destination. `forward` moves away from the base of the arm along the line from the base to the object or to the spot, and `backward` moves toward the base. `left` and `right` are 90° from that line. |
| `distance` | float, metres | The distance along `direction`. |

`pickup` and `dropoff` ignore `speed` and use `grasping.velocity_scale` and `grasping.acceleration_scale`.

## home

Moves the arm to `hardware.home_joint_positions`.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `speed` | float | Not set | See above. |

Success message: `Moved home`.

## move_arm

Moves `tool_frame` to the centre of an object. The object is part of the planning scene, so the gripper must be able to surround it. Open the gripper first and give an `orientation` that lets the fingers straddle the object.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target` | string | Required | The object. |
| `orientation` | string | None | The name of an orientation. Without it, the planner picks the orientation. |
| `speed` | float | Not set | See above. |

Success message: `Moved to '<target>'`.

## relative_move

Moves `tool_frame` by the offset of a named movement, from its current position. The orientation does not change.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `vector` | string | Required | The name of the movement. |
| `speed` | float | Not set | See above. |

Success message: `Moved '<vector>'`.

## gripper

Moves the gripper to a position.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `position` | float | Required | The position of the gripper joint in radians. 0.0 is open and 0.8 is closed. The middleware does not check the range. The proxy accepts 0.0 to 1.0. |

Success message: `Moved gripper to <position>`.

## pickup

Plans a chain of waypoints, then grasps the object and lifts it. If another object is held, `pickup` puts it back first. See [Pickup and dropoff](../technical/motion-and-actions/pickup-and-dropoff.md).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target` | string | Required | The object. |
| `grasp_style` | `auto`, `top`, `side` | `auto` | `top` comes down from above. `side` comes in level. `auto` chooses from the shape. A later `pour` or `thrust` needs `side`. |

Success message: `Picked up '<target>' with a <style> grasp`.

## dropoff

Places the held object with the grasp that picked it up.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target` | string | The held object | The object. It must be the held object. |
| `destination` | string | None | The object to place it on. The object lands on the centre of its top. Without it, the object returns to where it was picked up. |
| `direction` | string | None | Shifts the spot. Needs `distance`. |
| `distance` | float | None | The shift in metres. |
| `place_offset` | float | `grasping.default_place_offset` | Hover height above the release pose, in metres. At most `grasping.max_place_offset`. |

Success message: `Placed '<target>' on '<destination>'`, or `Placed '<target>' back where it was`.

## pour

Lifts the held object, moves it over a destination, tilts it, holds the tilt and returns it level. The object must be held level, so `pickup` uses a side grasp. See [Pour](../technical/motion-and-actions/pour.md).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target` | string | The held object | The object. It must be the held object. |
| `destination` | string | None | Where to carry the object. |
| `direction` | string | None | A direction instead of `destination`. |
| `distance` | float | 0.3 | The distance along `direction`, in metres. |
| `lift_height` | float | 0.14 | Height of the lift, in metres. |
| `tilt_angle` | float | 2.356 | Tilt in radians. |
| `duration` | float | 1.5 | Time to hold the tilt, in seconds. |
| `speed` | float | Not set | See above. |

Without `destination` and `direction`, the object tilts where it is. Success message: `Poured '<target>' toward '<destination>'`, `Poured '<target>' <direction>` or `Poured '<target>' in place`.

## push

Slides an object across the table with the closed gripper, without grasping or lifting it. See [Push and thrust](../technical/motion-and-actions/push.md).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target` | string | Required | The object. It must not be held. |
| `destination` | string | None | Where to push it. |
| `direction` | string | None | A direction instead of `destination`. One of the two is required. |
| `distance` | float | Furthest reachable | The distance along `direction`, in metres. |
| `close_position` | float | 0.75 | How far the gripper closes. |
| `speed` | float | Not set | See above. |

Success message: `Pushed '<target>' to '<destination>'` or `Pushed '<target>' <direction>`.

## thrust

Raises the held object, turns to face the target and extends the arm. See [Push and thrust](../technical/motion-and-actions/push.md).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target` | string | The held object | The object. It must be the held object. |
| `destination` | string | None | Where to thrust. |
| `direction` | string | None | A direction instead of `destination`. One of the two is required. |
| `distance` | float | Furthest reachable | The distance along `direction`, in metres. |
| `lift_height` | float | 0.14 | Height of the lift, in metres. |
| `speed` | float | 1.0 | See above. |

Success message: `Thrust '<target>' toward '<destination>'` or `Thrust '<target>' <direction>`.

## throw

Swings the held object and releases it during the swing. The swing has a fixed shape. `destination` or `direction` only sets the direction that the arm faces. See [Throw](../technical/motion-and-actions/throw.md).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target` | string | The held object | The object. It must be the held object. |
| `destination` | string | None | Where to throw. |
| `direction` | string | None | A direction instead of `destination`. One of the two is required. |
| `distance` | float | 0.3 | The distance along `direction`, in metres. It only sets the facing. |
| `open_position` | float | 0.0 | The position of the gripper after the release. |
| `speed` | float | Not set | Scales the rotation and the wind-up. The fling runs at 1.0. |

Success message: `Threw '<target>' toward '<destination>'` or `Threw '<target>' <direction>`. After a throw, the object is removed from the scene. `/reset_environment` restores it.
