[Docs Home](../../README.md)

# Pour

`pour` lifts a held object, optionally moves it above a destination, tilts it to pour, then returns it level. It is implemented in `actions/pour.py`.

## Requirements

- The object must already be held. Otherwise the step fails.
- It must have been grasped from the side (`grasp_style: "side"` in `pickup`), which holds it level. A recipe with a `pour` after a `pickup` forces a side grasp automatically.

## Sequence

1. **Lift** the object by `lift_height` (default 0.14 m).
2. **Move above the destination**, if a `destination` or `direction` is given. With neither, the object is poured where it was lifted. The lift and the move keep the gripper's orientation exactly.
3. **Tilt** by rotating `joint_6` alone by `tilt_angle` (default 135 degrees). The tilt is a joint-space change, so the gripper's orientation is never recomputed.
4. **Return** `joint_6` to its starting value so the object is level again.

`pour` stops there, still holding the object above the destination. A separate `dropoff` step places it down and releases it. The parameters are listed in the action contract in [Configuration](../../installation/configuration.md).

## Captured stages

The motion is based on a demonstration driven by hand in RViz on simulated hardware (MoveIt's Joints tab, *Plan and Execute*), with one joint snapshot per stage. Joint values are in degrees.

| # | Label | joint_1 | joint_2 | joint_3 | joint_4 | joint_5 | joint_6 | tool xyz | tool rpy (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 1 | Side grasp | 25.13 | 85.92 | -124.53 | -63.38 | -112.28 | -74.76 | (-0.3685, -0.1325, 0.0173) | (95.3, -2.05, -89.67) |
| 2 | Level lift | 25.13 | 57.64 | -149.54 | -63.38 | -112.28 | -74.76 | (-0.3423, -0.1202, 0.1539) | (92.34, -3.42, -89.53) |
| 3 | Transit above target | 66.45 | 56.55 | -114.2 | -64.46 | -82.23 | -95.88 | (-0.2394, -0.4517, 0.1621) | (89.41, 1.91, -48.86) |
| 4 | Tilt to pour | 66.45 | 56.55 | -114.2 | -64.46 | -82.23 | 39.54 | (-0.2394, -0.4517, 0.1621) | (-89.14, -46.48, 130.54) |
| 5 | Return & place | 24.57 | 74.5 | -104.95 | -64.46 | -82.23 | -92.63 | (-0.4774, -0.1754, 0.0436) | (97.28, 2.37, -90.63) |

The positions come from an object that has since been removed from the object dictionary. The pattern of joint changes is what the implementation follows:

- **Lift** changes only `joint_2` and `joint_3`.
- **Transit** swings `joint_1` to the destination. Roll and pitch stay near 90 and 0 degrees, so the container stays upright. Only yaw changes.
- **Tilt** changes only `joint_6`, by about 135 degrees. The position does not change.
- **Return** rotates `joint_6` back, then lowers the object toward the destination.
