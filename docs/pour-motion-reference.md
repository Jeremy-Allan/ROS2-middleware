# Pour Motion Reference

This records a manually-driven demonstration of the intended `pour` motion,
captured on real-time fake hardware in RViz (MoveIt's interactive marker/
Joints tab, `Plan and Execute`), one joint-space snapshot per stage. It
exists so the eventual `pour` implementation in `arm_actions.py` is built
against a concrete, physically-verified sequence rather than guessed at.

Captured with the `box` object (`kinova_interface/data/configs/env/object_dictionary.json`,
resting at `x=-0.3255, y=-0.1235, z=0.01`), poured toward `delivery_tray`
(`x=-0.235, y=-0.425, z=0.001`). Raw CSV: `~/demo_logs/pour_demo_snapshots.csv`
on the dev VM.

## Why the current `pour` fails

`pickup` leaves grasp orientation completely unconstrained (deliberately -
constraining it breaks reachability at some approach points). `pour` then
applies a **relative** +90&deg; pitch delta on top of whatever arbitrary
orientation the planner happened to land on, which can produce a physically
extreme, unreachable target pose (observed in testing: a starting orientation
of `rpy = (-3.09, 2.18, -1.23)` rad led straight to a MoveIt error code
`99999` on the very first tilt attempt). The fix implied by this demo is to
stop leaving the grasp orientation to chance in the first place.

## The five stages

| # | Label | joint_1 | joint_2 | joint_3 | joint_4 | joint_5 | joint_6 | tool xyz | tool rpy (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 1 | Side grasp | 25.13 | 85.92 | -124.53 | -63.38 | -112.28 | -74.76 | (-0.3685, -0.1325, 0.0173) | (95.3, -2.05, -89.67) |
| 2 | Level lift | 25.13 | 57.64 | -149.54 | -63.38 | -112.28 | -74.76 | (-0.3423, -0.1202, 0.1539) | (92.34, -3.42, -89.53) |
| 3 | Transit above target | 66.45 | 56.55 | -114.2 | -64.46 | -82.23 | -95.88 | (-0.2394, -0.4517, 0.1621) | (89.41, 1.91, -48.86) |
| 4 | Tilt to pour | 66.45 | 56.55 | -114.2 | -64.46 | -82.23 | 39.54 | (-0.2394, -0.4517, 0.1621) | (-89.14, -46.48, 130.54) |
| 5 | Return & place | 24.57 | 74.5 | -104.95 | -64.46 | -82.23 | -92.63 | (-0.4774, -0.1754, 0.0436) | (97.28, 2.37, -90.63) |

(Gripper open, dropping the object, was described but not captured - it's
a separate discrete action, not a joint pose.)

### What's invariant vs. what moves, stage to stage

- **1 -> 2 (lift)**: only `joint_2`/`joint_3` (shoulder/elbow) move.
  `joint_4/5/6` - the wrist, which actually controls tilt - are untouched.
  Tool pitch barely drifts (-2.05&deg; -> -3.42&deg;). A pure lift.
- **2 -> 3 (transit)**: `joint_1` swings the base to reposition over the
  destination; tool z is preserved (0.1539 -> 0.1621). Roll/pitch (the
  axes that actually tip a held container) stay in-band (~90&deg;/~0&deg;);
  only yaw changes freely, which is expected and harmless - rotating
  around the vertical axis doesn't spill contents.
- **3 -> 4 (tilt)**: only `joint_6` changes (by ~135&deg;), position is
  *exactly* unchanged. Tool pitch swings from ~2&deg; to -46.48&deg; - the
  actual pour tip - driven entirely by that one joint.
- **4 -> 5 (return & place)**: `joint_6` rotates back, tool pitch returns
  to ~2&deg; (matching stages 1-3), then height drops toward table/resting
  level at the destination, ready for the gripper to open.

## Why this is implementable

This maps cleanly onto machinery already built and proven this session for
`throw`: a small joint-space sequence via `call_joint_move_service`
(`JointMove.srv`), rather than `pour`'s current approach of chaining
Cartesian relative moves with orientation deltas computed on top of an
unknown starting pose. Concretely:

1. **Force a known grasp orientation** for anything that's going to be
   poured - a new orientation preset (e.g. `side_grasp_flat`, roll~90&deg;,
   pitch~0&deg;) applied during the pickup step, replacing today's
   unconstrained approach. This is the actual fix for the root cause above:
   `pour` no longer has to guess what orientation it's starting from.
2. **Lift and transit** by moving to new (x, y, z) targets while re-applying
   that *same* orientation preset every time (not a relative delta) - i.e.
   treat orientation as locked/carried forward, only ever an explicit,
   deliberate absolute target.
3. **Tilt** via a pure joint-space delta on the final wrist joint alone
   (mirroring `_handle_throw`'s wind-up/fling pattern exactly), not an
   IK-solved Cartesian orientation change.
4. **Return** by reversing that same joint delta, then move down to the
   destination's resting height (reusing `dropoff`'s height math), then
   open the gripper.

## Generalizing the "stays upright" philosophy

The core idea - lock orientation across every *position*-changing move,
and only ever change it via one deliberate, isolated motion - is not
pour-specific and is worth applying anywhere a filled/spillable object is
being carried (this is exactly the bug pour had: an orientation change
leaking in where none was intended). Two things need to hold for it to
generalize correctly:

- **The forced grasp orientation must be the same canonical preset every
  time**, regardless of which object or destination is involved. The
  joint_6-rotation-equals-tip trick only works because the wrist is
  already sitting in that specific orientation (roll~90&deg;) from the
  grasp - a different grasp orientation would change which joint (or
  combination of joints) actually produces a "tip" in the world frame.
- **Every intermediate move (lift/transit) must re-supply that exact
  orientation as an explicit target**, not leave it to the planner or
  apply a relative delta - otherwise the same silent drift that broke the
  current `pour` can reappear.

Given those two constraints, yes - this pattern should generalize to any
future "carry something upright" action, not just this specific box/tray
case.

## Implementation

Implemented in `arm_actions.py`/`hardware_interface_client.py`:

- `pickup` gained an optional `orientation` param (unconstrained by
  default, unchanged for every existing recipe) - a new preset
  `side_grasp_flat` (`orientation_presets.json`) holds the exact roll/pitch
  captured in stage 1 above, for anything that needs a known, repeatable
  grasp rather than whatever the planner lands on.
- `pour` gained `destination`/`direction`, `lift_height` (default `0.14`,
  matching the ~0.137m measured here) and `tilt_angle` (default `radians(135)`,
  matching the captured joint_6 delta). Lift/transit are zero-delta
  `relative_move` calls (orientation preserved exactly, never recomputed).
- The tilt/return-to-level itself needed a new capability: `JointMove.srv`
  gained a `relative` field, and `hardware_interface_client` now caches
  `/joint_states` so a relative joint move can resolve to
  `current + delta` server-side - this is what lets the tilt be a pure
  `joint_6` delta rather than a Cartesian orientation change.
- `pour` deliberately stops once it's back level - still holding the
  object, hovering above the destination. Placing it down and releasing
  is left to a separate `dropoff` step in the recipe, reusing its
  existing height/release logic rather than duplicating it here.
