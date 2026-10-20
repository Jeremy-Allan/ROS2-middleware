[Docs Home](../../README.md)

# Throw Motion Reference

This records a manually-driven demonstration of the redesigned `throw`
motion, captured the same way as `pour`/`push`: joint-space snapshots via
RViz's MotionPlanning Joints tab and `Plan and Execute`, on fake hardware.

## Why the previous design was replaced

The original `throw` computed its wind-up/fling as offsets from `home`'s
own pose (`wind_up_angle`/`fling_angle` applied to `home`'s elbow, with a
small shoulder "rock"). That was a reasonable way to get the mechanism
working the first time, but `home` was never actually load-bearing for
the design - it was just a convenient, known starting shape while the
single-plane approach itself was being worked out. Once that was
understood, the ask was to skip `home` entirely and go straight to a
directly-demonstrated wind-up shape - simpler, and it also surfaced a
real bug in the release timing that the `home`-relative version had been
masking.

## The four stages

| # | Label | joint_1 | joint_2 | joint_3 | joint_4 | joint_5 | joint_6 |
|---|---|---|---|---|---|---|---|
| 1 | Wind-up | -0.0 | -22.84 | 56.01 | 80.71 | 50.6 | 0.0 |
| 2 | Fling begins | -0.0 | -4.35 | 17.95 | 80.71 | 50.6 | 0.0 |
| 3 | Release point | 0.56 | 5.44 | -4.89 | 80.71 | **-42.7** | 0.0 |
| 4 | End pose (already released) | 0.56 | 32.63 | -34.26 | 80.71 | -63.25 | 0.0 |

**What's invariant vs. what moves**: `joint_1` (base facing) and `joint_4`/
`joint_6` are fixed across the entire throw. `joint_2`/`joint_3`
(shoulder/elbow) sweep smoothly and continuously across all four stages -
a single, uninterrupted motion from wind-up straight through to the end
pose. `joint_5` (wrist) is the odd one out: it barely moves at all through
"fling begins", then swings hard (~93 degrees) between "fling begins" and
"release point", continuing further afterward - a wrist-snap concentrated
right at the release moment, not a uniform sweep. That makes `joint_5`
the natural signal for "we've reached the release point": nothing else
about its motion is ambiguous the way the elbow's continuous sweep would
be if used as a release trigger.

Confirmed directly with the user: this is **one continuous swing** from
wind-up straight to the end pose (stage 4), not a sequence of separate
waypoints - "fling begins" (stage 2) is a reference point showing the
motion's direction early on, not a stop along the way. The gripper opens
the instant `joint_5` crosses the stage-3 value during that single swing.

## The real bug this surfaced: release timing

The previous release mechanism was a computed `time.sleep()`, scaled to
an estimated fling duration. That's what silently broke: the fire-and-
forget fling call (`call_joint_move_service(..., wait_for_completion=False)`)
still blocks the *client* for up to
`HardwareInterfaceClient.FIRE_AND_FORGET_REJECTION_WINDOW_SEC` (0.5s) -
the server's own bounded wait to catch a fast rejection before replying.
With the old, larger 225-degree wind-up this was a small fraction of the
whole motion and went unnoticed; with a shorter, more direct wind-up like
this one, the entire fling could finish *during* that 0.5s block, before
any release-timing code had even started running - the gripper would
only open once the arm had already stopped, exactly the symptom
reported ("no velocity... releasing while the arm is in motion isn't
happening").

### The fix

- `call_joint_move_service_async` - a new method that fires the fling and
  returns the raw future immediately, waiting for **nothing** at all (not
  even the bounded fire-and-forget window `call_joint_move_service`
  still waits for). This is what actually unblocks the client in time.
- `wait_for_joint_crossing(joint_name, threshold, starting_value)` - a
  closed-loop release trigger. `actions/arm_actions.py` now subscribes to
  `/joint_states` (mirroring `hardware_interface_client`'s own pattern)
  and polls the arm's **real, live** `joint_5` value, returning the
  moment it actually crosses the release threshold. This ties release
  timing to where the arm truly is, not to an estimate of how long a
  motion *should* take - which is a fundamentally more robust fix than
  computing a better delay, given the previous, simpler delay had
  already gone through one round of tuning and still failed.

## Direction fix: the swing is mirrored

As demonstrated, this swing throws *away* from the way `joint_1` faces:
with `joint_1` at 0 (facing +X), the gripper winds up in front at
x = +0.53 m and finishes behind the base at x = -0.55 m (MoveIt FK). Since
`throw` sets `joint_1` to the target's bearing, every throw went roughly
180 degrees away from its target. Turning `joint_1` round by 180 degrees
instead isn't possible for targets in front of the arm (joint_1 is limited
to about ±154 degrees), so the code uses the mirrored shape: `joint_2` and
`joint_3` negated, `joint_4` - 180 degrees, `joint_5`/`joint_6` unchanged.

| # | Label | joint_2 | joint_3 | joint_4 | joint_5 | joint_6 |
|---|---|---|---|---|---|---|
| 1 | Wind-up | 22.84 | -56.01 | -99.29 | 50.6 | 0.0 |
| 4 | End pose | -32.63 | 34.26 | -99.29 | -63.25 | 0.0 |

FK confirms it's an exact mirror: same heights, x negated, so the gripper
winds up behind the base and finishes on the target's side, within about
3 degrees of `joint_1`'s bearing. `joint_5` sweeps exactly as before, so
the release trigger (`THROW_RELEASE_JOINT5`) is unchanged.

## Implementation

- `THROW_WINDUP_POSE`/`THROW_FLING_POSE` - fixed `joint_2..6` shapes
  from stages 1 and 4 above, mirrored (see the direction fix above). Only `joint_1` varies per
  throw (the facing direction), exactly like `push`/`thrust`'s single
  fixed shapes.
- **Rotate** (new first step): joint_1 only, to `atan2(release_y, release_x)`
  - current shoulder/elbow/wrist (read from `latest_joint_positions`,
    wherever `pickup` left them) held exactly as they are. This replaces
    routing through a fixed `face_pose` derived from `home`.
- **Wind-up**: a normal blocking joint move straight to `THROW_WINDUP_POSE`
  (joint_1 = the same face yaw).
- **Fling**: `call_joint_move_service_async` straight to `THROW_FLING_POSE`
  - fired, not waited on.
- **Release**: `wait_for_joint_crossing('joint_5', THROW_RELEASE_JOINT5, windup_joint5)`
  - the moment it returns, the gripper opens immediately.

`wind_up_angle`/`fling_angle`/`joint_2_rock_angle`/`release_delay` are all
gone - there's nothing left to tune per-call for the shape of the swing
itself; `distance`/`destination`/`direction` still control which way it
faces, the same as `push`/`thrust`.
