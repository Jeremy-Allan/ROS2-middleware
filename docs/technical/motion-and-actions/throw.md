[Docs Home](../../README.md)

# Throw

`throw` swings a held object and releases it partway through one continuous motion. It is implemented in `actions/throw.py`. The object must already be held.

## Sequence

1. **Face the direction** of the throw by rotating `joint_1` only. The shoulder, elbow and wrist stay as `pickup` left them.
2. **Wind up** by moving to a fixed wind-up pose (`THROW_WINDUP_POSE`).
3. **Fling** in one continuous swing to a fixed end pose (`THROW_FLING_POSE`). The fling is planned before anything moves.
4. **Release** by opening the gripper the moment `joint_5` crosses `THROW_RELEASE_JOINT5`. The trigger reads the arm's live `joint_5` value instead of using a timer. It gives up after the planned duration of the fling plus a fixed margin.
5. **Land** the object. It is moved along a short simplified arc from the release point to its landing spot, then removed from the scene so that no floating collision object blocks later moves. The landing position is an approximation, not a physics result.

Only `joint_1` changes between throws. `destination`, or `direction` and `distance`, decide which way the arm faces. The shape of the swing is fixed.

## Captured stages

The poses are based on a demonstration driven by hand in RViz on simulated hardware. Joint values are in degrees.

| # | Label | joint_1 | joint_2 | joint_3 | joint_4 | joint_5 | joint_6 |
|---|---|---|---|---|---|---|---|
| 1 | Wind-up | -0.0 | -22.84 | 56.01 | 80.71 | 50.6 | 0.0 |
| 2 | Fling begins | -0.0 | -4.35 | 17.95 | 80.71 | 50.6 | 0.0 |
| 3 | Release point | 0.56 | 5.44 | -4.89 | 80.71 | **-42.7** | 0.0 |
| 4 | End pose (already released) | 0.56 | 32.63 | -34.26 | 80.71 | -63.25 | 0.0 |

`joint_1`, `joint_4` and `joint_6` are fixed throughout. `joint_2` and `joint_3` sweep smoothly through all four stages. `joint_5` barely moves until "Fling begins", then swings about 93 degrees up to the release point. That snap is why `joint_5` is used as the release signal.
