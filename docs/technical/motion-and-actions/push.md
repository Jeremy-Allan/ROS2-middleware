[Docs Home](../../README.md)

# Push

`push` slides an object away from the arm by keeping the gripper in contact with it. The object is not lifted. `thrust`, described at the end of this page, uses the same mechanism for a held object. Both are implemented in `actions/push.py` and `actions/thrust.py`.

## Sequence

1. **Face the object** by rotating `joint_1` only.
2. **Move to the contact pose** and close the gripper on the object. The object is then attached, so its position in the planning scene follows the arm.
3. **Extend** by moving only `joint_2` and `joint_3`. The wrist stays level at (0, 0, 0), so the whole motion happens in one vertical plane.
4. **Open and detach** the object, update its stored pose, and retreat to the contact pose.

The target is either a named `destination`, or a `direction` (`forward`, `backward`, `left`, `right`, relative to the object's bearing from the arm) with a `distance`. Push only moves an object further from the arm's base. It does not drag one back in.

## How the poses are found

The contact pose and the extended pose are solved with `solve_planar_reach`. It uses forward kinematics (`/compute_fk`) and a Newton-Raphson search over `joint_2` and `joint_3` to reach a given radial distance and height. Both poses are checked with `check_joint_state_validity` before the arm moves.

Contact with the object is the purpose of a push, so the pushed object is temporarily exempted from collision checking (`set_collision_allowed`). The exemption covers only that object, and it is always reverted afterwards, even if the push fails. The target is the object's registered centre, so a large object may be pushed from a point a person would not choose.

## Captured stages

The motion is based on a demonstration driven by hand in RViz on simulated hardware. Joint values are in degrees.

| # | Label | joint_1 | joint_2 | joint_3 | joint_4/5/6 | tool xyz | tool rpy (deg) |
|---|---|---|---|---|---|---|---|
| 1 | Contact, gripper closed | -12.84 | -21.75 | 138.67 | 0, 0, 0 | (0.2034, -0.0566, 0.0320) | (160.42, 0.0, 77.16) |
| 2 | Extend | -12.84 | -54.92 | 81.02 | 0, 0, 0 | (0.5067, -0.1258, 0.0196) | (135.95, 0.0, 77.16) |
| 3 | Max extend | -12.84 | -89.73 | 23.93 | 0, 0, 0 | (0.6772, -0.1646, -0.0002) | (113.65, -0.0, 77.16) |

Across all three stages `joint_1` and the wrist (`joint_4`, `joint_5`, `joint_6`) do not change. Only `joint_2` and `joint_3` move.

## Thrust

`thrust` raises a held object and extends it toward a destination, using the same single-plane mechanism. The object must be held and grasped from the side.

1. **Raise** the object straight up by `lift_height`, facing the direction it was resting in before pickup. The wrist is reset to level.
2. **Turn** to face the thrust direction by rotating `joint_1` only.
3. **Extend** toward the `destination`, or the `direction` and `distance`, using `joint_2` and `joint_3` only.

The object is attached to the gripper, so `thrust` needs no collision exemption.
