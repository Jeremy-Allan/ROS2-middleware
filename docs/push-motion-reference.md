# Push Motion Reference

This records a manually-driven demonstration of the intended `push`
motion, captured the same way as `pour` (see `pour-motion-reference.md`):
one joint-space snapshot per stage, via RViz's MotionPlanning interactive
marker/Joints tab and `Plan and Execute`, on fake hardware.

Captured with a new dedicated test object, `push_block`
(`kinova_interface/data/configs/env/object_dictionary.json`), added
specifically so push testing didn't keep disturbing `box`'s state. Resting
at `x=0.25, y=-0.05, z=0.01`.

## Why the previous design would have failed anyway

Separately from the reach/distance issue that first surfaced `push`'s
failures, `push` never attaches the object it's sliding (deliberately - it
never grasps or lifts it). That means the object stays a **static
obstacle** in the planning scene for the entire motion. Sliding the
gripper across it in continuous contact - the entire point of a push -
would either be rejected outright as a collision, or worse, MoveIt could
find a path that avoids the object instead of dragging it, "succeeding"
while the object never actually moves. This needed fixing regardless of
the distance-based failure already found.

## The three stages

| # | Label | joint_1 | joint_2 | joint_3 | joint_4/5/6 | tool xyz | tool rpy (deg) |
|---|---|---|---|---|---|---|---|
| 1 | Contact, gripper closed | -12.84 | -21.75 | 138.67 | 0, 0, 0 | (0.2034, -0.0566, 0.0320) | (160.42, 0.0, 77.16) |
| 2 | Extend | -12.84 | -54.92 | 81.02 | 0, 0, 0 | (0.5067, -0.1258, 0.0196) | (135.95, 0.0, 77.16) |
| 3 | Max extend | -12.84 | -89.73 | 23.93 | 0, 0, 0 | (0.6772, -0.1646, -0.0002) | (113.65, -0.0, 77.16) |

**What's invariant vs. what moves**: `joint_1` and the entire wrist
(`joint_4/5/6`) are *exactly* unchanged across all three stages - only
`joint_2`/`joint_3` (shoulder/elbow) move. Tool pitch and yaw are also
exactly unchanged throughout; only roll drifts (a representation artifact
of this specific joint sweep, not a deliberately controlled axis - see the
same phenomenon noted for `pour`'s tilt). Height falls off naturally as a
consequence of the sweep, settling near table level by full extension -
it isn't separately controlled either.

**Explicit scope note from the demo**: push only ever extends an object
further from the arm's own base. Dragging one back in is out of scope -
consistent with the mechanism itself: a pure shoulder/elbow sweep is
inherently a radial reach outward, not an arbitrary directional drag.

## Implementation

The final design locks `joint_1` (facing the object) and the wrist
(`joint_4/5/6`, held level at `(0, 0, 0)`, matching the demo exactly) for
the *entire* push, and solves only `joint_2`/`joint_3` - the whole motion
stays in one vertical plane, exactly matching what was demonstrated.
Getting there took three attempts; the first two are recorded below
because they're genuine, verified dead ends, not just abandoned ideas.

### Attempt 1: general 6-DOF IK candidates (abandoned)

Reused `pickup`'s `grasp_style: 'side'` machinery
(`compute_side_grasp_candidates` + `find_ik_solution`) to find *some*
reachable, collision-free 6-DOF pose near the object, then a second
`find_ik_solution` (seeded at the first) for the extended pose, then a
relative joint delta between them. This is what actually ran on real
hardware and visibly produced an arm reaching **sideways** at an
unrelated angle - not a bug in execution, but in the search: unseeded
`/compute_ik` returns *some* valid solution to a Cartesian+orientation
goal, with no preference for a natural or reusable one, and nothing
about a single IK solve predicts whether a second, nearby target is also
reachable *from that specific configuration*. Confirmed directly:
seeding a second `/compute_ik` call at a real, working contact
configuration for a target just 5-20cm further away routinely returned
`NO_IK_SOLUTION`, or silently jumped to an unrelated configuration.

### Attempt 2: plan-only pre-check (abandoned)

Tried verifying the extend would succeed *before* committing to a
contact candidate, using a real planner instead of raw IK: neither the
raw `/plan_kinematic_path` service nor the actual `MoveGroup` action's
own `plan_only` flag (the same action every successful move already
uses, just not executing the result) could produce a plan for even a
trivial position-only goal with no custom start state - both returned
the same generic `FAILURE` code with zero diagnostic output, on requests
`/compute_ik` solved and `call_move_service` (actually executing) had
already been proven to handle correctly elsewhere. That points at "plan
without executing" not being reliably usable in this setup at all, not
at these specific goals being unreachable - so a "try for real, retreat
if it fails" version was built on top of attempt 1 instead. It worked in
principle but still inherited attempt 1's real problem: an essentially
arbitrary, hard-to-predict contact configuration each time.

### Attempt 3: single-plane forward-kinematics solve (current)

Locking `joint_1` and the wrist removes the ambiguity entirely - with
only 2 unknowns (`joint_2`, `joint_3`) left, "where can the gripper
reach" is a small, smooth, well-behaved function, not a general 6-DOF
search:

- `base_yaw = atan2(object_y, object_x)` - face the object, exactly like
  `throw`'s `face_pose`. Fixed for the whole action.
- `solve_planar_reach(base_yaw, x, y, z, seed_shoulder, seed_elbow)` -
  Newton-Raphson on (radial distance from the base, height), evaluated
  via `/compute_fk` (not `/compute_ik`) at each iteration. With a smooth,
  invertible 2-variable function, this converges to near machine
  precision from a reasonable seed - verified directly: seeded at a
  generic "bent forward and down" default, it solved `push_block`'s
  contact pose to an error of `~1e-17`, collision-free
  (`/check_state_validity` confirmed no contact); seeded at that contact
  solution, it solved a destination 20cm further with the same
  precision. A seed near `(0, 0)` was tried and found to converge to the
  wrong side (behind the arm) - the default seed needs to already be a
  genuinely bent posture, not straightened.
- Both the contact and extend joint states are checked with
  `check_joint_state_validity` (`/check_state_validity` against the
  *exact* known joint values - no IK ambiguity at all) before any real
  motion happens.
- **Collision handling**: `set_collision_allowed(object_id, allowed)` - a
  scoped update to the planning scene's allowed collision matrix (ACM),
  permitting (then reverting) contact between just the pushed object and
  everything else, for the duration of the push. Always reverted in a
  `finally` block, even on failure.

  This needed a second fix after the single-plane solve above still
  failed for real, mid-extend, with `INVALID_MOTION_PLAN`: interpolating
  the joint-space path from the (verified, collision-free) contact state
  to the (also verified) extend state, `/check_state_validity` showed
  the *start* of that path genuinely in contact with the pushed object
  (correct - the gripper is closed on it) even with
  `set_collision_allowed(object_id, True)` already applied. The original
  implementation only set the ACM's `default_entry` fallback for the
  object; MoveIt auto-populates *explicit* disallow entries for a
  collision object against nearby links as soon as it's added to the
  scene, and those explicit entries take precedence over a blanket
  default - so the "permission" was silently having no effect. Fixed by
  querying the current ACM (`/get_planning_scene`) and writing an
  explicit row/column for the object against every known link, not
  relying on the default fallback at all.

`solve_planar_reach` was deliberately written generic (facing angle +
fixed wrist + 2-joint solve) rather than push-specific, since the same
single-plane extend/retract shape is intended to generalize to `thrust`.

### Generalizing past `push_block`: bigger objects

Retested end to end against `box` (~107x75mm, much bigger than
`push_block`'s ~60x60mm), targeting it by name with no code changes -
confirming the mechanism itself is generic. It failed at the contact
check: `check_joint_state_validity` correctly found the gripper
genuinely overlapping `box` at its solved contact pose. Verified
directly (`/check_state_validity`) that this is a real, single-cause
collision - `(gripper finger links, box)` only, nothing else (no table,
no self-collision) - because reaching a bigger object's exact registered
*center* inherently requires the gripper to overlap it more than a small
object like `push_block` ever does.

Rather than computing a near-face offset from the object's own
dimensions (the fix used for `pickup`'s equivalent problem), the chosen
fix here is simpler and was an explicit choice: contact with the target
object is the entire point of a push, so `set_collision_allowed` is now
applied *before* the geometry/validity checks, not after - the checks
still correctly catch a collision with anything else (table, other
objects), just not with the object being pushed. This accepts that a
bigger object may get pushed slightly off from how a human would
naturally grip it (by its center rather than a computed near-face point)
in exchange for keeping the single-plane motion exactly as simple as
it already is.

## Reused by `thrust`

`thrust` reuses this exact mechanism (`solve_planar_reach` +
`check_joint_state_validity`) for a held object rather than a resting
one: raise straight up (facing wherever it was originally resting,
wrist reset to `_PLANAR_REACH_WRIST` regardless of whatever orientation
`pickup`'s `grasp_style: 'side'` search happened to land on), spin to
face the thrust direction (`joint_1` only - shoulder/elbow held exactly
where the raise left them, so an arbitrarily large turn doesn't disturb
the object's orientation, only which way the arm points), then extend
(shoulder/elbow only, seeded at the spin position). Since the object is
attached by that point (from `pickup`), it's excluded from collision
checking against the gripper automatically - unlike `push`, `thrust`
needs no `set_collision_allowed` at all.
