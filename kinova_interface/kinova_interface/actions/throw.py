"""'throw' recipe action (see docs/throw-motion-reference.md)."""
import math
import time
from typing import TYPE_CHECKING

from kinova_interface.actions.motion import MotionUnavailable
from kinova_interface.utils.geometry import resolve_direction_offset

# Only for the ctx type hint (editor go-to-definition). Not imported at runtime,
# because arm_actions imports this module and that would be circular.
if TYPE_CHECKING:
    from kinova_interface.actions.arm_actions import ArmActions


# A captured, fixed joint_2..6 shape (see docs/throw-motion-reference.md)
# for a real, manually-demonstrated wind-up - not derived from home by
# a wind-up angle like an earlier version of this; joint_1 (base
# facing) is the only thing that varies per throw.
THROW_WINDUP_POSE = [math.radians(v) for v in [-22.84, 56.01, 80.71, 50.6, 0.0]]
# The fling's own end pose - a single continuous swing straight from
# the wind-up to here, joint_1 fixed the whole way.
THROW_FLING_POSE = [math.radians(v) for v in [32.63, -34.26, 80.71, -63.25, 0.0]]
# joint_5 barely moves during the early part of the fling, then swings
# hard right at the release moment - this is where the gripper should
# open. -42.7 deg was the value directly captured from the original
# manual demo (see docs/throw-motion-reference.md), but in-person
# testing on the real arm found that released too late - visibly near
# the very end of the fling rather than clearly mid-swing - so this has
# been nudged earlier to -35.0 deg (~75% through the joint_5 sweep from
# wind-up to fling-end, vs. ~82% before). Still empirical, not computed -
# further tuning may still be needed on real hardware.
THROW_RELEASE_JOINT5 = math.radians(-35.0)
# Extra time allowed on top of the fling's own planned duration, for
# controller dispatch/startup jitter between firing execution and the
# arm actually beginning to move.
RELEASE_TIMEOUT_MARGIN_SEC = 3.0

# A quick, deliberately simplified stand-in for the thrown object
# actually flying to its landing spot, instead of teleporting straight
# there the instant it's released (see _animate_flight).
FLIGHT_STEPS = 8
FLIGHT_DURATION_SEC = 0.6
FLIGHT_ARC_FRACTION = 0.15  # peak height above a straight line, as a fraction of horizontal distance


def _animate_flight(ctx: 'ArmActions', target_name: str, start, end, orientation):
    """Step the detached object from `start` to `end` ((x, y, z) each) over
    a few moments with a simple rise-then-fall arc, rather than jumping
    straight to its landing spot in one update_object_pose call - a single
    teleport can't look like anything but a drop, no matter how well-timed
    the release itself is. This isn't real ballistics (no actual velocity/
    gravity/air resistance), just enough of a visual cue that it was
    thrown, not placed."""
    (sx, sy, sz), (ex, ey, ez) = start, end
    horizontal_distance = math.hypot(ex - sx, ey - sy)
    peak_height = FLIGHT_ARC_FRACTION * horizontal_distance
    for step in range(1, FLIGHT_STEPS + 1):
        t = step / FLIGHT_STEPS
        x = sx + (ex - sx) * t
        y = sy + (ey - sy) * t
        z = sz + (ez - sz) * t + peak_height * 4 * t * (1 - t)  # parabola: 0 at t=0/1, peak_height at t=0.5
        if not ctx.update_object_pose(target_name, x, y, z, orientation):
            ctx.get_logger().error(f"Failed to update pose for {target_name} mid-flight, but continuing...")
        if step < FLIGHT_STEPS:
            time.sleep(FLIGHT_DURATION_SEC / FLIGHT_STEPS)


def run(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    """A genuine joint-space throw, captured from a manual RViz
    demonstration (see docs/throw-motion-reference.md): rotate to
    face the throw direction (joint_1 only, current shoulder/elbow/
    wrist held exactly where pickup left them), move straight to a
    captured wind-up shape (THROW_WINDUP_POSE), then one continuous
    fling straight to a captured end pose (THROW_FLING_POSE) -
    releasing the gripper the instant joint_5 crosses
    THROW_RELEASE_JOINT5, not after a fixed delay or once the arm
    has stopped.

    Assumes 'target' is already grasped - fails cleanly rather than
    guessing if it isn't.

    The fling is planned explicitly first (ctx.motion.plan_joints),
    *before* anything executes, rather than fired blind through the
    joint-move service the way every other action's moves are. That
    separation is what makes the release trigger below reliable:
    planning (the slow, variable part - Pilz usually answers in
    milliseconds, but a rejection falling back to OMPL RRT* can take
    several seconds) now happens up front, so the release-timeout only
    has to cover the plan's own known execution duration, not an
    unpredictable mix of planning-plus-execution time. A real case hit
    this: Pilz rejected a fling (a joint_1 goal landing right at the
    limit), the RRT* fallback took ~10s just to find a plan, and the
    old flat 5s release-timeout had already expired before the fling
    had moved at all - the gripper never opened, and the fling
    continued unsupervised well after the recipe had already reported
    failure.

    The release itself is still a closed-loop trigger on the arm's
    real, live joint_5 position (wait_for_joint_crossing), not a
    computed time.sleep() - same reasoning as before (see
    docs/throw-motion-reference.md): it's tied to where the arm
    actually is, not an assumption about how long a motion should
    take. What's new is the timeout passed to it: plan.duration (the
    fling's own real planned length) plus a fixed dispatch-jitter
    margin, instead of a flat guess that had no relationship to
    whichever planner actually produced the trajectory.

    Once released, the object is stepped through a short, simplified
    flight arc (_animate_flight) from where the gripper actually was
    at the moment of release to its landing spot, instead of
    teleporting straight there - the previous single update_object_pose
    call couldn't look like anything but the object vanishing from the
    gripper and reappearing at the destination, regardless of how well
    release was timed. This still isn't a physics simulation - the
    landing position itself remains an approximation of the intended
    target, not a computed trajectory - just enough to visually read as
    thrown rather than placed.

    After that, the object is removed from the scene entirely
    (ctx.remove_object) rather than left behind as a real collision
    object. Detaching mid-swing puts the actual collision geometry back
    wherever the gripper happened to be at that instant, nowhere near
    the approximate landing spot above - with no physics confirming it
    actually comes to rest there (or anywhere), leaving it in the scene
    would mean a floating obstacle that can block later moves, like the
    arm being unable to plan its way home afterward."""
    # No target means the held object
    target_name = params.get('target') or ctx.held_object
    destination_name = params.get('destination')
    direction = params.get('direction')
    if not target_name:
        return False, "throw action requires a held object, but nothing is currently held"
    if target_name != ctx.held_object:
        return False, f"Cannot throw '{target_name}': held object is '{ctx.held_object}'"
    if not destination_name and not direction:
        return False, "throw action requires either 'destination' or 'direction'"

    target_info = ctx.get_object_info(target_name)
    if not target_info:
        return False, f"Could not resolve held object '{target_name}' for throw"
    origin = target_info['pose']['position']

    if destination_name:
        dest_info = ctx.get_object_info(destination_name)
        if not dest_info:
            return False, f"Could not resolve throw destination '{destination_name}'"
        release_x, release_y = dest_info['pose']['position']['x'], dest_info['pose']['position']['y']
    else:
        distance = float(params.get('distance', 0.3))
        offset = resolve_direction_offset(origin['x'], origin['y'], direction, distance)
        if offset is None:
            return False, f"Unknown throw direction '{direction}'"
        release_x, release_y = offset

    base_yaw = math.atan2(release_y, release_x)
    motion_params = ctx.build_motion_params(params.get('speed'))

    # 1. Rotate to face the throw direction - joint_1 only, current
    # shoulder/elbow/wrist (wherever pickup left them) held exactly
    if ctx.latest_joint_positions is None:
        return False, "No joint state available to rotate for throw"
    try:
        current_arm_joints = [ctx.latest_joint_positions[f'joint_{i}'] for i in range(2, 7)]
    except KeyError as e:
        return False, f"Missing joint {e} in latest joint state"
    rotate_joints = [base_yaw, *current_arm_joints]
    r = ctx.call_joint_move_service(rotate_joints, motion_params=motion_params)
    if not r['success']:
        return False, f"Failed to rotate to face the throw direction for '{target_name}': {r['message']}"

    # 2. Wind-up - go straight to the captured wind-up shape
    windup_joints = [base_yaw, *THROW_WINDUP_POSE]
    r = ctx.call_joint_move_service(windup_joints, motion_params=motion_params)
    if not r['success']:
        return False, f"Failed to wind up for throw of '{target_name}': {r['message']}"

    # 3. Fling - planned explicitly first (not yet executed), so the
    # release trigger below has a known, bounded execution duration to
    # work with (see this function's docstring for why that matters).
    fling_joints = [base_yaw, *THROW_FLING_POSE]
    speed = float(params.get('speed', 1.0)) or 1.0
    fling_config = {
        'velocity_scale': speed, 'acceleration_scale': speed,
        'fallback_planning_time': ctx.grasp_config['fallback_planning_time'],
    }
    try:
        plan, reason = ctx.motion.plan_joints(fling_joints, fling_config, label=f"throw fling of '{target_name}'")
    except MotionUnavailable as error:
        return False, f"Can't plan the throw fling for '{target_name}': {error}"
    if plan is None:
        return False, f"Can't plan the throw fling for '{target_name}': {reason}"

    windup_joint5 = THROW_WINDUP_POSE[3]
    if plan.time_at_joint_crossing('joint_5', THROW_RELEASE_JOINT5) is None:
        return False, f"Planned fling for '{target_name}' never reaches the usual release point"

    # Fired without waiting for any response at all (see
    # Motion.execute_async's docstring for why)
    fling_future = ctx.motion.execute_async(plan)
    if fling_future is None:
        return False, f"Failed to start throw fling for '{target_name}'"

    # 4. Release the instant joint_5 crosses the captured release
    # point - a closed-loop trigger on the arm's real position, now
    # bounded by this specific plan's own real duration rather than a
    # flat guess (see this function's docstring).
    crossed = ctx.wait_for_joint_crossing(
        'joint_5', THROW_RELEASE_JOINT5, windup_joint5, timeout=plan.duration + RELEASE_TIMEOUT_MARGIN_SEC)
    if not crossed:
        return False, f"Timed out waiting for the release point during throw of '{target_name}'"

    # Where the gripper actually is right now, for the flight animation
    # below to start from - falls back to the pre-throw position if FK
    # isn't available, rather than failing the throw over a cosmetic step.
    release_point = None
    try:
        release_joints = [base_yaw, *(ctx.latest_joint_positions[f'joint_{i}'] for i in range(2, 7))]
        release_point = ctx.compute_fk(release_joints)
    except (KeyError, TypeError):
        pass
    if release_point is None:
        release_point = (origin['x'], origin['y'], origin['z'])

    open_pos = float(params.get('open_position', 0.0))
    rg = ctx.call_move_gripper_service(open_pos)
    if not rg['success']:
        return False, f"Failed to release gripper during throw: {rg['message']}"

    ctx.detach_object(target_name)
    _animate_flight(ctx, target_name, release_point, (release_x, release_y, origin['z']), None)
    # Detaching mid-swing put the real collision object back into the
    # world scene at wherever the gripper was at that instant - nowhere
    # near the landing spot above, and with no physics simulating where
    # it actually settles, there's no pose we could set it to that's
    # trustworthy as a solid obstacle. Removed rather than left behind as
    # a floating collision object that could block later moves (e.g. the
    # arm being unable to plan home afterward) - a /reset_environment_scene
    # call is what brings it back.
    ctx.remove_object(target_name)
    ctx.held_object = None
    ctx.held_grasp = None
    where = f"toward '{destination_name}'" if destination_name else direction
    return True, f"Threw '{target_name}' {where}"
