"""'pickup' recipe action: pre-grasp -> straight approach -> close -> attach -> straight lift.

Grasp candidates come from the object's shape and pose. Each one's moves are planned in a chain before
the arm moves, and the first fully plannable one is used. On failure the arm backs off and goes home."""
from collections import Counter
from typing import TYPE_CHECKING

import numpy as np

from kinova_interface.actions import dropoff
from kinova_interface.actions.motion import MotionUnavailable
from kinova_interface.utils.grasping import GraspCandidate, Waypoint, grasp_candidates, object_centre, object_rotation, rank
from kinova_interface.utils.robot import GRIPPER_CLOSED, GRIPPER_OPEN

# Only for the ctx type hint (editor go-to-definition). Not imported at runtime,
# because arm_actions imports this module and that would be circular.
if TYPE_CHECKING:
    from kinova_interface.actions.arm_actions import ArmActions

GRASP_STYLES = ('auto', 'top', 'side')


def _fmt(position) -> list[float]:
    return np.round(position, 3).tolist()


def _summary(rejected: Counter) -> str:
    """e.g. 'side 16 tried (4 too wide: 0.120 m > max 0.094; 12 no path to pre-grasp (...)), top 4 tried (...)'"""
    parts = []
    for style in ('side', 'top'):
        reasons = [(reason, n) for (s, reason), n in rejected.items() if s == style]
        if reasons:
            total = sum(n for _, n in reasons)
            parts.append(f"{style} {total} tried ({'; '.join(f'{n} {r}' for r, n in reasons)})")
    return ', '.join(parts) or 'no candidates'


def _back_off(ctx: 'ArmActions', candidate: GraspCandidate, pre_grasp: Waypoint, message: str) -> tuple[bool, str]:
    """Open, back out to the pre-grasp in a straight line, then go home and fail.
    If the arm can't get clear of the object, it stays put rather than dragging it."""
    ctx.get_logger().warn(f"{message}; opening the gripper and backing off to the pre-grasp")
    gripper = ctx.call_move_gripper_service(GRIPPER_OPEN)
    if not gripper['success']:
        return False, f"{message}; failed to open the gripper to back off: {gripper['message']}"
    ok, reason = ctx.motion.move(pre_grasp, candidate.rotation, ctx.grasp_config)
    if not ok:
        return False, f"{message}; failed to back off to the pre-grasp: {reason}"
    return ctx.fail_at_home(message)


def run(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    """'grasp_style' can be 'auto' (default, top or side from the object's shape), 'top' or 'side'."""
    log = ctx.get_logger()
    target_name = params['target']
    style = params.get('grasp_style', 'auto')
    if style not in GRASP_STYLES:
        return False, f"Unknown grasp_style '{style}', expected one of {', '.join(GRASP_STYLES)}"

    target_info = ctx.get_object_info(target_name)
    if not target_info:
        return False, f"Could not resolve pickup target '{target_name}'"
    log.debug(f"Pickup target '{target_name}': pose={target_info['pose']}, shape={target_info['shape']}")

    config = ctx.grasp_config
    try:
        candidates, rejected = grasp_candidates(target_info, config, style)
    except ValueError as e:
        return False, f"Can't grasp '{target_name}': {e}"
    candidates = rank(candidates, target_info, config, style)
    log.info(f"Pickup '{target_name}': {len(candidates)} {style} grasp candidate(s), "
             f"{sum(rejected.values())} ruled out by size")
    if rejected:
        log.debug(f"Ruled out by size: {_summary(rejected)}")

    # Put back anything already held, otherwise the open below drops it wherever the arm is
    if ctx.held_object and ctx.held_object != target_name:
        held = ctx.held_object
        log.info(f"Pickup '{target_name}': putting '{held}' back first")
        ok, message = dropoff.run(ctx, {'target': held})
        if not ok:
            return False, f"Failed to release '{held}' before picking up '{target_name}': {message}"

    # Open first so the planned moves see the fingers open
    gripper = ctx.call_move_gripper_service(GRIPPER_OPEN)
    if not gripper['success']:
        return ctx.fail_at_home(f"Failed to open the gripper for pickup: {gripper['message']}")

    # The first candidate whose whole pre-grasp -> grasp -> lift chain can be planned
    for index, candidate in enumerate(candidates, 1):
        try:
            plans, reason = ctx.motion.plan_chain(candidate.waypoints, candidate.rotation, config)
        except MotionUnavailable as e:
            return ctx.fail_at_home(f"Can't plan the pickup of '{target_name}': {e}")
        if plans is not None:
            break
        log.debug(f"Grasp {index}/{len(candidates)} ({candidate.style} at {_fmt(candidate.grasp)}) rejected: {reason}")
        rejected[(candidate.style, reason)] += 1
    else:
        return ctx.fail_at_home(f"No reachable grasp for '{target_name}': {_summary(rejected)}")

    pre_grasp, grasp, lift = candidate.waypoints
    log.info(f"Pickup '{target_name}': grasp {index}/{len(candidates)}, {candidate.style} at {_fmt(candidate.grasp)} "
             f"(planned with {', '.join(f'{name}: {plan.planner}' for name, plan in plans.items())})")

    # The pre-grasp plan starts from the current state, so it can run as is.
    # Later moves are re-planned from where the arm actually is.
    log.info(f"Pickup '{target_name}': moving to the pre-grasp")
    ok, message = ctx.motion.execute(plans['pre-grasp'])
    if not ok:
        return ctx.fail_at_home(f"Failed to reach the pre-grasp for '{target_name}': {message}")

    log.info(f"Pickup '{target_name}': approaching")
    ok, message = ctx.motion.move(grasp, candidate.rotation, config)
    if not ok:
        return _back_off(ctx, candidate, pre_grasp, f"Failed to approach '{target_name}': {message}")

    gripper = ctx.call_move_gripper_service(GRIPPER_CLOSED)
    if not gripper['success']:
        return _back_off(ctx, candidate, pre_grasp, f"Failed to close the gripper on '{target_name}': {gripper['message']}")
    if not ctx.attach_object(target_name):
        return _back_off(ctx, candidate, pre_grasp, f"Failed to attach '{target_name}' after closing")

    # Held from here on, even if the lift fails, so a dropoff can still put it down
    ctx.held_object = target_name
    # How the object sits in the gripper, kept relative to the gripper since that doesn't change while held.
    # Dropoff works backwards from it to find where the gripper goes.
    ctx.held_grasp = {
        'tool_rotation': candidate.rotation,
        'object_position': candidate.rotation.inv().apply(object_centre(target_info['pose']) - candidate.grasp),
        'object_rotation': candidate.rotation.inv() * object_rotation(target_info['pose']),
    }

    # Re-planned now the object is attached, so the lift is checked with it. Whatever the object
    # stands on would count as a collision at the start, so that one contact is allowed for the lift.
    try:
        supports = ctx.motion.touching(target_name)
    except MotionUnavailable as e:
        return ctx.fail_at_home(f"Grasped '{target_name}' but can't plan the lift: {e}")
    if supports:
        log.info(f"Pickup '{target_name}': resting on {', '.join(supports)}, allowing that contact for the lift")
        if not ctx.motion.allow_contact(target_name, supports, True):
            return ctx.fail_at_home(f"Grasped '{target_name}' but couldn't allow its contact with {', '.join(supports)}")

    log.info(f"Pickup '{target_name}': lifting")
    try:
        ok, message = ctx.motion.move(lift, candidate.rotation, config)
    finally:
        if supports and not ctx.motion.allow_contact(target_name, supports, False):
            log.warn(f"Couldn't restore collision checking between '{target_name}' and {', '.join(supports)}")
    if not ok:
        return ctx.fail_at_home(f"Grasped '{target_name}' but failed to lift it: {message}")
    return True, f"Picked up '{target_name}' with a {candidate.style} grasp"
