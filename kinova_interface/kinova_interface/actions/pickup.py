"""'pickup' recipe action: choose a grasp from the object's shape and pose, check it with IK,
then pre-grasp -> grasp -> close -> attach -> lift. On failure the arm backs off and goes home."""
from collections import Counter
from typing import TYPE_CHECKING

import numpy as np

from kinova_interface.actions import dropoff
from kinova_interface.utils.grasping import grasp_candidates, object_centre, object_rotation, rank
from kinova_interface.utils.robot import BEFORE_MOTION_ERRORS, GRIPPER_CLOSED, GRIPPER_OPEN

# Only for the ctx type hint (editor go-to-definition). Not imported at runtime,
# because arm_actions imports this module and that would be circular.
if TYPE_CHECKING:
    from kinova_interface.actions.arm_actions import ArmActions

GRASP_STYLES = ('auto', 'top', 'side')


def _summary(rejected: Counter) -> str:
    """e.g. 'side 16 tried (4 too wide: 0.120 m > max 0.094; 12 no IK at pre-grasp), top 4 tried (4 no IK at grasp)'"""
    parts = []
    for style in ('side', 'top'):
        reasons = [(reason, n) for (s, reason), n in rejected.items() if s == style]
        if reasons:
            total = sum(n for _, n in reasons)
            parts.append(f"{style} {total} tried ({'; '.join(f'{n} {r}' for r, n in reasons)})")
    return ', '.join(parts) or 'no candidates'


def _back_off(ctx: 'ArmActions', pre_grasp: list[float], message: str) -> tuple[bool, str]:
    """Let go and return to the pre-grasp, then go home and fail.
    If the arm can't get clear of the object, it stays put rather than dragging it."""
    gripper_response = ctx.call_move_gripper_service(GRIPPER_OPEN)
    if not gripper_response['success']:
        return False, f"{message}; failed to open the gripper to back off: {gripper_response['message']}"
    move_service_result = ctx.call_joint_move_service(pre_grasp)
    if not move_service_result['success']:
        return False, f"{message}; failed to back off to the pre-grasp: {move_service_result['message']}"
    return ctx.fail_at_home(message)


def run(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    """'grasp_style' can be 'auto' (default, top or side from the object's shape), 'top' or 'side'."""
    target_name = params['target']
    style = params.get('grasp_style', 'auto')
    if style not in GRASP_STYLES:
        return False, f"Unknown grasp_style '{style}', expected one of {', '.join(GRASP_STYLES)}"

    target_info = ctx.get_object_info(target_name)
    if not target_info:
        return False, f"Could not resolve pickup target '{target_name}'"

    grasp_config = ctx.grasp_config
    try:
        candidate_grasps, rejected_grasps = grasp_candidates(target_info, grasp_config, style)
    except ValueError as e:
        return False, f"Can't grasp '{target_name}': {e}"
    # sort candidate grasps based on preferred style etc
    ranked_grasps = rank(candidate_grasps, target_info, grasp_config, style)
    ctx.get_logger().info(f"'{target_name}': {len(ranked_grasps)} {style} grasps to check, "
                          f"dropped {dict(rejected_grasps) or 'none'}")

    # Put back anything already held, otherwise the open below drops it wherever the arm is
    if ctx.held_object and ctx.held_object != target_name:
        held = ctx.held_object
        ctx.get_logger().info(f"Releasing '{held}' before picking up '{target_name}'")
        ok, message = dropoff.run(ctx, {'target': held})
        if not ok:
            return False, f"Failed to release '{held}' before picking up '{target_name}': {message}"

    # Open first so the IK checks see the fingers open
    gripper_response = ctx.call_move_gripper_service(GRIPPER_OPEN)
    if not gripper_response['success']:
        return ctx.fail_at_home(f"Failed to open gripper for pickup: {gripper_response['message']}")

    # Move in through the pre-grasp of the first candidate that checks out
    for grasp_candidate in ranked_grasps:
        joints, reason = ctx.solve_ik_chain(grasp_candidate)
        if joints is None:
            ctx.get_logger().debug(f"{grasp_candidate.style} grasp at {np.round(grasp_candidate.grasp, 3).tolist()} rejected: {reason}")
            rejected_grasps[(grasp_candidate.style, reason)] += 1
            continue
        failed_at = None
        for waypoint in ('pre-grasp', 'grasp'):
            move_service_result = ctx.call_joint_move_service(joints[waypoint])
            if not move_service_result['success']:
                failed_at = waypoint
                break
        if failed_at is None:
            break
        # The arm didn't move, so it's still clear of the object and can try the next grasp from here
        if move_service_result['error_code'] in BEFORE_MOTION_ERRORS:
            ctx.get_logger().info(f"Planning to the {grasp_candidate.style} {failed_at} failed "
                                  f"({move_service_result['message']}), trying the next grasp")
            rejected_grasps[(grasp_candidate.style, f'planning failed to {failed_at}')] += 1
            continue
        message = f"Failed partway to the {failed_at} for '{target_name}': {move_service_result['message']}"
        if failed_at == 'pre-grasp':
            return ctx.fail_at_home(message)
        return _back_off(ctx, joints['pre-grasp'], message)
    else:
        return ctx.fail_at_home(f"No reachable grasp for '{target_name}': {_summary(rejected_grasps)}")

    pre_grasp = joints['pre-grasp']
    ctx.get_logger().info(f"Picking '{target_name}' with a {grasp_candidate.style} grasp at {np.round(grasp_candidate.grasp, 3).tolist()}")

    gripper_move_service_result = ctx.call_move_gripper_service(GRIPPER_CLOSED)
    if not gripper_move_service_result['success']:
        return _back_off(ctx, pre_grasp, f"Failed to close gripper on '{target_name}': {gripper_move_service_result['message']}")

    if not ctx.attach_object(target_name):
        return _back_off(ctx, pre_grasp, f"Failed to attach '{target_name}' after closing")

    # Held from here on, even if the lift fails, so a dropoff can still put it down
    centre = object_centre(target_info['pose'])
    ctx.held_object = target_name
    # How the object sits in the gripper. Dropoff works backwards from this to find where the tool
    # goes so the object lands in the right spot. Kept relative to the tool, since that doesn't change
    # while the object is held.
    ctx.held_grasp = {
        'tool_rotation': grasp_candidate.rotation,  # gripper tilt at pickup, reused at dropoff
        # Gripper to object centre, in the gripper's own axes
        'object_position': grasp_candidate.rotation.inv().apply(centre - grasp_candidate.grasp),
        # Object orientation relative to the gripper
        'object_rotation': grasp_candidate.rotation.inv() * object_rotation(target_info['pose']),
    }

    move_service_result = ctx.call_joint_move_service(joints['lift'])
    if not move_service_result['success']:
        return ctx.fail_at_home(f"Grasped '{target_name}' but failed to lift it: {move_service_result['message']}")
    return True, f"Picked up '{target_name}' with a {grasp_candidate.style} grasp"
