"""'dropoff' recipe action: pickup in reverse. Hover -> straight descent -> open -> detach -> straight back-off.

The object is placed with the grasp it was picked up with, trying other yaws about vertical if the pickup one
can't be planned. Each yaw's moves are planned in a chain before the arm moves. On failure the arm backs off
and goes home."""
from collections import Counter
from typing import TYPE_CHECKING

import numpy as np

from kinova_interface.actions.motion import MotionUnavailable
from kinova_interface.utils.geometry import resolve_direction_offset
from kinova_interface.utils.grasping import (
    PlaceCandidate, Waypoint, contains_xy, half_extent, object_centre, object_rotation, place_candidates,
)
from kinova_interface.utils.robot import GRIPPER_OPEN

# Only for the ctx type hint (editor go-to-definition). Not imported at runtime,
# because arm_actions imports this module and that would be circular.
if TYPE_CHECKING:
    from kinova_interface.actions.arm_actions import ArmActions

DEFAULT_PLACE_OFFSET = 0.05  # hover height above the release pose (m)
MAX_PLACE_OFFSET = 0.12  # higher hovers tend to be out of reach


def _rounded(position) -> list[float]:
    return np.round(position, 3).tolist()


def _back_off(ctx: 'ArmActions', candidate: PlaceCandidate, hover: Waypoint, message: str) -> tuple[bool, str]:
    """Rise back to the hover in a straight line, then go home and fail. If it can't get clear, it stays put."""
    ctx.get_logger().warn(f"{message}; backing off to the hover pose")
    succeeded, reason = ctx.motion.move(hover, candidate.rotation, ctx.grasp_config)
    if not succeeded:
        return False, f"{message}; failed to back off to the hover pose: {reason}"
    return ctx.fail_at_home(message)


def run(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    """All parameters optional. 'target' defaults to the held object.
    The object goes on the centre of 'destination''s top, or back where it was picked up if there's no destination.
    'direction'/'distance' move that spot, relative to the arm's base."""
    logger = ctx.get_logger()
    held = ctx.held_object
    target_name = params.get('target') or held
    if not held:
        return False, "Dropoff needs a held object, but nothing is held"
    if target_name != held:
        return False, f"Not holding '{target_name}' (holding '{held}')"

    target_info = ctx.get_object_info(target_name)
    if not target_info:
        return False, f"Could not resolve dropoff target '{target_name}'"

    # Where the object's bottom centre should end up
    destination_name = params.get('destination')
    try:
        if destination_name:
            destination_info = ctx.get_object_info(destination_name)
            if not destination_info:
                return False, f"Could not resolve destination '{destination_name}'"
            logger.debug(f"Dropoff destination '{destination_name}': pose={destination_info['pose']}, "
                         f"shape={destination_info['shape']}")
            release_point = object_centre(destination_info['pose'])
            release_point[2] += half_extent(destination_info['shape'], object_rotation(destination_info['pose']))
        else:
            release_point = object_centre(target_info['pose'])
            release_point[2] -= half_extent(target_info['shape'], object_rotation(target_info['pose']))
    except ValueError as error:
        return False, f"Can't place '{target_name}': {error}"

    direction = params.get('direction')
    if direction:
        if 'distance' not in params:
            return False, f"Dropoff with direction '{direction}' also needs a 'distance'"
        shifted_xy = resolve_direction_offset(release_point[0], release_point[1], direction, float(params['distance']))
        if shifted_xy is None:
            return False, f"Unknown dropoff direction '{direction}'"
        release_point[0], release_point[1] = shifted_xy
        if destination_name and not contains_xy(destination_info, *shifted_xy):
            return False, f"Release point is off '{destination_name}', try a smaller distance"

    place_offset = float(params.get('place_offset', DEFAULT_PLACE_OFFSET))
    if not 0.0 <= place_offset <= MAX_PLACE_OFFSET:
        return False, f"place_offset must be between 0 and {MAX_PLACE_OFFSET} m, got {place_offset}"

    where = f"on '{destination_name}'" if destination_name else "back where it was"
    config = ctx.grasp_config
    candidates = list(place_candidates(ctx.held_grasp, target_info['shape'], release_point, config, place_offset))
    logger.info(f"Dropoff '{target_name}' {where}: bottom centre at {_rounded(release_point)}, "
                f"{np.hypot(release_point[0], release_point[1]):.3f} m from the base, {len(candidates)} yaw(s) to try")

    # The first yaw whose whole hover -> release -> back-off chain can be planned, the pickup one first
    rejected = Counter()
    for index, candidate in enumerate(candidates, 1):
        try:
            plans, reason = ctx.motion.plan_chain(candidate.waypoints, candidate.rotation, config)
        except MotionUnavailable as error:
            return ctx.fail_at_home(f"Can't plan the dropoff of '{target_name}': {error}")
        if plans is not None:
            break
        logger.debug(f"Yaw {candidate.yaw_deg:+.0f} deg ({index}/{len(candidates)}) rejected: {reason}")
        rejected[reason] += 1
    else:
        reasons = '; '.join(f'{n} {reason}' for reason, n in rejected.items())
        return ctx.fail_at_home(f"No reachable place pose for '{target_name}': {len(candidates)} tried ({reasons})")

    hover, release, back_off = candidate.waypoints
    logger.info(f"Dropoff '{target_name}': yaw {candidate.yaw_deg:+.0f} deg, release at {_rounded(candidate.release)} "
                f"(planned with {', '.join(f'{name}: {plan.planner}' for name, plan in plans.items())})")

    # The hover plan starts from the current state, so it can run as is.
    # Later moves are re-planned from where the arm actually is.
    logger.info(f"Dropoff '{target_name}': moving to the hover pose")
    succeeded, message = ctx.motion.execute(plans['hover'])
    if not succeeded:
        return ctx.fail_at_home(f"Failed to reach the hover pose for '{target_name}': {message}")

    logger.info(f"Dropoff '{target_name}': descending")
    succeeded, message = ctx.motion.move(release, candidate.rotation, config)
    if not succeeded:
        return _back_off(ctx, candidate, hover, f"Failed to lower '{target_name}' to the release pose: {message}")

    gripper_result = ctx.call_move_gripper_service(GRIPPER_OPEN)
    if not gripper_result['success']:
        return _back_off(ctx, candidate, hover, f"Failed to open the gripper to release '{target_name}': {gripper_result['message']}")
    if not ctx.detach_object(target_name):
        return _back_off(ctx, candidate, hover, f"Released '{target_name}' but failed to detach it")

    # A failure here is logged by update_object_pose, the object is still placed
    quaternion = candidate.object_rotation.as_quat()
    ctx.update_object_pose(target_name, *map(float, candidate.object_centre),
                           {'x': quaternion[0], 'y': quaternion[1], 'z': quaternion[2], 'w': quaternion[3]})
    ctx.held_object = None
    ctx.held_grasp = None

    # Re-planned now the object is detached
    logger.info(f"Dropoff '{target_name}': backing off")
    succeeded, message = ctx.motion.move(back_off, candidate.rotation, config)
    if not succeeded:
        return ctx.fail_at_home(f"Placed '{target_name}' {where} but failed to back away: {message}")
    return True, f"Placed '{target_name}' {where}"
