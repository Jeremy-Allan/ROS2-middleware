"""'dropoff' recipe action: put the held object down with the grasp it was picked up with,
then hover -> release -> open -> detach -> back off -> up. On failure the arm backs off and goes home."""
from collections import Counter
from typing import TYPE_CHECKING

import numpy as np

from kinova_interface.utils.geometry import resolve_direction_offset
from kinova_interface.utils.grasping import contains_xy, half_extent, object_centre, object_rotation, place_candidates
from kinova_interface.utils.robot import BEFORE_MOTION_ERRORS, GRIPPER_OPEN

# Only for the ctx type hint (editor go-to-definition). Not imported at runtime,
# because arm_actions imports this module and that would be circular.
if TYPE_CHECKING:
    from kinova_interface.actions.arm_actions import ArmActions

DEFAULT_PLACE_OFFSET = 0.1  # hover height above the release pose (m)


def _back_off(ctx: 'ArmActions', hover: list[float], message: str) -> tuple[bool, str]:
    """Rise back to the hover pose, then go home and fail. If it can't get clear, it stays put."""
    ctx.get_logger().warn("Executing dropoff recovery: backing off to hover pose...")
    move_service_result = ctx.call_joint_move_service(hover)
    if not move_service_result['success']:
        ctx.get_logger().error(f"Failed to back off to hover pose: {move_service_result['message']}")
        return False, f"{message}; failed to back off to the hover pose: {move_service_result['message']}"
    ctx.get_logger().info("Successfully backed off to hover pose, now returning home...")
    return ctx.fail_at_home(message)


def run(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    """All parameters optional. 'target' defaults to the held object.
    The object goes on the centre of 'destination''s top, or back where it was picked up if there's no destination.
    'direction'/'distance' move that spot, relative to the arm's base."""
    held = ctx.held_object
    target_name = params.get('target') or held
    ctx.get_logger().debug(f"Dropoff initiated: held_object='{held}', target='{target_name}', params={params}")

    if not held:
        return False, "Dropoff needs a held object, but nothing is held"
    if target_name != held:  # possible improvement: dropoff held and pickup given target if they are not the same
        return False, f"Not holding '{target_name}' (holding '{held}')"

    target_info = ctx.get_object_info(target_name)
    if not target_info:
        return False, f"Could not resolve dropoff target '{target_name}'"
    ctx.get_logger().debug(f"Target '{target_name}' info: pose={target_info.get('pose')}, shape={target_info.get('shape')}")

    # Where the object's bottom centre should end up
    destination_name = params.get('destination')
    try:
        if destination_name:
            destination_info = ctx.get_object_info(destination_name)
            if not destination_info:
                return False, f"Could not resolve destination '{destination_name}'"
            ctx.get_logger().debug(f"Destination '{destination_name}' info: pose={destination_info.get('pose')}, shape={destination_info.get('shape')}")
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

    where = f"on '{destination_name}'" if destination_name else "back where it was"
    ctx.get_logger().info(f"Placing '{target_name}' {where}: bottom centre at {np.round(release_point, 3).tolist()}, "
                          f"{np.hypot(release_point[0], release_point[1]):.3f} m from the base")

    grasp_config = ctx.grasp_config
    place_offset = float(params.get('place_offset', DEFAULT_PLACE_OFFSET))
    candidates = list(place_candidates(ctx.held_grasp, target_info['shape'], release_point, grasp_config, place_offset))
    ctx.get_logger().info(f"Checking {len(candidates)} place candidate(s) for '{target_name}' {where}...")

    rejected_places = Counter()
    # Move down through the hover of the first candidate that checks out, trying other yaws if the pickup one doesn't work
    for idx, place_candidate in enumerate(candidates):
        ctx.get_logger().debug(
            f"Checking candidate [{idx+1}/{len(candidates)}] (yaw={place_candidate.yaw_deg:+.0f}°): "
            f"hover={np.round(place_candidate.hover, 3).tolist()}, "
            f"release={np.round(place_candidate.release, 3).tolist()}, "
            f"back={np.round(place_candidate.back, 3).tolist()}, "
            f"up={np.round(place_candidate.up, 3).tolist()}")
        joints, reason = ctx.solve_ik_chain(place_candidate)
        if joints is None:
            ctx.get_logger().debug(
                f"Candidate [{idx+1}/{len(candidates)}] (yaw={place_candidate.yaw_deg:+.0f}°) rejected: {reason}")
            rejected_places[reason] += 1
            continue

        ctx.get_logger().debug(
            f"Candidate [{idx+1}/{len(candidates)}] (yaw={place_candidate.yaw_deg:+.0f}°) passed IK pre-check for all waypoints")
        ctx.get_logger().info(
            f"Selected place candidate at yaw {place_candidate.yaw_deg:+.0f} deg. Starting dropoff motion...")

        failed_at = None
        for waypoint in ('hover', 'release'):
            target_pos = place_candidate.hover if waypoint == 'hover' else place_candidate.release
            ctx.get_logger().info(f"Moving to {waypoint} pose at {np.round(target_pos, 3).tolist()} (yaw {place_candidate.yaw_deg:+.0f} deg)...")
            ctx.get_logger().debug(f"Calling joint_move for {waypoint} with joints={np.round(joints[waypoint], 3).tolist()}")
            move_service_result = ctx.call_joint_move_service(joints[waypoint])
            if not move_service_result['success']:
                failed_at = waypoint
                ctx.get_logger().warn(
                    f"Failed to move to {waypoint} pose: {move_service_result['message']} "
                    f"(error_code={move_service_result.get('error_code')})")
                break
            ctx.get_logger().debug(f"Successfully reached {waypoint} pose")

        if failed_at is None:
            break
        # The arm didn't move, so it can try the next yaw from here
        if move_service_result['error_code'] in BEFORE_MOTION_ERRORS:
            ctx.get_logger().info(f"Planning to the {failed_at} pose failed ({move_service_result['message']}), trying the next yaw")
            rejected_places[f'planning failed to {failed_at}'] += 1
            continue
        message = f"Failed partway to the {failed_at} pose for '{target_name}': {move_service_result['message']}"
        ctx.get_logger().error(f"Execution failed partway while moving to {failed_at}: {move_service_result['message']}")
        if failed_at == 'hover':
            return ctx.fail_at_home(message)
        return _back_off(ctx, joints['hover'], message)
    else:
        reasons = ', '.join(f'{n} {reason}' for reason, n in rejected_places.items())
        ctx.get_logger().error(f"Dropoff candidate pre-check failed: no reachable place pose found for '{target_name}'. Tried {sum(rejected_places.values())} candidate(s) ({reasons})")
        return ctx.fail_at_home(f"No reachable place pose for '{target_name}': {sum(rejected_places.values())} tried ({reasons})")

    hover = joints['hover']
    ctx.get_logger().info(f"Releasing '{target_name}' at yaw {place_candidate.yaw_deg:+.0f} deg, "
                          f"tool at {np.round(place_candidate.release, 3).tolist()}")

    ctx.get_logger().info(f"Opening gripper to release '{target_name}'...")
    gripper_move_service_result = ctx.call_move_gripper_service(GRIPPER_OPEN)
    ctx.get_logger().debug(f"Gripper open result: {gripper_move_service_result}")
    if not gripper_move_service_result['success']:
        ctx.get_logger().error(f"Failed to open gripper to release '{target_name}': {gripper_move_service_result['message']}")
        return _back_off(ctx, hover, f"Failed to open gripper to release '{target_name}': {gripper_move_service_result['message']}")

    ctx.get_logger().info(f"Detaching '{target_name}' from robot in planning scene...")
    if not ctx.detach_object(target_name):
        ctx.get_logger().error(f"Failed to detach '{target_name}' from planning scene")
        return _back_off(ctx, hover, f"Released '{target_name}' but failed to detach it")

    q = place_candidate.object_rotation.as_quat()
    ctx.get_logger().debug(f"Updating object pose for '{target_name}' at bottom centre {np.round(place_candidate.object_centre, 3).tolist()}...")
    if not ctx.update_object_pose(target_name, *map(float, place_candidate.object_centre),
                                  {'x': q[0], 'y': q[1], 'z': q[2], 'w': q[3]}):
        ctx.get_logger().error(f"Failed to update pose for {target_name}, but continuing...")
    ctx.held_object = None
    ctx.held_grasp = None

    ctx.get_logger().info("Retreating: moving through back-off and up poses...")
    for waypoint in ('back-off', 'up'):
        target_pos = place_candidate.back if waypoint == 'back-off' else place_candidate.up
        ctx.get_logger().info(f"Moving to {waypoint} pose at {np.round(target_pos, 3).tolist()}...")
        ctx.get_logger().debug(f"Calling joint_move for {waypoint} with joints={np.round(joints[waypoint], 3).tolist()}")
        move_service_result = ctx.call_joint_move_service(joints[waypoint])
        if not move_service_result['success']:
            ctx.get_logger().error(f"Failed to move to {waypoint} pose: {move_service_result['message']}")
            return ctx.fail_at_home(f"Placed '{target_name}' {where} but failed to back away: {move_service_result['message']}")
        ctx.get_logger().debug(f"Successfully reached {waypoint} pose")
    ctx.get_logger().info(f"Successfully placed '{target_name}' {where}")
    return True, f"Placed '{target_name}' {where}"
