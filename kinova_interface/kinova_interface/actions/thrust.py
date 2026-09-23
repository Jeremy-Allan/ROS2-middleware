"""'thrust' recipe action (single-plane reach, see docs/push-motion-reference.md)."""
import math

from kinova_interface.actions.pour import POUR_DEFAULT_LIFT_HEIGHT
from kinova_interface.utils.geometry import resolve_direction_offset


def run(ctx, params):
    """Raise a held object and thrust it toward a destination -
    reuses the exact same single-plane mechanism as 'push' (see
    docs/push-motion-reference.md): joint_1 fixed per stage, wrist
    held level at _PLANAR_REACH_WRIST, only joint_2/joint_3
    (shoulder/elbow) ever change to reach a position.

    Assumes 'target' is already grasped - fails cleanly rather than
    guessing if it isn't - and expects it to have been grasped via
    pickup's grasp_style='side' (the same verified, level side grasp
    'pour' uses), so it starts out level and pointing outward.

    Sequence:
    1. Raise straight up from the grasp position, facing wherever it
       was originally resting (before pickup), to 'lift_height'
       above it. This also resets the wrist to _PLANAR_REACH_WRIST
       regardless of whatever specific orientation pickup's
       grasp_style='side' search happened to land on, so the result
       is level and facing outward the same way every time - not
       dependent on which grasp candidate was chosen.
    2. Spin to face the thrust direction, if it differs from where
       it's currently facing - joint_1 only; joint_2/joint_3/wrist
       stay exactly where the raise left them, so nothing about the
       object's orientation changes, only which way the arm points.
       There's no separate 'spin angle' parameter - an arbitrarily
       large turn (e.g. facing a destination on the opposite side of
       where it started) falls directly out of whichever
       destination/direction is given, the same way push's own
       facing does.
    3. Extend toward the destination/direction at that same height,
       joint_2/joint_3 only - exactly like push's own extend.

    Where to thrust it is either a named 'destination' object, or a
    'direction' ('forward'/'backward'/'left'/'right', relative to
    the object's own original resting bearing from the arm) plus a
    'distance' - both resolved from the object's pre-pickup
    registered position, the same as 'push'."""
    target_name = params.get('target')
    if not target_name:
        ctx.get_logger().error("thrust action requires 'target' naming the held object")
        return False
    if target_name != ctx.held_object:
        ctx.get_logger().error(f"Cannot thrust '{target_name}': held object is '{ctx.held_object}'")
        return False

    destination_name = params.get('destination')
    direction = params.get('direction')
    if not destination_name and not direction:
        ctx.get_logger().error("thrust action requires either 'destination' or 'direction'")
        return False

    target_info = ctx.get_object_info(target_name)
    if not target_info:
        ctx.get_logger().error(f"Could not resolve held object '{target_name}' for thrust")
        return False
    origin = target_info['pose']['position']

    # Fast by default (unlike push/pour's more careful pace) - a
    # thrust is meant to be a forceful, punchy motion; still
    # overridable via an explicit 'speed' if a slower one is wanted.
    motion_params = ctx.build_motion_params(params.get('speed', 1.0))
    lift_height = float(params.get('lift_height', POUR_DEFAULT_LIFT_HEIGHT))
    raise_z = origin['z'] + lift_height

    # 1. Solve + validate the raise pose - facing wherever the object
    # was originally resting, wrist reset to level regardless of
    # pickup's own grasp orientation. Solved (but not yet executed)
    # before the destination/direction is resolved below, since its
    # shoulder/elbow double as the seed for the dynamic thrust-distance
    # search, and are reused unchanged for the real raise move further
    # down - this is geometry only so far, the arm hasn't moved yet.
    face_yaw = math.atan2(origin['y'], origin['x'])
    raise_solved = ctx.solve_planar_reach(face_yaw, origin['x'], origin['y'], raise_z)
    if raise_solved is None or raise_solved[3] > ctx._PLANAR_REACH_MAX_ERROR:
        ctx.get_logger().error(f"Could not solve a planar reach to raise '{target_name}'")
        return False
    raise_shoulder, raise_elbow, _, _ = raise_solved
    raise_joints = [face_yaw, raise_shoulder, raise_elbow, *ctx._PLANAR_REACH_WRIST]
    if not ctx.check_joint_state_validity(raise_joints):
        ctx.get_logger().error(f"Raised pose for '{target_name}' is in collision")
        return False

    if destination_name:
        dest_info = ctx.get_object_info(destination_name)
        if not dest_info:
            ctx.get_logger().error(f"Could not resolve thrust destination '{destination_name}'")
            return False
        release_x, release_y = dest_info['pose']['position']['x'], dest_info['pose']['position']['y']
    else:
        requested_distance = params.get('distance')
        if requested_distance is not None:
            distance = float(requested_distance)
        else:
            # No explicit distance - find how far this specific object
            # position/direction/lift_height combination can actually
            # reach, rather than assuming a fixed default is reachable
            # (see _find_max_planar_reach_distance).
            distance = ctx._find_max_planar_reach_distance(origin, direction, raise_z, raise_shoulder, raise_elbow)
            if distance is None:
                ctx.get_logger().error(
                    f"No reachable thrust distance found for '{target_name}' in direction '{direction}'"
                )
                return False
        offset = resolve_direction_offset(origin['x'], origin['y'], direction, distance)
        if offset is None:
            ctx.get_logger().error(f"Unknown thrust direction '{direction}'")
            return False
        release_x, release_y = offset

    r = ctx.call_joint_move_service(raise_joints, motion_params=motion_params)
    if not (r and r['success']):
        ctx.get_logger().error(f"Failed to raise '{target_name}'")
        return False

    # 2. Spin to face the thrust direction - joint_1 only,
    # shoulder/elbow held exactly where the raise left them
    thrust_yaw = math.atan2(release_y, release_x)
    spin_joints = [thrust_yaw, raise_shoulder, raise_elbow, *ctx._PLANAR_REACH_WRIST]
    if not ctx.check_joint_state_validity(spin_joints):
        ctx.get_logger().error(f"Spin pose for '{target_name}' is in collision")
        return False

    r = ctx.call_joint_move_service(spin_joints, motion_params=motion_params)
    if not (r and r['success']):
        ctx.get_logger().error(f"Failed to spin to face the thrust direction for '{target_name}'")
        return False

    # 3. Extend toward the destination - same height, seeded at the
    # spin position so it stays a small, local adjustment
    extend_solved = ctx.solve_planar_reach(
        thrust_yaw, release_x, release_y, raise_z,
        seed_shoulder=raise_shoulder, seed_elbow=raise_elbow
    )
    if extend_solved is None or extend_solved[3] > ctx._PLANAR_REACH_MAX_ERROR:
        ctx.get_logger().error(f"Could not solve a planar reach to thrust '{target_name}' to its destination")
        return False
    extend_shoulder, extend_elbow, _, _ = extend_solved
    extend_joints = [thrust_yaw, extend_shoulder, extend_elbow, *ctx._PLANAR_REACH_WRIST]
    if not ctx.check_joint_state_validity(extend_joints):
        ctx.get_logger().error(f"Thrust end pose for '{target_name}' is in collision")
        return False

    r = ctx.call_joint_move_service(extend_joints, motion_params=motion_params)
    if not (r and r['success']):
        ctx.get_logger().error(f"Failed to thrust '{target_name}' to its destination")
        return False

    where = f"toward '{destination_name}'" if destination_name else direction
    ctx.get_logger().info(f"Thrust '{target_name}' {where}")
    return True
