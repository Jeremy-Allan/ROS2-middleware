"""'pour' recipe action (see docs/pour-motion-reference.md)."""
import math
import time

from kinova_interface.utils.geometry import resolve_direction_offset


# Default tilt: matches the ~135 degree joint_6 delta captured in the
# manual RViz demo this was built from - see docs/pour-motion-reference.md.
POUR_DEFAULT_TILT_ANGLE = math.radians(135)
POUR_DEFAULT_LIFT_HEIGHT = 0.14  # meters; demo measured ~0.137m

def run(ctx, params):
    """Lift a held object and tip it to pour, then return level -
    directly above wherever it currently is by default, or above a
    'destination'/'direction' first if one is given. Assumes 'target'
    is already grasped - fails cleanly rather than guessing if it
    isn't - and, importantly, assumes it was grasped in a known,
    level orientation (e.g. via pickup's 'side_grasp_flat' preset),
    which lift/transit below then preserve exactly rather than assume
    or recompute.

    'destination'/'direction' are optional, unlike push/thrust/throw:
    a pour doesn't inherently need to go anywhere - pouring is the
    point, not the travel - so with neither given this skips the
    horizontal move entirely and pours right where the object already
    was. Only when a destination/direction actually is given does it
    first move to hover above that point before tilting, exactly the
    same as before.

    This replaces the previous design, which applied a relative
    orientation delta on top of whatever arbitrary orientation an
    unconstrained pickup happened to produce - workable in principle,
    but only if the starting orientation is actually known, which it
    wasn't. See docs/pour-motion-reference.md for the manually-driven
    RViz demonstration this sequence is built from, and why.

    The actual tilt is a pure joint-space delta on joint_6 alone (like
    'throw's wind-up/fling), not a Cartesian orientation change -
    deliberately: 'side_grasp_flat' holds the wrist at roughly a 90
    degree roll, and composing a Cartesian relative orientation delta
    from there hits exactly the asin()-based gimbal-lock-adjacent
    coupling that handle_relative_move's own docstring already warns
    about (see hardware_interface_client.py) - which is what produced
    the unreachable target that made the original design fail."""
    target_name = params.get('target')
    if not target_name:
        ctx.get_logger().error("pour action requires 'target' naming the held object")
        return False
    if target_name != ctx.held_object:
        ctx.get_logger().error(f"Cannot pour '{target_name}': held object is '{ctx.held_object}'")
        return False

    destination_name = params.get('destination')
    direction = params.get('direction')

    target_info = ctx.get_object_info(target_name)
    if not target_info:
        ctx.get_logger().error(f"Could not resolve held object '{target_name}' for pour")
        return False
    origin = target_info['pose']['position']

    if destination_name:
        dest_info = ctx.get_object_info(destination_name)
        if not dest_info:
            ctx.get_logger().error(f"Could not resolve pour destination '{destination_name}'")
            return False
        release_x, release_y = dest_info['pose']['position']['x'], dest_info['pose']['position']['y']
    elif direction:
        distance = float(params.get('distance', 0.3))
        offset = resolve_direction_offset(origin['x'], origin['y'], direction, distance)
        if offset is None:
            ctx.get_logger().error(f"Unknown pour direction '{direction}'")
            return False
        release_x, release_y = offset
    else:
        # No destination/direction - pour stays exactly where the
        # object already was, so there's nothing to move sideways to.
        release_x, release_y = origin['x'], origin['y']

    motion_params = ctx.build_motion_params(params.get('speed'))

    # 1. Lift straight up from the grasp height, holding whatever
    # orientation it was grasped in exactly unchanged (a zero-delta
    # relative move - orientation is preserved, never recomputed)
    lift_height = float(params.get('lift_height', POUR_DEFAULT_LIFT_HEIGHT))
    r = ctx.call_relative_move_service(0.0, 0.0, lift_height, True, 0.0, 0.0, 0.0, motion_params)
    if not (r and r['success']):
        ctx.get_logger().error('Failed to lift for pour')
        return False

    # 2. Move horizontally to hover above the destination, at that same
    # lifted height - orientation still untouched. Skipped entirely
    # (no real move issued) when neither destination nor direction was
    # given, since release_x/release_y then equal origin exactly.
    dx, dy = release_x - origin['x'], release_y - origin['y']
    if dx != 0.0 or dy != 0.0:
        r = ctx.call_relative_move_service(dx, dy, 0.0, True, 0.0, 0.0, 0.0, motion_params)
        if not (r and r['success']):
            ctx.get_logger().error('Failed to move above pour destination')
            return False

    # 3. Tilt: a pure joint-space delta on joint_6 alone (see docstring)
    tilt_angle = float(params.get('tilt_angle', POUR_DEFAULT_TILT_ANGLE))
    tilt_delta = [0.0, 0.0, 0.0, 0.0, 0.0, tilt_angle]
    r = ctx.call_joint_move_service(tilt_delta, motion_params=motion_params, relative=True)
    if not (r and r['success']):
        ctx.get_logger().error('Failed to tilt for pour')
        return False

    # 4. Hold the tilt so contents can pour out
    dwell = float(params.get('duration', 1.5))
    time.sleep(dwell)

    # 5. Rotate back level - the exact negated delta
    untilt_delta = [0.0, 0.0, 0.0, 0.0, 0.0, -tilt_angle]
    r = ctx.call_joint_move_service(untilt_delta, motion_params=motion_params, relative=True)
    if not (r and r['success']):
        ctx.get_logger().error('Failed to return to level after pour')
        return False

    if destination_name:
        where = f"toward '{destination_name}'"
    elif direction:
        where = direction
    else:
        where = "in place"
    ctx.get_logger().info(f"Poured '{target_name}' {where}")
    return True
