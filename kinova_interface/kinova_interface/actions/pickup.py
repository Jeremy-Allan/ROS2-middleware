"""'pickup' recipe action, plus its side-grasp candidate generation."""
import math

from shape_msgs.msg import SolidPrimitive

from kinova_interface.utils.geometry import quaternion_to_euler


# A flat, level wrist (not pointing down) - the one part of a side
# grasp that's genuinely independent of which object it is.
SIDE_GRASP_ROLL = math.pi / 2.0
# Candidate yaw rotations relative to the object's own registered yaw -
# covers approaching aligned with, or perpendicular to, its own frame,
# from either side. Which of these is actually correct depends on the
# gripper's own closing-axis convention, which isn't assumed here -
# every candidate is verified for real (see verify_grasp_pose) rather
# than trusted from geometry alone.
SIDE_GRASP_YAW_OFFSETS = [0.0, math.pi / 2.0, -math.pi / 2.0, math.pi]
# A CYLINDER is radially symmetric - there's no "face" to align with the
# way BOX's 4 offsets (above) align with its own registered frame, every
# angle around its circumference is an equally valid grasp. 8 evenly
# spaced absolute yaws (not relative to the object's own yaw, which has
# no real meaning for a symmetric shape) gives verify_grasp_pose a much
# wider net to find one the arm can actually reach - important since
# cylinders (bottles, cups) need this to work reliably for pouring.
SIDE_GRASP_CYLINDER_YAWS = [math.radians(a) for a in range(0, 360, 45)]
# Small position nudges tried only if the object's exact center doesn't
# verify at any of the yaw offsets above - meters, along each world axis.
SIDE_GRASP_POSITION_OFFSETS = [0.02, -0.02, 0.04, -0.04]

def compute_side_grasp_candidates(ctx, target_info):
    """Generate candidate flat, side-on grasp poses for a BOX or
    CYLINDER object from its own registered shape and pose - not a
    fixed preset calibrated to one specific object/position (see
    docs/pour-motion-reference.md for why that didn't generalize).
    Each candidate still needs verifying (verify_grasp_pose) before
    being trusted - this generates plausible options, it doesn't
    guarantee any one of them is actually reachable/collision-free.
    Returns a list of (x, y, z, roll, pitch, yaw) tuples, cheapest/most
    likely first; [] if the shape isn't supported."""
    shape = target_info['shape']
    if shape['type'] not in (SolidPrimitive.BOX, SolidPrimitive.CYLINDER):
        ctx.get_logger().error(f"Side grasp only supports BOX/CYLINDER shapes currently (got shape type {shape['type']})")
        return []

    pos = target_info['pose']['position']

    if shape['type'] == SolidPrimitive.CYLINDER:
        yaw_candidates = SIDE_GRASP_CYLINDER_YAWS
    else:
        orient = target_info['pose']['orientation']
        _, _, object_yaw = quaternion_to_euler(orient['x'], orient['y'], orient['z'], orient['w'])
        yaw_candidates = [object_yaw + offset for offset in SIDE_GRASP_YAW_OFFSETS]

    candidates = []
    # Pass 1: the object's exact center, at each candidate yaw - the
    # cheapest and most likely to work.
    for yaw in yaw_candidates:
        candidates.append((pos['x'], pos['y'], pos['z'], SIDE_GRASP_ROLL, 0.0, yaw))

    # Pass 2: only if none of those verify - small offsets along each
    # world axis, at every candidate yaw again.
    for offset in SIDE_GRASP_POSITION_OFFSETS:
        for yaw in yaw_candidates:
            candidates.append((pos['x'] + offset, pos['y'], pos['z'], SIDE_GRASP_ROLL, 0.0, yaw))
            candidates.append((pos['x'], pos['y'] + offset, pos['z'], SIDE_GRASP_ROLL, 0.0, yaw))

    return candidates


def run(ctx, params):
    """Default behaviour is unchanged: unconstrained orientation at the
    object's registered center (forcing one can make an
    otherwise-reachable approach infeasible for the planner, same
    caveat as 'push').

    'grasp_style': 'side' switches to a flat, side-on grasp instead -
    computed from the object's own shape/pose (BOX or CYLINDER
    currently), not a fixed preset for one specific object, and each candidate
    pose is verified reachable/collision-free (via /compute_ik) before
    being trusted, rather than assumed correct from geometry alone.
    See compute_side_grasp_candidates/verify_grasp_pose and
    docs/pour-motion-reference.md for why that verification step
    matters - a manually-demonstrated pose used earlier turned out to
    have never actually been validated this way.

    'orientation'/'grasp_offset' remain available for a fully manual
    override (an explicit preset name, plus an x/y/z shift off the
    object's center) when 'grasp_style' isn't given."""
    target_name = params['target']
    open_pos = float(params.get('open_position', 0.0))
    close_pos = float(params.get('close_position', 0.8))

    if params.get('grasp_style') == 'side':
        target_info = ctx.get_object_info(target_name)
        if not target_info:
            return False
        candidates = compute_side_grasp_candidates(ctx, target_info)
        if not candidates:
            return False
        chosen = next((c for c in candidates if ctx.verify_grasp_pose(*c)), None)
        if chosen is None:
            ctx.get_logger().error(f"No valid side-grasp pose found for '{target_name}'")
            return False
        target_x, target_y, target_z, roll, pitch, yaw = chosen
        has_orientation = True
    else:
        coords = ctx.get_static_object_coords(target_name)
        if not coords:
            return False

        orientation = ctx.resolve_orientation(params.get('orientation'))
        if orientation is None:
            ctx.get_logger().error(f"Unknown orientation preset '{params.get('orientation')}' for pickup")
            return False
        has_orientation, roll, pitch, yaw = orientation

        grasp_offset = params.get('grasp_offset') or {}
        target_x = coords['x'] + float(grasp_offset.get('x', 0.0))
        target_y = coords['y'] + float(grasp_offset.get('y', 0.0))
        target_z = coords['z'] + float(grasp_offset.get('z', 0.0))

    # 1. Open gripper before moving
    rg = ctx.call_move_gripper_service(open_pos)
    if not (rg and rg['success']):
        ctx.get_logger().error('Failed to open gripper for pickup')
        return False

    # 2. Descend to the chosen approach pose
    r = ctx.call_move_service(target_x, target_y, target_z, has_orientation, roll, pitch, yaw)
    if not (r and r['success']):
        ctx.get_logger().error('Failed to move to object position')
        return False

    # 3. Close gripper
    rg = ctx.call_move_gripper_service(close_pos)
    if not (rg and rg['success']):
        return False

    # 4. Remove object from planning scene (attach)
    if not ctx.attach_object(target_name):
        ctx.get_logger().error("Failed to attach object after pickup")
        return False

    ctx.held_object = target_name
    ctx.get_logger().info(f"Picked up '{target_name}'")
    return True
