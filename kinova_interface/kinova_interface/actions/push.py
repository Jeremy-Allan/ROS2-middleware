"""'push' recipe action (see docs/push-motion-reference.md)."""
import math
from typing import TYPE_CHECKING

from kinova_interface.utils.geometry import resolve_direction_offset

# Only for the ctx type hint (editor go-to-definition). Not imported at runtime,
# because arm_actions imports this module and that would be circular.
if TYPE_CHECKING:
    from kinova_interface.actions.arm_actions import ArmActions


def run(ctx: 'ArmActions', params: dict) -> bool:
    """Slide an object to a destination by sustained contact, without
    ever grasping or lifting it - captured from a manual RViz
    demonstration (see docs/push-motion-reference.md): face the
    object (joint_1 only), then move only the shoulder/elbow
    (joint_2/joint_3) - the wrist and base stay fixed the whole time,
    so the entire push happens in a single vertical plane, not a
    general 6-DOF reach.

    Both the contact pose and the extended end pose are solved with
    solve_planar_reach (forward-kinematics-based, not IK) and checked
    collision-free with check_joint_state_validity *before* any real
    motion happens - unlike an earlier version of this, which
    searched a handful of differently-rotated 6-DOF grasp-style
    candidates via general IK and had to actually attempt (and
    sometimes retreat from) each one in turn. That approach worked
    but visibly produced arbitrary, sideways-looking approaches whose
    IK solutions happened to be reachable in some direction unrelated
    to the object's own bearing; this one only ever considers the
    single, deliberate plane facing the object, matching the manual
    demonstration this was built from (see
    docs/push-motion-reference.md for the full history, including why
    a general IK/6-DOF search and a plan-only pre-check were both
    tried and ruled out first).

    Push only ever extends an object further from the arm's own base
    - it's not designed to drag one back in.

    Sustained contact is deliberate here (unlike every other action),
    so a normal collision-aware plan would otherwise reject it, or
    silently route around the object instead of actually pushing it -
    set_collision_allowed temporarily exempts just this one object for
    the duration, and is always reverted in a 'finally' block even if
    the push fails partway through.

    Where to push it is either a named 'destination' object, or a
    'direction' ('forward'/'backward'/'left'/'right', relative to the
    object's own original bearing from the arm) plus a 'distance'."""
    target_name = params.get('target')
    destination_name = params.get('destination')
    direction = params.get('direction')
    if not target_name:
        ctx.get_logger().error("push action requires 'target'")
        return False
    if not destination_name and not direction:
        ctx.get_logger().error("push action requires either 'destination' or 'direction'")
        return False

    target_info = ctx.get_object_info(target_name)
    if not target_info:
        ctx.get_logger().error(f"Could not resolve push target '{target_name}'")
        return False
    origin = target_info['pose']['position']

    # Face the object - joint_1 fixed for the whole push, the single
    # plane every subsequent move (including the dynamic distance
    # search below) stays within.
    base_yaw = math.atan2(origin['y'], origin['x'])

    # Contact with the target object itself is the entire point of a
    # push, not a collision to avoid - permitted up front, before the
    # validity checks below, not just before the real motion. A
    # bigger object (e.g. 'box', ~107x75mm, vs. push_block's small
    # ~60x60mm) genuinely needs the gripper to overlap it to reach its
    # exact registered center, which the pre-check would otherwise
    # (correctly) reject - verified directly that the only contact at
    # that pose is (gripper, target object), nothing else (see
    # docs/push-motion-reference.md), so exempting just this object
    # doesn't hide a real problem with anything else in the scene.
    # This intentionally doesn't try to approach a bigger object's
    # near face instead - see docs/push-motion-reference.md for why
    # that's a known, accepted limitation for now: it may push a
    # larger object slightly off from how a human would naturally
    # grip it, favoring keeping the single-plane motion simple.
    if not ctx.set_collision_allowed(target_name, True):
        ctx.get_logger().error(f"Failed to allow contact with '{target_name}' for push")
        return False

    try:
        contact = ctx.solve_planar_reach(base_yaw, origin['x'], origin['y'], origin['z'])
        if contact is None or contact[3] > ctx._PLANAR_REACH_MAX_ERROR:
            ctx.get_logger().error(f"Could not solve a planar reach to '{target_name}'")
            return False
        contact_shoulder, contact_elbow, _, _ = contact
        contact_joints = [base_yaw, contact_shoulder, contact_elbow, *ctx._PLANAR_REACH_WRIST]
        if not ctx.check_joint_state_validity(contact_joints):
            ctx.get_logger().error(f"Push contact pose for '{target_name}' is in collision")
            return False

        if destination_name:
            dest_info = ctx.get_object_info(destination_name)
            if not dest_info:
                ctx.get_logger().error(f"Could not resolve push destination '{destination_name}'")
                return False
            release_x, release_y = dest_info['pose']['position']['x'], dest_info['pose']['position']['y']
        else:
            requested_distance = params.get('distance')
            if requested_distance is not None:
                distance = float(requested_distance)
            else:
                # No explicit distance - find how far this specific
                # object position/direction combination can actually
                # reach, rather than assuming a fixed default is
                # reachable (see _find_max_planar_reach_distance).
                # Push never reorients (fixed_yaw=base_yaw) - unlike
                # thrust, there's no separate spin step to search over.
                distance = ctx._find_max_planar_reach_distance(
                    origin, direction, origin['z'], contact_shoulder, contact_elbow, fixed_yaw=base_yaw
                )
                if distance is None:
                    ctx.get_logger().error(
                        f"No reachable push distance found for '{target_name}' in direction '{direction}'"
                    )
                    return False
            offset = resolve_direction_offset(origin['x'], origin['y'], direction, distance)
            if offset is None:
                ctx.get_logger().error(f"Unknown push direction '{direction}'")
                return False
            release_x, release_y = offset

        # Same height, same plane, seeded at the contact solution so
        # the extend stays a small, local adjustment rather than
        # jumping to an unrelated configuration.
        extend = ctx.solve_planar_reach(
            base_yaw, release_x, release_y, origin['z'],
            seed_shoulder=contact_shoulder, seed_elbow=contact_elbow
        )
        if extend is None or extend[3] > ctx._PLANAR_REACH_MAX_ERROR:
            ctx.get_logger().error(f"Could not solve a planar reach to the push destination for '{target_name}'")
            return False
        extend_shoulder, extend_elbow, _, _ = extend
        extend_joints = [base_yaw, extend_shoulder, extend_elbow, *ctx._PLANAR_REACH_WRIST]
        if not ctx.check_joint_state_validity(extend_joints):
            ctx.get_logger().error(f"Push end pose for '{target_name}' is in collision")
            return False

        motion_params = ctx.build_motion_params(params.get('speed'))
        close_pos = float(params.get('close_position', 0.75))

        # 1. Open the gripper before approaching
        rg = ctx.call_move_gripper_service(0.0)
        if not (rg and rg['success']):
            ctx.get_logger().error('Failed to open gripper before push approach')
            return False

        # 2. Move to the contact pose
        r = ctx.call_joint_move_service(contact_joints, motion_params=motion_params)
        if not (r and r['success']):
            ctx.get_logger().error('Failed to approach push target')
            return False

        # 3. Close the gripper onto it
        rg = ctx.call_move_gripper_service(close_pos)
        if not (rg and rg['success']):
            ctx.get_logger().error('Failed to set gripper for push')
            return False

        # 4. Extend - shoulder/elbow only, same plane
        r = ctx.call_joint_move_service(extend_joints, motion_params=motion_params)
        if not (r and r['success']):
            ctx.get_logger().error('Failed to push to destination')
            return False
    finally:
        ctx.set_collision_allowed(target_name, False)

    # It moved by contact, not attachment - update its known position.
    orient = target_info['pose']['orientation']
    if not ctx.update_object_pose(target_name, release_x, release_y, origin['z'], orient):
        ctx.get_logger().error(f"Failed to update pose for {target_name}, but continuing...")

    where = f"to '{destination_name}'" if destination_name else f"{direction}"
    ctx.get_logger().info(f"Pushed '{target_name}' {where}")
    return True
