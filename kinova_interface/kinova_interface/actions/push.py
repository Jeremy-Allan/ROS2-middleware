"""'push' recipe action (see docs/technical/motion-and-actions/push.md)."""
import math
from typing import TYPE_CHECKING

from kinova_interface.actions.motion import MotionUnavailable
from kinova_interface.utils.geometry import resolve_direction_offset
from kinova_interface.utils.robot import GRIPPER_OPEN

# Only for the ctx type hint (editor go-to-definition). Not imported at runtime,
# because arm_actions imports this module and that would be circular.
if TYPE_CHECKING:
    from kinova_interface.actions.arm_actions import ArmActions


def run(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    """Slide an object to a destination by sustained contact, without
    ever grasping or lifting it - captured from a manual RViz
    demonstration (see docs/technical/motion-and-actions/push.md): face the
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
    docs/technical/motion-and-actions/push.md for the full history, including why
    a general IK/6-DOF search and a plan-only pre-check were both
    tried and ruled out first).

    Push only ever extends an object further from the arm's own base
    - it's not designed to drag one back in.

    Sustained contact is deliberate here (unlike every other action),
    so a normal collision-aware plan would otherwise reject it, or
    silently route around the object instead of actually pushing it -
    set_collision_allowed temporarily exempts just this one object for
    the whole approach/extend/retreat sequence, and is always reverted
    in a 'finally' block even if the push fails partway through.

    Once the gripper closes on the object it's also attached (like
    pickup/dropoff/throw), so its position in the planning scene - and
    in RViz - tracks the real arm motion through the extend, rather
    than staying frozen at its pre-push pose the whole time. It's
    opened and detached again (same open -> detach -> update-pose
    ordering as dropoff/throw) before the arm retreats back to the
    contact pose, which only makes sense because the object has by
    then genuinely relocated out of the way.

    Being attached means the object is now rigidly carried with the
    gripper through the extend's joint-space interpolated path, which -
    unlike the single solved contact/extend endpoints - isn't
    guaranteed to stay perfectly level in between, and can dip the
    object fractionally into whatever it's resting on (the table).
    Same reasoning as pickup's lift: touching()/allow_contact()
    temporarily permits just that one contact for the extend, reverted
    immediately after, rather than it being flagged a real collision.

    Where to push it is either a named 'destination' object, or a
    'direction' ('forward'/'backward'/'left'/'right', relative to the
    object's own original bearing from the arm) plus a 'distance'."""
    target_name = params.get('target')
    destination_name = params.get('destination')
    direction = params.get('direction')
    if not target_name:
        return False, "push action requires 'target'"
    if not destination_name and not direction:
        return False, "push action requires either 'destination' or 'direction'"

    target_info = ctx.get_object_info(target_name)
    if not target_info:
        return False, f"Could not resolve push target '{target_name}'"
    origin = target_info['pose']['position']

    # Face the object - joint_1 fixed for the whole push, the single
    # plane every subsequent move (including the dynamic distance
    # search below) stays within.
    base_yaw = math.atan2(origin['y'], origin['x'])

    # Contact with the target object itself is the entire point of a
    # push, not a collision to avoid - permitted up front, before the
    # validity checks below, not just before the real motion. A
    # bigger object (e.g. blue_cube, ~107x43mm, vs. push_block's small
    # ~60x60mm) genuinely needs the gripper to overlap it to reach its
    # exact registered center, which the pre-check would otherwise
    # (correctly) reject - verified directly that the only contact at
    # that pose is (gripper, target object), nothing else (see
    # docs/technical/motion-and-actions/push.md), so exempting just this object
    # doesn't hide a real problem with anything else in the scene.
    # This intentionally doesn't try to approach a bigger object's
    # near face instead - see docs/technical/motion-and-actions/push.md for why
    # that's a known, accepted limitation for now: it may push a
    # larger object slightly off from how a human would naturally
    # grip it, favoring keeping the single-plane motion simple.
    if not ctx.set_collision_allowed(target_name, True):
        return False, f"Failed to allow contact with '{target_name}' for push"

    try:
        contact = ctx.solve_planar_reach(base_yaw, origin['x'], origin['y'], origin['z'])
        if contact is None or contact[3] > ctx._PLANAR_REACH_MAX_ERROR:
            return False, f"Could not solve a planar reach to '{target_name}'"
        contact_shoulder, contact_elbow, _, _ = contact
        contact_joints = [base_yaw, contact_shoulder, contact_elbow, *ctx._PLANAR_REACH_WRIST]
        if not ctx.check_joint_state_validity(contact_joints):
            return False, f"Push contact pose for '{target_name}' is in collision"

        if destination_name:
            dest_info = ctx.get_object_info(destination_name)
            if not dest_info:
                return False, f"Could not resolve push destination '{destination_name}'"
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
                    return False, f"No reachable push distance found for '{target_name}' in direction '{direction}'"
            offset = resolve_direction_offset(origin['x'], origin['y'], direction, distance)
            if offset is None:
                return False, f"Unknown push direction '{direction}'"
            release_x, release_y = offset

        # Same height, same plane, seeded at the contact solution so
        # the extend stays a small, local adjustment rather than
        # jumping to an unrelated configuration.
        extend = ctx.solve_planar_reach(
            base_yaw, release_x, release_y, origin['z'],
            seed_shoulder=contact_shoulder, seed_elbow=contact_elbow
        )
        if extend is None or extend[3] > ctx._PLANAR_REACH_MAX_ERROR:
            return False, f"Could not solve a planar reach to the push destination for '{target_name}'"
        extend_shoulder, extend_elbow, _, _ = extend
        extend_joints = [base_yaw, extend_shoulder, extend_elbow, *ctx._PLANAR_REACH_WRIST]
        if not ctx.check_joint_state_validity(extend_joints):
            return False, f"Push end pose for '{target_name}' is in collision"

        motion_params = ctx.build_motion_params(params.get('speed'))
        close_pos = float(params.get('close_position', 0.75))

        # 1. Open the gripper before approaching
        rg = ctx.call_move_gripper_service(GRIPPER_OPEN)
        if not rg['success']:
            return False, f"Failed to open gripper before push approach: {rg['message']}"

        # 2. Move to the contact pose
        r = ctx.call_joint_move_service(contact_joints, motion_params=motion_params)
        if not r['success']:
            return False, f"Failed to approach push target: {r['message']}"

        # 3. Close the gripper onto it, then attach it so its position in
        # the planning scene (and RViz) tracks the real arm motion
        # through the extend below, instead of staying frozen at its
        # pre-push pose for the entire push the way a bare collision
        # exemption would leave it.
        rg = ctx.call_move_gripper_service(close_pos)
        if not rg['success']:
            return False, f"Failed to set gripper for push: {rg['message']}"
        if not ctx.attach_object(target_name):
            return False, f"Failed to attach '{target_name}' for push"

        # Now attached, the object's geometry is rigidly carried with the
        # tool_frame through the joint-space interpolated extend below -
        # which, unlike the single solved endpoint, isn't guaranteed to
        # stay exactly level, so it can dip the object fractionally into
        # whatever it's resting on (the table). Same reasoning as
        # pickup's lift: allow that one specific contact for the
        # duration, rather than it being flagged as a real collision.
        try:
            resting_on = ctx.motion.touching(target_name)
        except MotionUnavailable as error:
            return False, f"Attached '{target_name}' but can't check what it's resting on: {error}"
        if resting_on and not ctx.motion.allow_contact(target_name, resting_on, True):
            return False, f"Attached '{target_name}' but couldn't allow its contact with {', '.join(resting_on)}"

        # 4. Extend - shoulder/elbow only, same plane
        try:
            r = ctx.call_joint_move_service(extend_joints, motion_params=motion_params)
        finally:
            if resting_on and not ctx.motion.allow_contact(target_name, resting_on, False):
                ctx.get_logger().warn(f"Couldn't restore collision checking between '{target_name}' and {', '.join(resting_on)}")
        if not r['success']:
            return False, f"Failed to push to destination: {r['message']}"

        # 5. Open the gripper, detach (mirrors dropoff/throw's release
        # ordering: open -> detach -> update the registered pose), then
        # retreat to the contact pose before the collision exemption
        # below is revoked. Retreating is safe now because the object
        # has genuinely relocated away from contact_joints during the
        # attached extend above - push only ever extends an object
        # further away, never back towards the arm (see docstring
        # above) - rather than relying on it having silently stayed put
        # the whole time, which left the gripper retreating straight
        # back into it.
        rg = ctx.call_move_gripper_service(GRIPPER_OPEN)
        if not rg['success']:
            return False, f"Pushed '{target_name}' but failed to open the gripper to retreat: {rg['message']}"
        if not ctx.detach_object(target_name):
            return False, f"Pushed '{target_name}' but failed to detach it afterward"
        orient = target_info['pose']['orientation']
        if not ctx.update_object_pose(target_name, release_x, release_y, origin['z'], orient):
            ctx.get_logger().error(f"Failed to update pose for {target_name}, but continuing...")
        r = ctx.call_joint_move_service(contact_joints, motion_params=motion_params)
        if not r['success']:
            return False, f"Pushed '{target_name}' but failed to retreat afterward: {r['message']}"
    finally:
        ctx.set_collision_allowed(target_name, False)

    where = f"to '{destination_name}'" if destination_name else f"{direction}"
    return True, f"Pushed '{target_name}' {where}"
