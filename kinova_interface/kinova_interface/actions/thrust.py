"""'thrust' recipe action (single-plane reach, see docs/technical/motion-and-actions/push.md)."""
import math
from typing import TYPE_CHECKING

from kinova_interface.actions.pour import POUR_DEFAULT_LIFT_HEIGHT
from kinova_interface.utils.geometry import DIRECTION_ROTATIONS, resolve_direction_offset

# Only for the ctx type hint (editor go-to-definition). Not imported at runtime,
# because arm_actions imports this module and that would be circular.
if TYPE_CHECKING:
    from kinova_interface.actions.arm_actions import ArmActions

# How close to the base the held object is drawn in before striking, so the
# extend always has a real stroke. Level reaches at thrust height collide
# with the arm itself inside ~0.25 m; 0.30 m leaves room for the object.
THRUST_PULLBACK_RADIUS = 0.30
# If the pull-back pose is blocked, step outward by this much at a time,
# never past where the object started.
THRUST_PULLBACK_STEP = 0.05


def run(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    """Raise a held object and thrust it toward a destination -
    reuses the exact same single-plane mechanism as 'push' (see
    docs/technical/motion-and-actions/push.md): joint_1 fixed per stage, wrist
    held level at _PLANAR_REACH_WRIST, only joint_2/joint_3
    (shoulder/elbow) ever change to reach a position.

    Assumes 'target' is already grasped - fails cleanly rather than
    guessing if it isn't - and expects it to have been grasped via
    pickup's grasp_style='side' (the same verified, level side grasp
    'pour' uses), so it starts out level and pointing outward.

    Sequence:
    1. Pull back: raise the object to 'lift_height' above where it
       rested and draw it in toward the base, along its own bearing, to
       THRUST_PULLBACK_RADIUS (stepping outward if that's blocked). This
       also resets the wrist to _PLANAR_REACH_WRIST, so the object is
       level and facing outward the same way every time.
    2. Spin to face the thrust bearing - joint_1 only, shoulder/elbow/
       wrist exactly where the pull-back left them.
    3. Extend along that bearing at the same height, joint_2/joint_3
       only, as far as the arm can reach (_find_max_planar_reach_distance)
       - it doesn't stop at the destination, so there's always a full
       stroke even when the destination is no further out than the
       object started. The extend (and the reach search) goes straight
       through any other object, so it can actually stab the destination:
       every object but the held one is exempt from collision checking
       for it (ArmActions.exempt_from_collisions), and the held object may
       touch them too (motion.allow_contact), all restored afterwards even
       on failure. The held object itself stays checked against the table.
    4. Retreat back along the same line to the spin pose, still exempt,
       so the arm isn't left inside whatever it stabbed; collision
       checking is only restored once it's clear. The pull-back and spin still avoid everything;
       the table and walls are always checked. Objects that get hit
       don't move in the planning scene - there's no physics.

    The bearing comes from a named 'destination' object, or a
    'direction' ('forward'/'backward'/'left'/'right', relative to the
    object's own original resting bearing from the arm), the same as
    'push'. An explicit 'distance' (direction only) is still honoured
    as the end point; without one the reach limit is used.

    Everything is solved before the arm moves, except the extend's own
    solve after the spin (seeded from the pull-back pose)."""
    # No target means the held object
    target_name = params.get('target') or ctx.held_object
    if not target_name:
        return False, "thrust action requires a held object, but nothing is currently held"
    if target_name != ctx.held_object:
        return False, f"Cannot thrust '{target_name}': held object is '{ctx.held_object}'"

    destination_name = params.get('destination')
    direction = params.get('direction')
    if not destination_name and not direction:
        return False, "thrust action requires either 'destination' or 'direction'"

    target_info = ctx.get_object_info(target_name)
    if not target_info:
        return False, f"Could not resolve held object '{target_name}' for thrust"
    origin = target_info['pose']['position']

    # Fast by default (unlike push/pour's more careful pace) - a
    # thrust is meant to be a forceful, punchy motion; still
    # overridable via an explicit 'speed' if a slower one is wanted.
    motion_params = ctx.build_motion_params(params.get('speed', 1.0))
    lift_height = float(params.get('lift_height', POUR_DEFAULT_LIFT_HEIGHT))
    raise_z = origin['z'] + lift_height

    # 1. Pull-back pose: in toward the base along the object's own bearing
    face_yaw = math.atan2(origin['y'], origin['x'])
    object_radius = math.hypot(origin['x'], origin['y'])
    pulled = _solve_pullback(ctx, face_yaw, object_radius, raise_z)
    if pulled is None:
        return False, f"Could not solve a pulled-back raise for '{target_name}'"
    pull_radius, raise_shoulder, raise_elbow = pulled
    raise_joints = [face_yaw, raise_shoulder, raise_elbow, *ctx._PLANAR_REACH_WRIST]

    # Bearing to thrust along, and how far
    if destination_name:
        dest_info = ctx.get_object_info(destination_name)
        if not dest_info:
            return False, f"Could not resolve thrust destination '{destination_name}'"
        thrust_yaw = math.atan2(dest_info['pose']['position']['y'], dest_info['pose']['position']['x'])
        release = None
    elif direction not in DIRECTION_ROTATIONS:
        return False, f"Unknown thrust direction '{direction}'"
    elif params.get('distance') is not None:
        release = resolve_direction_offset(origin['x'], origin['y'], direction, float(params['distance']))
        thrust_yaw = math.atan2(release[1], release[0])
    else:
        # The object's own bearing, turned by the direction ('backward' = the far side of the base)
        turned = face_yaw + DIRECTION_ROTATIONS[direction]
        thrust_yaw = math.atan2(math.sin(turned), math.cos(turned))  # wrapped to +-pi for joint_1
        release = None

    # The stab goes straight through other objects (so it can actually hit
    # what it's aimed at): they're exempt from collision checking for the
    # reach search and the extend, never for the pull-back or spin.
    object_names = ctx.exemptible_objects()
    if object_names is None:
        return False, f"Couldn't list the scene's objects to thrust '{target_name}' through them"
    exempt = [name for name in object_names if name != target_name]

    if release is None:
        # To the reach limit along the bearing, from the pull-back point
        start_x, start_y = pull_radius * math.cos(thrust_yaw), pull_radius * math.sin(thrust_yaw)
        with ctx.passing_through(target_name, exempt, 'thrust') as error:
            if error:
                return False, error
            reach = ctx._find_max_planar_reach_distance(
                {'x': start_x, 'y': start_y}, 'forward', raise_z, raise_shoulder, raise_elbow, fixed_yaw=thrust_yaw)
        if reach is None:
            return False, f"No reachable thrust distance found for '{target_name}'"
        release = (start_x + reach * math.cos(thrust_yaw), start_y + reach * math.sin(thrust_yaw))
    release_x, release_y = release

    r = ctx.call_joint_move_service(raise_joints, motion_params=motion_params)
    if not r['success']:
        return False, f"Failed to pull back '{target_name}': {r['message']}"

    # 2. Spin to face the thrust direction - joint_1 only,
    # shoulder/elbow held exactly where the raise left them
    spin_joints = [thrust_yaw, raise_shoulder, raise_elbow, *ctx._PLANAR_REACH_WRIST]
    if not ctx.check_joint_state_validity(spin_joints):
        return False, f"Spin pose for '{target_name}' is in collision"

    r = ctx.call_joint_move_service(spin_joints, motion_params=motion_params)
    if not r['success']:
        return False, f"Failed to spin to face the thrust direction for '{target_name}': {r['message']}"

    # 3. Extend - same height, seeded at the spin position so it stays a
    # small, local adjustment - straight through any other object
    with ctx.passing_through(target_name, exempt, 'thrust') as error:
        if error:
            return False, error
        extend_solved = ctx.solve_planar_reach(
            thrust_yaw, release_x, release_y, raise_z,
            seed_shoulder=raise_shoulder, seed_elbow=raise_elbow
        )
        if extend_solved is None or extend_solved[3] > ctx._PLANAR_REACH_MAX_ERROR:
            return False, f"Could not solve a planar reach to thrust '{target_name}' to its destination"
        extend_shoulder, extend_elbow, _, _ = extend_solved
        extend_joints = [thrust_yaw, extend_shoulder, extend_elbow, *ctx._PLANAR_REACH_WRIST]
        if not ctx.check_joint_state_validity(extend_joints):
            return False, f"Thrust end pose for '{target_name}' is in collision"

        r = ctx.call_joint_move_service(extend_joints, motion_params=motion_params)
        if not r['success']:
            return False, f"Failed to thrust '{target_name}' to its destination: {r['message']}"

        # 4. Retreat back along the same line to the spin pose (checked clear
        # before any exemptions), while still exempt, so the arm isn't left
        # inside whatever it stabbed once collision checking is restored.
        r = ctx.call_joint_move_service(spin_joints, motion_params=motion_params)
        if not r['success']:
            return False, f"Thrust '{target_name}' but failed to retreat afterward: {r['message']}"

    where = f"toward '{destination_name}'" if destination_name else direction
    return True, f"Thrust '{target_name}' {where}"


def _solve_pullback(ctx: 'ArmActions', face_yaw: float, object_radius: float, height: float):
    """The level, collision-free pull-back pose closest to
    THRUST_PULLBACK_RADIUS along face_yaw, stepping outward but never past
    object_radius. Returns (radius, shoulder, elbow), or None."""
    radius = min(THRUST_PULLBACK_RADIUS, object_radius)
    while radius <= object_radius + 1e-9:
        solved = ctx.solve_planar_reach(face_yaw, radius * math.cos(face_yaw), radius * math.sin(face_yaw), height)
        if solved is not None and solved[3] <= ctx._PLANAR_REACH_MAX_ERROR:
            joints = [face_yaw, solved[0], solved[1], *ctx._PLANAR_REACH_WRIST]
            if ctx.check_joint_state_validity(joints):
                return radius, solved[0], solved[1]
        if radius >= object_radius - 1e-9:
            break
        radius = min(radius + THRUST_PULLBACK_STEP, object_radius)
    return None

