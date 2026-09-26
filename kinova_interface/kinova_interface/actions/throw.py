"""'throw' recipe action (see docs/throw-motion-reference.md)."""
import math
from typing import TYPE_CHECKING

from kinova_interface.utils.geometry import resolve_direction_offset

# Only for the ctx type hint (editor go-to-definition). Not imported at runtime,
# because arm_actions imports this module and that would be circular.
if TYPE_CHECKING:
    from kinova_interface.actions.arm_actions import ArmActions


# A captured, fixed joint_2..6 shape (see docs/throw-motion-reference.md)
# for a real, manually-demonstrated wind-up - not derived from home by
# a wind-up angle like an earlier version of this; joint_1 (base
# facing) is the only thing that varies per throw.
THROW_WINDUP_POSE = [math.radians(v) for v in [-22.84, 56.01, 80.71, 50.6, 0.0]]
# The fling's own end pose - a single continuous swing straight from
# the wind-up to here, joint_1 fixed the whole way.
THROW_FLING_POSE = [math.radians(v) for v in [32.63, -34.26, 80.71, -63.25, 0.0]]
# joint_5 barely moves during the early part of the fling, then swings
# hard right at the release moment - captured directly from the demo,
# this is where the gripper should open.
THROW_RELEASE_JOINT5 = math.radians(-42.7)

def run(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    """A genuine joint-space throw, captured from a manual RViz
    demonstration (see docs/throw-motion-reference.md): rotate to
    face the throw direction (joint_1 only, current shoulder/elbow/
    wrist held exactly where pickup left them), move straight to a
    captured wind-up shape (_THROW_WINDUP_POSE), then one continuous
    fling straight to a captured end pose (_THROW_FLING_POSE) -
    releasing the gripper the instant joint_5 crosses
    _THROW_RELEASE_JOINT5, not after a fixed delay or once the arm
    has stopped.

    Assumes 'target' is already grasped - fails cleanly rather than
    guessing if it isn't.

    The release is a closed-loop trigger on the arm's real, live
    joint_5 position (wait_for_joint_crossing), not a computed
    time.sleep(). A timed sleep - even one scaled to an estimated
    fling duration - was tried first and found unreliable: the
    fire-and-forget fling call itself blocked the client for up to
    HardwareInterfaceClient.FIRE_AND_FORGET_REJECTION_WINDOW_SEC
    (0.5s) before any release-timing code could even start running,
    which for a short, fast fling could consume the entire motion -
    the object would still be gripped once the arm had already
    stopped. call_joint_move_service_async (no wait at all, not even
    that bounded one) plus polling the real joint state fixes this by
    not depending on timing at all - see docs/throw-motion-reference.md
    for the full diagnosis.

    Because release timing is now driven by the arm's real position
    rather than a fixed delay, the landing position recorded
    afterward is still a rough approximation - it isn't computing
    where the object will actually land physically, just recording
    the intended target."""
    target_name = params.get('target')
    destination_name = params.get('destination')
    direction = params.get('direction')
    if not target_name:
        return False, "throw action requires 'target' naming the held object"
    if target_name != ctx.held_object:
        return False, f"Cannot throw '{target_name}': held object is '{ctx.held_object}'"
    if not destination_name and not direction:
        return False, "throw action requires either 'destination' or 'direction'"

    target_info = ctx.get_object_info(target_name)
    if not target_info:
        return False, f"Could not resolve held object '{target_name}' for throw"
    origin = target_info['pose']['position']

    if destination_name:
        dest_info = ctx.get_object_info(destination_name)
        if not dest_info:
            return False, f"Could not resolve throw destination '{destination_name}'"
        release_x, release_y = dest_info['pose']['position']['x'], dest_info['pose']['position']['y']
    else:
        distance = float(params.get('distance', 0.3))
        offset = resolve_direction_offset(origin['x'], origin['y'], direction, distance)
        if offset is None:
            return False, f"Unknown throw direction '{direction}'"
        release_x, release_y = offset

    base_yaw = math.atan2(release_y, release_x)
    motion_params = ctx.build_motion_params(params.get('speed'))

    # 1. Rotate to face the throw direction - joint_1 only, current
    # shoulder/elbow/wrist (wherever pickup left them) held exactly
    if ctx.latest_joint_positions is None:
        return False, "No joint state available to rotate for throw"
    try:
        current_arm_joints = [ctx.latest_joint_positions[f'joint_{i}'] for i in range(2, 7)]
    except KeyError as e:
        return False, f"Missing joint {e} in latest joint state"
    rotate_joints = [base_yaw, *current_arm_joints]
    r = ctx.call_joint_move_service(rotate_joints, motion_params=motion_params)
    if not r['success']:
        return False, f"Failed to rotate to face the throw direction for '{target_name}': {r['message']}"

    # 2. Wind-up - go straight to the captured wind-up shape
    windup_joints = [base_yaw, *THROW_WINDUP_POSE]
    r = ctx.call_joint_move_service(windup_joints, motion_params=motion_params)
    if not r['success']:
        return False, f"Failed to wind up for throw of '{target_name}': {r['message']}"

    # 3. Fling - one continuous swing straight to the end pose, fired
    # without waiting for any response at all (see
    # call_joint_move_service_async's docstring for why)
    fling_joints = [base_yaw, *THROW_FLING_POSE]
    fling_motion = ctx.build_motion_params(params.get('speed', 1.0))
    fling_future = ctx.call_joint_move_service_async(fling_joints, motion_params=fling_motion)
    if fling_future is None:
        return False, f"Failed to start throw fling for '{target_name}'"

    # 4. Release the instant joint_5 crosses the captured release
    # point - a closed-loop trigger on the arm's real position
    windup_joint5 = THROW_WINDUP_POSE[3]
    crossed = ctx.wait_for_joint_crossing('joint_5', THROW_RELEASE_JOINT5, windup_joint5)
    if not crossed:
        return False, f"Timed out waiting for the release point during throw of '{target_name}'"

    open_pos = float(params.get('open_position', 0.0))
    rg = ctx.call_move_gripper_service(open_pos)
    if not rg['success']:
        return False, f"Failed to release gripper during throw: {rg['message']}"

    ctx.detach_object(target_name)
    ctx.update_object_pose(target_name, release_x, release_y, origin['z'], None)
    ctx.held_object = None
    where = f"toward '{destination_name}'" if destination_name else direction
    return True, f"Threw '{target_name}' {where}"
