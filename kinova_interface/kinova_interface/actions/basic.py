"""The simple one-call recipe actions: 'home', 'move_arm', 'relative_move', 'gripper'."""
from typing import TYPE_CHECKING

# Only for the ctx type hint (editor go-to-definition). Not imported at runtime,
# because arm_actions imports this module and that would be circular.
if TYPE_CHECKING:
    from kinova_interface.actions.arm_actions import ArmActions


def home(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    motion_params = ctx.build_motion_params(params.get('speed'))
    result = ctx.call_home_service(motion_params)
    if not result['success']:
        return False, f"Failed to move home: {result['message']}"
    return True, "Moved home"


def move_arm(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    target_name = params['target']
    coords = ctx.get_static_object_coords(target_name)
    if not coords:
        return False, f"Could not resolve target '{target_name}'"

    orientation = ctx.resolve_orientation(params.get('orientation'))
    if orientation is None:
        return False, f"Unknown orientation preset '{params.get('orientation')}'"
    has_orientation, roll, pitch, yaw = orientation

    motion_params = ctx.build_motion_params(params.get('speed'))
    result = ctx.call_move_service(coords['x'], coords['y'], coords['z'], has_orientation, roll, pitch, yaw, motion_params)
    if not result['success']:
        return False, f"Failed to move to '{target_name}': {result['message']}"
    return True, f"Moved to '{target_name}'"


def relative_move(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    vector_name = params['vector']
    vector = ctx.get_relative_movement_vector(vector_name)
    if not vector:
        return False, f"Could not resolve movement '{vector_name}'"

    if params.get('orientation'):
        ctx.get_logger().warn(f"relative_move ignores 'orientation' ('{params['orientation']}'); use move_arm for that")

    motion_params = ctx.build_motion_params(params.get('speed'))
    result = ctx.call_relative_move_service(vector['x'], vector['y'], vector['z'], motion_params=motion_params)
    if not result['success']:
        return False, f"Failed to move '{vector_name}': {result['message']}"
    return True, f"Moved '{vector_name}'"


def gripper(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    position = float(params['position'])
    result = ctx.call_move_gripper_service(position)
    if not result['success']:
        return False, f"Failed to move gripper to {position}: {result['message']}"
    return True, f"Moved gripper to {position}"
