"""'dropoff' recipe action."""
from typing import TYPE_CHECKING

from kinova_interface.utils.geometry import object_half_height

# Only for the ctx type hint (editor go-to-definition). Not imported at runtime,
# because arm_actions imports this module and that would be circular.
if TYPE_CHECKING:
    from kinova_interface.actions.arm_actions import ArmActions


def run(ctx: 'ArmActions', params: dict) -> tuple[bool, str]:
    # Fall back to the object we actually know is held if the recipe
    # step didn't name one - the release-height math below needs the
    # held object's height to avoid releasing into the destination.
    target_name = params.get('target') or ctx.held_object
    destination_name = params.get('destination')
    open_pos = float(params.get('open_position', 0.0))
    hover_clearance = float(params.get('place_offset', 0.1))
    release_clearance = 0.02

    target_info = ctx.get_object_info(target_name) if target_name else None
    target_half_height = object_half_height(target_info['shape']) if target_info else 0.0

    if destination_name:
        dest_info = ctx.get_object_info(destination_name)
        if not dest_info:
            return False, f"Could not resolve destination '{destination_name}'"

        dest_pos = dest_info['pose']['position']
        dest_top_z = dest_pos['z'] + object_half_height(dest_info['shape'])

        # release_z is where the target object's center should end up, resting
        # on top of the destination rather than at the destination's own center
        release_z = dest_top_z + target_half_height
        px, py = dest_pos['x'], dest_pos['y']
    else:
        # No destination: put it back at its registered position (where it was picked up)
        if not target_info:
            return False, "dropoff with no 'destination' needs a resolvable 'target' to release in place"
        pos = target_info['pose']['position']
        px, py = pos['x'], pos['y']
        release_z = pos['z'] + target_half_height

    # 1. Move to a hover position above the destination, collision-safe approach
    r = ctx.call_move_service(px, py, release_z + hover_clearance)
    if not r['success']:
        return False, f"Failed to move to hover position above destination: {r['message']}"

    # 2. Lower to a small clearance above the release height before opening,
    # so the object isn't dropped from the hover height
    r = ctx.call_move_service(px, py, release_z + release_clearance)
    if not r['success']:
        return False, f"Failed to lower to release position: {r['message']}"

    # 3. Open gripper to release
    rg = ctx.call_move_gripper_service(open_pos)
    if not rg['success']:
        return False, f"Failed to open gripper during place: {rg['message']}"

    if target_name:
        orient = target_info['pose']['orientation'] if target_info else None
        # Update pose to the actual release position, not the hover offset
        if not ctx.update_object_pose(target_name, px, py, release_z, orient):
            ctx.get_logger().error(f"Failed to update pose for {target_name}, but continuing...")

        # 4. Add object back to planning scene (detach)
        ctx.detach_object(target_name)

    if target_name == ctx.held_object:
        ctx.held_object = None
    if destination_name:
        return True, f"Placed '{target_name or 'object'}' at '{destination_name}'"
    return True, f"Released '{target_name}' in place"
