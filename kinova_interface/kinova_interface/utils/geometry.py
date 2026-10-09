import math

import numpy as np
from scipy.spatial.transform import Rotation


def orientation_from_axes(approach, closing):
    """Rotation that points the gripper along `approach` with its fingers
    closing along `closing` (base-frame vectors, any length). tool_frame's
    fingers point along +Z and close along X. Raises
    ValueError if the two are parallel."""
    z = np.asarray(approach, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.asarray(closing, dtype=float)
    x = x - x.dot(z) * z
    if np.linalg.norm(x) < 1e-9:
        raise ValueError(f"closing {closing} is parallel to approach {approach}")
    x = x / np.linalg.norm(x)
    # Columns are the tool's X, Y, Z axes in the base frame
    return Rotation.from_matrix(np.column_stack([x, np.cross(z, x), z]))


def pose_in_new_frame(pose_dict, transform):
    """Re-express a pose (dict with position x/y/z and orientation x/y/z/w)
    in the frame that `transform` (a TransformStamped converting into that
    frame) targets."""
    pos = pose_dict['position']
    orient = pose_dict['orientation']
    t = transform.transform.translation
    r = transform.transform.rotation
    rotation = Rotation.from_quat([r.x, r.y, r.z, r.w])

    new_pos = rotation.apply([pos['x'], pos['y'], pos['z']]) + [t.x, t.y, t.z]
    new_orient = (rotation * Rotation.from_quat([orient['x'], orient['y'], orient['z'], orient['w']])).as_quat()
    return {
        'position': {'x': new_pos[0], 'y': new_pos[1], 'z': new_pos[2]},
        'orientation': {'x': new_orient[0], 'y': new_orient[1], 'z': new_orient[2], 'w': new_orient[3]}
    }


# Directions are relative to the line from the arm's base out to the point, as seen
# from the base: left is anticlockwise from above. Not yet checked on the real arm.
DIRECTION_ROTATIONS = {
    'forward': 0.0,
    'left': math.pi / 2.0,
    'right': -math.pi / 2.0,
    'backward': math.pi,
}


def resolve_direction_offset(reference_x, reference_y, direction, distance):
    """The (x, y) point `distance` m from the reference point in `direction`, or None for an unknown direction.
    Moves in a straight line, so left/right end up slightly further from the base."""
    if direction not in DIRECTION_ROTATIONS:
        return None

    bearing_len = math.hypot(reference_x, reference_y)
    if bearing_len < 1e-6:
        # Point is on the base, so there's no line to follow; use +X
        bx, by = 1.0, 0.0
    else:
        bx, by = reference_x / bearing_len, reference_y / bearing_len

    angle = DIRECTION_ROTATIONS[direction]
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    dx = bx * cos_a - by * sin_a
    dy = bx * sin_a + by * cos_a

    return reference_x + dx * distance, reference_y + dy * distance
