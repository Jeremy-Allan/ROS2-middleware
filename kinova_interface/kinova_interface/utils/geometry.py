import math

import numpy as np
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive


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


def object_half_height(shape):
    """Half the object's extent along Z, from its shape type/dimensions,
    used to place one object on top of another from their center poses."""
    stype = shape['type']
    dims = shape['dimensions']
    if stype == SolidPrimitive.BOX:
        return dims[2] / 2.0
    if stype in (SolidPrimitive.CYLINDER, SolidPrimitive.CONE):
        return dims[0] / 2.0
    if stype == SolidPrimitive.SPHERE:
        return dims[0]
    return 0.0


# Direction keywords for 'push'/'throw' as an alternative to a named
# destination, as an angle to rotate the reference bearing by. The
# workspace frame's origin is the arm's own base, so a held/pushed
# object's own (x, y) position doubles as the bearing vector from the
# arm out to it. 'left' is a +90 degree (counter-clockwise, viewed from
# above) rotation of that bearing, 'right' is -90 degrees - this matches
# the object's original position, not the arm's own facing, and should
# be validated against the real arm before relying on it.
DIRECTION_ROTATIONS = {
    'forward': 0.0,
    'left': math.pi / 2.0,
    'right': -math.pi / 2.0,
    'backward': math.pi,
}


def resolve_direction_offset(reference_x, reference_y, direction, distance):
    """Given the point a held/pushed object originally rested at
    (reference_x, reference_y), return an (x, y) point further out
    along that same bearing from the arm's base, rotated by the named
    direction and displaced by 'distance'. Returns None for an
    unrecognised direction."""
    if direction not in DIRECTION_ROTATIONS:
        return None

    bearing_len = math.hypot(reference_x, reference_y)
    if bearing_len < 1e-6:
        # No meaningful bearing to rotate (object rests essentially at
        # the arm's base) - fall back to a fixed +X bearing.
        bx, by = 1.0, 0.0
    else:
        bx, by = reference_x / bearing_len, reference_y / bearing_len

    angle = DIRECTION_ROTATIONS[direction]
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    dx = bx * cos_a - by * sin_a
    dy = bx * sin_a + by * cos_a

    return reference_x + dx * distance, reference_y + dy * distance
