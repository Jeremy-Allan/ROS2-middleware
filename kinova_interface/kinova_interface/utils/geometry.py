import math

from shape_msgs.msg import SolidPrimitive


def euler_to_quaternion(roll, pitch, yaw):
    """Convert roll/pitch/yaw (radians) to a quaternion, returned as (x, y, z, w)."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quaternion_to_euler(x, y, z, w):
    """Convert a quaternion (x, y, z, w) to roll/pitch/yaw (radians).

    Pitch is derived via asin() and clamped to +-90deg, so this is not a
    full-range inverse for gimbal-locked orientations - fine for this
    project's use (deriving a yaw estimate, or composing a small rotation
    delta onto the arm's current orientation), not for arbitrary poses.
    """
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return (x, y, z, w)


def quat_rotate_vector(q, v):
    qv = (q[0], q[1], q[2])
    qw = q[3]

    def cross(a, b):
        return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

    t = tuple(2*c for c in cross(qv, v))
    ct = cross(qv, t)
    return (v[0]+qw*t[0]+ct[0], v[1]+qw*t[1]+ct[1], v[2]+qw*t[2]+ct[2])


def pose_in_new_frame(pose_dict, transform):
    """Re-express a pose (dict with position x/y/z and orientation x/y/z/w)
    in the frame that `transform` (a TransformStamped converting into that
    frame) targets."""
    pos = pose_dict['position']
    orient = pose_dict['orientation']
    t = transform.transform.translation
    r = transform.transform.rotation
    tq = (r.x, r.y, r.z, r.w)

    rotated = quat_rotate_vector(tq, (pos['x'], pos['y'], pos['z']))
    new_pos = (rotated[0] + t.x, rotated[1] + t.y, rotated[2] + t.z)
    new_orient = quat_multiply(tq, (orient['x'], orient['y'], orient['z'], orient['w']))
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
