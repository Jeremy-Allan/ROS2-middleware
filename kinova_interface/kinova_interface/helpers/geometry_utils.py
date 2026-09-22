import math


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
