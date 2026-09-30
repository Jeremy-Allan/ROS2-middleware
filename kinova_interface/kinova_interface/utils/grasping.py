"""Grasp candidates for pickup, from an object's shape and pose.

Everything is in base_link, and base_link +Z is taken as up (arm mounted upright).
tool_frame points along +Z (approach) and the fingers close along X. Its origin is the pad centre.
"""
import math
from collections import Counter
from collections.abc import Iterator
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive

from kinova_interface.utils.geometry import orientation_from_axes
from kinova_interface.utils.robot import FINGERTIP_LENGTH, GRIPPER_MAX_OPENING

SUPPORTED_SHAPES = (SolidPrimitive.BOX, SolidPrimitive.CYLINDER) # temporary for now

# How steep an approach (tool_frame +Z) can be for each style, in base_link.
# Up to TOP_MAX_TILT_DEG off straight down is a top grasp. From there up to SIDE_MAX_RISE_DEG
# above horizontal is a side grasp. Anything coming up from further below is dropped.
TOP_MAX_TILT_DEG = 45.0
SIDE_MAX_RISE_DEG = 15.0
TOP_MAX_Z = -math.cos(math.radians(TOP_MAX_TILT_DEG))
SIDE_MAX_Z = math.sin(math.radians(SIDE_MAX_RISE_DEG))

# Signs for "either end of an axis": which face, or wrist turned 0/180 deg
BOTH_WAYS = (1, -1)

# (approach, closing, width) in the object's own frame
ObjectGrasp = tuple[np.ndarray, np.ndarray, float]

@dataclass
class GraspCandidate:
    style: str              # 'top' or 'side'
    rotation: Rotation      # tool_frame orientation in the base frame
    grasp: np.ndarray       # tool_frame position at the grasp
    pre_grasp: np.ndarray
    lift: np.ndarray
    width: float            # object width across the fingers


def object_rotation(pose: dict) -> Rotation:
    q = pose['orientation']
    return Rotation.from_quat([q['x'], q['y'], q['z'], q['w']])


def half_extent(shape: dict, rotation: Rotation, axis: int = 2) -> float:
    """Half the object's size along base axis 0/1/2 (x/y/z), for any orientation."""
    R = rotation.as_matrix()
    d = shape['dimensions']
    if shape['type'] == SolidPrimitive.BOX:
        return sum(abs(R[axis, i]) * d[i] / 2.0 for i in range(3))
    if shape['type'] == SolidPrimitive.CYLINDER:
        a = abs(R[axis, 2])  # cylinder axis is the object's Z
        height, radius = d[SolidPrimitive.CYLINDER_HEIGHT], d[SolidPrimitive.CYLINDER_RADIUS]
        return a * height / 2.0 + radius * math.sqrt(max(0.0, 1.0 - a * a))
    raise ValueError(f"unsupported shape type {shape['type']}")


def _box_grasps(dims: list[float]) -> Iterator[ObjectGrasp]:
    axes = np.eye(3)  # the box's own X, Y, Z
    for i in range(3):
        for face in BOTH_WAYS:
            # Come in from the +i or -i face, heading towards the centre
            approach = -face * axes[i]
            for j in range(3):
                if j == i:
                    continue  # can't close along the approach
                for flip in BOTH_WAYS:
                    # Close across axis j, so the fingers span that side
                    yield approach, flip * axes[j], dims[j]


def _cylinder_grasps(dims: list[float], step_deg: float) -> Iterator[ObjectGrasp]:
    diameter = 2 * dims[SolidPrimitive.CYLINDER_RADIUS]
    # Sample angles around the cylinder's axis (its own Z)
    for t in np.radians(np.arange(0.0, 360.0, step_deg)):
        radial = np.array([math.cos(t), math.sin(t), 0.0])    # out from the axis at angle t
        tangent = np.array([-math.sin(t), math.cos(t), 0.0])  # along the rim at angle t
        for end in BOTH_WAYS:
            # End-on: down onto the +Z or -Z end, closing across the diameter.
            # t and t+180 already cover the wrist flip.
            yield np.array([0.0, 0.0, -end]), radial, diameter
        for flip in BOTH_WAYS:
            # From the curved side at angle t, straddling the axis
            yield -radial, flip * tangent, diameter


def _object_frame_grasps(shape: dict, step_deg: float) -> Iterator[ObjectGrasp]:
    """Every (approach, closing, width) for the shape, in the object's own frame.

    approach becomes tool_frame +Z, closing becomes X, width is what the fingers must span.
    All faces are listed, since which one is on the bottom depends on the object's pose.
    grasp_candidates drops the ones that end up coming from below.
    """
    if shape['type'] == SolidPrimitive.BOX:
        return _box_grasps(shape['dimensions'])
    return _cylinder_grasps(shape['dimensions'], step_deg)


def grasp_candidates(target_info: dict, cfg: dict) -> tuple[list[GraspCandidate], Counter]:
    """All usable grasps for the object, plus a Counter of (style, reason) for the ones dropped."""
    shape = target_info['shape']
    if shape['type'] not in SUPPORTED_SHAPES:
        raise ValueError(f"unsupported shape type {shape['type']}")
    pos = target_info['pose']['position']
    centre = np.array([pos['x'], pos['y'], pos['z']], dtype=float)
    rot = object_rotation(target_info['pose'])
    h = half_extent(shape, rot, 2)
    top_z, floor_z = centre[2] + h, centre[2] - h

    candidates, rejected = [], Counter()
    for approach, closing, width in _object_frame_grasps(shape, cfg['yaw_step_deg']):
        a, c = rot.apply(approach), rot.apply(closing)
        if a[2] <= TOP_MAX_Z:
            style = 'top'
        elif a[2] <= SIDE_MAX_Z:
            style = 'side'
        else:
            continue  # from below

        if width > GRIPPER_MAX_OPENING - cfg['width_margin']:
            rejected[(style, 'too wide')] += 1
            continue
        if width < cfg['min_width']:
            rejected[(style, 'too narrow')] += 1
            continue

        if style == 'top':
            # Pads just below the top, but keep the fingertips off the floor
            pad_z = max(top_z - cfg['top_grasp_depth'],
                        floor_z + cfg['tip_clearance'] - FINGERTIP_LENGTH * a[2])
            if pad_z > top_z:
                rejected[(style, 'too short')] += 1
                continue
            grasp = centre + a * (pad_z - centre[2]) / a[2]  # slide along the approach to that height
        else:
            grasp = centre.copy()
            grasp[2] = min(centre[2], floor_z + cfg['tall_grip_height'])

        candidates.append(GraspCandidate(
            style=style,
            rotation=orientation_from_axes(a, c),
            grasp=grasp,
            pre_grasp=grasp - cfg['standoff'] * a,
            lift=grasp + np.array([0.0, 0.0, cfg['lift_height']]),
            width=width,
        ))
    return candidates, rejected


def preferred_style(target_info: dict, cfg: dict) -> str:
    """'side' for tall, thin objects, otherwise 'top'."""
    shape = target_info['shape']
    rot = object_rotation(target_info['pose'])
    height = 2 * half_extent(shape, rot, 2)
    width = 2 * min(half_extent(shape, rot, 0), half_extent(shape, rot, 1))
    return 'side' if height > cfg['tall_ratio'] * width else 'top'


def _natural_rotation(style: str, bearing: np.ndarray) -> Rotation:
    """Reaching straight out from the base, fingers closing sideways. The closing side is a guess, tune in sim."""
    left = np.cross([0.0, 0.0, 1.0], bearing)
    approach = [0.0, 0.0, -1.0] if style == 'top' else bearing
    return orientation_from_axes(approach, left)


def rank(candidates: list[GraspCandidate], target_info: dict, cfg: dict,
         style: str = 'auto') -> list[GraspCandidate]:
    """Best first. 'auto' puts the preferred style first; 'top'/'side' keeps only that style.
    Within a style, the closest to the natural wrist pose wins."""
    if style == 'auto':
        preferred = preferred_style(target_info, cfg)
    else:
        preferred = style
        candidates = [c for c in candidates if c.style == style]

    pos = target_info['pose']['position']
    bearing = np.array([pos['x'], pos['y'], 0.0])
    norm = np.linalg.norm(bearing)
    bearing = bearing / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0])
    natural = {s: _natural_rotation(s, bearing) for s in ('top', 'side')}

    return sorted(candidates, key=lambda c: (
        c.style != preferred,
        (c.rotation * natural[c.style].inv()).magnitude(),
    ))
