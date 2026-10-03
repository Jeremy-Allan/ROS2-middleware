"""Grasp candidates for pickup, from an object's shape and pose.

Everything is in base_link, and base_link +Z is taken as up (arm mounted upright).
tool_frame points along +Z (approach) and the fingers close along X. Its origin is the pad centre.
"""
import math
from collections import Counter
from collections.abc import Iterator
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive

from kinova_interface.utils.geometry import orientation_from_axes
from kinova_interface.utils.robot import FINGERTIP_LENGTH, GRIPPER_MAX_OPENING

# Settings under 'grasping' in data/configs/grasping.yaml, all doubles
CONFIG_KEYS = (
    'ik_timeout_sec', 'wrist_flip_threshold_rad', 'yaw_step_deg', 'standoff', 'lift_height',
    'top_grasp_depth', 'tip_clearance', 'tall_grip_height', 'tall_ratio', 'width_margin',
    'min_width', 'place_clearance',
)

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

class Waypoint(NamedTuple):
    name: str
    position: np.ndarray            # tool_frame position
    avoid_collisions: bool = True


@dataclass
class GraspCandidate:
    style: str              # 'top' or 'side'
    rotation: Rotation      # tool_frame orientation in the base frame
    grasp: np.ndarray       # tool_frame position at the grasp
    pre_grasp: np.ndarray
    lift: np.ndarray

    @property
    def waypoints(self) -> list[Waypoint]:
        # The object is still in the scene when these are checked, so the lift only checks reach.
        # The planner collision-checks the real lift once the object is attached.
        return [Waypoint('pre-grasp', self.pre_grasp), Waypoint('grasp', self.grasp),
                Waypoint('lift', self.lift, avoid_collisions=False)]


def object_centre(pose: dict) -> np.ndarray:
    p = pose['position']
    return np.array([p['x'], p['y'], p['z']], dtype=float)


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


def grasp_candidates(target_info: dict, grasp_config: dict,
                     style: str = 'auto') -> tuple[list[GraspCandidate], Counter]:
    """Usable grasps of `style` ('auto' for both top and side), plus a Counter of (style, reason) for the ones dropped."""
    shape = target_info['shape']
    centre = object_centre(target_info['pose'])
    rot = object_rotation(target_info['pose'])
    h = half_extent(shape, rot, 2)
    top_z, floor_z = centre[2] + h, centre[2] - h

    max_width = GRIPPER_MAX_OPENING - grasp_config['width_margin']
    candidates, rejected = [], Counter()
    for approach, closing, width in _object_frame_grasps(shape, grasp_config['yaw_step_deg']):
        a, c = rot.apply(approach), rot.apply(closing)
        if a[2] <= TOP_MAX_Z:
            grasp_style = 'top'
        elif a[2] <= SIDE_MAX_Z:
            grasp_style = 'side'
        else:
            continue  # from below
        if style != 'auto' and grasp_style != style:
            continue

        # Reasons carry the numbers, so the failure message says what to tune
        if width > max_width:
            rejected[(grasp_style, f'too wide: {width:.3f} m > max {max_width:.3f}')] += 1
            continue
        if width < grasp_config['min_width']:
            rejected[(grasp_style, f"too narrow: {width:.3f} m < min {grasp_config['min_width']:.3f}")] += 1
            continue

        if grasp_style == 'top':
            # Pads just below the top, but keep the fingertips off the floor
            pad_z = max(top_z - grasp_config['top_grasp_depth'],
                        floor_z + grasp_config['tip_clearance'] - FINGERTIP_LENGTH * a[2])
            if pad_z > top_z:
                rejected[(grasp_style, f'too short: {top_z - floor_z:.3f} m tall')] += 1
                continue
            grasp = centre + a * (pad_z - centre[2]) / a[2]  # slide along the approach to that height
        else:
            grasp = centre.copy()
            grasp[2] = min(centre[2], floor_z + grasp_config['tall_grip_height'])

        candidates.append(GraspCandidate(
            style=grasp_style,
            rotation=orientation_from_axes(a, c),
            grasp=grasp,
            pre_grasp=grasp - grasp_config['standoff'] * a,
            lift=grasp + np.array([0.0, 0.0, grasp_config['lift_height']]),
        ))
    return candidates, rejected


def preferred_style(target_info: dict, grasp_config: dict) -> str:
    """'side' for tall, thin objects, otherwise 'top'."""
    shape = target_info['shape']
    rot = object_rotation(target_info['pose'])
    height = 2 * half_extent(shape, rot, 2)
    width = 2 * min(half_extent(shape, rot, 0), half_extent(shape, rot, 1))
    return 'side' if height > grasp_config['tall_ratio'] * width else 'top'


def _natural_rotation(style: str, bearing: np.ndarray) -> Rotation:
    """Reaching straight out from the base, fingers closing sideways. The closing side is a guess, tune in sim."""
    left = np.cross([0.0, 0.0, 1.0], bearing)
    approach = [0.0, 0.0, -1.0] if style == 'top' else bearing
    return orientation_from_axes(approach, left)


def rank(candidates: list[GraspCandidate], target_info: dict, grasp_config: dict,
         style: str = 'auto') -> list[GraspCandidate]:
    """Best first. 'auto' puts the preferred style first.
    Within a style, the closest to the natural wrist pose wins."""
    preferred = preferred_style(target_info, grasp_config) if style == 'auto' else style

    pos = target_info['pose']['position']
    bearing = np.array([pos['x'], pos['y'], 0.0])
    norm = np.linalg.norm(bearing)
    bearing = bearing / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0])
    natural = {s: _natural_rotation(s, bearing) for s in ('top', 'side')}

    return sorted(candidates, key=lambda c: (
        c.style != preferred,
        (c.rotation * natural[c.style].inv()).magnitude(),
    ))


def yaw_offsets(step_deg: float) -> list[float]:
    """0, +step, -step, +2 step, -2 step, ... then 180, in degrees."""
    offsets = [0.0]
    k = 1
    while k * step_deg < 180.0:
        offsets += [k * step_deg, -k * step_deg]
        k += 1
    return offsets + [180.0]


def contains_xy(target_info: dict, x: float, y: float) -> bool:
    """True if (x, y) is inside the object's outline, at its centre height."""
    shape = target_info['shape']
    pos = target_info['pose']['position']
    local = object_rotation(target_info['pose']).inv().apply([x - pos['x'], y - pos['y'], 0.0])
    d = shape['dimensions']
    if shape['type'] == SolidPrimitive.BOX:
        return all(abs(local[i]) <= d[i] / 2.0 for i in range(3))
    if shape['type'] == SolidPrimitive.CYLINDER:
        return (abs(local[2]) <= d[SolidPrimitive.CYLINDER_HEIGHT] / 2.0
                and math.hypot(local[0], local[1]) <= d[SolidPrimitive.CYLINDER_RADIUS])
    raise ValueError(f"unsupported shape type {shape['type']}")


@dataclass
class PlaceCandidate:
    yaw_deg: float              # turn from the pickup yaw
    rotation: Rotation          # tool_frame orientation, the pickup one turned about vertical
    hover: np.ndarray           # tool_frame positions, in order
    release: np.ndarray
    back: np.ndarray
    up: np.ndarray
    object_centre: np.ndarray   # where the object ends up
    object_rotation: Rotation

    @property
    def waypoints(self) -> list[Waypoint]:
        # The object is still attached when these are checked, so the retreat only checks reach.
        # The planner collision-checks the real retreat after detaching.
        return [Waypoint('hover', self.hover), Waypoint('release', self.release),
                Waypoint('back-off', self.back, avoid_collisions=False),
                Waypoint('up', self.up, avoid_collisions=False)]


def place_candidates(held_grasp: dict, shape: dict, point: np.ndarray, grasp_config: dict,
                     place_offset: float) -> Iterator[PlaceCandidate]:
    """Tool poses that put the held object's bottom centre at `point`, with the pickup tilt.
    One per yaw about vertical, same yaw as the pickup first."""
    for yaw in yaw_offsets(grasp_config['yaw_step_deg']):
        tool = Rotation.from_euler('z', yaw, degrees=True) * held_grasp['tool_rotation']
        obj = tool * held_grasp['object_rotation']
        centre = point + [0.0, 0.0, grasp_config['place_clearance'] + half_extent(shape, obj, 2)]
        release = centre - tool.apply(held_grasp['object_position'])
        # Retreat reverses the approach, then goes up
        back = release - grasp_config['standoff'] * tool.apply([0.0, 0.0, 1.0])
        yield PlaceCandidate(
            yaw_deg=yaw,
            rotation=tool,
            hover=release + [0.0, 0.0, place_offset],
            release=release,
            back=back,
            up=back + [0.0, 0.0, grasp_config['lift_height']],
            object_centre=centre,
            object_rotation=obj,
        )
