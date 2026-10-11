"""Grasp candidates for pickup, from an object's shape and pose.

Everything is in base_link, and base_link +Z is taken as up (arm mounted upright).
tool_frame points along +Z (approach) and the fingers close along X. Its origin is the pad centre.
"""
import math
from collections import Counter
from collections.abc import Iterator
from dataclasses import dataclass
from typing import NamedTuple, Optional

import numpy as np
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive

from kinova_interface.utils.geometry import orientation_from_axes
from kinova_interface.utils.robot import FINGERTIP_LENGTH, GRIPPER_MAX_OPENING, GRIPPER_OPEN

# Settings under 'grasping' in data/configs/motion_settings.yaml, all doubles
CONFIG_KEYS = (
    'velocity_scale', 'acceleration_scale', 'fallback_planning_time', 'upright_tolerance',
    'position_tolerance', 'orientation_tolerance', 'cartesian_step', 'max_joint_jump',
    'yaw_step_deg', 'standoff', 'lift_height', 'top_grasp_depth', 'tip_clearance', 'tall_ratio', 'width_margin',
    'min_width', 'place_clearance', 'default_place_offset', 'max_place_offset',
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

# base_link axis indices, for half_extent
X, Y, Z = 0, 1, 2
# tool_frame +Z, the way the gripper points
TOOL_APPROACH = np.array([0.0, 0.0, 1.0])

# (approach, closing, width) in the object's own frame
ObjectGrasp = tuple[np.ndarray, np.ndarray, float]

class Waypoint(NamedTuple):
    name: str
    position: np.ndarray            # tool_frame position
    straight: bool                  # reach it in a straight line, rather than any collision-free path
    gripper: Optional[float] = None # gripper position when this move starts, if it changes just before it
    upright: bool = False           # keep the gripper's tilt the whole way (carrying something that mustn't tip)


@dataclass
class GraspCandidate:
    style: str              # 'top' or 'side'
    rotation: Rotation      # tool_frame orientation in the base frame
    grasp: np.ndarray       # tool_frame position at the grasp
    pre_grasp: np.ndarray
    lift: np.ndarray

    @property
    def waypoints(self) -> list[Waypoint]:
        return [Waypoint('pre-grasp', self.pre_grasp, straight=False),
                Waypoint('grasp', self.grasp, straight=True),
                Waypoint('lift', self.lift, straight=True)]


def object_centre(pose: dict) -> np.ndarray:
    position = pose['position']
    return np.array([position['x'], position['y'], position['z']], dtype=float)


def object_rotation(pose: dict) -> Rotation:
    orientation = pose['orientation']
    return Rotation.from_quat([orientation['x'], orientation['y'], orientation['z'], orientation['w']])


def half_extent(shape: dict, rotation: Rotation, axis: int = Z) -> float:
    """Half the object's size along base axis X, Y or Z, for any orientation."""
    matrix = rotation.as_matrix()
    dimensions = shape['dimensions']
    if shape['type'] == SolidPrimitive.BOX:
        return sum(abs(matrix[axis, object_axis]) * dimensions[object_axis] / 2.0 for object_axis in range(3))
    if shape['type'] == SolidPrimitive.CYLINDER:
        # How much the cylinder's own axis (its Z) lines up with the base axis
        axis_alignment = abs(matrix[axis, 2])
        height, radius = dimensions[SolidPrimitive.CYLINDER_HEIGHT], dimensions[SolidPrimitive.CYLINDER_RADIUS]
        return axis_alignment * height / 2.0 + radius * math.sqrt(max(0.0, 1.0 - axis_alignment ** 2))
    raise ValueError(f"unsupported shape type {shape['type']}")


def _box_grasps(dimensions: list[float]) -> Iterator[ObjectGrasp]:
    axes = np.eye(3)  # the box's own X, Y, Z
    for approach_axis in range(3):
        for face in BOTH_WAYS:
            # Come in from the + or - face on this axis, heading towards the centre
            approach = -face * axes[approach_axis]
            for closing_axis in range(3):
                if closing_axis == approach_axis:
                    continue  # can't close along the approach
                for flip in BOTH_WAYS:
                    # Close across this axis, so the fingers span that side
                    yield approach, flip * axes[closing_axis], dimensions[closing_axis]


def _cylinder_grasps(dimensions: list[float], step_deg: float) -> Iterator[ObjectGrasp]:
    diameter = 2 * dimensions[SolidPrimitive.CYLINDER_RADIUS]
    # Sample angles around the cylinder's axis (its own Z)
    for angle in np.radians(np.arange(0.0, 360.0, step_deg)):
        radial = np.array([math.cos(angle), math.sin(angle), 0.0])    # out from the axis at this angle
        tangent = np.array([-math.sin(angle), math.cos(angle), 0.0])  # along the rim at this angle
        for end in BOTH_WAYS:
            # End-on: down onto the +Z or -Z end, closing across the diameter.
            # The angle 180 degrees on already covers the wrist flip.
            yield np.array([0.0, 0.0, -end]), radial, diameter
        for flip in BOTH_WAYS:
            # From the curved side at this angle, straddling the axis
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
    rotation = object_rotation(target_info['pose'])
    half_height = half_extent(shape, rotation, Z)
    top_z, floor_z = centre[2] + half_height, centre[2] - half_height

    max_width = GRIPPER_MAX_OPENING - grasp_config['width_margin']
    candidates, rejected = [], Counter()
    for approach, closing, width in _object_frame_grasps(shape, grasp_config['yaw_step_deg']):
        world_approach, world_closing = rotation.apply(approach), rotation.apply(closing)
        if world_approach[2] <= TOP_MAX_Z:
            grasp_style = 'top'
        elif world_approach[2] <= SIDE_MAX_Z:
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
                        floor_z + grasp_config['tip_clearance'] - FINGERTIP_LENGTH * world_approach[2])
            if pad_z > top_z:
                rejected[(grasp_style, f'too short: {top_z - floor_z:.3f} m tall')] += 1
                continue
            # Slide along the approach to that height
            grasp = centre + world_approach * (pad_z - centre[2]) / world_approach[2]
        else:
            grasp = centre.copy()  # side: pads at the object's centre

        candidates.append(GraspCandidate(
            style=grasp_style,
            rotation=orientation_from_axes(world_approach, world_closing),
            grasp=grasp,
            pre_grasp=grasp - grasp_config['standoff'] * world_approach,
            lift=grasp + np.array([0.0, 0.0, grasp_config['lift_height']]),
        ))
    return candidates, rejected


def preferred_style(target_info: dict, grasp_config: dict) -> str:
    """'side' for tall, thin objects, otherwise 'top'."""
    shape = target_info['shape']
    rotation = object_rotation(target_info['pose'])
    height = 2 * half_extent(shape, rotation, Z)
    width = 2 * min(half_extent(shape, rotation, X), half_extent(shape, rotation, Y))
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

    position = target_info['pose']['position']
    bearing = np.array([position['x'], position['y'], 0.0])
    distance = np.linalg.norm(bearing)
    bearing = bearing / distance if distance > 1e-6 else np.array([1.0, 0.0, 0.0])
    natural = {grasp_style: _natural_rotation(grasp_style, bearing) for grasp_style in ('top', 'side')}

    return sorted(candidates, key=lambda candidate: (
        candidate.style != preferred,
        (candidate.rotation * natural[candidate.style].inv()).magnitude(),
    ))


def yaw_offsets(step_deg: float) -> list[float]:
    """0, +step, -step, +2 step, -2 step, ... then 180, in degrees."""
    offsets = [0.0]
    steps = 1
    while steps * step_deg < 180.0:
        offsets += [steps * step_deg, -steps * step_deg]
        steps += 1
    return offsets + [180.0]


def contains_xy(target_info: dict, x: float, y: float) -> bool:
    """True if (x, y) is inside the object's outline, at its centre height."""
    shape = target_info['shape']
    position = target_info['pose']['position']
    # The point in the object's own axes
    local = object_rotation(target_info['pose']).inv().apply([x - position['x'], y - position['y'], 0.0])
    dimensions = shape['dimensions']
    if shape['type'] == SolidPrimitive.BOX:
        return all(abs(local[axis]) <= dimensions[axis] / 2.0 for axis in range(3))
    if shape['type'] == SolidPrimitive.CYLINDER:
        return (abs(local[2]) <= dimensions[SolidPrimitive.CYLINDER_HEIGHT] / 2.0
                and math.hypot(local[0], local[1]) <= dimensions[SolidPrimitive.CYLINDER_RADIUS])
    raise ValueError(f"unsupported shape type {shape['type']}")


@dataclass
class PlaceCandidate:
    yaw_deg: float              # turn from the pickup yaw
    rotation: Rotation          # tool_frame orientation, the pickup one turned about vertical
    hover: np.ndarray           # tool_frame positions, in order
    release: np.ndarray
    back_off: np.ndarray
    object_centre: np.ndarray   # where the object ends up
    object_rotation: Rotation
    keep_upright: bool = False  # carry it to the hover without tipping it

    @property
    def waypoints(self) -> list[Waypoint]:
        # Pickup in reverse: down to the release in a straight line, then back out along the approach.
        # The gripper opens before backing off, so that's planned with it open (an open finger can hit
        # something next to the object).
        return [Waypoint('hover', self.hover, straight=False, upright=self.keep_upright),
                Waypoint('release', self.release, straight=True),
                Waypoint('back-off', self.back_off, straight=True, gripper=GRIPPER_OPEN)]


def place_candidates(held_grasp: dict, shape: dict, point: np.ndarray, grasp_config: dict,
                     place_offset: float) -> Iterator[PlaceCandidate]:
    """Tool poses that put the held object's bottom centre at `point`, with the pickup tilt.
    One per yaw about vertical, same yaw as the pickup first."""
    for yaw in yaw_offsets(grasp_config['yaw_step_deg']):
        tool_rotation = Rotation.from_euler('z', yaw, degrees=True) * held_grasp['tool_rotation']
        placed_object_rotation = tool_rotation * held_grasp['object_rotation']
        resting_centre = point + [0.0, 0.0, half_extent(shape, placed_object_rotation, Z)]
        release_centre = resting_centre + [0.0, 0.0, grasp_config['place_clearance']]
        release = release_centre - tool_rotation.apply(held_grasp['object_position'])
        # Retreat reverses the approach, like pickup's pre-grasp
        back_off = release - grasp_config['standoff'] * tool_rotation.apply(TOOL_APPROACH)
        yield PlaceCandidate(
            yaw_deg=yaw,
            rotation=tool_rotation,
            hover=release + [0.0, 0.0, place_offset],
            release=release,
            back_off=back_off,
            object_centre=resting_centre,
            object_rotation=placed_object_rotation,
            keep_upright=held_grasp.get('keep_upright', False),
        )
