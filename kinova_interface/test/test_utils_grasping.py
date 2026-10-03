import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive

from kinova_interface.utils import grasping
from kinova_interface.utils.grasping import (
    contains_xy, grasp_candidates, half_extent, place_candidates, preferred_style, rank, yaw_offsets,
)

GRASP_CONFIG = yaml.safe_load((Path(__file__).parent.parent / 'data' / 'configs' / 'grasping.yaml').read_text())['/**']['ros__parameters']['grasping']

BOTTLE = {'type': SolidPrimitive.CYLINDER, 'dimensions': [0.2, 0.03]}
CUBE = {'type': SolidPrimitive.BOX, 'dimensions': [0.05, 0.05, 0.05]}
LYING = Rotation.from_euler('x', 90, degrees=True)


def _obj(shape, x=0.4, y=0.0, z=0.0, rotation=None):
    q = (rotation if rotation is not None else Rotation.identity()).as_quat()
    return {
        'shape': shape,
        'pose': {'position': {'x': x, 'y': y, 'z': z},
                 'orientation': {'x': q[0], 'y': q[1], 'z': q[2], 'w': q[3]}},
    }


def _approach(c):
    return c.rotation.apply([0, 0, 1])


def _closing(c):
    return c.rotation.apply([1, 0, 0])


def test_config_has_every_key():
    assert set(GRASP_CONFIG) == set(grasping.CONFIG_KEYS)
    # typed as doubles, so an int in the file would fail to load
    assert all(isinstance(v, float) for v in GRASP_CONFIG.values())


# half_extent()
def test_half_extent_upright_and_lying_cylinder():
    assert half_extent(BOTTLE, Rotation.identity(), 2) == pytest.approx(0.1)
    assert half_extent(BOTTLE, LYING, 2) == pytest.approx(0.03)
    assert half_extent(BOTTLE, LYING, 1) == pytest.approx(0.1)


def test_half_extent_box_on_its_side_and_tilted():
    box = {'type': SolidPrimitive.BOX, 'dimensions': [0.1, 0.06, 0.02]}
    assert half_extent(box, Rotation.from_euler('y', 90, degrees=True), 2) == pytest.approx(0.05)
    tilted = Rotation.from_euler('x', 45, degrees=True)
    assert half_extent(box, tilted, 2) == pytest.approx((0.06 + 0.02) / 2 * math.sqrt(0.5))


def test_unsupported_shape_raises():
    sphere = {'type': SolidPrimitive.SPHERE, 'dimensions': [0.03]}
    with pytest.raises(ValueError):
        grasp_candidates(_obj(sphere), GRASP_CONFIG)


# grasp_candidates()
def test_cube_has_top_and_side_grasps_but_none_from_below():
    candidates, rejected = grasp_candidates(_obj(CUBE, z=0.025), GRASP_CONFIG)

    styles = [c.style for c in candidates]
    assert styles.count('top') == 4      # 2 closing axes x 2 flips
    assert styles.count('side') == 16    # 4 faces x 2 closing axes x 2 flips
    assert not rejected
    for c in candidates:
        assert _approach(c)[2] <= grasping.SIDE_MAX_Z


def test_too_wide_closing_is_rejected():
    long_box = {'type': SolidPrimitive.BOX, 'dimensions': [0.2, 0.05, 0.05]}
    candidates, rejected = grasp_candidates(_obj(long_box, z=0.025), GRASP_CONFIG)

    assert all(abs(_closing(c)[0]) < 1e-9 for c in candidates)  # never across the 0.2 m side
    assert rejected[('top', 'too wide: 0.200 m > max 0.094')] == 2
    assert rejected[('side', 'too wide: 0.200 m > max 0.094')] == 4


def test_top_grasp_pads_sit_below_the_top():
    candidates, _ = grasp_candidates(_obj(BOTTLE, z=0.1), GRASP_CONFIG)
    top = next(c for c in candidates if c.style == 'top')

    assert top.grasp == pytest.approx([0.4, 0.0, 0.2 - GRASP_CONFIG['top_grasp_depth']])


def test_top_grasp_is_raised_to_keep_tips_off_the_floor():
    candidates, _ = grasp_candidates(_obj(CUBE, z=0.025), GRASP_CONFIG)
    top = next(c for c in candidates if c.style == 'top')

    tip_z = top.grasp[2] - grasping.FINGERTIP_LENGTH
    assert tip_z == pytest.approx(GRASP_CONFIG['tip_clearance'])


def test_top_grasp_dropped_when_object_too_short():
    flat = {'type': SolidPrimitive.BOX, 'dimensions': [0.05, 0.05, 0.02]}
    candidates, rejected = grasp_candidates(_obj(flat, z=0.01), GRASP_CONFIG)

    assert not [c for c in candidates if c.style == 'top']
    assert rejected[('top', 'too short: 0.020 m tall')] == 4


def test_side_grasp_height_capped_for_tall_objects():
    candidates, _ = grasp_candidates(_obj(BOTTLE, z=0.1), GRASP_CONFIG)
    side = [c for c in candidates if c.style == 'side']

    assert side
    for c in side:
        assert c.grasp[2] == pytest.approx(GRASP_CONFIG['tall_grip_height'])
        assert abs(_approach(c)[2]) == pytest.approx(0.0, abs=1e-9)


def test_side_grasp_at_centre_for_short_objects():
    candidates, _ = grasp_candidates(_obj(CUBE, z=0.025), GRASP_CONFIG)
    side = next(c for c in candidates if c.style == 'side')

    assert side.grasp[2] == pytest.approx(0.025)


def test_lying_cylinder_gets_top_grasp_across_the_diameter():
    candidates, _ = grasp_candidates(_obj(BOTTLE, z=0.03, rotation=LYING), GRASP_CONFIG)
    top = [c for c in candidates if c.style == 'top']

    assert top
    for c in top:
        # closing across the diameter, not along the bottle's axis (base Y)
        assert abs(_closing(c)[1]) == pytest.approx(0.0, abs=1e-9)


def test_pre_grasp_backs_off_along_approach_and_lift_goes_up():
    candidates, _ = grasp_candidates(_obj(BOTTLE, z=0.1), GRASP_CONFIG)
    for c in candidates:
        assert c.pre_grasp == pytest.approx(c.grasp - GRASP_CONFIG['standoff'] * _approach(c))
        assert c.lift == pytest.approx(c.grasp + [0, 0, GRASP_CONFIG['lift_height']])


# preferred_style() / rank()
def test_auto_style_prefers_side_for_tall_objects_and_top_otherwise():
    assert preferred_style(_obj(BOTTLE, z=0.1), GRASP_CONFIG) == 'side'
    assert preferred_style(_obj(CUBE, z=0.025), GRASP_CONFIG) == 'top'
    assert preferred_style(_obj(BOTTLE, z=0.03, rotation=LYING), GRASP_CONFIG) == 'top'


def test_rank_auto_puts_preferred_style_first_but_keeps_the_rest():
    target = _obj(BOTTLE, z=0.1)
    candidates, _ = grasp_candidates(target, GRASP_CONFIG)
    ranked = rank(candidates, target, GRASP_CONFIG)

    assert len(ranked) == len(candidates)
    styles = [c.style for c in ranked]
    assert styles[0] == 'side'
    assert styles.index('top') > max(i for i, s in enumerate(styles) if s == 'side')


def test_forced_style_keeps_only_that_style():
    long_box = {'type': SolidPrimitive.BOX, 'dimensions': [0.2, 0.05, 0.05]}
    candidates, rejected = grasp_candidates(_obj(long_box, z=0.025), GRASP_CONFIG, 'top')

    assert {c.style for c in candidates} == {'top'}
    assert {style for style, _ in rejected} == {'top'}


def test_rank_prefers_reaching_straight_out_from_the_base():
    target = _obj(BOTTLE, x=0.0, y=0.4, z=0.1)
    candidates, _ = grasp_candidates(target, GRASP_CONFIG)
    best = rank(candidates, target, GRASP_CONFIG)[0]

    assert _approach(best) == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)


# yaw_offsets() / contains_xy() / place_candidates()
def test_yaw_offsets_alternate_out_to_180():
    assert yaw_offsets(60.0) == [0.0, 60.0, -60.0, 120.0, -120.0, 180.0]
    assert yaw_offsets(100.0) == [0.0, 100.0, -100.0, 180.0]


def test_contains_xy_uses_the_objects_own_axes():
    tray = {'type': SolidPrimitive.BOX, 'dimensions': [0.3, 0.1, 0.02]}
    assert contains_xy(_obj(tray, x=0.5), 0.64, 0.0)
    assert not contains_xy(_obj(tray, x=0.5), 0.5, 0.06)
    turned = _obj(tray, x=0.5, rotation=Rotation.from_euler('z', 90, degrees=True))
    assert contains_xy(turned, 0.5, 0.14)
    assert not contains_xy(turned, 0.6, 0.0)

    assert contains_xy(_obj(BOTTLE), 0.42, 0.0)
    assert not contains_xy(_obj(BOTTLE), 0.44, 0.0)


def _held_grasp(target):
    """held_grasp as pickup stores it, for the best-ranked grasp."""
    c = rank(grasp_candidates(target, GRASP_CONFIG)[0], target, GRASP_CONFIG)[0]
    pos = target['pose']['position']
    centre = np.array([pos['x'], pos['y'], pos['z']])
    return {
        'tool_rotation': c.rotation,
        'object_position': c.rotation.inv().apply(centre - c.grasp),
        'object_rotation': c.rotation.inv() * grasping.object_rotation(target['pose']),
    }, c


def test_place_at_the_pickup_spot_repeats_the_grasp():
    target = _obj(BOTTLE, z=0.1)
    held, grasp = _held_grasp(target)
    point = np.array([0.4, 0.0, 0.0])  # the bottle's bottom centre

    first = next(place_candidates(held, BOTTLE, point, GRASP_CONFIG, 0.1))

    lifted = [0.0, 0.0, GRASP_CONFIG['place_clearance']]
    assert first.release == pytest.approx(grasp.grasp + lifted)
    assert (first.rotation * grasp.rotation.inv()).magnitude() == pytest.approx(0.0, abs=1e-9)
    assert first.object_centre == pytest.approx([0.4, 0.0, 0.1 + GRASP_CONFIG['place_clearance']])
    assert first.hover == pytest.approx(first.release + [0.0, 0.0, 0.1])


def test_place_candidates_keep_the_tilt_and_retreat_back_then_up():
    target = _obj(BOTTLE, z=0.1)
    held, _ = _held_grasp(target)
    point = np.array([0.3, 0.2, 0.05])

    candidates = list(place_candidates(held, BOTTLE, point, GRASP_CONFIG, 0.1))

    assert len(candidates) == len(yaw_offsets(GRASP_CONFIG['yaw_step_deg']))
    for c in candidates:
        approach = c.rotation.apply([0, 0, 1])
        assert approach[2] == pytest.approx(0.0, abs=1e-9)  # still a side grasp
        assert c.object_centre[:2] == pytest.approx(point[:2])
        assert c.release == pytest.approx(c.object_centre - c.rotation.apply(held['object_position']))
        assert c.back == pytest.approx(c.release - GRASP_CONFIG['standoff'] * approach)
        assert c.up == pytest.approx(c.back + [0, 0, GRASP_CONFIG['lift_height']])
