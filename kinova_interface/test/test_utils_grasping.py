import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive

from kinova_interface.utils import grasping
from kinova_interface.utils.grasping import (
    grasp_candidates, half_extent, preferred_style, rank,
)

CFG = yaml.safe_load((Path(__file__).parent.parent / 'data' / 'configs' / 'grasping.yaml').read_text())

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
    assert set(CFG) == {
        'ik_timeout_sec', 'yaw_step_deg', 'wrist_flip_threshold_rad', 'standoff', 'lift_height',
        'top_grasp_depth', 'tip_clearance', 'tall_ratio', 'tall_grip_height',
        'width_margin', 'min_width', 'place_clearance',
    }


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
        grasp_candidates(_obj(sphere), CFG)


# grasp_candidates()
def test_cube_has_top_and_side_grasps_but_none_from_below():
    candidates, rejected = grasp_candidates(_obj(CUBE, z=0.025), CFG)

    styles = [c.style for c in candidates]
    assert styles.count('top') == 4      # 2 closing axes x 2 flips
    assert styles.count('side') == 16    # 4 faces x 2 closing axes x 2 flips
    assert not rejected
    for c in candidates:
        assert _approach(c)[2] <= grasping.SIDE_MAX_Z


def test_too_wide_closing_is_rejected():
    long_box = {'type': SolidPrimitive.BOX, 'dimensions': [0.2, 0.05, 0.05]}
    candidates, rejected = grasp_candidates(_obj(long_box, z=0.025), CFG)

    assert all(c.width < 0.1 for c in candidates)
    assert rejected[('top', 'too wide')] == 2
    assert rejected[('side', 'too wide')] == 4


def test_top_grasp_pads_sit_below_the_top():
    candidates, _ = grasp_candidates(_obj(BOTTLE, z=0.1), CFG)
    top = next(c for c in candidates if c.style == 'top')

    assert top.grasp == pytest.approx([0.4, 0.0, 0.2 - CFG['top_grasp_depth']])
    assert top.width == pytest.approx(0.06)


def test_top_grasp_is_raised_to_keep_tips_off_the_floor():
    candidates, _ = grasp_candidates(_obj(CUBE, z=0.025), CFG)
    top = next(c for c in candidates if c.style == 'top')

    tip_z = top.grasp[2] - grasping.FINGERTIP_LENGTH
    assert tip_z == pytest.approx(CFG['tip_clearance'])


def test_top_grasp_dropped_when_object_too_short():
    flat = {'type': SolidPrimitive.BOX, 'dimensions': [0.05, 0.05, 0.02]}
    candidates, rejected = grasp_candidates(_obj(flat, z=0.01), CFG)

    assert not [c for c in candidates if c.style == 'top']
    assert rejected[('top', 'too short')] == 4


def test_side_grasp_height_capped_for_tall_objects():
    candidates, _ = grasp_candidates(_obj(BOTTLE, z=0.1), CFG)
    side = [c for c in candidates if c.style == 'side']

    assert side
    for c in side:
        assert c.grasp[2] == pytest.approx(CFG['tall_grip_height'])
        assert abs(_approach(c)[2]) == pytest.approx(0.0, abs=1e-9)


def test_side_grasp_at_centre_for_short_objects():
    candidates, _ = grasp_candidates(_obj(CUBE, z=0.025), CFG)
    side = next(c for c in candidates if c.style == 'side')

    assert side.grasp[2] == pytest.approx(0.025)


def test_lying_cylinder_gets_top_grasp_across_the_diameter():
    candidates, _ = grasp_candidates(_obj(BOTTLE, z=0.03, rotation=LYING), CFG)
    top = [c for c in candidates if c.style == 'top']

    assert top
    for c in top:
        assert c.width == pytest.approx(0.06)
        # closing across the diameter, not along the bottle's axis (base Y)
        assert abs(_closing(c)[1]) == pytest.approx(0.0, abs=1e-9)


def test_pre_grasp_backs_off_along_approach_and_lift_goes_up():
    candidates, _ = grasp_candidates(_obj(BOTTLE, z=0.1), CFG)
    for c in candidates:
        assert c.pre_grasp == pytest.approx(c.grasp - CFG['standoff'] * _approach(c))
        assert c.lift == pytest.approx(c.grasp + [0, 0, CFG['lift_height']])


# preferred_style() / rank()
def test_auto_style_prefers_side_for_tall_objects_and_top_otherwise():
    assert preferred_style(_obj(BOTTLE, z=0.1), CFG) == 'side'
    assert preferred_style(_obj(CUBE, z=0.025), CFG) == 'top'
    assert preferred_style(_obj(BOTTLE, z=0.03, rotation=LYING), CFG) == 'top'


def test_rank_auto_puts_preferred_style_first_but_keeps_the_rest():
    target = _obj(BOTTLE, z=0.1)
    candidates, _ = grasp_candidates(target, CFG)
    ranked = rank(candidates, target, CFG)

    assert len(ranked) == len(candidates)
    styles = [c.style for c in ranked]
    assert styles[0] == 'side'
    assert styles.index('top') > max(i for i, s in enumerate(styles) if s == 'side')


def test_rank_forced_style_keeps_only_that_style():
    target = _obj(BOTTLE, z=0.1)
    candidates, _ = grasp_candidates(target, CFG)

    assert {c.style for c in rank(candidates, target, CFG, 'top')} == {'top'}


def test_rank_prefers_reaching_straight_out_from_the_base():
    target = _obj(BOTTLE, x=0.0, y=0.4, z=0.1)
    candidates, _ = grasp_candidates(target, CFG)
    best = rank(candidates, target, CFG)[0]

    assert _approach(best) == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)
