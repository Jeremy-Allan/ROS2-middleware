import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from kinova_interface.utils.geometry import orientation_from_axes


def _unit(v):
    return np.asarray(v, dtype=float) / np.linalg.norm(v)


# orientation_from_axes()
@pytest.mark.parametrize("approach, closing", [
    ((0, 0, -1), (0, 1, 0)),    # top down, close side to side
    ((0, 0, -1), (1, 0, 0)),    # top down, close front to back
    ((1, 0, 0), (0, 1, 0)),     # level, pointing forward
    ((0, 1, 0), (0, 0, 1)),     # level, pointing left, close vertically
    ((1, 1, -1), (1, -1, 0)),   # arbitrary diagonal
])
def test_orientation_from_axes_points_gripper_axes_as_asked(approach, closing):
    rotation = orientation_from_axes(approach, closing)
    assert rotation.apply([0, 0, 1]) == pytest.approx(_unit(approach))
    assert rotation.apply([1, 0, 0]) == pytest.approx(_unit(closing))


def test_orientation_from_axes_drops_closing_component_along_approach():
    # closing tilted 45deg toward the approach axis still means "close along X"
    rotation = orientation_from_axes((0, 0, -1), (1, 0, -1))
    assert rotation.apply([1, 0, 0]) == pytest.approx([1, 0, 0])


def test_orientation_from_axes_rejects_parallel_axes():
    with pytest.raises(ValueError):
        orientation_from_axes((0, 0, -1), (0, 0, 2))


# orientation_presets.json
_PRESETS_PATH = Path(__file__).resolve().parents[1] / 'data' / 'configs' / 'env' / 'orientation_presets.json'
_PRESET_AXES = {
    # name: (approach, closing), base frame
    'top_down': ((0, 0, -1), (0, 1, 0)),
    'top_down_90': ((0, 0, -1), (1, 0, 0)),
    'side_level': ((1, 0, 0), (0, 1, 0)),
}


def test_orientation_presets_match_their_axes():
    """The RPY in orientation_presets.json must be the orientation its
    name describes, per the gripper axis convention."""
    presets = json.loads(_PRESETS_PATH.read_text())
    assert set(presets) == set(_PRESET_AXES)
    for name, (approach, closing) in _PRESET_AXES.items():
        p = presets[name]
        preset = Rotation.from_euler('xyz', [p['roll'], p['pitch'], p['yaw']])
        assert (preset.inv() * orientation_from_axes(approach, closing)).magnitude() < 1e-5, name
