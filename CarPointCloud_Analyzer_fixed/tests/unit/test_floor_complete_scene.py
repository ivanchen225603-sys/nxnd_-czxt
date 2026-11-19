"""冻结完整车厢：端墙不能成为底板高差和三段高度的参考。"""
from pathlib import Path

import numpy as np
import pytest

from carpointcloud_analyzer import Config, PointCloudAnalyzer


@pytest.mark.parametrize('case,kind,drop,heights', [
    ('step', 2, 50., [2.1, 1.6, 1.6]),
    ('ramp', 3, 50., [2.1, 1.6, 1.6]),
    ('slope', 4, 60., [2.2, 2., 1.8]),
])
def test_floor_profile_uses_bottom_surface_not_end_wall(case, kind, drop, heights):
    with np.load(Path(__file__).parents[1] / 'fixtures/floor_profile_complete_scene.npz') as saved:
        points = saved[case]
    analyzer = PointCloudAnalyzer(Config(
        save_clean_pointcloud=False, weighbridge_plane_source=(0., 0., 1., 0.),
        weighbridge_plane_orientation_verified=True))
    output = analyzer._analyze_points(points, '')
    payload = output['integrationPayloads']['vehicleDimensions']
    profile = payload['floorProfile']
    # 沿用原冻结场景的全部尺寸/分段容差。
    assert payload['dimensions']['trailerLength'] == pytest.approx(1000., abs=5.)
    assert payload['dimensions']['highLowPlateHeight'] == pytest.approx(drop, abs=5.)
    assert profile['case'] == kind
    assert len(profile['segments']) == 3
    assert [s['heightToReference'] for s in profile['segments']] == pytest.approx(
        heights, abs=.06 if case == 'ramp' else .05)
    assert [s['lengthToPrev'] for s in profile['segments']] == pytest.approx(
        [4., 3., 3.] if case == 'step' else [5., 2.5, 2.5] if case == 'ramp' else [10/3]*3,
        abs={'step': .25, 'ramp': .35, 'slope': .01}[case])
    plates = payload['highLowPlates']
    assert [p['plateId'] for p in plates] == [1, 2, 3]
    assert [p['lengthCm'] for p in plates] == pytest.approx(
        [s['lengthToPrev'] * 100 for s in profile['segments']], abs=.1)
    assert [p['lowestToGround'] for p in plates] == pytest.approx(
        [s['heightToReference'] * 100 for s in profile['segments']], abs=.1)
    assert payload['wires'] == []
