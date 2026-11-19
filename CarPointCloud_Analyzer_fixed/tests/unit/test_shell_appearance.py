"""展示副本不改变实测、尺寸或其他车型；完整覆盖、预算、对称及折边回归。"""

import copy
from unittest.mock import Mock

import numpy as np
import pytest
from scipy.spatial import cKDTree

from carpointcloud_analyzer import Config
from carpointcloud_analyzer import PointCloudAnalyzer
from carpointcloud_analyzer.domain.box_geometry import rotate_wl
from carpointcloud_analyzer.domain.shell_appearance import build_shell_appearance
from carpointcloud_analyzer.domain.wall_reference import complete_rectangular_wall_reference
from test_wall_reference import rectangular_shell, shell_with_front_lip, stub_preparation


def test_dense_shell_preserves_sources_and_has_full_symmetric_walls_and_lip():
    measured = shell_with_front_lip()
    original = measured.copy()
    reference = complete_rectangular_wall_reference(measured, Config())
    old_points, old_meta = reference.points.copy(), copy.deepcopy(reference.metadata)
    result = build_shell_appearance(measured, reference)
    assert result.metadata['frontLipIncluded']
    assert result.triangles.shape == (30 + 6 * bool(old_meta['frontLip']['innerDropGenerated']), 3)
    assert result.metadata['replacedDisplayPointCount'] > 10000
    assert len(result.context) > 500  # 车头不丢
    assert 400000 < len(result.shell) <= 550000
    np.testing.assert_array_equal(measured, original)
    np.testing.assert_array_equal(reference.points, old_points)
    assert reference.metadata == old_meta
    q = rotate_wl(result.shell, -np.deg2rad(old_meta['yawDeg']))
    box = reference.box
    tree = cKDTree(q)
    mirror = q.copy()
    mirror[:, 0] = sum(box.w_range) - q[:, 0]
    assert tree.query(mirror)[0].max() < 1e-10
    assert np.ptp(q[:, 2]) == pytest.approx(2.6, abs=.06)
    assert len(np.unique(q, axis=0)) == len(q)
    # 独立均匀随机位置验证每张壁面覆盖，不只验证点数。
    rng = np.random.default_rng(28)
    bounds = [box.w_range, (box.rear_l, box.front_l), box.h_range]
    for axis, value in [(0, box.w_range[0]), (0, box.w_range[1]), (1, box.rear_l), (1, box.front_l)]:
        probes = np.column_stack([rng.uniform(lo, hi, 3000) for lo, hi in bounds])
        probes[:, axis] = value
        assert tree.query(probes)[0].max() <= .008
    # 中部既不封顶也不凭空补底板。
    interior = ((q[:, 0] > box.w_range[0] + .1) & (q[:, 0] < box.w_range[1] - .1)
                & (q[:, 1] > box.rear_l + .5) & (q[:, 1] < box.front_l - .5))
    assert not interior.any()


@pytest.mark.parametrize('budget', [1000, 29000, 100000])
def test_budget_resamples_whole_faces_deterministically(budget):
    p = rectangular_shell()
    ref = complete_rectangular_wall_reference(p, Config())
    a = build_shell_appearance(p, ref, max_points=budget)
    b = build_shell_appearance(p, ref, max_points=budget)
    assert len(a.shell) <= budget
    np.testing.assert_array_equal(a.shell, b.shell)
    q = rotate_wl(a.shell, -np.deg2rad(ref.metadata['yawDeg']))
    heights = np.unique(q[:, 2])
    assert np.ptp(np.diff(heights)) < 1e-10


@pytest.mark.parametrize('change', [{'spacing': 0}, {'spacing': .001}, {'spacing': float('nan')},
                                    {'max_points': 999}, {'max_points': 10000000}, {'max_points': True}])
def test_invalid_resource_settings_fail_before_allocation(change):
    with pytest.raises(ValueError):
        build_shell_appearance(None, None, **change)


def test_unmatched_profile_is_not_restyled():
    """车型窗口不匹配时不生成展示外壳；长宽窗口是唯一的识别依据。"""
    p = rectangular_shell()
    for cfg in [Config(carriage_wall_reference_profile_length_range=(10., 13.)),
                Config(carriage_wall_reference_profile_width_range=(3.5, 4.))]:
        ref = complete_rectangular_wall_reference(p, cfg)
        assert build_shell_appearance(p, ref) is None


def test_automatic_preview_uses_display_copy_but_formal_and_audit_keep_originals(monkeypatch):
    p = shell_with_front_lip()
    original = p.copy()
    analyzer = PointCloudAnalyzer()
    stub_preparation(monkeypatch, analyzer, p)
    monkeypatch.setattr(analyzer.loader, 'load', lambda _: p.copy())
    forbidden = Mock(side_effect=AssertionError('display attempted persistence'))
    monkeypatch.setattr(analyzer, '_save_clean_pointcloud', forbidden)
    before = analyzer.analyze('unused')
    preview = analyzer.analyze_preview(p)
    appearance = analyzer.analyze_appearance('unused')
    audit = analyzer.analyze_wall_reference('unused')
    after = analyzer.analyze('unused')
    assert before['dimensions'] == after['dimensions']
    assert before['integrationPayloads'] == after['integrationPayloads']
    assert before['integrationPayloads']['vehicleDimensions']['detectStatus'] == 'success'
    assert before['integrationPayloads']['vehicleDimensions']['dimensions']['trailerBodyHeight'] is None
    assert before['loadStatus']['status'] == after['loadStatus']['status'] == 'Uncertain'
    assert before['reviewMeasurements']['dimensions'] == after['reviewMeasurements']['dimensions']
    assert before['reviewMeasurements']['dimensions']['carriage']['height'] is None
    assert np.isfinite(before['reviewMeasurements']['dimensions']['carriage']['length'])
    np.testing.assert_array_equal(p, original)
    assert 'appearance' not in after['wallCompletion']
    assert 'appearance' not in audit['wallCompletion']
    assert audit['measuredPointCount'] == len(p)
    for result in [preview, appearance]:
        meta = result['wallCompletion']['appearance']
        assert meta['applied'] and not meta['measurementInputChanged']
        assert result['syntheticPointCount'] == meta['shellPointCount']
        assert result['measuredPointCount'] < len(p)
        assert result['loadStatus']['status'] == 'Uncertain'
        assert 'integrationPayloads' not in result
        assert result['displayMesh']['displayOnly']
        assert len(result['displayMesh']['coordinates']) == 90 + 18 * bool(result['wallCompletion']['frontLip']['innerDropGenerated'])
    assert len(preview['coordinates']) <= 60000
    forbidden.assert_not_called()


def test_display_mesh_has_only_four_walls_when_no_lip_is_supported():
    p = rectangular_shell()
    ref = complete_rectangular_wall_reference(p, Config())
    result = build_shell_appearance(p, ref, max_points=1000)
    assert result.triangles.shape == (24, 3)
    triangles = rotate_wl(result.triangles, -np.deg2rad(ref.metadata['yawDeg'])).reshape(-1, 3, 3)
    for triangle in triangles:
        # 仅垂直壁面，不凭空增加顶/底面；低预算也完整覆盖高度。
        assert np.ptp(triangle[:, 2]) == pytest.approx(2.6, abs=.06)
        assert min(np.ptp(triangle[:, 0]), np.ptp(triangle[:, 1])) < 1e-10


def test_existing_profile_preview_is_byte_identical_to_audit_sampling(monkeypatch):
    p = rectangular_shell()
    analyzer = PointCloudAnalyzer(
        Config(carriage_wall_reference_profile_length_range=(10., 13.)))
    stub_preparation(monkeypatch, analyzer, p)
    monkeypatch.setattr(analyzer.loader, 'load', lambda _: p)
    actual = analyzer.analyze_preview(p)
    original = analyzer.analyze_wall_reference('unused')
    n = original['measuredPointCount']
    expected = np.vstack([original['coordinates'][:n: max(1, int(np.ceil(n / 30000)))],
        original['coordinates'][n:][::max(1, int(np.ceil(original['syntheticPointCount'] / 30000)))]])
    np.testing.assert_array_equal(actual['coordinates'], expected)


def test_appearance_entry_forbids_identity_and_persistence():
    for cfg in [Config(license_plate='TEST'), Config(save_clean_pointcloud=True)]:
        with pytest.raises(ValueError, match='preview must disable'):
            PointCloudAnalyzer(cfg).analyze_appearance('unused')
