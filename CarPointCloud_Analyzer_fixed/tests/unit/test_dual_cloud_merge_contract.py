"""共同场景融合包装层保留底层质量诊断，不重新解释门禁。"""

from types import SimpleNamespace
import json

import numpy as np
import pytest

from carpointcloud_analyzer.application import dual_cloud_merge
from carpointcloud_analyzer.domain.fusion import CalibrationQuality, Extrinsic, MergeResult
from carpointcloud_analyzer.domain.site_frame import SiteFrame
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.domain.regional_fusion import RegionalSelection


@pytest.mark.parametrize("quality_passed", [True, False])
def test_vehicle_merge_preserves_overlap_notes_without_changing_gate(
    monkeypatch, quality_passed,
):
    rng = np.random.default_rng(42)
    points_a = rng.random((600, 3)) * [2.0, 8.0, 2.0]
    points_b = rng.random((600, 3)) * [2.0, 8.0, 2.0]
    scene = np.vstack([points_a, points_b])
    frame = SiteFrame(
        origin=np.zeros(3), axes=np.eye(3), plane=np.array([0., 0., 1., 0.]),
        inlier_count=100, sampled_points=len(scene), spans=(8., 2.),
        one_sided_ratio=1.,
    )
    filtered = SimpleNamespace(
        vehicle_points_list=[scene], primary_index=0,
        primary_vehicle_points=scene, stats={}, edge_truncated=False,
    )
    notes = ["两片点云没有可测的重叠区，无法校验外参是否仍然有效"]
    warnings = [] if quality_passed else ["雷达 A 轻滤后点数不足"]
    site_result = MergeResult(
        points=scene, n_from_a=600, n_from_b=600, n_after_uniform=len(scene),
        overlap_rmse=None, overlap_notes=notes, warnings=warnings,
        quality_passed=quality_passed,
    )
    monkeypatch.setattr(dual_cloud_merge, "estimate_site_frame", lambda _: frame)
    monkeypatch.setattr(dual_cloud_merge, "filter_scene", lambda *_: filtered)
    monkeypatch.setattr(dual_cloud_merge, "merge", lambda *_: site_result)

    result = dual_cloud_merge.merge_vehicle_clouds(
        points_a, points_b, Extrinsic(matrix=np.eye(4)),
    ).fusion
    payload = result.to_dict()

    assert payload["overlapNotes"] == notes
    assert payload["qualityPassed"] is quality_passed
    assert payload["warnings"] == warnings
    assert payload["reason"] == site_result.reason
    # 返回结果独立拥有诊断列表，不能因后续修改包装结果而改写原融合证据。
    result.overlap_notes.append("追加诊断")
    assert site_result.overlap_notes == notes


@pytest.mark.parametrize("degenerate,warnings", [(["longitudinal"], []), ([], ["质量告警"])])
def test_independent_merge_rejects_quality_that_only_claims_trustworthy(
    tmp_path, degenerate, warnings,
):
    extrinsic = Extrinsic(matrix=np.eye(4), camera_a="CAM_004", camera_b="CAM_005",
                          quality=CalibrationQuality(.8, .01, 200, degenerate,
                                                     warnings=warnings))
    document = extrinsic.to_dict()
    document["quality"]["trustworthy"] = True
    path = tmp_path / "extrinsic.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    parsed = Extrinsic.from_dict(document)
    assert parsed.quality.trustworthy is False
    with pytest.raises(dual_cloud_merge.DualCloudMergeError, match="可信"):
        dual_cloud_merge._load_verified_extrinsic(
            path, "CAM_004", "CAM_005", require_verified_quality=True)
    _, metadata = dual_cloud_merge._load_verified_extrinsic(
        path, "CAM_004", "CAM_005", require_verified_quality=False)
    assert metadata["qualityVerified"] is False


def test_regional_selection_precedes_output_voxels_and_raw_measurement_copy(monkeypatch):
    """质量证据仍全双云；最终点和原回波副本不得把副雷达厢体带回来。"""
    rng = np.random.default_rng(15)
    body = rng.uniform([-.8, -4., 1.], [.8, 0., 3.], (600, 3))
    cab = rng.uniform([-.8, 1., 1.], [.8, 3., 3.], (600, 3))
    points_a = np.vstack((body + [0., .008, .008], cab))
    points_b = body.copy()
    scene = np.vstack((points_a, points_b))
    frame = SiteFrame(np.zeros(3), np.eye(3), np.array([0., 0., 1., 0.]),
                      100, len(scene), (8., 2.), 1.)
    filtered = SimpleNamespace(vehicle_points_list=[scene], primary_index=0,
                               primary_vehicle_points=scene, stats={}, edge_truncated=False)
    result = MergeResult(scene, 1200, 600, len(scene), .04,
                         n_combined=len(scene), quality_passed=True,
                         filtered_points_a=points_a, aligned_filtered_points_b=points_b)
    plan = RegionalSelection(CoordinateTransform(np.zeros(3), np.eye(3)), 1, .35, 1)
    monkeypatch.setattr(dual_cloud_merge, "estimate_site_frame", lambda _: frame)
    monkeypatch.setattr(dual_cloud_merge, "filter_scene", lambda *_: filtered)
    monkeypatch.setattr(dual_cloud_merge, "merge", lambda *_: result)
    monkeypatch.setattr(dual_cloud_merge, "select_carriage_source", lambda *_: (plan, {"applied": True}))

    actual = dual_cloud_merge.merge_vehicle_clouds(points_a, points_b, Extrinsic(np.eye(4)))

    np.testing.assert_array_equal(actual.selected_observed_points, np.vstack((cab, body)))
    expected_cloud = dual_cloud_merge.o3d.geometry.PointCloud(
        dual_cloud_merge.o3d.utility.Vector3dVector(np.vstack((cab, body))))
    expected = np.asarray(expected_cloud.voxel_down_sample(.02).points)
    np.testing.assert_array_equal(actual.points, expected)
    assert actual.fusion.n_combined == len(scene)
    assert actual.fusion.overlap_rmse == .04
    assert actual.fusion.quality_passed
