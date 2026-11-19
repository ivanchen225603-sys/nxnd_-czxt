"""离线外廓高度假设：固定观测上沿，仅向下延伸四壁；不修改生产参数。"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import open3d as o3d

from carpointcloud_analyzer import Config
from carpointcloud_analyzer import PointCloudAnalyzer
from carpointcloud_analyzer.domain.box_geometry import mask_points_away_from, rotate_wl
from carpointcloud_analyzer.domain.fusion import Extrinsic
from carpointcloud_analyzer.domain.wall_reference import complete_rectangular_wall_reference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--extrinsic", type=Path, required=True)
    parser.add_argument("--height", type=float, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if not np.isfinite(args.height) or not 1.8 <= args.height <= 4:
        raise ValueError("示例整体高度必须在 1.8–4 m")
    digest = hashlib.sha256(args.pcd.read_bytes()).hexdigest()
    ext = Extrinsic.load(str(args.extrinsic))
    cfg = Config(input_unit="m", scene_preprocessing_enable=False,
                 weighbridge_plane_source=tuple(ext.weighbridge_plane_a),
                 weighbridge_plane_orientation_verified=True, scene_auto_level_required=True)
    analyzer = PointCloudAnalyzer(cfg)
    raw = analyzer.loader.load(str(args.pcd))
    leveled, plane, _, _ = analyzer._level_scene(raw)
    measured, transform = analyzer.geometry.vertical_candidate(leveled, plane[:3], "weighbridge")
    reference = complete_rectangular_wall_reference(measured, cfg)
    if reference.box is None:
        raise ValueError("输入不适用矩形四壁补全")
    box = reference.box
    lower, upper = box.h_range
    target_lower = upper - args.height
    if target_lower >= lower:
        raise ValueError("此工具仅演示向下延伸，不裁剪实测点或原补点")
    yaw = np.deg2rad(reference.metadata["yawDeg"])
    w = np.linspace(*box.w_range, int(np.ceil(box.width / .04)) + 1)
    length_axis = np.linspace(box.rear_l, box.front_l, int(np.ceil(box.length / .04)) + 1)
    h = np.linspace(target_lower, lower, int(np.ceil((lower - target_lower) / .04)) + 1)[:-1]
    faces = []
    for axis, value in [(0, w[0]), (0, w[-1]), (1, length_axis[0]), (1, length_axis[-1])]:
        horizontal, height = np.meshgrid(length_axis if axis == 0 else w, h, indexing="ij")
        face = np.empty((horizontal.size, 3))
        face[:, axis], face[:, 1 - axis], face[:, 2] = value, horizontal.ravel(), height.ravel()
        faces.append(face)
    extension_aligned = np.unique(np.vstack(faces), axis=0)
    extension = rotate_wl(extension_aligned, yaw)
    extension = extension[mask_points_away_from(
        extension, np.vstack([measured, reference.points]), cfg.carriage_completion_dedup_radius)]
    extension_aligned = rotate_wl(extension, -yaw)
    assert np.isclose(extension_aligned[:, 2].min(), target_lower)
    assert np.all(extension_aligned[:, 2] < lower)
    assert np.all(np.any(np.isclose(extension_aligned[:, :2],
                                   [w[0], length_axis[0]]), axis=1) |
                  np.any(np.isclose(extension_aligned[:, :2], [w[-1], length_axis[-1]]), axis=1))
    base_source = analyzer._restore_source_coordinates(reference.points, transform, analyzer._last_site_frame)
    extra_source = analyzer._restore_source_coordinates(extension, transform, analyzer._last_site_frame)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    for filename, points, counts in [
        (f"carriage_height_{args.height:g}m".replace(".", "_") + ".pcd", np.vstack([raw, base_source, extra_source]),
         [len(raw), len(base_source), len(extra_source)]),
        ("height_extension_only.pcd", extra_source, [0, 0, len(extra_source)]),
    ]:
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        cloud.colors = o3d.utility.Vector3dVector(np.vstack([
            np.tile(color, (count, 1)) for count, color in zip(counts,
                [[.4, .55, .65], [1., .45, .08], [.7, .25, .85]])]))
        path = args.output_dir / filename
        if not o3d.io.write_point_cloud(str(path), cloud):
            raise IOError(path)
        restored = np.asarray(o3d.io.read_point_cloud(str(path)).points)
        assert len(restored) == len(points) and np.isfinite(restored).all()
        np.testing.assert_allclose(restored, points, atol=2e-6, rtol=0)
    assert hashlib.sha256(args.pcd.read_bytes()).hexdigest() == digest
    local_plane = transform.plane_to_local(plane)
    center = rotate_wl(np.array([[np.mean(w), np.mean(length_axis), upper]]), yaw)[0]
    top_ground = float((center @ local_plane[:3] + local_plane[3]) / np.linalg.norm(local_plane[:3]))
    report = {"referenceOnly": True, "inputSha256": digest, "source": str(args.pcd.resolve()),
              "coordinateFrame": "CAM_004", "anchor": "observed_top_fixed_extend_downward",
              "nominalOverallDimensionsM": [6.06, 2.55, args.height],
              "beforeDimensionsM": [box.length, box.width, box.height],
              "afterDimensionsM": [box.length, box.width, args.height],
              "addedHeightM": lower - target_lower,
              "modelUpperToWeighbridgeM": top_ground,
              "modelLowerBeforeToWeighbridgeM": top_ground - box.height,
              "modelLowerAfterToWeighbridgeM": top_ground - args.height,
              "inputPointCount": len(raw), "originalSyntheticPointCount": len(base_source),
              "extensionPointCount": len(extra_source), "roofGenerated": False, "floorGenerated": False}
    (args.output_dir / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    # 只抽样显示，不改导出的点云。绘图坐标为箱体 L/W/距地磅高度。
    def sample(points, limit):
        q = rotate_wl(points, -yaw)
        keep = ((q[:, 0] >= w[0] - .3) & (q[:, 0] <= w[-1] + .3)
                & (q[:, 1] >= length_axis[0] - .3) & (q[:, 1] <= length_axis[-1] + .3)
                & (q[:, 2] >= target_lower - .15) & (q[:, 2] <= upper + .15))
        q = q[keep][::max(1, int(np.ceil(keep.sum() / limit)))]
        return np.round(np.column_stack([q[:, 1] - np.mean(length_axis), q[:, 0] - np.mean(w),
                                         q[:, 2] - upper + top_ground]), 3).tolist()
    visual = {"report": report, "measured": sample(measured, 4500),
              "original": sample(reference.points, 3500), "extra": sample(extension, 3500)}
    (args.output_dir / "visual_data.json").write_text(
        json.dumps(visual, separators=(",", ":")), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
