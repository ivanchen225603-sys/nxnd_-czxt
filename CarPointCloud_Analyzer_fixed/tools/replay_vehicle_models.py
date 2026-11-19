"""离线双车型回放：不接服务/数据库，不修改原始文件或生产配置。

python -m tools.replay_vehicle_models --extrinsic calibration.json
  --case short A.pcd B.pcd --case long A2.pcd B2.pcd --output-dir new-directory
所有合成点只在原始双云融合及测量结束后生成，独立着色保存。
"""

import argparse
import hashlib
import importlib.metadata
import json
import platform
from dataclasses import replace
from pathlib import Path

import numpy as np
import open3d as o3d

from carpointcloud_analyzer import Config
from carpointcloud_analyzer import PointCloudAnalyzer
from carpointcloud_analyzer.domain.box_geometry import rotate_wl
from carpointcloud_analyzer.application.dual_cloud_merge import merge_vehicle_clouds
from carpointcloud_analyzer.domain.fusion import Extrinsic
from carpointcloud_analyzer.domain.wall_reference import complete_rectangular_wall_reference


def _hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write_cloud(path, points, real_count):
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    colors = np.tile([1., .45, .08], (len(points), 1))
    colors[:real_count] = [.4, .55, .65]
    cloud.colors = o3d.utility.Vector3dVector(colors)
    if not o3d.io.write_point_cloud(str(path), cloud):
        raise IOError(path)
    check = np.asarray(o3d.io.read_point_cloud(str(path)).points)
    if len(check) != len(points) or not np.isfinite(check).all():
        raise IOError(f"PCD readback failed: {path}")


def _verify_stability(item):
    results = [r["analysis"] for r in item["runs"]] + item["sameCloudRuns"]
    shell = "carriageObservation" in results[0]
    if any(("carriageObservation" in r) != shell for r in results):
        raise AssertionError("Algorithm branch changed between replays")
    if any(r["loadStatus"] != results[0]["loadStatus"] for r in results):
        raise AssertionError("Load status changed between replays")
    if shell:
        dimensions = [[r["dimensions"]["carriage"][k] for k in ["length", "width", "height"]]
                      for r in results]
        for result in results:
            payload = result['integrationPayloads'].get('vehicleDimensions', {})
            if payload.get('detectStatus') != 'low_confidence' or result["measurementStatus"]["reviewStatus"] != "pending":
                raise AssertionError("Review quality flag or normal persistence payload lost")
            expected = result["carriageObservation"]
            opening = result['wallCompletion'].get('symmetricOpening', {})
            if result["dimensions"]["carriage"] != {
                "length": opening['lengthM'] if opening.get('applied') else expected["lengthM"],
                "width": opening['widthM'] if opening.get('applied') else expected["widthM"],
                "height": expected["visibleHeightM"],
            }:
                raise AssertionError("Calculated dimensions were suppressed or replaced")
    else:
        dimensions = [[r["dimensions"]["carriage"][k] for k in ["length", "width", "height"]]
                      for r in results]
        control = item["originalPathControl"]
        if control["loadStatus"] != results[0]["loadStatus"]:
            raise AssertionError("Original path load status changed")
        dimensions.append([control["dimensions"]["carriage"][k] for k in ["length", "width", "height"]])
    spread = np.ptp(dimensions, axis=0)
    if shell:
        lips = [r['wallCompletion'].get('frontLip', {}) for r in results]
        if len({bool(lip.get('completed')) for lip in lips}) != 1:
            raise AssertionError('Front lip branch changed between replays')
        if lips[0].get('completed'):
            values = [[lip[key] for key in ['depthM', 'innerEdgeHeightM', 'innerDropLowerHeightM']]
                      for lip in lips]
            if np.ptp(values, axis=0).max() > .001:
                raise AssertionError('Front lip geometry variation exceeds 1 mm')
            if len({lip['synthesizedPoints'] for lip in lips}) != 1:
                raise AssertionError('Front lip grid count changed between replays')
    if max(spread) > .001:
        raise AssertionError(f"Replay variation exceeds 1 mm: {spread}")
    return {"checkedResults": len(dimensions), "maxMinusMinM": spread.tolist(),
            "toleranceM": .001, "passed": True}


def _raw_interior_evidence(analyzer, points, raw_a, aligned_b):
    """对原始数据核查中部回波，排除前处理删掉底板这一可能。"""
    leveled, _, _, _ = analyzer._level_scene(points)
    local, transform = analyzer.geometry.vertical_candidate(leveled, [0, 0, 1], "weighbridge")
    reference = complete_rectangular_wall_reference(local, analyzer.config)
    box, frame = reference.box, analyzer._last_site_frame
    rows = {}
    for name, raw in [("CAM_004", raw_a), ("CAM_005", aligned_b)]:
        q = rotate_wl(transform.to_local(frame.transform(raw)),
                       -np.deg2rad(reference.metadata["yawDeg"]))
        interior = q[(q[:, 0] > box.w_range[0] + .2) & (q[:, 0] < box.w_range[1] - .2)
                     & (q[:, 1] > box.rear_l + .4) & (q[:, 1] < box.front_l - .4)]
        h = interior[:, 2] + transform.origin[2]
        rows[name] = {"rawFootprintPoints": len(interior),
                      "aboveWeighbridgeBelowRimPoints": int(np.sum(
                          (h > .15) & (h < box.h_range[1] + transform.origin[2] - .25)
                      ))}
    return rows


def replay(cases, extrinsic_path, output_dir, repeat):
    if repeat < 1:
        raise ValueError("repeat must be positive")
    output_dir.mkdir(parents=True, exist_ok=False)
    extrinsic = Extrinsic.load(str(extrinsic_path))
    report = {"offlineOnly": True, "stationaryVerified": False,
              "python": platform.python_version(), "dependencies": {}, "cases": {}}
    for name in ["open3d", "numpy", "scipy", "fastapi", "uvicorn", "dmPython"]:
        try:
            report["dependencies"][name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            report["dependencies"][name] = "not installed"
    cfg = Config(input_unit="m", weighbridge_plane_source=tuple(extrinsic.weighbridge_plane_a),
                 scene_auto_level_required=True, scene_preprocessing_fail_open=False)
    measured_cfg = replace(cfg, scene_preprocessing_enable=False,
                           weighbridge_plane_orientation_verified=True)
    for index, (label, a_path, b_path) in enumerate(cases):
        hashes = {str(path): _hash(path) for path in [a_path, b_path, extrinsic_path]}
        loader = PointCloudAnalyzer(cfg).loader
        a, b = loader.load(a_path), loader.load(b_path)
        item = {"sourceSha256": hashes, "runs": [], "sameCloudRuns": [],
                "syntheticPointsUsedForFusion": 0, "syntheticPointsUsedForMeasurement": 0}
        for run in range(repeat):
            merged = merge_vehicle_clouds(a, b, extrinsic, camera_a="CAM_004", camera_b="CAM_005",
                                         config=cfg)
            analyzer = PointCloudAnalyzer(measured_cfg)
            result = analyzer._analyze_points(merged.points, "offline_only.pcd")
            item["runs"].append({"fusion": merged.to_dict(), "analysis": result})
            if run == 0:
                if result.get('reviewMeasurements') is not None:
                    (output_dir / f'case_{index}_review_measurements.json').write_text(
                        json.dumps({'reviewMeasurements': result['reviewMeasurements'],
                                    'fusion': merged.to_dict(), 'loadStatus': result['loadStatus'],
                                    'sourceSha256': hashes}, ensure_ascii=False, indent=2) + '\n',
                        encoding='utf-8',
                    )
                # 非新车型必须对照开关关闭时的原路径；不改变任何算法阈值。
                if "carriageObservation" not in result:
                    control = PointCloudAnalyzer(replace(measured_cfg, carriage_wall_reference_enable=False))
                    item["originalPathControl"] = control._analyze_points(merged.points, "control.pcd")
                else:
                    item["rawInteriorEvidence"] = _raw_interior_evidence(analyzer, merged.points, a,
                                                                         extrinsic.apply(b))
                for order in range(5):
                    shuffled = merged.points[np.random.default_rng(order).permutation(len(merged.points))]
                    item["sameCloudRuns"].append(
                        PointCloudAnalyzer(measured_cfg)._analyze_points(shuffled, "repeat.pcd")
                    )
                preview = analyzer._analyze_points(merged.points, "", preview=True, full_reference=True)
                real_count = preview["measuredPointCount"]
                _write_cloud(output_dir / f"case_{index}_completed_reference.pcd",
                             preview["coordinates"], real_count)
                _write_cloud(output_dir / f"case_{index}_measured_fused.pcd",
                             merged.points, len(merged.points))
                item["completion"] = {key: value for key, value in preview.items() if key != "coordinates"}
            print(f"{label} run={run + 1}: " + json.dumps({
                "points": len(merged.points), "dimensions": result["dimensions"],
                "observation": result.get("carriageObservation", {}),
                "loadStatus": result["loadStatus"],
            }, ensure_ascii=True), flush=True)
        for path, digest in hashes.items():
            if _hash(path) != digest:
                raise RuntimeError(f"Source changed during replay: {path}")
        item["stability"] = _verify_stability(item)
        report["cases"][label] = item
        (output_dir / "replay_report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--extrinsic", type=Path, required=True)
    parser.add_argument("--case", nargs=3, action="append", required=True, metavar=("NAME", "A", "B"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repeat", type=int, default=3)
    args = parser.parse_args()
    replay(args.case, args.extrinsic, args.output_dir, args.repeat)


if __name__ == "__main__":
    main()
