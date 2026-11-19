"""只读重算既有双云融合中间 PCD，记录 Analyzer 内高计算的上下端点。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from carpointcloud_analyzer import Config
from carpointcloud_analyzer import PointCloudAnalyzer
from carpointcloud_analyzer.application import pipeline as pipeline_module
from carpointcloud_analyzer.domain.fusion import Extrinsic


def _config(extrinsic: Extrinsic) -> Config:
    config = Config()
    config.input_unit = "m"
    config.scene_preprocessing_fail_open = False
    config.scene_auto_level_enable = True
    config.scene_auto_level_required = True
    config.floor_require_weighbridge_plane = True
    config.weighbridge_plane_source = tuple(float(x) for x in extrinsic.weighbridge_plane_a)
    config.weighbridge_plane_orientation_verified = True
    config.scene_preprocessing_enable = False
    config.save_clean_pointcloud = False
    config.validate()
    return config


def _box_height_evidence(observed, synthetic, box, floor_z, wall_top, result) -> dict:
    """按产品函数同一空间掩码重算高度两个端点，不改变其返回值。"""
    full = np.vstack((observed, synthetic))
    mid_l = (full[:, 1] >= box.rear_l + .5) & (full[:, 1] <= box.front_l - .5)
    mid_w = (full[:, 0] >= box.w_range[0] + .3) & (full[:, 0] <= box.w_range[1] - .3)
    floor_zone = full[(np.abs(full[:, 2] - floor_z) <= .08) & mid_l & mid_w]
    top_zone = full[(np.abs(full[:, 0] - box.w_range[0]) <= .23)
                    & mid_l & (full[:, 2] >= box.h_range[1] - .15)]
    floor_h = float(np.median(floor_zone[:, 2])) if len(floor_zone) else None
    top_h = float(np.percentile(top_zone[:, 2], 99)) if len(top_zone) else None
    # 仅报告底部空间的原始扫描高度分布，用来检查是否另有更高的底板层。
    raw_mid = observed[((observed[:, 1] >= box.rear_l + .5)
                        & (observed[:, 1] <= box.front_l - .5)
                        & (observed[:, 0] >= box.w_range[0] + .3)
                        & (observed[:, 0] <= box.w_range[1] - .3))]
    low = float(box.h_range[0])
    edges = np.arange(low, low + .8, .04)
    counts, _ = np.histogram(raw_mid[:, 2], bins=edges)
    peaks = sorted(range(len(counts)), key=lambda index: int(counts[index]), reverse=True)[:5]
    return {
        "outerBottomH": low,
        "outerTopH": float(box.h_range[1]),
        "profileFloorAnchorH": float(floor_z),
        "profileWallTopH": float(wall_top),
        "innerFloorH": floor_h,
        "innerRimTopH": top_h,
        "calculatedHeightM": result.get("heightM") if result else None,
        "floorZonePointCount": int(len(floor_zone)),
        "topZonePointCount": int(len(top_zone)),
        "rawInteriorBottomPeaks": [
            {"hLow": round(float(edges[index]), 4),
             "hHigh": round(float(edges[index + 1]), 4),
             "pointCount": int(counts[index])}
            for index in peaks
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--extrinsic", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)

    extrinsic = Extrinsic.load(str(args.extrinsic))
    rows = [json.loads(line) for line in args.results.read_text(encoding="utf-8").splitlines()]
    original_inner = pipeline_module.completed_inner_dimensions
    findings = []
    with args.output.open("w", encoding="utf-8") as stream:
        for index, row in enumerate(rows, 1):
            response = json.loads(Path(row["responsePath"]).read_text(encoding="utf-8"))
            analyzer = PointCloudAnalyzer(_config(extrinsic))
            evidence = {}
            original_calculate = analyzer.dim_calc.calculate

            def capture_inner(observed, synthetic, box, floor_z, wall_top):
                result = original_inner(observed, synthetic, box, floor_z, wall_top)
                evidence["box"] = _box_height_evidence(
                    observed, synthetic, box, floor_z, wall_top, result)
                return result

            def capture_long(all_points, carriage_points, split, profile):
                evidence["long"] = {
                    "floorH": float(profile.floor_z),
                    "leftTopH": profile.wall_top_left,
                    "rightTopH": profile.wall_top_right,
                    "adoptedTopH": float(profile.wall_top),
                    "calculatedHeightM": float(profile.wall_depth),
                    "finalCarriagePointCount": int(len(carriage_points)),
                }
                return original_calculate(all_points, carriage_points, split, profile)

            pipeline_module.completed_inner_dimensions = capture_inner
            analyzer.dim_calc.calculate = capture_long
            try:
                replay = analyzer.analyze(response["pcd_path"])
            finally:
                pipeline_module.completed_inner_dimensions = original_inner
                analyzer.dim_calc.calculate = original_calculate
            expected = (response.get("outputs") or {}).get("dimensions") or {}
            measured = replay.get("dimensions") or {}
            record = {
                "id": row["id"],
                "replayedHeightM": (measured.get("carriage") or {}).get("height"),
                "batchHeightM": (expected.get("carriage") or {}).get("height"),
                "heightDeltaM": None,
                **evidence,
            }
            if record["replayedHeightM"] is not None and record["batchHeightM"] is not None:
                record["heightDeltaM"] = record["replayedHeightM"] - record["batchHeightM"]
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
            findings.append(record)
            print(f"{index:02d}/{len(rows)} {row['id']} height={record['replayedHeightM']} "
                  f"delta={record['heightDeltaM']}", flush=True)
    print(f"captured={len(findings)} output={args.output}", flush=True)


if __name__ == "__main__":
    main()
