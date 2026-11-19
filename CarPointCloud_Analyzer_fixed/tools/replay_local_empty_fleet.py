"""对本地双雷达采集做只读的离线 HTTP 回放。

只配对原始的 CAM_004/CAM_005 文件。全程不访问运行中的 Analyzer 与 DM8；
每个请求都使用禁用的尺寸存储与档案。
"""

import argparse
import hashlib
import json
import re
import shutil
from datetime import datetime
from pathlib import Path

from fastapi.testclient import TestClient

from carpointcloud_analyzer.infrastructure.persistence import DisabledDimensionStore
from carpointcloud_analyzer.infrastructure.profile_storage import DisabledEmptyVehicleProfileStorage
from carpointcloud_analyzer.composition import create_app


CAPTURE_NAME = re.compile(r"^CAM_(004|005)_(2026\d{4})_(\d{6})_(\d{3})_[^.]+\.pcd$")
MAX_PAIR_DELTA_SECONDS = 16.0


def _timestamp(path: Path) -> datetime | None:
    match = CAPTURE_NAME.match(path.name)
    if match is None:
        return None
    return datetime.strptime("".join(match.groups()[1:]), "%Y%m%d%H%M%S%f")


def discover(captures_root: Path, research_root: Path, legacy_cloud_root: Path):
    """找出严格一一对应的原始配对；无法配对的单帧仍保留在结果中。"""
    groups = {}
    for path in captures_root.rglob("CAM_*.pcd"):
        if _timestamp(path) is not None:
            groups.setdefault(str(path.parent), []).append(path)
    research_day = research_root / "20260907"
    if research_day.is_dir():
        groups[str(research_day)] = [
            path for path in research_day.glob("CAM_*.pcd") if _timestamp(path) is not None
        ]
    pairs, unmatched = [], []
    for folder, paths in sorted(groups.items()):
        aa = sorted((p for p in paths if p.name.startswith("CAM_004_")), key=_timestamp)
        bb = sorted((p for p in paths if p.name.startswith("CAM_005_")), key=_timestamp)
        available_a, available_b = set(aa), set(bb)
        candidates = sorted(
            (abs((_timestamp(b) - _timestamp(a)).total_seconds()), a, b)
            for a in aa for b in bb
        )
        for delta, a, b in candidates:
            if delta > MAX_PAIR_DELTA_SECONDS:
                break
            if a not in available_a or b not in available_b:
                continue
            available_a.remove(a)
            available_b.remove(b)
            pairs.append({
                "id": f"{_timestamp(a):%Y%m%d_%H%M%S_%f}_{a.stem[-8:]}",
                "a": str(a.resolve()), "b": str(b.resolve()),
                "deltaSeconds": delta, "inputUnit": "m", "folder": folder,
            })
        unmatched.extend(str(path) for path in sorted(available_a | available_b,
                                                     key=_timestamp))
    originals = [
        (captures_root / "0903" / "CAM_004_20260903_test1.pcd",
         captures_root / "0903" / "CAM_005_20260903_test1.pcd",
         "20260903_test1", "m"),
        (legacy_cloud_root / "1256_258460_20260910_164433.pcd",
         legacy_cloud_root / "1260_258460_20260910_164531.pcd",
         "20260910_1644_legacy", "mm"),
    ]
    for a, b, pair_id, unit in originals:
        if a.is_file() and b.is_file():
            pairs.append({"id": pair_id, "a": str(a.resolve()), "b": str(b.resolve()),
                          "deltaSeconds": None, "inputUnit": unit, "folder": str(a.parent)})
    return sorted(pairs, key=lambda item: item["id"]), unmatched


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _compact_result(pair, response, body):
    outputs = body.get("outputs") or {}
    status = outputs.get("measurementStatus") or {}
    load = outputs.get("loadStatus") or {}
    fusion = body.get("fusion") or {}
    dims = outputs.get("dimensions") or {}
    return {
        "id": pair["id"], "a": pair["a"], "b": pair["b"],
        "deltaSeconds": pair["deltaSeconds"], "inputUnit": pair["inputUnit"],
        "httpStatus": response.status_code,
        "fusionPassed": fusion.get("qualityPassed"),
        "loadStatus": load.get("status"), "loadReason": load.get("reason"),
        "measurementValid": status.get("valid"),
        "measurementReason": status.get("reason"),
        "calculationSucceeded": status.get("calculationSucceeded"),
        "partial": status.get("partial"),
        "dimensions": dims, "syntheticPointCount": outputs.get("syntheticPointCount"),
        "boundaryMethod": status.get("boundaryMethod"),
        "lengthSource": (status.get("lengthMeasurement") or {}).get("source"),
        "widthSource": (status.get("widthMeasurement") or {}).get("source"),
        "error": body.get("detail") if response.status_code >= 400 else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures-root", type=Path, required=True)
    parser.add_argument("--research-root", type=Path, required=True)
    parser.add_argument("--legacy-cloud-root", type=Path, required=True)
    parser.add_argument("--extrinsic", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--day", action="append", help="Replay only IDs starting with this day")
    args = parser.parse_args()

    pairs, unmatched = discover(args.captures_root, args.research_root, args.legacy_cloud_root)
    root = args.output_dir.resolve()
    root.mkdir(parents=True, exist_ok=True)
    calibration = root / "extrinsic.json"
    if not calibration.exists():
        shutil.copyfile(args.extrinsic, calibration)
    (root / "manifest.json").write_text(
        json.dumps({"pairs": pairs, "unmatched": unmatched}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    results_path = root / "results.jsonl"
    completed = set()
    if results_path.exists():
        for line in results_path.read_text(encoding="utf-8").splitlines():
            completed.add(json.loads(line)["id"])
    selected = [pair for pair in pairs if pair["id"] not in completed
                and (not args.day or any(pair["id"].startswith(day) for day in args.day))]
    if not selected:
        print(f"No pending pairs; discovered={len(pairs)}, unmatched={len(unmatched)}", flush=True)
        return

    staging = root / "staging"
    staging.mkdir(exist_ok=True)
    staged_a, staged_b = staging / "CAM_004.pcd", staging / "CAM_005.pcd"
    app = create_app(
        max_concurrency=1,
        dimension_store=DisabledDimensionStore(),
        empty_profile_storage=DisabledEmptyVehicleProfileStorage(),
        extrinsic_path=str(calibration),
    )
    with TestClient(app) as client, results_path.open("a", encoding="utf-8") as result_stream:
        for index, pair in enumerate(selected, 1):
            source_a, source_b = Path(pair["a"]), Path(pair["b"])
            before = (_sha256(source_a), _sha256(source_b))
            try:
                shutil.copyfile(source_a, staged_a)
                shutil.copyfile(source_b, staged_b)
                response = client.post("/analyze-pair", json={
                    "pcdPathA": str(staged_a), "cameraA": "CAM_004",
                    "pcdPathB": str(staged_b), "cameraB": "CAM_005",
                    "pairId": f"offline-fleet-{pair['id']}",
                    "workflowId": f"offline-fleet-{pair['id']}",
                    "inputUnit": pair["inputUnit"], "plate": "",
                })
                body = response.json()
                result = _compact_result(pair, response, body)
            except Exception as exc:  # 单帧出错不能掩盖后续采集的结果
                result = {"id": pair["id"], "a": pair["a"], "b": pair["b"],
                          "error": f"{type(exc).__name__}: {exc}"}
            after = (_sha256(source_a), _sha256(source_b))
            if before != after:
                raise RuntimeError(f"Original PCD changed during replay: {pair['id']}")
            result["sourceSha256"] = {str(source_a): before[0], str(source_b): before[1]}
            result_stream.write(json.dumps(result, ensure_ascii=False, allow_nan=False) + "\n")
            result_stream.flush()
            carriage = (result.get("dimensions") or {}).get("carriage") or {}
            print(f"{index}/{len(selected)} {pair['id']} "
                  f"load={result.get('loadStatus')} length={carriage.get('length')} "
                  f"width={carriage.get('width')} height={carriage.get('height')} "
                  f"error={result.get('error')}", flush=True)


if __name__ == "__main__":
    main()
