"""将既有 Empty 清单的原始双云复制到隔离目录，并用当前 Analyzer HTTP 服务逐对重放。

选择清单只决定输入范围；本轮 Empty/Loaded、尺寸、错误均完全取自新的服务响应。
不调用正在运行的进程、真实数据库或永久空车档案。
"""

import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from fastapi.testclient import TestClient

from carpointcloud_analyzer.infrastructure.persistence import DisabledDimensionStore
from carpointcloud_analyzer.infrastructure.profile_storage import DisabledEmptyVehicleProfileStorage
from carpointcloud_analyzer.composition import create_app


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _selected_pairs(selection: Path) -> list[dict]:
    """固定待重放文件集；绝不复用旧尺寸作为本轮结果。"""
    pairs = [json.loads(line) for line in selection.read_text(encoding="utf-8").splitlines()]
    pairs = [item for item in pairs if item.get("loadStatus") == "Empty"]
    if len(pairs) != 23 or len({item["id"] for item in pairs}) != 23:
        raise ValueError(f"期望 23 对唯一 Empty 输入，实际 {len(pairs)} 对")
    if any(item.get("inputUnit") != "m" for item in pairs):
        raise ValueError("本批 23 对必须全是米制原始 PCD")
    return sorted(pairs, key=lambda item: item["id"])


def _code_fingerprint(project_root: Path) -> dict[str, str]:
    """记录当前未提交工作树的源码哈希，避免仅凭 HEAD 误认运行版本。"""
    files = sorted((project_root / "carpointcloud_analyzer").rglob("*.py"))
    files.append(Path(__file__).resolve())
    return {str(path.relative_to(project_root)).replace("\\", "/"): _sha256(path)
            for path in files}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection-results", type=Path, required=True)
    parser.add_argument("--extrinsic", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    root = args.output_dir.resolve()
    if root.exists():
        raise FileExistsError(f"为避免混合不同版本的结果，输出目录必须不存在：{root}")
    pairs = _selected_pairs(args.selection_results)
    project_root = Path(__file__).resolve().parents[1]
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=project_root, check=True,
        capture_output=True, text=True,
    ).stdout.strip()
    root.mkdir(parents=True)
    calibration = root / "extrinsic.json"
    shutil.copy2(args.extrinsic.resolve(), calibration)
    manifest = {
        "createdAtUtc": datetime.now(timezone.utc).isoformat(),
        "selectionSource": str(args.selection_results.resolve()),
        "selectionRule": "previous Analyzer loadStatus == Empty; rerun output is not filtered",
        "pairCount": len(pairs),
        "gitHead": head,
        "codeSha256": _code_fingerprint(project_root),
        "extrinsicSha256": _sha256(calibration),
        "databaseEnabled": False,
        "archiveEnabled": False,
        "pairs": [],
    }
    for index, pair in enumerate(pairs, 1):
        folder = root / "cases" / f"{index:02d}_{pair['id']}"
        folder.mkdir(parents=True)
        copied = {}
        for camera, key in (("CAM_004", "a"), ("CAM_005", "b")):
            source = Path(pair[key]).resolve()
            expected = pair.get("sourceSha256", {}).get(str(source))
            before = _sha256(source)
            if expected is not None and expected != before:
                raise ValueError(f"原始文件与选择清单的哈希不一致：{source}")
            target = folder / source.name
            shutil.copy2(source, target)
            if _sha256(target) != before or _sha256(source) != before:
                raise IOError(f"复制或复制期间原始文件发生变化：{source}")
            copied[key] = {"camera": camera, "original": str(source),
                           "copy": str(target), "sha256": before,
                           "bytes": target.stat().st_size}
        manifest["pairs"].append({"id": pair["id"], "inputUnit": pair["inputUnit"],
                                   "previousLoadStatus": pair["loadStatus"], **copied})
    (root / "input_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")

    app = create_app(
        max_concurrency=1,
        dimension_store=DisabledDimensionStore(),
        empty_profile_storage=DisabledEmptyVehicleProfileStorage(),
        extrinsic_path=str(calibration),
    )
    with TestClient(app) as client, (root / "results.jsonl").open("w", encoding="utf-8") as stream:
        for index, pair in enumerate(manifest["pairs"], 1):
            case_dir = Path(pair["a"]["copy"]).parent
            try:
                response = client.post("/analyze-pair", json={
                    "pcdPathA": pair["a"]["copy"], "cameraA": "CAM_004",
                    "pcdPathB": pair["b"]["copy"], "cameraB": "CAM_005",
                    "pairId": f"offline-empty23-{pair['id']}",
                    "workflowId": f"offline-empty23-{pair['id']}",
                    "inputUnit": pair["inputUnit"], "plate": "",
                })
                body = response.json()
                (case_dir / "analyzer_response.json").write_text(
                    json.dumps(body, ensure_ascii=False, indent=2, allow_nan=False),
                    encoding="utf-8",
                )
                outputs = body.get("outputs") or {}
                status = outputs.get("measurementStatus") or {}
                dimensions = outputs.get("dimensions")
                row = {
                    "id": pair["id"], "httpStatus": response.status_code,
                    "fusionPassed": (body.get("fusion") or {}).get("qualityPassed"),
                    "loadStatus": (outputs.get("loadStatus") or {}).get("status"),
                    "measurementValid": status.get("valid"),
                    "measurementReason": status.get("reason"),
                    "dimensions": dimensions,
                    "pcdPath": body.get("pcd_path"),
                    "responsePath": str(case_dir / "analyzer_response.json"),
                    "error": body.get("detail") if response.status_code >= 400 else None,
                }
            except Exception as exc:  # 单对失败也须保留证据并继续其他输入
                row = {"id": pair["id"], "httpStatus": None, "dimensions": None,
                       "error": f"{type(exc).__name__}: {exc}"}
            for key in ("a", "b"):
                item = pair[key]
                if _sha256(Path(item["original"])) != item["sha256"] or _sha256(Path(item["copy"])) != item["sha256"]:
                    raise RuntimeError(f"重放期间原始或复制 PCD 发生变化：{pair['id']}")
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
            carriage = (row.get("dimensions") or {}).get("carriage") or {}
            print(f"{index:02d}/23 {pair['id']} "
                  f"load={row.get('loadStatus')} "
                  f"L/W/H={carriage.get('length')}/{carriage.get('width')}/{carriage.get('height')} "
                  f"error={row.get('error')}", flush=True)


if __name__ == "__main__":
    main()
