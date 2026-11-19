"""用 Analyzer 正式双云入口重跑 22 对原始 PCD，并保存程序生成的最终 PCD。

只使用旧清单确定输入身份，不复用任何旧尺寸或状态。原始 PCD 只读，
数据库与永久档案禁用；仅打开 Analyzer 正式使用的最终 PCD 写出开关。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from fastapi.testclient import TestClient

from carpointcloud_analyzer import PointCloudAnalyzer
from carpointcloud_analyzer.infrastructure.persistence import DisabledDimensionStore
from carpointcloud_analyzer.infrastructure.profile_storage import DisabledEmptyVehicleProfileStorage
from carpointcloud_analyzer.composition import create_app


EXCLUDED_ID = "20260926_112627_252000_b0f2289f"


def file_sha256(path: Path) -> str:
    """逐块计算文件摘要，不改变文件。"""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def selected_pairs(manifest: dict, pair_ids: list[str] | None = None) -> list[dict]:
    """仅排除已确认坏帧；不按本轮结果反向筛选输入。"""
    all_pairs = manifest["pairs"]
    pairs = [pair for pair in all_pairs if pair["id"] != EXCLUDED_ID]
    if (len(all_pairs) != 23 or len(pairs) != 22
            or len({pair["id"] for pair in all_pairs}) != 23
            or not any(pair["id"] == EXCLUDED_ID for pair in all_pairs)):
        raise ValueError("输入清单不是预期的 23 对、排除 1 对后保留 22 对")
    if any(pair["inputUnit"] != "m" for pair in pairs):
        raise ValueError("22 对输入必须均为米制")
    if pair_ids is not None:
        requested = set(pair_ids)
        if not requested or len(requested) != len(pair_ids):
            raise ValueError("指定的输入 ID 必须非空且不重复")
        unknown = requested - {pair["id"] for pair in pairs}
        if unknown:
            raise ValueError(f"输入 ID 不在 22 对清单中：{sorted(unknown)}")
        pairs = [pair for pair in pairs if pair["id"] in requested]
    return sorted(pairs, key=lambda pair: pair["id"])


def source_entry(pair: dict, camera: str, case_dir: Path) -> dict:
    """校验原始 PCD 摘要并复制到隔离目录。"""
    item = pair[camera]
    original = Path(item["original"]).resolve()
    expected = item["sha256"]
    if file_sha256(original) != expected:
        raise ValueError(f"原始 PCD 摘要与历史输入清单不符：{original}")
    copy = case_dir / original.name
    shutil.copy2(original, copy)
    if file_sha256(copy) != expected or file_sha256(original) != expected:
        raise IOError(f"PCD 复制校验失败：{original}")
    return {"original": str(original), "copy": str(copy), "sha256": expected,
            "bytes": copy.stat().st_size}


def code_fingerprint(project_root: Path) -> dict[str, str]:
    """记录运行时源码内容，覆盖未提交的工作区修改。"""
    files = sorted((project_root / "carpointcloud_analyzer").rglob("*.py"))
    files.append(Path(__file__).resolve())
    return {str(path.relative_to(project_root)).replace("\\", "/"): file_sha256(path)
            for path in files}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--pair-id", action="append", help="只重跑指定 ID；可重复，省略则重跑 22 对")
    args = parser.parse_args()

    root = args.output_dir.resolve()
    if root.exists():
        raise FileExistsError(f"输出目录必须全新，防止混入旧结果：{root}")
    manifest_path = args.manifest.resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    pairs = selected_pairs(manifest, args.pair_id)
    source_extrinsic = manifest_path.parent / "extrinsic.json"
    if file_sha256(source_extrinsic) != manifest["extrinsicSha256"]:
        raise ValueError("外参与历史输入清单的摘要不符")

    project_root = Path(__file__).resolve().parents[1]
    root.mkdir(parents=True)
    extrinsic = root / "extrinsic.json"
    shutil.copy2(source_extrinsic, extrinsic)
    if file_sha256(extrinsic) != manifest["extrinsicSha256"]:
        raise IOError("隔离外参复制校验失败")

    prepared = []
    for index, pair in enumerate(pairs, 1):
        case_dir = root / "cases" / f"{index:02d}_{pair['id']}"
        case_dir.mkdir(parents=True)
        prepared.append({"id": pair["id"], "caseDir": str(case_dir),
                         "a": source_entry(pair, "a", case_dir),
                         "b": source_entry(pair, "b", case_dir)})

    head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=project_root,
                          check=True, capture_output=True, text=True).stdout.strip()
    run_record = {
        "createdAtUtc": datetime.now(timezone.utc).isoformat(),
        "selectionManifest": str(manifest_path),
        "excludedId": EXCLUDED_ID,
        "selectedIds": [pair["id"] for pair in pairs],
        "gitHead": head,
        "codeSha256": code_fingerprint(project_root),
        "extrinsicSha256": file_sha256(extrinsic),
        "databaseEnabled": False,
        "finalPcdWritingEnabled": True,
        "pairs": prepared,
    }
    (root / "run_record.json").write_text(
        json.dumps(run_record, ensure_ascii=False, indent=2), encoding="utf-8")

    # 真实 DB 和永久档案均禁用；只将 Analyzer 的最终 PCD 写出开关设为
    # 正式归档模式的同一值，不注入车牌、尺寸或点云，不修改算法响应。
    def analyzer_factory(config):
        config.save_clean_pointcloud = True
        return PointCloudAnalyzer(config)

    app = create_app(
        max_concurrency=1, extrinsic_path=str(extrinsic),
        analyzer_factory=analyzer_factory,
        dimension_store=DisabledDimensionStore(),
        empty_profile_storage=DisabledEmptyVehicleProfileStorage(),
    )
    rows = []
    with TestClient(app) as client, (root / "results.jsonl").open("w", encoding="utf-8") as stream:
        for index, pair in enumerate(prepared, 1):
            case_dir = Path(pair["caseDir"])
            try:
                response = client.post("/analyze-pair", json={
                    "pcdPathA": pair["a"]["copy"], "cameraA": "CAM_004",
                    "pcdPathB": pair["b"]["copy"], "cameraB": "CAM_005",
                    "pairId": f"empty22-final-{pair['id']}",
                    "workflowId": f"empty22-final-{pair['id']}",
                    "inputUnit": "m", "plate": "",
                })
                body = response.json()
                raw_response = response.text
                status_code = response.status_code
                error = None
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
                body = {"transportError": error}
                raw_response = json.dumps(body, ensure_ascii=False)
                status_code = None
            response_path = case_dir / "analyzer_response.json"
            response_path.write_text(raw_response, encoding="utf-8")
            outputs = body.get("outputs") or {}
            load_status = (outputs.get("loadStatus") or {}).get("status")
            final_path = outputs.get("pcd") or ""
            final_sha = None
            if final_path:
                final_file = Path(final_path).resolve()
                final_file.relative_to(root)
                final_sha = file_sha256(final_file)
            for side in ("a", "b"):
                item = pair[side]
                if (file_sha256(Path(item["original"])) != item["sha256"]
                        or file_sha256(Path(item["copy"])) != item["sha256"]):
                    raise RuntimeError(f"输入 PCD 在运行中发生变化：{pair['id']}")
            row = {
                "id": pair["id"], "httpStatus": status_code,
                "error": error if error else body.get("detail") if status_code != 200 else None,
                "fusionPassed": (body.get("fusion") or {}).get("qualityPassed"),
                "loadStatus": load_status,
                "measurementValid": (outputs.get("measurementStatus") or {}).get("valid"),
                "dimensions": outputs.get("dimensions"),
                "pcdPointCount": outputs.get("pcdPointCount"),
                "syntheticPointCount": outputs.get("syntheticPointCount"),
                "finalPcdPath": final_path or None,
                "finalPcdSha256": final_sha,
                "responsePath": str(response_path),
            }
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
            rows.append(row)
            print(f"{index:02d}/{len(prepared)} {pair['id']} http={status_code} "
                  f"fusion={row['fusionPassed']} load={load_status} "
                  f"finalPoints={row['pcdPointCount']} error={row['error']}", flush=True)

    summary = {
        "sourceCodeUnchanged": code_fingerprint(project_root) == run_record["codeSha256"],
        "pairCount": len(rows),
        "http200Count": sum(row["httpStatus"] == 200 for row in rows),
        "fusionPassedCount": sum(row["fusionPassed"] is True for row in rows),
        "emptyCount": sum(row["loadStatus"] == "Empty" for row in rows),
        "finalPcdCount": sum(bool(row["finalPcdPath"]) for row in rows),
        "missingFinalPcdIds": [row["id"] for row in rows if not row["finalPcdPath"]],
        "resultsPath": str(root / "results.jsonl"),
    }
    (root / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
