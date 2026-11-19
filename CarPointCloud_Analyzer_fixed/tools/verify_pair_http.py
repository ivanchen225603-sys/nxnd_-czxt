"""以实际 HTTP 编排离线回放双雷达；禁用数据库/档案，不接触运行中的服务。"""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

from fastapi.testclient import TestClient

from carpointcloud_analyzer.infrastructure.persistence import DisabledDimensionStore
from carpointcloud_analyzer.infrastructure.profile_storage import DisabledEmptyVehicleProfileStorage
from carpointcloud_analyzer.composition import create_app


def validate_dimension_output(body, name):
    """核对成功/逐项厘米载荷，不再依赖人工审核状态。"""
    outputs = body['outputs']
    status = outputs.get('measurementStatus', {})
    payload = body.get('integrationPayloads', {}).get('vehicleDimensions')
    if not isinstance(payload, dict):
        if status.get('calculationSucceeded') is True:
            raise AssertionError(f'{name}: 尺寸计算成功但缺少入库载荷')
        return
    if (payload.get('detectStatus') != 'success' or not status.get('calculationSucceeded')
            or 'reviewStatus' in status or 'requiresManualReview' in status):
        raise AssertionError(f'{name}: 成功载荷仍带人工审核状态或缺少计算状态')
    carriage = (outputs.get('dimensions') or {}).get('carriage') or {}
    for meter_key, cm_key in [('length', 'trailerLength'), ('width', 'trailerWidth'),
                              ('height', 'trailerBodyHeight')]:
        value = carriage.get(meter_key)
        expected = round(value * 100, 1) if value is not None else None
        if payload.get('dimensions', {}).get(cm_key) != expected:
            raise AssertionError(f'{name}: {cm_key} 与米制计算值不一致')
    if status.get('valid') is False and status.get('partial') is not True:
        raise AssertionError(f'{name}: 未通过且非逐项结果不能入库')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", nargs=3, action="append", required=True, metavar=("NAME", "A", "B"))
    parser.add_argument("--extrinsic", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    root = Path(args.output_dir).resolve()
    root.mkdir(parents=True, exist_ok=False)
    calibration = root / "extrinsic.json"
    shutil.copyfile(args.extrinsic, calibration)
    app = create_app(max_concurrency=1,
                     dimension_store=DisabledDimensionStore(),
                     empty_profile_storage=DisabledEmptyVehicleProfileStorage(),
                     extrinsic_path=str(calibration))
    report = {"offline": True, "databaseEnabled": False, "cases": []}
    with TestClient(app) as client:
        for index, (name, source_a, source_b) in enumerate(args.case):
            paths, hashes = [], {}
            for camera, source in [("CAM_004", source_a), ("CAM_005", source_b)]:
                source = Path(source).resolve()
                hashes[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
                target = root / f"case_{index}_{camera}.pcd"
                shutil.copyfile(source, target)
                paths.append(str(target))
            response = client.post("/analyze-pair", json={
                "pcdPathA": paths[0], "cameraA": "CAM_004",
                "pcdPathB": paths[1], "cameraB": "CAM_005",
                "pairId": f"offline-http-{index}", "workflowId": f"offline-http-{index}",
                "inputUnit": "m", "plate": "",
            })
            response.raise_for_status()
            body = response.json()
            (root / f"case_{index}_http_response.json").write_text(
                json.dumps(body, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
            outputs = body["outputs"]
            if not outputs.get("dimensions"):
                raise AssertionError(f"{name}: HTTP 输出缺少计算尺寸")
            if not body.get("fusion", {}).get("qualityPassed"):
                raise AssertionError(f"{name}: 未通过双云融合")
            status = outputs.get("measurementStatus", {})
            validate_dimension_output(body, name)
            for source, digest in hashes.items():
                if hashlib.sha256(Path(source).read_bytes()).hexdigest() != digest:
                    raise AssertionError(f"原始文件发生变化：{source}")
            item = {"name": name, "httpStatus": response.status_code, "sourceSha256": hashes,
                    "dimensions": outputs["dimensions"], "measurementStatus": status,
                    "reviewFieldCount": len(outputs.get("reviewMeasurements") or {})}
            report["cases"].append(item)
            print(json.dumps(item, ensure_ascii=False), flush=True)
    (root / "http_replay_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
