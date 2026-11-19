#!/usr/bin/env python3
"""用一份真实 PCD 跑通已部署的 Analyzer，并清理测试记录与文件。"""

from __future__ import annotations

import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path


DATA_ROOT = Path("/data/carpointcloud").resolve()
APP_ROOT = Path("/opt/carpointcloud-analyzer/app")
SITE_PACKAGES = Path("/opt/carpointcloud-analyzer/venv/lib/python3.11/site-packages")
ENV_PATH = Path("/etc/carpointcloud-analyzer.env")


def load_env() -> None:
    for raw in ENV_PATH.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.removeprefix("export ").split("=", 1)
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "'\"":
            value = value[1:-1]
        os.environ[key.strip()] = value


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: server-functional-smoke.py /data/carpointcloud/.../sample.pcd")
    source = Path(sys.argv[1]).resolve()
    if DATA_ROOT not in source.parents or source.suffix.lower() != ".pcd" or not source.is_file():
        raise ValueError("source must be an existing PCD below /data/carpointcloud")

    load_env()
    sys.path[:0] = [str(SITE_PACKAGES), str(APP_ROOT)]
    from carpointcloud_analyzer.infrastructure.persistence import Dm8DimensionStore

    request_body = json.dumps(
        {
            "pcdPath": str(source),
            "camera": "CAM_004",
            "plate": "粤AMECT5",
            "inputUnit": "auto",
            "saveVisualization": False,
        },
        ensure_ascii=False,
    ).encode("utf-8")
    request = urllib.request.Request(
        "http://127.0.0.1:8010/analyze",
        data=request_body,
        method="POST",
        headers={"Content-Type": "application/json; charset=utf-8"},
    )
    status = 0
    payload = None
    record_id = None
    files = {source}
    store = Dm8DimensionStore.from_env()
    cleanup: dict[str, int] = {}
    try:
        try:
            with urllib.request.urlopen(request, timeout=180) as response:
                status = response.status
                payload = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            status = exc.code
            payload = json.loads(exc.read().decode("utf-8"))

        if status != 200 or not isinstance(payload, dict) or payload.get("success") is not True:
            raise RuntimeError(f"analyzer returned HTTP {status}: {payload}")
        for value in payload.get("outputs", {}).values():
            if isinstance(value, str) and value:
                path = Path(value).resolve()
                if DATA_ROOT in path.parents:
                    files.add(path)
        persistence = payload.get("persistence", {})
        child_counts = None
        measurement_status = None
        load_status = None
        result_json = payload.get("outputs", {}).get("json")
        if result_json:
            result_payload = json.loads(Path(result_json).read_text(encoding="utf-8"))
            measurement_status = result_payload.get("measurementStatus")
            load_status = result_payload.get("loadStatus")

        if "recordId" in persistence:
            record_id = int(persistence["recordId"])
            connection = store._connect()
            prefix = f"{store._settings.schema.upper()}." if store._settings.schema else ""
            cursor = connection.cursor()
            try:
                cursor.execute(
                    f"SELECT LicensePlate, CameraIdentifier, DetectStatus FROM {prefix}VehicleDimensionRecord WHERE Id = ?",
                    record_id,
                )
                row = cursor.fetchone()
                if not row or tuple(row) != ("粤AMECT5", "CAM_004", "success"):
                    raise RuntimeError(f"unexpected DM8 row: {row}")
                child_counts = {}
                for table in ("VehicleDimensionRib", "VehicleDimensionWire", "VehicleDimensionHighLowPlate"):
                    cursor.execute(f"SELECT COUNT(*) FROM {prefix}{table} WHERE DimensionRecordId = ?", record_id)
                    child_counts[table] = int(cursor.fetchone()[0])
            finally:
                cursor.close()
                connection.close()
        elif payload.get("integrationPayloads") != {} or not isinstance(measurement_status, dict):
            raise RuntimeError(f"missing persistence without an explicit measurement boundary: {payload}")

        print(json.dumps({
            "passed": True,
            "httpStatus": status,
            "elapsedMs": payload.get("elapsed_ms"),
            "recordId": record_id,
            "detectStatus": "success" if record_id is not None else "not-persisted-by-design",
            "childCounts": child_counts,
            "measurementStatus": measurement_status,
            "loadStatus": load_status,
            "outputs": payload.get("outputs"),
        }, ensure_ascii=False, indent=2))
        return 0
    finally:
        if record_id is not None:
            connection = store._connect()
            prefix = f"{store._settings.schema.upper()}." if store._settings.schema else ""
            cursor = connection.cursor()
            try:
                for table in ("VehicleDimensionRib", "VehicleDimensionWire", "VehicleDimensionHighLowPlate"):
                    cursor.execute(f"DELETE FROM {prefix}{table} WHERE DimensionRecordId = ?", record_id)
                    cleanup[table] = max(int(cursor.rowcount), 0)
                cursor.execute(f"DELETE FROM {prefix}VehicleDimensionRecord WHERE Id = ?", record_id)
                cleanup["VehicleDimensionRecord"] = max(int(cursor.rowcount), 0)
                connection.commit()
            except Exception:
                connection.rollback()
                raise
            finally:
                cursor.close()
                connection.close()
        for path in files:
            try:
                resolved = path.resolve()
                if DATA_ROOT in resolved.parents and resolved.is_file():
                    resolved.unlink()
            except OSError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
