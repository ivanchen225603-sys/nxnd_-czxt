#!/usr/bin/env python3
"""生产环境冒烟测试：逐次扫描历史与最新空车档案。"""

from __future__ import annotations

import hashlib
import json
import os
import grp
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any


ENV_PATH = Path("/etc/carpointcloud-analyzer.env")
WEBAPI_ENV_PATH = Path(
    "/opt/carpointcloud/CarPointCloudWebApi/"
    "CarPointCloudWebApi_V1.0.1/carpointcloudwebapi.env"
)
ANALYZER_URL = "http://127.0.0.1:8010"
WEBAPI_URL = "http://127.0.0.1:5031"
PLATE = "TESTV7EMPTY"
SOURCE_PATHS: set[Path] = set()
GENERATED_FIXTURE_DIR: Path | None = None
WORKFLOW_IDS: list[str] = []


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


def read_env_value(path: Path, key: str) -> str:
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        current_key, value = line.removeprefix("export ").split("=", 1)
        if current_key.strip() != key:
            continue
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "'\"":
            value = value[1:-1]
        return value
    raise KeyError(f"missing {key} in {path}")


def http_json(
    method: str,
    url: str,
    payload: dict[str, Any] | None = None,
    headers: dict[str, str] | None = None,
    timeout: int = 240,
) -> tuple[int, dict[str, Any]]:
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    request_headers = {"Accept": "application/json", **(headers or {})}
    if body is not None:
        request_headers["Content-Type"] = "application/json; charset=utf-8"
    request = urllib.request.Request(
        url, data=body, method=method, headers=request_headers
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode("utf-8", errors="replace")
        try:
            error = json.loads(raw)
        except json.JSONDecodeError:
            error = {"raw": raw}
        return exc.code, error


def analyze_pair(
    pcd_a: str,
    pcd_b: str,
    extrinsic: str,
    pair_id: str,
    workflow_id: str,
    captured_at: datetime,
) -> dict[str, Any]:
    status, result = http_json(
        "POST",
        f"{ANALYZER_URL}/analyze-pair",
        {
            "pcdPathA": pcd_a,
            "cameraA": "CAM_004",
            "pcdPathB": pcd_b,
            "cameraB": "CAM_005",
            "pairId": pair_id,
            "workflowId": workflow_id,
            "extrinsicPath": extrinsic,
            "plate": PLATE,
            "inputUnit": "m",
            "saveVisualization": False,
            "capturedAt": captured_at.astimezone().isoformat(),
        },
    )
    if status != 200 or result.get("success") is not True:
        raise RuntimeError(f"analyze-pair failed: HTTP {status}: {result}")
    if result.get("fusion", {}).get("qualityPassed") is not True:
        raise RuntimeError(f"fusion quality gate did not pass: {result.get('fusion')}")
    profile = result.get("persistence", {}).get("emptyVehicleProfile")
    if not isinstance(profile, dict):
        raise RuntimeError(f"empty profile was not persisted: {result}")
    return result


def generate_synthetic_fixture() -> tuple[str, str, str]:
    """生成两台雷达观测结果完全一致的确定性空车场景。"""
    import numpy as np
    import open3d as o3d

    global GENERATED_FIXTURE_DIR
    run_id = datetime.now().strftime("%Y%m%d%H%M%S%f")
    root = Path(os.environ["CARPOINT_SERVICE_ALLOWED_ROOT"]).resolve()
    fixture_dir = root / "integration-tests" / f"empty-profile-e2e-{run_id}"
    fixture_dir.mkdir(parents=True, exist_ok=False)
    shared_gid = grp.getgrnam("carpointcloud-shared").gr_gid
    os.chown(fixture_dir, -1, shared_gid)
    fixture_dir.chmod(0o2770)
    GENERATED_FIXTURE_DIR = fixture_dir

    rng = np.random.default_rng(20260720)

    def slab(n, x0, x1, y0, y1, z0, z1):
        return np.column_stack(
            [
                rng.uniform(x0, x1, n),
                rng.uniform(y0, y1, n),
                rng.uniform(z0, z1, n),
            ]
        )

    floor_z = 1.20
    ground = slab(90000, -20, 20, -10, 10, -0.01, 0.01)
    ground = ground[
        ~(
            (ground[:, 0] > -4.5)
            & (ground[:, 0] < 6.7)
            & (np.abs(ground[:, 1]) < 1.6)
        )
    ]
    parts = [
        ground,
        slab(12000, -4, 4, -1.25, 1.25, floor_z - 0.01, floor_z + 0.01),
    ]
    for wall_y in (-1.25, 1.25):
        parts.append(slab(7000, -4, 4, wall_y - 0.02, wall_y + 0.02, floor_z, 2.6))
    for wall_x in (-4, 4):
        parts.append(slab(2500, wall_x - 0.02, wall_x + 0.02, -1.25, 1.25, floor_z, 2.6))
    parts.append(slab(6000, 4.5, 6.5, -1.1, 1.1, 0.5, 3.0))
    parts.append(slab(9000, -18, 18, 9.0, 9.05, 0.0, 1.1))
    parts.append(slab(1600, 12.0, 12.1, 5.0, 5.1, 0.0, 6.0))
    scene = np.vstack(parts)

    pcd_a = fixture_dir / "CAM_004-empty-truck.pcd"
    pcd_b = fixture_dir / "CAM_005-empty-truck.pcd"
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(scene))
    if not o3d.io.write_point_cloud(str(pcd_a), cloud, write_ascii=False):
        raise IOError(f"failed to write {pcd_a}")
    if not o3d.io.write_point_cloud(str(pcd_b), cloud, write_ascii=False):
        raise IOError(f"failed to write {pcd_b}")

    extrinsic = fixture_dir / "identity-trusted-extrinsic.json"
    extrinsic.write_text(
        json.dumps(
            {
                "version": "v1",
                "frame": "B_to_A",
                "cameraA": "CAM_004",
                "cameraB": "CAM_005",
                "matrix": [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                "source": "GENERATED_E2E_TEST_ONLY",
                "baselineResidual": 0.001,
                "weighbridgePlaneA": None,
                "quality": {
                    "fitness": 1.0,
                    "inlierRmse": 0.001,
                    "correspondenceCount": int(len(scene)),
                    "degenerateAxes": [],
                    "conditionNumber": 1.0,
                    "trustworthy": True,
                    "warnings": [],
                },
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    return str(pcd_a), str(pcd_b), str(extrinsic)


def connect_db():
    import dmPython

    return dmPython.connect(
        user=os.environ["CARPOINT_DB_USER"],
        password=os.environ["CARPOINT_DB_PASSWORD"],
        server=os.environ.get("CARPOINT_DB_HOST", "127.0.0.1"),
        port=int(os.environ.get("CARPOINT_DB_PORT", "5236")),
        autoCommit=False,
        connection_timeout=10,
    )


def read_database_state(connection) -> dict[str, Any]:
    schema = os.environ.get("CARPOINT_DB_SCHEMA", "CarPointCloud").upper()
    cursor = connection.cursor()
    try:
        cursor.execute(
            f"SELECT Id, IdempotencyKey FROM {schema}.VehicleDimensionRecord "
            "WHERE LicensePlate = ? ORDER BY Id",
            PLATE,
        )
        records = [(int(row[0]), str(row[1])) for row in cursor.fetchall()]
        cursor.execute(
            f"SELECT DimensionRecordId, PointCloudPath, PointCloudSha256, "
            f"PointCloudSize, PointCount, ScanTime FROM {schema}.EmptyVehicleProfile "
            "WHERE LicensePlate = ?",
            PLATE,
        )
        row = cursor.fetchone()
        profile = None
        if row:
            profile = {
                "dimensionRecordId": int(row[0]),
                "pointCloudPath": str(row[1]),
                "pointCloudSha256": str(row[2]),
                "pointCloudSize": int(row[3]),
                "pointCount": int(row[4]),
                "scanTime": row[5].isoformat(),
            }
        return {"history": records, "profile": profile}
    finally:
        cursor.close()


def insert_test_arrivals(connection, workflow_ids: list[str]) -> None:
    schema = os.environ.get("CARPOINT_DB_SCHEMA", "CarPointCloud").upper()
    now = datetime.now()
    cursor = connection.cursor()
    try:
        for index, workflow_id in enumerate(workflow_ids, start=1):
            cursor.execute(
                f"INSERT INTO {schema}.VehicleArrival "
                "(EventId, DeviceId, CameraIdentifier, EventType, SessionId, "
                "LicensePlate, PlateStatus, PlateSource, PlateCapturedAt, ArrivalTime, "
                "RadarStatus, RadarError, CreatedAt, UpdatedAt) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                f"event-{workflow_id}",
                "E2E_TEST_DEVICE",
                "CAM_004",
                "ARRIVAL",
                workflow_id,
                PLATE,
                "SUCCEEDED",
                "E2E_TEST",
                now,
                now - timedelta(seconds=index),
                "SUCCEEDED",
                None,
                now,
                now,
            )
        connection.commit()
    except Exception:
        connection.rollback()
        raise
    finally:
        cursor.close()


def cleanup_database(connection) -> list[str]:
    schema = os.environ.get("CARPOINT_DB_SCHEMA", "CarPointCloud").upper()
    cursor = connection.cursor()
    paths: list[str] = []
    try:
        cursor.execute(
            f"SELECT PointCloudPath FROM {schema}.EmptyVehicleProfile WHERE LicensePlate = ?",
            PLATE,
        )
        paths.extend(str(row[0]) for row in cursor.fetchall())
        cursor.execute(
            f"SELECT Id FROM {schema}.VehicleDimensionRecord WHERE LicensePlate = ?",
            PLATE,
        )
        record_ids = [int(row[0]) for row in cursor.fetchall()]
        cursor.execute(
            f"DELETE FROM {schema}.EmptyVehicleProfile WHERE LicensePlate = ?",
            PLATE,
        )
        for record_id in record_ids:
            for table in (
                "VehicleDimensionRib",
                "VehicleDimensionWire",
                "VehicleDimensionHighLowPlate",
            ):
                cursor.execute(
                    f"DELETE FROM {schema}.{table} WHERE DimensionRecordId = ?",
                    record_id,
                )
        cursor.execute(
            f"DELETE FROM {schema}.VehicleDimensionRecord WHERE LicensePlate = ?",
            PLATE,
        )
        for workflow_id in WORKFLOW_IDS:
            cursor.execute(
                f"DELETE FROM {schema}.VehicleArrival WHERE SessionId = ?",
                workflow_id,
            )
        connection.commit()
        return paths
    except Exception:
        connection.rollback()
        raise
    finally:
        cursor.close()


def remove_generated_files(results: list[dict[str, Any]], profile_paths: list[str]) -> None:
    allowed_root = Path(os.environ["CARPOINT_SERVICE_ALLOWED_ROOT"]).resolve()
    candidates = {Path(path) for path in profile_paths}
    for result in results:
        if result.get("pcd_path"):
            candidates.add(Path(result["pcd_path"]))
        for value in result.get("outputs", {}).values():
            if isinstance(value, str) and value:
                candidates.add(Path(value))
    for candidate in candidates:
        try:
            resolved = candidate.resolve()
            resolved.relative_to(allowed_root)
            if resolved not in SOURCE_PATHS and resolved.is_file():
                resolved.unlink()
        except (OSError, ValueError):
            pass
    baseline = Path(os.environ["CARPOINT_SERVICE_BASELINE_DIR"]) / f"{PLATE}_baseline.json"
    baseline.unlink(missing_ok=True)
    profile_root = Path(os.environ["CARPOINT_EMPTY_PROFILE_ROOT"])
    for directory in sorted(profile_root.glob(f"{PLATE}_*"), reverse=True):
        try:
            directory.rmdir()
        except OSError:
            pass
    if GENERATED_FIXTURE_DIR is not None:
        for path in GENERATED_FIXTURE_DIR.iterdir():
            if path.is_file():
                path.unlink(missing_ok=True)
        try:
            GENERATED_FIXTURE_DIR.rmdir()
        except OSError:
            pass


def main() -> int:
    load_env()
    if len(sys.argv) == 2 and sys.argv[1] == "--synthetic":
        pcd_a, pcd_b, extrinsic = generate_synthetic_fixture()
    elif len(sys.argv) == 4:
        pcd_a, pcd_b, extrinsic = (str(Path(arg).resolve()) for arg in sys.argv[1:])
    else:
        raise SystemExit(
            "usage: server-empty-profile-e2e.py --synthetic | PCD_A PCD_B EXTRINSIC_JSON"
        )
    SOURCE_PATHS.update({Path(pcd_a), Path(pcd_b), Path(extrinsic)})
    for source in SOURCE_PATHS:
        if not source.is_file():
            raise FileNotFoundError(source)

    connection = connect_db()
    results: list[dict[str, Any]] = []
    profile_paths: list[str] = []
    started_at = datetime.now().astimezone()
    run_id = started_at.strftime("%Y%m%d%H%M%S")
    WORKFLOW_IDS.extend(
        [
            f"wf-v7-first-{run_id}",
            f"wf-v7-second-{run_id}",
            f"wf-v7-older-{run_id}",
        ]
    )
    try:
        if read_database_state(connection)["history"]:
            raise RuntimeError(f"test plate {PLATE} already exists")
        insert_test_arrivals(connection, WORKFLOW_IDS)

        first = analyze_pair(
            pcd_a,
            pcd_b,
            extrinsic,
            f"pair-v7-first-{run_id}",
            WORKFLOW_IDS[0],
            started_at - timedelta(minutes=2),
        )
        results.append(first)
        first_state = read_database_state(connection)
        if len(first_state["history"]) != 1 or first_state["profile"] is None:
            raise RuntimeError(f"unexpected first state: {first_state}")
        first_path = Path(first_state["profile"]["pointCloudPath"])
        if not first_path.is_file():
            raise RuntimeError(f"first permanent PCD is missing: {first_path}")

        second = analyze_pair(
            pcd_a,
            pcd_b,
            extrinsic,
            f"pair-v7-second-{run_id}",
            WORKFLOW_IDS[1],
            started_at - timedelta(minutes=1),
        )
        results.append(second)
        second_state = read_database_state(connection)
        if len(second_state["history"]) != 2:
            raise RuntimeError(f"history did not keep both scans: {second_state}")
        if second_state["profile"]["dimensionRecordId"] != second_state["history"][-1][0]:
            raise RuntimeError(f"latest profile does not reference second scan: {second_state}")
        second_path = Path(second_state["profile"]["pointCloudPath"])
        if second_path == first_path or first_path.exists() or not second_path.is_file():
            raise RuntimeError("latest-file replacement lifecycle failed")

        older = analyze_pair(
            pcd_a,
            pcd_b,
            extrinsic,
            f"pair-v7-older-{run_id}",
            WORKFLOW_IDS[2],
            started_at - timedelta(minutes=10),
        )
        results.append(older)
        final_state = read_database_state(connection)
        if len(final_state["history"]) != 3:
            raise RuntimeError(f"out-of-order scan was not retained: {final_state}")
        if final_state["profile"]["pointCloudPath"] != str(second_path):
            raise RuntimeError(f"older result replaced the latest profile: {final_state}")

        admin_username = os.getenv("CARPOINT_WEBAPI_ADMIN_USERNAME", "admin").strip()
        admin_password = os.getenv("CARPOINT_WEBAPI_ADMIN_PASSWORD", "")
        if not admin_username or not admin_password:
            raise RuntimeError(
                "set CARPOINT_WEBAPI_ADMIN_PASSWORD before running the Web API smoke test"
            )
        login_status, login = http_json(
            "POST",
            f"{WEBAPI_URL}/api/auth/login",
            {"username": admin_username, "password": admin_password},
            timeout=30,
        )
        if login_status != 200 or not login.get("data", {}).get("token"):
            raise RuntimeError(f"admin login failed: HTTP {login_status}: {login}")
        auth = {"Authorization": f"Bearer {login['data']['token']}"}
        plate_url = urllib.parse.quote(PLATE, safe="")
        metadata_status, metadata = http_json(
            "GET", f"{WEBAPI_URL}/api/empty-vehicle-profiles/{plate_url}", headers=auth
        )
        if metadata_status != 200:
            raise RuntimeError(f"profile API failed: HTTP {metadata_status}: {metadata}")
        api_profile = metadata.get("data", {})
        if api_profile.get("sourceRecordId") != final_state["profile"]["dimensionRecordId"]:
            raise RuntimeError(f"profile API returned the wrong record: {api_profile}")

        request = urllib.request.Request(
            f"{WEBAPI_URL}/api/empty-vehicle-profiles/{plate_url}/pointcloud",
            headers=auth,
        )
        with urllib.request.urlopen(request, timeout=30) as response:
            point_cloud = response.read()
        if len(point_cloud) != final_state["profile"]["pointCloudSize"]:
            raise RuntimeError("downloaded PCD size does not match DM8")
        if hashlib.sha256(point_cloud).hexdigest() != final_state["profile"]["pointCloudSha256"]:
            raise RuntimeError("downloaded PCD SHA-256 does not match DM8")

        timestamp = str(int(time.time() * 1000))
        app_secret = read_env_value(WEBAPI_ENV_PATH, "Integration__AppSecret")
        signature = hashlib.md5(
            f"{timestamp}{app_secret}".encode("utf-8")
        ).hexdigest()
        history_status, history = http_json(
            "GET",
            f"{WEBAPI_URL}/api/v1/vehicleDimensions?limit=100",
            headers={"Timestamp": timestamp, "Signature": signature},
            timeout=30,
        )
        if history_status != 200 or not isinstance(history.get("data"), list):
            raise RuntimeError(f"history API failed: HTTP {history_status}: {history}")
        test_history = [
            row for row in history.get("data", []) if row.get("licensePlate") == PLATE
        ]
        if history_status != 200 or len(test_history) != 3:
            raise RuntimeError(f"history API did not return all scans: {history}")

        missing_status, _ = http_json(
            "GET",
            f"{WEBAPI_URL}/api/empty-vehicle-profiles/NO_SUCH_TEST_PLATE",
            headers=auth,
            timeout=30,
        )
        if missing_status != 404:
            raise RuntimeError(f"missing profile should return 404, got {missing_status}")

        with urllib.request.urlopen(
            f"{WEBAPI_URL}/pointcloud-viewer/index.html", timeout=30
        ) as response:
            viewer = response.read().decode("utf-8")
        with urllib.request.urlopen(
            f"{WEBAPI_URL}/pointcloud-viewer/js/main.js", timeout=30
        ) as response:
            viewer_js = response.read().decode("utf-8")
        if "plateInput" not in viewer or "/api/empty-vehicle-profiles/" not in viewer_js:
            raise RuntimeError("point-cloud viewer was not upgraded to plate lookup")

        print(
            json.dumps(
                {
                    "passed": True,
                    "historyRows": len(final_state["history"]),
                    "latestRows": 1,
                    "latestRecordId": final_state["profile"]["dimensionRecordId"],
                    "pointCloudBytes": len(point_cloud),
                    "pointCloudSha256Verified": True,
                    "olderScanDidNotReplaceLatest": True,
                    "adminLogin": True,
                    "historyApi": True,
                    "profileApi": True,
                    "pointCloudDownload": True,
                    "viewerPlateLookup": True,
                    "missingPlate404": True,
                },
                ensure_ascii=False,
            )
        )
        return 0
    finally:
        try:
            profile_paths = cleanup_database(connection)
        finally:
            connection.close()
            remove_generated_files(results, profile_paths)


if __name__ == "__main__":
    raise SystemExit(main())
