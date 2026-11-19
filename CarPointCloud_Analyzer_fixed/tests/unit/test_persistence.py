from types import SimpleNamespace
from copy import deepcopy
from datetime import datetime
from decimal import Decimal
from pathlib import Path

import pytest

from carpointcloud_analyzer.infrastructure.persistence import Dm8DimensionStore, Dm8Settings


def _payload():
    return {
        "idempotencyKey": "scan_001",
        "cameraIdentifier": "CAM_004",
        "licensePlate": "川A12047",
        "getTime": None,
        "detectStatus": "success",
        "dimensions": {
            "cabToTrailerFrontDist": 100.04,
            "trailerLength": 960.04,
            "trailerWidth": 255,
            "trailerBodyHeight": 180.36,
            "trailerBottomToGroundHeight": 95.1,
            "highLowPlateHeight": 47.0,
        },
        "ribs": [{"ribId": 1, "distToPrev": 480.0}],
        "wires": [],
        "highLowPlates": [
            {"plateId": 1, "highestToGround": 142.1, "lowestToGround": 95.1}
        ],
        "sourceFile": "/data/scan_001.pcd",
        "resultJsonPath": "/data/scan_001.json",
        "resultImagePath": None,
    }


def test_long_unsegmented_carriage_persists_and_retries_idempotently():
    database = FakeDatabase()
    data = _payload()
    data["dimensions"]["trailerLength"] = 1134.4
    data["ribs"] = [{"ribId": 1, "distToPrev": 1134.4}]
    store = _store(database)
    first = store.persist_vehicle_dimensions(data)
    second = store.persist_vehicle_dimensions(data)
    assert first.inserted and not second.inserted
    assert first.record_id == second.record_id
    assert len(database.records) == 1


def test_empty_partial_measurement_persists_nulls_without_inventing_dimensions():
    database = FakeDatabase()
    payload = _payload()
    payload['idempotencyKey'] = 'partial_empty_001'
    payload['dimensions'].update(
        cabToTrailerFrontDist=None, trailerLength=None, trailerWidth=238.8,
        trailerBodyHeight=180.5, trailerBottomToGroundHeight=None,
        highLowPlateHeight=None)
    payload['ribs'] = []
    payload['highLowPlates'] = []
    result = _store(database).persist_vehicle_dimensions(payload)
    assert result.inserted
    record = next(row for kind, row in database.details if kind == 'record')
    assert record[8] is None and record[9] is None
    assert record[10:12] == (Decimal('238.8'), Decimal('180.5'))
    assert record[12] is None and record[13] is None
    assert database.profiles == {}


def test_preanalysis_workflow_plate_is_read_only_and_requires_confirmed_arrival():
    database = FakeDatabase()
    database.arrivals["ready"] = ("粤A64727", "SUCCEEDED")
    database.arrivals["pending"] = ("粤B76198", "PENDING")
    store = _store(database)
    assert store.get_workflow_plate("ready") == "粤A64727"
    assert store.get_workflow_plate("pending") is None
    assert store.get_workflow_plate("missing") is None
    assert not database.records
    assert not database.profiles


def test_dimension_unavailable_is_idempotent_and_does_not_claim_measurement_identity():
    db = FakeDatabase()
    store = _store(db)
    at = datetime(2026, 11, 19, 10, 30)
    workflow = "paired-001"
    db.arrivals[workflow] = ("川A12047", "SUCCEEDED")
    assert not store.record_dimension_unavailable("unknown-flow", "CAM_004", at)
    assert store.record_dimension_unavailable(workflow, "CAM_004", at)
    assert store.record_dimension_unavailable(workflow, "CAM_005", at.replace(minute=31))
    assert db.unavailable_dimensions == {workflow: (at, "CAM_004")}
    assert not db.records

    measurement = _empty_scan(workflow, at)
    assert store.persist_vehicle_dimensions(measurement).inserted
    assert store.record_dimension_unavailable(workflow, "CAM_004", at)
    assert not store.persist_vehicle_dimensions(measurement).inserted


def test_pair_unavailable_http_records_the_capture_without_calculating():
    from fastapi.testclient import TestClient
    from carpointcloud_analyzer.composition import create_app

    db = FakeDatabase()
    db.arrivals["paired-http"] = ("川A12047", "SUCCEEDED")
    app = create_app(dimension_store=_store(db))
    with TestClient(app) as client:
        response = client.post("/analyze-pair/unavailable", json={
            "workflowId": "paired-http", "cameraA": "CAM_004",
            "capturedAt": "2026-11-19T10:30:00",
        })
    assert response.status_code == 200
    assert response.json() == {"status": "recorded"}
    assert db.unavailable_dimensions == {"paired-http": (datetime(2026, 11, 19, 10, 30), "CAM_004")}
    assert not db.records


def test_same_workflow_equal_scan_time_profile_retry_is_idempotent(tmp_path):
    db = FakeDatabase()
    store = _store(db)
    at = datetime(2026, 10, 21, 10)
    scan = _empty_scan("equal-same-workflow", at)
    db.arrivals[scan["workflowId"]] = ("川A12047", "SUCCEEDED")
    first = store.persist_vehicle_dimensions(scan, _profile(tmp_path / "first.pcd", at))
    retry = store.persist_vehicle_dimensions(scan, _profile(tmp_path / "retry.pcd", at))
    assert retry.record_id == first.record_id and not retry.inserted
    assert retry.empty_vehicle_profile["updated"] is True
    assert len(db.records) == 1
    assert db.profiles["川A12047"][3:] == (first.record_id, scan["workflowId"])


def test_old_workflow_equal_scan_time_cannot_replace_already_committed_workflow(tmp_path):
    db = FakeDatabase()
    store = _store(db)
    at = datetime(2026, 10, 21, 10)
    old = _empty_scan("old-late-plate-equal", at)
    old["licensePlate"] = "UNKNOWN"
    db.arrivals[old["workflowId"]] = (None, "PENDING")
    old_result = store.persist_vehicle_dimensions(old)
    newer = _empty_scan("newer-committed-equal", at)
    db.arrivals[newer["workflowId"]] = ("川A12047", "SUCCEEDED")
    new_result = store.persist_vehicle_dimensions(newer, _profile(tmp_path / "newer.pcd", at))
    db.arrivals[old["workflowId"]] = ("川A12047", "SUCCEEDED")
    late = store.persist_vehicle_dimensions(old, _profile(tmp_path / "older.pcd", at))
    assert late.record_id == old_result.record_id and not late.inserted
    assert late.empty_vehicle_profile["updated"] is False
    assert late.empty_vehicle_profile["pointCloudPath"] == str((tmp_path / "newer.pcd").resolve())
    assert len(db.records) == 2
    assert db.record_plates[old_result.record_id] == "UNKNOWN"
    assert db.profiles["川A12047"][3:] == (new_result.record_id, newer["workflowId"])


@pytest.mark.parametrize("ready", [True, False])
def test_analysis_resolves_workflow_plate_before_selecting_vehicle_baseline(tmp_path, ready):
    from carpointcloud_analyzer.application.service import AnalysisService, AnalyzeRequest

    database = FakeDatabase()
    database.arrivals["workflow-manual"] = ("粤A64727", "SUCCEEDED" if ready else "PENDING")
    configured_plates = []

    class Analyzer:
        def __init__(self, config):
            configured_plates.append(config.license_plate)

        def analyze(self, path):
            return {"integrationPayloads": {}}

    pcd = tmp_path / "manual.pcd"
    pcd.write_text("placeholder", encoding="utf-8")
    svc = AnalysisService(analyzer_factory=Analyzer, dimension_store=_store(database))
    try:
        svc.analyze_sync(AnalyzeRequest(str(pcd), plate="WRONG", workflow_id="workflow-manual"))
    finally:
        svc.shutdown()
    assert configured_plates == ["粤A64727" if ready else ""]


def test_pair_analysis_resolves_manual_plate_before_loading_clouds(tmp_path, monkeypatch):
    from carpointcloud_analyzer.application import service

    database = FakeDatabase()
    database.arrivals["workflow-manual"] = ("粤A64727", "SUCCEEDED")
    for name in ("a.pcd", "b.pcd", "extrinsic.json"):
        (tmp_path / name).write_text("test-only", encoding="utf-8")
    plates = []

    class StopBeforeGeometry:
        def __init__(self, config):
            plates.append(config.license_plate)

        def load(self, path):
            raise RuntimeError("test stops before geometry")

    monkeypatch.setattr(service, "PointCloudLoader", StopBeforeGeometry)
    monkeypatch.setattr(service, "load_trusted_extrinsic",
                        lambda _: SimpleNamespace(validate_camera_pair=lambda *_: None))
    svc = service.AnalysisService(dimension_store=_store(database))
    try:
        request = service.PairAnalyzeRequest(
            pcd_path_a=str(tmp_path / "a.pcd"), pcd_path_b=str(tmp_path / "b.pcd"),
            camera_a="CAM_004", camera_b="CAM_005",
            extrinsic_path=str(tmp_path / "extrinsic.json"), pair_id="pair-manual",
            workflow_id="workflow-manual", plate="WRONG",
        )
        with pytest.raises(RuntimeError, match="test stops before geometry"):
            svc.analyze_pair_sync(request)
    finally:
        svc.shutdown()
    assert plates == ["粤A64727"]
    assert not database.records


class FakeDatabase:
    def __init__(self):
        self.records = {}
        self.record_plates = {}
        self.arrivals = {}
        self.unavailable_dimensions = {}
        self.profiles = {}
        self.details = []
        self.next_id = 1
        self.next_profile_id = 1
        self.fail_on_plate = False
        self.fail_on_profile = False
        self.arrival_locks = []
        self.arrival_events = []
        self.profile_events = []

    def connect(self, **kwargs):
        assert kwargs["autoCommit"] is False
        assert kwargs["server"] == "127.0.0.1"
        return FakeConnection(self)


class FakeConnection:
    def __init__(self, database):
        self.database = database
        self.snapshot = (
            deepcopy(database.records),
            deepcopy(database.record_plates),
            deepcopy(database.profiles),
            deepcopy(database.unavailable_dimensions),
            list(database.details),
        )
        self.committed = False
        self.closed = False

    def cursor(self):
        return FakeCursor(self.database)

    def commit(self):
        self.committed = True

    def rollback(self):
        (
            self.database.records,
            self.database.record_plates,
            self.database.profiles,
            self.database.unavailable_dimensions,
            self.database.details,
        ) = deepcopy(self.snapshot)

    def close(self):
        self.closed = True


class FakeCursor:
    def __init__(self, database):
        self.database = database
        self.row = None

    def execute(self, sql, *params):
        normalized = " ".join(sql.split()).upper()
        if normalized == "SELECT 1":
            self.row = (1,)
        elif normalized.startswith("SELECT TOP 1 LICENSEPLATE FROM EMPTYVEHICLEPROFILE"):
            self.row = next(iter(self.database.profiles), None)
        elif normalized.startswith("SELECT ID, PAYLOADHASH"):
            if "WHERE IDEMPOTENCYKEY" in normalized:
                self.row = self.database.records.get(params[0])
            else:
                identity_index = 1 if "WHERE WORKFLOWID" in normalized else 2
                self.row = next(
                    (
                        self.database.records[record_params[0]]
                        for kind, record_params in self.database.details
                        if kind == "record" and record_params[identity_index] == params[0]
                    ),
                    None,
                )
        elif normalized.startswith("SELECT LICENSEPLATE, PLATESTATUS FROM VEHICLEARRIVAL"):
            self.database.arrival_events.append(("read", params[0]))
            self.row = self.database.arrivals.get(params[0])
        elif normalized.startswith("UPDATE VEHICLEARRIVAL SET UPDATEDAT = UPDATEDAT"):
            self.database.arrival_locks.append(params[0])
            self.database.arrival_events.append(("lock", params[0]))
            self.rowcount = 1 if params[0] in self.database.arrivals else 0
        elif normalized.startswith("UPDATE VEHICLEARRIVAL SET DIMENSIONUNAVAILABLEAT"):
            captured_at, camera, workflow = params
            self.rowcount = int(
                workflow in self.database.arrivals and workflow not in self.database.unavailable_dimensions
            )
            if self.rowcount:
                self.database.unavailable_dimensions[workflow] = (captured_at, camera)
        elif normalized.startswith("SELECT ID FROM VEHICLEARRIVAL WHERE SESSIONID"):
            self.row = (1,) if params[0] in self.database.arrivals else None
        elif normalized.startswith("SELECT ID, POINTCLOUDPATH, SCANTIME"):
            self.database.profile_events.append(('read', params[0]))
            profile = self.database.profiles.get(params[0])
            self.row = (profile[0], profile[1], profile[2], profile[4]) if profile else None
        elif "INSERT INTO VEHICLEDIMENSIONRECORD" in normalized:
            key, payload_hash = params[0], params[3]
            record_id = self.database.next_id
            self.database.next_id += 1
            self.database.records[key] = (record_id, payload_hash)
            self.database.record_plates[record_id] = params[4]
            self.database.details.append(("record", params))
        elif normalized.startswith("UPDATE VEHICLEDIMENSIONRECORD SET LICENSEPLATE"):
            record_id = params[2]
            current = self.database.record_plates[record_id]
            if current is None or not str(current).strip() or str(current).upper() == "UNKNOWN":
                self.database.record_plates[record_id] = params[0]
                self.rowcount = 1
            else:
                self.rowcount = 0
        elif "INSERT INTO VEHICLEDIMENSIONHIGHLOWPLATE" in normalized:
            if self.database.fail_on_plate:
                raise RuntimeError("simulated child insert failure")
            self.database.details.append(("plate", params))
        elif "INSERT INTO VEHICLEDIMENSIONRIB" in normalized:
            self.database.details.append(("rib", params))
        elif "INSERT INTO VEHICLEDIMENSIONWIRE" in normalized:
            self.database.details.append(("wire", params))
        elif "INSERT INTO EMPTYVEHICLEPROFILE" in normalized:
            if self.database.fail_on_profile:
                raise RuntimeError("simulated profile insert failure")
            profile_id = self.database.next_profile_id
            self.database.next_profile_id += 1
            self.database.profiles[params[0]] = (
                profile_id,
                params[5],
                params[9],
                params[1],
                params[2],
            )
            self.database.details.append(("profile_insert", params))
        elif normalized.startswith("UPDATE EMPTYVEHICLEPROFILE SET UPDATEDAT = UPDATEDAT"):
            self.database.profile_events.append(('lock', params[0]))
            self.rowcount = 1 if params[0] in self.database.profiles else 0
        elif normalized.startswith("UPDATE EMPTYVEHICLEPROFILE SET"):
            plate = params[-4]
            existing = self.database.profiles[plate]
            if (existing[2] is None or existing[2] < params[-3]
                    or (existing[2] == params[-2] and existing[4] == params[-1])):
                self.database.profiles[plate] = (
                    existing[0],
                    params[4],
                    params[8],
                    params[0],
                    params[1],
                )
                self.database.details.append(("profile_update", params))
                self.rowcount = 1
            else:
                self.rowcount = 0
        else:  # pragma: no cover - SQL 一旦变更就在这里直接失败。
            raise AssertionError(sql)
        return self

    def fetchone(self):
        return self.row

    def close(self):
        pass


def _store(database):
    return Dm8DimensionStore(
        Dm8Settings(
            enabled=True,
            host="127.0.0.1",
            port=5236,
            user="APP",
            password="secret",
        ),
        connect=database.connect,
    )


def test_dm8_store_inserts_parent_and_children_then_deduplicates_retry():
    database = FakeDatabase()
    store = _store(database)

    first = store.persist_vehicle_dimensions(_payload())
    second = store.persist_vehicle_dimensions(_payload())

    assert first.inserted is True
    assert second.inserted is False
    assert first.record_id == second.record_id == 1
    assert [kind for kind, _ in database.details] == ["record", "rib", "plate"]


def test_dm8_store_ignores_retry_generated_result_paths_in_idempotency_hash():
    database = FakeDatabase()
    store = _store(database)
    store.persist_vehicle_dimensions(_payload())
    retried = _payload()
    retried["resultJsonPath"] = "/data/scan_001_retry_uuid.json"
    retried["resultImagePath"] = "/data/scan_001_retry_uuid.png"

    result = store.persist_vehicle_dimensions(retried)

    assert result.inserted is False
    assert result.record_id == 1


def test_dm8_store_rejects_same_key_with_different_payload():
    database = FakeDatabase()
    store = _store(database)
    store.persist_vehicle_dimensions(_payload())
    changed = _payload()
    changed["dimensions"]["trailerWidth"] = 260.0

    with pytest.raises(ValueError, match="幂等键"):
        store.persist_vehicle_dimensions(changed)


def test_dm8_store_deduplicates_same_workflow_when_retry_changes_idempotency_key():
    database = FakeDatabase()
    store = _store(database)
    first_payload = _payload()
    first_payload["workflowId"] = "workflow-001"
    first_payload["pairId"] = "pair-001"
    first = store.persist_vehicle_dimensions(first_payload)

    retried = _payload()
    retried["idempotencyKey"] = "scan_001_retry"
    retried["workflowId"] = "workflow-001"
    retried["pairId"] = "pair-001"
    second = store.persist_vehicle_dimensions(retried)

    assert first.inserted is True
    assert second.inserted is False
    assert second.record_id == first.record_id
    assert len(database.records) == 1


def test_dm8_store_uses_succeeded_arrival_plate_as_workflow_authority():
    database = FakeDatabase()
    database.arrivals["workflow-authority"] = ("粤A64727", "SUCCEEDED")
    payload = _payload()
    payload["workflowId"] = "workflow-authority"
    payload["licensePlate"] = "UNKNOWN"

    result = _store(database).persist_vehicle_dimensions(payload)

    assert result.inserted is True
    assert database.record_plates[result.record_id] == "粤A64727"
    assert database.arrival_locks == []
    assert database.arrival_events == [("read", "workflow-authority")]




def test_dm8_store_rolls_back_parent_when_child_insert_fails():
    database = FakeDatabase()
    database.fail_on_plate = True
    store = _store(database)

    with pytest.raises(RuntimeError, match="child insert failure"):
        store.persist_vehicle_dimensions(_payload())

    assert database.records == {}
    assert database.details == []


def test_dm8_store_health_executes_database_probe():
    assert _store(FakeDatabase()).health() == {"enabled": True, "status": "ok"}


def test_dm8_store_accepts_normal_long_trailer():
    database = FakeDatabase()
    payload = _payload()
    payload["dimensions"]["trailerLength"] = 1124.4

    assert _store(database).persist_vehicle_dimensions(payload).inserted is True


def _profile(path: Path, scan_time: datetime):
    return {
        "pointCloudPath": str(path.resolve()),
        "pointCloudSha256": "a" * 64,
        "pointCloudSize": 1234,
        "pointCount": 456,
        "scanTime": scan_time,
    }


def _empty_scan(key: str, scan_time: datetime):
    payload = _payload()
    payload["idempotencyKey"] = key
    payload["workflowId"] = key
    payload["pairId"] = f"pair-{key}"
    payload["getTime"] = scan_time.isoformat()
    return payload








@pytest.mark.parametrize('pending_review', [False, True])
def test_dm8_store_keeps_every_scan_but_upserts_one_latest_profile_per_plate(tmp_path, pending_review):
    database = FakeDatabase()
    store = _store(database)
    first_time = datetime(2026, 8, 13, 9, 0, 0)
    second_time = datetime(2026, 8, 13, 10, 0, 0)
    first_path = tmp_path / "first.pcd"
    second_path = tmp_path / "second.pcd"

    first = store.persist_vehicle_dimensions(
        _empty_scan("scan-first", first_time), _profile(first_path, first_time)
    )
    second_payload = _empty_scan("scan-second", second_time)
    second_profile = _profile(second_path, second_time)
    if pending_review:
        second_payload['detectStatus'] = 'low_confidence'
        second_profile['reviewStatus'] = 'pending'
    second = store.persist_vehicle_dimensions(second_payload, second_profile)

    assert len(database.records) == 2
    assert len(database.profiles) == 1
    assert database.profile_events == [('read', '川A12047')] * 2
    profile = database.profiles["川A12047"]
    assert profile[1] == str(second_path.resolve())
    assert profile[3] == second.record_id
    assert first.empty_vehicle_profile["updated"] is True
    assert second.empty_vehicle_profile["updated"] is True
    assert second.empty_vehicle_profile["previousPointCloudPath"] == str(first_path.resolve())


def test_dm8_store_does_not_let_an_older_late_result_replace_latest_profile(tmp_path):
    database = FakeDatabase()
    store = _store(database)
    newest_time = datetime(2026, 8, 13, 11, 0, 0)
    older_time = datetime(2026, 8, 13, 10, 0, 0)
    newest_path = tmp_path / "newest.pcd"
    older_path = tmp_path / "older.pcd"
    store.persist_vehicle_dimensions(
        _empty_scan("scan-newest", newest_time), _profile(newest_path, newest_time)
    )

    result = store.persist_vehicle_dimensions(
        _empty_scan("scan-older", older_time), _profile(older_path, older_time)
    )

    assert len(database.records) == 2
    assert database.profiles["川A12047"][1] == str(newest_path.resolve())
    assert result.empty_vehicle_profile["updated"] is False
    assert result.empty_vehicle_profile["pointCloudPath"] == str(newest_path.resolve())


def test_dm8_store_rolls_back_history_when_latest_profile_write_fails(tmp_path):
    database = FakeDatabase()
    database.fail_on_profile = True
    scan_time = datetime(2026, 8, 13, 12, 0, 0)

    with pytest.raises(RuntimeError, match="profile insert failure"):
        _store(database).persist_vehicle_dimensions(
            _empty_scan("scan-failed", scan_time),
            _profile(tmp_path / "failed.pcd", scan_time),
        )

    assert database.records == {}
    assert database.profiles == {}
    assert database.details == []
