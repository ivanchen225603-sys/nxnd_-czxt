"""独立本机 DM8 回归。让同号两趟车都读到档案尚不存在，再按指定次序提交。"""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta
import os
from threading import Barrier, Event
from uuid import uuid4

import pytest

from carpointcloud_analyzer.infrastructure.persistence import Dm8DimensionStore, Dm8Settings


@pytest.mark.skipif(not os.getenv("STANDSTILL_ISOLATED_DM_PORT"), reason="需要专用的本机临时 DM8")
@pytest.mark.parametrize("first", [0, 1], ids=["older-then-newer", "newer-then-older"])
def test_simultaneous_first_archive_keeps_both_dimensions_and_newer_pcd(first, tmp_path):
    import dmPython

    settings = Dm8Settings(host="127.0.0.1", port=int(os.environ["STANDSTILL_ISOLATED_DM_PORT"]),
                           schema="CARPOINTCLOUD", user="SYSDBA",
                           password=os.environ["STANDSTILL_ISOLATED_DM_PASSWORD"])
    plate = f"宁A{uuid4().int % 100000:05}"
    workflows = ["race-" + uuid4().hex for _ in range(2)]
    scans = [datetime(2026, 11, 26, 12), datetime(2026, 11, 26, 12) + timedelta(seconds=20)]
    barrier = Barrier(2, timeout=8)
    winner_committed = Event()
    collisions = []
    root = "CARPOINTCLOUD."

    class Cursor:
        def __init__(self, real, i):
            self.real, self.i = real, i

        def __getattr__(self, name):
            return getattr(self.real, name)

        def execute(self, sql, *args):
            if not sql.startswith("INSERT INTO CARPOINTCLOUD.EmptyVehicleProfile "):
                return self.real.execute(sql, *args)
            barrier.wait()
            if self.i != first:
                assert winner_committed.wait(8)
            try:
                return self.real.execute(sql, *args)
            except dmPython.Error as exc:
                collisions.extend(getattr(part, "code", None) for part in exc.args)
                raise

    class Connection:
        def __init__(self, real, i):
            self.real, self.i = real, i

        def __getattr__(self, name):
            return getattr(self.real, name)

        def cursor(self):
            return Cursor(self.real.cursor(), self.i)

    def persist(i):
        store = Dm8DimensionStore(settings, connect=lambda **kw: Connection(dmPython.connect(**kw), i))
        workflow = workflows[i]
        payload = {"workflowId": workflow, "pairId": "p-" + workflow, "idempotencyKey": workflow,
                   "licensePlate": plate, "getTime": scans[i].isoformat(), "cameraIdentifier": "CAM_004",
                   "dimensions": {"trailerLength": 1000 + i, "trailerWidth": 239, "trailerBodyHeight": 188}}
        source = tmp_path / f"{i}.pcd"
        source.write_bytes(b"archive")
        profile = {"pointCloudPath": str(source), "pointCloudSha256": str(i) * 64,
                   "pointCloudSize": 7, "pointCount": 1, "scanTime": scans[i]}
        try:
            return store.persist_vehicle_dimensions(payload, profile)
        finally:
            if i == first:
                winner_committed.set()

    db = dmPython.connect(server=settings.host, port=settings.port, user=settings.user,
                          password=settings.password, autoCommit=False)
    try:
        cursor = db.cursor()
        for workflow, scan in zip(workflows, scans):
            cursor.execute(f"INSERT INTO {root}VehicleArrival "
                           "(EventId, SessionId, DeviceId, EventType, CameraIdentifier, ArrivalTime, LicensePlate, PlateStatus) "
                           "VALUES (?, ?, 'TEST', 'ARRIVAL', 'CAM_004', ?, ?, 'SUCCEEDED')", workflow, workflow, scan, plate)
        db.commit()
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(persist, [0, 1]))
        assert all(result.inserted for result in results)
        assert collisions == [-6602]
        cursor.execute(f"SELECT WorkflowId, TrailerLength FROM {root}VehicleDimensionRecord "
                       "WHERE WorkflowId IN (?,?)", *workflows)
        assert dict(cursor.fetchall()) == {workflows[0]: 1000, workflows[1]: 1001}
        cursor.execute(f"SELECT WorkflowId, DimensionRecordId, PointCloudPath FROM {root}EmptyVehicleProfile "
                       "WHERE LicensePlate=?", plate)
        assert cursor.fetchall() == [(workflows[1], results[1].record_id, str(tmp_path / "1.pcd"))]
    finally:
        db.close()
