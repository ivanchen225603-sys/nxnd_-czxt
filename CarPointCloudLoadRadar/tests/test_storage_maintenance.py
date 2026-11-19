"""日清真实临时SQLite/文件回归；不启动设备、模拟服务或业务数据库。"""
from datetime import datetime, timedelta
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from app.load_radar_bridge import ResultReservation
from app.load_radar_transport import SQLiteOutbox
from app.cloud_archive import ArchiveStore
from app.pending_measurements import PendingTaskStore
from app.result_finalizer import ResultFinalizer
from app.service import LidarService
from app.settings import ServiceCfg, Settings, load_channels
from app.storage_maintenance import StorageMaintenance
from tests.test_cloud_archive import example_bundle, payload_for


class TestStorageMaintenance(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="daily-cloud-cleanup-")
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.output = self.root / "field"
        self.baselines = self.output / "baselines"
        self.baselines.mkdir(parents=True)
        self.outbox = SQLiteOutbox(self.root / "outbox")
        self.store = PendingTaskStore(self.root / "outbox", ["CH1"])
        self.addCleanup(self.outbox.close)
        self.addCleanup(self.store.close)
        self.outbox.archives.ensure_cleanup_schema()
        self.store.ensure_cleanup_schema()
        channel = SimpleNamespace(id="CH1", abs_output_dir=str(self.output), abs_baseline_dir=str(self.baselines))
        self.events = []
        self.service = SimpleNamespace(
            st=SimpleNamespace(service=SimpleNamespace(data_root=str(self.root)), channels=[channel]),
            workers=[SimpleNamespace(ch=channel, _control_lock=threading.RLock(), integration=None, _local_passage=None)],
            integration=SimpleNamespace(outbox=self.outbox),
            deferred=SimpleNamespace(store=self.store, channels=["CH1"]),
            finalizer=SimpleNamespace(_store_lock=threading.Lock(), _stores={}),
            bus=SimpleNamespace(publish=lambda kind, body: self.events.append((kind, body))))
        self.now = (datetime.now().replace(hour=4, minute=0, second=0, microsecond=0) + timedelta(days=2)).timestamp()
        self.cutoff = datetime.fromtimestamp(self.now).replace(hour=3).timestamp()
        self.cleanup = StorageMaintenance(self.service, "03:00", clock=lambda: self.now)

    def archive(self, mid, *, status="sent", complete=True, committed=True, empty=False, ts=1000., large=False, recalculated=True):
        bundle = example_bundle(measurement=mid, passage="p-" + mid, empty=empty, ts=ts)
        payload = payload_for(bundle)
        payload["verdict"] = "unknown" if empty is None else "empty" if empty else "loaded"
        self.outbox.save(payload, archive=bundle)
        path = None
        if complete:
            path = Path(self.outbox.archives.export(mid, str(self.output), channel="CH1"))
        if status == "sent":
            self.outbox.mark_sent(mid)
        elif status == "blocked":
            self.outbox.mark_blocked(mid, RuntimeError("blocked"))
        identity = ResultReservation(mid, "CH1", "p-" + mid, "token-" + mid)
        start = json.dumps({"passageId": identity.passage_id}).encode()
        raw = b"original-" + (b"x" * 2_000_000 if large else mid.encode())
        self.store.put(identity, start, raw)
        self.store.freeze_preparation("CH1", mid, b"proof")
        self.store.freeze_prepared("CH1", mid, b"prepared" + mid.encode())
        self.store.freeze_exit("CH1", mid, b'{"frames":[]}')
        self.store.register_recalculation("CH1", mid, payload)
        if committed:
            self.store.mark_committed("CH1", mid)
            if recalculated:
                self.store.save_recalculation("CH1", "revision-" + mid, mid, b"completed-fixture")
                self.store.acknowledge_recalculation("CH1", "revision-" + mid)
        return bundle, payload, path, (identity, start, raw)

    def test_completed_measurement_waiting_for_ch4_recalculation_is_not_deleted(self):
        bundle, _, path, _ = self.archive("m-await-ch4", recalculated=False)
        self.cleanup.run_once()
        self.assertTrue(path.exists())
        self.assertIsNotNone(self.outbox.archives.get(bundle.measurement_id))
        self.assertFalse(self.store.get("CH1", bundle.measurement_id).compacted)

    def test_revision_delivery_failure_pins_archives_and_inputs_after_restart(self):
        bundle, _, path, _ = self.archive("m-revision-unsent", recalculated=False)
        self.store.save_recalculation("CH1", "unsent-revision", bundle.measurement_id, b"original-final-bytes")
        self.store.close()
        self.store = PendingTaskStore(self.root / "outbox", ["CH1"])
        self.addCleanup(self.store.close)
        self.service.deferred.store = self.store
        self.cleanup.run_once()
        self.assertTrue(path.exists())
        self.assertFalse(self.store.get("CH1", bundle.measurement_id).compacted)

    def loose(self, relative, *, passage=None, when=None):
        import os
        path = self.output / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"complete-cloud")
        meta = Path(str(path) + ".meta.json")
        meta.write_text(json.dumps({"channel": "CH1", "passage_id": passage}), encoding="utf-8")
        stamp = self.cutoff - 10 if when is None else when
        os.utime(path, (stamp, stamp))
        os.utime(meta, (stamp, stamp))
        return path

    def test_sent_complete_committed_clouds_purge_but_receipts_remain_across_restart(self):
        bundle, payload, path, original = self.archive("m-done")
        report = self.cleanup.run_once()
        self.assertFalse(path.parent.exists())
        self.assertGreater(report["sqlite_bytes_reusable"], 0)
        self.assertIsNone(self.outbox.archives.get("m-done"))
        self.assertEqual(bundle.manifest, self.outbox.archives.receipt("m-done"))
        row = self.store.get("CH1", "m-done")
        self.assertTrue(row.compacted)
        self.assertEqual(b"", row.input_bytes)
        self.assertEqual(row, self.store.put(*original))
        self.assertEqual(row, self.store.mark_committed("CH1", "m-done"))
        with self.assertRaises(ValueError):
            self.store.put(original[0], original[1], b"changed")
        self.assertEqual("m-done", self.outbox.save(payload, archive=bundle))
        self.outbox.close()
        self.outbox = SQLiteOutbox(self.root / "outbox")
        self.addCleanup(self.outbox.close)
        self.assertEqual("m-done", self.outbox.save(payload, archive=bundle))
        with self.assertRaises(ValueError):
            self.outbox.archives.export("m-done", str(self.output), channel="CH1")
        self.assertFalse(path.parent.exists())

    def test_baseline_candidate_asset_and_command_receipt_survive_cleanup_and_restart(self):
        self.service.st.channels[0].id = "CH4"
        self.cleanup = StorageMaintenance(self.service, "03:00", clock=lambda: self.now)
        bundle = example_bundle(measurement="m-candidate", channel="CH4", passage="p-candidate", empty=True)
        asset = b"opaque-formal-candidate-bytes"
        payload = dict(payload_for(bundle), baselineCandidateSha256=hashlib.sha256(asset).hexdigest())
        command = dict(requestId="candidate-request", channelId="CH4", passageId="p-candidate",
                       kind="baseline-capture", confirmedEmpty=True)
        self.outbox.save(payload, candidate_bytes=asset, command=command, archive=bundle)
        path = Path(self.outbox.archives.export(bundle.measurement_id, str(self.output), channel="CH4"))
        self.outbox.mark_sent(bundle.measurement_id)
        original = self.outbox.command_result("candidate-request")
        self.cleanup.run_once()
        self.assertFalse(path.exists())
        self.assertIsNone(self.outbox.archives.get(bundle.measurement_id))
        self.assertEqual(original, self.outbox.command_result("candidate-request"))
        self.outbox.close()
        self.outbox = SQLiteOutbox(self.root / "outbox")
        self.addCleanup(self.outbox.close)
        self.assertEqual(original, self.outbox.command_result("candidate-request"))
        self.assertEqual(bundle.measurement_id,
                         self.outbox.save(payload, candidate_bytes=asset, command=command, archive=bundle))

    def test_pending_blocked_unexported_and_uncommitted_are_kept(self):
        cases = [("m-pending", "pending", True, True), ("m-blocked", "blocked", True, True),
                 ("m-unexported", "sent", False, True), ("m-uncommitted", "sent", True, False)]
        bundles = [self.archive(mid, status=status, complete=complete, committed=committed)[0]
                   for mid, status, complete, committed in cases]
        self.cleanup.run_once()
        for bundle in bundles:
            self.assertEqual(bundle, self.outbox.archives.get(bundle.measurement_id))
            self.assertFalse(self.store.get("CH1", bundle.measurement_id).compacted)

    def test_acknowledged_unknown_physical_verdict_is_a_completed_ordinary_sample(self):
        bundle, _, path, _ = self.archive("m-unknown", empty=None)
        self.cleanup.run_once()
        self.assertIsNone(self.outbox.archives.get(bundle.measurement_id))
        self.assertFalse(path.exists())

    def test_legacy_completed_timestamp_backfill_resumes_when_column_already_exists(self):
        self.archive("m-legacy")
        self.outbox._connection.execute("UPDATE cloud_archives SET completed_at=NULL WHERE measurement_id='m-legacy'")
        self.outbox.close()
        self.outbox = SQLiteOutbox(self.root / "outbox")
        self.addCleanup(self.outbox.close)
        self.outbox.archives.ensure_cleanup_schema()
        observed = self.outbox._connection.execute("SELECT completed_at FROM cloud_archives WHERE measurement_id='m-legacy'").fetchone()[0]
        self.assertIsInstance(observed, float)
        self.assertGreater(observed, 0.)

    def test_standalone_cold_start_discovers_existing_store_without_reexport(self):
        local = ArchiveStore(str(self.output))
        bundle = example_bundle(measurement="m-local", empty=False)
        local.save(bundle)
        path = Path(local.journal.export(bundle.measurement_id, str(self.output), channel="CH1"))
        local.close()
        self.service.integration = None
        self.service.deferred = None
        finalizer = ResultFinalizer({"CH1": (str(self.output), False)})
        self.service.finalizer = finalizer
        self.addCleanup(finalizer.close)
        self.assertEqual({}, finalizer._stores)
        self.cleanup.run_once()
        self.assertFalse(path.parent.exists())
        journal = next(iter(finalizer._stores.values())).journal
        self.assertTrue(journal.is_pruned(bundle.measurement_id))

    def test_low_disk_keeps_pending_blob_when_archive_is_small_or_missing(self):
        for archive_exists in (True, False):
            with self.subTest(archive_exists=archive_exists):
                mid = "m-low-%s" % archive_exists
                self.archive(mid, large=True)
                if not archive_exists:
                    self.outbox._connection.execute("DELETE FROM cloud_archive_files WHERE measurement_id=?", (mid,))
                    self.outbox._connection.execute("DELETE FROM cloud_archives WHERE measurement_id=?", (mid,))
                with patch("app.pending_measurements.shutil.disk_usage", return_value=SimpleNamespace(free=18 * 1024 * 1024)):
                    with self.assertRaises(OSError):
                        self.cleanup.run_once()
                self.assertFalse(self.cleanup.state_path.exists())
                row = self.store.get("CH1", mid)
                self.assertFalse(row.compacted)
                self.assertGreater(len(row.input_bytes), 1_000_000)

    def test_low_disk_archive_guard_does_not_start_projection_deletion(self):
        bundle, _, path, _ = self.archive("m-low-archive")
        with patch("app.cloud_archive.shutil.disk_usage", return_value=SimpleNamespace(free=0)):
            with self.assertRaises(OSError):
                self.cleanup.run_once()
        self.assertTrue(path.exists())
        self.assertEqual(bundle, self.outbox.archives.get(bundle.measurement_id))
        self.assertFalse(self.outbox.archives.is_pruned(bundle.measurement_id))

    def test_real_service_init_validates_policy_before_acquiring_sources_or_bridge(self):
        bad = Settings(service=ServiceCfg(data_root=str(self.root), daily_cleanup_at="03:00"),
                       channels=[SimpleNamespace(id="CH1", abs_output_dir=str(self.root.parent), abs_baseline_dir=str(self.baselines))])
        with patch("app.service.ChannelWorker") as worker, patch("app.load_radar_bridge.LoadRadarBridge") as bridge:
            with self.assertRaises(ValueError):
                LidarService(settings=bad, webapi_config="must-not-be-opened.json")
            worker.assert_not_called()
            bridge.assert_not_called()
        settings = Settings(service=ServiceCfg(data_root=str(self.root), daily_cleanup_at="03:00"))
        service = LidarService(settings=settings)
        self.assertTrue(service.stop())

    def test_real_service_stop_waits_for_maintenance_before_closing_finalizer(self):
        service = LidarService(settings=Settings(service=ServiceCfg(data_root=str(self.root), daily_cleanup_at="03:00")))
        entered, release = threading.Event(), threading.Event()
        def busy():
            entered.set()
            release.wait(3.)
        service.maintenance.run_once = busy
        try:
            with patch.object(service.finalizer, "close", wraps=service.finalizer.close) as close:
                service.start()
                self.assertTrue(entered.wait(3.))
                self.assertFalse(service.stop(timeout=.01))
                close.assert_not_called()
                release.set()
                service._pending_integration_stop.join(3.)
                self.assertFalse(service._pending_integration_stop.is_alive())
                close.assert_called_once()
        finally:
            release.set()
            service.stop()

    def test_finished_after_cutoff_is_kept_even_for_old_capture(self):
        for mid in ("m-late-export", "m-late-send", "m-late-commit"):
            self.archive(mid)
        connection = self.outbox._connection
        connection.execute("UPDATE cloud_archives SET completed_at=? WHERE measurement_id='m-late-export'", (self.cutoff + 1,))
        connection.execute("UPDATE outbox SET sent_at_ns=? WHERE measurement_id='m-late-send'", (int((self.cutoff + 1) * 1e9),))
        self.store._connections["CH1"].execute("UPDATE pending_tasks SET committed_at=? WHERE measurement_id='m-late-commit'", (self.cutoff + 1,))
        self.cleanup.run_once()
        for mid in ("m-late-export", "m-late-send", "m-late-commit"):
            self.assertIsNotNone(self.outbox.archives.get(mid))

    def test_all_completed_empty_observations_are_purged_formal_npz_kept(self):
        old, _, old_path, _ = self.archive("m-old-empty", empty=True)
        latest, _, latest_path, _ = self.archive("m-new-empty", empty=True, ts=2000.)
        npz = self.baselines / "plate.baseline.npz"
        npz.write_bytes(b"unchanged-formal-baseline")
        baseline_cloud = self.loose("baselines/canary.xyz")
        digest = hashlib.sha256(npz.read_bytes()).hexdigest()
        self.cleanup.run_once()
        self.assertFalse(old_path.parent.exists())
        self.assertIsNone(self.outbox.archives.get(old.measurement_id))
        self.assertIsNone(self.outbox.archives.get(latest.measurement_id))
        self.assertFalse(latest_path.exists())
        self.assertIsNone(self.outbox.archives.reference("CH1"))
        self.assertTrue(baseline_cloud.exists())
        self.assertEqual(digest, hashlib.sha256(npz.read_bytes()).hexdigest())

    def test_daily_configuration_accepts_local_time_and_rejects_invalid_values(self):
        config = self.root / "channels.json"
        for value in ("", "03:00", "00:00", "23:59"):
            config.write_text(json.dumps({"service": {"daily_cleanup_at": value}, "channels": []}), encoding="utf-8")
            self.assertEqual(value, load_channels(str(config))[0].daily_cleanup_at)
        for value in (None, 3, "3:00", "24:00", "03:60", " 03:00", "０３:００"):
            config.write_text(json.dumps({"service": {"daily_cleanup_at": value}, "channels": []}), encoding="utf-8")
            with self.subTest(value=value), self.assertRaises(ValueError):
                load_channels(str(config))

    def test_loose_motion_files_use_fixed_cutoff_and_protect_active_passage(self):
        self.archive("m-pending", status="pending")
        old = self.loose("enter/CH1/old.xyz", passage="finished")
        active = self.loose("raw/CH1/active.xyz", passage="p-m-pending")
        new = self.loose("raw/CH1/new.xyz", when=self.cutoff + 1)
        boundary = self.loose("exit/CH1/boundary.xyz", when=self.cutoff)
        incomplete = self.output / "incomplete.xyz"
        incomplete.write_bytes(b"unfinished")
        self.cleanup.run_once()
        self.assertFalse(old.exists())
        for path in (active, new, boundary, incomplete):
            self.assertTrue(path.exists())
        self.assertIsNone(self.cleanup.run_once())
        restarted = StorageMaintenance(self.service, "03:00", clock=lambda: self.now)
        self.assertIsNone(restarted.run_once())

    def test_delete_failure_keeps_blob_and_durable_intent_then_resumes(self):
        bundle, _, path, _ = self.archive("m-delete-retry")
        with patch("app.cloud_archive.shutil.rmtree", side_effect=PermissionError("locked")):
            with self.assertRaises(PermissionError):
                self.cleanup.run_once()
        self.assertFalse(self.cleanup.state_path.exists())
        count = self.outbox._connection.execute("SELECT count(*) FROM cloud_archive_files WHERE measurement_id=?", (bundle.measurement_id,)).fetchone()[0]
        self.assertGreater(count, 0)
        with self.assertRaises(ValueError):
            self.outbox.archives.export(bundle.measurement_id, str(self.output), channel="CH1")
        self.cleanup.run_once()
        self.assertFalse(path.parent.exists())
        self.assertTrue(self.store.get("CH1", bundle.measurement_id).compacted)

    def test_compaction_reuses_sqlite_pages_without_claiming_file_shrink(self):
        self.archive("m-large", large=True)
        connection = self.store._connections["CH1"]
        pages_before = connection.execute("PRAGMA page_count").fetchone()[0]
        self.cleanup.run_once()
        self.assertGreater(connection.execute("PRAGMA freelist_count").fetchone()[0], 0)
        self.store.put(ResultReservation("m-next", "CH1", "p-next", "token-next"), b"{}", b"z" * 2_000_000)
        self.assertLessEqual(connection.execute("PRAGMA page_count").fetchone()[0], pages_before + 8)

    def test_bad_watermark_does_not_delete_and_automatic_thread_recovers_after_write_failure(self):
        old = self.loose("raw/CH1/old.xyz")
        self.cleanup.state_path.write_text("{broken", encoding="utf-8")
        with self.assertRaises(ValueError):
            self.cleanup.run_once()
        self.assertTrue(old.exists())
        self.cleanup.state_path.unlink()
        cleanup = StorageMaintenance(self.service, "03:00", clock=lambda: self.now, retry_interval=.01)
        actual_complete = cleanup._complete
        failed = threading.Event()
        def write(cutoff):
            if not failed.is_set():
                failed.set()
                raise PermissionError("state temporarily unavailable")
            actual_complete(cutoff)
        cleanup._complete = write
        cleanup.start()
        self.addCleanup(cleanup.close)
        self.assertTrue(failed.wait(3.))
        deadline = time.monotonic() + 3.
        while not cleanup.state_path.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(cleanup.state_path.exists())
        self.assertFalse(old.exists())
        self.assertTrue(cleanup.close())

    def test_directory_read_failure_is_retried_by_same_background_thread(self):
        import os
        old = self.loose("enter/CH1/locked.xyz")
        actual_scandir = os.scandir
        denied, release = threading.Event(), threading.Event()
        def scandir(path):
            if Path(path) == old.parent and not release.is_set():
                denied.set()
                raise PermissionError("目录读锁")
            return actual_scandir(path)
        cleanup = StorageMaintenance(self.service, "03:00", clock=lambda: self.now, retry_interval=.01)
        with patch("app.storage_maintenance.os.scandir", side_effect=scandir):
            cleanup.start()
            self.addCleanup(cleanup.close)
            self.assertTrue(denied.wait(3.))
            self.assertTrue(old.exists())
            self.assertFalse(cleanup.state_path.exists())
            release.set()
            deadline = time.monotonic() + 3.
            while not cleanup.state_path.exists() and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue(cleanup.state_path.exists())
            self.assertFalse(old.exists())
            self.assertTrue(cleanup.close())

    def test_nested_link_is_skipped_without_blocking_other_completed_samples(self):
        import os
        import subprocess
        outside = self.root / "outside"
        outside.mkdir()
        canary = outside / "canary.xyz"
        canary.write_bytes(b"external-data")
        link = self.output / "external-link"
        if os.name == "nt":
            env = dict(os.environ, CLEANUP_TEST_LINK=str(link), CLEANUP_TEST_TARGET=str(outside))
            result = subprocess.run(["powershell.exe", "-NoProfile", "-NonInteractive", "-Command",
                "New-Item -ItemType Junction -Path $env:CLEANUP_TEST_LINK -Target $env:CLEANUP_TEST_TARGET -ErrorAction Stop | Out-Null"],
                env=env, capture_output=True, text=True, timeout=20, creationflags=subprocess.CREATE_NO_WINDOW)
            self.assertEqual(0, result.returncode, result.stderr)
        else:
            link.symlink_to(outside, target_is_directory=True)
        old = self.loose("raw/old.xyz")
        self.cleanup.run_once()
        self.assertTrue(canary.exists())
        self.assertFalse(old.exists())
        self.assertTrue(self.cleanup.state_path.exists())

    def test_migration_low_disk_rejects_before_rewriting_old_large_rows_and_recovers(self):
        self.legacy_stores()
        self.archive("m-migrate-low", large=True)
        connection = self.store._connections["CH1"]
        with patch("app.cloud_archive.shutil.disk_usage", return_value=SimpleNamespace(free=0)):
            with self.assertRaises(OSError):
                self.outbox.archives.ensure_cleanup_schema()
        self.assertFalse(self.outbox.archives._cleanup_ready)
        self.assertIsNotNone(self.outbox.archives.get("m-migrate-low"))
        with patch("app.pending_measurements.shutil.disk_usage", return_value=SimpleNamespace(free=0)):
            with self.assertRaises(OSError):
                self.store.ensure_cleanup_schema()
        self.assertFalse(self.store._cleanup_ready)
        self.assertNotIn("committed_at", {r[1] for r in connection.execute("PRAGMA table_info(pending_tasks)")})
        self.assertGreater(len(self.store.get("CH1", "m-migrate-low").input_bytes), 1_000_000)
        self.cleanup.run_once()
        self.assertTrue(self.store.get("CH1", "m-migrate-low").compacted)

    def test_baseline_archive_overlap_rejected_and_delete_guard_protects_runtime_roots(self):
        bundle, _, path, _ = self.archive("m-overlap")
        permanent = path.parent / "formal"
        permanent.mkdir()
        npz = permanent / "plate.baseline.npz"
        npz.write_bytes(b"formal-baseline")
        self.service.st.channels[0].abs_baseline_dir = str(permanent)
        with self.assertRaises(ValueError):
            StorageMaintenance(self.service, "03:00")
        with self.assertRaises(ValueError):
            self.outbox.archives.purge_completed([bundle.measurement_id], self.cutoff,
                {"CH1": str(self.output)}, protected_roots=[permanent])
        self.assertEqual(b"formal-baseline", npz.read_bytes())
        self.assertFalse(self.outbox.archives.is_pruned(bundle.measurement_id))

    def test_non_stop_interrupt_retries_same_thread_and_stop_interrupt_exits(self):
        import errno
        cleanup = StorageMaintenance(self.service, "03:00", clock=lambda: self.now, retry_interval=.01)
        actual = cleanup.run_once
        calls = []
        def interrupted():
            calls.append(1)
            if len(calls) == 1:
                raise InterruptedError(errno.EINTR, "临时中断")
            return actual()
        cleanup.run_once = interrupted
        cleanup.start()
        try:
            deadline = time.monotonic() + 3.
            while not cleanup.state_path.exists() and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue(cleanup.state_path.exists())
            self.assertGreater(len(calls), 1)
            self.assertTrue(cleanup._thread.is_alive())
            self.assertTrue(any(kind == "error" for kind, _ in self.events))
        finally:
            self.assertTrue(cleanup.close())

    def legacy_stores(self):
        self.outbox = SQLiteOutbox(self.root / "legacy")
        self.store = PendingTaskStore(self.root / "legacy", ["CH1"])
        self.addCleanup(self.outbox.close)
        self.addCleanup(self.store.close)
        self.service.integration.outbox = self.outbox
        self.service.deferred.store = self.store

    def test_disabled_cleanup_keeps_old_schema_capture_and_commit_working(self):
        self.legacy_stores()
        bundle, payload, path, original = self.archive("m-legacy-live")
        self.assertNotIn("completed_at", {row[1] for row in self.outbox._connection.execute("PRAGMA table_info(cloud_archives)")})
        self.assertNotIn("committed_at", {row[1] for row in self.store._connections["CH1"].execute("PRAGMA table_info(pending_tasks)")})
        self.assertEqual(bundle, self.outbox.archives.get(bundle.measurement_id))
        self.assertTrue(path.exists())
        self.assertEqual("committed", self.store.get("CH1", bundle.measurement_id).state)
        self.assertEqual(bundle.measurement_id, self.outbox.save(payload, archive=bundle))
        self.assertFalse(self.store.put(*original).compacted)

    def assert_background_migration_recovers(self, owner):
        import sqlite3
        self.legacy_stores()
        bundle, _, path, _ = self.archive("m-migrate-" + owner)
        connection = self.outbox._connection if owner == "archive" else self.store._connections["CH1"]
        denied = threading.Event()
        def authorizer(action, table, column, database, source):
            if action == sqlite3.SQLITE_ALTER_TABLE:
                denied.set()
                return sqlite3.SQLITE_DENY
            return sqlite3.SQLITE_OK
        connection.set_authorizer(authorizer)
        cleanup = StorageMaintenance(self.service, "03:00", clock=lambda: self.now, retry_interval=.01)
        try:
            cleanup.start()
            self.assertTrue(denied.wait(3.))
            self.assertTrue(path.exists())
            self.assertFalse(cleanup.state_path.exists())
            self.assertTrue(cleanup._thread.is_alive())
            connection.set_authorizer(None)
            deadline = time.monotonic() + 3.
            while not cleanup.state_path.exists() and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue(cleanup.state_path.exists())
            self.assertFalse(path.exists())
            self.assertTrue(self.store.get("CH1", bundle.measurement_id).compacted)
        finally:
            connection.set_authorizer(None)
            cleanup.close()

    def test_archive_migration_failure_does_not_stop_background_and_recovers(self):
        self.assert_background_migration_recovers("archive")

    def test_pending_migration_failure_does_not_stop_background_and_recovers(self):
        self.assert_background_migration_recovers("pending")

    def test_disabled_lane_old_tasks_protect_archive_and_entry_cloud(self):
        hidden = PendingTaskStore(self.root / "outbox", ["CH2"])
        identity = ResultReservation("m-hidden", "CH2", "p-hidden", "token-hidden")
        hidden.put(identity, b"{}", b"original-disabled-lane-cloud")
        hidden.close()  # 模拟重启后只启用CH1；CH2旧库没有新清理列。
        self.service.st.channels.append(SimpleNamespace(id="CH2", abs_output_dir=str(self.output),
                                                        abs_baseline_dir=str(self.baselines)))
        bundle = example_bundle(measurement="m-hidden", channel="CH2", passage="p-hidden", empty=False)
        self.outbox.save(dict(payload_for(bundle), verdict="loaded"), archive=bundle)
        path = Path(self.outbox.archives.export("m-hidden", str(self.output), channel="CH2"))
        self.outbox.mark_sent("m-hidden")
        entry = self.loose("enter/CH2/hidden.xyz", passage="p-hidden")
        _, _, completed, _ = self.archive("m-enabled-done")
        cleanup = StorageMaintenance(self.service, "03:00", clock=lambda: self.now)
        cleanup.run_once()
        self.assertTrue(path.exists())
        self.assertTrue(entry.exists())
        self.assertIsNotNone(self.outbox.archives.get("m-hidden"))
        self.assertFalse(completed.exists())
        self.assertTrue(cleanup.state_path.exists())

    def test_unreadable_old_lane_database_prevents_success_watermark_and_deletion(self):
        import sqlite3
        unknown = self.store.pending_dir / "channel-orphan.sqlite3"
        unknown.write_bytes(b"invalid-historical-task-store")
        old = self.loose("raw/old.xyz")
        with self.assertRaises(sqlite3.DatabaseError):
            self.cleanup.run_once()
        self.assertTrue(old.exists())
        self.assertFalse(self.cleanup.state_path.exists())

    def test_second_lane_migration_failure_reopens_and_completes_missing_columns(self):
        import sqlite3
        store = PendingTaskStore(self.root / "migration-two", ["CH1", "CH2"])
        connection = store._connections["CH2"]
        connection.set_authorizer(lambda action, *_: sqlite3.SQLITE_DENY
                                  if action == sqlite3.SQLITE_ALTER_TABLE else sqlite3.SQLITE_OK)
        try:
            with self.assertRaises(sqlite3.DatabaseError):
                store.ensure_cleanup_schema()
            self.assertFalse(store._cleanup_ready)
            for channel, expected in (("CH1", True), ("CH2", False)):
                columns = {row[1] for row in store._connections[channel].execute("PRAGMA table_info(pending_tasks)")}
                self.assertEqual(expected, "compacted_at" in columns)
        finally:
            connection.set_authorizer(None)
            store.close()
        reopened = PendingTaskStore(self.root / "migration-two", ["CH1", "CH2"])
        self.addCleanup(reopened.close)
        reopened.ensure_cleanup_schema()
        for connection in reopened._connections.values():
            columns = {row[1] for row in connection.execute("PRAGMA table_info(pending_tasks)")}
            self.assertTrue({"committed_at", "compacted_at", "metadata_sha256"}.issubset(columns))
