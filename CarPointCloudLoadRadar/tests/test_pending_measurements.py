"""待计算任务的持久化、身份、容量和通道隔离定向回归。"""
from contextlib import closing
from dataclasses import replace
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

import app.pending_measurements as pending_tasks
from app.load_radar_bridge import ResultReservation
from app.pending_measurements import PendingTaskStore


class TestPendingMeasurements(unittest.TestCase):
    def setUp(self):
        root = Path(tempfile.gettempdir()) / "load-radar-async-tests"
        root.mkdir(exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(prefix="pending-tasks-", dir=root)
        self.addCleanup(self.tmp.cleanup)
        self.storage = Path(self.tmp.name) / "store"
        self.store = self.open_store()

    def open_store(self, **kw):
        store = PendingTaskStore(self.storage, ("CH1", "CH2", "CH3", "CH4"), **kw)
        self.addCleanup(store.close)
        return store

    @staticmethod
    def identity(number=1, channel="CH1", passage=None, command=None):
        return ResultReservation("m-%s" % number, channel, passage or "p-%s" % number,
                                 "token-%s" % number, command)

    def put(self, identity=None, **kw):
        return self.store.put(identity or self.identity(), kw.get("start", b'{"passageId":"p-1"}'),
                              kw.get("data", b"original-cloud"))

    def test_put_same_id_is_byte_exact_and_conflict_does_not_overwrite(self):
        row = self.put()
        self.assertEqual(row, self.put())
        for change in ({"data": b"changed"}, {"start": b'{"passageId":"other"}'}):
            with self.assertRaises(ValueError):
                self.put(**change)
        with self.assertRaises(ValueError):
            self.put(replace(row.identity, origin_token="new-token"))
        self.assertEqual(row, self.store.get("CH1", row.identity.measurement_id))

    def test_same_passage_reuses_original_task_across_new_session_and_restart(self):
        first = self.put()
        duplicate = self.put(self.identity(2, passage="p-1"), data=b"later-frame")
        self.assertEqual(first, duplicate)
        self.store.close()
        self.store = self.open_store()
        self.assertEqual(first, self.put(self.identity(3, passage="p-1")))
        self.assertIsNone(self.store.get("CH1", "m-2"))

    def test_abrupt_process_exit_retains_input_revision_and_cleanup_protection(self):
        """子进程不执行close/finally，重启仍恢复原输入与待发布修订。"""
        self.store.close()
        child = """
import os, sys
from app.pending_measurements import PendingTaskStore
from app.load_radar_bridge import ResultReservation
store = PendingTaskStore(sys.argv[1], ('CH1',))
identity = ResultReservation('m-crash', 'CH1', 'p-crash', 'token-crash', None)
store.put(identity, b'{"passageId":"p-crash"}', b'original-cloud')
store.register_recalculation('CH1', 'm-crash', dict(measuredAt='2026-11-03T00:00:00+08:00',
    sourceMode='live', verdict='loaded', licensePlate='TEST-CRASH'))
store.save_recalculation('CH1', 'revision-crash', 'm-crash', b'{"volumeM3":49.69,"volumeStatus":"success"}')
os._exit(91)
"""
        result = subprocess.run([sys.executable, '-c', child, str(self.storage)],
            cwd=Path(__file__).resolve().parents[1], capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 91, result.stderr)
        self.store = self.open_store()
        row = self.store.get('CH1', 'm-crash')
        self.assertEqual(row.input_bytes, b'original-cloud')
        self.assertEqual(row.identity.passage_id, 'p-crash')
        self.assertEqual(self.store.pending_recalculation('CH1'), (
            'revision-crash', 'm-crash', b'{"volumeM3":49.69,"volumeStatus":"success"}'))
        self.assertIn('m-crash', self.store.protected_measurements(2000000000.))

    def test_old_source_fingerprint_task_calculates_after_store_reopens(self):
        from app.frozen_detection import FrozenDetection, calculate_frozen
        from app.measurement_runtime import canonical
        from tests.test_frozen_detection import fixture, baseline_and_model

        worker, frame, stability, identity, start = fixture()
        baseline = baseline_and_model(worker, identity)
        data = FrozenDetection.freeze(worker, frame, stability, identity, start)
        record = json.loads(data)
        record['algorithm'] = dict(volume={**record['algorithm'], 'fingerprint': 'f' * 64},
                                   service='0' * 64, adapter='1' * 64)
        old_input = canonical(record)
        self.store.put(identity, canonical(start), old_input)
        self.store.close()
        self.store = self.open_store()
        task = self.store.get(identity.channel, identity.measurement_id)
        self.assertEqual(old_input, task.input_bytes)
        result = calculate_frozen(task.input_bytes, task.identity, baseline, '宁A05922F',
                                  worker.integration.config)
        payload = json.loads(result.result_json)
        self.assertEqual(identity, result.identity)
        self.assertEqual('version-original', result.baseline_version_id)
        self.assertEqual('宁A05922F', payload['plate'])
        self.assertAlmostEqual(9.6, payload['volume']['volume_m3'], places=4)

    def test_manual_command_deduplicates_request_separately_from_automatic(self):
        automatic = self.put()
        command = json.dumps({"requestId": "r-1", "channelId": "CH1", "passageId": "p-1",
                              "kind": "remeasure", "confirmedEmpty": False}).encode()
        manual = self.put(self.identity(2, passage="p-1", command=command))
        self.assertNotEqual(automatic.identity, manual.identity)
        self.assertEqual(manual, self.put(self.identity(3, passage="p-1", command=command)))
        changed = command.replace(b"remeasure", b"baseline-capture")
        with self.assertRaises(ValueError):
            self.put(self.identity(4, passage="p-1", command=changed))

    def test_freeze_once_persists_original_bytes_and_transition(self):
        row = self.put()
        self.store.freeze_preparation("CH1", "m-1", b"proof")
        self.store.freeze_preparation("CH1", "m-1", b"proof")
        with self.assertRaises(ValueError):
            self.store.freeze_preparation("CH1", "m-1", b"different")
        self.store.freeze_prepared("CH1", "m-1", b"finished-payload-and-archive")
        self.store.freeze_prepared("CH1", "m-1", b"finished-payload-and-archive")
        with self.assertRaises(ValueError):
            self.store.freeze_prepared("CH1", "m-1", b"replacement")
        self.store.close()
        self.store = self.open_store()
        actual = self.store.get("CH1", "m-1")
        self.assertEqual("prepared", actual.state)
        self.assertEqual(row.identity, actual.identity)
        self.assertEqual(b"proof", actual.preparation_bytes)
        self.assertEqual(b"finished-payload-and-archive", actual.prepared_bytes)
        self.assertEqual([actual], self.store.due("CH1", now=10**12))

    def test_prepared_requires_preparation_and_commit_requires_prepared(self):
        self.put()
        with self.assertRaises(ValueError):
            self.store.freeze_prepared("CH1", "m-1", b"premature")
        with self.assertRaises(ValueError):
            self.store.mark_committed("CH1", "m-1")

    def test_retry_keeps_frozen_bytes_and_due_uses_persisted_wall_time(self):
        self.put()
        self.store.freeze_preparation("CH1", "m-1", b"proof")
        self.store.freeze_prepared("CH1", "m-1", b"prepared")
        self.store.retry("CH1", "m-1", "temporary", 5, now=100)
        self.assertEqual([], self.store.due("CH1", now=104.9))
        self.store.close()
        self.store = self.open_store()
        row, = self.store.due("CH1", now=105)
        self.assertEqual((1, 105, "temporary", "prepared"),
                         (row.attempts, row.next_at, row.error, row.state))
        self.assertEqual(b"prepared", row.prepared_bytes)

    def test_retry_rejects_invalid_delay_without_mutating_task(self):
        original = self.put()
        for delay in (None, -1, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                self.store.retry("CH1", "m-1", "invalid", delay, now=100)
        self.assertEqual(original, self.store.get("CH1", "m-1"))

    def test_due_orders_equal_deadlines_by_creation_not_prepared_state(self):
        self.put()
        self.put(self.identity(2))
        self.store.freeze_preparation("CH1", "m-2", b"proof")
        self.store.freeze_prepared("CH1", "m-2", b"prepared")
        self.assertEqual("m-1", self.store.due("CH1", now=100)[0].identity.measurement_id)

    def test_prepared_short_retry_does_not_starve_unprepared_tasks(self):
        self.put()
        self.store.freeze_preparation("CH1", "m-1", b"proof")
        self.store.freeze_prepared("CH1", "m-1", b"prepared")
        for number in range(2, 5):
            self.put(self.identity(number))
        selected = []
        # 轮询周期大于prepared的0.2秒退避，模拟真实processor的1至2秒循环。
        for now in range(100, 110, 2):
            row, = self.store.due("CH1", now=now)
            selected.append(row.identity.measurement_id)
            self.store.retry("CH1", row.identity.measurement_id, "仍等待", .2, now=now)
        self.assertEqual(["m-1", "m-2", "m-3", "m-4", "m-1"], selected)
        self.assertEqual(b"prepared", self.store.get("CH1", "m-1").prepared_bytes)

    def test_corrupted_frozen_input_or_result_is_rejected(self):
        for field in ("input_bytes", "preparation_bytes", "prepared_bytes"):
            number = len(field)
            identity = self.identity(number)
            self.put(identity)
            self.store.freeze_preparation("CH1", identity.measurement_id, b"proof")
            self.store.freeze_prepared("CH1", identity.measurement_id, b"prepared")
            self.store.mark_committed("CH1", identity.measurement_id)
            self.store._connections["CH1"].execute(
                "UPDATE pending_tasks SET " + field + "=? WHERE measurement_id=?",
                (b"corrupted", identity.measurement_id))
            with self.assertRaises(ValueError):
                self.store.get("CH1", identity.measurement_id)

    def test_blocked_task_is_preserved_but_not_due_or_resurrected(self):
        self.put()
        self.store.block("CH1", "m-1", "identity-conflict")
        self.assertEqual([], self.store.due("CH1", now=10**12))
        row = self.store.get("CH1", "m-1")
        self.assertEqual("blocked", row.state)
        with self.assertRaises(ValueError):
            self.store.retry("CH1", "m-1", "retry", 1)
        with self.assertRaises(ValueError):
            self.store.freeze_preparation("CH1", "m-1", b"proof")
        self.assertEqual(row, self.put())

    def test_committed_keeps_all_bytes_and_is_idempotent(self):
        self.put()
        self.store.freeze_preparation("CH1", "m-1", b"proof")
        self.store.freeze_prepared("CH1", "m-1", b"prepared")
        self.store.mark_committed("CH1", "m-1")
        self.store.mark_committed("CH1", "m-1")
        row = self.store.get("CH1", "m-1")
        self.assertEqual(("committed", b"original-cloud", b"proof", b"prepared"),
                         (row.state, row.input_bytes, row.preparation_bytes, row.prepared_bytes))
        self.assertEqual([], self.store.due("CH1", now=10**12))
        self.assertEqual(row, self.put(self.identity(2, passage="p-1")))

    def test_count_limit_includes_blocked_and_excludes_committed(self):
        self.store.close()
        self.store = self.open_store(max_jobs_per_channel=1)
        self.put()
        with self.assertRaises(RuntimeError):
            self.put(self.identity(2))
        self.put(self.identity(2, channel="CH2"))
        self.store.freeze_preparation("CH1", "m-1", b"proof")
        self.store.freeze_prepared("CH1", "m-1", b"prepared")
        self.store.mark_committed("CH1", "m-1")
        self.put(self.identity(2))
        self.store.block("CH1", "m-2", "blocked")
        with self.assertRaises(RuntimeError):
            self.put(self.identity(3))
        self.assertEqual("committed", self.store.get("CH1", "m-1").state)

    def test_byte_capacity_is_checked_for_insert_and_freeze_growth(self):
        self.store.close()
        self.store = self.open_store(max_bytes_per_channel=1024,
            max_completion_bytes_per_channel=pending_tasks.MAX_PREPARED_TASK_BYTES + 128)
        self.put()
        original = self.store.get("CH1", "m-1")
        with self.assertRaises(pending_tasks.CompletionBusy):
            self.store.freeze_preparation("CH1", "m-1", b"x" * 1024)
        self.assertEqual(original, self.store.get("CH1", "m-1"))
        with self.assertRaises(RuntimeError):
            self.put(self.identity(2), data=b"x" * 1024)
        self.assertIsNone(self.store.get("CH1", "m-2"))
        self.store.freeze_preparation("CH1", "m-1", b"proof")
        with patch.object(pending_tasks, "MAX_PREPARED_TASK_BYTES", 128):
            with self.assertRaises(ValueError):
                self.store.freeze_prepared("CH1", "m-1", b"x" * 129)
        self.assertIsNone(self.store.get("CH1", "m-1").prepared_bytes)

    def test_completion_reserves_before_compute_and_commit_releases_for_next_task(self):
        self.put()
        self.put(self.identity(2))
        self.store.freeze_preparation("CH1", "m-1", b"proof")
        before = self.store.get("CH1", "m-2")
        with self.assertRaises(pending_tasks.CompletionBusy):
            self.store.freeze_preparation("CH1", "m-2", b"other-proof")
        self.assertEqual(before, self.store.get("CH1", "m-2"))
        snap = self.store.snapshot("CH1")
        self.assertEqual(len(b"proof") + pending_tasks.MAX_PREPARED_TASK_BYTES,
                         snap["completion_reserved_bytes"])
        self.store.freeze_prepared("CH1", "m-1", b"prepared")
        self.store.mark_committed("CH1", "m-1")
        self.assertEqual(0, self.store.snapshot("CH1")["completion_reserved_bytes"])
        self.store.freeze_preparation("CH1", "m-2", b"other-proof")
        self.assertEqual(b"prepared", self.store.get("CH1", "m-1").prepared_bytes)

    def test_blocked_proof_releases_unused_reservation_and_keeps_actual_bytes_on_restart(self):
        self.put()
        self.put(self.identity(2))
        self.store.freeze_preparation("CH1", "m-1", b"proof")
        self.store.block("CH1", "m-1", "manual-review")
        self.assertEqual(5, self.store.snapshot("CH1")["completion_reserved_bytes"])
        self.store.close()
        self.store = self.open_store()
        self.assertEqual(5, self.store.snapshot("CH1")["completion_reserved_bytes"])
        self.store.freeze_preparation("CH1", "m-2", b"other-proof")
        self.assertEqual("blocked", self.store.get("CH1", "m-1").state)
        self.assertEqual(b"proof", self.store.get("CH1", "m-1").preparation_bytes)

    def test_blocked_prepared_retains_only_actual_completion_bytes(self):
        self.put()
        self.put(self.identity(2))
        self.store.freeze_preparation("CH1", "m-1", b"proof")
        self.store.freeze_prepared("CH1", "m-1", b"prepared")
        self.store.block("CH1", "m-1", "payload-conflict")
        self.assertEqual(13, self.store.snapshot("CH1")["completion_reserved_bytes"])
        self.store.freeze_preparation("CH1", "m-2", b"other-proof")
        self.assertEqual(b"prepared", self.store.get("CH1", "m-1").prepared_bytes)

    def test_completion_bytes_do_not_consume_original_input_pool(self):
        self.store.close()
        self.store = self.open_store(max_bytes_per_channel=1024)
        self.put()
        initial = self.store.snapshot("CH1")["pending_bytes"]
        self.store.freeze_preparation("CH1", "m-1", b"p" * 4096)
        self.store.freeze_prepared("CH1", "m-1", b"f" * 2048)
        self.assertEqual(initial, self.store.snapshot("CH1")["pending_bytes"])
        self.put(self.identity(2))
        self.assertEqual(6144, self.store.snapshot("CH1")["completion_bytes"])
        self.assertEqual(2, self.store.snapshot("CH1")["pending_jobs"])

    def test_legacy_preparation_gets_durable_completion_reservation(self):
        self.put()
        self.store.freeze_preparation("CH1", "m-1", b"proof")
        database = self.store._connections["CH1"].execute("PRAGMA database_list").fetchone()[2]
        self.store.close()
        with closing(sqlite3.connect(database)) as connection:
            connection.execute("ALTER TABLE pending_tasks DROP COLUMN completion_reserved_bytes")
            connection.commit()
        self.store = self.open_store()
        self.assertEqual(pending_tasks.MAX_PREPARED_TASK_BYTES + 5,
                         self.store.snapshot("CH1")["completion_reserved_bytes"])
        self.store.freeze_prepared("CH1", "m-1", b"prepared")
        self.assertEqual(b"prepared", self.store.get("CH1", "m-1").prepared_bytes)

    def test_legacy_blocked_task_migrates_only_actual_completion_bytes(self):
        self.put()
        self.store.freeze_preparation("CH1", "m-1", b"proof")
        self.store.freeze_prepared("CH1", "m-1", b"prepared")
        self.store.block("CH1", "m-1", "manual-review")
        database = self.store._connections["CH1"].execute("PRAGMA database_list").fetchone()[2]
        self.store.close()
        with closing(sqlite3.connect(database)) as connection:
            connection.execute("ALTER TABLE pending_tasks DROP COLUMN completion_reserved_bytes")
            connection.commit()
        self.store = self.open_store()
        self.assertEqual(13, self.store.snapshot("CH1")["completion_reserved_bytes"])
        self.put(self.identity(2))
        self.store.freeze_preparation("CH1", "m-2", b"other-proof")
        self.assertEqual(b"prepared", self.store.get("CH1", "m-1").prepared_bytes)

    def test_other_channel_is_not_blocked_by_one_channel_lock(self):
        errors = []
        done = threading.Event()
        def write_other():
            try:
                self.put(self.identity(2, channel="CH2"))
            except Exception as error:
                errors.append(error)
            finally:
                done.set()
        with self.store._locks["CH1"]:
            thread = threading.Thread(target=write_other)
            thread.start()
            self.assertTrue(done.wait(2), "CH1的锁不能阻塞CH2")
        thread.join(2)
        self.assertEqual([], errors)

    def test_concurrent_same_passage_returns_one_original_id(self):
        results, errors = [], []
        barrier = threading.Barrier(4)
        def put(number):
            try:
                barrier.wait(2)
                results.append(self.put(self.identity(number, passage="p-same")))
            except Exception as error:
                errors.append(error)
        threads = [threading.Thread(target=put, args=(i,)) for i in range(1, 5)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(3)
        self.assertEqual([], errors)
        self.assertEqual(4, len(results))
        self.assertEqual(1, len({row.identity.measurement_id for row in results}))

    def test_database_files_stay_under_pending_with_wal_full(self):
        paths = list((self.storage / "pending").glob("*.sqlite3"))
        self.assertEqual(4, len(paths))
        for connection in self.store._connections.values():
            self.assertEqual("wal", connection.execute("PRAGMA journal_mode").fetchone()[0])
            self.assertEqual(2, connection.execute("PRAGMA synchronous").fetchone()[0])
        self.assertTrue(all(path.resolve().is_relative_to((self.storage / "pending").resolve()) for path in paths))

    def test_snapshot_does_not_load_cloud_blobs(self):
        self.put()
        statements = []
        self.store._connections["CH1"].set_trace_callback(statements.append)
        snap = self.store.snapshot("CH1")
        self.assertEqual(1, snap["pending_jobs"])
        self.assertGreater(snap["pending_bytes"], len(b"original-cloud"))
        self.assertFalse(any("select *" in sql.lower() for sql in statements))

    def test_invalid_inputs_unknown_channel_and_closed_store(self):
        for value in (-1, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                self.store.due("CH1", now=value)
        with self.assertRaises(ValueError):
            self.store.due("unknown")
        with self.assertRaises(ValueError):
            self.store.put(self.identity(), b"not-json", b"cloud")
        with self.assertRaises(ValueError):
            self.store.put(replace(self.identity(), passage_id=None), b"{}", b"cloud")
        self.store.close()
        self.store.close()
        with self.assertRaises(RuntimeError):
            self.store.get("CH1", "m-1")

    def test_path_symlink_is_rejected_when_supported(self):
        target = Path(self.tmp.name) / "target"
        target.mkdir()
        link = Path(self.tmp.name) / "link"
        try:
            link.symlink_to(target, target_is_directory=True)
        except OSError as error:
            self.skipTest("本机不允许创建符号链接: %s" % type(error).__name__)
        with self.assertRaises(ValueError):
            PendingTaskStore(link, ("CH1",))

    def test_failed_write_rolls_back_and_can_continue(self):
        connection = self.store._connections["CH1"]
        connection.execute("CREATE TRIGGER reject_task BEFORE INSERT ON pending_tasks BEGIN SELECT RAISE(ABORT, 'forced'); END")
        with self.assertRaises(sqlite3.DatabaseError):
            self.put()
        connection.execute("DROP TRIGGER reject_task")
        self.assertEqual("m-1", self.put().identity.measurement_id)


if __name__ == "__main__":
    unittest.main()
