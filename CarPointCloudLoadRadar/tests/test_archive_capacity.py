"""容量盘点只读、有界、单飞和所有者关闭；仅使用临时文件和测试数据库。"""
import copy
import io
import json
import os
from pathlib import Path
import sqlite3
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from app.settings import BASELINE_SUBDIR
from app.archive_capacity import CapacityProbe, CapacityScanLimits, scan_capacity_files
from app.cloud_archive import ArchiveStore
from app.load_radar_transport import SQLiteOutbox
from app.result_finalizer import ResultFinalizer
from tests.test_cloud_archive import example_bundle, payload_for


def completed(owner):
    deadline = time.perf_counter() + 5
    while time.perf_counter() < deadline:
        result = owner.capacity_snapshot() if hasattr(owner, "capacity_snapshot") else owner.snapshot()
        if result["status"] == "closed":
            return result
        if not result["query_running"] and result["cached"] is not None:
            return result["cached"]
        threading.Event().wait(.005)
    raise AssertionError("容量采样未完成")


class TestCapacityFiles(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_empty_and_missing_are_distinct(self):
        empty = scan_capacity_files([self.root])
        self.assertEqual(("complete", 0, 0), (empty["status"], empty["files"], empty["apparent_bytes"]))
        self.assertEqual("unavailable", scan_capacity_files([self.root / "missing"])["status"])

    def test_explicit_only_stat_and_categories(self):
        paths = {"archives-v1/m/detection.xyz": 20, "outbox.sqlite3": 7, "outbox.sqlite3-wal": 8,
                 "outbox.sqlite3-shm": 9, "a" * 64 + ".npz": 11, "old.xyz": 13,
                 "old.xyz.meta.json": 17, ".pending-one/member": 19}
        for name, size in paths.items():
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"x" * size)
        with patch("builtins.open", side_effect=AssertionError("只允许stat")), \
                patch("sqlite3.connect", side_effect=AssertionError("不可开库")):
            report = scan_capacity_files([self.root])
        self.assertEqual("complete", report["status"])
        self.assertEqual(sum(paths.values()), report["apparent_bytes"])
        self.assertEqual(8, report["files"])
        self.assertEqual(20, report["categories"]["archive_projection_unverified"]["apparent_bytes"])
        self.assertEqual("unknown", report["database_references"])
        self.assertEqual("not_checked", report["integrity"])

    def test_overlap_and_hardlink_not_double_counted(self):
        nested = self.root / "nested"
        nested.mkdir()
        (nested / "a.xyz").write_bytes(b"123")
        os.link(nested / "a.xyz", nested / "b.xyz")
        report = scan_capacity_files([nested, self.root, self.root])
        self.assertEqual(("complete", 1, 3, 1, 1), (report["status"], report["files"], report["apparent_bytes"],
                                                  report["hardlink_aliases"], report["overlapping_directories"]))
        os.link(nested / "a.xyz", nested / "other.db")
        report = scan_capacity_files([self.root])
        self.assertEqual("partial", report["status"])
        self.assertIn("hardlink_category_ambiguous", [e["code"] for e in report["errors"]])

    def test_limits_and_invalid_inputs(self):
        for value in (0, -1, True, 1.5):
            with self.subTest(value=value), self.assertRaises(ValueError):
                CapacityScanLimits(entries=value)
        for value in (float("nan"), float("inf"), 0):
            with self.assertRaises(ValueError):
                CapacityScanLimits(seconds=value)
        for roots in ([], str(self.root), [""]):
            with self.assertRaises(ValueError):
                scan_capacity_files(roots)
        for i in range(5):
            (self.root / str(i)).write_bytes(b"x")
        report = scan_capacity_files([self.root], limits=CapacityScanLimits(entries=2))
        self.assertEqual(("partial", 2), (report["status"], report["files"]))
        with patch("app.archive_capacity.time.monotonic", side_effect=[0., 10., 10.]):
            self.assertEqual("unavailable", scan_capacity_files([self.root])["status"])

    def test_symlink_is_not_followed(self):
        (self.root / "file").write_bytes(b"123")
        try:
            (self.root / "link").symlink_to(self.root / "file")
        except OSError:
            self.skipTest("当前Windows用户没有创建符号链接权限")
        report = scan_capacity_files([self.root])
        self.assertEqual(("partial", 3), (report["status"], report["apparent_bytes"]))

    def test_stat_failure_is_unknown_not_empty(self):
        with patch("app.archive_capacity._path_guard", side_effect=PermissionError()):
            self.assertEqual("unavailable", scan_capacity_files([self.root])["status"])

    def test_depth_and_error_list_are_bounded(self):
        (self.root / "a" / "b" / "c").mkdir(parents=True)
        report = scan_capacity_files([self.root], limits=CapacityScanLimits(depth=1))
        self.assertEqual("partial", report["status"])
        self.assertIn("depth_budget_exceeded", [e["code"] for e in report["errors"]])
        report = scan_capacity_files([self.root / "missing1", self.root / "missing2"],
                                     limits=CapacityScanLimits(errors=1))
        self.assertEqual((2, 1, True), (report["error_count"], len(report["errors"]), report["errors_truncated"]))

    def test_cli_no_config_and_report_is_separate_exclusive(self):
        from cli.capacity import main
        source = self.root / "source"
        source.mkdir()
        target = self.root / "report.json"
        with patch("sys.stdout", new=io.StringIO()), patch("sqlite3.connect", side_effect=AssertionError("不可开库")):
            self.assertEqual(0, main(["--dir", str(source), "--json", str(target)]))
            self.assertEqual("complete", json.loads(target.read_text(encoding="utf-8"))["status"])
        for name in (target, source / "inside.json", Path(__file__).resolve().parents[1] / "re" / "forbidden.json",
                     Path(__file__).resolve().parents[1].joinpath(*BASELINE_SUBDIR, "forbidden.json")):
            with self.subTest(path=name), patch("sys.stderr", new=io.StringIO()), self.assertRaises(SystemExit):
                main(["--dir", str(source), "--json", str(name)])

    def test_cli_rejects_windows_ads_before_scan_or_write(self):
        from cli.capacity import main
        for name in (str(self.root) + ":report.json", str(self.root / "re") + ":report.json",
                     str(self.root / "loaded") + ":report.json", "\\\\?\\C:\\report.json"):
            with self.subTest(path=name), patch("sys.stderr", new=io.StringIO()), \
                    patch("cli.capacity.scan_capacity_files", side_effect=AssertionError("应提前拒绝")), \
                    self.assertRaises(SystemExit):
                main(["--dir", str(self.root), "--json", name])


class TestCapacityDatabase(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.outbox = SQLiteOutbox(self.temp.name)
        self.addCleanup(self.outbox.close)
        self.db = self.outbox._connection

    def probe(self, budget=6000, kind="outbox", lock=None):
        return CapacityProbe(self.db, lock or self.outbox._lock, lambda: self.outbox.archives._closed,
                             kind, row_budget=budget)

    def test_single_statement_utf8_and_no_side_effects_or_blob_output(self):
        bundle = example_bundle()
        self.outbox.save(payload_for(bundle), archive=bundle)
        self.outbox.save(dict(measurementId="legacy", channelId="CH1", reason="中文"))
        expected = sum(len(r[0].encode("utf-8")) for r in self.db.execute("SELECT payload_json FROM outbox"))
        before = self.db.total_changes
        trace = []
        self.db.set_trace_callback(trace.append)
        self.addCleanup(lambda: self.db.set_trace_callback(None))
        factory = self.db.row_factory
        result = completed(self.outbox)
        self.assertEqual("complete", result["status"])
        self.assertEqual(1, len([sql for sql in trace if sql.startswith("SELECT")]))
        self.assertEqual(["-- PRAGMA encoding"], [sql for sql in trace if not sql.startswith("SELECT")])
        self.assertTrue(trace[0].startswith("SELECT"))
        self.assertEqual(before, self.db.total_changes)
        self.assertFalse(self.db.in_transaction)
        self.assertIs(factory, self.db.row_factory)
        table = result["tables"]["outbox"]
        self.assertEqual(expected, table["logical_bytes"])
        self.assertEqual({"pending/projection_pending": 1, "pending/no_archive": 1}, table["ack_projection_pairs"])
        self.assertNotIn("单测结果", json.dumps(result, ensure_ascii=False))
        self.assertNotIn("manifest", result)

    def test_limits_below_equal_above_and_deep_copy(self):
        for count in range(4):
            with self.subTest(count=count):
                result = completed(self.probe(budget=12))
                table = result["tables"]["outbox"]
                self.assertEqual(min(count, 2), table["observed_rows"])
                self.assertEqual(count > 2, table["truncated"])
                self.assertLessEqual(result["returned_rows"], 18)
            self.outbox.save(dict(measurementId="m-%d" % count, channelId="CH1"))
        probe = self.probe()
        completed(probe)
        snap = probe.snapshot()
        snap["cached"]["tables"].clear()
        self.assertTrue(probe.snapshot()["cached"]["tables"])

    def test_parent_outside_sample_and_dangling_metadata(self):
        for i in range(4):
            bundle = example_bundle("m-%d" % i, passage="p-%d" % i, ts=1000. + i)
            self.outbox.save(payload_for(bundle), archive=bundle)
        self.db.execute("INSERT INTO empty_observation_latest VALUES ('CH1','m-3',1003.,4)")
        report = completed(self.probe(budget=6))
        self.assertTrue(report["tables"]["cloud_archives"]["truncated"])
        self.assertEqual({"matching_metadata": 1}, report["tables"]["empty_observation_latest"]["references"])
        self.db.execute("UPDATE empty_observation_latest SET archive_seq=99")
        report = completed(self.probe(budget=6))
        self.assertEqual({"missing_or_mismatched": 1}, report["tables"]["empty_observation_latest"]["references"])

    def test_ack_and_projection_are_independent(self):
        expected = {}
        for ack in ("pending", "sent", "blocked"):
            for projection in ("pending", "complete"):
                name = ack + "-" + projection
                bundle = example_bundle(name)
                self.outbox.save(payload_for(bundle), archive=bundle)
                self.db.execute("UPDATE outbox SET status=? WHERE measurement_id=?", (ack, name))
                self.db.execute("UPDATE cloud_archives SET status=? WHERE measurement_id=?", (projection, name))
                expected[ack + "/projection_" + projection] = 1
        report = completed(self.outbox)
        self.assertEqual(expected, report["tables"]["outbox"]["ack_projection_pairs"])
        self.assertEqual("not_checked", report["integrity"])

    def test_transaction_no_sql_no_rollback(self):
        self.db.execute("BEGIN")
        trace = []
        self.db.set_trace_callback(trace.append)
        try:
            self.assertEqual("transaction_active", completed(self.outbox)["reason"])
            self.assertEqual([], trace)
            self.assertTrue(self.db.in_transaction)
        finally:
            self.db.set_trace_callback(None)
            self.db.rollback()

    def test_missing_table_and_existing_authorizer_fail_closed(self):
        def deny(*args):
            return sqlite3.SQLITE_DENY
        self.db.set_authorizer(deny)
        report = completed(self.probe())
        self.assertEqual("unavailable", report["status"])
        with self.assertRaises(sqlite3.DatabaseError):
            self.db.execute("SELECT 1")
        self.db.set_authorizer(None)
        self.db.execute("DROP TABLE empty_observations")
        self.assertEqual("unavailable", completed(self.probe())["status"])

    def test_existing_progress_callback_not_replaced(self):
        calls = []
        def interrupt():
            calls.append(True)
            return 1
        self.db.set_progress_handler(interrupt, 1)
        try:
            self.assertEqual("unavailable", completed(self.probe())["status"])
            self.assertTrue(calls)
            with self.assertRaises(sqlite3.OperationalError):
                self.db.execute("SELECT 1")
        finally:
            self.db.set_progress_handler(None, 0)

    def test_queued_query_sees_close_without_any_sql(self):
        trace = []
        self.db.set_trace_callback(trace.append)
        with self.outbox._lock:
            probe = self.probe()
            probe.snapshot()
            # 本线程的RLock可重入，工作线程尚未取得锁；先关闭再释放。
            self.outbox.close()
        deadline = time.perf_counter() + 3
        while probe.snapshot()["query_running"] and time.perf_counter() < deadline:
            threading.Event().wait(.005)
        self.assertFalse(probe.snapshot()["query_running"])
        self.assertEqual([], trace)

    def test_busy_and_closed_not_empty(self):
        lock = threading.Lock()
        lock.acquire()
        try:
            self.assertEqual("owner_lock_busy", completed(self.probe(lock=lock))["reason"])
        finally:
            lock.release()
        self.outbox.close()
        self.assertEqual("closed", self.outbox.capacity_snapshot()["status"])

    def test_slow_query_single_flight_close_and_no_post_close_sql(self):
        entered, release, closed = threading.Event(), threading.Event(), threading.Event()
        trace = []
        def slow(sql):
            trace.append(sql)
            entered.set()
            release.wait(5)
        self.db.set_trace_callback(slow)
        first = self.outbox.capacity_snapshot()
        self.assertTrue(first["query_running"])
        self.assertTrue(entered.wait(2))
        start = time.perf_counter()
        for _ in range(50):
            self.assertTrue(self.outbox.capacity_snapshot(refresh=True)["query_running"])
        self.assertLess(time.perf_counter() - start, .5)
        def close():
            self.outbox.close()
            closed.set()
        thread = threading.Thread(target=close)
        thread.start()
        try:
            self.assertFalse(closed.wait(.05))
        finally:
            release.set()
            thread.join(5)
        self.assertTrue(closed.is_set())
        self.assertEqual("closed", self.outbox.capacity_snapshot()["status"])
        self.assertEqual(1, len([sql for sql in trace if sql.startswith("SELECT")]))
        self.assertEqual(["-- PRAGMA encoding"], [sql for sql in trace if not sql.startswith("SELECT")])

    def test_bad_budget_and_unknown_owner(self):
        for budget in (0, 5, True, 100001):
            with self.assertRaises(ValueError):
                self.probe(budget)
        with self.assertRaises(ValueError):
            self.probe(kind="anything")

    def test_utf16_database_is_not_reported_as_utf8_bytes(self):
        from app.cloud_archive import ArchiveJournal
        for encoding in ("UTF-16le", "UTF-16be"):
            with self.subTest(encoding=encoding):
                connection = sqlite3.connect(":memory:", check_same_thread=False, isolation_level=None)
                connection.execute("PRAGMA encoding='%s'" % encoding)
                lock = threading.RLock()
                journal = ArchiveJournal(connection, lock)
                try:
                    report = completed(journal)
                    self.assertEqual("unavailable", report["status"])
                    self.assertEqual("unsupported_text_encoding", report["reason"])
                    self.assertNotIn("tables", report)
                    self.assertEqual(encoding, connection.execute("PRAGMA encoding").fetchone()[0])
                finally:
                    journal.close_connection()

    def test_invalid_command_status_is_json_safe_unknown(self):
        connection = sqlite3.connect(":memory:", check_same_thread=False, isolation_level=None)
        self.addCleanup(connection.close)
        connection.executescript("CREATE TABLE lanes(channel TEXT PRIMARY KEY,ready INTEGER);"
                                 "CREATE TABLE commands(status TEXT,body TEXT);")
        for status in (b"pending", None, "unexpected", "pending" * 20, "pending\x00unexpected", "succeeded\x00failed"):
            with self.subTest(status=status):
                connection.execute("DELETE FROM commands")
                connection.execute("INSERT INTO commands VALUES (?,?)", (status, '{}'))
                probe = CapacityProbe(connection, threading.Lock(), lambda: False, "passages")
                report = completed(probe)
                self.assertEqual("unavailable", report["status"])
                self.assertEqual("invalid_sample_metadata", report["reason"])
                json.dumps(report, allow_nan=False)

    def test_standalone_archive_has_no_outbox_requirement(self):
        with tempfile.TemporaryDirectory() as directory:
            store = ArchiveStore(directory)
            try:
                store.save(example_bundle())
                report = completed(store)
                self.assertEqual("complete", report["status"])
                self.assertNotIn("outbox", report["tables"])
            finally:
                store.close()
            self.assertEqual("closed", store.capacity_snapshot()["status"])


class TestCapacityRuntime(unittest.TestCase):
    def test_finalizer_busy_state_is_unknown_not_zero(self):
        owner = ResultFinalizer({"CH1": ("unused", True)})
        entered, release = threading.Event(), threading.Event()
        def hold():
            with owner._lock:
                entered.set()
                release.wait(3)
        thread = threading.Thread(target=hold)
        thread.start()
        try:
            self.assertTrue(entered.wait(1))
            report = owner.capacity_snapshot()
            self.assertEqual("unavailable", report["runtime"]["status"])
            self.assertNotIn("slot_count", report["runtime"])
        finally:
            release.set()
            thread.join(3)
            self.assertTrue(owner.close(0))

    def test_finalizer_does_not_open_store_or_consume_events(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "not-created")
            owner = ResultFinalizer({"CH1": (path, True)})
            owner._event("CH1", {"kind": "durable", "measurement_id": "m"})
            with patch.object(owner, "_store", side_effect=AssertionError("不可懒建库")), \
                    patch.object(owner, "poll", side_effect=AssertionError("不可消费事件")):
                report = owner.capacity_snapshot()
            self.assertEqual("not_opened", report["archive_databases"]["status"])
            self.assertEqual(1, report["runtime"]["event_count"])
            self.assertEqual(1, len(owner._events["CH1"]))
            self.assertFalse(Path(path).exists())
            self.assertTrue(owner.close(0))
            self.assertTrue(owner.capacity_snapshot()["runtime"]["closed"])

    def test_bridge_is_pure_copy_not_quarantine_or_command(self):
        from tests.test_load_radar_bridge import TestLoadRadarBridge
        helper = TestLoadRadarBridge()
        helper.setUp()
        try:
            owner = helper.bridge
            # 过期帧会让旧 snapshot 隔离车道；容量查询不得触发它。
            owner._frames["CH1"] = 0
            before = copy.deepcopy((owner._lanes, owner._loaded, owner._reserved, owner._uncommitted, owner._command_dirty))
            with patch.object(owner, "current_passage", side_effect=AssertionError("不可隔离")):
                report = owner.capacity_snapshot()
            self.assertEqual(before, (owner._lanes, owner._loaded, owner._reserved, owner._uncommitted, owner._command_dirty))
            self.assertEqual("sampled", report["runtime"]["status"])
            self.assertEqual("complete", completed(owner._capacity)["status"])
            completed(owner.outbox)
            owner.close()
            self.assertEqual("closed", owner.capacity_snapshot()["passages"]["status"])
        finally:
            helper.doCleanups()
