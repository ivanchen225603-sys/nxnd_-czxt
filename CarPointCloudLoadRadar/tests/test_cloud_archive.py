"""成套云图原字节、事务、故障恢复及非权威引用；只使用临时目录。"""
import hashlib
import errno
import io
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import threading
import tempfile
import unittest
from datetime import datetime, timezone
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import patch

import numpy as np

from app import cloud_archive as archive
from app.cloud_archive import ArchiveConflict, ArchiveStore, freeze_bundle
from app.load_radar_transport import SQLiteOutbox
from app.pointcloud import Frame, load_xyz, save_xyz
from app import storage


def example_bundle(measurement="m-1", channel="CH1", passage="p-1", ts=1000.,
                   empty=True, carriage=True, complete=True, measurable=True, reason="单测结果"):
    # 重复坐标、不同 ring 必须保留原行号，不能按坐标去重。
    xyz = np.array([[1., 2., 3.], [1., 2., 3.], [2., 3., 4.], [9., 9., 0.]], np.float32)
    frame = Frame(xyz, np.array([10, 11, 12, 13], np.uint16), seq=11, ts=ts, gen=2, packets=450)
    return freeze_bundle(measurement_id=measurement, channel=channel, passage_id=passage,
                         observation=frame, detection=frame._mask(np.array([0, 2])),
                         vehicle_indices=np.array([0, 1, 2]),
                         carriage_indices=np.array([1, 2] if carriage else [], dtype=np.int64),
                         result={"empty": empty, "reason": reason, "channel": channel, "seq": 11, "ts": ts},
                         roi={"carriage_status": "partial" if carriage else "unavailable",
                              "front": {"status": "unsupported"}},
                         configuration_hash="a" * 64, algorithm_version="test-v1",
                         complete=complete, measurable=measurable,
                         transforms={"analysis": {"R": np.eye(3).tolist(), "t": [0., 0., 0.]}})


def payload_for(bundle):
    meta = bundle.metadata
    return {"measurementId": meta["measurement_id"], "channelId": meta["channel"],
            "passageId": meta["passage_id"], "configurationHash": meta["configuration_hash"],
            "algorithmVersion": meta["algorithm_version"], "baselineVersionId": None,
            "measuredAt": datetime.fromtimestamp(meta["acquired_at"], timezone.utc).isoformat().replace("+00:00", "Z"),
            "reason": json.loads(dict(bundle.files)["detection.xyz.meta.json"])["reason"],
            "verdict": "empty", "sourceMode": "live", "frameDegraded": False}


class TestCloudArchive(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="cloud-archive-")
        self.addCleanup(self.temp.cleanup)
        self.output = self.temp.name
        self.store = ArchiveStore(self.output)
        self.addCleanup(self.store.close)
        self.journal = self.store.journal

    def export(self, bundle):
        self.store.save(bundle)
        return Path(self.journal.export(bundle.measurement_id, self.output, channel=bundle.metadata["channel"]))

    def test_readback_preserves_roles_indices_ring_and_metadata(self):
        bundle = example_bundle()
        path = self.export(bundle)
        self.assertEqual(bundle, self.journal.get(bundle.measurement_id))
        manifest = json.loads((path.parent / "manifest.json").read_bytes())
        for name, data in bundle.files:
            self.assertEqual(data, (path.parent / name).read_bytes())
            self.assertEqual(hashlib.sha256(data).hexdigest(), manifest["files"][name]["sha256"])
        observed = load_xyz(str(path.parent / "observation.xyz"))
        carriage = load_xyz(str(path.parent / "carriage.xyz"))
        indices = np.load(io.BytesIO(dict(bundle.files)["carriage.indices.npy"]), allow_pickle=False)
        np.testing.assert_array_equal(carriage.xyz, observed.xyz[indices])
        np.testing.assert_array_equal(carriage.ring, [11, 12])
        self.assertEqual(manifest["frame"], dict(seq=11, ts=1000., gen=2, packets=450, complete=True, measurable=True))
        self.assertEqual("unsupported", manifest["roi"]["front"]["status"])
        self.assertFalse(manifest["authoritative"])
        self.assertEqual([], self.journal.pending("CH1"))

    def test_source_mutation_after_freeze_does_not_change_bytes(self):
        frame = Frame(np.ones((5, 3)), np.arange(5), seq=3, ts=10)
        result = {"empty": True, "reason": "原结果"}
        roi = {"carriage_status": "available"}
        bundle = freeze_bundle(measurement_id="m-1", channel="CH1", passage_id="p-1",
                               observation=frame, detection=frame, vehicle_indices=np.arange(5),
                               carriage_indices=np.arange(5), result=result, roi=roi,
                               configuration_hash="a" * 64, algorithm_version="v1", complete=True, measurable=True)
        before = bundle.manifest, bundle.files
        frame.xyz[:] = 99
        frame.ring[:] = 99
        result["empty"] = False
        roi["carriage_status"] = "unavailable"
        self.assertEqual(before, (bundle.manifest, bundle.files))
        self.assertTrue(bundle.metadata["empty_observation"])
        self.export(bundle)

    def test_idempotent_original_bytes_and_conflicting_id(self):
        first = example_bundle()
        self.store.save(first)
        self.store.save(first)
        with self.assertRaises(ArchiveConflict):
            self.store.save(example_bundle(ts=1001.))
        self.assertEqual(first, self.journal.get(first.measurement_id))
        self.assertEqual(["m-1"], self.journal.pending("CH1"))

    def test_insertion_requires_explicit_transaction(self):
        with self.assertRaises(RuntimeError):
            self.journal.put(example_bundle())

    def test_each_file_export_failure_retries_original_without_partial_commit(self):
        bundle = example_bundle()
        self.store.save(bundle)
        original = archive._write_once
        targets = [name for name, _ in bundle.files] + ["manifest.json"]
        # 每个点使用新的包和路径，故障必须真发生，不靠上次已有文件通过。
        for index, target in enumerate(targets):
            current = example_bundle("m-fail-%d" % index, passage="p-fail-%d" % index, ts=1001. + index)
            self.store.save(current)
            attempts = []

            def fail(path, data):
                attempts.append(path.name)
                if path.name == target:
                    raise OSError(errno.ENOSPC, "注入成员或manifest写入空间不足")
                original(path, data)

            with self.subTest(target=target), patch.object(archive, "_write_once", side_effect=fail):
                with self.assertRaises(OSError):
                    self.journal.export(current.measurement_id, self.output, channel="CH1")
            self.assertIn(target, attempts)
            self.assertIn(current.measurement_id, self.journal.pending("CH1", 100))
            self.assertIsNone(self.journal.reference("CH1", current.metadata["passage_id"]))
            path = self.export(current)
            self.assertEqual(current.manifest, (path.parent / "manifest.json").read_bytes())

    def test_reference_transaction_failure_replays_existing_manifest(self):
        bundle = example_bundle()
        self.store.save(bundle)
        self.store.connection.execute("CREATE TRIGGER fail_ref BEFORE INSERT ON empty_observations BEGIN SELECT RAISE(ABORT,'fault'); END")
        with self.assertRaises(sqlite3.IntegrityError):
            self.journal.export("m-1", self.output, channel="CH1")
        self.assertEqual(["m-1"], self.journal.pending("CH1"))
        self.assertIsNone(self.journal.reference("CH1"))
        self.assertEqual(1, len(list(Path(self.output).rglob("manifest.json"))))
        self.store.connection.execute("DROP TRIGGER fail_ref")
        self.export(bundle)
        self.assertEqual("m-1", self.journal.reference("CH1")["measurement_id"])

    def test_temporary_write_flush_and_fsync_enospc_never_publish_partial_file(self):
        original_open = open
        original_fsync = os.fsync
        for phase in ("write", "flush", "fsync"):
            target = Path(self.output) / ("fault-" + phase + ".bin")
            opened_fds = set()
            class FaultStream:
                def __init__(self, stream):
                    self.stream = stream
                    opened_fds.add(stream.fileno())
                def __enter__(self):
                    return self
                def __exit__(self, *args):
                    self.stream.close()
                def write(self, data):
                    if phase == "write":
                        self.stream.write(data[:1])
                        raise OSError(errno.ENOSPC, "写入空间不足")
                    return self.stream.write(data)
                def flush(self):
                    if phase == "flush":
                        raise OSError(errno.ENOSPC, "flush空间不足")
                    return self.stream.flush()
                def fileno(self):
                    return self.stream.fileno()
            def fail_fsync(fd):
                if phase == "fsync" and fd in opened_fds:
                    raise OSError(errno.ENOSPC, "fsync空间不足")
                return original_fsync(fd)
            with self.subTest(phase=phase), patch.object(archive, "open", create=True,
                    side_effect=lambda *args, **kw: FaultStream(original_open(*args, **kw))), \
                    patch.object(archive.os, "fsync", side_effect=fail_fsync):
                with self.assertRaises(OSError) as raised:
                    archive._write_once(target, b"frozen-original-bytes")
                self.assertEqual(errno.ENOSPC, raised.exception.errno)
            self.assertFalse(target.exists())
            self.assertEqual([], list(Path(self.output).glob(".pending-*")))
            archive._write_once(target, b"frozen-original-bytes")
            self.assertEqual(b"frozen-original-bytes", target.read_bytes())

    def test_export_failure_and_failure_log_failure_keep_durable_bytes(self):
        bundle = example_bundle()
        self.store.save(bundle)
        self.store.connection.execute("CREATE TRIGGER fail_log BEFORE UPDATE ON cloud_archives "
                                      "BEGIN SELECT RAISE(ABORT,'failure log fault'); END")
        with patch.object(archive, "_write_once", side_effect=OSError(errno.ENOSPC, "空间不足")):
            # 2026-09-23（M13）：记账失败不再向外抛记账自己的异常 ——
            # 原始导出错误（空间不足）必须留在消息里并挂 __cause__。
            with self.assertRaises(RuntimeError) as caught:
                self.journal.export(bundle.measurement_id, self.output, channel="CH1")
            self.assertIn("空间不足", str(caught.exception))
            self.assertIn("failure log fault", str(caught.exception))
            self.assertIsInstance(caught.exception.__cause__, OSError)
        self.assertEqual(bundle, self.journal.get(bundle.measurement_id))
        self.assertEqual([bundle.measurement_id], self.journal.pending("CH1"))
        self.assertIsNone(self.journal.reference("CH1"))
        self.assertFalse(list(Path(self.output).rglob("manifest.json")))
        self.store.connection.execute("DROP TRIGGER fail_log")
        path = self.export(bundle)
        self.assertEqual(bundle.manifest, (path.parent / "manifest.json").read_bytes())

    def test_existing_different_file_is_not_overwritten_or_regenerated(self):
        bundle = example_bundle()
        path = self.export(bundle)
        path.write_bytes(b"damaged")
        with self.assertRaises(ArchiveConflict):
            self.journal.export("m-1", self.output, channel="CH1")
        self.assertEqual(b"damaged", path.read_bytes())
        self.assertEqual(bundle, self.journal.get("m-1"))
        self.assertEqual(["m-1"], self.journal.pending("CH1"))
        self.assertIsNone(self.journal.reference("CH1"))

    def test_missing_projection_file_can_be_recreated_from_durable_bytes(self):
        bundle = example_bundle()
        path = self.export(bundle)
        path.unlink()
        self.journal.export("m-1", self.output, channel="CH1")
        self.assertEqual(dict(bundle.files)["detection.xyz"], path.read_bytes())

    def test_db_blob_and_manifest_corruption_are_detected(self):
        bundle = example_bundle()
        self.store.save(bundle)
        self.store.connection.execute("UPDATE cloud_archive_files SET content=? WHERE name='vehicle.xyz'", (b"bad",))
        with self.assertRaises(ArchiveConflict):
            self.journal.get("m-1")
        self.store.connection.execute("UPDATE cloud_archives SET manifest=?", (b"{}",))
        with self.assertRaises(ArchiveConflict):
            self.journal.get("m-1")

    def test_four_channels_same_second_and_plate_are_independent(self):
        # 各通道测量 ID 也必须全局唯一；相同 ID 不允许借通道分目录绕过。
        bundles = [example_bundle("m-%d" % i, "CH%d" % i) for i in range(1, 5)]
        with ThreadPoolExecutor(max_workers=4) as pool:
            paths = list(pool.map(self.export, bundles))
        self.assertEqual(4, len(set(paths)))
        for path, bundle in zip(paths, bundles):
            self.assertEqual(bundle.metadata["channel"], json.loads((path.parent / "manifest.json").read_bytes())["channel"])

    def test_separate_sqlite_connections_replay_same_bundle_concurrently(self):
        bundle = example_bundle()

        def replay(_):
            store = ArchiveStore(self.output)
            try:
                store.save(bundle)
                return store.journal.export("m-1", self.output, channel="CH1")
            finally:
                store.close()

        with ThreadPoolExecutor(max_workers=4) as pool:
            paths = list(pool.map(replay, range(4)))
        self.assertEqual(1, len(set(paths)))
        self.assertEqual(bundle.manifest, (Path(paths[0]).parent / "manifest.json").read_bytes())
        self.assertEqual(1, self.store.connection.execute("SELECT COUNT(*) FROM cloud_archives").fetchone()[0])

    def test_slow_export_does_not_hold_database_lock_or_block_another_channel(self):
        self.store.save(example_bundle())
        started, release = threading.Event(), threading.Event()
        original = archive._write_once

        def slow(path, data):
            started.set()
            if not release.wait(5):
                raise TimeoutError("测试导出释放超时")
            return original(path, data)

        with ThreadPoolExecutor(max_workers=2) as pool, patch.object(archive, "_write_once", side_effect=slow):
            export = pool.submit(self.journal.export, "m-1", self.output, channel="CH1")
            try:
                self.assertTrue(started.wait(3))
                pending = pool.submit(self.journal.pending, "CH1")
                self.assertEqual(["m-1"], pending.result(timeout=1))
                commit = pool.submit(self.store.save, example_bundle("m-2", "CH2"))
                self.assertEqual("m-2", commit.result(timeout=1))
            finally:
                release.set()
            self.assertTrue(Path(export.result(timeout=3)).exists())

    def test_close_waits_for_active_export_in_standalone_and_shared_outbox(self):
        for shared in (False, True):
            with self.subTest(shared=shared), tempfile.TemporaryDirectory() as output:
                owner = SQLiteOutbox(os.path.join(output, "outbox.sqlite3")) if shared else ArchiveStore(output)
                journal = owner.archives if shared else owner.journal
                bundle = example_bundle()
                if shared:
                    owner.save(payload_for(bundle), archive=bundle)
                else:
                    owner.save(bundle)
                entered, release, closing = threading.Event(), threading.Event(), threading.Event()
                original = archive._write_once

                def slow(path, data):
                    entered.set()
                    if not release.wait(5):
                        raise TimeoutError("测试释放超时")
                    return original(path, data)

                def close():
                    closing.set()
                    owner.close()

                with ThreadPoolExecutor(max_workers=2) as pool, patch.object(archive, "_write_once", side_effect=slow):
                    exporting = pool.submit(journal.export, "m-1", output, channel="CH1")
                    try:
                        self.assertTrue(entered.wait(3))
                        stopping = pool.submit(close)
                        self.assertTrue(closing.wait(1))
                        with self.assertRaises(TimeoutError):
                            stopping.result(timeout=.1)
                    finally:
                        release.set()
                    self.assertTrue(Path(exporting.result(timeout=3)).exists())
                    stopping.result(timeout=3)
                owner.close()
                with self.assertRaisesRegex(RuntimeError, "已关闭"):
                    journal.export("m-1", output, channel="CH1")

    def test_reference_never_activates_formal_baseline(self):
        baseline = Path(self.output) / "CH1.baseline.npz"
        baseline.write_bytes(b"formal-baseline-not-an-observed-reference")
        self.export(example_bundle())
        ref = self.journal.reference("CH1")
        self.assertFalse(ref["authoritative"])
        self.assertTrue(ref["carriage_available"])
        self.assertEqual("partial", ref["roi"]["carriage_status"])
        self.assertEqual(hashlib.sha256(example_bundle().manifest).hexdigest(), ref["manifest_sha256"])
        self.assertEqual(b"formal-baseline-not-an-observed-reference", baseline.read_bytes())

    def test_loaded_unknown_incomplete_unmeasurable_and_unavailable_do_not_update_reference(self):
        for index, options in enumerate((dict(empty=False), dict(empty=None), dict(carriage=False),
                                         dict(complete=False), dict(measurable=False), dict(passage=None))):
            with self.subTest(options=options):
                self.export(example_bundle("m-%d" % index, **options))
                self.assertIsNone(self.journal.reference("CH1"))

    def test_old_delayed_export_and_late_commit_cannot_replace_latest(self):
        old = example_bundle("m-old", passage="p-old", ts=1000.)
        newer = example_bundle("m-new", passage="p-new", ts=1100.)
        self.store.save(old)
        self.export(newer)
        self.export(old)
        self.export(example_bundle("m-older-late", passage="p-older", ts=900.))
        self.assertEqual("m-new", self.journal.reference("CH1")["measurement_id"])
        self.assertEqual("m-old", self.journal.reference("CH1", "p-old")["measurement_id"])

    def test_corrupt_latest_keeps_high_watermark_instead_of_falling_back_to_old_vehicle(self):
        old = example_bundle("m-old", passage="p-old", ts=1000.)
        newer = example_bundle("m-new", passage="p-new", ts=1100.)
        self.export(old)
        path = self.export(newer)
        path.write_bytes(b"corrupt")
        with self.assertRaises(ArchiveConflict):
            self.journal.export("m-new", self.output, channel="CH1")
        self.assertIsNone(self.journal.reference("CH1"))
        self.export(old)
        self.assertIsNone(self.journal.reference("CH1"))
        # 人工移走损坏投影后，仍用同一包恢复；无需新测量或更换 ID。
        path.unlink()
        self.export(newer)
        self.assertEqual("m-new", self.journal.reference("CH1")["measurement_id"])

    def test_wall_clock_reversal_or_equal_time_is_not_guessed_as_newer(self):
        self.export(example_bundle("m-1", ts=1000.))
        self.export(example_bundle("m-2", passage="p-2", ts=1000.))
        self.export(example_bundle("m-3", passage="p-3", ts=900.))
        self.assertEqual("m-1", self.journal.reference("CH1")["measurement_id"])

    def test_same_passage_delayed_old_export_requires_both_time_and_durable_order(self):
        old = example_bundle("m-old", ts=2000.)
        newer = example_bundle("m-new", ts=1000.)
        self.store.save(old)
        self.export(newer)
        self.export(old)
        self.assertEqual("m-new", self.journal.reference("CH1", "p-1")["measurement_id"])

    def test_corrupt_reference_and_journal_indices_fail_closed(self):
        self.export(example_bundle("m-1", "CH1"))
        self.export(example_bundle("m-2", "CH2"))
        self.store.connection.execute("UPDATE empty_observation_latest SET measurement_id='m-2' WHERE channel='CH1'")
        with self.assertRaises(ArchiveConflict):
            self.journal.reference("CH1")
        self.store.connection.execute("UPDATE cloud_archives SET channel='CH3' WHERE measurement_id='m-2'")
        with self.assertRaises(ArchiveConflict):
            self.journal.get("m-2")

    def test_interface_identifiers_are_encoded_not_used_as_paths(self):
        paths = [self.export(example_bundle(measurement=value, channel="CON", passage="passage:1"))
                 for value in ("Node:abc", "node:abc", "NUL")]
        self.assertEqual(3, len(set(paths)))
        for path in paths:
            self.assertNotIn(":", str(path.relative_to(self.output)))
            self.assertLess(len(str(path)), 260)
        with self.assertRaises(ValueError):
            example_bundle(measurement="../../outside")

    def test_legacy_cleanup_cannot_split_bundle_even_when_called_directly(self):
        path = self.export(example_bundle())
        old = Path(self.output) / "legacy" / "old.xyz"
        save_xyz(str(old), Frame(np.zeros((1, 3))))
        os.utime(str(old), (1, 1))
        self.assertEqual([str(old)], [row[0] for row in storage.scan(self.output)])
        self.assertEqual([], storage.scan(str(path.parent)))
        self.assertEqual((0, 0), storage.remove_sample(str(path)))
        self.assertEqual(1, storage.purge(self.output, 1)["removed"])
        self.assertTrue(path.exists())
        self.assertTrue((path.parent / "manifest.json").exists())

    def test_wrong_channel_and_baseline_output_are_rejected(self):
        self.store.save(example_bundle())
        with self.assertRaises(ValueError):
            self.journal.export("m-1", self.output, channel="CH2")
        with self.assertRaises(ValueError):
            self.journal.export("m-1", storage.baseline_dir(), channel="CH1")
        from app.settings import root
        with self.assertRaises(ValueError):
            ArchiveStore(root("re", "forbidden-archive-test"))

    def test_invalid_indices_do_not_silently_drop_or_reorder_rings(self):
        frame = Frame(np.ones((3, 3)), ts=1)
        for indices in (np.array([-1]), np.array([3]), np.array([1, 0]), np.array([0, 0]), np.array([1.])):
            with self.subTest(indices=indices), self.assertRaises(ValueError):
                freeze_bundle(measurement_id="m", channel="CH1", passage_id="p", observation=frame,
                              detection=frame, vehicle_indices=np.arange(3), carriage_indices=indices,
                              result={}, roi={}, configuration_hash="a" * 64, algorithm_version="v1",
                              complete=True, measurable=True)

    def test_mismatched_frame_identity_or_non_boolean_quality_is_rejected(self):
        frame = Frame(np.ones((3, 3)), seq=2, ts=10, gen=4)
        for detection, complete in ((Frame(frame.xyz, seq=3, ts=10, gen=4), True),
                                    (Frame(frame.xyz, seq=2, ts=11, gen=4), True),
                                    (Frame(frame.xyz, seq=2, ts=10, gen=5), True), (frame, "false")):
            with self.subTest(seq=detection.seq, ts=detection.ts, gen=detection.gen, complete=complete), self.assertRaises(ValueError):
                freeze_bundle(measurement_id="m", channel="CH1", passage_id="p", observation=frame,
                              detection=detection, vehicle_indices=np.arange(3), carriage_indices=np.arange(3),
                              result={}, roi={}, configuration_hash="a" * 64, algorithm_version="v1",
                              complete=complete, measurable=True)

    def test_restart_in_another_process_needs_no_hardware_or_recomputation(self):
        bundle = example_bundle()
        self.store.save(bundle)
        # 子进程只有存储代码，完整导出原包；父进程连接仍开着以覆盖 WAL 可见性。
        script = ("import sys; from app.cloud_archive import ArchiveStore; "
                  "s=ArchiveStore(sys.argv[1]); "
                  "print(s.journal.export('m-1',sys.argv[1],channel='CH1')); s.close()")
        result = subprocess.run([sys.executable, "-c", script, self.output], capture_output=True,
                                text=True, encoding="utf-8", timeout=20,
                                env={**os.environ, "PYTHONIOENCODING": "utf-8"})
        self.assertEqual(0, result.returncode, result.stderr)
        path = Path(result.stdout.strip())
        self.assertEqual(bundle.manifest, (path.parent / "manifest.json").read_bytes())
        self.assertEqual("m-1", self.journal.reference("CH1")["measurement_id"])

    def test_hard_process_exit_at_commit_file_and_manifest_boundaries_recovers(self):
        script = """
import os, sys
from app import cloud_archive as archive
from tests.test_cloud_archive import example_bundle
store = archive.ArchiveStore(sys.argv[1])
bundle = example_bundle('m-crash-' + sys.argv[2], passage='p-crash-' + sys.argv[2])
store.save(bundle)
if sys.argv[2] == 'commit':
    os._exit(73)
original = archive._write_once
def write(path, data):
    original(path, data)
    if path.name == sys.argv[2]:
        os._exit(73)
archive._write_once = write
store.journal.export(bundle.measurement_id, sys.argv[1], channel='CH1')
"""
        for stage in ("commit", "carriage.xyz", "manifest.json"):
            with self.subTest(stage=stage):
                result = subprocess.run([sys.executable, "-c", script, self.output, stage],
                                        capture_output=True, text=True, encoding="utf-8", timeout=20,
                                        env={**os.environ, "PYTHONIOENCODING": "utf-8"})
                self.assertEqual(73, result.returncode, result.stderr)
                measurement = "m-crash-" + stage
                self.assertIn(measurement, self.journal.pending("CH1"))
                bundle = example_bundle(measurement, passage="p-crash-" + stage)
                self.assertEqual(bundle, self.journal.get(measurement))
                path = Path(self.journal.export(measurement, self.output, channel="CH1"))
                self.assertEqual(bundle.manifest, (path.parent / "manifest.json").read_bytes())
                for name, data in bundle.files:
                    self.assertEqual(data, (path.parent / name).read_bytes())
                self.assertNotIn(measurement, self.journal.pending("CH1"))


class TestArchiveOutboxTransaction(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="archive-outbox-")
        self.addCleanup(self.temp.cleanup)
        self.outbox = SQLiteOutbox(self.temp.name)
        self.addCleanup(self.outbox.close)

    def test_archive_payload_asset_receipt_commit_and_replay_together(self):
        bundle = example_bundle(channel="CH4")
        payload = payload_for(bundle)
        asset = b"test-asset-bytes"
        payload["baselineCandidateSha256"] = hashlib.sha256(asset).hexdigest()
        command = dict(requestId="request-1", channelId="CH4", passageId="p-1", kind="baseline-capture", confirmedEmpty=True)
        self.outbox.save(payload, asset, command, archive=bundle)
        self.outbox.save(payload, asset, command, archive=bundle)
        self.assertEqual(bundle, self.outbox.archives.get("m-1"))
        self.assertEqual("m-1", self.outbox.command_result("request-1")["measurement"].measurement_id)
        self.assertEqual(asset, self.outbox.get("m-1").asset)

    def test_failure_in_each_table_rolls_back_entire_joint_transaction(self):
        for table in ("outbox", "command_results", "cloud_archives", "cloud_archive_files"):
            with self.subTest(table=table):
                bundle = example_bundle()
                payload = payload_for(bundle)
                command = dict(requestId="request-1", channelId="CH1", passageId="p-1", kind="remeasure", confirmedEmpty=False)
                self.outbox._connection.execute("CREATE TRIGGER fail_insert BEFORE INSERT ON " + table +
                                                " BEGIN SELECT RAISE(ABORT,'fault'); END")
                with self.assertRaises(sqlite3.IntegrityError):
                    self.outbox.save(payload, command=command, archive=bundle)
                for name in ("outbox", "command_results", "cloud_archives", "cloud_archive_files"):
                    self.assertEqual(0, self.outbox._connection.execute("SELECT COUNT(*) FROM " + name).fetchone()[0])
                self.outbox._connection.execute("DROP TRIGGER fail_insert")
        self.outbox.save(payload, command=command, archive=bundle)
        self.assertEqual(bundle, self.outbox.archives.get("m-1"))

    def test_real_sqlite_full_rolls_back_joint_transaction_and_retries_original(self):
        bundle = example_bundle(channel="CH4")
        payload = payload_for(bundle)
        asset = b"capacity-full-asset" * 32768
        payload["baselineCandidateSha256"] = hashlib.sha256(asset).hexdigest()
        command = dict(requestId="full-request", channelId="CH4", passageId="p-1",
                       kind="baseline-capture", confirmedEmpty=True)
        connection = self.outbox._connection
        old_limit = connection.execute("PRAGMA max_page_count").fetchone()[0]
        pages = connection.execute("PRAGMA page_count").fetchone()[0]
        connection.execute("PRAGMA max_page_count=%d" % pages)
        try:
            with self.assertRaises(sqlite3.OperationalError) as raised:
                self.outbox.save(payload, asset, command, archive=bundle)
            # Python 3.8 无 sqlite_errorcode 属性；旧版本检查 SQLite 官方错误文本。
            self.assertEqual(13, getattr(raised.exception, "sqlite_errorcode", 13))
            self.assertIn("full", str(raised.exception).lower())
            self.assertFalse(connection.in_transaction)
            for table in ("outbox", "command_results", "cloud_archives", "cloud_archive_files"):
                self.assertEqual(0, connection.execute("SELECT COUNT(*) FROM " + table).fetchone()[0])
        finally:
            connection.execute("PRAGMA max_page_count=%d" % old_limit)
        self.outbox.save(payload, asset, command, archive=bundle)
        self.outbox.save(payload, asset, command, archive=bundle)
        self.assertEqual(asset, self.outbox.get(bundle.measurement_id).asset)
        self.assertEqual(payload, self.outbox.get(bundle.measurement_id).payload)
        self.assertEqual(bundle, self.outbox.archives.get(bundle.measurement_id))
        self.assertEqual(bundle.measurement_id, self.outbox.command_result("full-request")["measurement"].measurement_id)
        self.assertEqual(1, connection.execute("SELECT COUNT(*) FROM outbox").fetchone()[0])

    def test_existing_measurement_cannot_be_given_new_clouds_retroactively(self):
        bundle = example_bundle()
        payload = payload_for(bundle)
        self.outbox.save(payload)
        with self.assertRaises(ValueError):
            self.outbox.save(payload, archive=bundle)
        self.assertIsNone(self.outbox.archives.get("m-1"))

    def test_archive_identity_and_version_must_match_outbox(self):
        bundle = example_bundle()
        for key in ("measurementId", "channelId", "passageId", "configurationHash", "algorithmVersion", "baselineVersionId", "sourceMode"):
            payload = payload_for(bundle)
            payload[key] = "different"
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.outbox.save(payload, archive=bundle)
        self.assertEqual([], self.outbox.pending())

    def test_source_verdict_and_completeness_cannot_disagree(self):
        bundle = example_bundle()
        for change in ({"sourceMode": "demo"}, {"verdict": "loaded"}, {"verdict": "unknown"},
                       {"frameDegraded": True}, {"frameDegraded": 0},
                       {"measuredAt": "2000-01-01T00:00:00Z"}, {"reason": "另一次结果"}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.outbox.save({**payload_for(bundle), **change}, archive=bundle)
        self.assertEqual([], self.outbox.pending())


class TestArchiveBridge(unittest.TestCase):
    def setUp(self):
        from tests.test_load_radar_bridge import TestLoadRadarBridge
        TestLoadRadarBridge.setUp(self)

    def enter(self):
        from tests.test_load_radar_bridge import TestLoadRadarBridge
        return TestLoadRadarBridge.enter(self)

    def record(self, factory, command=None):
        from tests.test_load_radar_bridge import TestLoadRadarBridge
        return self.bridge.record(TestLoadRadarBridge.result(self), configuration_hash="a" * 64,
                                  source_mode="live", archive_factory=factory, command=command)

    @staticmethod
    def factory(payload):
        return example_bundle(payload["measurementId"], payload["channelId"], payload["passageId"],
                              datetime.fromisoformat(payload["measuredAt"].replace("Z", "+00:00")).timestamp(),
                              reason=payload["reason"])

    def test_db_failure_retains_one_original_archive_and_never_invokes_factory_again(self):
        self.enter()
        calls = []

        def factory(payload):
            calls.append(payload)
            return self.factory(payload)

        with patch.object(self.bridge.outbox, "save", side_effect=OSError("磁盘故障")):
            with self.assertRaises(OSError):
                self.record(factory)
        pending = self.bridge._uncommitted["CH1"]
        original = pending[3]
        measurement = pending[0]["measurementId"]
        self.bridge.flush_pending("CH1")
        self.assertEqual(1, len(calls))
        self.assertEqual(original, self.bridge.outbox.archives.get(measurement))
        self.assertEqual(measurement, self.bridge.outbox.pending()[0].measurement_id)

    def test_missing_or_failed_archive_preparation_is_never_silently_legacy_or_recaptured(self):
        from app.load_radar_bridge import BridgeConflict
        self.enter()
        calls = []

        def factory(payload):
            calls.append(payload["measurementId"])
            return None

        with self.assertRaises(ValueError):
            self.record(factory)
        self.assertEqual([], self.bridge.outbox.pending())
        original = self.bridge._uncommitted["CH1"]
        self.assertEqual(calls[0], original[0]["measurementId"])
        with self.assertRaises(BridgeConflict):
            self.bridge.flush_pending("CH1")
        with self.assertRaises(BridgeConflict):
            self.record(factory)
        self.assertEqual(1, len(calls))
        self.assertIs(original, self.bridge._uncommitted["CH1"])

    def test_archive_builder_exception_retains_identity_and_blocks_fresh_measurement(self):
        from app.load_radar_bridge import BridgeConflict
        self.enter()
        with patch.object(self, "factory", side_effect=OSError("冻结失败")) as factory:
            with self.assertRaises(OSError):
                self.record(factory)
            original_id = self.bridge._uncommitted["CH1"][0]["measurementId"]
            with self.assertRaises(BridgeConflict):
                self.record(factory)
            self.assertEqual(1, factory.call_count)
        self.assertEqual(original_id, self.bridge._uncommitted["CH1"][0]["measurementId"])
        self.assertEqual([], self.bridge.outbox.pending())

    def test_late_a_commit_never_projects_into_b_passage(self):
        first = self.enter()
        with patch.object(self.bridge.outbox, "save", side_effect=OSError("磁盘故障")):
            with self.assertRaises(OSError):
                self.record(self.factory)
        pending = self.bridge._uncommitted["CH1"]
        second = self.enter()
        self.assertNotEqual(first, second)
        self.bridge.flush_pending("CH1")
        self.assertEqual(second, self.bridge.current_passage("CH1"))
        self.assertIsNone(self.bridge._lane("CH1")["measurement"])
        self.assertEqual(first, self.bridge.outbox.archives.get(pending[0]["measurementId"]).metadata["passage_id"])

    def test_command_replay_returns_original_without_new_archive(self):
        passage = self.enter()
        body = dict(requestId="req-1", channelId="CH1", passageId=passage, kind="remeasure", confirmedEmpty=False)
        self.bridge.queue_command(body)
        command = self.bridge.claim_command("CH1")
        measurement = self.record(self.factory, command)
        with patch.object(self, "factory", side_effect=AssertionError("不得重新归档")) as factory:
            self.assertEqual(measurement, self.record(factory, command))
        self.assertEqual(0, factory.call_count)
        self.assertEqual(1, len(self.bridge.outbox.pending()))

    def test_quarantined_passage_does_not_invoke_bound_archive_factory(self):
        with patch.object(self, "factory", side_effect=AssertionError("不得猜车次")) as factory:
            self.assertIsNone(self.record(factory))
        self.assertEqual(0, factory.call_count)
        self.assertEqual([], self.bridge.outbox.pending())


if __name__ == "__main__":
    unittest.main()

class TestArchiveRetention(unittest.TestCase):
    """★ 成套归档的保留策略（2026-09-18）。

    现场要求：**最新的空车车厢永久保存，载重云图只留最近 N 天。**

    在此之前归档库**没有任何删除路径**：`cli/cleanup.py` 走
    `storage.scan` / `remove_sample`，而这两个函数看到 `archives-v1`
    一律跳过（`storage.is_archive`）—— 那是对的，成套包不能被逐个 XYZ
    地拆开删，但结果就是归档只涨不删，磁盘迟早满。
    """

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="archive-retention-")
        self.addCleanup(self.temp.cleanup)
        self.store = ArchiveStore(self.temp.name)
        self.addCleanup(self.store.close)

    def save(self, measurement, ts, *, empty, complete=True, export=True):
        bundle = example_bundle(measurement=measurement, passage="p-" + measurement,
                                ts=ts, empty=empty)
        self.store.save(bundle)
        if export and complete:
            self.store.journal.export(measurement, self.temp.name, channel="CH1")
        return bundle

    def ids(self):
        return {row[0] for row in self.store.connection.execute(
            "SELECT measurement_id FROM cloud_archives").fetchall()}

    def test_old_loaded_bundles_are_removable_but_dry_run_keeps_them(self):
        """默认只报不删 —— 与 cleanup.py 同一套规矩。"""
        self.save("m-old", 1000., empty=False)
        report = self.store.prune(2000.)
        self.assertEqual(1, len(report["removable"]))
        self.assertFalse(report["applied"])
        self.assertEqual(0, report["removed"])
        self.assertEqual({"m-old"}, self.ids(), "预演竟然真删了")
        self.assertGreater(report["removed_bytes"], 0)

    def test_apply_removes_old_loaded_bundle_and_its_blobs(self):
        self.save("m-old", 1000., empty=False)
        report = self.store.prune(2000., apply=True)
        self.assertEqual(1, report["removed"])
        self.assertEqual(set(), self.ids())
        left = self.store.connection.execute(
            "SELECT COUNT(*) FROM cloud_archive_files WHERE measurement_id=?", ("m-old",)).fetchone()[0]
        self.assertEqual(0, left, "包删了但点云 blob 还留着，等于没释放空间")

    def test_empty_observation_is_never_removed(self):
        """★ 这条就是"最新空车永久保存"的落点。

        空车点云是基线的原始材料，删了补不回来 —— 那辆车得重新开进来标定。
        """
        self.save("m-empty", 1000., empty=True)
        report = self.store.prune(9e9, apply=True)
        self.assertEqual(0, report["removed"])
        self.assertEqual(1, report["kept_empty"])
        self.assertEqual({"m-empty"}, self.ids(), "空车归档被按天数删掉了")

    def test_pending_bundle_is_never_removed(self):
        """还没导出/上报就删 = 直接丢数据。"""
        self.save("m-pending", 1000., empty=False, export=False)
        report = self.store.prune(9e9, apply=True)
        self.assertEqual(0, report["removed"])
        self.assertEqual(1, report["kept_pending"])
        self.assertEqual({"m-pending"}, self.ids())

    def test_recent_loaded_bundle_survives(self):
        self.save("m-new", 5000., empty=False)
        report = self.store.prune(2000., apply=True)
        self.assertEqual(0, report["removed"])
        self.assertEqual(1, report["kept_recent"])
        self.assertEqual({"m-new"}, self.ids())

    def test_mixed_store_keeps_empty_and_recent_drops_only_old_loaded(self):
        self.save("m-empty", 1000., empty=True)
        self.save("m-old", 1100., empty=False)
        self.save("m-new", 5000., empty=False)
        self.save("m-pending", 1200., empty=False, export=False)
        report = self.store.prune(2000., apply=True)
        self.assertEqual(1, report["removed"])
        self.assertEqual({"m-empty", "m-new", "m-pending"}, self.ids())
        self.assertEqual(4, report["scanned"])

    def test_latest_empty_reference_still_resolves_after_pruning(self):
        """清理之后"本通道最新空车"这条引用不能悬空。"""
        self.save("m-empty", 1000., empty=True)
        self.save("m-old", 1100., empty=False)
        self.store.prune(2000., apply=True)
        reference = self.store.journal.reference("CH1")
        self.assertIsNotNone(reference, "最新空车引用在清理后失效了")
        self.assertEqual("m-empty", reference["measurement_id"])

    def test_unreadable_manifest_is_kept_not_dropped(self):
        """读不懂的包按"可能是空车"处理 —— 宁留勿删。"""
        self.save("m-old", 1000., empty=False)
        with self.store.lock:
            self.store.connection.execute("UPDATE cloud_archives SET manifest=? WHERE measurement_id=?",
                                          (b"not json", "m-old"))
        report = self.store.prune(9e9, apply=True)
        self.assertEqual(0, report["removed"])
        self.assertEqual({"m-old"}, self.ids())

    def test_invalid_cutoff_is_rejected(self):
        for bad in (float("nan"), float("inf"), "1000"):
            with self.assertRaises((ValueError, TypeError)):
                self.store.prune(bad)

    def test_apply_also_removes_the_exported_projection(self):
        """★ prune 必须连磁盘投影一起删（2026-09-23，两轮审查 H4）。

        原来只删三张表的行：`export()` 投影出的成套文件目录没有任何删除
        路径 —— DB 删了文件还在，孤儿投影仍会被 `discover_samples` 加载，
        磁盘只涨不降，"载重留 N 天"对投影副本从未生效。
        （目录名是哈希编码，只能靠 export 的返回值定位。）
        """
        from pathlib import Path
        self.save("m-old", 1000., empty=False, export=False)
        exported = Path(self.store.journal.export("m-old", self.temp.name,
                                                  channel="CH1")).parent
        self.assertTrue(exported.is_dir(), "前提：投影目录在")
        report = self.store.prune(2000., apply=True)
        self.assertEqual(1, report["removed"])
        self.assertEqual(1, report["removed_dirs"])
        self.assertFalse(exported.exists(), "投影目录必须一起删掉")

    def test_projection_delete_failure_keeps_the_row_for_a_retry(self):
        """文件删不动就跳过该行：保持「目录在 = 行在」，重跑清理即可续删。"""
        from pathlib import Path
        import app.cloud_archive as ca
        self.save("m-old", 1000., empty=False, export=False)
        exported = Path(self.store.journal.export("m-old", self.temp.name,
                                                  channel="CH1")).parent
        with patch.object(ca.shutil, "rmtree", side_effect=PermissionError("占用中")):
            report = self.store.prune(2000., apply=True)
        self.assertEqual(0, report["removed"])
        self.assertEqual({"m-old"}, self.ids(), "投影没删成，行不能删 —— 否则又成孤儿")
        self.assertTrue(exported.exists())
        self.assertEqual(1, len(report["dir_failures"]))
        self.assertIn("m-old", report["dir_failures"][0]["measurement_id"])

    def test_export_bookkeeping_crash_does_not_mask_the_original_error(self):
        """★ 记账失败不得吞掉/替换原始导出错误（2026-09-23，两轮审查 M13）。

        记账（UPDATE 置回 pending）自己炸掉时，原来会把记账异常向外抛，
        磁盘满这类真正的根因从此没人看见。现在：原始错误进消息并挂
        __cause__，记账失败原因同时可见，包状态保持 pending 安全。
        （sqlite3.Connection.execute 是 C 属性不可 patch，用既有测试同款
        CREATE TRIGGER 造记账失败。）
        """
        import app.cloud_archive as ca
        self.save("m-x", 1000., empty=False, export=False)
        self.store.connection.execute("CREATE TRIGGER fail_log BEFORE UPDATE ON cloud_archives "
                                      "BEGIN SELECT RAISE(ABORT,'记账失败'); END")
        try:
            with patch.object(ca, "_write_once", side_effect=OSError("磁盘满")):
                with self.assertRaises(RuntimeError) as caught:
                    self.store.journal.export("m-x", self.temp.name, channel="CH1")
        finally:
            self.store.connection.execute("DROP TRIGGER fail_log")
        message = str(caught.exception)
        self.assertIn("磁盘满", message, "原始导出错误被记账失败替换掉了")
        self.assertIn("记账失败", message, "记账失败的原因也要可见")
        self.assertIsInstance(caught.exception.__cause__, OSError,
                              "异常链必须保留原始导出错误")
        # 包状态仍安全（记账没提交，行未被 UPDATE 波及）
        status = self.store.connection.execute(
            "SELECT status FROM cloud_archives WHERE measurement_id=?", ("m-x",)).fetchone()[0]
        self.assertEqual("pending", status)

