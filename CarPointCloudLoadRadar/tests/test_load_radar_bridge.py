"""接口状态机、命令防串车和本地持久补传测试，不连接真实后台。"""
import hashlib
import io
import json
import tempfile
import threading
import time
import unittest
from datetime import datetime, timezone, timedelta
from types import SimpleNamespace
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, build_opener, ProxyHandler

import numpy as np

from app.load_radar_bridge import BridgeConflict, LoadRadarBridge
from app.load_radar_transport import BridgeConfig, HttpError


class MemoryClient:
    def __init__(self):
        self.rows = {}
        self.posts = []
        self.versions = []
        self.assets = {}
        self.failure = None

    def post_measurement(self, payload):
        self.posts.append(payload.copy())
        # 可模拟后台已持久化但 ACK 丢失。
        self.rows.setdefault(payload["measurementId"], {"measurement": payload.copy(), "binding": None})
        if self.failure:
            raise self.failure
        return self.rows[payload["measurementId"]]

    def get_measurement(self, measurement):
        return self.rows[measurement]

    def report_standstill(self, payload):
        return {"status": "not_required", "passageId": payload["passageId"]}

    def report_departure(self, payload):
        return {"status": "not_required", "passageId": payload["passageId"]}

    def list_baselines(self):
        return self.versions

    def upload_baseline(self, measurement, sha, content):
        self.assets[sha] = content
        return {"sha256": sha}

    def download_baseline(self, version):
        return self.assets[version]


class TestLoadRadarBridge(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.config = BridgeConfig(node_id="jt-test", base_url="http://127.0.0.1:1",
                                   agent_key="test-key-" * 5, storage_dir=self.tmp.name,
                                   algorithm_version="test-v1", channels={"CH1": {"calibration_id": "cal-1"},
                                       "CH4": {"calibration_id": "cal-4"}},
                                   enabled=True, timeout_sec=1, retry_sec=1)
        self.client = MemoryClient()
        self.bridge = LoadRadarBridge(self.config, self.client)
        self.addCleanup(lambda: self.bridge.close())

    def enter(self, channel="CH1"):
        for _ in range(3):
            self.bridge.observe(channel, False, True)
            self.bridge._idle[channel] = (3, time.monotonic() - 1)
        self.bridge.observe(channel, True, True)
        return self.bridge.current_passage(channel)

    def result(self):
        return {"channel": "CH1", "ts": time.time(), "empty": True, "reason": "空车",
                "frame_degraded": False, "n_points": 100, "cloud": [], "volume": {"ok": False}}

    def record(self, **kwargs):
        result = self.result()
        if kwargs.get("command"):
            result["channel"] = kwargs["command"]["channelId"]
        return self.bridge.record(result, configuration_hash="a" * 64, source_mode="live", **kwargs)

    def body(self, passage=None, request="request-1", kind="remeasure"):
        channel = "CH4" if kind == "baseline-capture" else "CH1"
        return {"requestId": request, "channelId": channel, "passageId": passage or self.enter(channel),
                "kind": kind, "confirmedEmpty": kind == "baseline-capture"}

    def test_prepared_plate_only_belongs_to_current_live_passage(self):
        passage = self.enter()
        binding = {"licensePlate": "宁A25771"}
        self.bridge._channel_contexts["CH1"] = ("a" * 64, "live")
        self.bridge._prepared["CH1"] = (passage, "baseline_required", binding, time.monotonic())
        self.assertEqual(self.bridge.prepared_plate("CH1"), "宁A25771")
        self.bridge._prepared["CH1"] = (passage, "awaiting_plate", binding, time.monotonic())
        self.assertEqual(self.bridge.prepared_plate("CH1"), "")
        self.bridge._prepared["CH1"] = (passage, "ready", binding, time.monotonic() - 11)
        self.assertEqual(self.bridge.prepared_plate("CH1"), "")
        self.bridge._prepared["CH1"] = ("other-passage", "ready", binding, time.monotonic())
        self.assertEqual(self.bridge.prepared_plate("CH1"), "")

    def test_startup_occupied_is_quarantined(self):
        self.bridge.observe("CH1", True, True)
        self.assertIsNone(self.bridge.current_passage("CH1"))
        self.assertIsNone(self.record())
        self.assertEqual(self.bridge.outbox.pending(), [])

    def test_passages_write_lock_does_not_block_capture_path(self):
        import sqlite3
        from pathlib import Path
        from app.load_radar_contract import build_measurement
        passage = self.enter()
        body = self.body(passage=passage)
        self.bridge.queue_command(body)
        other = sqlite3.connect(str(Path(self.tmp.name) / "passages.sqlite3"))
        other.execute("BEGIN IMMEDIATE")
        finished, errors = threading.Event(), []
        def observe_and_claim():
            try:
                self.bridge.observe("CH1", True, True)
                self.assertEqual(self.bridge.current_passage("CH1"), passage)
                self.bridge.baseline("CH1", time.time())
                command = self.bridge.claim_command_fast("CH1")
                reservation = self.bridge.reserve_result("CH1", "origin", command)
                result = self.result()
                result.update(empty=False, step=10)
                payload = build_measurement(result, measurement_id=reservation.measurement_id,
                    node_id=self.config.node_id, passage_id=passage, calibration_id="cal-1",
                    algorithm_version="test-v1", configuration_hash="a" * 64, source_mode="live")
                self.bridge.commit_reserved(reservation, payload)
                self.assertIsNone(self.bridge.claim_command_fast("CH1"))
            except Exception as error:
                errors.append(error)
            finally:
                finished.set()
        thread = threading.Thread(target=observe_and_claim)
        thread.start()
        try:
            self.assertTrue(finished.wait(1.), "磁盘锁不能阻止 observe/claim/reserve/原结果提交")
        finally:
            other.rollback()
            other.close()
            thread.join(5.)
        self.assertEqual(errors, [])
        self.assertEqual(self.bridge.command(body["requestId"])["status"], "succeeded")

    def test_admission_delayed_by_database_does_not_claim_old_vehicle(self):
        import sqlite3
        from pathlib import Path
        body = self.body()
        other = sqlite3.connect(str(Path(self.tmp.name) / "passages.sqlite3"))
        other.execute("BEGIN IMMEDIATE")
        entered, done, results = threading.Event(), threading.Event(), []
        # 跟投影线程共用连接；设置/移除trace也须遵守连接锁，否则
        # sqlite连接互斥与Python回调的GIL可能形成死锁，掩盖被测并发语义。
        with self.bridge._db_lock:
            self.bridge._db.set_trace_callback(lambda sql: entered.set() if sql.startswith("INSERT INTO commands") else None)
        def admit():
            try:
                results.append(self.bridge.queue_command(body))
            finally:
                done.set()
        thread = threading.Thread(target=admit)
        thread.start()
        try:
            self.assertTrue(entered.wait(1.))
            new_passage = self.enter()
            self.assertNotEqual(new_passage, body["passageId"])
            self.assertIsNone(self.bridge.claim_command_fast("CH1"))
        finally:
            other.rollback()
            other.close()
            thread.join(5.)
            with self.bridge._db_lock:
                self.bridge._db.set_trace_callback(None)
        self.assertTrue(done.is_set())
        self.assertEqual(results[0]["status"], "failed")
        self.assertIsNone(self.bridge.claim_command_fast("CH1"))

    def test_only_complete_absence_arms_new_passage(self):
        for _ in range(10):
            self.bridge.observe("CH1", False, False)
        self.bridge.observe("CH1", True, True)
        self.assertIsNone(self.bridge.current_passage("CH1"))
        first = self.enter()
        self.assertIsNotNone(first)
        for _ in range(20):
            self.bridge.observe("CH1", True, True)
        self.assertEqual(first, self.bridge.current_passage("CH1"))
        self.assertNotEqual(first, self.enter())

    def test_repeated_measurements_keep_passage(self):
        passage = self.enter()
        first, second = self.record(), self.record()
        self.assertNotEqual(first, second)
        self.assertEqual({x.payload["passageId"] for x in self.bridge.outbox.pending()}, {passage})

    def test_lost_ack_retry_retains_exact_payload_and_id(self):
        self.enter()
        measurement = self.record()
        self.client.failure = HttpError()
        with self.assertRaises(HttpError):
            self.bridge.sync_once()
        self.assertEqual(self.bridge.outbox.get(measurement).status, "pending")
        self.client.failure = None
        self.bridge.sync_once()
        self.assertEqual(self.client.posts[0], self.client.posts[1])
        self.assertEqual(self.bridge.outbox.get(measurement).status, "sent")
        self.assertEqual(len(self.client.rows), 1)

    def test_conflict_blocks_without_deleting_or_renumbering(self):
        self.enter()
        measurement = self.record()
        self.client.failure = HttpError(409)
        self.bridge.sync_once()
        self.assertEqual(self.bridge.outbox.get(measurement).status, "blocked")
        self.bridge.sync_once()
        self.assertEqual(len(self.client.posts), 1)

    def test_restart_keeps_outbox_but_quarantines_vehicle_and_command(self):
        passage = self.enter()
        measurement = self.record()
        self.bridge.queue_command(self.body(passage))
        self.bridge.close()
        self.bridge = LoadRadarBridge(self.config, self.client)
        self.bridge.observe("CH1", True, True)
        self.assertIsNone(self.bridge.current_passage("CH1"))
        self.assertEqual(self.bridge.command("request-1")["status"], "failed")
        self.bridge.sync_once()
        self.assertEqual(self.bridge.outbox.get(measurement).status, "sent")

    def test_stream_gap_quarantines_and_rejects_old_command(self):
        passage = self.enter()
        self.bridge._frames["CH1"] = time.monotonic() - 6
        self.bridge.observe("CH1", True, True)
        with self.assertRaises(BridgeConflict):
            self.bridge.queue_command(self.body(passage))

    def test_command_idempotency_and_payload_conflict(self):
        body = self.body()
        first = self.bridge.queue_command(body)
        self.assertEqual(first, self.bridge.queue_command(body))
        with self.assertRaises(BridgeConflict):
            self.bridge.queue_command(dict(body, confirmedEmpty=True))
        claimed = self.bridge.claim_command("CH1")
        self.assertEqual(claimed["status"], "running")
        measurement = self.record(command=claimed)
        self.bridge.finish_command(body["requestId"], measurement)
        self.assertEqual(self.bridge.queue_command(body)["measurementId"], measurement)

    def test_departure_cancels_pending_commands(self):
        self.bridge.queue_command(self.body())
        self.enter()
        self.assertEqual(self.bridge.command("request-1")["status"], "failed")
        self.assertIsNone(self.bridge.claim_command("CH1"))

    def test_capture_requires_explicit_confirmation_and_current_live_empty(self):
        body = self.body(kind="baseline-capture")
        with self.assertRaises(ValueError):
            self.bridge.queue_command(dict(body, confirmedEmpty=False))
        self.bridge.queue_command(body)
        command = self.bridge.claim_command("CH4")
        with self.assertRaises(BridgeConflict):
            self.bridge.record(dict(self.result(), channel="CH4"), configuration_hash="a" * 64, source_mode="demo", command=command)
        with self.assertRaises(BridgeConflict):
            self.bridge.record(dict(self.result(), channel="CH4", empty=False), configuration_hash="a" * 64,
                               source_mode="live", command=command)

    def test_candidate_asset_and_hash_persist_before_network(self):
        body = self.body(kind="baseline-capture")
        self.bridge.queue_command(body)
        command = self.bridge.claim_command("CH4")
        with patch("app.load_radar_bridge.make_candidate", return_value=b"candidate") as make:
            measurement = self.record(command=command)
        make.assert_called_once()
        item = self.bridge.outbox.get(measurement)
        self.assertEqual(item.asset, b"candidate")
        self.assertEqual(item.payload["baselineCandidateSha256"], hashlib.sha256(item.asset).hexdigest())
        self.assertEqual(self.client.posts, [])
        self.bridge.sync_once()
        self.assertEqual(self.bridge.outbox.get(measurement).status, "sent")

    def test_no_automatic_candidate_generation(self):
        self.enter()
        with patch("app.load_radar_bridge.make_candidate") as make:
            self.record()
        make.assert_not_called()

    def test_command_disk_failure_keeps_original_result_and_running_until_flush(self):
        body = self.body()
        self.bridge.queue_command(body)
        command = self.bridge.claim_command("CH1")
        with patch.object(self.bridge.outbox, "save", side_effect=OSError("磁盘满")):
            with self.assertRaises(OSError):
                self.record(command=command)
        pending = self.bridge._uncommitted["CH1"]
        self.bridge.finish_command("request-1", error="命令未完成")
        self.assertEqual(self.bridge.command("request-1")["status"], "running")
        with self.assertRaises(BridgeConflict):
            self.record()
        self.assertIs(self.bridge._uncommitted["CH1"], pending)
        self.bridge.flush_pending("CH1")
        result = self.bridge.command("request-1")
        self.assertEqual((result["status"], result["measurementId"]), ("succeeded", pending[0]["measurementId"]))
        self.assertEqual(self.bridge.outbox.get(result["measurementId"]).payload, pending[0])
        self.bridge.sync_once()
        self.assertEqual(self.client.posts, [pending[0]])

    def test_command_restart_after_durable_save_recovers_without_finish(self):
        body = self.body()
        self.bridge.queue_command(body)
        command = self.bridge.claim_command("CH1")
        with patch.object(self.bridge, "_project_measurement", side_effect=OSError("投影失败")):
            with self.assertRaises(OSError):
                self.record(command=command)
        measurement = self.bridge.command("request-1")["measurementId"]
        self.assertIsNotNone(measurement)
        self.bridge.close()
        self.bridge = LoadRadarBridge(self.config, self.client)
        result = self.bridge.queue_command(body)
        self.assertEqual((result["status"], result["measurementId"]), ("succeeded", measurement))
        self.assertIsNone(self.bridge.claim_command("CH1"))
        self.assertIsNone(self.bridge.current_passage("CH1"))
        self.bridge.sync_once()
        self.assertEqual(self.bridge.outbox.get(measurement).status, "sent")

    def test_committed_result_wins_over_late_error_and_pending_projection(self):
        body = self.body()
        self.bridge.queue_command(body)
        command = self.bridge.claim_command("CH1")
        measurement = self.record(command=command)
        self.bridge.finish_command("request-1", error="迟到异常")
        self.assertEqual(self.bridge.command("request-1")["status"], "succeeded")
        with self.bridge._db:
            self.bridge._db.execute("UPDATE commands SET status='pending',measurement=NULL")
        self.assertIsNone(self.bridge.claim_command("CH1"))
        self.assertEqual(self.record(command=command), measurement)
        self.assertEqual(self.bridge.outbox.stats()["total"], 1)

    def test_measurement_id_alone_cannot_claim_command_success(self):
        body = self.body()
        self.bridge.queue_command(body)
        self.bridge.claim_command("CH1")
        automatic = self.record()
        self.bridge.finish_command("request-1", automatic)
        result = self.bridge.command("request-1")
        self.assertEqual(result["status"], "failed")
        self.assertIsNone(result["measurementId"])

    def test_flush_old_result_cannot_update_new_vehicle_lane(self):
        body = self.body()
        self.bridge.queue_command(body)
        command = self.bridge.claim_command("CH1")
        with patch.object(self.bridge, "_project_measurement", side_effect=OSError("投影失败")):
            with self.assertRaises(OSError):
                self.record(command=command)
        new_passage = self.enter()
        self.bridge.flush_pending("CH1")
        lane = self.bridge._lane("CH1")
        self.assertEqual(lane["passage"], new_passage)
        self.assertIsNone(lane["measurement"])
        self.assertEqual(self.bridge.queue_command(body)["status"], "succeeded")

    def test_outbox_read_failure_does_not_turn_into_missing_command(self):
        self.bridge.queue_command(self.body())
        with patch.object(self.bridge.outbox, "command_result", side_effect=OSError("读取失败")):
            with self.assertRaises(OSError):
                self.bridge.command("request-1")
            with self.assertRaises(OSError):
                self.bridge.claim_command("CH1")

    def test_disk_failure_preserves_original_uncommitted_id(self):
        self.enter()
        with patch.object(self.bridge.outbox, "save", side_effect=OSError("磁盘满")):
            with self.assertRaises(OSError):
                self.record()
        original = self.bridge._uncommitted["CH1"][0]
        self.bridge.flush_pending("CH1")
        self.assertEqual(self.bridge.outbox.pending()[0].payload, original)
        self.assertEqual(self.bridge._uncommitted, {})

    def test_bad_receipt_does_not_acknowledge(self):
        self.enter()
        measurement = self.record()
        with patch.object(self.client, "post_measurement", return_value={"measurement": {}}):
            self.bridge.sync_once()
        self.assertEqual(self.bridge.outbox.get(measurement).status, "blocked")

    def test_poison_measurement_does_not_starve_the_outbox(self):
        """★ 后台在线却持续 5xx 的单条，不得堵死整个发件箱（2026-09-23，M10）。

        原来首个可重试错误就 raise：排在前面的"毒"测量让后面的好测量
        发不出去、基线同步段也执行不到 —— _loaded 10 秒过期后全通道
        volume 不可用。现在 HTTP 429/5xx 只记账跳过该条继续发；那条仍留
        pending（不是终态 blocked），过载恢复后照常自动发出。
        """
        self.enter()
        bad = self.record()
        self.enter()
        good = self.record()
        # ★ 毒条必须排在**发件箱队头**才验得出队头阻塞。发件箱按 seq（插入顺序）取，
        #   见 `Outbox.pending` 的 `ORDER BY seq` —— 所以要看的是**队列里的实际顺序**。
        #
        #   这里原来写的是 `assertLess(bad, good)`：比的是两个测量 ID 的字典序，
        #   而 ID 是随机 UUID（m-<uuid4>），与处理顺序毫无关系 —— 约一半概率
        #   反序而误报失败（2026-09-23 全量回归连续两次撞上）。前置检查查错了东西，
        #   用例本身的逻辑是对的：bad 先记录，seq 更小，本来就在队头。
        order = [item.measurement_id for item in self.bridge.outbox.pending()]
        self.assertLess(order.index(bad), order.index(good),
                        "毒条必须排在发件箱队头，否则验不出队头阻塞")
        real_post = self.client.post_measurement

        def selective(payload):
            if payload["measurementId"] == bad:
                raise HttpError(500)
            return real_post(payload)

        with patch.object(self.client, "post_measurement", side_effect=selective):
            self.bridge.sync_once()                   # 不许抛 —— 好数据必须发完
        self.assertEqual(self.bridge.outbox.get(bad).status, "pending",
                         "毒条留在 pending 等重试，不得误升级成终态 blocked")
        self.assertEqual(self.bridge.outbox.get(good).status, "sent",
                         "排在毒条后面的好测量必须照常发出")

    def test_connection_failure_still_stops_the_round(self):
        """连接失败（后台不可达）保留提前收手语义 —— 后面的项大概率同因。"""
        self.enter()
        first = self.record()
        self.enter()
        second = self.record()
        calls = []

        def unreachable(payload):
            calls.append(payload["measurementId"])
            raise HttpError()                         # status=None：连接失败

        with patch.object(self.client, "post_measurement", side_effect=unreachable):
            with self.assertRaises(HttpError):
                self.bridge.sync_once()
        self.assertEqual(calls, [first], "连接失败后不该继续白打后续项")
        self.assertEqual(self.bridge.outbox.get(second).status, "pending")

    def test_exact_binding_installs_version_and_stream_gap_unloads(self):
        self.enter()
        measurement = self.record()
        self.bridge.sync_once()
        binding = {"vehicleIdentity": "truck:trailer", "licensePlate": "宁A25771"}
        self.client.rows[measurement]["binding"] = binding
        buf = io.BytesIO()
        np.savez_compressed(buf, floor_z=np.array(1.), box=np.array([0., 1., 0., 1., 1., 2.]),
                            cloud=np.zeros((50, 3)), created=np.array("测试确认"))
        data = buf.getvalue()
        now = datetime.now(timezone.utc) - timedelta(seconds=1)
        version = dict(binding, versionId="v-one", nodeId="jt-test", channelId="CH4", calibrationId="cal-4",
                       active=True, activatedAt=now.isoformat(), createdAt=now.isoformat(), measurementId="m-baseline",
                       assetSha256=hashlib.sha256(data).hexdigest(), scopeKey="a" * 64,
                       actor="测试管理员", reason="人工确认空车")
        self.client.assets["v-one"] = data
        self.client.versions = [version]
        self.client.rows["m-baseline"] = {"measurement": dict(self.client.rows[measurement]["measurement"],
                                                            measurementId="m-baseline", channelId="CH4", calibrationId="cal-4",
                                                            baselineCandidateSha256=version["assetSha256"])}
        self.bridge.sync_once()
        baseline, actual, plate = self.bridge.baseline("CH1", time.time())
        self.assertEqual(actual, "v-one")
        self.assertEqual(plate, "宁A25771")
        self.assertEqual(baseline.n, 50)
        self.assertIsNone(self.bridge.baseline("CH1", now.timestamp() - 1)[0])
        self.bridge._frames["CH1"] = time.monotonic() - 6
        self.assertIsNone(self.bridge.baseline("CH1", time.time())[0])

    def test_duplicate_process_is_rejected(self):
        with self.assertRaises(ValueError):
            LoadRadarBridge(self.config, self.client)

    def test_receipt_matches_single_precision_and_rejects_changed_business_values(self):
        self.enter()
        result = dict(self.result(), cloud=[[.1, .2, .3]])
        measurement = self.bridge.record(result, configuration_hash="a" * 64, source_mode="live")
        payload = self.bridge.outbox.get(measurement).payload
        actual = dict(payload, points=[[.1, .2, .3]])
        self.assertTrue(self.bridge._same_measurement({"measurement": actual}, payload))
        for key, value in (("baselineCandidateSha256", "c" * 64), ("calibrationId", "other"),
                           ("volumeM3", 0), ("frameDegraded", True), ("sourceMode", "demo")):
            self.assertFalse(self.bridge._same_measurement({"measurement": dict(actual, **{key: value})}, payload))

    def test_worker_hook_reports_real_runtime_mode_without_legacy_baseline(self):
        import copy
        from app.service import ChannelWorker, EventBus
        from tests import _helpers as helpers
        settings, channel, _, _, _ = helpers.ctx()
        channel = copy.deepcopy(channel)
        channel.id = "CH1"
        with patch("app.application.channel_worker.EmptyBaseline.load", side_effect=AssertionError("不能读取旧基线")):
            worker = ChannelWorker(channel, EventBus(), settings.plate, save_override=False,
                                   demo=True, integration=self.bridge)
        try:
            self.enter()
            frame = worker.source.read()
            self.assertIsNotNone(frame)
            measurement = worker._run_detection(frame, SimpleNamespace(deviation=0))
            from tests.test_result_finalizer import await_result
            await_result(worker, exported=False)
            payload = self.bridge.outbox.get(measurement).payload
            self.assertEqual(payload["sourceMode"], "demo")
            self.assertEqual(len(payload["configurationHash"]), 64)
            self.assertIsNone(payload["baselineVersionId"])
            self.assertIsNone(payload["volumeM3"])
            self.assertIsNone(payload["baselineCandidateSha256"])
            self.assertEqual(worker.st.last_result["integration"]["measurementId"], measurement)
        finally:
            worker.stop()
            self.assertTrue(worker.close_results(5.))
            worker.source.close()

    def test_manual_command_clears_the_stale_settle_window(self):
        """★ 人工命令接管测量的那一刻必须清掉自动沉降窗口（2026-09-23，M5）。

        原来 finally 只清 5 个装配字段，_settle_t0/_settle_frames/_settle_pairs
        一直残留：① 余下会话的进场帧采集被 _settle_t0 非空挡死；
        ② 检测诊断会读到旧窗口帧交叉污染。这条走 _tick_observation 全链路：
        真停稳、真开窗、真认领命令，断言 _run_detection 被调用的**那一刻**
        窗口已经清空（即清窗发生在拦截点之前，且不依赖 finally 兜底）。
        """
        import copy
        from app.pointcloud import Frame
        from app.radar_source import build_source
        from app.service import ChannelWorker, EventBus
        from tests import _helpers as helpers
        from tests._observations import ObservationClock

        clock = ObservationClock(self)
        settings, channel, _, _, _ = helpers.ctx()
        channel = copy.deepcopy(channel)
        channel.id, channel.mode = "CH1", "live"
        channel.abs_output_dir = self.tmp.name
        channel.abs_baseline_dir = self.tmp.name
        # live 模式才能过人工命令闸门（eff_mode=="live"），但数据源换成
        # synthetic —— 测试机器上不许开真 UDP socket。
        with patch("app.application.channel_worker.build_source",
                   side_effect=lambda *a, **k: build_source(*a, **dict(k, mode="synthetic"))):
            worker = ChannelWorker(channel, EventBus(), settings.plate,
                                    save_override=False, integration=self.bridge)
        self.addCleanup(worker.stop)
        self.addCleanup(worker.source.close)
        worker.settle_sec = 100              # 只攒窗口帧，不让自动测量抢跑
        passage = self.enter()               # bridge 先确认车次
        # live 通道过「准备就绪」闸门：preparation_status 返回 ready
        # （生产里由后台准备回执写入；本用例只测窗口清空，不测准备流程本身）。
        self.bridge._prepared["CH1"] = (passage, "ready", None, time.monotonic())
        rng = np.random.default_rng(20260923)
        frame = Frame(np.column_stack((rng.uniform(-1, 1, 18000),
                                       rng.uniform(-5, 5, 18000),
                                       rng.uniform(-3.7, -2, 18000))))
        clock.settle(worker, frame)           # 判稳 -> 自动沉降窗口开启
        self.assertIsNotNone(worker._settle_t0, "自动窗口应当先开着（构造残留场景）")
        for _ in range(4):                    # 攒够人工命令要的三帧完整观测
            clock.tick(worker, frame)
        self.assertGreaterEqual(len(worker._integration_frames), 3)

        seen = {}

        def fake_run(f, stab, command=None, *args, **kw):
            seen["settle_t0"] = worker._settle_t0
            seen["settle_frames"] = list(worker._settle_frames)
            return "m-manual"
        worker._run_detection = fake_run
        self.bridge.queue_command(self.body(passage=passage, kind="remeasure"))
        clock.tick(worker, frame)             # 下一帧认领命令并接管测量
        self.assertIn("settle_t0", seen, "人工命令必须真的接管测量（claim 到并走到检测）")
        self.assertIsNone(seen["settle_t0"],
                          "命令接管测量的那一刻窗口必须已清空 —— "
                          "修复前残留的 _settle_t0 会挡死进场帧采集并污染诊断")
        self.assertEqual(seen["settle_frames"], [])
        self.assertIsNone(worker._settle_t0)
        self.assertEqual(worker._settle_frames, [])

    def test_http_commands_authentication_and_old_writes_blocked(self):
        from http.server import ThreadingHTTPServer
        from app.web.server import DashboardHandler
        handler = type("Handler", (DashboardHandler,), {"service": SimpleNamespace(integration=self.bridge)})
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        base = "http://127.0.0.1:%d" % server.server_port
        urlopen = build_opener(ProxyHandler({})).open
        body = self.body()
        data = json.dumps(body).encode()
        headers = {"X-Load-Radar-Node": self.config.node_id, "X-Load-Radar-Key": self.config.agent_key}
        with self.assertRaises(HTTPError) as error:
            urlopen(Request(base + "/api/integration/commands", data=data), timeout=2)
        self.assertEqual(error.exception.code, 401)
        error.exception.close()
        with urlopen(Request(base + "/api/integration/commands", data=data, headers=headers), timeout=2) as response:
            self.assertEqual(json.load(response)["status"], "pending")
        with urlopen(Request(base + "/api/integration/commands/request-1", headers=headers), timeout=2) as response:
            self.assertEqual(json.load(response)["passageId"], body["passageId"])
        command = self.bridge.claim_command("CH1")
        measurement = self.record(command=command)
        # 内部完成凭据损坏不能冒充客户端格式错误/未执行；原编号与测量均保留。
        self.bridge.outbox._connection.execute("UPDATE command_results SET command_json='{' WHERE request_id='request-1'")
        for request in (Request(base + "/api/integration/commands/request-1", headers=headers),
                        Request(base + "/api/integration/commands", data=data, headers=headers)):
            with self.assertRaises(HTTPError) as error:
                urlopen(request, timeout=2)
            self.assertEqual(error.exception.code, 503)
            error.exception.close()
        self.assertIsNotNone(self.bridge.outbox.get(measurement))
        self.assertEqual(self.bridge.outbox.stats()["total"], 1)
        for _ in range(32):
            with self.assertRaises(HTTPError) as error:
                urlopen(Request(base + "/api/baseline/capture", data=b"{}", headers=headers), timeout=2)
            self.assertEqual(error.exception.code, 403)
            self.assertEqual(error.exception.headers["Connection"], "close")
            error.exception.close()

    def test_takeover_blocks_only_old_pages_and_keeps_read_interfaces(self):
        from http.server import ThreadingHTTPServer
        from app.web.server import DashboardHandler
        for integrated in (True, False):
            with self.subTest(integrated=integrated):
                service = SimpleNamespace(integration=self.bridge if integrated else None,
                                          workers=[], snapshot=lambda: {"channels": [], "ok": True})
                handler = type("Handler", (DashboardHandler,), {"service": service})
                server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
                thread = threading.Thread(target=server.serve_forever, daemon=True)
                thread.start()
                try:
                    base = "http://127.0.0.1:%d" % server.server_port
                    urlopen = build_opener(ProxyHandler({})).open
                    for path in ("/", "/index.html?old=1", "/static/index.html"):
                        if integrated:
                            with self.assertRaises(HTTPError) as error:
                                urlopen(base + path, timeout=2)
                            with error.exception as response:
                                self.assertEqual(response.code, 403)
                                self.assertEqual(response.headers["Cache-Control"], "no-store")
                                self.assertIn("CarPointCloudWebApi", json.load(response)["error"])
                        else:
                            with urlopen(base + path, timeout=2) as response:
                                self.assertEqual(response.code, 200)
                                self.assertIn(b"<html", response.read().lower())
                    for path in ("/api/health", "/api/state"):
                        with urlopen(base + path, timeout=2) as response:
                            self.assertTrue(json.load(response)["ok"])
                finally:
                    server.shutdown()
                    server.server_close()
                    thread.join(2)

    def test_takeover_does_not_open_old_browser_but_standalone_still_can(self):
        from app.web.server import serve
        for integrated in (True, False):
            service = SimpleNamespace(integration=self.bridge if integrated else None,
                                      st=SimpleNamespace(service=SimpleNamespace(host="127.0.0.1", port=8080)))
            with patch("app.web.server._ExclusiveHTTPServer") as server, patch("app.web.server.threading.Timer") as timer:
                self.assertIs(serve(service, open_browser=True), server.return_value)
                if integrated:
                    timer.assert_not_called()
                else:
                    timer.assert_called_once()
                    timer.return_value.start.assert_called_once()

    def test_rejected_post_body_drain_is_bounded_and_restores_timeout(self):
        from email.message import Message
        from unittest.mock import Mock
        from app.web.server import DashboardHandler
        for size, consumed in ((2, 2), (4096, 4096), (4097, 0)):
            headers = Message()
            headers["Content-Length"] = str(size)
            connection = Mock()
            connection.gettimeout.return_value = 8
            handler = SimpleNamespace(command="POST", headers=headers, connection=connection,
                                      rfile=io.BytesIO(b"x" * size))
            DashboardHandler._close_rejected_request(handler)
            self.assertEqual(handler.rfile.tell(), consumed)
            self.assertTrue(handler.close_connection)
            if consumed:
                connection.settimeout.assert_called_with(8)
        connection = Mock()
        connection.gettimeout.return_value = None
        headers = Message()
        headers["Content-Length"] = "2"
        body = Mock()
        body.read1.side_effect = TimeoutError("客户端未发完整正文")
        handler = SimpleNamespace(command="POST", headers=headers, connection=connection, rfile=body)
        DashboardHandler._close_rejected_request(handler)
        connection.settimeout.assert_called_with(None)
        self.assertEqual(body.read1.call_count, 1)

    def test_stop_keeps_database_and_lease_until_running_worker_finishes(self):
        from app.service import LidarService, EventBus
        release = threading.Event()
        class SlowWorker(threading.Thread):
            ch = SimpleNamespace(id="CH1")
            def stop(self):
                pass
            def run(self):
                release.wait(3)
        worker = SlowWorker()
        worker.start()
        service = LidarService.__new__(LidarService)
        service.director, service.hub = None, None
        service.workers, service.integration, service.bus = [worker], self.bridge, EventBus()
        try:
            self.assertFalse(service.stop(timeout=0))
            self.assertFalse(self.bridge._closed)
            self.assertEqual(self.bridge.outbox.pending(), [])
        finally:
            release.set()
            worker.join(3)
            service._pending_integration_stop.join(3)
        self.assertTrue(self.bridge._closed)


if __name__ == "__main__":
    unittest.main()
