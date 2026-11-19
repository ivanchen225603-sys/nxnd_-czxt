# -*- coding: utf-8 -*-
"""双雷达采集管理的用例 —— 对应 `docs/68` §10 的 30 条场景。

怎么测的
--------

* **假 WebApi 起在 `http.server` 上**（真 HTTP、真 `WebApiClient`），可编程：
  固定 Token、受理状态码与响应体、进度阶段序列、取消结果、状态接口结果，
  并记录收到的**每一个请求**（用于断言幂等键、请求头、请求次数）。
* **时钟注入，不真 sleep、不起线程**：直接驱动 `CaptureSession`；
  记录器那几条（车道校验、无账号、启动自检、会话号变化）直接调
  `field_standstill_log._poll_once()` / `_build_capture()`，喂一份快照即可。
* 台账写在 `tmpdir` 里，用例之间互不干扰。

▲ 这份文件自己也受 GBK 规则约束（`tests/test_encoding.py` 扫 tests/）：
  不写 GBK 编不进的字面量符号。
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from tools import dual_radar_capture as drc
from tools import field_standstill_log as recorder
from tools import field_watchdog as watchdog

PLATE = "宁A34076T"
MID = "m-53626d48080b49a1b5346629eaf04f8f"
TEST_RECORDER_TOKEN = "test-only-recorder-token-0123456789abcdef"


# ═══════════════════════════════════════════════════════════ 假 WebApi


class FakeApi:
    """假 WebApi 的"剧本"。被 `_Handler` 在 HTTP 线程里调用。"""

    def __init__(self):
        self.requests = []
        self.logins = 0
        self.login_status = 200
        self.login_message = "success"
        self.status_status = 200
        self.status_body = {"code": 200, "message": "success", "success": True,
                            "data": {"scan": {"state": "idle"}, "preview": {"available": True}}}
        self.capture_script = []
        self.progress_script = []
        self.cancel_status = 200
        self.cancel_message = "已取消"
        self.unauthorized_times = 0
        self.unauthorized_paths = ""        # 非空时只对这些路径返回 401（其余正常）
        self.login_fail_times = 0           # 前几次登录返回 login_fail_status（模拟 WebApi 重启中）
        self.login_fail_status = 503

    # ── 剧本推进 ──
    def _next(self, script):
        if not script:
            return 200, {"code": 200, "message": "success", "success": True, "data": {}}
        return script.pop(0) if len(script) > 1 else script[0]

    def route(self, method, path, body, headers):
        self.requests.append({"method": method, "path": path, "body": body,
                              "idem": headers.get("Idempotency-Key"),
                              "auth": headers.get("Authorization"),
                              "fixed_token": headers.get("X-Radar-Live-Token")})
        if path == "/api/auth/login":
            self.logins += 1
            if self.login_fail_times > 0:
                self.login_fail_times -= 1
                return self.login_fail_status, {"code": self.login_fail_status, "success": False,
                                                "message": "服务暂不可用", "data": None}
            data = {"token": "tk-%d" % self.logins} if self.login_status == 200 else None
            return self.login_status, {"code": self.login_status, "message": self.login_message,
                                       "success": self.login_status == 200, "data": data}
        if headers.get("X-Radar-Live-Token") != TEST_RECORDER_TOKEN:
            return 401, {"code": 401, "message": "缺少令牌", "data": None, "success": False}
        if self.unauthorized_times > 0 and (not self.unauthorized_paths
                                            or self.unauthorized_paths in path):
            self.unauthorized_times -= 1
            return 401, {"code": 401, "message": "令牌已过期", "data": None, "success": False}
        if path == "/api/v1/radar-live/status":
            if self.status_status != 200:
                return self.status_status, {"code": self.status_status, "message": "雷达服务状态不可用",
                                            "data": None, "success": False}
            return 200, self.status_body
        if method == "POST" and path == "/api/v1/radar-live/captures":
            status, payload = self._next(self.capture_script)
            return status, payload
        if method == "POST" and path.endswith("/cancel"):
            return self.cancel_status, {"code": self.cancel_status, "message": self.cancel_message,
                                        "success": self.cancel_status == 200, "data": {}}
        if method == "GET" and path.startswith("/api/v1/radar-live/captures/"):
            status, payload = self._next(self.progress_script)
            return status, payload
        return 404, {"code": 404, "message": "没有这个接口", "data": None, "success": False}

    # ── 断言辅助 ──
    def posts(self):
        return [r for r in self.requests
                if r["method"] == "POST" and r["path"] == "/api/v1/radar-live/captures"]

    def cancels(self):
        return [r for r in self.requests if r["path"].endswith("/cancel")]

    def completed(self):
        return [r for r in self.requests if r["path"].startswith("/api/v1/radar-live/captures/")]


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):                        # 别把测试输出刷满
        pass

    def _go(self, method):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b""
        try:
            body = json.loads(raw.decode("utf-8")) if raw else None
        except ValueError:
            body = None
        status, payload = self.server.api.route(method, self.path, body, self.headers)
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self._go("GET")

    def do_POST(self):
        self._go("POST")


def start_fake_api():
    api = FakeApi()
    server = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
    server.api = api
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    host, port = server.server_address[:2]
    return api, server, "http://%s:%d" % (host, port)


# ═══════════════════════════════════════════════════════════ 夹具


class Harness:
    """把假 WebApi + 台账 + 同步会话装在一起；时钟自己走。"""

    def __init__(self, **cfg_kw):
        self.tmp = tempfile.mkdtemp(prefix="dual-radar-")
        cfg_kw.setdefault("record_dir", self.tmp)
        token_file = os.path.join(self.tmp, "recorder.json")
        with open(token_file, "w", encoding="utf-8") as stream:
            json.dump({"recorderToken": TEST_RECORDER_TOKEN}, stream)
        cfg_kw.setdefault("token_file", token_file)
        self.api, self.server, self.base = start_fake_api()
        self.cfg = drc.CaptureConfig(**cfg_kw)
        self.clock = [1000.0]
        self.out = []
        self.rings = []
        self.lock = threading.RLock()
        self.ledger = drc.VehicleLedger(self.cfg.ledger_path(), self.lock, ttl=self.cfg.ledger_ttl)
        self.client = drc.WebApiClient(self.base, self.cfg)
        self.session = self.new_session()

    def new_session(self, **kw):
        """再建一个会话 —— 模拟"记录器被看门狗重启"（台账是同一个文件）。"""
        return drc.CaptureSession(self.client, self.ledger, self.out.append, self.cfg,
                                  clock=lambda: self.clock[0], wall=lambda: self.clock[0],
                                  stamp=lambda: "10:04:29",
                                  ring=lambda: self.rings.append(1), **kw)

    # ── 驱动 ──
    def tick(self, seconds=2.0):
        self.clock[0] += seconds
        self.session.tick()

    def advance(self, seconds, step=2.0):
        done = 0.0
        while done < seconds - 1e-9:
            take = min(step, seconds - done)
            self.tick(take)
            done += take

    def leave(self, plate=PLATE):
        """离场 —— 与停稳记录器一样：**主线程先关台账，再通知管理器**。"""
        self.ledger.close_vehicle(plate, self.clock[0])
        self.session.left(self.clock[0])

    # ── 读 ──
    def text(self):
        return "\n".join(self.out)

    def records(self):
        rows = []
        for name in sorted(os.listdir(self.tmp)):
            if name.endswith(".jsonl"):
                with open(os.path.join(self.tmp, name), encoding="utf-8") as fh:
                    rows += [json.loads(line) for line in fh if line.strip()]
        return rows

    def ledger_raw(self):
        try:
            with open(self.cfg.ledger_path(), encoding="utf-8") as fh:
                return json.load(fh)["entries"]
        except (OSError, ValueError, KeyError):
            return {}

    def entry(self, plate=PLATE):
        return self.ledger_raw().get(plate, {})

    def close(self):
        try:
            self.server.shutdown()
            self.server.server_close()
        except Exception:                                # noqa: BLE001
            pass


def accepted_body(workflow="wf-1", **extra):
    data = {"workflowId": workflow, "licensePlate": PLATE, "duplicate": False,
            "status": "accepted", "message": "已受理", "needsManualConfirmation": False}
    data.update(extra)
    return 200, {"code": 200, "message": "success", "success": True, "data": data}


def progress(stage, terminal=False, **extra):
    data = {"workflowId": "wf-1", "stage": stage, "message": "", "terminal": terminal,
            "capturedCameras": [], "persistedCameras": [], "persistenceState": None,
            "safetyBlockReason": None, "error": None}
    data.update(extra)
    return 200, {"code": 200, "message": "success", "success": True, "data": data}


def stored_dimensions(review="approved", record_id=12345):
    return {"reviewStatus": review, "recordId": record_id,
            "dimensions": {"cabToTrailerFrontDist": 120.0, "trailerLength": 1133.6,
                           "trailerWidth": 255.0, "trailerBodyHeight": 180.0,
                           "trailerBottomToGroundHeight": 154.4, "highLowPlateHeight": 4.4,
                           "ribs": [{"distToPrev": None}, {"distToPrev": 374.8}],
                           "highLowPlates": [{"plateId": 1, "highestToGround": 154.4,
                                              "lowestToGround": 150.0}]}}


LENGTH_MEASUREMENT = {"stage": "completed",
                      "measurementStatus": {"reviewStatus": "pending",
                                            "lengthMeasurement": {
                                                "valid": True,
                                                "definition": "same_side_lower_band_longitudinal_projection",
                                                "heightAboveFloorM": [0.5, 1.0],
                                                "inwardBandM": [0.05, 0.35]}}}


def channel_state(fps=9.5, incomplete=0.0, rate=8800.0):
    return {"id": "CH4", "fps": fps, "incomplete_ratio": incomplete,
            "source": {"packet_rate": rate}}


# ═══════════════════════════════════════════════════════════ §10 的 30 条


class TestHappyPath(unittest.TestCase):
    """用例 1、2、3（含 6 步顺序、可以离开、结果字段、口径说明、JSONL）。"""

    def setUp(self):
        self.h = Harness()
        self.addCleanup(self.h.close)

    def test_01_normal_flow_to_completed(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [
            progress("waiting"),
            progress("captured", capturedCameras=["CAM_004", "CAM_005"]),
            progress("saved", persistedCameras=["CAM_004", "CAM_005"], persistenceState="saved"),
            progress("queued"),
            progress("analyzing"),
            progress("completed", terminal=True, message="计算完成并写入数据库",
                     dimensions=stored_dimensions(), analysis=LENGTH_MEASUREMENT),
        ]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(30)

        text = h.text()
        # 步骤条顺序
        for label in ("等待安全完整场", "双雷达点云已捕获", "两份 PCD 已落盘",
                      "已配对，排队分析", "Analyzer 正在融合计算", "计算完成并写入数据库"):
            self.assertIn(label, text)
        # 落盘那一行必须带"车可以离开"
        saved_line = [line for line in h.out if "两份 PCD 已落盘" in line and "√" in line]
        self.assertTrue(saved_line, "没有找到带'车可以离开'的落盘行")
        self.assertIn("车可以离开", saved_line[0])
        # 结果字段与单位
        self.assertIn("车厢内长 1133.6 cm", text)
        self.assertIn("拉筋 1 根", text)              # 2 段 -> 1 根
        self.assertIn("底板离地 154.4 cm", text)
        # 口径说明行（不能省）
        self.assertIn("内长测线：同侧下部，底板以上 50.0～100.0 cm", text)
        # JSONL 一行
        rows = h.records()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["outcome"], "completed")
        self.assertEqual(rows[0]["measurement_id"], MID)
        self.assertEqual(rows[0]["idempotency_key"], "lr-" + MID)
        self.assertEqual(rows[0]["vehicle_times"]["measured"], 1000.0)
        self.assertIn("saved", rows[0]["stage_times"])
        # 受理请求的键
        self.assertEqual(h.api.posts()[0]["idem"], "lr-" + MID)
        self.assertEqual(h.api.posts()[0]["body"]["idempotencyKey"], "lr-" + MID)
        self.assertTrue(h.api.posts()[0]["body"]["confirmOverwriteEmptyProfile"])
        # 台账终局
        self.assertTrue(h.entry()["terminal"])
        self.assertEqual(h.entry()["workflow_id"], "wf-1")

    def test_02_completed_pending_review(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [
            progress("analyzing"),
            progress("completed", terminal=True, message="已入库",
                     dimensions=stored_dimensions(review="pending"), analysis=LENGTH_MEASUREMENT),
        ]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(20)
        self.assertIn("已入库，待人工审核", h.text())
        self.assertEqual(h.records()[0]["outcome"], "completed_pending_review")

    def test_03_pending_review_and_no_dimensions_are_not_failures(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [
            progress("pending_review", terminal=True, message="计算值已返回",
                     analysis={"stage": "completed", "measurementStatus": {"reviewStatus": "pending"},
                               "reviewMeasurements": {"cabToTrailerFrontDist": 1.2,
                                                      "dimensions": {"carriage": {"length": 11.336,
                                                                                  "width": 2.55,
                                                                                  "height": 1.8}},
                                                      "trailerBottomToGroundHeight": 1.544,
                                                      "ribCount": 2, "ribIntervals": [2.4, 3.75, 9.4],
                                                      "highLowPlates": [], "highLowPlateHeight": 0.044}}),
        ]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(20)
        self.assertIn("▲ 双雷达：计算值已返回，待人工审核（未入库）", h.text())
        self.assertNotIn("×", h.text())
        self.assertEqual(h.records()[0]["outcome"], "pending_review")

        # completed_without_dimensions：终局、不是故障
        h2 = Harness()
        self.addCleanup(h2.close)
        h2.api.capture_script = [accepted_body()]
        h2.api.progress_script = [progress("completed_without_dimensions", terminal=True,
                                           message="载货状态不确定")]
        h2.session.measured(h2.clock[0], MID, PLATE, channel_state())
        h2.advance(20)
        self.assertIn("▲ 双雷达：分析完成，本次不产出尺寸", h2.text())
        self.assertEqual(h2.records()[0]["outcome"], "no_dimensions")


class TestDedup(unittest.TestCase):
    """用例 4、23、24、25、28 —— 去重与重启。"""

    def setUp(self):
        self.h = Harness()
        self.addCleanup(self.h.close)

    def test_04_same_vehicle_measured_twice_posts_once(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(6)
        # 停着不走，服务 180 秒超时重测 -> 新测量号
        h.session.measured(h.clock[0], "m-retest-1", PLATE, channel_state())
        h.advance(6)
        self.assertEqual(len(h.api.posts()), 1)
        self.assertIn("同一辆车的重复测量", h.text())

    def test_23_record_restart_with_open_entry_does_not_trigger(self):
        """记录器重启后，同一辆车（新测量号）不得二次触发。"""
        h = self.h
        h.ledger.open_vehicle(PLATE, h.clock[0] - 300)
        h.ledger.mark_triggered(PLATE, MID, "lr-" + MID, h.clock[0] - 290)
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("saved")]
        # "重启"= 新会话（新进程），台账是同一个文件
        h.session = h.new_session()
        h.session.measured(h.clock[0], "m-retest-2", PLATE, channel_state())
        h.advance(4)
        self.assertEqual(h.api.posts(), [])
        self.assertIn("本车次已触发过", h.text())
        self.assertEqual(h.records()[0]["outcome"], "duplicate_skipped")

    def test_23b_open_entry_with_unfinished_workflow_resumes_instead(self):
        h = self.h
        h.ledger.open_vehicle(PLATE, h.clock[0] - 300)
        h.ledger.mark_triggered(PLATE, MID, "lr-" + MID, h.clock[0] - 290)
        h.ledger.mark_accepted(PLATE, "wf-old")
        h.ledger.mark_progress(PLATE, "saved", False)
        h.api.progress_script = [progress("completed", terminal=True, message="已入库",
                                          dimensions=stored_dimensions())]
        h.session = h.new_session()
        h.session.measured(h.clock[0], "m-retest-3", PLATE, channel_state())
        h.advance(4)
        self.assertEqual(h.api.posts(), [])
        self.assertIn("接力监控流程 wf-old", h.text())

    def test_24_leave_then_reenter_triggers_again(self):
        h = self.h
        h.api.capture_script = [accepted_body("wf-1"), accepted_body("wf-2")]
        h.api.progress_script = [progress("completed", terminal=True, message="已入库",
                                         dimensions=stored_dimensions())]
        h.ledger.open_vehicle(PLATE, h.clock[0])
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(6)                                  # 采完、终局、作业已收尾
        h.leave()                                     # 车倒出去
        self.assertEqual(h.entry()["state"], "closed")
        # 倒回来重进 = 新的一辆车
        h.ledger.open_vehicle(PLATE, h.clock[0])
        h.session.measured(h.clock[0], "m-second", PLATE, channel_state())
        h.advance(6)
        self.assertEqual(len(h.api.posts()), 2, "倒车重进应当能再采一次（方案 D4）")
        self.assertEqual({p["idem"] for p in h.api.posts()},
                         {"lr-" + MID, "lr-m-second"})

    def test_25_ttl_expiry_releases_the_plate(self):
        h = self.h
        h.ledger.open_vehicle(PLATE, h.clock[0] - 5000)     # 远超 TTL(1800)
        h.ledger.mark_triggered(PLATE, MID, "lr-" + MID, h.clock[0] - 5000)
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        h.session = h.new_session()
        h.session.measured(h.clock[0], "m-new", PLATE, channel_state())
        h.advance(4)
        self.assertEqual(len(h.api.posts()), 1, "TTL 过了应当允许再次触发")

    def test_28_killed_after_trigger_uses_the_same_key(self):
        """触发已落盘、请求没发出去就被杀 -> 重启后用同一个键重新受理。"""
        h = self.h
        h.ledger.open_vehicle(PLATE, h.clock[0] - 60)
        h.ledger.mark_triggered(PLATE, MID, "lr-" + MID, h.clock[0] - 55)
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        h.session = h.new_session()
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(4)
        posts = h.api.posts()
        self.assertEqual(len(posts), 1)
        self.assertEqual(posts[0]["idem"], "lr-" + MID, "必须用同一个幂等键，绝不换键")
        self.assertEqual(posts[0]["body"]["idempotencyKey"], "lr-" + MID)


class TestAcceptBranches(unittest.TestCase):
    """用例 5、6、7、8、9、29 —— 受理的每个分支。"""

    def setUp(self):
        self.h = Harness()
        self.addCleanup(self.h.close)

    def test_05_uncertain_retries_same_key_twice(self):
        h = self.h
        h.api.capture_script = [
            (502, {"code": 502, "message": "雷达服务返回其它非预期错误"}),
            (504, {"code": 504, "message": "雷达服务调用超时"}),
            (502, {"code": 502, "message": "雷达服务返回其它非预期错误"}),
        ]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(20)
        posts = h.api.posts()
        self.assertEqual(len(posts), 3, "同键重试最多 2 次（共 3 次请求）")
        self.assertEqual({p["idem"] for p in posts}, {"lr-" + MID})
        self.assertEqual({p["body"]["idempotencyKey"] for p in posts}, {"lr-" + MID})
        self.assertIn("勿换键重发", h.text())
        self.assertEqual(h.records()[0]["outcome"], "not_accepted:结果不确定")

    def test_05b_connection_failure_counts_as_uncertain(self):
        h = self.h
        calls = {"n": 0}

        def boom(*args, **kw):
            calls["n"] += 1
            raise urllib.error.URLError("connection refused")

        h.client.post_capture = boom
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(20)
        self.assertEqual(calls["n"], 3)
        self.assertEqual(h.records()[0]["outcome"], "not_accepted:结果不确定")

    def test_06_401_does_not_retry_with_account(self):
        h = self.h
        h.api.unauthorized_times = 1
        h.api.unauthorized_paths = "/captures"
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(6)
        self.assertEqual(h.api.logins, 0, "固定 Token 不触发账号登录")
        self.assertEqual(len(h.api.posts()), 1, "明确 401 后不重复采集")
        self.assertEqual({p["idem"] for p in h.api.posts()}, {"lr-" + MID})
        self.assertEqual({p["fixed_token"] for p in h.api.posts()}, {TEST_RECORDER_TOKEN})
        self.assertEqual({p["auth"] for p in h.api.posts()}, {None})
        self.assertTrue(h.records()[0]["outcome"].startswith("not_accepted"))

    def test_07_manual_confirmation_stops_without_force(self):
        h = self.h
        h.api.capture_script = [(409, {"code": 409, "message": "无法核实上次派发结果",
                                       "success": False,
                                       "data": {"workflowId": "wf-9",
                                                "status": "needs_manual_confirmation",
                                                "needsManualConfirmation": True}})]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(20)
        posts = h.api.posts()
        self.assertEqual(len(posts), 1, "人工确认不许自动重试")
        self.assertNotIn("forceRedispatch", posts[0]["body"])
        self.assertIn("需人工确认", h.text())
        self.assertIn("人工核实后处理", h.text())
        self.assertEqual(h.records()[0]["outcome"], "manual_confirmation")

    def test_08_busy_retries_while_present_and_gives_up_after_leave(self):
        h = self.h
        h.api.capture_script = [
            (409, {"code": 409, "message": "Worker busy", "success": False, "data": None}),
            accepted_body(),
        ]
        h.api.progress_script = [progress("waiting")]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(10)
        posts = h.api.posts()
        self.assertEqual(len(posts), 2)
        self.assertEqual({p["idem"] for p in posts}, {"lr-" + MID})
        self.assertIn("双雷达正忙，重试中", h.text())

        # 一直 busy + 车离场 -> 放弃
        h2 = Harness()
        self.addCleanup(h2.close)
        h2.api.capture_script = [(409, {"code": 409, "message": "Worker busy",
                                        "success": False, "data": None})]
        h2.session.measured(h2.clock[0], MID, PLATE, channel_state())
        h2.advance(6)
        before = len(h2.api.posts())
        h2.leave()
        h2.advance(6)
        self.assertTrue(h2.records()[0]["outcome"].startswith("not_accepted"))
        self.assertLessEqual(len(h2.api.posts()), before + 1)

    def test_09_gate_503_is_not_retried(self):
        h = self.h
        h.api.capture_script = [(503, {"code": 503, "success": False,
                                       "message": "采集安全门禁未满足：acquisition_unverified 完整场尚未确认",
                                       "data": None})]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(20)
        self.assertEqual(len(h.api.posts()), 1, "503 描述的是真实设备状态，不许重试")
        self.assertIn("采集未启动：完整场采集条件尚未确认", h.text())
        self.assertEqual(h.records()[0]["outcome"], "not_accepted:acquisition_unverified")

    def test_29_backend_login_lock_does_not_affect_recorder(self):
        h = self.h
        h.api.login_status = 423
        h.api.login_message = "账号因连续登录失败被临时锁定"
        h.api.capture_script = [accepted_body()]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        self.assertEqual(h.api.logins, 0)
        self.assertEqual(len(h.api.posts()), 1)
        self.assertEqual(h.api.posts()[0]["fixed_token"], TEST_RECORDER_TOKEN)
        self.assertEqual(h.session.disabled, "")


class TestLeaveAndMoving(unittest.TestCase):
    """用例 10、11、12、13、14、17、18、19。"""

    def setUp(self):
        self.h = Harness()
        self.addCleanup(self.h.close)

    def test_10_leave_while_waiting_cancels_once(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(6)
        h.leave()
        h.advance(4)
        self.assertEqual(len(h.api.cancels()), 1)
        h.advance(40)
        self.assertEqual(len(h.api.cancels()), 1, "取消只发一次")
        self.assertEqual(h.records()[0]["outcome"], "cancelled_left_before_capture")

    def test_11_leave_after_captured_does_not_cancel(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [
            progress("captured", capturedCameras=["CAM_004", "CAM_005"]),
            progress("saved", persistedCameras=["CAM_004", "CAM_005"], persistenceState="saved"),
            progress("completed", terminal=True, message="已入库", dimensions=stored_dimensions()),
        ]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(2)                                 # 只走到 captured
        h.leave()
        self.assertIn("点云已捕获，继续监控落盘", h.text())
        h.advance(20)
        self.assertEqual(h.api.cancels(), [], "已捕获的数据有效，取消反而丢数据（方案 D5）")
        self.assertEqual(h.records()[0]["outcome"], "saved_after_leave")

    def test_12_persistence_failed_after_leave_rings(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [
            progress("captured", capturedCameras=["CAM_004"]),
            progress("persistence_failed", terminal=True, message="两份 PCD 未落盘"),
        ]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(2)                                 # 只走到 captured
        h.leave()
        h.advance(10)
        self.assertEqual(h.api.cancels(), [])
        self.assertIn("落盘失败，本车双雷达数据丢失", h.text())
        self.assertTrue(h.rings, "落盘失败必须响铃")
        self.assertEqual(h.records()[0]["outcome"], "persistence_failed_after_leave")

    def test_13_moving_while_waiting_alarms_and_repeats_then_stops(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(4)
        h.session.moving(h.clock[0])
        self.assertIn("请让司机停车", h.text())
        self.assertEqual(len(h.rings), 1)
        h.advance(6, step=1.0)                       # 没重新停住 -> 每 5 秒重复
        self.assertEqual(len(h.rings), 2)
        h.session.stopped(h.clock[0])
        h.advance(12, step=1.0)
        self.assertEqual(len(h.rings), 2, "重新停住后不再重复报警")

    def test_13b_moving_after_captured_is_just_a_note(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("captured", capturedCameras=["CAM_004"])]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(4)
        h.session.moving(h.clock[0])
        self.assertIn("点云已捕获，等待落盘", h.text())
        self.assertEqual(h.rings, [])

    def test_17_unknown_for_35s_reposts_the_same_key(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("unknown")]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(40, step=2.0)
        posts = h.api.posts()
        self.assertEqual(len(posts), 2, "unknown 持续 35 秒后同键重投一次")
        self.assertEqual({p["idem"] for p in posts}, {"lr-" + MID})
        h.advance(20)
        self.assertLessEqual(len(h.api.posts()), 2, "只重投一次")

    def test_18_awaiting_database_warns_once_after_45s(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("analyzing", message="分析已完成，正在确认数据库记录")]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(50, step=2.0)
        self.assertIn("数据库记录超过 45 秒仍未出现", h.text())
        self.assertIn("不要重新发起采集", h.text())
        self.assertEqual(h.text().count("数据库记录超过 45 秒仍未出现"), 1, "只警告一次")

    def test_19_timeout_keeps_the_ledger_open(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("analyzing")]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(200, step=4.0)
        self.assertEqual(h.records()[0]["outcome"], "timeout")
        self.assertEqual(h.entry()["state"], "open", "超时保留流程编号，台账条目仍开着")
        self.assertIn("请到后台核查", h.text())

    def test_bonus_big_payload_warns_once(self):
        h = Harness(response_warn_bytes=10)
        self.addCleanup(h.close)
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting", message="x" * 200)]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(10)
        self.assertEqual(h.text().count("进度响应体"), 1)


class TestResume(unittest.TestCase):
    """用例 20 —— 启动时接力监控。"""

    def setUp(self):
        self.h = Harness()
        self.addCleanup(self.h.close)

    def test_20_resume_unfinished_workflow(self):
        h = self.h
        h.ledger.open_vehicle(PLATE, h.clock[0] - 300)
        h.ledger.mark_triggered(PLATE, MID, "lr-" + MID, h.clock[0] - 290)
        h.ledger.mark_accepted(PLATE, "wf-old")
        h.ledger.mark_progress(PLATE, "saved", False)
        h.api.progress_script = [progress("completed", terminal=True, message="已入库",
                                          dimensions=stored_dimensions())]
        h.session = h.new_session()
        self.assertTrue(h.session.resume())
        self.assertIn("接着监控上次未结束的流程 wf-old", h.text())
        h.advance(10)
        self.assertEqual(h.api.posts(), [], "接力监控不许重复受理")
        self.assertEqual(h.entry()["state"], "closed", "终局后条目置 closed")
        self.assertEqual(h.records()[0]["outcome"], "completed")

    def test_20b_nothing_to_resume(self):
        h = self.h
        self.assertFalse(h.session.resume())
        h.ledger.open_vehicle(PLATE, h.clock[0])      # 只有进场记录，没有流程
        self.assertFalse(h.session.resume())


class TestGates(unittest.TestCase):
    """用例 26、27 —— 两道门。"""

    def setUp(self):
        self.h = Harness()
        self.addCleanup(self.h.close)

    def _assert_skipped(self, state, expected):
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        before = len(h.out)
        h.session.measured(h.clock[0], MID, PLATE, state)
        h.advance(6)
        self.assertEqual(h.api.posts(), [], "闸门不放行时不许受理")
        self.assertEqual(h.records()[-1]["outcome"], expected)

    def test_26a_load_guard_fps(self):
        self._assert_skipped(channel_state(fps=6.9), "skipped_load_guard")
        self.assertIn("载货链负载：fps 6.9 < 7.5", self.h.text())
        self.assertIn("双雷达采集已跳过", self.h.text())

    def test_26b_load_guard_incomplete(self):
        self._assert_skipped(channel_state(incomplete=0.31), "skipped_load_guard")
        self.assertIn("残缺帧 31.0%", self.h.text())

    def test_26c_load_guard_packet_rate_zero(self):
        self._assert_skipped(channel_state(fps=9.0, rate=0), "skipped_load_guard")
        self.assertIn("packet_rate 0", self.h.text())

    def test_26d_guard_can_be_disabled(self):
        h = Harness(guard_enabled=False)
        self.addCleanup(h.close)
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        h.session.measured(h.clock[0], MID, PLATE, channel_state(fps=1.0))
        h.advance(6)
        self.assertEqual(len(h.api.posts()), 1)

    def test_26e_guard_skips_only_one_line(self):
        h = self.h
        h.api.capture_script = [accepted_body()]
        before = len(h.out)
        h.session.measured(h.clock[0], MID, PLATE, channel_state(fps=6.0))
        h.advance(30)
        added = [line for line in h.out[before:] if "跳过" in line]
        self.assertEqual(len(added), 1, "跳过只打一行，不报警、不重试")

    def test_27_status_unavailable_skips(self):
        h = self.h
        h.api.status_body = {"code": 200, "message": "success", "success": True,
                             "data": {"scan": {"state": "idle"}, "preview": {"available": False}}}
        self._assert_skipped(channel_state(), "skipped_dual_unavailable")
        self.assertIn("双雷达状态不可用", h.text())

    def test_27b_status_request_fails_skips(self):
        h = self.h

        def dead():
            raise urllib.error.URLError("timed out")

        h.client.get_status = dead
        self._assert_skipped(channel_state(), "skipped_dual_unavailable")

    def test_27c_status_busy_still_posts(self):
        h = self.h
        h.api.status_body = {"code": 200, "message": "success", "success": True,
                             "data": {"scan": {"state": "scanning"}, "preview": {"available": True}}}
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(6)
        self.assertEqual(len(h.api.posts()), 1, "正在会话 -> 交给 §5 的 busy 逻辑")


class TestPlateValidation(unittest.TestCase):
    """用例 15 —— 车牌校验。"""

    def test_15_invalid_plates_do_not_trigger(self):
        cases = ["未知车牌", "未知", "无车牌", "unknown", "N/A", "  ", "", None,
                 "X" * 33, "宁A\x01123"]
        for raw in cases:
            h = Harness()
            try:
                h.api.capture_script = [accepted_body()]
                h.session.measured(h.clock[0], MID, raw, channel_state())
                h.advance(4)
                self.assertEqual(h.api.posts(), [], "车牌 %r 不该触发" % (raw,))
                self.assertIn("未触发双雷达采集", h.text())
            finally:
                h.close()

    def test_15b_valid_plate_is_uppercased_and_whitespace_stripped(self):
        h = Harness()
        self.addCleanup(h.close)
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        h.session.measured(h.clock[0], MID, "  ning-a123  ", channel_state())
        h.advance(4)
        self.assertEqual(h.api.posts()[0]["body"]["licensePlate"], "NING-A123")

    def test_15c_inner_whitespace_is_stripped_like_the_server(self):
        """接口文档 §3：服务端会**去掉全部空白**（含中间空格）后转大写。"""
        self.assertEqual(drc.normalize_plate("宁A123 45"), "宁A25771")
        self.assertEqual(drc.normalize_plate("宁A\t123"), "宁A123")


class TestRecorderWiring(unittest.TestCase):
    """用例 14、16、21、30 —— 记录器那一侧（直接用假快照驱动，不起线程）。"""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="dual-radar-rec-")
        self.token_file = os.path.join(self.tmp, "recorder.json")
        with open(self.token_file, "w", encoding="utf-8") as stream:
            json.dump({"recorderToken": TEST_RECORDER_TOKEN}, stream)
        self.out = []
        self.seen = {"save_check": False}
        self.state = {}

    def args(self, **kw):
        base = dict(url="http://127.0.0.1:1/api/state", out="", enter=15000, leave_ratio=0.5,
                    capture_token_file=getattr(self, "token_file", drc.DEFAULT_TOKEN_FILE),
                    leave_frames=2, interval=2.0, capture_webapi="http://127.0.0.1:5031",
                    capture_channel="CH4", no_capture=False, no_bell=False, ledger_ttl=1800.0,
                    guard_fps=7.5, guard_incomplete=0.10, no_load_guard=False, capture_poll=2.0)
        base.update(kw)
        import argparse
        return argparse.Namespace(**base)

    def feed(self, state_dict, args=None, session=None, ledger=None, state_key=None, say=None):
        class _Resp:
            def __init__(self, payload):
                self._raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")

            def read(self):
                return self._raw

            def __enter__(self):
                return self

            def __exit__(self, *exc):
                return False

        return recorder._poll_once("http://x/api/state", self.state, args or self.args(),
                                  ledger, say or self.out.append, session, self.seen,
                                  opener=lambda url, timeout=3: _Resp(state_dict))

    @staticmethod
    def snapshot(n=20000, state=1, mid=None, session=7, plate=PLATE, last=None):
        ch = {"id": "CH4", "session": session, "fps": 9.5, "incomplete_ratio": 0.0,
              "source": {"packet_rate": 8800.0},
              "stability": {"state": state, "n_vehicle_points": n, "stationary_sec": 3.0,
                            "degraded": False, "triggers": [], "state_cn": "已停稳"},
              "last_result": last if last is not None else ({"plate": plate, "measurement_id": mid}
                                                            if mid else {}),
              "last_plate": plate, "last_reason": "-", "saved_count": 0}
        if mid:
            ch["current_result"] = {"measurement_id": mid}
        return {"channels": [ch]}

    class StubSession:
        """记录器这一侧看到的调用。

        ★ 方法名**必须**是 `on_*`（方案 §9.2 第 4 条）：管理器只暴露 `on_*`，
          写成 `measured()` 会 AttributeError，而它会被本通道的异常兜底吞掉 ——
          双雷达采集一次都不会触发、屏幕上只有一行"本通道本次采样处理出错"。
          所以这里**刻意用与 `CaptureManager` 相同的方法名**，让签名对上。
        """

        def __init__(self):
            self.calls = []

        def on_entered(self, *a):
            self.calls.append(("entered",))

        def on_stable(self, *a):
            self.calls.append(("stable",))

        def on_measured(self, *a):
            self.calls.append(("measured", a[1]))

        def on_moving(self, *a):
            self.calls.append(("moving",))

        def on_stopped(self, *a):
            self.calls.append(("stopped",))

        def on_left(self, *a):
            self.calls.append(("left",))

    def test_14_session_change_is_not_moving(self):
        stub = self.StubSession()
        self.feed(self.snapshot(state=1), session=stub)
        self.feed(self.snapshot(state=2), session=stub)
        self.feed(self.snapshot(state=2, mid=MID), session=stub)
        self.feed(self.snapshot(state=1, mid=MID), session=stub)   # 2 -> 1，会话没变 = 开始移动
        self.assertEqual(stub.calls, [("entered",), ("stable",), ("measured", MID), ("moving",)])

        stub2 = self.StubSession()
        self.state = {}
        self.feed(self.snapshot(state=1, mid=MID, session=7), session=stub2)
        self.feed(self.snapshot(state=2, mid=MID, session=7), session=stub2)
        stub2.calls = []
        self.feed(self.snapshot(state=1, mid=MID, session=8), session=stub2)   # 会话变了
        self.assertEqual(stub2.calls, [], "会话号变化不算开始移动（180 秒超时重开会话）")

    def test_16_only_ch4_triggers_and_bad_channel_exits(self):
        stub = self.StubSession()
        snapshot = self.snapshot(state=2, mid=MID)
        snapshot["channels"][0]["id"] = "CH3"
        self.feed(snapshot, session=stub)
        self.assertEqual(stub.calls, [], "非 CH4 车道不许触发")
        # 传别的车道必须**启动即报错退出**（fail-fast，避免误配后静默无效）
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        proc = subprocess.run(
            [sys.executable, os.path.join("tools", "field_standstill_log.py"),
             "--capture-channel", "CH3"],
            cwd=root, capture_output=True, timeout=60)
        self.assertEqual(proc.returncode, 2)
        self.assertIn("只接受 CH4", proc.stdout.decode("utf-8", errors="replace")
                      + proc.stderr.decode("utf-8", errors="replace"))

    def test_21_without_account_credentials_capture_uses_private_token(self):
        saved = {k: os.environ.pop(k, None)
                 for k in ("LOAD_RADAR_CAPTURE_USER", "LOAD_RADAR_CAPTURE_PASSWORD")}
        try:
            session, ledger = recorder._build_capture(self.args(), self.out.append,
                                                     threading.RLock())
            self.assertIsNotNone(session)
            self.assertIsNotNone(ledger)
            self.assertFalse(hasattr(session.session.client, "login"))
        finally:
            for key, value in saved.items():
                if value is not None:
                    os.environ[key] = value

    def test_21b_no_capture_turns_it_off(self):
        session, ledger = recorder._build_capture(
            self.args(no_capture=True, capture_channel=""),
            self.out.append, threading.RLock())
        self.assertIsNone(session)
        self.assertIsNone(ledger)

    def test_30_save_selfcheck_warns_once(self):
        stub = self.StubSession()
        self.feed(self.snapshot(last={"plate": PLATE}), session=stub)
        self.assertIn("▲▲ 测量服务未产生测量号", "\n".join(self.out))
        before = len(self.out)
        self.feed(self.snapshot(last={"plate": PLATE}), session=stub)
        self.assertEqual([line for line in self.out[before:] if "▲▲" in line], [],
                         "启动自检只喊一次")

    def test_30b_selfcheck_helper(self):
        self.assertFalse(drc.needs_save_selfcheck({}))
        self.assertFalse(drc.needs_save_selfcheck(None))
        self.assertFalse(drc.needs_save_selfcheck({"measurement_id": "m-1"}))
        self.assertTrue(drc.needs_save_selfcheck({"plate": PLATE}))

    def test_leave_line_carries_the_dual_conclusion(self):
        ledger = drc.VehicleLedger(os.path.join(self.tmp, "台账.json"), threading.RLock())
        now = time.time()
        ledger.open_vehicle(PLATE, now)
        ledger.mark_triggered(PLATE, MID, "lr-" + MID, now)
        ledger.mark_progress(PLATE, "saved", False)
        stub = self.StubSession()
        self.feed(self.snapshot(state=1), session=stub, ledger=ledger)
        self.feed(self.snapshot(state=2), session=stub, ledger=ledger)
        self.feed(self.snapshot(state=2, mid=MID), session=stub, ledger=ledger)
        self.feed(self.snapshot(n=100, state=1), session=stub, ledger=ledger)  # 第 1 次低于门槛
        self.feed(self.snapshot(n=100, state=1), session=stub, ledger=ledger)  # 第 2 次 -> 离场
        text = "\n".join(self.out)
        self.assertIn("车离场", text)
        self.assertIn("双雷达：√ 已落盘", text)
        self.assertEqual(ledger.open_entry(PLATE, time.time()), None, "离场后条目应已 closed")
        self.assertIn(("left",), stub.calls)

    def test_32_real_manager_accepts_every_recorder_call(self):
        """★ 回归守卫（方案 §9.2 第 4 条）——**别用 Stub 代替这一步**。

        记录器调的是管理器的 `on_*`。方法名一旦写错（比如写成 `session.measured()`），
        抛出的 `AttributeError` 会被本通道的异常兜底吞成一行
        "本通道本次采样处理出错，已跳过" —— **双雷达采集一次都不会触发**，
        而现场看到的只是屏幕上多一行小字。所以这里用**真的 `CaptureManager`** 走一遍。
        """
        h = Harness()
        self.addCleanup(h.close)
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        manager = drc.CaptureManager(h.client, h.ledger, h.out.append, h.cfg)
        manager.start()
        self.addCleanup(manager.close)

        for snapshot in (self.snapshot(state=1), self.snapshot(state=2),
                         self.snapshot(state=2, mid=MID)):
            self.feed(snapshot, session=manager, ledger=h.ledger, say=h.out.append)

        deadline = time.time() + 10
        while time.time() < deadline and not h.api.posts():
            time.sleep(0.05)
        text = "\n".join(h.out) + "\n" + "\n".join(self.out)
        self.assertNotIn("本通道本次采样处理出错", text)
        self.assertTrue(h.api.posts(), "记录器把 on_* 调对了才可能真的发出受理请求")

    def test_33_terminal_record_carries_all_vehicle_times(self):
        """方案 §8.2：`vehicle_times` 五个键齐全，`accept_status` 要落盘。"""
        h = Harness()
        self.addCleanup(h.close)
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("completed", terminal=True,
                                          dimensions=stored_dimensions(),
                                          analysis=LENGTH_MEASUREMENT)]
        # ★ 顺序要紧：moving / stopped / left 都必须发生在**终局之前**
        #   （终局那一行写完，后面再来的事件就进不了这份记录了 —— 这本身是对的）
        h.session.entered(1000.0)
        h.session.stable(1001.0)
        h.session.measured(1002.0, MID, PLATE, channel_state())   # 内部同步发出受理
        h.session.moving(1003.0)
        h.session.stopped(1004.0)
        h.leave()
        h.advance(6)

        rows = h.records()
        self.assertEqual(len(rows), 1)
        times = rows[0]["vehicle_times"]
        self.assertEqual(sorted(times), sorted(drc.VEHICLE_TIME_KEYS),
                         "五个键必须齐全（没有的写 null，不是省略）")
        for key in ("entered", "stable", "measured", "moving", "left"):
            self.assertIsNotNone(times[key], "%s 没被记下来" % key)
        self.assertEqual(times["entered"], 1000.0)
        self.assertEqual(times["stable"], 1001.0)
        self.assertEqual(times["measured"], 1002.0)
        self.assertEqual(rows[0]["accept_status"], "accepted")


class TestSourceGbkSafe(unittest.TestCase):
    """用例 22 —— 源文件必须能被 GBK 编码（中文 Windows 控制台）。"""

    def test_22_modified_sources_are_gbk_encodable(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for rel in ("tools/dual_radar_capture.py", "tools/field_standstill_log.py",
                    "tools/field_watchdog.py", "tests/test_dual_radar_capture.py"):
            path = os.path.join(root, rel)
            with open(path, encoding="utf-8") as fh:
                text = fh.read()
            text.encode("gbk")           # 编不出会抛 UnicodeEncodeError，测试即失败


class TestClient(unittest.TestCase):
    """客户端自身的两条硬要求：绕开代理、响应体大小可读。"""

    def setUp(self):
        self.h = Harness()
        self.addCleanup(self.h.close)

    def test_client_bypasses_system_proxy(self):
        os.environ["HTTP_PROXY"] = "http://127.0.0.1:14593"
        os.environ["http_proxy"] = "http://127.0.0.1:14593"
        try:
            self.h.api.status_body = self.h.api.status_body
            status, _ = self.h.client.get_status()
            self.assertEqual(status, 200, "回环地址不能走系统代理（会被返回 502）")
        finally:
            os.environ.pop("HTTP_PROXY", None)
            os.environ.pop("http_proxy", None)

    def test_client_records_response_size(self):
        self.h.client.get_status()
        self.assertGreater(self.h.client.last_size, 0)


class TestPureFunctions(unittest.TestCase):
    """纯函数的边界（也是 §2.4 / §6.1 的逐项对照）。"""

    def test_stage_bar_covers_every_stage(self):
        self.assertIn("○等待安全完整场", drc.render_steps(None))
        self.assertIn("×PCD 落盘失败", drc.render_steps({"stage": "persistence_failed"}))
        self.assertIn("×采集失败", drc.render_steps({"stage": "capture_failed"}))
        self.assertIn("×分析失败", drc.render_steps({"stage": "analysis_failed"}))
        self.assertIn("×分析完成，本次不产出尺寸",
                      drc.render_steps({"stage": "completed_without_dimensions"}))
        self.assertIn("◎待人工审核（未入库）", drc.render_steps({"stage": "pending_review"}))
        self.assertIn("融合与计算完成", drc.render_steps({"stage": "pending_review"}))
        self.assertIn("◎已入库，待人工审核",
                      drc.render_steps({"stage": "completed",
                                        "dimensions": {"reviewStatus": "pending"}}))
        bar = drc.render_steps({"stage": "captured", "capturedCameras": ["CAM_004", "CAM_005"]})
        self.assertIn("双雷达点云已捕获（CAM_004、CAM_005）", bar)
        self.assertNotIn("（CAM", drc.render_steps({"stage": "saved",
                                                   "capturedCameras": ["CAM_004"]}))

    def test_rib_count_is_segments_minus_one(self):
        fields = drc.collect_result_fields({
            "stage": "completed",
            "dimensions": {"reviewStatus": "approved", "recordId": 1,
                           "dimensions": {"trailerLength": 100.0,
                                          "ribs": [{"distToPrev": None},
                                                   {"distToPrev": 374.8},
                                                   {"distToPrev": 378.6}]}}})
        self.assertEqual(fields["ribCount"], 2)
        self.assertEqual(fields["ribCountText"], "2 根")
        self.assertEqual(fields["ribGaps"][0], "缺测")           # null 不是 0（页面 formatRibGap）
        self.assertEqual(fields["ribGaps"][1], "374.8 cm")

    def test_rib_count_out_of_range_is_flagged(self):
        self.assertEqual(drc.describe_rib_count(7), "7 根 ▲")
        self.assertEqual(drc.describe_rib_count(None), "未确认")

    def test_missing_rib_gap_is_not_zero(self):
        self.assertEqual(drc.format_rib_gap(None, drc.format_cm), "缺测")
        self.assertEqual(drc.format_rib_gap(0, drc.format_cm), "0.0 cm")

    def test_review_branch_needs_the_compound_condition(self):
        """分析失败时不许把上一轮数据渲染成有尺寸结果。"""
        progress_failed = {"stage": "pending_review",
                           "analysis": {"stage": "failed",
                                        "measurementStatus": {"reviewStatus": "pending"},
                                        "reviewMeasurements": {"trailerBottomToGroundHeight": 1.5}},
                           "dimensions": None}
        self.assertIsNone(drc.collect_result_fields(progress_failed))
        # 只有 stage/analysis 都对上才走待审核分支
        ok = {"stage": "pending_review",
              "analysis": {"stage": "completed",
                           "measurementStatus": {"reviewStatus": "pending"},
                           "reviewMeasurements": {"dimensions": {"carriage": {"length": 11.3}}}},
              "dimensions": None}
        fields = drc.collect_result_fields(ok)
        self.assertTrue(fields["pending"])
        self.assertEqual(fields["innerLength"], "11.300 m", "待审核值用米，不与厘米混排")

    def test_length_note_requires_valid_definition(self):
        good = {"analysis": LENGTH_MEASUREMENT}
        self.assertIn("底板以上 50.0～100.0 cm", drc.length_measurement_note(good))
        bad = {"analysis": {"stage": "completed",
                            "measurementStatus": {"lengthMeasurement": {
                                "valid": True, "definition": "其它口径",
                                "heightAboveFloorM": [0.5, 1.0], "inwardBandM": [0.05, 0.35]}}}}
        self.assertEqual(drc.length_measurement_note(bad), "")
        self.assertEqual(drc.length_measurement_note({}), "")

    def test_measurement_range_rejects_bad_input(self):
        self.assertEqual(drc.measurement_range_cm([0.5, 1.0]), "50.0～100.0")
        self.assertEqual(drc.measurement_range_cm([1.0, 0.5]), "")
        self.assertEqual(drc.measurement_range_cm([0.5]), "")
        self.assertEqual(drc.measurement_range_cm([0.5, 11.0]), "")

    def test_idempotency_key_format(self):
        self.assertTrue(drc.key_is_valid(drc.idempotency_key(MID)))
        self.assertFalse(drc.key_is_valid("a" * 65))
        self.assertFalse(drc.key_is_valid("带中文"))

    def test_leave_action_table(self):
        self.assertEqual(drc.leave_action(None, False), ("never", None))
        self.assertEqual(drc.leave_action(None, True), ("cancel", None))
        self.assertEqual(drc.leave_action({"stage": "waiting"}, True), ("cancel", "waiting"))
        self.assertEqual(drc.leave_action({"stage": "unknown"}, True), ("cancel", "unknown"))
        self.assertEqual(drc.leave_action({"stage": "captured"}, True), ("watch", "captured"))
        self.assertEqual(drc.leave_action({"stage": "saved"}, True), ("watch", "saved"))
        self.assertEqual(drc.leave_action({"stage": "completed", "terminal": True}, True),
                         ("done", "completed"))

    def test_dual_status_table(self):
        self.assertEqual(drc.dual_status(None), "unavailable")
        self.assertEqual(drc.dual_status({"data": {"scan": {"state": "scanning"}}}), "busy")
        self.assertEqual(drc.dual_status({"data": {"preview": {"available": False}}}),
                         "unavailable")
        self.assertEqual(drc.dual_status({"data": {"scan": {"state": "idle"},
                                                   "preview": {"available": True}}}), "ok")
        self.assertEqual(drc.dual_status({"data": {}}), "ok", "读不到就放行，别自己吓自己")

    def test_config_rejects_unknown_keys(self):
        with self.assertRaises(AttributeError):
            drc.CaptureConfig(nonsense=1)
        self.assertTrue(drc.CaptureConfig().ledger_path().endswith(drc.LEDGER_NAME))


class TestRecorderTokenConfiguration(unittest.TestCase):
    """配置缺失时只停用采集；不联网，也不泄露令牌内容。"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="recorder-token-")
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "recorder.json")

    def write_config(self, value):
        with open(self.path, "w", encoding="utf-8") as stream:
            json.dump(value, stream)

    def test_valid_private_file_is_loaded(self):
        for token in (TEST_RECORDER_TOKEN, "a" * 32, "b" * 256):
            self.write_config({"recorderToken": token})
            self.assertEqual(drc.load_recorder_token(self.path), token)

    def test_missing_blank_invalid_or_oversized_config_is_rejected(self):
        with self.assertRaises(ValueError):
            drc.WebApiClient("http://127.0.0.1:1", drc.CaptureConfig(token_file=self.path))
        for value in ({}, [], {"recorderToken": None}, {"recorderToken": 123},
                      {"recorderToken": ""}, {"recorderToken": "short"},
                      {"recorderToken": TEST_RECORDER_TOKEN + " "},
                      {"recorderToken": TEST_RECORDER_TOKEN + "\n"},
                      {"recorderToken": TEST_RECORDER_TOKEN + "中文"},
                      {"recorderToken": "a" * 257}, {"extra": "a" * 4097}):
            with self.subTest(value_type=type(value).__name__):
                self.write_config(value)
                with self.assertRaises(ValueError) as raised:
                    drc.load_recorder_token(self.path)
                self.assertNotIn(TEST_RECORDER_TOKEN, str(raised.exception))

    def test_malformed_json_error_does_not_include_content(self):
        with open(self.path, "w", encoding="utf-8") as stream:
            stream.write(TEST_RECORDER_TOKEN)
        with self.assertRaises(ValueError) as raised:
            drc.load_recorder_token(self.path)
        self.assertNotIn(TEST_RECORDER_TOKEN, str(raised.exception))

    def test_missing_token_disables_capture_but_not_recorder(self):
        args = recorder.build_parser().parse_args(["--capture-channel", "CH4", "--capture-token-file", self.path])
        output = []
        manager, ledger = recorder._build_capture(args, output.append, threading.RLock())
        self.assertIsNone(manager)
        self.assertIsNone(ledger)
        self.assertIn("只做停稳记录", "\n".join(output))

    def test_watchdog_only_forwards_private_file_path(self):
        args = TestWatchdogPassthrough().wargs(capture_token_file=self.path)
        argv = watchdog._capture_argv(args)
        self.assertEqual(argv, ["--capture-token-file", self.path])
        self.assertEqual(recorder.build_parser().parse_args(argv).capture_token_file, self.path)

    def test_recorder_reads_light_snapshot_with_every_field_it_uses(self):
        """记录器默认只要轻量快照（不带最近结果）；它读的每个通道字段在轻量快照里都必须还在。"""
        self.assertTrue(recorder.build_parser().parse_args([]).url.endswith("/api/state?history=0"))
        from app.service import ChannelWorker, EventBus
        from app.settings import load_settings
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        worker = ChannelWorker(ch, EventBus(), st.plate, save_override=False, plate_mode="null")
        light = worker.snapshot(include_history=False)
        used = {"id", "frames", "stability", "session", "saved_count", "last_reason", "last_result", "current_result"}
        self.assertFalse(used - set(light), "轻量快照缺了记录器要读的字段")
        self.assertNotIn("history", light)

    def test_client_reloads_rotated_token_when_recreated(self):
        self.write_config({"recorderToken": TEST_RECORDER_TOKEN})
        cfg = drc.CaptureConfig(token_file=self.path)
        first = drc.WebApiClient("http://127.0.0.1:1", cfg)
        rotated = "rotated-test-only-recorder-token-0123456789"
        self.write_config({"recorderToken": rotated})
        second = drc.WebApiClient("http://127.0.0.1:1", cfg)
        self.assertEqual(first._recorder_token, TEST_RECORDER_TOKEN)
        self.assertEqual(second._recorder_token, rotated)

    def test_redirect_does_not_forward_token(self):
        h = Harness()
        self.addCleanup(h.close)

        class RedirectHandler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                self.send_response(302)
                self.send_header("Location", h.base + "/api/v1/radar-live/status")
                self.end_headers()

        server = ThreadingHTTPServer(("127.0.0.1", 0), RedirectHandler)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        client = drc.WebApiClient("http://127.0.0.1:%s" % server.server_port, h.cfg)
        status, _ = client.get_status()
        self.assertEqual(status, 302)
        self.assertEqual(h.api.requests, [])


class TestWatchdogPassthrough(unittest.TestCase):
    """方案 D8 / §9.3 —— 看门狗透传给停稳记录器的那段 argv 必须被记录器原样接受。

    ▲ 这条链只有到现场才会暴露：看门狗拼的 argv 若与记录器的 argparse 对不上，
      记录器会**启动即报错退出**，而看门狗只会看到"进程没了又起"、
      于是**每几秒重启一次**，双雷达采集从头到尾一次都不会触发。
    """

    def wargs(self, **kw):
        base = dict(no_capture=False, capture_webapi="", capture_channel="",
                    no_bell=False, no_load_guard=False, guard_fps=None,
                    guard_incomplete=None, ledger_ttl=None, capture_poll=None)
        base.update(kw)
        import argparse
        return argparse.Namespace(**base)

    def test_31a_defaults_pass_nothing_through(self):
        """全不给 = 不干预，记录器用自己的默认值（默认就是启用 CH4 采集）。"""
        self.assertEqual(watchdog._capture_argv(self.wargs()), [])

    def test_31b_only_the_given_params_are_passed(self):
        argv = watchdog._capture_argv(self.wargs(
            capture_webapi="http://127.0.0.1:5031", capture_channel="CH4",
            guard_fps=6.5, capture_poll=3.0))
        self.assertEqual(argv, ["--capture-webapi", "http://127.0.0.1:5031",
                                "--capture-channel", "CH4",
                                "--guard-fps", "6.5", "--capture-poll", "3.0"])

    def test_31c_flags_are_passed_as_flags(self):
        argv = watchdog._capture_argv(self.wargs(
            no_capture=True, no_bell=True, no_load_guard=True, ledger_ttl=600.0))
        self.assertEqual(argv, ["--no-capture", "--no-bell", "--no-load-guard",
                                "--ledger-ttl", "600.0"])

    def test_31d_never_reuses_the_webapi_name(self):
        """`--webapi` 是看门狗自己的"也看管 WebApi 进程"，同名不同义，禁止复用（D8）。"""
        argv = watchdog._capture_argv(self.wargs(
            capture_webapi="http://127.0.0.1:5031", capture_channel="CH4"))
        self.assertNotIn("--webapi", argv)

    def test_31e_passthrough_argv_is_accepted_by_the_recorder(self):
        """★ 契约：看门狗的输出直接喂给记录器的解析器，必须能解出来且值对得上。"""
        argv = watchdog._capture_argv(self.wargs(
            capture_webapi="http://127.0.0.1:5031", capture_channel="CH4",
            no_bell=True, guard_fps=6.5, guard_incomplete=0.2,
            ledger_ttl=600.0, capture_poll=3.0))
        a = recorder.build_parser().parse_args(argv)
        self.assertEqual(a.capture_webapi, "http://127.0.0.1:5031")
        self.assertEqual(a.capture_channel, "CH4")
        self.assertTrue(a.no_bell)
        self.assertEqual(a.guard_fps, 6.5)
        self.assertEqual(a.guard_incomplete, 0.2)
        self.assertEqual(a.ledger_ttl, 600.0)
        self.assertEqual(a.capture_poll, 3.0)
        # 不给的项落到记录器自己的默认值上（= 不改行为）
        self.assertFalse(a.no_capture)
        self.assertEqual(a.guard_fps, 6.5)
        b = recorder.build_parser().parse_args([])
        # 2026-11-02 起默认关闭，CH4 由服务的 passages/standstill 通知触发
        self.assertEqual(b.capture_channel, "")
        self.assertEqual(b.capture_webapi, drc.DEFAULT_CAPTURE_WEBAPI)
        self.assertEqual(b.capture_poll, drc.DEFAULT_POLL_SEC)
        self.assertTrue(b.no_load_guard is False)


class TestManagerThread(unittest.TestCase):
    """后台线程能起能收（事件入队、不阻塞主线程）。"""

    def test_manager_starts_and_closes_cleanly(self):
        h = Harness()
        self.addCleanup(h.close)
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]      # 停在跑，别终局
        manager = drc.CaptureManager(h.client, h.ledger, h.out.append, h.cfg)
        manager.start()
        # ★ 时间戳必须是**真实墙上时钟**：管理器把它原样写进台账的 opened_at，
        #   传 1.0 会被 TTL 扫描当成"1970 年开的条目"立刻关掉，断言就假失败了。
        h.ledger.open_vehicle(PLATE, time.time())      # 记录器那一侧的"进场"由它写
        manager.on_entered(time.time())
        manager.on_measured(time.time(), MID, PLATE, channel_state())
        self.assertTrue(manager.is_alive())
        deadline = time.time() + 10
        while time.time() < deadline and not h.api.posts():
            time.sleep(0.05)
        self.assertTrue(h.api.posts(), "后台线程必须真的把受理请求发出去")
        manager.close(timeout=10)
        self.assertFalse(manager.is_alive())
        # ★ close() 是"Ctrl-C 退出"语义（方案 §9.1）：进行中的作业**不取消**、台账条目保留，
        #   下次启动接着盯。所以这里断言的是"没发过取消请求 + 台账条目仍开着"，
        #   而**不是** idle()——线程刚好在处理作业时被叫停，job 停在手里才是对的
        #   （曾经写成 assertEqual(idle(), True)，是个会随调度抖动的假断言）。
        self.assertEqual(h.api.cancels(), [], "close() 不许取消进行中的作业")
        entries = h.ledger.resumable(time.time())
        self.assertTrue(entries, "已触发过的车次，台账条目必须留给下一班接力")
        self.assertEqual(entries[0]["state"], "open")
        self.assertTrue(entries[0]["opened_at"] > 0,
                        "触发过的条目 opened_at 必须有效，否则会被 TTL 扫描作废")
        self.assertEqual(entries[0]["workflow_id"], "wf-1")


class _Resp:
    """给记录器 `_poll_once` 喂快照用的假响应。"""

    def __init__(self, payload):
        self._raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")

    def read(self):
        return self._raw

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class TestAuditFixes20260925(unittest.TestCase):
    """2026-09-25 对照 `docs/68` v2 与 WebApi / Worker 源码逐项审查查出的问题，每条一个回归用例。

    每条都写明"修复前是什么样"。WebApi / Worker 侧的事实（模拟车牌 60 秒轮换、
    取消后终局是 capture_failed、Worker 可受理状态有 idle 与 ready）都是对着源码核实过的。
    """

    def setUp(self):
        self.h = Harness()
        self.addCleanup(self.h.close)

    # ── 1. WebApi 暂时连不上不应停用本班次 ──────────────────────────
    def test_a1_webapi_unreachable_at_status_does_not_disable_the_shift(self):
        """状态接口暂不可用时只跳过本车，恢复后下一辆仍可触发。"""
        h = self.h
        h.api.status_status = 503
        h.session.entered(h.clock[0])
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        self.assertEqual(h.api.posts(), [])
        self.assertEqual(h.session.disabled, "", "WebApi 暂时连不上不许停用整个班次")
        self.assertEqual(h.records()[-1]["outcome"], "skipped_dual_unavailable")
        self.assertIn("双雷达状态不可用", h.text())
        # WebApi 恢复后，下一辆车照常触发
        h.api.status_status = 200
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        h.session.entered(h.clock[0])
        h.session.measured(h.clock[0], "m-next", "宁B54516", channel_state())
        self.assertEqual(len(h.api.posts()), 1)

    def test_a1b_connection_refused_returns_no_response(self):
        """连接被拒时返回 0，不尝试账号登录。"""
        client = drc.WebApiClient("http://127.0.0.1:1", self.h.cfg)
        self.assertEqual(client.get_status()[0], 0)

    def test_a1c_single_auth_rejection_skips_one_vehicle_three_disable(self):
        """固定 Token 被拒（401）：一次只放弃这一辆；连续 3 次才停用。"""
        h = self.h
        h.api.status_status = 401
        for index in range(2):
            h.session.entered(h.clock[0])
            h.session.measured(h.clock[0], "m-%d" % index, "宁B0000%d" % index, channel_state())
            self.assertEqual(h.session.disabled, "", "第 %d 次认证失败还不该停用" % (index + 1))
        h.session.entered(h.clock[0])
        h.session.measured(h.clock[0], "m-3", "宁B91952", channel_state())
        self.assertTrue(h.session.disabled, "连续 3 次认证失败才停用本班次")
        requests = len(h.api.requests)
        h.session.entered(h.clock[0])
        h.session.measured(h.clock[0], "m-4", "宁B47424", channel_state())
        self.assertEqual(len(h.api.requests), requests, "停用后不再发送采集请求")
        self.assertIn("双雷达采集已停用", h.text())

    # ── 2. 查不到进度也要看总时限 ─────────────────────────────────────
    def test_a2_progress_failures_still_hit_the_total_timeout(self):
        """修复前：进度接口一直失败时 `_poll` 早退、从不看总时限 -> 作业永不收尾，
        排队的下一辆车永远发不出去。"""
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [(500, {"code": 500, "message": "内部错误", "data": None})]
        h.session.entered(h.clock[0])
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.session.left(h.clock[0])                    # 这辆车离场（watch 不到阶段 -> 取消 -> 继续查）
        h.session.entered(h.clock[0])                 # 下一辆车来了、测完 -> 排队
        h.session.measured(h.clock[0], "m-next", "宁B54516", channel_state())
        self.assertIsNotNone(h.session.pending)
        h.advance(240, step=4.0)
        outcomes = [row["outcome"] for row in h.records()]
        self.assertIn(outcomes[0], ("cancelled_left_before_capture", "timeout"))
        self.assertEqual(len(h.api.posts()), 2, "卡住的作业收尾后，排队的车必须发出去")

    def test_a2b_never_accepted_progress_failures_timeout(self):
        """一直停在场、进度接口一直失败：180 秒后按 timeout 收尾（台账保持 open）。"""
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [(502, {"code": 502, "message": "雷达服务不可用", "data": None})]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(200, step=4.0)
        self.assertEqual(h.records()[0]["outcome"], "timeout")
        self.assertEqual(h.entry()["state"], "open")

    # ── 3. 离场取消后的终局 ───────────────────────────────────────────
    def test_a3_cancelled_capture_is_recorded_as_cancelled(self):
        """修复前：WebApi 把"采集已取消"映射成 capture_failed（终局），工具记成
        `failed:capture_failed` 并打一行"× 采集失败" —— 方案 §8.2 的
        `cancelled_left_before_capture` 在真实环境里永远出不来。"""
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting"),
                                 progress("capture_failed", terminal=True, message="采集已取消")]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(2)
        h.leave()
        h.advance(6)
        self.assertEqual(len(h.api.cancels()), 1)
        self.assertEqual(h.records()[0]["outcome"], "cancelled_left_before_capture")
        self.assertIn("本次采集已取消", h.text())
        self.assertNotIn("× 双雷达：采集失败", h.text())
        self.assertEqual(h.entry()["last_stage"], "cancelled")

    def test_a3b_cancel_409_keeps_polling_for_the_real_end(self):
        """修复前：取消返回 409 就当场记"已取消" —— 而 409 多半是这次采集刚好已经收完
        （进度落后 2 秒）。方案 §7.2：200/409 都再轮询拿终局，以终局为准。"""
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.cancel_status = 409
        h.api.cancel_message = "会话不属于该流程"
        h.api.progress_script = [
            progress("waiting"),
            progress("saved", persistedCameras=["CAM_004", "CAM_005"], persistenceState="saved"),
            progress("completed", terminal=True, dimensions=stored_dimensions()),
        ]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(2)
        h.leave()
        h.advance(10)
        self.assertEqual(h.records()[0]["outcome"], "saved_after_leave",
                         "取消没生效、数据其实已落盘入库 -> 按终局记")

    # ── 4. 车次：排队的车离场、下一辆车的事件不串到上一辆的作业上 ───────
    def test_a4_queued_vehicle_that_left_is_never_posted(self):
        """修复前：排队项不认车次 —— 排队的车已经走了，上一辆收尾后照样给它发受理。"""
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("analyzing")]
        h.session.entered(h.clock[0])
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(2)
        h.session.left(h.clock[0])                    # 第一辆离场，作业继续盯分析
        h.session.entered(h.clock[0])
        h.session.measured(h.clock[0], "m-next", "宁B54516", channel_state())
        self.assertIsNotNone(h.session.pending)
        h.session.left(h.clock[0])                    # 第二辆还在排队就走了
        self.assertIsNone(h.session.pending)
        h.api.progress_script = [progress("completed", terminal=True, dimensions=stored_dimensions())]
        h.advance(10)
        self.assertEqual(len(h.api.posts()), 1, "已经离场的排队车不许再发受理")
        self.assertIn("not_accepted:离场前未受理", [row["outcome"] for row in h.records()])

    def test_a4b_next_vehicle_events_do_not_touch_the_previous_job(self):
        """修复前：上一辆车离场后作业还在盯落盘，下一辆车的 移动/离场 事件会算到它头上。"""
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("captured", capturedCameras=["CAM_004"])]
        h.session.entered(h.clock[0])
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(2)
        h.session.left(h.clock[0])
        self.assertEqual(h.text().count("继续监控落盘"), 1)
        h.session.entered(h.clock[0])                 # 下一辆车，还没测完
        h.session.moving(h.clock[0])
        h.session.left(h.clock[0])
        self.assertEqual(h.text().count("继续监控落盘"), 1, "下一辆的离场不许再算到上一辆作业上")
        self.assertEqual(h.api.cancels(), [])

    def test_a4c_moving_after_a_finished_capture_is_silent(self):
        """修复前：作业收尾后车倒出去，一律打"本车未发起双雷达采集"，误导现场。"""
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("completed", terminal=True, dimensions=stored_dimensions())]
        h.session.entered(h.clock[0])
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        h.advance(4)
        self.assertEqual(h.records()[0]["outcome"], "completed")
        h.session.moving(h.clock[0])
        self.assertNotIn("本车未发起双雷达采集", h.text())

    # ── 5. 双雷达状态：按 Worker 的真实状态值判 ─────────────────────────
    def test_a5_dual_status_covers_every_worker_state(self):
        """Worker `DualRadarScanOnceState`：可受理有 idle（持续扫描）与 ready（硬件待机）两个。
        修复前 ready 被当成"忙"，disabled / initializing 也被当成"忙"照发。"""
        def status(state=None, **scan):
            if state is not None:
                scan["state"] = state
            return {"data": {"scan": scan, "preview": {"available": True}}}
        for state in ("idle", "ready"):
            self.assertEqual(drc.dual_status(status(state)), "ok", state)
        for state in ("opening", "scanning", "stopping"):
            self.assertEqual(drc.dual_status(status(state)), "busy", state)
        for state in ("disabled", "initializing"):
            self.assertEqual(drc.dual_status(status(state)), "unavailable", state)
        self.assertEqual(drc.dual_status(status("idle", enabled=False)), "unavailable")

    def test_a5b_disabled_scan_skips_with_the_reason(self):
        h = self.h
        h.api.status_body = {"code": 200, "success": True, "message": "success",
                             "data": {"scan": {"enabled": False, "state": "disabled"},
                                      "preview": {"available": True}}}
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        self.assertEqual(h.api.posts(), [])
        self.assertIn("单次扫描功能未启用", h.text())
        self.assertEqual(h.records()[0]["outcome"], "skipped_dual_unavailable")

    def test_a5c_busy_note_names_the_right_cause(self):
        """状态"忙" = Worker 在**别的**会话里；修复前提示成"等本工具自己的作业"。"""
        h = self.h
        h.api.status_body = {"code": 200, "success": True, "message": "success",
                             "data": {"scan": {"state": "scanning"}, "preview": {"available": True}}}
        h.api.capture_script = [(409, {"code": 409, "message": "Worker busy", "success": False,
                                       "data": None}), accepted_body()]
        h.api.progress_script = [progress("waiting")]
        h.session.measured(h.clock[0], MID, PLATE, channel_state())
        self.assertNotIn("等本工具自己的作业", h.text())
        self.assertIn("别的采集会话", h.text())

    # ── 6. 负载闸门用近期帧率 ─────────────────────────────────────────
    def test_a6_load_guard_prefers_recent_fps(self):
        """`/api/state` 的 fps 是开机以来的平均值；闸门必须看近期帧率。"""
        cfg = drc.CaptureConfig()
        self.assertIsNotNone(drc.load_guard({"fps": 9.9, "recent_fps": 5.0}, cfg),
                             "开机平均 9.9、近 2 秒只有 5 帧 -> 必须拦")
        self.assertIsNone(drc.load_guard({"fps": 3.0, "recent_fps": 9.8}, cfg),
                          "开机平均被启动拖低、近期正常 -> 放行")
        self.assertIsNotNone(drc.load_guard({"fps": 5.0}, cfg), "没有近期值才退回 fps")

    def test_a6b_recorder_measures_recent_fps_from_frame_deltas(self):
        # ① 近期帧率 = 相邻两次采样的 frames 差 / 时间差（时间直接传，不打全局时钟的补丁）
        row = recorder._new_row()
        recorder._update_recent_fps({"frames": 1000, "fps": 9.9}, row, 100.0)
        self.assertIsNone(row["recent_fps"], "只有一次采样时还算不出来")
        recorder._update_recent_fps({"frames": 1010, "fps": 9.9}, row, 102.0)
        self.assertAlmostEqual(row["recent_fps"], 5.0, places=3,
                               msg="(1010-1000)/2 秒 = 5 帧/秒，不是开机平均的 9.9")
        recorder._update_recent_fps({"frames": 5}, row, 104.0)       # 服务重启，计数从头来
        self.assertIsNone(row["recent_fps"])

        # ② 测完那一刻把近期帧率随快照交给管理器（闸门据此判断）
        seen = {}

        class Stub(TestRecorderWiring.StubSession):
            def on_measured(self, *a):
                seen["state"] = a[3]
                super().on_measured(*a)

        stub = Stub()
        state, out = {}, []
        args = TestRecorderWiring.args(TestRecorderWiring())
        for index, (st, mid) in enumerate(((1, None), (2, None), (2, MID))):
            snap = TestRecorderWiring.snapshot(state=st, mid=mid)
            snap["channels"][0]["frames"] = 1000 + index
            if index == 2:
                state["CH4"]["recent_fps"] = 5.0     # 前两次采样算出来的值（间隔太短不会被本次覆盖）
            recorder._poll_once("http://x/api/state", state, args, None, out.append, stub,
                                {"save_check": True},
                                opener=lambda url, timeout=3, s=snap: _Resp(s))
        self.assertEqual(seen["state"]["recent_fps"], 5.0)

    # ── 7. 车牌只取这一份测量的 ────────────────────────────────────────
    def test_a7_recorder_never_reopens_the_previous_vehicles_entry(self):
        """修复前：新车一进场就拿 `last_result.plate`（上一辆的牌）去 `open_vehicle`，
        把上一辆车已关闭的台账条目重新打开并清空字段。"""
        ledger = self.h.ledger
        old = "宁B08669"
        ledger.open_vehicle(old, 100.0)
        ledger.mark_triggered(old, "m-old", "lr-m-old", 100.0)
        ledger.mark_accepted(old, "wf-old")
        ledger.close_vehicle(old, 200.0)
        stub = TestRecorderWiring.StubSession()
        state, out = {}, []
        args = TestRecorderWiring.args(TestRecorderWiring())
        previous = {"plate": old, "measurement_id": "m-old"}
        for snap in (TestRecorderWiring.snapshot(state=1, last=previous),
                     TestRecorderWiring.snapshot(state=2, last=previous)):
            recorder._poll_once("http://x/api/state", state, args, ledger, out.append, stub,
                                {"save_check": True},
                                opener=lambda url, timeout=3, s=snap: _Resp(s))
        entry = self.h.ledger_raw()[old]
        self.assertEqual(entry["state"], "closed", "上一辆车的条目不许被重新打开")
        self.assertEqual(entry["workflow_id"], "wf-old", "上一辆车的记录不许被清空")
        self.assertNotIn(("measured", MID), stub.calls)

    def test_a7b_plate_comes_from_the_matching_result(self):
        seen = {}

        class Stub(TestRecorderWiring.StubSession):
            def on_measured(self, *a):
                seen["plate"] = a[2]
                super().on_measured(*a)

        stub = Stub()
        state, out = {}, []
        args = TestRecorderWiring.args(TestRecorderWiring())
        stale = {"plate": "宁B08669", "measurement_id": "m-old"}   # current 已换、last 还没换
        snaps = (TestRecorderWiring.snapshot(state=1, last=stale),
                 TestRecorderWiring.snapshot(state=2, mid=MID, last=stale))
        for snap in snaps:
            recorder._poll_once("http://x/api/state", state, args, None, out.append, stub,
                                {"save_check": True},
                                opener=lambda url, timeout=3, s=snap: _Resp(s))
        self.assertEqual(seen["plate"], "", "对不上测量号的 last_result 不能拿来当本车车牌")

    # ── 8. duplicate_of：与车牌无关的去重（模拟车牌 60 秒轮换） ──────────
    def test_a8_duplicate_chain_blocks_retrigger_after_restart_with_rotating_plates(self):
        """修复前：方案 §4.2 的第一层（`duplicate_of`）根本没实现，只有车牌台账。
        模拟车牌每 60 秒换一个 -> 记录器重启后，停着的空车在下一次超时重测时
        车牌已经变了，台账认不出，再采一遍（往 DM8 多写一条）。"""
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("completed", terminal=True, dimensions=stored_dimensions())]
        h.session.entered(h.clock[0])
        h.session.measured(h.clock[0], MID, "宁A09185", channel_state())
        h.advance(4)
        self.assertEqual(len(h.api.posts()), 1)

        h.session = h.new_session()                   # 记录器被看门狗重启
        h.session.entered(h.clock[0])                 # 车还停着，被重新记成"进场"
        second = dict(channel_state(), last_result={"measurement_id": "m-2", "plate": "宁A27066",
                                                    "duplicate_of": MID})
        h.session.measured(h.clock[0] + 180, "m-2", "宁A27066", second)
        third = dict(channel_state(), last_result={"measurement_id": "m-3", "plate": "宁A49586",
                                                   "duplicate_of": "m-2"})
        h.session.measured(h.clock[0] + 360, "m-3", "宁A49586", third)
        self.assertEqual(len(h.api.posts()), 1, "同一辆车的超时重测（链）不许再采")
        self.assertIn("被超时重测", h.text())

        h.session.entered(h.clock[0])                 # 真正的下一辆车：没有 duplicate_of
        h.api.progress_script = [progress("waiting")]
        h.session.measured(h.clock[0] + 400, "m-4", "宁A92083", channel_state())
        self.assertEqual(len(h.api.posts()), 2, "不同的车照常触发")

    # ── 9. 后台线程不许悄悄死掉 ────────────────────────────────────────
    def test_a9_manager_thread_survives_an_exception(self):
        """修复前：`run()` 没有兜底，台账写不进去之类的异常会让线程死掉，之后一次都不触发。"""
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        real = h.ledger.open_entry
        calls = {"n": 0}

        def flaky(plate, now):
            calls["n"] += 1
            if calls["n"] == 1:
                raise OSError("台账被占用")
            return real(plate, now)

        h.ledger.open_entry = flaky
        manager = drc.CaptureManager(h.client, h.ledger, h.out.append, h.cfg)
        manager.start()
        self.addCleanup(manager.close)
        manager.on_entered(time.time())
        manager.on_measured(time.time(), MID, PLATE, channel_state())
        manager.on_entered(time.time())
        manager.on_measured(time.time(), "m-next", "宁B54516", channel_state())
        deadline = time.time() + 10
        while time.time() < deadline and not h.api.posts():
            time.sleep(0.05)
        self.assertTrue(manager.is_alive(), "出一次错线程不许死")
        self.assertIn("双雷达采集后台出错", h.text())
        self.assertTrue(h.api.posts(), "出错之后的下一辆车照常触发")

    def test_a9b_tick_catches_a_job_error_and_keeps_the_vehicle(self):
        """`tick` **自己**那层兜底：作业在轮询里抛错（例如阶段变化时写台账失败），
        不许把整条线程带走，而且**这份作业还得能接着推进**。

        ▲ 与 a9 的区别：a9 的错发生在**事件分发**里（`open_entry` → `_drain` 的兜底），
          本条的错发生在**推进**里（`mark_progress` → `tick` 自己的 `except`，见
          `tools/dual_radar_capture.py` 的 `JOB_ERROR_LIMIT` 段）。
          两处是**分开的两层**守卫，谁也替代不了谁 —— 少任何一层，删掉它的变异都不会变红。
        """
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        real = h.ledger.mark_progress
        calls = {"n": 0}

        def flaky(plate, stage, terminal):
            calls["n"] += 1
            if calls["n"] == 1:
                raise OSError("台账写不进去")          # 阶段变化时写台账失败
            return real(plate, stage, terminal)

        h.ledger.mark_progress = flaky
        manager = drc.CaptureManager(h.client, h.ledger, h.out.append, h.cfg)
        manager.start()
        self.addCleanup(manager.close)
        manager.on_entered(time.time())
        manager.on_measured(time.time(), MID, PLATE, channel_state())
        deadline = time.time() + 10
        while time.time() < deadline and "双雷达采集处理出错" not in h.text():
            time.sleep(0.05)
        self.assertTrue(manager.is_alive(), "推进时出一次错，线程不许死")
        # ★ 这一层是 `tick` 内部接住的（"处理出错（第 N 次）"），不是 `run()` 的"后台出错"：
        #   `tick` 的 try 罩住了 `_repeat_alarm`/`_step`，错在这里就被拦下。
        self.assertIn("双雷达采集处理出错（第 1 次）", h.text())
        self.assertNotIn("双雷达采集后台出错", h.text(), "这一层不该漏到 run() 的兜底")
        # ★ 与 a9 不同：这一层不让作业作废。出错一次之后**同一份作业**还得能接着推进：
        #   把进度推到终局，它应当正常收尾（`tick` 在成功一步后会清零 `errors`）。
        h.api.progress_script = [progress("completed", terminal=True,
                                          dimensions=stored_dimensions())]
        deadline = time.time() + 10
        while time.time() < deadline and "计算完成并写入数据库" not in h.text():
            time.sleep(0.05)
        self.assertIn("计算完成并写入数据库", h.text(), "出错一次之后这份作业仍要能收尾")
        self.assertEqual(len(h.api.posts()), 1, "还是同一份作业，不该重发受理")
        # 队列也没被堵住：下一辆车照常受理
        h.api.capture_script = [accepted_body("wf-2")]
        h.api.progress_script = [progress("waiting")]
        manager.on_entered(time.time())
        manager.on_measured(time.time(), "m-next", "宁B54516", channel_state())
        deadline = time.time() + 10
        while time.time() < deadline and len(h.api.posts()) < 2:
            time.sleep(0.05)
        self.assertEqual(len(h.api.posts()), 2, "出错之后线程必须继续处理后面的车")

    def test_a9c_run_level_tick_guard_survives_a_begin_failure(self):
        """`run()` 里"推进作业"那层兜底：只有 `tick` **自己没接住**的错才走得到这里。

        `tick` 的 try 只罩住 `_repeat_alarm`/`_step`，**排队车开局**（`self._begin(...)`，
        在 try 之外）如果抛错就会穿出 `tick` —— 这一条专走那条路：
        第 2 辆车开局时炸 → `run()` 的 `_guarded("推进作业", ...)` 接住 → 线程继续。
        把 `run()` 里那层兜底删掉，本条会红（变异验证实测）；a9 / a9b 那两条删了照样过。
        """
        h = self.h
        h.api.capture_script = [accepted_body()]
        h.api.progress_script = [progress("waiting")]
        manager = drc.CaptureManager(h.client, h.ledger, h.out.append, h.cfg)
        real_begin = manager.session._begin
        calls = {"n": 0}

        def flaky(*args, **kwargs):
            calls["n"] += 1
            if calls["n"] == 2:                     # 第 1 辆正常开局；排队那辆开局时炸
                raise OSError("开局写台账失败")
            return real_begin(*args, **kwargs)

        manager.session._begin = flaky
        manager.start()
        self.addCleanup(manager.close)
        # 第 1 辆：正常开局，占住作业位
        manager.on_entered(time.time())
        manager.on_measured(time.time(), MID, PLATE, channel_state())
        deadline = time.time() + 10
        while time.time() < deadline and manager.session.job is None:
            time.sleep(0.05)
        self.assertIsNotNone(manager.session.job, "第 1 辆应当已经开局")
        # 第 2 辆：第 1 辆还占着，先排队（此时**不会**调 `_begin`）
        manager.on_entered(time.time())
        manager.on_measured(time.time(), "m-queued", "宁B54516", channel_state())
        deadline = time.time() + 10
        while time.time() < deadline and manager.session.pending is None:
            time.sleep(0.05)
        self.assertIsNotNone(manager.session.pending, "第 2 辆应当先排队")
        # 放行第 1 辆收尾 -> 下一轮 tick 给排队那辆开局 -> 抛错 -> 穿出 tick -> run() 的兜底
        h.api.progress_script = [progress("completed", terminal=True, dimensions=stored_dimensions())]
        deadline = time.time() + 10
        while time.time() < deadline and "双雷达采集后台出错" not in h.text():
            time.sleep(0.05)
        self.assertTrue(manager.is_alive(), "推进时 `_begin` 抛错，线程不许死")
        self.assertIn("后台出错（推进作业", h.text())
        # 还得继续干活：第 3 辆照常受理（calls=3，不再炸）
        h.api.capture_script = [accepted_body("wf-3")]
        h.api.progress_script = [progress("waiting")]
        manager.on_entered(time.time())
        manager.on_measured(time.time(), "m-next", "宁C36935", channel_state())
        deadline = time.time() + 10
        while time.time() < deadline and len(h.api.posts()) < 2:
            time.sleep(0.05)
        self.assertGreaterEqual(len(h.api.posts()), 2, "出错之后线程必须继续处理后面的车")

    # ── 10. 记录器：采样间隔下限、读不到 /api/state 要说出来 ──────────────
    def test_a10_recorder_clamps_interval_and_reports_read_failures(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        proc = subprocess.Popen(
            [sys.executable, "-u", os.path.join("tools", "field_standstill_log.py"),
             "--interval", "0.1", "--no-capture", "--url", "http://127.0.0.1:1/api/state"],
            cwd=root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
        try:
            time.sleep(4.0)
        finally:
            proc.kill()
            output = proc.communicate(timeout=30)[0].decode("utf-8", errors="replace")
        self.assertIn("太密", output)
        self.assertIn("读不到", output)


if __name__ == "__main__":
    unittest.main(verbosity=2)
