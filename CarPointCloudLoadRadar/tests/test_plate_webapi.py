# -*- coding: utf-8 -*-
"""按点位从后台拉车牌（GET /api/v1/radar/licensePlate?cameraIdentifier=CAM_00x）。

这条链和"相机直连"共用 app/plate.py，但有两处不一样，都踩过：

1. **没有"先抓拍"这一步。** 后台是直接返回当前点位的车牌，没有 snap 动作。
   而 `HttpPlateReader.read()` 原本无条件先请求一次 snap_url ——
   留空时 `_request("")` 立刻报「URL 未配置」，四次重试白等 2.6 秒，
   最后返回「抓拍请求失败」，**永远走不到取结果那一步**，
   且报错指向抓拍，与真正的接口毫无关系。

2. **点位是逐通道的。** plate 段是全局一份，而 CH1~CH4 对应 CAM_001~CAM_004。
   不按通道覆盖的话三条车道拿同一个点位去查，取回同一辆车的牌，
   而且不报错 —— 落盘目录和空车/载货配对会静默串到一起。
"""
from __future__ import annotations

import io
import json
import os
import threading
import unittest
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, HTTPServer

from app.plate import EMPTY_PLATE, HttpPlateReader
from app.settings import PlateCfg, load_settings

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHANNELS = os.path.join(ROOT, "config", "channels.json")


class _Backend(BaseHTTPRequestHandler):
    """照 CarPointCloudWebApi 的响应信封与取牌契约实现。"""

    protocol_version = "HTTP/1.1"
    seen = []                                  # 收到过的 cameraIdentifier

    def do_GET(self):                          # noqa: N802
        camera = ""
        if "cameraIdentifier=" in self.path:
            camera = self.path.split("cameraIdentifier=", 1)[1].split("&")[0]
        _Backend.seen.append(camera)
        body = json.dumps({"code": 200, "message": "success", "success": True,
                           "data": {"cameraId": camera,
                                    "licensePlate": "宁A%s" % camera[-3:],
                                    "captureTime": "2026-09-22 13:00:00",
                                    "source": "MOCK"}},
                          ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):                 # 测试里不要往 stderr 刷日志
        pass


class _Server:
    def __enter__(self):
        _Backend.seen = []
        self.httpd = HTTPServer(("127.0.0.1", 0), _Backend)
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        return "http://127.0.0.1:%d" % self.httpd.server_address[1]

    def __exit__(self, *_):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.thread.join(timeout=5)


def _cfg(base_url: str, camera: str) -> PlateCfg:
    return PlateCfg(
        enabled=True, mode="http", base_url=base_url, camera_identifier=camera,
        snap_url="",                                   # ★ 本接口没有抓拍步骤
        result_url="/api/v1/radar/licensePlate?cameraIdentifier={camera_identifier}",
        method="GET", plate_field="data.licensePlate")


class TestPullsPlateWithoutSnapStep(unittest.TestCase):

    def test_empty_snap_url_goes_straight_to_the_result_call(self):
        with _Server() as base:
            res = HttpPlateReader(_cfg(base, "CAM_002")).read()
        self.assertTrue(res.ok, res.error)
        self.assertEqual(res.plate, "宁A002")
        self.assertEqual(_Backend.seen, ["CAM_002"])
        # 只请求了一次：没有把 4 次抓拍重试算进去
        self.assertEqual(res.attempts, 1)

    def test_it_does_not_burn_the_snap_retry_budget(self):
        """三次重试间隔 300+800+1500 = 2.6 秒，白等下去会吃掉现场仅有的停留时间。"""
        with _Server() as base:
            res = HttpPlateReader(_cfg(base, "CAM_003")).read()
        self.assertLess(res.elapsed, 2.0)

    def test_a_configured_snap_url_is_still_honoured(self):
        """相机直连那条路不受影响 —— 配了 snap_url 就照旧先抓拍。"""
        with _Server() as base:
            cfg = replace(_cfg(base, "CAM_004"), snap_url="/snap")
            res = HttpPlateReader(cfg).read()
        self.assertTrue(res.ok, res.error)
        self.assertEqual(_Backend.seen[0], "")        # 第一发是 snap，没有点位参数
        self.assertIn("CAM_004", _Backend.seen)

    def test_unreachable_backend_degrades_to_unknown_plate(self):
        """车牌取不到绝不能拖垮检测：返回未知车牌，不抛异常。"""
        cfg = replace(_cfg("http://127.0.0.1:1", "CAM_002"),
                      f3_timeout_ms=300, f2_timeout_ms=100)
        res = HttpPlateReader(cfg).read()
        self.assertFalse(res.ok)
        self.assertEqual(res.plate, EMPTY_PLATE)


class TestPerChannelCamera(unittest.TestCase):

    def test_each_channel_queries_its_own_camera(self):
        with _Server() as base:
            for camera in ("CAM_002", "CAM_003", "CAM_004"):
                HttpPlateReader(_cfg(base, camera)).read()
        self.assertEqual(_Backend.seen, ["CAM_002", "CAM_003", "CAM_004"])

    def test_channels_json_gives_every_enabled_channel_a_unique_camera(self):
        st = load_settings(CHANNELS)
        enabled = [c for c in st.channels if c.enabled]
        cameras = [c.camera_identifier for c in enabled]
        self.assertTrue(all(cameras), "启用通道必须都配了 camera_identifier")
        self.assertEqual(len(set(cameras)), len(cameras),
                         "点位编码不能重复，否则两条车道会取到同一辆车的牌")

    def test_worker_overrides_the_global_camera(self):
        """ChannelWorker 必须用通道自己的点位，而不是 plate 段的全局值。"""
        st = load_settings(CHANNELS)
        for ch in st.channels:
            if not ch.camera_identifier:
                continue
            cfg = replace(st.plate, camera_identifier=ch.camera_identifier)
            url = HttpPlateReader(cfg)._url(cfg.result_url)
            self.assertIn("cameraIdentifier=" + ch.camera_identifier, url)
            self.assertNotIn("{camera_identifier}", url)


class TestOfflineRunsDoNotCallTheBackend(unittest.TestCase):
    """★★ 非 live 通道默认不读牌 —— 否则离线运行会反过来依赖在线服务。

    真事故（2026-09-22 迁仓时暴露）：`config/channels.json` 的 plate 段改成
    `http` 指向后台之后，**后台一停，整套用例就挂起**。
    因为仿真通道每帧也去读牌，而读不到要空等 `f3_timeout_ms`（默认 5 秒），
    一条 tick 60 帧的用例就是 5 分钟起步 —— 表现是"测试卡住不动"，
    而真正的原因（后台没起）在任何日志里都看不出来。

    实测：修之前 `TestDuplicateOfMark` 永远跑不完；修之后 3.2 秒。
    """

    def _worker(self, mode, plate_mode=""):
        from app.service import ChannelWorker, EventBus
        from app.settings import load_settings
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = mode
        # 强制成"真机 HTTP 读牌"的配置，模拟现场那份 channels.json
        plate = replace(st.plate, enabled=True, mode="", base_url="http://127.0.0.1:1",
                        snap_url="", result_url="/x?cameraIdentifier={camera_identifier}",
                        plate_field="data.licensePlate")
        w = ChannelWorker(ch, EventBus(), plate, save_override=False, plate_mode=plate_mode)
        try:
            return w.plate.kind
        finally:
            w.stop()
            w.source.close()

    def test_simulated_and_replay_channels_do_not_read_plates(self):
        for mode in ("synthetic", "simulate", "replay"):
            with self.subTest(mode=mode):
                self.assertEqual("null", self._worker(mode),
                                 "非 live 通道不该去调真实读牌接口")

    def test_live_channel_still_reads_plates(self):
        self.assertEqual("http", self._worker("live"))

    def test_explicit_plate_mode_still_wins(self):
        """要在仿真里读牌是可以的，但必须显式说 —— `--plate http`。"""
        self.assertEqual("http", self._worker("synthetic", plate_mode="http"))
        self.assertEqual("simulate", self._worker("synthetic", plate_mode="simulate"))


if __name__ == "__main__":
    unittest.main()
