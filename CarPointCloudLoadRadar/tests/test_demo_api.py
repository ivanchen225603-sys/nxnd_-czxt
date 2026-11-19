# -*- coding: utf-8 -*-
"""
看板接口测试（app/web/server.py）—— 只测"演示"相关的那几个 POST。

为什么要专门测 HTTP 层：前端的「自动演示 / 演示：空车·载货 / 强制重测」
三个按钮全靠这三个接口，接口写错了**界面上看不出任何异常**（按钮点下去
什么都不发生、也不报错），现场演示时才发现就晚了。

注意：**不用 urllib** —— 本机代理会拦掉它（502）。统一走 http.client。
服务绑 127.0.0.1 + 高位端口，起停了就关，不碰 8080。
"""
from __future__ import annotations

import http.client
import json
import os
import re
import socket
import threading
import time
import unittest

from tests import _helpers as H
from app.service import LidarService
from app.web.server import serve


def _pick_port(lo: int = 18100, hi: int = 18200) -> int:
    """找一个真能排他绑上的端口。

    ★ 探测必须与 _ExclusiveHTTPServer 的策略一致（Windows 用
      SO_EXCLUSIVEADDRUSE）。否则探测永远返回"空闲"，起服务时才炸。
    """
    import socket as _s
    for p in range(lo, hi):
        s = _s.socket(_s.AF_INET, _s.SOCK_STREAM)
        try:
            if os.name == "nt":
                try:
                    s.setsockopt(_s.SOL_SOCKET, _s.SO_EXCLUSIVEADDRUSE, 1)
                except (AttributeError, OSError):
                    pass
            s.bind(("127.0.0.1", p))
            return p
        except OSError:
            continue
        finally:
            s.close()
    raise RuntimeError(f"{lo}-{hi} 内没有空闲端口")


class _ServerFixture:

    def __init__(self, **kw):
        st, ch, _, _, _ = H.ctx()
        kw.setdefault("settings", st)
        kw.setdefault("save_override", False)
        kw.setdefault("plate_mode", "simulate")
        kw.setdefault("demo", True)
        kw.setdefault("only", [ch.id])
        self.ch_id = ch.id
        self.svc = LidarService(**kw)
        self.port = _pick_port()
        self.httpd = serve(self.svc, host="127.0.0.1", port=self.port)
        self.thread = threading.Thread(target=self.httpd.serve_forever,
                                       kwargs={"poll_interval": 0.1}, daemon=True)
        self.svc.start()
        self.thread.start()
        self._wait_up()

    def _wait_up(self, timeout: float = 5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                code, _ = self.get("/api/health")
                if code == 200:
                    return
            except OSError:
                time.sleep(0.05)
        raise RuntimeError("看板服务在 5s 内没起来")

    def close(self):
        try:
            self.httpd.shutdown()
        except Exception:                                # noqa: BLE001
            pass
        try:
            self.httpd.server_close()
        except Exception:                                # noqa: BLE001
            pass
        self.svc.stop()

    # ── HTTP 小工具（刻意不用 urllib：本机代理会拦）──
    def _req(self, method: str, path: str, body=None):
        c = http.client.HTTPConnection("127.0.0.1", self.port, timeout=6)
        try:
            payload = None
            headers = {}
            if body is not None:
                payload = json.dumps(body).encode("utf-8")
                headers["Content-Type"] = "application/json"
            c.request(method, path, body=payload, headers=headers)
            r = c.getresponse()
            raw = r.read()
            return r.status, raw
        finally:
            c.close()

    def get(self, path):
        st, raw = self._req("GET", path)
        try:
            return st, json.loads(raw.decode("utf-8"))
        except Exception:                                # noqa: BLE001
            return st, raw

    def post(self, path, body=None):
        st, raw = self._req("POST", path,
                            {} if body is None else body)
        try:
            return st, json.loads(raw.decode("utf-8"))
        except Exception:                                # noqa: BLE001
            return st, raw


class TestDemoApi(unittest.TestCase):
    """演示服务（auto_demo=True）下的接口行为。"""

    @classmethod
    def setUpClass(cls):
        cls.fx = _ServerFixture(auto_demo=True)

    @classmethod
    def tearDownClass(cls):
        cls.fx.close()

    def test_health(self):
        code, body = self.fx.get("/api/health")
        self.assertEqual(code, 200)
        self.assertTrue(body["ok"])
        self.assertEqual(body["channels"], 1)

    def test_state_history_can_be_omitted(self):
        """?history=0 只省略各通道最近结果；不带参数仍是全量（看板与记录器不变）。"""
        code, full = self.fx.get("/api/state")
        self.assertEqual(code, 200)
        self.assertTrue(all("history" in c for c in full["channels"]))
        code, light = self.fx.get("/api/state?history=0")
        self.assertEqual(code, 200)
        self.assertTrue(all("history" not in c for c in light["channels"]))
        self.assertEqual([c["id"] for c in full["channels"]], [c["id"] for c in light["channels"]])

    def test_state_reports_demo_mode(self):
        code, body = self.fx.get("/api/state")
        self.assertEqual(code, 200)
        self.assertEqual(body["service"]["sim_channels"], 1)
        self.assertEqual(body["service"]["live_channels"], 0)
        self.assertIsNotNone(body["director"])
        self.assertTrue(body["director"]["auto"])

    def test_index_has_demo_banner(self):
        """看板必须带"这是演示数据"的横幅元素 —— 不能让领导误以为接了硬件。"""
        code, raw = self.fx._req("GET", "/")
        self.assertEqual(code, 200)
        html = raw.decode("utf-8", "replace")
        self.assertIn("banner-demo", html, "首页缺少演示横幅占位")
        self.assertIn("自动演示", html, "首页缺少自动演示按钮")

    def test_index_layout_keeps_four_cards_on_one_screen(self):
        """
        首页布局必须能让 4 路卡片在 1600 宽的桌面窗口里一屏排开。

        曾经的坑（两个，必须一起改）：
          ① `.grid` 的 minmax 是 520px，1600 宽只排得下 3 列，
             第 4 路被挤到第二行 —— 演示时要滚动才看得到第 4 路，
             而桌面窗口是固定 1600x980（desktop/launcher.chromium_cmd）。
          ② **光改 minmax 没用**：网格项默认 min-width:auto，卡片里
             「六面拟合」表格是 white-space:nowrap，min-content ≈ 560px，
             会把轨道撑破（CSS Grid blowout），1fr 压不下去 → 3 列 + 横向溢出。
             所以必须配 `.grid > .card{min-width:0}`。
        另外诊断/六面/历史三张表要有"折叠"（默认收起），
        否则光卡片自身就有两三屏高。
        """
        _, raw = self.fx._req("GET", "/")
        html = raw.decode("utf-8", "replace")
        m = re.search(r"\.grid\{[^}]*minmax\(min\(100%,(\d+)px\)", html)
        self.assertIsNotNone(m, "`.grid` 的列宽写法变了，请重新核对能否一屏 4 列")
        col = int(m.group(1))
        # 1600 宽（减左右 padding 40 与 3 个 16px 间隙）要放得下 4 列
        self.assertLessEqual(col * 4 + 16 * 3 + 40, 1600,
                             f"最小列宽 {col}px 时 1600 宽的窗口排不下 4 路")
        # ★ 必须配套 min-width:0，否则表头 nowrap 会撑破轨道
        self.assertRegex(
            html, r"\.grid\s*>\s*\.card\{[^}]*min-width\s*:\s*0",
            "缺少 `.grid > .card{min-width:0}` —— 表格的 min-content 会撑破列宽")
        self.assertIn(".detail", html, "缺少详情折叠容器")
        self.assertIn(".detail.open", html, "缺少详情展开样式")
        self.assertIn("展开详情", html, "缺少详情折叠按钮")

    def test_index_says_settling_window(self):
        """停稳后要如实说明"在等沉降窗口"，不能只写"检测中"让人以为卡住。"""
        _, raw = self.fx._req("GET", "/")
        html = raw.decode("utf-8", "replace")
        self.assertIn("settle_remain", html,
                      "前端没读 st.stability.settle_remain —— 停稳后无法显示剩余秒数")
        self.assertIn("沉降窗口", html)

    def test_auto_off_then_on(self):
        code, body = self.fx.post("/api/demo/auto", {"enabled": False})
        self.assertEqual(code, 200)
        self.assertTrue(body["ok"])
        self.assertFalse(body["auto"])
        self.assertFalse(body["director"]["auto"])
        self.assertFalse(self.fx.get("/api/state")[1]["director"]["auto"])

        code, body = self.fx.post("/api/demo/auto", {"enabled": True})
        self.assertEqual(code, 200)
        self.assertTrue(body["auto"])
        self.assertTrue(self.fx.get("/api/state")[1]["director"]["auto"])

    def test_auto_without_enabled_toggles(self):
        before = self.fx.get("/api/state")[1]["director"]["auto"]
        code, body = self.fx.post("/api/demo/auto", {})
        self.assertEqual(code, 200)
        self.assertEqual(body["auto"], not before)
        code, body = self.fx.post("/api/demo/auto", {})
        self.assertEqual(body["auto"], before)         # 再翻回来

    def test_loaded_switch_pauses_auto(self):
        code, body = self.fx.post("/api/demo/loaded",
                                  {"channel": self.fx.ch_id, "loaded": True})
        self.assertEqual(code, 200)
        self.assertTrue(body["ok"])
        self.assertTrue(body["loaded"])
        d = self.fx.get("/api/state")[1]["director"]
        self.assertFalse(d["auto"], "手动切场景后编排器没让位")
        self.assertTrue(d["paused_reason"])
        # 收尾：恢复自动演示
        self.fx.post("/api/demo/auto", {"enabled": True})

    def test_loaded_unknown_channel_400(self):
        code, body = self.fx.post("/api/demo/loaded",
                                  {"channel": "NOPE", "loaded": True})
        self.assertEqual(code, 400)
        self.assertFalse(body["ok"])

    def test_force_new_session(self):
        code, body = self.fx.post("/api/demo/force", {"channel": self.fx.ch_id})
        self.assertEqual(code, 200)
        self.assertEqual(body["channels"], 1)
        self.fx.post("/api/demo/auto", {"enabled": True})

    def test_force_all_channels(self):
        code, body = self.fx.post("/api/demo/force", {})
        self.assertEqual(code, 200)
        self.assertEqual(body["channels"], 1)
        self.fx.post("/api/demo/auto", {"enabled": True})

    def test_force_missing_channel_does_not_claim_success(self):
        code, body = self.fx.post("/api/demo/force", {"channel": "missing-channel"})
        self.assertEqual(code, 409)
        self.assertFalse(body["ok"])
        self.assertEqual(body["channels"], 0)

    def test_unknown_path_404(self):
        self.assertEqual(self.fx.post("/api/nope", {})[0], 404)
        self.assertEqual(self.fx.get("/api/nope")[0], 404)

    def test_history_endpoint(self):
        code, body = self.fx.get("/api/history?channel=%s&limit=5" % self.fx.ch_id)
        self.assertEqual(code, 200)
        self.assertIn("rows", body)
        self.assertIn("count", body)

    def test_config_endpoint(self):
        code, body = self.fx.get("/api/config")
        self.assertEqual(code, 200)
        self.assertIsInstance(body, dict)


class TestAutoApiWithoutDirector(unittest.TestCase):
    """非演示启动（没有编排器）时 /api/demo/auto 必须明确报错，不能假装成功。"""

    @classmethod
    def setUpClass(cls):
        # demo=True 但 auto_demo=False → 场景静止、没有编排器
        cls.fx = _ServerFixture(auto_demo=False)

    @classmethod
    def tearDownClass(cls):
        cls.fx.close()

    def test_state_director_is_null(self):
        self.assertIsNone(self.fx.get("/api/state")[1]["director"])

    def test_auto_returns_400(self):
        code, body = self.fx.post("/api/demo/auto", {"enabled": True})
        self.assertEqual(code, 400)
        self.assertFalse(body["ok"])
        self.assertIn("error", body)

    def test_loaded_still_works(self):
        """没有编排器时，手动切场景按钮照样要能用。"""
        code, body = self.fx.post("/api/demo/loaded",
                                  {"channel": self.fx.ch_id, "loaded": True})
        self.assertEqual(code, 200)
        self.assertTrue(body["ok"])


if __name__ == "__main__":
    unittest.main()
