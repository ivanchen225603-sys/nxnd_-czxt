# -*- coding: utf-8 -*-
"""看门狗的判据测试。

这个工具会在无人现场放着跑一整天，判错的代价很不对称：
· 该重启不重启 -> 采集悄悄停了，一天的数据没了
· 不该重启却重启 -> 无限重启循环，把日志刷爆、CPU 吃满，同样采不到

所以两条边界必须钉死：**进程死了要拉起**，**通道离线绝不重启**。
"""
from __future__ import annotations

import unittest

from tools.watchdog import BACKOFF_MAX, BACKOFF_START, next_backoff, should_restart


class TestShouldRestart(unittest.TestCase):

    def test_dead_process_restarts_immediately(self):
        need, why = should_restart(proc_alive=False, api_ok=False,
                                   api_bad_sec=0.0, api_grace_sec=60.0)
        self.assertTrue(need)
        self.assertIn("退出", why)

    def test_healthy_service_is_left_alone(self):
        need, _ = should_restart(True, True, 0.0, 60.0)
        self.assertFalse(need)

    def test_api_blip_within_grace_does_not_restart(self):
        """接口偶尔超时很正常（正在写盘、GC），没到宽限期不能动它。"""
        need, _ = should_restart(True, False, 30.0, 60.0)
        self.assertFalse(need)

    def test_api_wedged_past_grace_restarts(self):
        need, why = should_restart(True, False, 61.0, 60.0)
        self.assertTrue(need)
        self.assertIn("卡死", why)

    def test_grace_boundary_is_inclusive(self):
        self.assertTrue(should_restart(True, False, 60.0, 60.0)[0])


class TestChannelsOfflineNeverRestarts(unittest.TestCase):
    """★ 最重要的一条边界。

    雷达断电 / 网线松 / CH1 压根没接 —— 都是设备问题，重启服务修不了。
    若把"通道离线"当重启条件，会变成 重启 -> 还是离线 -> 再重启 的死循环，
    反而把本来还在正常采集的另外几路也一起打断。
    """

    def test_signature_has_no_channel_input(self):
        """判据函数**根本不接收**通道状态 —— 从接口上就杜绝这种误用。"""
        import inspect
        params = list(inspect.signature(should_restart).parameters)
        self.assertEqual(params,
                         ["proc_alive", "api_ok", "api_bad_sec", "api_grace_sec"])
        for p in params:
            self.assertNotIn("channel", p)
            self.assertNotIn("online", p)

    def test_all_channels_offline_but_api_alive_is_not_a_restart(self):
        """全部离线但接口还活着 = 设备问题，服务本身没毛病，不许重启。"""
        need, _ = should_restart(proc_alive=True, api_ok=True,
                                 api_bad_sec=0.0, api_grace_sec=60.0)
        self.assertFalse(need)


class TestBackoff(unittest.TestCase):
    """退避是防"配置写错导致每秒重启一次"的。"""

    def test_grows(self):
        self.assertGreater(next_backoff(BACKOFF_START), BACKOFF_START)

    def test_capped(self):
        v = BACKOFF_START
        for _ in range(50):
            v = next_backoff(v)
        self.assertEqual(v, BACKOFF_MAX)

    def test_never_below_start(self):
        self.assertGreaterEqual(next_backoff(0.0), BACKOFF_START)
        self.assertGreaterEqual(next_backoff(-5.0), BACKOFF_START)


if __name__ == "__main__":
    unittest.main()
