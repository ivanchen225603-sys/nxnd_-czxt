#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_net_diag.py — 钉住 tools/net_diag.py 的纯函数（现场诊断口径）

背景：2026-09-13 现场 4 台雷达全部以 255.255.255.255 广播点云，
合计 38 MB/s 把同网段百兆设备（198.51.100.5/.6）打到不可达。
这里钉住"识别雷达 MAC / 解析 ARP / 风暴分级"三处判定口径，
避免以后把现场诊断工具改坏还看不出来。
"""
from __future__ import annotations

import os
import sys
import unittest
from unittest import mock

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from tools.net_diag import (          # noqa: E402
    is_radar_mac, mbps_of, norm_mac, parse_arp, storm_level,
    _in_prefixes, _is_noise_ip,
    STORM_ALERT_MBPS, STORM_FLOOD_MBPS, STORM_NOTICE_MBPS,
)


class TestMacHelpers(unittest.TestCase):
    def test_norm_mac_unifies_dash_and_colon(self):
        self.assertEqual(norm_mac("EC-9F-0D-02-85-15"), "ec:9f:0d:02:85:15")
        self.assertEqual(norm_mac(" ec:9f:0d:02:85:15 "), "ec:9f:0d:02:85:15")
        self.assertEqual(norm_mac("EC-9F-0D-02-85-15"), norm_mac("ec:9f:0d:02:85:15"))

    def test_radar_prefix_recognized(self):
        """ec:9f:0d 是雷达 MAC —— 现场三台实测都是这个前缀。"""
        for mac in ("ec-9f-0d-02-85-00", "ec:9f:0d:02:85:15",
                    "EC-9F-0D-03-24-9A"):
            self.assertTrue(is_radar_mac(mac), mac)

    def test_other_vendor_not_radar(self):
        """198.51.100.5/.6 是保定银河电子（00-21-92），绝不能算雷达。"""
        for mac in ("00-21-92-04-8d-fb", "00-21-92-03-be-ca",
                    "ff-ff-ff-ff-ff-ff", ""):
            self.assertFalse(is_radar_mac(mac), mac)


class TestParseArp(unittest.TestCase):
    # 直接抄一份中文 Windows `arp -a` 的真实输出
    SAMPLE = (
        "\u63a5\u53e3: 192.168.50.215 --- 0xb\n"
        "  Internet \u5730\u5740         \u7269\u7406\u5730\u5740              "
        "\u7c7b\u578b\n"
        "  192.168.50.153        ea-45-f5-68-6a-37     \u52a8\u6001\n"
        "  255.255.255.255       ff-ff-ff-ff-ff-ff     \u9759\u6001\n"
        "\n"
        "\u63a5\u53e3: 192.168.1.15 --- 0xc\n"
        "  198.51.100.2         ec-9f-0d-02-85-15     \u52a8\u6001\n"
        "  198.51.100.3         ec-9f-0d-03-24-9a     \u52a8\u6001\n"
        "  198.51.100.5         00-21-92-04-8d-fb     \u52a8\u6001\n"
    )

    def test_rows_parsed(self):
        rows = parse_arp(self.SAMPLE)
        ips = [r[1] for r in rows]
        self.assertIn("192.168.50.153", ips)
        self.assertIn("198.51.100.2", ips)
        self.assertIn("198.51.100.5", ips)
        self.assertIn("255.255.255.255", ips)

    def test_interface_carried_through(self):
        rows = parse_arp(self.SAMPLE)
        m = {r[1]: r[0] for r in rows}
        self.assertIn("192.168.1.15", m["198.51.100.2"])
        self.assertIn("192.168.50.215", m["192.168.50.153"])

    def test_radar_count_is_three(self):
        """把雷达从整表里挑出来 —— 现场就是 3 台（第 4 台已拔线）。"""
        rows = parse_arp(self.SAMPLE)
        radars = [r for r in rows if is_radar_mac(r[2])]
        self.assertEqual([r[1] for r in radars],
                         ["198.51.100.2", "198.51.100.3"])

    def test_header_and_blank_lines_ignored(self):
        rows = parse_arp("Internet \u5730\u5740 \u7269\u7406\u5730\u5740 \u7c7b\u578b\n\n")
        self.assertEqual(rows, [])


class TestScanScoping(unittest.TestCase):
    """★ 扫 192.168.1 时必须把 ARP 表里 198.51.100.2 的残留项分出去。

    实测踩坑（2026-09-14）：本工具曾把上一轮留下的 198.51.100.2 当成
    "在线雷达"打印，而当时 192.168.1.201 根本不应答 ARP —— 直接误导判断。
    """

    def test_noise_ips_filtered(self):
        """多播 / 广播 / 子网广播 / 回环 都不是设备。"""
        for ip in ("224.0.0.22", "224.0.0.251", "239.255.255.250",
                   "255.255.255.255", "192.168.1.255", "192.168.1.0",
                   "127.0.0.1"):
            self.assertTrue(_is_noise_ip(ip), ip)

    def test_real_hosts_not_noise(self):
        for ip in ("192.168.1.201", "198.51.100.2", "192.168.1.15"):
            self.assertFalse(_is_noise_ip(ip), ip)

    def test_prefix_membership(self):
        prefixes = ("192.168.1",)
        self.assertTrue(_in_prefixes("192.168.1.201", prefixes))
        self.assertTrue(_in_prefixes("192.168.1.15", prefixes))
        # 关键：换成 201 网段的设备就不属于 1 网段的扫描范围
        self.assertFalse(_in_prefixes("198.51.100.2", prefixes))
        self.assertFalse(_in_prefixes("192.168.50.153", prefixes))

    def test_stale_radar_entry_is_out_of_scope(self):
        """雷达 MAC 但不在扫描网段 → 必须被划到"残留"，不能算作在线雷达。"""
        rows = [("192.168.1.15", "198.51.100.2", "ec-9f-0d-02-85-15"),
                ("192.168.1.15", "198.51.100.6", "00-21-92-03-be-ca"),
                ("192.168.1.15", "192.168.1.255", "ff-ff-ff-ff-ff-ff")]
        prefixes = ("192.168.1",)
        in_scope = [r for r in rows
                    if _in_prefixes(r[1], prefixes) and not _is_noise_ip(r[1])]
        out_scope = [r for r in rows
                     if not _in_prefixes(r[1], prefixes) and not _is_noise_ip(r[1])]
        self.assertEqual(in_scope, [])                       # 该网段其实一台都没有
        self.assertEqual([r[1] for r in out_scope],
                         ["198.51.100.2", "198.51.100.6"])
        self.assertEqual([r[1] for r in in_scope
                          if is_radar_mac(r[2])], [])        # 在线雷达数必须为 0


class TestStormLevel(unittest.TestCase):
    def test_boundaries(self):
        self.assertEqual(storm_level(0), "ok")
        self.assertEqual(storm_level(STORM_NOTICE_MBPS), "ok")
        self.assertEqual(storm_level(STORM_NOTICE_MBPS + 1), "notice")
        self.assertEqual(storm_level(STORM_ALERT_MBPS + 1), "alert")
        self.assertEqual(storm_level(STORM_FLOOD_MBPS + 1), "flood")

    def test_real_field_measurement_is_flood(self):
        """实测 4 路广播 = 306 Mbps -> 必须判成 flood（这是当时的现场事实）。"""
        self.assertEqual(storm_level(306), "flood")

    def test_single_radar_is_not_flagged(self):
        """单台雷达约 79 Mbps —— 不该报风暴（否则天天误警）。"""
        self.assertEqual(storm_level(79), "ok")

    def test_after_unicast_fix_not_flagged(self):
        """改单播后 3 路 = 237 Mbps，但**都是单播**不泛洪 ->
        这里只看速率，故仍会提示；提示文案已说明要看是不是广播。"""
        self.assertIn(storm_level(237), ("alert", "flood"))


class TestMbpsOf(unittest.TestCase):
    def test_basic(self):
        # 1 MB in 1 s = 8 Mbps
        self.assertEqual(mbps_of(1, 1_000_000, 1.0), 8.0)

    def test_zero_seconds_safe(self):
        self.assertEqual(mbps_of(0, 0, 0), 0.0)


if __name__ == "__main__":
    unittest.main()


# ==========================================================================
# reach —— 「ping 不通」的因果判定
# ==========================================================================
import socket as _sock          # noqa: E402
from tools.net_diag import (    # noqa: E402
    classify_tcp_error, egress_for, reach_verdict, same_subnet24,
    tcp_probe, _REACH_TEXT,
)


class TestSameSubnet24(unittest.TestCase):
    """出口网卡判错是现场真踩过的坑（WLAN 抢了出口），这条判据不能含糊。"""

    def test_same(self):
        self.assertTrue(same_subnet24("192.168.1.15", "192.168.1.201"))

    def test_different(self):
        self.assertFalse(same_subnet24("198.51.100.15", "192.168.1.201"))

    def test_wlan_vs_radar(self):
        self.assertFalse(same_subnet24("192.168.50.215", "192.168.1.201"))

    def test_garbage_is_false_not_crash(self):
        self.assertFalse(same_subnet24("(路由不到: x)", "192.168.1.201"))


class TestTcpProbe(unittest.TestCase):
    """★ 三种失败必须分开 —— 混成一句"连不上"就失去诊断价值了。"""

    def test_open_port(self):
        srv = _sock.socket(_sock.AF_INET, _sock.SOCK_STREAM)
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        try:
            ok, _ms, why = tcp_probe("127.0.0.1", srv.getsockname()[1], timeout=2.0)
            self.assertTrue(ok, why)
        finally:
            srv.close()

    def test_refused_is_not_reported_as_timeout(self):
        """端口没开 = 主机活着。报成"超时"会把人引去查网络，而其实设备在。

        直接测分类函数，不去连真端口 —— Windows 对本机关闭端口是静默丢弃
        （超时），Linux 才回 RST（拒绝），靠实连会得到两个平台不同的结果。
        """
        why = classify_tcp_error(ConnectionRefusedError(10061, "refused"))
        self.assertIn("拒绝", why)
        self.assertNotIn("超时", why)

    def test_timeout_and_unreachable_are_distinct(self):
        t = classify_tcp_error(TimeoutError("timed out"))
        u = classify_tcp_error(OSError(10051, "network unreachable"))
        self.assertIn("超时", t)
        self.assertIn("不可达", u)
        self.assertNotEqual(t, u)

    def test_never_raises(self):
        # 受控模拟连接超时；不要求开发机网络对 TEST-NET 地址一定拒绝连接。
        with mock.patch("tools.net_diag.socket.socket") as socket_factory:
            connection = socket_factory.return_value
            connection.connect.side_effect = TimeoutError("测试连接超时")
            ok, _ms, why = tcp_probe("192.0.2.1", 9347, timeout=0.5)
            connection.close.assert_called_once()
        self.assertFalse(ok)
        self.assertTrue(why)


class TestReachVerdict(unittest.TestCase):
    """判定次序是这个工具的全部价值所在，逐条钉死。"""

    def test_tcp_ok_but_no_icmp_is_not_a_fault(self):
        """★ 最重要的一条：控制口通 = 回复路径好的，ping 不通只是不认 ICMP。

        判成故障会让人去重启雷达 —— 而重启可能让挂起的改 IP 生效、地址搬家。
        """
        v = reach_verdict(tcp_ok=True, icmp_ok=False, arp_ok=True,
                          same_net=True, storm_mbps=0)
        self.assertEqual(v, "ok_no_icmp")
        self.assertIn("不用重启", _REACH_TEXT[v])

    def test_both_ok(self):
        self.assertEqual(reach_verdict(True, True, True, True, 0), "ok")

    def test_wrong_egress_wins_over_everything(self):
        """出口都选错了，后面几项测了也没意义 —— 必须先报这个。"""
        self.assertEqual(reach_verdict(False, False, False, False, 999),
                         "wrong_egress")

    def test_storm_is_checked_before_mask_misconfig(self):
        """★ 风暴会把 ARP/ICMP/TCP 一起压死。

        此时若报成"掩码配错"，人就会去改雷达网络配置 —— 白费功夫，
        2026-09-13 现场是拔掉网线立刻恢复，配置一个字没动。
        """
        v = reach_verdict(tcp_ok=False, icmp_ok=False, arp_ok=True,
                          same_net=True, storm_mbps=STORM_NOTICE_MBPS + 50)
        self.assertEqual(v, "storm_suppressed")

    def test_arp_ok_without_storm_means_mask_misconfig(self):
        v = reach_verdict(False, False, True, True, 0)
        self.assertEqual(v, "l2_only")

    def test_nothing_at_all(self):
        self.assertEqual(reach_verdict(False, False, False, True, 0), "unreachable")

    def test_every_verdict_has_text(self):
        for code in ("ok", "ok_no_icmp", "wrong_egress", "storm_suppressed",
                     "l2_only", "unreachable"):
            self.assertIn(code, _REACH_TEXT)
            self.assertTrue(_REACH_TEXT[code].strip())


class TestEgressFor(unittest.TestCase):

    def test_loopback(self):
        """路由决策不发包，这条在任何机器上都该稳定成立。"""
        self.assertEqual(egress_for("127.0.0.1"), "127.0.0.1")

    def test_never_raises(self):
        self.assertIsInstance(egress_for("192.0.2.1"), str)
