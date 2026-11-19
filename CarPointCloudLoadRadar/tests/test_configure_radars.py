#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_configure_radars.py — 4 台雷达批量配置工具（tools/configure_radars.py）回归测试

覆盖三层：
  ① 纯函数：CIDR 展开、掩码换算、ipconfig 解析、显示宽度补齐
  ② 计划与回写：build_plan / sync_channels（含"绝不猜"的冲突拒绝）
  ③ 真实网络：起一个 mock 雷达，走完 scan → identify 的真实 TCP 路径

★ 关键设计断言（都是现场踩过的坑，改动时不要放宽）：
  · build_plan 在"两台雷达同 IP"时必须报错，不能自动分配
  · build_plan 在"待改数量 > 空闲目标"时必须报错
  · sync_channels 一定要先备份再写
  · frame_ms 默认**不动**（转速单位未确证，乱写会让点云缺扇面）
  · cidr_hosts 拒绝不带前缀的网段、拒绝超大网段（防手滑扫爆内网）
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import shutil
import socket
import struct
import sys
import tempfile
import threading
import unittest
from unittest import mock

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from tools.hr40_ip import (                                          # noqa: E402
    HDR1, HDR2, CMD_GET_ANGLE, CMD_GET_CONFIG, CMD_GET_INVENTORY,
    CMD_RESTART, CMD_SET_DEST, CMD_SET_SELF, _ip4, build_frame,
)
from tools import configure_radars as cr                             # noqa: E402


# ══════════════════════════════════════════════════════ 可改配置的 mock 雷达
class MockRadar:
    """能记住"自己被改成什么 IP"的最小 HR40 设备。

    和 test_hr40_protocol.py 里的 mock 不同：这里 get_config 返回的是
    **可变状态**，收到 0x21 / 0x20 会更新自己的字段 —— 这样才能验证
    「写入 → 复读」闭环，而不是每次都读回同一份死数据。
    """

    def __init__(self, ip="192.168.1.201", dest_ip="192.168.1.100",
                 dest_port=2368, spin_rate=10, serial="SN0001"):
        self.ip = ip
        self.mask = "255.255.255.0"
        self.gateway = "192.168.1.1"
        self.dest_ip = dest_ip
        self.dest_port = dest_port
        self.spin_rate = spin_rate
        self.serial = serial
        self.received = []
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(8)
        self.port = self.srv.getsockname()[1]
        self._stop = False
        threading.Thread(target=self._serve, daemon=True).start()

    # ---- payload 构造
    def _config(self) -> bytes:
        p = bytearray()
        p += bytes(_ip4(self.ip)) + bytes(_ip4(self.mask))
        p += bytes(_ip4(self.gateway)) + bytes(_ip4(self.dest_ip))
        p += struct.pack(">H", self.dest_port) + b"\x00\x00"
        p += struct.pack(">H", self.spin_rate)
        p += bytes([1]) + struct.pack(">H", 18000) + b"\x00" * 4
        p += bytes([2]) + b"\x00" + bytes([1, 3, 0]) + b"\x00"
        p += bytes([0]) + struct.pack(">H", 0)
        assert len(p) == 38, len(p)
        return bytes(p)

    def _inventory(self) -> bytes:
        def fixed(s, n):
            b = s.encode("ascii")
            return (b + b"\x00" * n)[:n]

        p = bytearray()
        p += fixed(self.serial, 18) + fixed("HR40-128", 32)
        p += fixed("2024-06-01", 16)
        p += bytes([0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E])
        p += fixed("v1.2.3", 16) + b"\x00" * 16 + fixed("v0.9.1", 16)
        assert len(p) == 120, len(p)
        return bytes(p)

    # ---- 服务
    def _serve(self):
        while not self._stop:
            try:
                conn, _ = self.srv.accept()
            except OSError:
                return
            threading.Thread(target=self._handle, args=(conn,), daemon=True).start()

    @staticmethod
    def _read_exact(conn, n):
        buf = bytearray()
        while len(buf) < n:
            c = conn.recv(n - len(buf))
            if not c:
                return None
            buf += c
        return bytes(buf)

    def _handle(self, conn):
        try:
            while True:
                head = self._read_exact(conn, 8)
                if head is None:
                    return
                h1, h2, cmd = head[0], head[1], head[2]
                (plen,) = struct.unpack(">I", head[4:8])
                body = self._read_exact(conn, plen) if plen else b""
                self.received.append((cmd, body))
                if (h1, h2) != (HDR1, HDR2):
                    return

                if cmd == CMD_GET_CONFIG:
                    resp = self._config()
                elif cmd == CMD_GET_INVENTORY:
                    resp = self._inventory()
                elif cmd == CMD_GET_ANGLE:
                    resp = b"Channel,0,0\r\n0,0.01,0.02\r\n"
                elif cmd == CMD_SET_SELF:
                    # 15 字节：ip(4) mask(4) gw(4) tail(3)
                    self.ip = ".".join(str(b) for b in body[0:4])
                    self.mask = ".".join(str(b) for b in body[4:8])
                    self.gateway = ".".join(str(b) for b in body[8:12])
                    self.self_tail = body[12:15]
                    resp = b""
                elif cmd == CMD_SET_DEST:
                    self.dest_ip = ".".join(str(b) for b in body[0:4])
                    self.dest_port = struct.unpack(">H", body[4:6])[0]
                    resp = b""
                else:
                    resp = b""            # 重启等
                conn.sendall(build_frame(cmd, resp))
        except OSError:
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def stop(self):
        self._stop = True
        try:
            self.srv.close()
        except OSError:
            pass


def _read_json(path):
    """安全读 JSON（用完就关，避免 ResourceWarning 污染测试输出）"""
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _channels_json(ips, frame_ms=100, udp=2368):
    return {
        "service": {"host": "0.0.0.0", "port": 8080},
        "channels": [
            {
                "id": f"CH{i + 1}", "name": f"{i + 1}号通道", "enabled": True,
                "mode": "live", "group": "",
                "lidar": {"ip": ip, "source_ip": ip, "udp_port": udp,
                          "tcp_port": 9347, "bind_ip": "",
                          "range_mode": "raw_offset", "z_down": True,
                          "frame_ms": frame_ms, "max_range_m": 60.0},
                "output_dir": "data/field",
            }
            for i, ip in enumerate(ips)
        ],
    }


def _radar(ip, serial="SN"):
    return {"ip": ip, "cfg": {"ip": ip, "subnet_mask": "255.255.255.0",
                              "gateway": "", "dest_ip": "192.168.1.100",
                              "dest_port": 2368, "spin_rate": 10},
            "inv": {"serial_number": serial}, "error": ""}


# ══════════════════════════════════════════════════════ ① 纯函数
class TestCidr(unittest.TestCase):
    def test_slash24_hosts(self):
        h = cr.cidr_hosts("192.168.1.0/24")
        self.assertEqual(len(h), 254)                 # 去掉网络号与广播
        self.assertEqual(h[0], "192.168.1.1")
        self.assertEqual(h[-1], "192.168.1.254")

    def test_slash32_single_host(self):
        self.assertEqual(cr.cidr_hosts("127.0.0.1/32"), ["127.0.0.1"])

    def test_slash30_two_hosts(self):
        self.assertEqual(cr.cidr_hosts("10.0.0.0/30"), ["10.0.0.1", "10.0.0.2"])

    def test_non_network_address_is_masked(self):
        # 传 192.168.1.37/24 应该按 192.168.1.0/24 展开（strict=False 语义）
        h = cr.cidr_hosts("192.168.1.37/24")
        self.assertEqual(h[0], "192.168.1.1")
        self.assertEqual(len(h), 254)

    def test_missing_prefix_rejected(self):
        # ★ 不带前缀绝不能静默当 /24 —— 本项目禁止静默默认值
        with self.assertRaises(ValueError):
            cr.cidr_hosts("192.168.1.0")

    def test_bad_prefix_rejected(self):
        with self.assertRaises(ValueError):
            cr.cidr_hosts("192.168.1.0/33")
        with self.assertRaises(ValueError):
            cr.cidr_hosts("192.168.1.0/abc")

    def test_huge_network_rejected(self):
        # ★ 防手滑：/16 有 65534 个地址，必须报错而不是扫爆内网
        with self.assertRaises(ValueError):
            cr.cidr_hosts("10.0.0.0/16")
        # 显式放开上限时应能算出来
        self.assertEqual(len(cr.cidr_hosts("10.0.0.0/24", cap=1000)), 254)

    def test_bad_ip_rejected(self):
        with self.assertRaises(ValueError):
            cr.cidr_hosts("192.168.1.999/24")


class TestMaskAndIpKey(unittest.TestCase):
    def test_mask_to_prefix(self):
        self.assertEqual(cr.mask_to_prefix("255.255.255.0"), 24)
        self.assertEqual(cr.mask_to_prefix("255.255.0.0"), 16)
        self.assertEqual(cr.mask_to_prefix("255.255.254.0"), 23)
        self.assertEqual(cr.mask_to_prefix("255.0.0.0"), 8)

    def test_mask_invalid(self):
        self.assertEqual(cr.mask_to_prefix("255.0.255.0"), -1)   # 不连续
        self.assertEqual(cr.mask_to_prefix("nonsense"), -1)

    def test_ip_key_sorts_numerically(self):
        got = sorted(["198.51.100.10", "198.51.100.2", "198.51.100.1"],
                     key=cr._ip_key)
        self.assertEqual(got, ["198.51.100.1", "198.51.100.2", "198.51.100.10"])

    def test_ip_key_garbage_sinks(self):
        self.assertEqual(cr._ip_key("bad")[0], 999)

    def test_same_slash24(self):
        self.assertTrue(cr.same_slash24("192.168.1.201", "192.168.1.250"))
        self.assertFalse(cr.same_slash24("192.168.1.201", "198.51.100.1"))


class TestDisplayWidth(unittest.TestCase):
    def test_chinese_counts_double(self):
        self.assertEqual(cr._disp_w("网卡"), 4)
        self.assertEqual(cr._disp_w("ab"), 2)

    def test_pad_aligns_by_display_width(self):
        self.assertEqual(cr.pad("网卡", 10), "网卡" + " " * 6)
        self.assertEqual(cr.pad("abcdef", 4), "abcdef")        # 超宽不截断
        self.assertEqual(cr.pad("", 3), "   ")
        self.assertEqual(cr.pad(None, 2), "  ")

    def test_pad_right_align(self):
        self.assertEqual(cr.pad("x", 4, ">"), "   x")


class TestParseIpconfig(unittest.TestCase):
    ZH = """\
Windows IP 配置

以太网适配器 以太网:

   媒体状态  . . . . . . . . . . . . : 媒体已断开连接
   连接特定的 DNS 后缀 . . . . . . . :

无线局域网适配器 WLAN:

   IPv4 地址 . . . . . . . . . . . . : 192.168.100.3
   子网掩码  . . . . . . . . . . . . : 255.255.255.0
   默认网关. . . . . . . . . . . . . : 192.168.100.1
"""

    EN = """\
Windows IP Configuration

Ethernet adapter Ethernet:

   Media State . . . . . . . . . . . : Media disconnected

Wireless LAN adapter WLAN:

   IPv4 Address. . . . . . . . . . . : 192.168.50.215
   Subnet Mask . . . . . . . . . . . : 255.255.255.0
   Default Gateway . . . . . . . . . : 192.168.50.1
"""

    def test_chinese(self):
        b = cr.parse_ipconfig(self.ZH)
        names = {x["name"] for x in b}
        self.assertIn("以太网", names)
        self.assertIn("WLAN", names)
        eth = next(x for x in b if x["name"] == "以太网")
        self.assertTrue(eth["media_disconnected"])
        self.assertEqual(eth["ip"], "")
        wlan = next(x for x in b if x["name"] == "WLAN")
        self.assertEqual(wlan["ip"], "192.168.100.3")
        self.assertEqual(wlan["mask"], "255.255.255.0")
        self.assertEqual(wlan["gateway"], "192.168.100.1")
        self.assertFalse(wlan["media_disconnected"])

    def test_english(self):
        b = cr.parse_ipconfig(self.EN)
        wlan = next(x for x in b if "WLAN" in x["name"])
        self.assertEqual(wlan["ip"], "192.168.50.215")
        self.assertEqual(wlan["gateway"], "192.168.50.1")
        eth = next(x for x in b if "Ethernet" in x["name"])
        self.assertTrue(eth["media_disconnected"])

    def test_empty_input(self):
        self.assertEqual(cr.parse_ipconfig(""), [])


class TestDedup(unittest.TestCase):
    def test_dedup_keeps_order(self):
        self.assertEqual(cr._dedup(["b", "a", "b", "c", "a"]), ["b", "a", "c"])


# ══════════════════════════════════════════════════════ ② 计划
class TestBuildPlan(unittest.TestCase):
    TARGETS = ["198.51.100.1", "198.51.100.2", "198.51.100.3"]

    def test_factory_radar_gets_one_by_one_warning(self):
        """出厂默认雷达必须带「一台一台改」的告警（同 IP 会互相抢包）"""
        items, errs, notes = cr.build_plan([_radar("192.168.1.201")], self.TARGETS)
        self.assertEqual(errs, [])
        self.assertTrue(any("一台一台" in n for n in notes), notes)

    def test_single_factory_radar_goes_to_first_free_target(self):
        items, errs, notes = cr.build_plan([_radar("192.168.1.201")], self.TARGETS)
        self.assertEqual(errs, [])
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["host"], "192.168.1.201")
        self.assertEqual(items[0]["target"], "198.51.100.1")
        self.assertFalse(items[0].get("skip"))
        # 位号是现场知识 → 工具必须把"可能选错位"讲出来
        self.assertTrue(any("--ip" in n for n in notes), notes)

    def test_single_mover_single_free_target_no_ambiguity_note(self):
        found = [_radar("192.168.1.201"), _radar("198.51.100.1")]
        items, errs, notes = cr.build_plan(found, ["198.51.100.1", "198.51.100.2"])
        self.assertEqual(errs, [])
        self.assertFalse(any("检测位" in n for n in notes))
        moved = [i for i in items if not i.get("skip")]
        self.assertEqual(moved[0]["target"], "198.51.100.2")

    def test_radar_already_on_target_is_skipped(self):
        found = [_radar("198.51.100.1"), _radar("198.51.100.2")]
        items, errs, _ = cr.build_plan(found, ["198.51.100.1", "198.51.100.2"])
        self.assertEqual(errs, [])
        self.assertTrue(all(i.get("skip") for i in items))
        notes = cr.build_plan(found, ["198.51.100.1", "198.51.100.2"])[2]
        self.assertTrue(any("无需改动" in n for n in notes))

    def test_more_movers_than_free_targets_is_an_error(self):
        # 3 台待改、只剩 1 个空位 → 必须报错，不能猜
        found = [_radar("198.51.100.1"),
                 _radar("192.168.1.201"), _radar("10.0.0.9"), _radar("10.0.0.10")]
        items, errs, _ = cr.build_plan(found, ["198.51.100.1", "198.51.100.2"])
        self.assertTrue(errs)
        self.assertIn("空闲目标", " ".join(errs))
        self.assertIn("--map", " ".join(errs))      # 必须给出逃生路径

    def test_no_targets_conflict(self):
        # 目标里如果有重复，也会被 map 分支挡住
        items, errs, _ = cr.build_plan([_radar("192.168.1.201")],
                                       ["198.51.100.1"],
                                       [("192.168.1.201", "198.51.100.2"),
                                        ("192.168.1.201", "198.51.100.2")])
        self.assertTrue(errs)
        self.assertIn("重复", " ".join(errs))

    def test_explicit_map_happy_path(self):
        found = [_radar("192.168.1.201"), _radar("10.0.0.5")]
        items, errs, _ = cr.build_plan(
            found, self.TARGETS,
            [("192.168.1.201", "198.51.100.3"), ("10.0.0.5", "198.51.100.4")])
        self.assertEqual(errs, [])
        got = {i["host"]: i["target"] for i in items}
        self.assertEqual(got["192.168.1.201"], "198.51.100.3")
        self.assertEqual(got["10.0.0.5"], "198.51.100.4")

    def test_explicit_map_missing_radar(self):
        items, errs, _ = cr.build_plan(
            [_radar("192.168.1.201")], self.TARGETS,
            [("192.168.1.201", "198.51.100.1"), ("172.16.0.9", "198.51.100.2")])
        self.assertTrue(errs)
        self.assertIn("未发现雷达 172.16.0.9", " ".join(errs))

    def test_explicit_map_invalid_target(self):
        items, errs, _ = cr.build_plan(
            [_radar("192.168.1.201")], self.TARGETS,
            [("192.168.1.201", "999.1.1.1")])
        self.assertTrue(errs)

    def test_explicit_map_duplicate_target(self):
        items, errs, _ = cr.build_plan(
            [_radar("192.168.1.201"), _radar("10.0.0.5")], self.TARGETS,
            [("192.168.1.201", "198.51.100.9"), ("10.0.0.5", "198.51.100.9")])
        self.assertTrue(errs)
        self.assertIn("重复", " ".join(errs))

    def test_empty_found(self):
        items, errs, _ = cr.build_plan([], self.TARGETS)
        self.assertEqual(items, [])
        self.assertEqual(errs, [])


# ══════════════════════════════════════════════════════ ② 回写 channels.json
class TestSyncChannels(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="cfgchan_")
        self.path = os.path.join(self.dir, "channels.json")
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump(_channels_json(["192.168.1.201", "192.168.1.201",
                                      "192.168.1.201", "192.168.1.201"]), f)

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def _items(self, pairs):
        out = []
        for old, new in pairs:
            out.append({"host": old, "target": new,
                        "radar": {"ip": old, "inv": {"serial_number": "S-" + old},
                                  "cfg": {"spin_rate": 10}}})
        return out

    def test_writes_ip_and_source_ip_with_backup(self):
        items = self._items([("192.168.1.201", f"198.51.100.{i}") for i in (1, 2, 3, 4)])
        changes, err = cr.sync_channels(items, self.path)
        self.assertEqual(err, "")
        self.assertTrue(changes)
        data = _read_json(self.path)
        got = [(c["lidar"]["ip"], c["lidar"]["source_ip"]) for c in data["channels"]]
        self.assertEqual(got, [("198.51.100.1", "198.51.100.1"),
                               ("198.51.100.2", "198.51.100.2"),
                               ("198.51.100.3", "198.51.100.3"),
                               ("198.51.100.4", "198.51.100.4")])
        baks = [f for f in os.listdir(self.dir) if ".bak-" in f]
        self.assertEqual(len(baks), 1, "必须留一份备份")

    def test_backup_content_is_original(self):
        cr.sync_channels(self._items([("192.168.1.201", "198.51.100.1")]), self.path)
        bak = [f for f in os.listdir(self.dir) if ".bak-" in f][0]
        old = _read_json(os.path.join(self.dir, bak))
        self.assertEqual(old["channels"][0]["lidar"]["source_ip"], "192.168.1.201")

    def test_frame_ms_untouched_by_default(self):
        # ★ 转速单位未确证 —— 默认绝不能顺手改 frame_ms
        items = self._items([("192.168.1.201", "198.51.100.1")])
        cr.sync_channels(items, self.path)
        data = _read_json(self.path)
        self.assertEqual(data["channels"][0]["lidar"]["frame_ms"], 100)

    def test_frame_ms_opt_in(self):
        items = self._items([("192.168.1.201", "198.51.100.1")])
        cr.sync_channels(items, self.path, frame_ms=True)
        data = _read_json(self.path)
        self.assertEqual(data["channels"][0]["lidar"]["frame_ms"], 100)  # 1000/10

    def test_frame_ms_skipped_for_absurd_spin_rate(self):
        items = self._items([("192.168.1.201", "198.51.100.1")])
        items[0]["radar"]["cfg"]["spin_rate"] = 0        # 不合理 → 不写
        changes, err = cr.sync_channels(items, self.path, frame_ms=True)
        self.assertEqual(err, "")
        data = _read_json(self.path)
        self.assertEqual(data["channels"][0]["lidar"]["frame_ms"], 100)
        self.assertTrue(any("未改动" in c for c in changes))

    def test_dry_run_writes_nothing(self):
        items = self._items([("192.168.1.201", "198.51.100.1")])
        changes, err = cr.sync_channels(items, self.path, dry_run=True)
        self.assertEqual(err, "")
        self.assertTrue(changes)
        data = _read_json(self.path)
        self.assertEqual(data["channels"][0]["lidar"]["ip"], "192.168.1.201")
        self.assertEqual([f for f in os.listdir(self.dir) if ".bak-" in f], [])

    def test_matches_by_target_ip_too(self):
        # 配置里已经写成新 IP 时，也应该能对上（按 target 键命中）
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump(_channels_json(["198.51.100.1", "198.51.100.2"]), f)
        items = self._items([("192.168.1.201", "198.51.100.1")])
        changes, err = cr.sync_channels(items, self.path, dry_run=True)
        self.assertEqual(err, "")
        self.assertEqual(changes, [])       # 已经是目标值 → 没有改动

    def test_no_match_no_change(self):
        items = self._items([("172.16.9.9", "198.51.100.99")])
        changes, err = cr.sync_channels(items, self.path)
        self.assertEqual((changes, err), ([], ""))
        self.assertEqual([f for f in os.listdir(self.dir) if ".bak-" in f], [])

    def test_skip_items_ignored(self):
        items = [{"host": "198.51.100.1", "target": "198.51.100.1",
                  "radar": {"ip": "198.51.100.1", "cfg": {}, "inv": {}},
                  "skip": True}]
        changes, err = cr.sync_channels(items, self.path)
        self.assertEqual((changes, err), ([], ""))

    def test_missing_file_reports_error(self):
        changes, err = cr.sync_channels(self._items([("a", "b")]),
                                        os.path.join(self.dir, "nope.json"))
        self.assertEqual(changes, [])
        self.assertTrue(err)


# ══════════════════════════════════════════════════════ ② channels 读取
class TestChannelTargets(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="cfgchan2_")
        self.path = os.path.join(self.dir, "channels.json")
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump(_channels_json(["198.51.100.1", "198.51.100.2",
                                      "198.51.100.3", "198.51.100.4"]), f)

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def test_channel_targets(self):
        t = cr.channel_targets(self.path)
        self.assertEqual([x["id"] for x in t], ["CH1", "CH2", "CH3", "CH4"])
        self.assertEqual([x["udp_port"] for x in t], [2368] * 4)
        self.assertEqual(t[3]["source_ip"], "198.51.100.4")

    def test_desired_targets(self):
        self.assertEqual(cr.desired_targets(self.path),
                         ["198.51.100.1", "198.51.100.2",
                          "198.51.100.3", "198.51.100.4"])

    def test_source_ip_falls_back_to_ip(self):
        with open(self.path, "w", encoding="utf-8") as f:
            d = _channels_json(["10.0.0.1"])
            del d["channels"][0]["lidar"]["source_ip"]
            json.dump(d, f)
        self.assertEqual(cr.desired_targets(self.path), ["10.0.0.1"])

    def test_disabled_channel_skipped(self):
        with open(self.path, "w", encoding="utf-8") as f:
            d = _channels_json(["10.0.0.1", "10.0.0.2"])
            d["channels"][1]["enabled"] = False
            json.dump(d, f)
        self.assertEqual(cr.desired_targets(self.path), ["10.0.0.1"])

    def test_real_project_channels_has_four(self):
        """项目自带的 channels.json 必须声明 4 条通道，且 4 条都满足分流前提

        ★ 量的是**声明**的 4 条，不是**启用**的那几条。
          现场会临时停用某条车道（2026-09-19 停了 CH1），拿 channel_targets()
          量就会跟着部署形态一起翻红；更要紧的是，停用的那条反而**一条都不检查了** ——
          而它重新启用时同样要靠源 IP 分流，同样不能撞端口/撞 IP。
          所以共用端口与 source_ip 唯一这两条，对 4 条一起验。
        """
        if not os.path.isfile(cr.DEFAULT_CHANNELS):
            self.skipTest("没有项目配置")
        chans = cr.load_channels(cr.DEFAULT_CHANNELS).get("channels", [])
        self.assertEqual(len(chans), 4)
        lidars = [c.get("lidar") or {} for c in chans]
        udps = {int(l.get("udp_port") or 2368) for l in lidars}
        self.assertEqual(len(udps), 1, "4 路必须共用同一个 UDP 端口（靠源 IP 分流）")
        srcs = {l.get("source_ip") or l.get("ip") for l in lidars}
        self.assertEqual(len(srcs), 4, "source_ip 必须唯一")
        # 启用的那几条是声明的子集，且至少还剩一条 —— 全停用等于整套系统不工作
        on = cr.channel_targets(cr.DEFAULT_CHANNELS)
        self.assertTrue(on, "config/channels.json 里一条启用的通道都没有")
        self.assertLessEqual(len(on), 4)
        self.assertTrue({x["source_ip"] for x in on} <= srcs)


# ══════════════════════════════════════════════════════ ② 下发命令文本
class TestProvisionCmdline(unittest.TestCase):
    def test_cmdline_contents(self):
        it = {"host": "192.168.1.201", "target": "198.51.100.1", "radar": {}}
        s = cr.provision_cmdline(it, "198.51.100.15", 2368, 90)
        self.assertIn("-H 192.168.1.201", s)
        self.assertIn("provision", s)
        self.assertIn("--ip 198.51.100.1", s)
        self.assertIn("--dest-ip 198.51.100.15", s)
        self.assertIn("--dest-port 2368", s)
        self.assertTrue(s.rstrip().endswith("-y"))

    def test_cmdline_without_dest(self):
        it = {"host": "192.168.1.201", "target": "198.51.100.1", "radar": {}}
        s = cr.provision_cmdline(it, "", 0, 0)
        self.assertNotIn("--dest-ip", s)


class TestApplyPlanDryRun(unittest.TestCase):
    def test_dry_run_sends_nothing(self):
        items = [{"host": "192.168.1.201", "target": "198.51.100.1",
                  "radar": _radar("192.168.1.201")}]
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            results, failures = cr.apply_plan(items, dest_ip="198.51.100.15",
                                              dry_run=True)
        self.assertEqual(failures, 0)
        self.assertEqual(len(results), 1)
        self.assertTrue(results[0]["dry"])
        self.assertIn("dry-run", buf.getvalue())

    def test_dry_run_skips_skip_items(self):
        items = [{"host": "198.51.100.1", "target": "198.51.100.1",
                  "radar": _radar("198.51.100.1"), "skip": True}]
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            results, failures = cr.apply_plan(items, dry_run=True)
        self.assertEqual((results, failures), ([], 0))


class TestPreflightModeWording(unittest.TestCase):
    """preflight 的 mode 参数必须真的生效 —— 曾被同名局部变量覆盖。

    事故：preflight() 里读网卡时写了一句
        mode = ("动态 DHCP" if sc.get("dhcp_enabled") else "静态")   # ← 同名！
    把刚加进来的 mode 参数（status / apply 语境）覆盖成 "静态"，
    于是 cmd_apply 走的是 status 分支，打出：
        × 当前有 4 项阻塞，先解决再执行下发：
        （以上仅为提示，继续执行；确认无误可忽略）      ← 自相矛盾
    且"不拦截却打出阻塞"会让现场以为必须先解决网络才能动雷达。
    参数名与局部名重名属于最难发现的一类错误（不报错、只是行为悄悄变）。
    """

    def _run(self, mode="__default__"):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            if mode == "__default__":
                cr.preflight()
            else:
                cr.preflight(mode=mode)
        return buf.getvalue()

    def test_apply_mode_says_notice_not_blocker(self):
        out = self._run("apply")
        self.assertIn("需要留意", out)
        self.assertNotIn("项阻塞", out,
                         "apply 语境下不该说『阻塞』，它是提示不是拦截")

    def test_status_mode_says_blocker(self):
        out = self._run("status")
        self.assertIn("项阻塞", out)
        self.assertNotIn("需要留意", out)

    def test_default_mode_is_status(self):
        self.assertIn("项阻塞", self._run())

    def test_mode_param_is_not_shadowed(self):
        """结构性守卫：preflight 体内不得再给 mode 赋值。"""
        import inspect
        import re
        src = inspect.getsource(cr.preflight)
        # 去掉签名行（含续行），只看函数体
        body = src.split("-> list:", 1)[1]
        bad = [ln.strip() for ln in body.splitlines()
               if re.match(r"^\s*mode\s*=[^=]", ln)]
        self.assertEqual(
            bad, [],
            "preflight 里又给 mode 赋值了，会覆盖 status/apply 语境参数；"
            f"请改用别的局部名（如 dhcp_mode）：{bad}")


# ══════════════════════════════════════════════════════ ③ 真实网络（本地 mock）
class TestScanAndIdentify(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.radar = MockRadar(ip="192.168.1.201", serial="SN-TEST-1")

    @classmethod
    def tearDownClass(cls):
        cls.radar.stop()

    def test_scan_finds_local_mock(self):
        found, total = cr.scan(["127.0.0.1/32"], self.radar.port,
                               timeout=0.5, workers=4, quiet=True)
        self.assertEqual(found, ["127.0.0.1"])
        self.assertEqual(total, 1)

    def test_scan_misses_closed_port(self):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        dead_port = s.getsockname()[1]
        s.close()
        found, _ = cr.scan(["127.0.0.1/32"], dead_port, timeout=0.4,
                           workers=2, quiet=True)
        self.assertEqual(found, [])

    def test_identify_reads_config_and_inventory(self):
        r = cr.identify("127.0.0.1", self.radar.port, timeout=3.0)
        self.assertEqual(r["error"], "")
        self.assertEqual(r["cfg"]["ip"], "192.168.1.201")
        self.assertEqual(r["cfg"]["subnet_mask"], "255.255.255.0")
        self.assertEqual(r["inv"]["serial_number"], "SN-TEST-1")

    def test_identify_reports_error_when_down(self):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        dead_port = s.getsockname()[1]
        s.close()
        r = cr.identify("127.0.0.1", dead_port, timeout=0.5)
        self.assertTrue(r["error"])

    def test_identify_all_sorts(self):
        out = cr.identify_all(["127.0.0.1"], self.radar.port, timeout=3.0)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0]["ip"], "127.0.0.1")

    def test_cmd_scan_cli(self):
        args = cr.build_parser().parse_args(
            ["--port", str(self.radar.port), "--scan", "127.0.0.1/32", "scan"])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = cr.cmd_scan(args)
        out = buf.getvalue()
        self.assertEqual(rc, 0)
        self.assertIn("发现 1 台雷达", out)
        self.assertIn("SN-TEST-1", out)

    def test_cmd_scan_cli_json(self):
        args = cr.build_parser().parse_args(
            ["--port", str(self.radar.port), "--scan", "127.0.0.1/32",
             "scan", "--json"])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = cr.cmd_scan(args)
        self.assertEqual(rc, 0)
        data = json.loads(buf.getvalue())
        self.assertEqual(data[0]["ip"], "127.0.0.1")
        self.assertEqual(data[0]["inv"]["serial_number"], "SN-TEST-1")

    def test_cmd_scan_cli_no_radar_returns_2(self):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        dead_port = s.getsockname()[1]
        s.close()
        args = cr.build_parser().parse_args(
            ["--port", str(dead_port), "--timeout", "0.3",
             "--scan", "127.0.0.1/32", "scan"])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = cr.cmd_scan(args)
        self.assertEqual(rc, 2)
        self.assertIn("没发现", buf.getvalue())


class TestMockRadarWriteChain(unittest.TestCase):
    """验证 mock 自身能正确反映 0x21 写入 —— 否则上面那些测试都是假绿。"""

    def setUp(self):
        self.radar = MockRadar(ip="192.168.1.201")

    def tearDown(self):
        self.radar.stop()

    def test_set_self_updates_and_tail_is_000001(self):
        from tools.hr40_ip import HR40Client
        with HR40Client("127.0.0.1", self.radar.port) as c:
            c.set_self_network("198.51.100.9", "255.255.255.0", "198.51.100.1")
        self.assertEqual(self.radar.ip, "198.51.100.9")
        self.assertEqual(self.radar.self_tail, b"\x00\x00\x01")

    def test_set_dest_updates(self):
        from tools.hr40_ip import HR40Client
        with HR40Client("127.0.0.1", self.radar.port) as c:
            c.set_destination("198.51.100.15", 2368)
        self.assertEqual(self.radar.dest_ip, "198.51.100.15")
        self.assertEqual(self.radar.dest_port, 2368)


class TestIcmpAlive(unittest.TestCase):
    def test_loopback_alive(self):
        self.assertTrue(cr.icmp_alive("127.0.0.1", 500))

    def test_unreachable_not_alive(self):
        # 文档网段可能被代理或测试网络接管；测试不可达回执，不依赖外部路由。
        with mock.patch.object(cr, "run_cmd", return_value=(0, "Destination host unreachable")) as command:
            self.assertFalse(cr.icmp_alive("192.0.2.123", 400))
        command.assert_called_once()


class TestHostSideHelpers(unittest.TestCase):
    def test_is_admin_returns_bool(self):
        self.assertIsInstance(cr.is_admin(), bool)

    @unittest.skipUnless(os.name == "nt", "Windows 专用")
    def test_list_interfaces_has_entries(self):
        ifs = cr.list_interfaces()
        self.assertTrue(ifs, "netsh 应该能列出网卡")
        for i in ifs:
            self.assertIn(i["status"], ("connected", "disconnected"))
            self.assertTrue(i["name"])

    @unittest.skipUnless(os.name == "nt", "Windows 专用")
    def test_iface_static_config_of_loopback(self):
        sc = cr.iface_static_config("Loopback Pseudo-Interface 1")
        # 回环没有 IP 地址行也没关系，关键是别抛异常、结构完整
        self.assertIsInstance(sc, dict)

    def test_auto_scan_cidrs_uses_extra(self):
        self.assertEqual(cr.auto_scan_cidrs("", ["10.1.2.0/24", "10.1.2.0/24"]),
                         ["10.1.2.0/24"])

    def test_blocked_nic_change_without_admin(self):
        """非管理员跑 nic add-temp 必须返回 3 且不发 netsh（除非本来就提权了）"""
        if cr.is_admin():
            self.skipTest("当前进程已是管理员")
        args = cr.build_parser().parse_args(
            ["nic", "add-temp", "--iface", "以太网", "--ip", "192.168.1.250"])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = cr.cmd_nic(args)
        self.assertEqual(rc, 3)
        self.assertIn("管理员", buf.getvalue())

    def test_nic_requires_iface(self):
        args = cr.build_parser().parse_args(["nic", "add-temp", "--ip", "1.2.3.4"])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = cr.cmd_nic(args)
        self.assertEqual(rc, 4)

    def test_nic_rejects_bad_ip(self):
        args = cr.build_parser().parse_args(
            ["nic", "add-temp", "--iface", "X", "--ip", "999.1.1.1"])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = cr.cmd_nic(args)
        self.assertEqual(rc, 4)


class TestParserSurface(unittest.TestCase):
    def test_all_subcommands_present(self):
        ap = cr.build_parser()
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            with self.assertRaises(SystemExit):
                ap.parse_args(["--help"])
        for name in ("status", "scan", "apply", "verify", "sync", "nic"):
            self.assertIn(name, buf.getvalue())

    def test_apply_defaults(self):
        a = cr.build_parser().parse_args(["apply"])
        self.assertEqual(a.ip, [])
        self.assertEqual(a.map, [])
        self.assertEqual(a.dest_port, 2368)
        self.assertFalse(a.yes)
        self.assertFalse(a.dry_run)

    def test_map_parsing(self):
        a = cr.build_parser().parse_args(
            ["apply", "--map", "192.168.1.201=198.51.100.4"])
        self.assertEqual(cr._parse_maps(a.map),
                         [("192.168.1.201", "198.51.100.4")])

    def test_map_parsing_rejects_garbage(self):
        with self.assertRaises(ValueError):
            cr._parse_maps(["198.51.100.4"])

    def test_nic_action_choices(self):
        a = cr.build_parser().parse_args(["nic", "list"])
        self.assertEqual(a.nic_action, "list")


if __name__ == "__main__":
    unittest.main(verbosity=2)
