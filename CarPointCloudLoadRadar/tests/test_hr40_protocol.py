#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_hr40_protocol.py — HR40 雷达配置协议（HR40改IP.exe 逆向）回归测试

用一个本地 mock 雷达服务验证 tools/hr40_ip.py 的组帧/解析与设备字节布局一致。
不依赖任何硬件。
"""
from __future__ import annotations

import contextlib
import io
import os
import shutil
import socket
import struct
import sys
import tempfile
import threading
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from tools.hr40_ip import (          # noqa: E402
    HR40Client, build_frame, ProtocolError, main, provision,
    HDR1, HDR2, CMD_GET_ANGLE, CMD_GET_INVENTORY, CMD_GET_CONFIG,
    CMD_RESTART, CMD_SET_DEST, CMD_SET_SELF, CMD_SET_BLOCKAGE, _ip4,
    _in_subnet, _is_any_broadcast, _dest_cannot_receive, _local_addr_in_subnet,
)


# ------------------------------------------------------------------ mock 雷达
def _mk_config_payload(dest_ip: str = "192.168.1.100",
                       ip: str = "192.168.1.201",
                       mask: str = "255.255.255.0",
                       gateway: str = "192.168.1.1",
                       dest_port: int = 2368) -> bytes:
    """构造 38 字节配置 payload（按逆向出的布局）。

    各字段可覆盖 —— 配合有状态 MockRadar 模拟"写入后用 read 读回新值"。
    """
    p = bytearray()
    p += bytes(_ip4(ip))                  # 0   自身 IP
    p += bytes(_ip4(mask))                # 4   掩码
    p += bytes(_ip4(gateway))             # 8   网关
    p += bytes(_ip4(dest_ip))             # 12  目的 IP
    p += struct.pack(">H", dest_port)     # 16  目的端口
    p += b"\x00\x00"                      # 18  保留
    p += struct.pack(">H", 200)           # 20  转速（原始值）
    p += bytes([1])                       # 22  同步使能
    p += struct.pack(">H", 18000)         # 23  同步角度
    p += b"\x00" * 4                      # 25  保留
    p += bytes([2])                       # 29  时钟源
    p += b"\x00"                          # 30  保留
    p += bytes([1])                       # 31  触发方式
    p += bytes([3])                       # 32  回波模式
    p += bytes([0])                       # 33  待机模式
    p += b"\x00"                          # 34  保留
    p += bytes([0])                       # 35  VLAN 标志
    p += struct.pack(">H", 0)             # 36  VLAN ID
    assert len(p) == 38, len(p)
    return bytes(p)


def _mk_inventory_payload() -> bytes:
    """构造 120 字节设备清单 payload"""

    def fixed(s: str, n: int) -> bytes:
        b = s.encode("ascii")
        return (b + b"\x00" * n)[:n]

    p = bytearray()
    p += fixed("SN1234567890ABCDEF", 18)      # 0
    p += fixed("HR40-128", 32)                # 18
    p += fixed("2024-06-01", 16)              # 50
    p += bytes([0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E])  # 66 MAC
    p += fixed("v1.2.3", 16)                  # 72
    p += b"\x00" * 16                         # 88 保留
    p += fixed("v0.9.1", 16)                  # 104 控制器固件
    assert len(p) == 120, len(p)
    return bytes(p)


class MockRadar:
    """只会说 HR40 协议的最小 TCP 服务"""

    def __init__(self, dest_ip: str = "192.168.1.100"):
        self._default_dest_ip = dest_ip
        self.reset()
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(4)
        self.port = self.srv.getsockname()[1]
        self.received = []          # 记录 (cmd, payload)
        self._stop = False
        self._t = threading.Thread(target=self._serve, daemon=True)
        self._t.start()

    def reset(self):
        """恢复初始状态。

        MockRadar **是有状态**的（会记住 0x21/0x20 写入的值，模拟真实设备的
        "写入后 read 能读回新值"）。因为测试类共享同一个 mock 实例，
        必须在每个测试前重置，否则前一个测试写的 IP 会污染后面的断言。
        """
        self.ip = "192.168.1.201"
        self.mask = "255.255.255.0"
        self.gateway = "192.168.1.1"
        self.dest_ip = self._default_dest_ip
        self.dest_port = 2368
        # 置 True 可模拟"设备报成功但没写进去"（ReturnCode 校验松的真实故障）
        self.drop_dest_write = False

    def _serve(self):
        while not self._stop:
            try:
                conn, _ = self.srv.accept()
            except OSError:
                return
            threading.Thread(target=self._handle, args=(conn,), daemon=True).start()

    def _read_exact(self, conn, n):
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
                h1, h2, cmd, _code = head[0], head[1], head[2], head[3]
                (plen,) = struct.unpack(">I", head[4:8])
                body = self._read_exact(conn, plen) if plen else b""
                self.received.append((cmd, body))
                if h1 != HDR1 or h2 != HDR2:
                    return

                if cmd == CMD_GET_CONFIG:
                    resp = _mk_config_payload(self.dest_ip, self.ip, self.mask,
                                              self.gateway, self.dest_port)
                elif cmd == CMD_GET_INVENTORY:
                    resp = _mk_inventory_payload()
                elif cmd == CMD_GET_ANGLE:
                    resp = b"Channel,0,0\r\n0,0.01,0.02\r\n1,0.03,0.04\r\n"
                elif cmd == CMD_SET_SELF and len(body) >= 12:
                    # 有状态：记住写入的自身网络，重启后 read 应读回新值
                    self.ip = ".".join(str(b) for b in body[0:4])
                    self.mask = ".".join(str(b) for b in body[4:8])
                    self.gateway = ".".join(str(b) for b in body[8:12])
                    resp = b""
                elif cmd == CMD_SET_DEST and len(body) >= 6:
                    if not self.drop_dest_write:
                        self.dest_ip = ".".join(str(b) for b in body[0:4])
                        self.dest_port = struct.unpack(">H", body[4:6])[0]
                    resp = b""
                else:
                    resp = b""      # 其它写命令 / 重启：空应答
                conn.sendall(build_frame(cmd, resp))
        except OSError:
            pass
        finally:
            conn.close()

    def stop(self):
        self._stop = True
        try:
            self.srv.close()
        except OSError:
            pass


# ------------------------------------------------------------------ 测试
class TestFrame(unittest.TestCase):
    def test_frame_layout(self):
        f = build_frame(0x08)
        self.assertEqual(f[:4], bytes([0x47, 0x74, 0x08, 0x00]))
        self.assertEqual(struct.unpack(">I", f[4:8])[0], 0)
        self.assertEqual(len(f), 8)

    def test_frame_with_payload(self):
        f = build_frame(0x21, b"\x01\x02\x03")
        self.assertEqual(struct.unpack(">I", f[4:8])[0], 3)
        self.assertEqual(f[8:], b"\x01\x02\x03")

    def test_payload_len_is_big_endian(self):
        f = build_frame(0x21, b"\x00" * 15)
        self.assertEqual(f[4:8], b"\x00\x00\x00\x0f")


class TestAgainstMock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = MockRadar()

    @classmethod
    def tearDownClass(cls):
        cls.srv.stop()

    def setUp(self):
        # mock 是有状态的（会记住 0x21/0x20 写入的 IP）→ 每个测试前必须重置，
        # 否则前一个测试写入的地址会污染后面的断言。
        self.srv.reset()

    def _client(self):
        return HR40Client("127.0.0.1", self.srv.port)

    def test_read_config_roundtrip(self):
        with self._client() as c:
            cfg = c.get_config()
        self.assertEqual(cfg["ip"], "192.168.1.201")
        self.assertEqual(cfg["subnet_mask"], "255.255.255.0")
        self.assertEqual(cfg["gateway"], "192.168.1.1")
        self.assertEqual(cfg["dest_ip"], "192.168.1.100")
        self.assertEqual(cfg["dest_port"], 2368)
        self.assertEqual(cfg["spin_rate"], 200)
        self.assertEqual(cfg["sync_angle"], 18000)
        self.assertEqual(cfg["clock_source"], 2)
        self.assertEqual(cfg["trigger_method"], 1)
        self.assertEqual(cfg["return_mode"], 3)
        self.assertTrue(cfg["sync_enable"])
        self.assertEqual(cfg["vlan_id"], 0)
        self.assertEqual(cfg["_payload_len"], 38)

    def test_inventory_roundtrip(self):
        with self._client() as c:
            inv = c.get_inventory()
        self.assertEqual(inv["serial_number"], "SN1234567890ABCDEF")
        self.assertEqual(inv["part_number"], "HR40-128")
        self.assertEqual(inv["date_of_manufacture"], "2024-06-01")
        self.assertEqual(inv["mac_address"], "00:1A:2B:3C:4D:5E")
        self.assertEqual(inv["software_version"], "v1.2.3")
        self.assertEqual(inv["controller_fw_version"], "v0.9.1")

    def test_angle_csv(self):
        with self._client() as c:
            csv = c.get_angle_csv()
        self.assertIn("Channel", csv)
        self.assertIn("0.01", csv)

    def test_set_self_network_wire_format(self):
        """15 字节载荷：IP(4)+掩码(4)+网关(4)+00 00 01（末字节是常量 1）"""
        with self._client() as c:
            c.set_self_network("10.0.0.5", "255.255.0.0", "10.0.0.1")
        cmd, body = self.srv.received[-1]
        self.assertEqual(cmd, CMD_SET_SELF)
        self.assertEqual(len(body), 15)
        self.assertEqual(body[:4], bytes([10, 0, 0, 5]))
        self.assertEqual(body[4:8], bytes([255, 255, 0, 0]))
        self.assertEqual(body[8:12], bytes([10, 0, 0, 1]))
        # 原厂 IL：payload[12]=0 payload[13]=0 payload[14]=**1**（硬编码常量，不是保留 0）
        self.assertEqual(body[12:], b"\x00\x00\x01")

    def test_set_destination_wire_format(self):
        """8 字节载荷：目的IP(4)+端口(2 大端)+0x0000"""
        with self._client() as c:
            c.set_destination("239.0.0.1", 2368)
        cmd, body = self.srv.received[-1]
        self.assertEqual(cmd, CMD_SET_DEST)
        self.assertEqual(len(body), 8)
        self.assertEqual(body[:4], bytes([239, 0, 0, 1]))
        self.assertEqual(body[4:6], b"\x09\x40")      # 2368 = 0x0940 大端
        self.assertEqual(body[6:], b"\x00\x00")

    def test_blockage_and_restart(self):
        with self._client() as c:
            c.set_blockage_detection(True)
            c.set_blockage_detection(False)
            c.restart()
        self.assertEqual(self.srv.received[-3], (CMD_SET_BLOCKAGE, b"\x01"))
        self.assertEqual(self.srv.received[-2], (CMD_SET_BLOCKAGE, b"\x00"))
        self.assertEqual(self.srv.received[-1], (CMD_RESTART, b""))

    def test_bad_ip_rejected_before_send(self):
        with self._client() as c:
            with self.assertRaises(ValueError):
                c.set_self_network("192.168.1", "255.255.255.0", "192.168.1.1")
            with self.assertRaises(ValueError):
                c.set_self_network("192.168.1.999", "255.255.255.0", "192.168.1.1")

    def test_fault_code_raises(self):
        """设备返回非 0 故障码必须抛错（对应原版 '参考文档 1.4.2'）"""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        port = srv.getsockname()[1]

        def bad():
            conn, _ = srv.accept()
            conn.recv(8)
            conn.sendall(bytes([HDR1, HDR2, 0x08, 0x05]) + struct.pack(">I", 0))
            conn.close()
            srv.close()

        threading.Thread(target=bad, daemon=True).start()
        with HR40Client("127.0.0.1", port) as c:
            with self.assertRaises(ProtocolError) as cm:
                c.get_config()
        self.assertIn("故障码", str(cm.exception))

    def test_header_mismatch_raises(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        port = srv.getsockname()[1]

        def bad():
            conn, _ = srv.accept()
            conn.recv(8)
            conn.sendall(bytes([0x00, 0x00, 0x08, 0x00]) + struct.pack(">I", 0))
            conn.close()
            srv.close()

        threading.Thread(target=bad, daemon=True).start()
        with HR40Client("127.0.0.1", port) as c:
            with self.assertRaises(ProtocolError) as cm:
                c.get_config()
        self.assertIn("协议头", str(cm.exception))


class TestCli(unittest.TestCase):
    """命令行层回归 —— 覆盖 argparse dest 冲突一类的问题。

    历史 bug：set-dest 子命令曾用 --port，与顶层 -p/--port(TCP端口) 共用 dest，
    子解析器的 default=2368 覆盖了顶层端口 → 连错端口。直接调 client 的测试
    抓不到，必须走 main()。
    """

    def setUp(self):
        self.srv = MockRadar()

    def tearDown(self):
        self.srv.stop()

    def _run(self, *argv, host="127.0.0.1"):
        import contextlib
        import io
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = main(["-H", host, "-p", str(self.srv.port), *argv])
        self._last_out = buf.getvalue()
        return rc

    def test_tcp_port_not_clobbered_by_subcommand(self):
        """顶层 -p 必须生效：所有子命令都要打到 mock 的端口上"""
        for argv in (["read"], ["inventory"], ["angle"], ["restart"],
                     ["set-self", "--ip", "10.0.0.1",
                      "--mask", "255.255.255.0", "--gateway", "10.0.0.254"],
                     ["set-dest", "--ip", "10.0.0.2", "--dest-port", "2369"],
                     ["blockage", "on"]):
            with self.subTest(argv=argv[0]):
                self.srv.received.clear()
                self._run(*argv)
                self.assertTrue(
                    self.srv.received,
                    f"{argv[0]} 未打到 mock 端口（-p 被覆盖？）")

    def _last_cmd(self, code):
        """取最后一次指定命令码的 (cmd, payload)。set-dest 现在写完会回读，
        所以不能用 received[-1]（那会是回读用的 0x08）。"""
        hits = [(c, b) for c, b in self.srv.received if c == code]
        self.assertTrue(hits, f"mock 未收到命令码 0x{code:02X}")
        return hits[-1]

    def test_set_dest_cli_payload(self):
        self._run("set-dest", "--ip", "239.1.2.3", "--dest-port", "2368")
        cmd, body = self._last_cmd(CMD_SET_DEST)
        self.assertEqual(body[:4], bytes([239, 1, 2, 3]))
        self.assertEqual(body[4:6], b"\x09\x40")     # 2368 大端
        self.assertEqual(body[6:], b"\x00\x00")      # 尾部 2 字节

    def test_set_dest_cli_default_port(self):
        """不给 --dest-port 时应为 2368"""
        self._run("set-dest", "--ip", "10.1.1.1")
        _, body = self._last_cmd(CMD_SET_DEST)
        self.assertEqual(body[4:6], b"\x09\x40")

    def test_set_dest_reads_back_and_leaves_self_ip_alone(self):
        """★ 0x20 写完必须回读校验，且只动"目的网络"、绝不改自身 IP。

        （现场问过："是不是改单播把网络搞不通了？" —— 这条测试就是答案：
          0x20 的载荷只有 8 字节目的地址，回读能证明自身 IP 原封不动。）
        """
        before_ip = self.srv.ip
        rc = self._run("set-dest", "--ip", "198.51.100.15", "--dest-port", "2368")
        self.assertEqual(rc, 0)
        codes = [c for c, _ in self.srv.received]
        i_set = codes.index(CMD_SET_DEST)
        self.assertIn(CMD_GET_CONFIG, codes[i_set + 1:],
                      "set-dest 之后没有回读校验")
        self.assertEqual(self.srv.dest_ip, "198.51.100.15")
        self.assertEqual(self.srv.ip, before_ip, "0x20 竟然改动了自身 IP！")
        self.assertIn("已回读确认", self._last_out)

    def test_set_dest_detects_silent_noop(self):
        """★ 设备"报成功但没生效"必须被发现 —— 这正是现场"改不了"的一种真因。

        设备对 ReturnCode 校验很松，载荷写错会静默不生效。没有回读校验时，
        工具会骗你说"修改成功"，现场就会误判成"网络有问题"。
        """
        self.srv.drop_dest_write = True
        rc = self._run("set-dest", "--ip", "198.51.100.15", "--dest-port", "2368")
        self.assertEqual(rc, 4, "回读不一致时必须返回非 0")
        self.assertIn("回读不一致", self._last_out)
        self.assertNotEqual(self.srv.dest_ip, "198.51.100.15")

    def test_set_self_cli_payload(self):
        self._run("set-self", "--ip", "198.51.100.5",
                  "--mask", "255.255.255.0", "--gateway", "198.51.100.1")
        cmd, body = self.srv.received[-1]
        self.assertEqual(cmd, CMD_SET_SELF)
        self.assertEqual(body, bytes([192, 168, 201, 5,
                                      255, 255, 255, 0,
                                      192, 168, 201, 1, 0, 0, 1]))

    def test_set_self_requires_mask_and_gateway(self):
        """★ set-self 必须显式给掩码/网关：不能静默兜底 192.168.1.1 这种默认值"""
        for missing in (["--mask", "255.255.255.0"], ["--gateway", "10.0.0.254"]):
            with self.subTest(missing=missing[0]):
                with self.assertRaises(SystemExit):        # argparse 报错退出
                    self._run("set-self", "--ip", "10.0.0.1", *missing)
        self.assertEqual(self.srv.received, [])

    def test_all_command_codes_reach_device(self):
        """把预期命令码全跑一遍，确证命令码取值"""
        self._run("read")
        self._run("inventory")
        self._run("angle")
        self._run("set-self", "--ip", "10.0.0.1",
                  "--mask", "255.255.255.0", "--gateway", "10.0.0.254")
        self._run("set-dest", "--ip", "10.0.0.2")
        self._run("blockage", "off")
        self._run("restart")
        codes = [c for c, _ in self.srv.received]
        self.assertEqual(codes, [CMD_GET_CONFIG, CMD_GET_INVENTORY, CMD_GET_ANGLE,
                                 CMD_SET_SELF, CMD_SET_DEST, CMD_GET_CONFIG,
                                 CMD_SET_BLOCKAGE, CMD_RESTART])


class TestProvision(unittest.TestCase):
    """provision 一步到位改 IP：读→存档→写→重启→(等)→验证"""

    def setUp(self):
        self.srv = MockRadar()
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        self.srv.stop()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _run(self, *argv, host="127.0.0.1"):
        import contextlib
        import io
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = main(
                ["-H", host, "-p", str(self.srv.port), "provision", *argv])
        self.out = buf.getvalue()
        self._last_out = self.out
        return rc

    def test_command_order_read_set_restart(self):
        """必须按 读配置→清单→写自身网络→重启 的顺序发命令"""
        self._run("--ip", "198.51.100.7", "--mask", "255.255.255.0",
                  "--gateway", "198.51.100.1",
                  "--archive-dir", self.tmp, "--wait", "0", "-y")
        codes = [c for c, _ in self.srv.received]
        self.assertEqual(codes, [CMD_GET_CONFIG, CMD_GET_INVENTORY,
                                 CMD_SET_SELF, CMD_RESTART])

    def test_restart_is_sent_to_old_ip(self):
        """重启必须发往旧 IP（重启后才切到新 IP）—— 这里 mock 就在旧地址上，
        四条命令都收到即证明重启没有发去新 IP。"""
        self._run("--ip", "198.51.100.7", "--archive-dir", self.tmp,
                  "--wait", "0", "-y")
        self.assertIn(CMD_RESTART, [c for c, _ in self.srv.received])

    def test_archive_written_before_change(self):
        self._run("--ip", "198.51.100.7", "--archive-dir", self.tmp,
                  "--wait", "0", "-y")
        files = os.listdir(self.tmp)
        self.assertEqual(len(files), 1)
        with open(os.path.join(self.tmp, files[0]), encoding="utf-8") as fh:
            txt = fh.read()
        self.assertIn("subnet_mask = 255.255.255.0", txt)
        self.assertIn("inventory.serial_number", txt)

    def test_verify_failure_is_graceful(self):
        """新 IP 连不上时应返回 2 并给出提示，而不是抛异常"""
        rc = self._run("--ip", "198.51.100.7", "--archive-dir", self.tmp,
                       "--wait", "0", "-y")
        self.assertEqual(rc, 2)
        self.assertIn("无法连接到新 IP", self.out)

    def test_same_ip_is_noop(self):
        """新 IP == 旧 IP 时不做任何操作"""
        rc = self._run("--ip", "127.0.0.1", "--archive-dir", self.tmp,
                       "--wait", "0", "-y")
        self.assertEqual(rc, 1)
        self.assertEqual(self.srv.received, [])

    def test_bad_ip_rejected_locally(self):
        """非法 IP 应在联网前就被拒"""
        with self.assertRaises(ValueError):
            self._run("--ip", "192.168.201", "--archive-dir", self.tmp,
                      "--wait", "0", "-y")
        self.assertEqual(self.srv.received, [])

    # ------------------------------------------------ 掩码/网关沿用原值
    def test_mask_gateway_kept_when_omitted(self):
        """★ mask/gateway 不填时必须沿用雷达当前值，不能静默填 192.168.1.1 之类默认值"""
        self._run("--ip", "198.51.100.7", "--archive-dir", self.tmp,
                  "--wait", "0", "-y")
        cmd, body = [r for r in self.srv.received if r[0] == CMD_SET_SELF][-1]
        self.assertEqual(cmd, CMD_SET_SELF)
        # mock 雷达当前 掩码=255.255.255.0 网关=192.168.1.1
        self.assertEqual(body[4:8], bytes([255, 255, 255, 0]))
        self.assertEqual(body[8:12], bytes([192, 168, 1, 1]))

    def test_mask_gateway_overridable(self):
        self._run("--ip", "198.51.100.7", "--mask", "255.255.0.0",
                  "--gateway", "198.51.100.1",
                  "--archive-dir", self.tmp, "--wait", "0", "-y")
        _, body = [r for r in self.srv.received if r[0] == CMD_SET_SELF][-1]
        self.assertEqual(body[4:8], bytes([255, 255, 0, 0]))
        self.assertEqual(body[8:12], bytes([192, 168, 201, 1]))

    def test_bad_mask_rejected_locally(self):
        with self.assertRaises(ValueError):
            self._run("--ip", "198.51.100.7", "--mask", "255.255.255",
                      "--archive-dir", self.tmp, "--wait", "0", "-y")
        self.assertEqual([c for c, _ in self.srv.received], [])

    # ------------------------------------------------ 目的 IP
    def test_dest_not_touched_when_omitted(self):
        """不给 --dest-ip 时不得发送 0x20"""
        self._run("--ip", "198.51.100.7", "--archive-dir", self.tmp,
                  "--wait", "0", "-y")
        self.assertNotIn(CMD_SET_DEST, [c for c, _ in self.srv.received])

    def test_dest_set_after_self_takes_effect(self):
        """★ 0x20 必须在 0x21 生效（重启 + 回读成功）**之后**才发。

        实测（2026-09-13 现场，第二台雷达）：同一次连接里先发 0x21 再发 0x20，
        结果是 0x20 生效、**0x21 被冲掉** —— 设备按"当前生效配置"整体回存，
        把刚写入的新自身 IP 又覆盖回旧值。现场现象是"重启完 IP 根本没变、
        目的 IP 却改了"，极具误导性。
        所以正确顺序必须是：0x21 → 0x10(重启) → 回读 → 0x20。

        这里用有状态 mock（会记住 0x21 写入的自身 IP）走通完整流程；
        host 用 localhost、新 IP 用 127.0.0.1，以绕过"新旧 IP 相同则跳过"的检查。
        """
        rc = self._run("--ip", "127.0.0.1", "--dest-ip", "127.0.0.15",
                       "--dest-port", "2368", "--archive-dir", self.tmp,
                       "--wait", "0", "-y", host="localhost")
        self.assertEqual(rc, 0, f"应成功，实际 rc={rc}\n{self._last_out}")
        codes = [c for c, _ in self.srv.received]
        self.assertIn(CMD_SET_SELF, codes)
        self.assertIn(CMD_SET_DEST, codes)
        # ★ 核心顺序断言：0x21 先于重启；0x20 必须在重启**之后**，不能紧跟 0x21
        self.assertLess(codes.index(CMD_SET_SELF), codes.index(CMD_RESTART))
        self.assertLess(codes.index(CMD_RESTART), codes.index(CMD_SET_DEST))
        _, body = [r for r in self.srv.received if r[0] == CMD_SET_DEST][-1]
        self.assertEqual(body[:4], bytes([127, 0, 0, 15]))
        self.assertEqual(body[4:6], b"\x09\x40")

    def test_warns_when_dest_out_of_subnet(self):
        """★ 目的 IP 不在新网段内必须告警：否则改完 IP 收不到点云却不报错

        mock 的当前目的 IP 是 192.168.1.100；目标 198.51.100.7/24 → 跨网段。
        """
        self._run("--ip", "198.51.100.7", "--archive-dir", self.tmp,
                  "--wait", "0", "-y")
        self.assertIn("不在新网段", self.out)
        self.assertIn("--dest-ip", self.out)

    def test_no_warn_when_dest_same_subnet(self):
        """目标同网段时不该有跨网段告警"""
        self._run("--ip", "192.168.1.7", "--archive-dir", self.tmp,
                  "--wait", "0", "-y")
        self.assertNotIn("不在新网段", self.out)

    def test_bad_dest_ip_rejected_locally(self):
        with self.assertRaises(ValueError):
            self._run("--ip", "198.51.100.7", "--dest-ip", "10.0.0",
                      "--archive-dir", self.tmp, "--wait", "0", "-y")
        self.assertEqual([c for c, _ in self.srv.received], [])

    def test_bad_dest_port_rejected_locally(self):
        with self.assertRaises(ValueError):
            self._run("--ip", "198.51.100.7", "--dest-port", "70000",
                      "--archive-dir", self.tmp, "--wait", "0", "-y")
        self.assertEqual([c for c, _ in self.srv.received], [])


class TestSubnetHelpers(unittest.TestCase):
    def test_in_subnet(self):
        from tools.hr40_ip import _in_subnet
        self.assertTrue(_in_subnet("192.168.1.5", "192.168.1.201", "255.255.255.0"))
        self.assertFalse(_in_subnet("198.51.100.4", "192.168.1.201", "255.255.255.0"))


class TestBroadcastDestHelpers(unittest.TestCase):
    """★ 广播目的地址不该被判成"跨网段、点云发不出去"。

    背景（2026-09-13 现场实测）：雷达改到 198.51.100.4、目的 IP 保持出厂默认的
    255.255.255.255，旧的 _in_subnet 判据把它当成"不在新网段"并打印告警；
    而实际点云 8971 包/s 全部正常到达本机 198.51.100.15 —— 纯误报。
    这类误报会让人以为"改完收不到数据"而做出错误处置（回滚、乱改目的 IP）。
    """

    NEW = "198.51.100.4"
    MASK = "255.255.255.0"

    def test_limited_broadcast_is_broadcast(self):
        self.assertTrue(_is_any_broadcast("255.255.255.255"))

    def test_limited_broadcast_never_warned(self):
        self.assertFalse(_dest_cannot_receive("255.255.255.255", self.NEW, self.MASK))

    def test_directed_broadcast_of_new_subnet(self):
        """本网段定向广播 198.51.100.255（主机位全 1）"""
        self.assertTrue(_is_any_broadcast("198.51.100.255", self.NEW, self.MASK))
        self.assertFalse(_dest_cannot_receive("198.51.100.255", self.NEW, self.MASK))

    def test_directed_broadcast_bitmath_direction(self):
        """★ 反证位运算方向：错误公式 b|m 会把网络位全置 1（得出 255.255.1.5 这种
        废地址）。正确公式是 b|(~m)。这里用 /16 掩码把方向钉死。
        """
        self.assertTrue(_is_any_broadcast("192.168.255.255", "192.168.1.5", "255.255.0.0"))
        # 255.255.1.5 正是 "b|m" 会误算出来的值，必须不是广播
        self.assertFalse(_is_any_broadcast("255.255.1.5", "192.168.1.5", "255.255.0.0"))

    def test_other_subnet_directed_broadcast_still_warns(self):
        """别的网段的广播，雷达确实发不过去 → 仍要告警"""
        self.assertFalse(_is_any_broadcast("192.168.1.255", self.NEW, self.MASK))
        self.assertTrue(_dest_cannot_receive("192.168.1.255", self.NEW, self.MASK))

    def test_unicast_cross_subnet_still_warns(self):
        """普通跨网段单播必须继续告警 —— 别把真 bug 一起修没了"""
        self.assertTrue(_dest_cannot_receive("192.168.1.100", self.NEW, self.MASK))
        self.assertTrue(_dest_cannot_receive("8.8.8.8", self.NEW, self.MASK))

    def test_unicast_same_subnet_no_warn(self):
        self.assertFalse(_dest_cannot_receive("198.51.100.15", self.NEW, self.MASK))

    def test_local_addr_in_subnet_empty_for_foreign_net(self):
        """本机不可能持有 TEST-NET-3 地址 → 返回空串且不抛异常"""
        self.assertEqual(_local_addr_in_subnet("203.0.113.0", "255.255.255.0"), "")

    def test_malformed_ip_is_not_broadcast(self):
        for bad in ("", "10.0.0", "a.b.c.d", "999.1.1.1"):
            self.assertFalse(_is_any_broadcast(bad, self.NEW, self.MASK))


class TestProvisionBroadcastDest(unittest.TestCase):
    """走完整 provision 流程，确认广播目的 IP 不再触发误报告警。"""

    @classmethod
    def setUpClass(cls):
        cls.srv = MockRadar(dest_ip="255.255.255.255")

    @classmethod
    def tearDownClass(cls):
        cls.srv.stop()

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="hr40bcast_")
        self.out = io.StringIO()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_broadcast_dest_no_false_warning(self):
        with contextlib.redirect_stdout(self.out):
            provision("127.0.0.1", "198.51.100.4", mask="255.255.255.0",
                      gateway="198.51.100.1", port=self.srv.port,
                      archive_dir=self.tmp, wait_s=0, assume_yes=True)
        text = self.out.getvalue()
        self.assertNotIn("不在新网段", text)
        self.assertNotIn("点云发不出去", text)
        # 写自身网络 + 重启两条命令必须照发（广播不该影响改 IP 本身）
        codes = [c for c, _ in self.srv.received]
        self.assertIn(CMD_SET_SELF, codes)
        self.assertIn(CMD_RESTART, codes)
        self.assertTrue(_in_subnet("198.51.100.4", "192.168.1.201", "255.255.0.0"))

    def test_in_subnet_tolerates_garbage(self):
        from tools.hr40_ip import _in_subnet
        self.assertFalse(_in_subnet("", "192.168.1.201", "255.255.255.0"))
        self.assertFalse(_in_subnet("x.y.z.w", "192.168.1.201", "255.255.255.0"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
