#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/configure_radars.py — 4 台雷达「一把配好」现场工具（纯标准库，Windows / 麒麟通用）

解决的问题
──────────
`tools/hr40_ip.py` 已经能对**一台**雷达完成「改 IP + 重启 + 验证」。
但现场要配 4 台，真正麻烦的不是发命令，而是这三件事：

  ① **先找到雷达**。雷达出厂默认都是同一个 `192.168.1.201`，
     接上网线后你并不知道它在哪、有几台在线、序列号是多少。
  ② **改完连不上**。改 IP 后必须用新地址复验，中途还得让电脑
     同时待在「旧网段」和「新网段」——手动改网卡极易漏步骤。
  ③ **收不到点云**。雷达把点云发给**目的 IP**，只改自身 IP 不改目的 IP，
     现象是"TCP 能连、命令都对、服务器一个点都没有"，极具迷惑性。

本工具把这三件事串成一条命令：
    体检 → (可选)加临时网段地址 → 扫描发现 → 读取识别 → 生成计划
         → 逐台下发(含目的IP) → 复验 → 回写 config/channels.json

────────────────────────────────────────────────────────────────────────
用法
────────────────────────────────────────────────────────────────────────
（下面示例按 Linux/麒麟 写 `python3`；在 Windows 现场请改用 `python`。
  工具自己打印出来的"等价命令"会按当前平台自动选对解释器名。）

    # 0. 先看现场状态（不发任何写命令，安全）
    python3 tools/configure_radars.py status

    # 1. 扫描发现（不改任何东西）
    python3 tools/configure_radars.py scan

    # 2. 从零配置 4 台（每台雷达出厂默认 192.168.1.201）
    #    —— 需要先让电脑能到 192.168.1.x，见下面 --temp-ip
    python3 tools/configure_radars.py nic list
    python3 tools/configure_radars.py nic add-temp --iface 以太网 --ip 192.168.1.250
    python3 tools/configure_radars.py scan
    python3 tools/configure_radars.py apply --dest-ip 198.51.100.15 -y

    # 3. 只配其中一台（雷达已上电、线已插好）
    python3 tools/configure_radars.py apply --ip 198.51.100.2 --dest-ip 198.51.100.15 -y

    # 4. 演练：只打印将要执行的命令，不碰雷达
    python3 tools/configure_radars.py apply --dest-ip 198.51.100.15 --dry-run

    # 5. 收尾：确认 4 台都在，并回写 channels.json
    python3 tools/configure_radars.py verify
    python3 tools/configure_radars.py sync

    # 6. 用完删掉临时地址
    python3 tools/configure_radars.py nic del-temp --iface 以太网 --ip 192.168.1.250

────────────────────────────────────────────────────────────────────────
★ 出厂默认 IP 相同的陷阱
────────────────────────────────────────────────────────────────────────
4 台雷达出厂都是 `192.168.1.201`。**同一个网段里不能有两台设备用同一个 IP**，
所以正确姿势是**一台一台来**：

    插第 1 台 → apply --ip 198.51.100.1 → 改完它会变成新 IP，不再冲突
    插第 2 台 → apply --ip 198.51.100.2 → …
    插第 3 台 → apply --ip 198.51.100.3
    插第 4 台 → apply --ip 198.51.100.4

如果 4 台同时上电且都还是出厂默认，扫描只会看到一个 `192.168.1.201`
（IP 冲突导致谁都不稳定），本工具会**明确告警并拒绝自动分配**，
不会猜"这台是几号雷达"。

────────────────────────────────────────────────────────────────────────
返回码
────────────────────────────────────────────────────────────────────────
    0 成功 / 2 没发现雷达 / 3 需要管理员权限 / 4 计划有冲突(拒绝执行)
    5 复验不一致 / 6 下发失败
"""
from __future__ import annotations

import argparse
import concurrent.futures
import ctypes
import json
import os
import re
import socket
import subprocess
import sys
import time
import unicodedata

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from tools.hr40_ip import (                                    # noqa: E402
    DEFAULT_TCP_PORT,
    HR40Client,
    ProtocolError,
    _in_subnet,
    _ip4,
    _route_source_for,
    _dest_cannot_receive,
    provision,
)

# 出厂默认地址（4 台出厂都是它）
FACTORY_IP = "192.168.1.201"
# 电脑在旧网段用的临时地址（.250 是常见空闲段位，且避开可能的相机 .100）
FACTORY_TEMP_IP = "192.168.1.250"

DEFAULT_CHANNELS = os.path.join(_ROOT, "config", "channels.json")

# 打印给现场看的「等价命令」一律用本平台的解释器名：
#   Windows 只有 python.exe（没有 python3，敲了会说"找不到"）
#   麒麟 / 其他 Linux 上 python3 才是正规叫法
# 之前在 Windows 上把提示写成 python3，现场照着抄会直接失败。
PYEXE = "python" if os.name == "nt" else "python3"

# 一次扫描最多展开多少个地址 —— 防止有人手滑传 /16 把内网扫爆
SCAN_CAP = 4096


# ══════════════════════════════════════════════════════════ 基础小工具
def is_admin() -> bool:
    """当前进程是否有管理员权限（netsh 改网卡必需）"""
    try:
        return bool(ctypes.windll.shell32.IsUserAnAdmin())        # type: ignore[attr-defined]
    except AttributeError:
        # 非 Windows（麒麟等）走 uid 判断
        try:
            return os.geteuid() == 0
        except AttributeError:
            return False


def _dec(b: bytes) -> str:
    """子进程输出解码：先 UTF-8 再 GBK。

    中文 Windows 的 netsh/ipconfig 输出是**系统代码页(GBK)**，
    直接 utf-8 解会抛 UnicodeDecodeError；而日志/新版工具又可能是 UTF-8。
    所以按顺序试，最后兜底 latin-1（永不抛，至少不丢信息）。
    """
    for enc in ("utf-8", "gbk", "cp936", "latin-1"):
        try:
            return b.decode(enc)
        except (UnicodeDecodeError, LookupError):
            continue
    return b.decode("latin-1", errors="replace")


def run_cmd(cmd, timeout: float = 25.0) -> tuple:
    """跑一条外部命令；返回 (rc, 合并输出)。失败不抛，返回 rc=-1。"""
    try:
        p = subprocess.run(cmd, capture_output=True, timeout=timeout, shell=False)
    except (OSError, subprocess.TimeoutExpired) as e:
        return -1, f"{type(e).__name__}: {e}"
    return p.returncode, _dec(p.stdout) + _dec(p.stderr)


def _disp_w(s: str) -> int:
    """字符串显示宽度（中日韩字符按 2 列算），用于手写表格对齐。"""
    w = 0
    for ch in s:
        w += 2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
    return w


def pad(s: str, width: int, align: str = "<") -> str:
    """按显示宽度补空格（不用 str.ljust，否则中文表格会歪）"""
    s = "" if s is None else str(s)
    gap = width - _disp_w(s)
    if gap <= 0:
        return s
    return s + " " * gap if align == "<" else " " * gap + s


# ══════════════════════════════════════════════════════════ 网卡信息
_RE_IF_ROW = re.compile(
    r"^\s*(\d+)\s+(\d+)\s+(\d+)\s+(connected|disconnected)\s+(.+?)\s*$",
    re.IGNORECASE)
_RE_ADAPTER_HEAD = re.compile(r"^[^\s].*?(?:适配器|adapter)[^:]*:\s*$", re.IGNORECASE)
_RE_IPV4 = re.compile(r"\b(\d{1,3}(?:\.\d{1,3}){3})\b")


def list_interfaces() -> list:
    """`netsh interface ipv4 show interfaces` → [{index,name,metric,status}]

    为什么用 netsh 而不是 ipconfig 取状态：netsh 的 connected/disconnected
    这两个值是**枚举名、不随语言变化**（中文系统上也是英文），可以直接匹配；
    而 ipconfig 的"媒体已断开连接"是翻译过的文字，脆弱。
    """
    rc, out = run_cmd(["netsh", "interface", "ipv4", "show", "interfaces"])
    if rc != 0:
        return []
    rows = []
    for line in out.splitlines():
        m = _RE_IF_ROW.match(line)
        if m:
            rows.append({
                "index": int(m.group(1)),
                "metric": int(m.group(2)),
                "mtu": int(m.group(3)),
                "status": m.group(4).lower(),
                "name": m.group(5).strip(),
            })
    return rows


def parse_ipconfig(text: str) -> list:
    """解析 ipconfig 输出 → [{name, ip, mask, gateway, media_disconnected, apipa}]

    按**块**解析（每块 = 一个适配器）。标签同时兼容中英文：
      IPv4 地址 / IPv4 Address / IP Address / IP 地址
      子网掩码   / Subnet Mask
      媒体已断开 / Media disconnected
    """
    blocks = []
    cur = None
    for raw in text.splitlines():
        line = raw.rstrip()
        if not line.strip():
            continue
        if _RE_ADAPTER_HEAD.match(line):
            head = line.strip().rstrip(":")
            for kw in ("适配器", "Adapter", "adapter"):
                if kw in head:
                    head = head.rsplit(kw, 1)[-1]
                    break
            cur = {"name": head.strip(), "ip": "", "mask": "", "gateway": "",
                   "media_disconnected": False, "apipa": False}
            blocks.append(cur)
            continue
        if cur is None:
            continue

        s = line.strip()
        # ★ 注意两边都要小写：拿 "Media disconnected" 去 in s.lower() 永远不匹配
        if ("媒体已断开" in s) or ("media disconnected" in s.lower()):
            cur["media_disconnected"] = True
            continue

        low = s.lower()
        is_ipv4_line = ("ipv4" in low) or re.search(r"(^|\s)IP\s*地址", s)
        if is_ipv4_line and ("地址" in s or "address" in low):
            m = _RE_IPV4.search(s)
            if m:
                val = m.group(1)
                # 「自动配置 IPv4 地址」= 169.254.x.x APIPA，不是有效配置
                if "自动配置" in s or "autoconfiguration" in low:
                    cur["apipa"] = True
                    if not cur["ip"]:
                        cur["ip"] = val
                elif not cur["ip"]:
                    cur["ip"] = val
            continue
        if ("子网掩码" in s) or ("subnet mask" in low):
            m = _RE_IPV4.search(s)
            if m and not cur["mask"]:
                cur["mask"] = m.group(1)
            continue
        if ("默认网关" in s) or ("default gateway" in low):
            m = _RE_IPV4.search(s)
            if m and not cur["gateway"]:
                cur["gateway"] = m.group(1)
            continue
    return blocks


def mask_to_prefix(mask: str) -> int:
    """255.255.255.0 → 24；非法返回 -1"""
    try:
        n = int.from_bytes(bytes(_ip4(mask)), "big")
    except (ValueError, AttributeError):
        return -1
    bits = f"{n:032b}"
    return bits.count("1") if "01" not in bits else -1


def local_networks(iface: str = "") -> list:
    """本机所有**已连接且拿到有效 IPv4** 的网段 → [{name,ip,prefix,cidr,mask}]

    iface 非空时只保留该网卡。APIPA(169.254) 与断开的网卡一律排除。
    """
    rc, out = run_cmd(["ipconfig"])
    if rc != 0:
        return []
    ifaces = {i["name"]: i for i in list_interfaces()}

    nets = []
    for b in parse_ipconfig(out):
        if not b["ip"] or b["apipa"] or not b["mask"]:
            continue
        if b["ip"].startswith("169.254."):
            continue
        if iface and b["name"] != iface:
            continue
        p = mask_to_prefix(b["mask"])
        if p < 0:
            continue
        nets.append({
            "name": b["name"], "ip": b["ip"], "mask": b["mask"],
            "prefix": p, "cidr": f"{b['ip']}/{p}", "gateway": b.get("gateway", ""),
            "status": ifaces.get(b["name"], {}).get("status", "?"),
        })
    return nets


def iface_status(name: str) -> str:
    """网卡状态：connected / disconnected / ?"""
    for i in list_interfaces():
        if i["name"] == name:
            return i["status"]
    return "?"


_RE_MASK_PAREN = re.compile(r"[（(]\s*(?:掩码|mask)\s*[:：]?\s*(\d{1,3}(?:\.\d{1,3}){3})", re.I)


def iface_static_config(name: str) -> dict:
    """读网卡的**持久化**配置（`netsh interface ipv4 show config`）。

    ★ 为什么不能只看 ipconfig：
      网卡**媒体断开**时，ipconfig 根本不会列出它的 IPv4 地址，
      但 netsh 读的是注册表里的持久化配置 —— 静态 IP 依然在。
      本项目当前现场就是这种情况（「以太网」断开但仍静态写着 198.51.100.15），
      漏掉它会导致"网段检查通过、实际完全不通"的误判。
    """
    rc, out = run_cmd(["netsh", "interface", "ipv4", "show", "config",
                       f"name={name}"])
    if rc != 0:
        return {}
    r = {"name": name, "ip": "", "mask": "", "prefix": -1, "gateway": "",
         "dhcp_enabled": None}
    for line in out.splitlines():
        s = line.strip()
        low = s.lower()
        if ("dhcp" in low) and ("启用" in s or "enabled" in low):
            # 中文「DHCP 已启用: 否」 / 英文「DHCP enabled: No」
            neg = ("否" in s) or bool(re.search(r":\s*(no|false)\b", low))
            r["dhcp_enabled"] = not neg
            continue
        if s.startswith("IP 地址") or s.startswith("IP Address") \
                or s.startswith("IP 地址:") or "IP Address" in s:
            if "子网" in s or "Subnet" in s:
                continue
            m = _RE_IPV4.search(s)
            if m and not r["ip"]:
                r["ip"] = m.group(1)
            continue
        if ("掩码" in s) or ("mask" in low):
            m = _RE_MASK_PAREN.search(s)
            if m and not r["mask"]:
                r["mask"] = m.group(1)
            m2 = re.search(r"/(\d{1,2})\b", s)
            if m2:
                r["prefix"] = int(m2.group(1))
            continue
        if ("默认网关" in s) or ("default gateway" in low):
            m = _RE_IPV4.search(s)
            if m and not r["gateway"]:
                r["gateway"] = m.group(1)
            continue
    if r["mask"] and r["prefix"] < 0:
        r["prefix"] = mask_to_prefix(r["mask"])
    return r


# ══════════════════════════════════════════════════════════ 扫描
def cidr_hosts(cidr: str, cap: int = SCAN_CAP) -> list:
    """把 CIDR 展开成主机地址列表。

    只接受 IPv4 且带前缀长度（如 192.168.1.0/24）；不带前缀会报错 ——
    静默按 /24 补全是本项目明令禁止的"静默默认值"。
    """
    if "/" not in cidr:
        raise ValueError(f"网段必须带前缀长度，例如 192.168.1.0/24（收到 {cidr!r}）")
    net_s, pfx_s = cidr.split("/", 1)
    net = _ip4(net_s)
    try:
        prefix = int(pfx_s)
    except ValueError:
        raise ValueError(f"前缀长度非法：{pfx_s!r}") from None
    if not 0 <= prefix <= 32:
        raise ValueError(f"前缀长度越界：{prefix}")

    total = 2 ** (32 - prefix)
    if total > cap:
        raise ValueError(
            f"{cidr} 含 {total} 个地址，超过单次扫描上限 {cap}。"
            f"请拆成更小的网段（如按 /24 分别扫描）。")

    base = int.from_bytes(bytes(net), "big")
    mask_int = (((1 << prefix) - 1) << (32 - prefix)) if prefix else 0
    net_int = base & mask_int
    if prefix >= 31:
        idxs = range(total)
    else:
        idxs = range(1, total - 1)          # 去掉网络号与广播地址
    return [str(_int_to_ip(net_int + i)) for i in idxs]


def _int_to_ip(n: int) -> str:
    return ".".join(str((n >> s) & 0xFF) for s in (24, 16, 8, 0))


def _ip_key(ip: str) -> tuple:
    try:
        return tuple(int(x) for x in ip.split("."))
    except ValueError:
        return (999, 999, 999, 999)


def tcp_open(ip: str, port: int, timeout: float = 0.6) -> bool:
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect((ip, port))
        return True
    except OSError:
        return False
    finally:
        try:
            s.close()
        except OSError:
            pass


def scan(cidrs, port: int = DEFAULT_TCP_PORT, timeout: float = 0.6,
         workers: int = 160, quiet: bool = False):
    """并发 TCP 扫描，返回 (发现的IP列表_已排序, 扫过的地址数)"""
    hosts, seen = [], set()
    for c in cidrs:
        for h in cidr_hosts(c):
            if h not in seen:
                seen.add(h)
                hosts.append(h)

    if not quiet:
        print(f"  扫描 {len(hosts)} 个地址，端口 {port}，并发 {workers} …")
    found = []
    t0 = time.time()
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
        futs = {ex.submit(tcp_open, h, port, timeout): h for h in hosts}
        for f in concurrent.futures.as_completed(futs):
            try:
                if f.result():
                    found.append(futs[f])
            except Exception:                                     # noqa: BLE001
                pass
    found.sort(key=_ip_key)
    if not quiet:
        print(f"  完成，用时 {time.time() - t0:.1f}s，发现 {len(found)} 台")
    return found, len(hosts)


class _FastClient(HR40Client):
    """把连接/收发超时收紧的客户端。

    hr40_ip 的默认超时是 10s（面向"确定在线的一台雷达"），
    扫描/批量时遇到半死不活的地址会拖很久，所以这里单独压到 3~4s。
    """

    def __init__(self, host, port, timeout=4.0):
        super().__init__(host, port)
        self.timeout = timeout

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return self


def identify(ip: str, port: int = DEFAULT_TCP_PORT, timeout: float = 4.0) -> dict:
    """读一台雷达的配置 + 设备清单。失败时 error 字段带原因。"""
    r = {"ip": ip, "cfg": {}, "inv": {}, "error": ""}
    try:
        with _FastClient(ip, port, timeout) as c:
            r["cfg"] = c.get_config()
            try:
                r["inv"] = c.get_inventory()
            except (OSError, ProtocolError) as e:
                r["inv"] = {"_err": f"{type(e).__name__}: {e}"}
    except (OSError, ProtocolError, ValueError) as e:
        r["error"] = f"{type(e).__name__}: {e}"
    return r


def identify_all(ips, port: int = DEFAULT_TCP_PORT, timeout: float = 4.0) -> list:
    """并发读取多台雷达（读操作，安全，不改任何配置）"""
    out = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, len(ips))) as ex:
        for r in ex.map(lambda i: identify(i, port, timeout), ips):
            out.append(r)
    out.sort(key=lambda r: _ip_key(r["ip"]))
    return out


# ══════════════════════════════════════════════════════════ 通道配置
def load_channels(path: str = DEFAULT_CHANNELS) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def channel_targets(path: str = DEFAULT_CHANNELS) -> list:
    """从 channels.json 读出「目标 IP 表」→ [(cid, ip, source_ip, tcp_port)]"""
    data = load_channels(path)
    out = []
    for ch in data.get("channels", []):
        if not ch.get("enabled", True):
            continue
        lid = ch.get("lidar") or {}
        out.append({
            "id": ch.get("id", "?"),
            "name": ch.get("name", ""),
            "ip": lid.get("ip", ""),
            "source_ip": lid.get("source_ip", "") or lid.get("ip", ""),
            "tcp_port": int(lid.get("tcp_port") or DEFAULT_TCP_PORT),
            "udp_port": int(lid.get("udp_port") or 2368),
        })
    return out


def desired_targets(path: str = DEFAULT_CHANNELS) -> list:
    """期望的 4 个目标地址（去重、保序）"""
    seen, out = set(), []
    for t in channel_targets(path):
        v = t["source_ip"] or t["ip"]
        if v and v not in seen:
            seen.add(v)
            out.append(v)
    return out


# ══════════════════════════════════════════════════════════ 计划
def build_plan(found, targets, mapping=None) -> tuple:
    """生成变更计划 → (items, errors, notes)

    items: [{"host": 雷达当前IP, "target": 目标IP, "radar": 识别结果}]
    errors: 致命问题（非空则拒绝执行）
    notes: 提示信息

    ★ 设计原则：**绝不猜**。任何"数量对不上""目标被占用""两台雷达同 IP"
      都当成错误返回，让人来决定，而不是自动挑一台改掉。
    """
    items, errors, notes = [], [], []

    by_ip = {}
    for r in found:
        by_ip.setdefault(r["ip"], []).append(r)
    for ip, group in by_ip.items():
        if len(group) > 1:
            notes.append(f"{ip} 上读到 {len(group)} 条记录（同一 IP），已按一条处理")

    if mapping:
        # ---- 显式映射：完全按人给的来
        used_targets = set()
        for old, new in mapping:
            try:
                _ip4(new)
            except ValueError:
                errors.append(f"目标 IP 非法：{new}")
                continue
            if new in used_targets:
                errors.append(f"目标 IP 重复：{new}")
                continue
            radars = [r for r in found if r["ip"] == old]
            if not radars:
                errors.append(f"未发现雷达 {old}（--map {old}={new}），"
                              f"已发现：{', '.join(sorted(by_ip, key=_ip_key)) or '无'}")
                continue
            used_targets.add(new)
            items.append({"host": old, "target": new, "radar": radars[0]})
        return items, errors, notes

    # ---- 自动分配
    occupied = set(by_ip.keys())
    pending = [t for t in targets if t not in occupied]
    movers = [by_ip[ip][0] for ip in by_ip if ip not in targets]
    # 出厂默认的先动（它们最可能是"刚上电的新雷达"）
    movers.sort(key=lambda r: (0 if r["ip"] == FACTORY_IP else 1, _ip_key(r["ip"])))

    already = [by_ip[t][0] for t in targets if t in occupied]

    if not movers:
        notes.append("所有目标地址上都已经是目标雷达，无需改动")
        for r in already:
            items.append({"host": r["ip"], "target": r["ip"], "radar": r, "skip": True})
        return items, errors, notes

    if len(movers) > len(pending):
        errors.append(
            f"发现 {len(movers)} 台待改雷达，但只剩 {len(pending)} 个空闲目标地址。"
            f"待改：{', '.join(r['ip'] for r in movers)}；"
            f"空闲目标：{', '.join(pending) or '无'}。"
            f"请用 --map 旧IP=新IP 明确指定，或一次只配一台。")

    if len(movers) < len(pending):
        # 有多个空位、却只有一台待改雷达 —— 位号只有现场知道，工具不能替他决定。
        notes.append(
            f"发现 {len(movers)} 台待改雷达、{len(pending)} 个空闲目标位，"
            f"本次只补最前面的 {len(movers)} 个：{', '.join(pending[:len(movers)])}。"
            f"若这台雷达其实是别的检测位，请改用 --ip <目标IP> 明确指定。")

    if any(r["ip"] == FACTORY_IP for r in movers):
        notes.append(
            f"有雷达仍在出厂默认 {FACTORY_IP}。★ 出厂默认地址全网唯一，"
            f"**必须一台一台改**：同一网段里同时插两台同 IP 的雷达会互相抢包，"
            f"谁也改不稳。")

    for t, r in zip(pending, movers):
        items.append({"host": r["ip"], "target": t, "radar": r})
    for r in already:
        items.append({"host": r["ip"], "target": r["ip"], "radar": r, "skip": True})
    return items, errors, notes


# ══════════════════════════════════════════════════════════ 展示
def print_radar_table(radars: list):
    if not radars:
        print("  （无）")
        return
    print("  " + pad("当前IP", 16) + pad("掩码", 16) + pad("网关", 16)
          + pad("目的IP", 16) + pad("目的端口", 9) + pad("转速", 6)
          + pad("序列号", 22) + "型号")
    print("  " + "-" * 118)
    for r in radars:
        if r["error"]:
            print("  " + pad(r["ip"], 16) + "× " + r["error"][:90])
            continue
        c, v = r["cfg"], r["inv"]
        print("  " + pad(r["ip"], 16) + pad(c.get("subnet_mask", ""), 16)
              + pad(c.get("gateway", ""), 16) + pad(c.get("dest_ip", ""), 16)
              + pad(c.get("dest_port", ""), 9) + pad(c.get("spin_rate", ""), 6)
              + pad(v.get("serial_number", "-"), 22) + str(v.get("part_number", "-")))


def print_plan(items: list):
    if not items:
        print("  （空）")
        return
    print("  " + pad("当前IP", 16) + pad("", 3) + pad("目标IP", 16)
          + pad("目的IP", 16) + pad("序列号", 22) + "动作")
    print("  " + "-" * 100)
    for it in items:
        r = it["radar"]
        v = r.get("inv") or {}
        act = "跳过（已在目标地址）" if it.get("skip") else "改 IP + 重启 + 验证"
        print("  " + pad(it["host"], 16) + pad("→", 3) + pad(it["target"], 16)
              + pad((r.get("cfg") or {}).get("dest_ip", "-"), 16)
              + pad(v.get("serial_number", "-"), 22) + act)


def provision_cmdline(item, dest_ip, dest_port, wait_s):
    parts = [f"{PYEXE} tools/hr40_ip.py -H {item['host']} provision",
             f"--ip {item['target']}"]
    if dest_ip:
        parts.append(f"--dest-ip {dest_ip}")
    if dest_port:
        parts.append(f"--dest-port {dest_port}")
    if wait_s:
        parts.append(f"--wait {int(wait_s)}")
    parts.append("-y")
    return " \\\n        ".join(parts)


# ══════════════════════════════════════════════════════════ 下发
def apply_plan(items, port=DEFAULT_TCP_PORT, dest_ip=None, dest_port=None,
               archive_dir="", wait_s=90.0, dry_run=False) -> tuple:
    """逐台执行 provision。返回 (结果列表, 失败数)"""
    results = []
    failures = 0
    todo = [it for it in items if not it.get("skip")]
    for n, it in enumerate(todo, 1):
        tag = f"[{n}/{len(todo)}] {it['host']} → {it['target']}"
        print()
        print("─" * 70)
        print(tag)
        print("─" * 70)

        cfg = it["radar"].get("cfg") or {}
        gw = cfg.get("gateway", "")
        mask = cfg.get("subnet_mask", "")
        if gw and mask and not _in_subnet(gw, it["target"], mask):
            print(f"  ▲ 该雷达当前网关 {gw} 不在新网段 {it['target']}/{mask} 内。")
            print( "     本次沿用该网关（不静默改）。若现场确实有路由器，请显式指定 --gateway；")
            print( "     若点云目标与本机同网段（本工具已设置 --dest-ip），网关不参与转发，可忽略。")

        if dry_run:
            print("  [dry-run] 将要执行：")
            print("        " + provision_cmdline(it, dest_ip, dest_port, wait_s))
            results.append({"item": it, "rc": 0, "dry": True})
            continue

        try:
            rc = provision(it["host"], it["target"], None, None,
                           dest_ip=dest_ip, dest_port=dest_port,
                           port=port, archive_dir=archive_dir,
                           wait_s=wait_s, assume_yes=True)
        except (ValueError, OSError, ProtocolError) as e:
            print(f"  × 下发异常：{type(e).__name__}: {e}")
            rc = 6
        results.append({"item": it, "rc": rc, "dry": False})
        if rc != 0:
            failures += 1
    return results, failures


# ══════════════════════════════════════════════════════════ 回写 channels.json
def sync_channels(items, path=DEFAULT_CHANNELS, frame_ms=False,
                  dry_run=False) -> tuple:
    """把改好的 IP 回写 config/channels.json。

    只动 lidar.ip / lidar.source_ip（必要时 lidar.frame_ms）。
    ★ 写前一定备份，因为这是现场唯一一份编排配置。
    ★ frame_ms 默认**不自动改**，而且现在基本用不上了：
      lidar.frame_mode 默认 "rotation"（按方位角整圈切帧），frame_ms 只在
      frame_mode="time" 时才生效。保留 --sync-frame-ms 只是为了兼容老配置。
      需要时显式 --sync-frame-ms，且只在 1..100 这个合理区间内才写。

    返回 (变更说明列表, 错误信息)
    """
    try:
        data = load_channels(path)
    except (OSError, ValueError) as e:
        return [], f"读不到 {path}：{type(e).__name__}: {e}"

    # ★ 必须「逐个消耗」地匹配，不能建成字典去查：
    #   4 条通道在没改之前**可能都写着同一个出厂默认 IP**（192.168.1.201），
    #   用字典查的话 4 条通道会全部命中同一条 map、全被改成同一个地址。
    #   改成"按通道顺序依次领取尚未使用的映射项"才是正确语义。
    pool = [it for it in items if not it.get("skip")]
    taken = set()

    changes = []
    for ch in data.get("channels", []):
        lid = ch.get("lidar")
        if not isinstance(lid, dict):
            continue
        cur_ip, cur_src = lid.get("ip", ""), lid.get("source_ip", "")
        hit, hit_idx = None, -1
        for idx, it in enumerate(pool):
            if idx in taken:
                continue
            keys = (it["target"], it["host"])
            if cur_ip in keys or cur_src in keys:
                hit, hit_idx = it, idx
                break
        if hit is None:
            continue
        taken.add(hit_idx)
        new_ip = hit["target"]
        if cur_ip != new_ip:
            changes.append(f"{ch.get('id')}.lidar.ip: {cur_ip} → {new_ip}")
            lid["ip"] = new_ip
        if cur_src != new_ip:
            changes.append(f"{ch.get('id')}.lidar.source_ip: {cur_src} → {new_ip}")
            lid["source_ip"] = new_ip

        if frame_ms:
            spin = (hit["radar"].get("cfg") or {}).get("spin_rate")
            if isinstance(spin, int) and 1 <= spin <= 100:
                want = int(round(1000.0 / spin))
                if lid.get("frame_ms") != want:
                    changes.append(f"{ch.get('id')}.lidar.frame_ms: "
                                   f"{lid.get('frame_ms')} → {want} (由转速 {spin} Hz 推得)")
                    lid["frame_ms"] = want
            else:
                changes.append(f"{ch.get('id')}.lidar.frame_ms: 未改动"
                               f"（转速 {spin!r} 不在 1..100，无法推算）")

    if not changes:
        return [], ""
    if dry_run:
        return changes, ""

    stamp = time.strftime("%Y%m%d-%H%M%S")
    bak = f"{path}.bak-{stamp}"
    try:
        with open(path, "rb") as src, open(bak, "wb") as dst:
            dst.write(src.read())
    except OSError as e:
        return [], f"备份失败，已中止回写：{type(e).__name__}: {e}"

    try:
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
            f.write("\n")
    except OSError as e:
        return [], f"写入失败（备份在 {bak}）：{type(e).__name__}: {e}"
    return changes, ""


# ══════════════════════════════════════════════════════════ 连通性小工具
def icmp_alive(ip: str, timeout_ms: int = 600) -> bool:
    """ICMP 探测。★ 不能只看 ping 的返回码 ——

    Windows 的 ping 在收到「无法访问目标主机」时**也返回 0**，
    只看 rc 会把"不可达"误判成"在线"。所以这里要求输出里出现 TTL=
    （只有真正的应答报文才有 TTL）。
    """
    rc, out = run_cmd(["ping", "-n", "1", "-w", str(int(timeout_ms)), ip],
                      timeout=max(5.0, timeout_ms / 1000.0 + 3))
    if rc != 0:
        return False
    return "ttl=" in out.lower()


def _dedup(seq) -> list:
    seen, out = set(), []
    for x in seq:
        if x not in seen:
            seen.add(x)
            out.append(x)
    return out


def auto_scan_cidrs(iface: str = "", extra=None) -> list:
    """决定扫哪些网段。

    --scan 给了就用给的（最可靠）；否则取**本机每个已连接网卡的 /24**。
    不自动加 192.168.1.0/24 —— 本机不在那个网段时扫了也是白扫，
    还会让人误以为"扫过了没有雷达"。
    """
    if extra:
        return _dedup(list(extra))
    cidrs = []
    for n in local_networks(iface):
        cidrs.append(f"{n['ip'].rsplit('.', 1)[0]}.0/24")
    return _dedup(cidrs)


def same_slash24(a: str, b: str) -> bool:
    return a.split(".")[:3] == b.split(".")[:3]


# ══════════════════════════════════════════════════════════ 前置体检
def preflight(channels_path=DEFAULT_CHANNELS, iface: str = "", extra_cidrs=None,
              mode: str = "status") -> list:
    """打印现场体检报告，返回 blocker 列表。

    mode 只影响"这批 blocker 怎么称呼"，因为同一批问题在两个子命令里
    的分量不同：
        · mode="status" —— 只是体检，全部算阻塞（告诉你现在配不成）；
        · mode="apply"  —— 正在下发，其中"目标网段不可达"是**正常的**
          （雷达还在出厂网段，本机当然不在新网段），只该提示不该拦。
    不加这个区分的话，会先打一句"× 有 N 项阻塞，先解决再执行"，
    紧接着又说"以上仅为提示，继续执行"，自己打自己脸。
    """
    blockers = []

    print("【1】权限")
    admin = is_admin()
    print("     管理员权限 : " + ("是" if admin else "否"))
    if not admin:
        print("     · 扫描雷达 / 读配置 / 改雷达 IP **都不需要**管理员。")
        print("     · 只有给网卡加临时地址（nic add-temp）需要，届时请以管理员身份运行。")
    print()

    print("【2】网卡（含媒体断开时仍保留在注册表里的静态配置）")
    ifs = list_interfaces()
    if not ifs:
        print("     ▲ 读不到网卡列表（netsh 不可用？）")
        blockers.append("无法读取网卡列表")
    print("     " + pad("状态", 8) + pad("网卡", 22) + pad("静态 IPv4", 17)
          + pad("掩码", 17) + pad("网关", 17) + "模式")
    print("     " + "-" * 92)
    static_cfgs = []
    for i in ifs:
        if "Loopback" in i["name"] or "回环" in i["name"]:
            continue
        tag = {"connected": "● 在线",
               "disconnected": "○ 断开"}.get(i["status"], i["status"])
        sc = iface_static_config(i["name"]) or {}
        static_cfgs.append((i, sc))
        # ★ 别叫 mode：会把上面的 mode 参数（status/apply 语境）覆盖掉，
        #   导致 apply 分支永远走不到。曾因此打出"× 有 N 项阻塞"又"继续执行"的矛盾输出。
        dhcp_mode = ("动态 DHCP" if sc.get("dhcp_enabled")
                     else ("静态" if sc.get("ip") else "-"))
        print("     " + pad(tag, 8) + pad(i["name"], 22)
              + pad(sc.get("ip") or "-", 17) + pad(sc.get("mask") or "-", 17)
              + pad(sc.get("gateway") or "-", 17) + dhcp_mode)
    for i, sc in static_cfgs:
        if i["status"] == "disconnected" and sc.get("ip"):
            print()
            print(f"     ▲ 「{i['name']}」**媒体已断开**，但静态地址还留着 "
                  f"({sc['ip']}/{sc.get('prefix', '?')})。")
            print("        这通常就是『不通』的直接原因 —— 插上网线即可。")
    print()

    print("【3】本机 IPv4 网段（扫描与点云目的地址的依据）")
    nets = local_networks(iface)
    if not nets:
        print("     ▲ 没有任何已连接且配置了有效 IPv4 的网卡")
        blockers.append("本机无可用 IPv4 网卡")
    for n in nets:
        print("     " + pad(n["name"], 18) + pad(n["cidr"], 20)
              + pad("网关 " + (n["gateway"] or "无"), 22) + f'状态 {n["status"]}')
    print()

    print("【4】目标地址（取自 config/channels.json）")
    targets = channel_targets(channels_path)
    if not targets:
        print("     ▲ 配置里没有启用的通道")
    print("     " + pad("通道", 8) + pad("目标/源 IP", 17) + pad("TCP", 7) + "UDP")
    for t in targets:
        print("     " + pad(t["id"], 8) + pad(t["source_ip"] or t["ip"], 17)
              + pad(t["tcp_port"], 7) + str(t["udp_port"]))
    # 配置自检：4 条通道的 UDP 端口必须一致（靠同端口 + 源 IP 区分雷达）
    udps = {t["udp_port"] for t in targets}
    if len(udps) > 1:
        print(f"     × 各通道 udp_port 不一致 {sorted(udps)} —— "
              f"本项目靠『同端口 + 源 IP』分流，必须统一。")
        blockers.append("channels.json 各通道 udp_port 不一致")
    ips = [t["source_ip"] or t["ip"] for t in targets]
    if len(ips) != len(set(ips)):
        print("     × 各通道 source_ip 有重复 —— 源 IP 是唯一分流依据，不能重。")
        blockers.append("channels.json 各通道 source_ip 重复")
    print()

    print("【5】本机能否到达目标网段")
    for ip in _dedup(ips):
        if not ip:
            continue
        exit_ip = _route_source_for(ip)
        if exit_ip and same_slash24(exit_ip, ip):
            print(f"     √ {pad(ip, 17)} 出口 {exit_ip}（同网段，可达）")
        else:
            print(f"     × {pad(ip, 17)} 出口 {exit_ip or '无'}（**不在**同一网段，不可达）")
            blockers.append(f"本机不能到达 {ip}（需先把网卡配到该网段）")
    print()

    print("【6】出厂默认网段 192.168.1.0/24")
    has_factory_net = any(same_slash24(n["ip"], FACTORY_IP) for n in nets)
    offline = [info["name"] for info, sc in static_cfgs
               if sc.get("ip") and same_slash24(sc["ip"], FACTORY_IP)
               and info["status"] != "connected"]
    if has_factory_net:
        print(f"     √ 本机已在其中（{FACTORY_TEMP_IP} 之类的临时地址）")
    elif offline:
        print(f"     ▲ 「{'、'.join(offline)}」的静态地址在 192.168.1.0/24，"
              f"但网卡**媒体已断开**。")
        print("        插上网线后该网段即可达；现在扫不到出厂默认雷达。")
    else:
        print(f"     ▲ 本机不在该网段 —— 出厂默认雷达（{FACTORY_IP}）将扫不到。")
        print("        若雷达还是出厂默认，需要先给网卡加一个该网段地址：")
        print(f"           {PYEXE} tools/configure_radars.py nic add-temp "
              f"--iface {iface or '<网卡名>'} --ip {FACTORY_TEMP_IP}")
    print()

    print("【7】隐患检查")
    target_set = {t["source_ip"] or t["ip"] for t in targets}
    clash = False
    for info, sc in static_cfgs:
        nm = info["name"]
        # (a) 网卡自己的静态地址被雷达目标占用了 → 直接 IP 冲突
        if sc.get("ip") and sc["ip"] in target_set:
            clash = True
            print(f"     × 「{nm}」的静态 IP = {sc['ip']}，与雷达目标地址**撞车**。")
            print(f"        本机不能占用雷达的地址，请改成同网段的其它空闲地址（如 "
                  f"{sc['ip'].rsplit('.', 1)[0]}.100）。")
        # (b) 默认网关就是某台雷达 → 多一条指向雷达的『默认』路由，排查时误导
        gw = sc.get("gateway") or ""
        if gw and gw in target_set:
            clash = True
            print(f"     ▲ 「{nm}」的网关 = {gw}，而这个地址正是雷达的目标 IP。")
            print("        网段内通信是二层直发，不需要网关；留着它会多一条")
            print("        『默认』路由指向雷达，排查网络问题时容易误导。建议删掉：")
            print(f'          netsh interface ipv4 delete route 0.0.0.0/0 '
                  f'name="{nm}" nexthop={gw}')
    if not clash:
        print("     √ 未发现网关/本机地址与雷达地址冲突")

    cidrs = auto_scan_cidrs(iface, extra_cidrs)
    print(f"     · 本次将扫描：{', '.join(cidrs) if cidrs else '（无 —— 请用 --scan 指定网段）'}")
    if not cidrs:
        blockers.append("没有可扫描的网段（本机无 IPv4，且未给 --scan）")

    print()
    if blockers:
        if mode == "apply":
            print("▲ 有 %d 项需要留意（下面会说明哪些不拦截）：" % len(blockers))
        else:
            print("× 当前有 %d 项阻塞，先解决再执行下发：" % len(blockers))
        for b in blockers:
            print("   · " + b)
    else:
        print("√ 体检通过")
    print()
    return blockers


# ══════════════════════════════════════════════════════════ 子命令：status
def cmd_status(args) -> int:
    print("=" * 72)
    print("载重雷达 · 现场配置体检")
    print("=" * 72)
    print()
    preflight(args.channels, args.iface, args.scan, mode="status")

    print("【8】目标地址实际可达性（TCP %d）" % args.port)
    targets = _dedup([t["source_ip"] or t["ip"]
                      for t in channel_targets(args.channels)])
    for ip in targets:
        if not ip:
            continue
        ok = tcp_open(ip, args.port, max(0.6, args.timeout))
        print("     " + pad(ip, 17) + ("√ 在线" if ok else "× 不通"))
    print()
    print("下一步建议：")
    print("   · 有雷达不通        → 检查网线/供电，或它还是出厂默认 IP（见【6】）")
    print("   · 全部不通且网卡断开 → 先插网线（netsh 显示 ○ 断开）")
    print(f"   · 都通了            → {PYEXE} tools/configure_radars.py verify")
    return 0


# ══════════════════════════════════════════════════════════ 子命令：scan
def cmd_scan(args) -> int:
    cidrs = auto_scan_cidrs(args.iface, args.scan)
    if not cidrs:
        print("× 没有可扫描的网段，请用 --scan 198.51.100.0/24 指定")
        return 4
    banner = "扫描网段： " + "  ".join(cidrs)
    if args.json:
        # JSON 模式下 stdout 必须是**纯净的 JSON**（供 json.loads / jq 直接消费），
        # 一切人类可读的提示都走 stderr。
        print(banner, file=sys.stderr)
    else:
        print(banner)
    found, total = scan(cidrs, args.port, max(0.2, args.timeout), quiet=args.json)
    if args.json:
        radars = identify_all(found, args.port, max(1.0, args.timeout))
        print(json.dumps(
            [{"ip": r["ip"], "cfg": r["cfg"], "inv": r["inv"], "error": r["error"]}
             for r in radars], ensure_ascii=False, indent=2))
        return 0

    if not found:
        print()
        print("没发现任何响应（TCP %d）的雷达。" % args.port)
        print("常见原因：")
        print("  1) 网线没插 / 雷达没上电        → 看【网卡】是否 ○ 断开")
        print("  2) 雷达还是出厂默认 %s，" % FACTORY_IP)
        print("     而本机不在 192.168.1.0/24   → 用 nic add-temp 加临时地址")
        print("  3) 扫错网段                     → 用 --scan 指定")
        print("  4) 雷达已被改成别的 IP          → --scan 覆盖全部可能的网段")
        return 2

    print()
    radars = identify_all(found, args.port, max(1.0, args.timeout))
    print("发现 %d 台雷达：" % len(radars))
    print()
    print_radar_table(radars)
    print()

    if any(r["ip"] == FACTORY_IP for r in radars):
        print("▲ 注意：仍在出厂默认地址的雷达必须**一台一台**改。")
        print("   同一个网段出现两台同 IP 设备会互相抢包，谁也改不稳。")
        print()
    targets = desired_targets(args.channels)
    idle = [t for t in targets if t not in {r["ip"] for r in radars}]
    print("配置期望的 %d 个目标地址：" % len(targets))
    for t in targets:
        print("   " + pad(t, 17) + ("√ 已在此地址" if t in {r["ip"] for r in radars}
                                   else "○ 空着"))
    if idle:
        print()
        print("建议逐个补齐（每插一台跑一次）：")
        for t in idle[:4]:
            print(f"   {PYEXE} tools/configure_radars.py apply --ip {t} "
                  f"--dest-ip <本机地址> -y")
    return 0


# ══════════════════════════════════════════════════════════ 子命令：apply
def _parse_maps(items) -> list:
    out = []
    for s in items or []:
        if "=" not in s:
            raise ValueError(f"--map 格式应为 旧IP=新IP（收到 {s!r}）")
        a, b = s.split("=", 1)
        out.append((a.strip(), b.strip()))
    return out


def _suggest_dest_ip(targets, iface="") -> str:
    """在本机地址里找一个与目标同网段的地址，作为点云目的 IP 的建议值。"""
    for n in local_networks(iface):
        for t in targets:
            if same_slash24(n["ip"], t):
                return n["ip"]
    return ""


def cmd_apply(args) -> int:
    try:
        mapping = _parse_maps(args.map)
    except ValueError as e:
        print(f"× {e}")
        return 4

    print("=" * 72)
    print("载重雷达 · 下发配置" + ("（dry-run 演练，不碰雷达）" if args.dry_run else ""))
    print("=" * 72)
    print()
    blockers = preflight(args.channels, args.iface, args.scan, mode="apply")
    # 体检里的"目标网段不可达"在下发时确实有意义，但用户可能只是要先改一台
    # （雷达还在出厂网段），本机确实不在新网段 —— 所以只提示不拦截；
    # 只有"配置本身有问题"才硬中止。
    if blockers and not args.force:
        hard = [b for b in blockers
                if "udp_port" in b or "source_ip 重复" in b or "没有可扫描的网段" in b]
        if hard:
            print("× 配置本身有问题，已中止（这是配置错，不是网络问题）：")
            for h in hard:
                print("   · " + h)
            print()
            print("   改好 config/channels.json 再跑；确认无碍可加 --force 跳过检查。")
            return 4
        print("▲ 上列问题里，『目标网段不可达』这一类**不拦截**：")
        print("   雷达如果还是出厂默认（192.168.1.201），本机本来就不该在新网段。")
        print("   先按【6】给网卡加个临时地址，再回来跑同一条命令即可。")
        print()

    cidrs = auto_scan_cidrs(args.iface, args.scan)
    print("【扫描】")
    found, _ = scan(cidrs, args.port, max(0.2, args.timeout))
    if not found:
        print()
        print("× 没发现雷达，先解决连通性再执行（见上面【6】）。")
        return 2

    radars = identify_all(found, args.port, max(1.0, args.timeout))
    print()
    print("【发现的雷达】")
    print_radar_table(radars)
    print()

    targets = list(args.ip) if args.ip else desired_targets(args.channels)
    if not targets:
        print("× 既没有 --ip，也读不到 channels.json 的目标地址")
        return 4

    items, errors, notes = build_plan(radars, targets, mapping)
    for n in notes:
        print("提示：" + n)
    print()
    print("【变更计划】")
    print_plan(items)
    print()

    if errors:
        print("× 计划有冲突，**拒绝执行**（不会自动猜哪台改哪个）：")
        for e in errors:
            print("   · " + e)
        return 4

    todo = [it for it in items if not it.get("skip")]
    if not todo:
        print("√ 无需改动：目标地址上已经是对应的雷达。")
        return 0

    # ---- 目的 IP：点云发往哪里。不设 = 改完收不到点云（本工具主动拦一下）
    dest_ip = args.dest_ip or ""
    if not dest_ip and not args.dry_run:
        guess = _suggest_dest_ip(targets, args.iface)
        print("▲ 未指定 --dest-ip。雷达把点云发给『目的 IP』，")
        print("   只改自身 IP 而不改目的 IP，会出现『TCP 能连、命令都对、")
        print("   服务器一个点都收不到』的迷惑现象。")
        if guess:
            print(f"   若服务器就是本机，应写：--dest-ip {guess} "
                  f"--dest-port {args.dest_port or 2368}")
        print("   确实暂不改目的 IP 时，请显式加 --no-dest 表达意图。")
        if not args.no_dest:
            return 4
        print()

    # ---- 目标地址是否已被别的设备占用（IP 冲突会让两台都不稳定）
    if not args.force:
        busy = [it["target"] for it in todo if icmp_alive(it["target"])]
        if busy:
            print("× 这些目标地址已经有设备在应答，继续写会造成 IP 冲突：")
            for b in busy:
                print("   · " + b)
            print("   请先确认那是谁（可能是别处搬来的雷达/相机/PLC），")
            print("   确认可以覆盖后加 --force 再来。")
            return 4

    # ---- 确认
    print(f"将要执行 {len(todo)} 台：")
    for it in todo:
        print(f"   {it['host']}  →  {it['target']}")
    if dest_ip:
        print(f"   同时把点云目的地址改为 {dest_ip}:{args.dest_port or 2368}")
    if not args.dry_run and not args.yes:
        try:
            ans = input("\n确认执行？输入 yes 继续：").strip().lower()
        except EOFError:
            ans = ""
        if ans != "yes":
            print("已取消，未做任何修改。")
            return 1
    print()

    results, failures = apply_plan(
        items, port=args.port, dest_ip=dest_ip or None, dest_port=args.dest_port,
        archive_dir=args.archive_dir, wait_s=args.wait, dry_run=args.dry_run)

    if args.dry_run:
        print()
        print("dry-run 结束：以上为将要执行的命令，未发送任何写命令。")
        return 0

    # ---- 复验
    print()
    print("═" * 72)
    print("【复验】")
    print("═" * 72)
    radars2 = identify_all([it["target"] for it in todo], args.port,
                           max(1.0, args.timeout))
    print_radar_table(radars2)
    dead = [r["ip"] for r in radars2 if r["error"]]

    # ---- 回写 channels.json
    if not args.no_sync:
        changes, sync_err = sync_channels(items, args.channels,
                                         frame_ms=args.sync_frame_ms)
        print()
        if sync_err:
            print("▲ 回写 channels.json 失败：" + sync_err)
        elif changes:
            print("已回写 config/channels.json：")
            for c in changes:
                print("   · " + c)
        else:
            print("config/channels.json 无需修改（IP 已经对上）。")

    print()
    print("─" * 72)
    if failures or dead:
        print(f"结果：{len(todo) - failures}/{len(todo)} 台成功；"
              f"复验未连上：{', '.join(dead) if dead else '无'}")
        print("排查：")
        print(f"   · {PYEXE} tools/hr40_ip.py -H <新IP> read   # 单独确认")
        print("   · 改前存档在 re/hr40ip/provision/           # 含序列号与全部原值")
        return 6
    print(f"结果：全部 {len(todo)} 台配置成功 √")
    print("下一步：")
    print("   1) 插好全部 4 台雷达（各自已是目标 IP，不会再冲突）")
    print(f"   2) {PYEXE} cli/capture.py --split-by-ip     # 确认 4 路都有包、读出转速")
    print(f"   3) {PYEXE} cli/run.py   或双击《启动桌面版.vbs》")
    return 0


# ══════════════════════════════════════════════════════════ 子命令：verify
def cmd_verify(args) -> int:
    targets = list(args.ip) if args.ip else desired_targets(args.channels)
    if not targets:
        print("× 没有可复验的地址")
        return 4
    print("=" * 72)
    print(f"复验 {len(targets)} 个目标地址（TCP {args.port}）")
    print("=" * 72)
    print()
    radars = identify_all(targets, args.port, max(1.0, args.timeout))
    print_radar_table(radars)
    print()

    want_dest = args.dest_ip or _suggest_dest_ip(targets, args.iface)
    bad = 0
    print("逐项判定：")
    print("  " + pad("地址", 17) + pad("可连", 7) + pad("自身IP", 11)
          + pad("点云目的", 12) + "说明")
    print("  " + "-" * 88)
    for r in radars:
        if r["error"]:
            bad += 1
            print("  " + pad(r["ip"], 17) + pad("×", 7) + pad("-", 11)
                  + pad("-", 12) + r["error"][:40])
            continue
        c = r["cfg"]
        ip_ok = c.get("ip") == r["ip"]
        dest = c.get("dest_ip", "")
        # ★ 用 _dest_cannot_receive 而非 _in_subnet：出厂默认目的 IP 是广播
        #   255.255.255.255，它不参与子网判断（广播只在发送方链路内传播）。
        #   直接用 _in_subnet 会把广播误判成"跨网段，点云发不出去"。
        dest_ok = not _dest_cannot_receive(dest, r["ip"], c.get("subnet_mask", ""))
        note = ""
        if want_dest and dest != want_dest:
            note = f"点云目的 {dest}（期望 {want_dest}）"
        if not ip_ok:
            bad += 1
        if not dest_ok:
            bad += 1
            note = (note + "；" if note else "") + "目的IP跨网段，点云发不出去"
        print("  " + pad(r["ip"], 17) + pad("√", 7) + pad("√" if ip_ok else "×", 11)
              + pad("√" if dest_ok else "×", 12) + note)

    # channels.json 与实测是否一致
    cfg_targets = {(t["source_ip"] or t["ip"]) for t in channel_targets(args.channels)}
    alive = {r["ip"] for r in radars if not r["error"]}
    missing = cfg_targets - alive
    extra = alive - cfg_targets
    print()
    if missing:
        print("▲ channels.json 里期望但没连上的："
              + ", ".join(sorted(missing, key=_ip_key)))
        bad += 1
    if extra:
        print("▲ 实际在线但 channels.json 没登记的："
              + ", ".join(sorted(extra, key=_ip_key)))
        bad += 1
    if not missing and not extra:
        print("√ channels.json 的 source_ip 与实际在线雷达完全一致")

    print()
    if bad:
        print(f"× 复验发现 {bad} 处问题")
        return 5
    print("√ 复验全部通过")
    return 0


# ══════════════════════════════════════════════════════════ 子命令：sync
def cmd_sync(args) -> int:
    try:
        mapping = _parse_maps(args.map)
    except ValueError as e:
        print(f"× {e}")
        return 4

    if mapping:
        items = [{"host": a, "target": b, "radar": {"ip": a, "cfg": {}, "inv": {}}}
                 for a, b in mapping]
        print("按 --map 直接回写（不扫描、不碰雷达）")
    else:
        # 扫描拿到"旧 IP → 新 IP"的对应关系，才能准确回写
        cidrs = auto_scan_cidrs(args.iface, args.scan)
        if not cidrs:
            print("× 没有可扫描的网段；也可以直接用 sync --map 旧IP=新IP")
            return 4
        print("扫描网段： " + "  ".join(cidrs))
        found, _ = scan(cidrs, args.port, max(0.2, args.timeout))
        if not found:
            print("× 没发现雷达；若配置里已经写的是新 IP，可加 --map 直接指定")
            return 2
        radars = identify_all(found, args.port, max(1.0, args.timeout))
        print()
        print_radar_table(radars)
        print()
        targets = list(args.ip) if args.ip else desired_targets(args.channels)
        items, errors, _ = build_plan(radars, targets, None)
        if errors:
            for e in errors:
                print("× " + e)
            print("提示：用 --map 旧IP=新IP 显式指定对应关系")
            return 4

    changes, err = sync_channels(items, args.channels,
                                 frame_ms=args.sync_frame_ms,
                                 dry_run=args.dry_run)
    if err:
        print("× " + err)
        return 6
    if not changes:
        print("无需修改（配置里的 IP 已经与实际情况一致）。")
        return 0
    print(("将要修改" if args.dry_run else "已修改") + " config/channels.json：")
    for c in changes:
        print("   · " + c)
    if not args.dry_run:
        print()
        print("已生成备份 config/channels.json.bak-*（改坏了可以直接换回来）")
    return 0


# ══════════════════════════════════════════════════════════ 子命令：nic
def cmd_nic(args) -> int:
    if args.nic_action == "list":
        print("网卡：")
        for i in list_interfaces():
            tag = {"connected": "● 在线",
                   "disconnected": "○ 断开"}.get(i["status"], i["status"])
            print("  " + pad(tag, 8) + pad(f'[{i["index"]}]', 6) + pad(i["name"], 18)
                  + f'跃点={i["metric"]}')
        print()
        print("已配置的 IPv4：")
        for n in local_networks():
            print("  " + pad(n["name"], 18) + pad(n["cidr"], 20)
                  + pad("网关 " + (n["gateway"] or "无"), 22) + f'状态 {n["status"]}')
        print()
        ok = any(same_slash24(n["ip"], FACTORY_IP) for n in local_networks())
        print("出厂默认网段是否可达：" + ("是" if ok else "否"))
        return 0

    iface = args.iface or ""
    if not iface:
        print("× 必须用 --iface 指定网卡名（用 nic list 查看）")
        return 4
    if not args.ip:
        print("× 必须用 --ip 指定地址")
        return 4
    try:
        _ip4(args.ip)
    except ValueError:
        print(f"× IP 非法：{args.ip}")
        return 4
    mask = args.mask or "255.255.255.0"

    verb = "add" if args.nic_action == "add-temp" else "delete"
    manual = (f'netsh interface ipv4 {verb} address name="{iface}" '
              f'address={args.ip}' + (f" mask={mask}" if verb == "add" else ""))

    if not is_admin():
        print("× 修改网卡需要**管理员权限**。")
        print("   请以管理员身份运行《配置雷达IP.bat》，或管理员命令行里手工执行：")
        print("   " + manual)
        return 3

    if args.nic_action == "add-temp":
        if args.ip in {n["ip"] for n in local_networks()}:
            print(f"√ {iface} 上已经有 {args.ip}，无需添加。")
            return 0
        # ★ 故意不带 gateway：加了会多一条默认路由，可能和 WLAN 抢出口
        cmd = ["netsh", "interface", "ipv4", "add", "address",
               f"name={iface}", f"address={args.ip}", f"mask={mask}"]
        rc, out = run_cmd(cmd)
        print("  " + " ".join(cmd))
        print(out.strip())
        if rc != 0:
            print("× 添加失败")
            return 6
        if args.ip in {n["ip"] for n in local_networks()}:
            print(f"√ 已添加 {args.ip}/{mask} 到「{iface}」")
            print(f"   用完删除：{PYEXE} tools/configure_radars.py nic del-temp "
                  f"--iface {iface} --ip {args.ip}")
            return 0
        print("▲ netsh 返回成功，但地址没出现在网卡上，请手工核对。")
        return 6

    rc, out = run_cmd(["netsh", "interface", "ipv4", "delete", "address",
                       f"name={iface}", f"address={args.ip}"])
    print("  " + manual)
    print(out.strip())
    if rc != 0:
        print("× 删除失败（地址本来就不存在？）")
        return 6
    print(f"√ 已从「{iface}」删除 {args.ip}")
    return 0


# ══════════════════════════════════════════════════════════ CLI
EPILOG = (
    "典型流程：\n"
    f"  {PYEXE} tools/configure_radars.py status\n"
    f"  {PYEXE} tools/configure_radars.py nic add-temp "
    "--iface 以太网 --ip 192.168.1.250\n"
    f"  {PYEXE} tools/configure_radars.py scan\n"
    f"  {PYEXE} tools/configure_radars.py apply --ip 198.51.100.1 "
    "--dest-ip 198.51.100.15 -y\n"
    f"  {PYEXE} tools/configure_radars.py verify\n")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="configure_radars.py",
        description="4 台雷达「一把配好」：体检 → 扫描 → 下发 IP → 复验 → 回写配置",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=EPILOG)
    ap.add_argument("--channels", default=DEFAULT_CHANNELS,
                    help="通道配置路径（默认 config/channels.json）")
    ap.add_argument("-p", "--port", type=int, default=DEFAULT_TCP_PORT,
                    help=f"雷达 TCP 控制端口，默认 {DEFAULT_TCP_PORT}")
    ap.add_argument("--timeout", type=float, default=4.0,
                    help="单次连接超时秒数（默认 4）")
    ap.add_argument("--scan", action="append", default=[],
                    help="要扫描的网段，可重复（默认取本机各网卡的 /24）")
    ap.add_argument("--iface", default="", help="限定网卡名（如 以太网）")

    sub = ap.add_subparsers(dest="action", required=True)

    sub.add_parser("status", help="现场体检：权限/网卡/网段/目标可达性")

    p_sc = sub.add_parser("scan", help="扫描发现雷达并读回配置")
    p_sc.add_argument("--json", action="store_true", help="输出 JSON（供程序消费）")

    p_ap = sub.add_parser("apply", help="★扫描 + 生成计划 + 逐台下发 IP")
    p_ap.add_argument("--ip", action="append", default=[],
                      help="目标 IP，可重复（默认取 channels.json 的 source_ip）")
    p_ap.add_argument("--map", action="append", default=[],
                      help="显式映射 旧IP=新IP，可重复（给了就不自动分配）")
    p_ap.add_argument("--dest-ip", dest="dest_ip", default="",
                      help="点云目的 IP（服务器地址）——★强烈建议显式指定")
    p_ap.add_argument("--dest-port", dest="dest_port", type=int, default=2368,
                      help="点云目的端口（默认 2368）")
    p_ap.add_argument("--no-dest", action="store_true",
                      help="明确表示本次不动目的 IP（否则缺 --dest-ip 会中止）")
    p_ap.add_argument("--archive-dir", default="",
                      help="改前存档目录（默认 re/hr40ip/provision）")
    p_ap.add_argument("--wait", type=float, default=90.0,
                      help="重启后等待新 IP 上线的秒数（默认 90）")
    p_ap.add_argument("--force", action="store_true",
                      help="忽略体检硬提示与目标地址占用检查（慎用）")
    p_ap.add_argument("--dry-run", action="store_true",
                      help="演练：只打印将要执行的命令")
    p_ap.add_argument("--no-sync", action="store_true",
                      help="不回写 config/channels.json")
    p_ap.add_argument("--sync-frame-ms", action="store_true", dest="sync_frame_ms",
                      help="顺带按实测转速推算并写入 frame_ms（默认不动；"
                           "frame_mode=rotation 时 frame_ms 不生效，一般不需要）")
    p_ap.add_argument("-y", "--yes", action="store_true",
                      help="跳过交互确认（脚本化用）")

    p_v = sub.add_parser("verify", help="复验所有目标地址是否配置正确")
    p_v.add_argument("--ip", action="append", default=[],
                     help="要复验的地址，可重复（默认取 channels.json）")
    p_v.add_argument("--dest-ip", dest="dest_ip", default="",
                     help="期望的点云目的 IP（用于比对）")

    p_s = sub.add_parser("sync", help="把 IP 变更回写到 config/channels.json")
    p_s.add_argument("--ip", action="append", default=[])
    p_s.add_argument("--map", action="append", default=[],
                     help="显式映射 旧IP=新IP（不扫描时用这个）")
    p_s.add_argument("--sync-frame-ms", action="store_true", dest="sync_frame_ms")
    p_s.add_argument("--dry-run", action="store_true", help="只看会改什么")

    p_n = sub.add_parser("nic", help="网卡临时地址管理（需管理员）")
    p_n.add_argument("nic_action", choices=["list", "add-temp", "del-temp"])
    p_n.add_argument("--iface", default="", help="网卡名（如 以太网）")
    p_n.add_argument("--ip", default="", help="临时地址（建议 192.168.1.250）")
    p_n.add_argument("--mask", default="", help="掩码（默认 255.255.255.0）")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.action == "status":
            return cmd_status(args)
        if args.action == "scan":
            return cmd_scan(args)
        if args.action == "apply":
            return cmd_apply(args)
        if args.action == "verify":
            return cmd_verify(args)
        if args.action == "sync":
            return cmd_sync(args)
        if args.action == "nic":
            return cmd_nic(args)
    except KeyboardInterrupt:
        print("\n已中断。")
        return 130
    print(f"未知子命令：{args.action}")
    return 4


if __name__ == "__main__":
    sys.exit(main())
