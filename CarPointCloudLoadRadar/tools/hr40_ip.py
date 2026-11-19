#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HR40 雷达网络配置工具 —— HR40改IP.exe 的净室复刻（纯 Python + 标准库）

原厂工具是 .NET x86 WinForms，只能在 Windows 上跑；本脚本用标准库 socket 实现同一协议，
可直接在麒麟 / 飞腾 / 鲲鹏 / 龙芯上使用。

协议（TCP 9347，全大端）：
    请求/应答帧：
        [0]      0x47                        Header1
        [1]      0x74                        Header2
        [2]      cmd                        命令码
        [3]      0x00(请求) / returnCode(应答)
        [4..7]   payloadLength  uint32 大端
        [8..]    payload

命令码：
    0x05 读角度校正表(ASCII CSV)   0x07 读设备清单   0x08 读配置
    0x10 重启                      0x20 写目的网络   0x21 写自身网络
    0x58 遮挡检测开关

注意：改「自身网络」后雷达会重启，届时需用新 IP 重连。
"""
import argparse
import os
import socket
import struct
import sys

HDR1 = 0x47
HDR2 = 0x74

CMD_GET_ANGLE = 0x05
CMD_GET_INVENTORY = 0x07
CMD_GET_CONFIG = 0x08
CMD_RESTART = 0x10
CMD_SET_DEST = 0x20
CMD_SET_SELF = 0x21
CMD_SET_BLOCKAGE = 0x58

DEFAULT_TCP_PORT = 9347
CONNECT_TIMEOUT = 10.0
RECV_TIMEOUT = 10.0
SEND_TIMEOUT = 5.0

# 提示里写的解释器名按平台取：Windows 只有 python，麒麟/Linux 才是 python3。
# （现场照着提示抄命令是很常见的事，写错会让排查多绕一圈。）
PYEXE = "python" if os.name == "nt" else "python3"

# 自身网络载荷（命令 0x21）尾部 3 字节：0x00 0x00 0x01
# ★ 逐条核对 IL 得到（re/hr40ip/il_full.txt::BuildSetSelfNetworkPayload）：
#     IL_1BA2 ldc.i4.0 → stelem.i1    payload[12] = 0
#     IL_1BAA ldc.i4.0 → stelem.i1    payload[13] = 0
#     IL_1BB2 ldc.i4.1 → stelem.i1    payload[14] = 1   ← 是 1，不是 0
#   第 3 字节语义二进制里无标注，最可能是"静态 IP 使能/立即生效"标志。
#   若误发 0x00，设备很可能仍回 ReturnCode=0（应答只校验帧头/命令码/故障码），
#   但不真正应用新 IP —— 现场表现为"工具报成功、雷达还在老 IP"，且无任何报错。
#   故严格照抄原厂取值。
SELF_NET_TAIL = b"\x00\x00\x01"
# 目的网络载荷（命令 0x20）尾部 2 字节：WriteUInt16BigEndian(0) → 0x00 0x00
DEST_NET_TAIL = b"\x00\x00"


# ---------------------------------------------------------------- 帧编解码
def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    """组帧：8 字节头 + payload"""
    return struct.pack(">BBBB", HDR1, HDR2, cmd & 0xFF, 0) + struct.pack(">I", len(payload)) + payload


class ProtocolError(Exception):
    pass


class HR40Client:
    def __init__(self, host: str, port: int = DEFAULT_TCP_PORT):
        self.host = host
        self.port = port
        self.sock = None

    # ---- 连接
    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=CONNECT_TIMEOUT)
        self.sock.settimeout(RECV_TIMEOUT)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return self

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *a):
        self.close()

    # ---- 收发
    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ProtocolError("连接意外关闭")
            buf += chunk
        return bytes(buf)

    def send_cmd(self, cmd: int, payload: bytes = b"", expect=None):
        """发一条命令并读回应；返回 (return_code, payload)"""
        if expect is None:
            expect = cmd
        self.sock.sendall(build_frame(cmd, payload))

        head = self._recv_exact(8)
        h1, h2, r_cmd, r_code = head[0], head[1], head[2], head[3]
        (plen,) = struct.unpack(">I", head[4:8])

        if h1 != HDR1 or h2 != HDR2:
            raise ProtocolError(f"协议头校验失败 (收到 {h1:#04x} {h2:#04x})")
        if r_cmd != expect:
            raise ProtocolError(f"命令码不匹配 (期望: 0x{expect:02X}, 收到: 0x{r_cmd:02X})")
        if r_code != 0:
            raise ProtocolError(f"设备返回故障码: 0x{r_code:02X} (参考文档 1.4.2)")
        body = self._recv_exact(plen) if plen else b""
        return r_code, body

    # ---- 读操作
    def get_angle_csv(self) -> str:
        _, body = self.send_cmd(CMD_GET_ANGLE)
        return body.decode("ascii", errors="replace")

    def get_inventory(self) -> dict:
        _, p = self.send_cmd(CMD_GET_INVENTORY)

        def asc(off, n):
            # 原版用 string.Trim() 仅去空白；实测设备用 NUL 填充，故两者都去
            return p[off:off + n].decode("ascii", errors="replace").strip("\x00 \t\r\n")

        return {
            "serial_number": asc(0, 18),
            "part_number": asc(18, 32),
            "date_of_manufacture": asc(50, 16),
            "mac_address": ":".join(f"{b:02X}" for b in p[66:72]),
            "software_version": asc(72, 16),
            "controller_fw_version": asc(104, 16),
        }

    def get_config(self) -> dict:
        _, p = self.send_cmd(CMD_GET_CONFIG)
        o = 0

        def ip():
            nonlocal o
            v = ".".join(str(b) for b in p[o:o + 4]); o += 4
            return v

        def u16():
            nonlocal o
            v = (p[o] << 8) | p[o + 1]; o += 2
            return v

        def u8():
            nonlocal o
            v = p[o]; o += 1
            return v

        cfg = {
            "ip": ip(), "subnet_mask": ip(), "gateway": ip(), "dest_ip": ip(),
            "dest_port": u16(),
        }
        o += 2                                   # 保留
        cfg["spin_rate"] = u16()
        cfg["sync_enable"] = bool(u8())
        cfg["sync_angle"] = u16()
        o += 4                                   # 保留
        cfg["clock_source"] = u8()
        o += 1                                   # 保留
        cfg["trigger_method"] = u8()
        cfg["return_mode"] = u8()
        cfg["standby_mode"] = u8()
        o += 1                                   # 保留
        cfg["vlan_flag"] = bool(u8())
        cfg["vlan_id"] = u16()
        cfg["_payload_len"] = len(p)
        return cfg

    # ---- 写操作
    def set_self_network(self, ip: str, mask: str, gateway: str):
        """改雷达自身 IP / 掩码 / 网关。

        ▲ 原厂 SetLidarSelfNetworkConfig **不会**顺手发重启命令（IL 里没有
        对 RestartLidar 的调用），它只打印「后续操作需要更新 CurrentLidarIp
        为新IP后重新连接」。所以生效方式要么是设备收到 0x21 自行重启，
        要么必须由调用方再发一次 0x10。现场以 --restart 显式重启为准。

        尾部 3 字节见模块级 SELF_NET_TAIL 的说明（是 00 00 01，不是 00 00 00）。
        """
        payload = bytes(_ip4(ip) + _ip4(mask) + _ip4(gateway)) + SELF_NET_TAIL
        assert len(payload) == 15, len(payload)
        return self.send_cmd(CMD_SET_SELF, payload)

    def set_destination(self, dest_ip: str, dest_port: int):
        """改点云目的 IP / 端口 —— 通常立即生效，无需重启"""
        payload = bytes(_ip4(dest_ip)) + struct.pack(">H", dest_port) + DEST_NET_TAIL
        assert len(payload) == 8
        return self.send_cmd(CMD_SET_DEST, payload)

    def set_blockage_detection(self, enable: bool):
        return self.send_cmd(CMD_SET_BLOCKAGE, bytes([1 if enable else 0]))

    def restart(self):
        return self.send_cmd(CMD_RESTART)


def _ip4(s: str) -> list:
    parts = s.split(".")
    if len(parts) != 4:
        raise ValueError("IP 地址格式错误")
    out = []
    for x in parts:
        v = int(x)
        if not 0 <= v <= 255:
            raise ValueError("IP 地址格式错误")
        out.append(v)
    return out


def _in_subnet(ip: str, net_ip: str, mask: str) -> bool:
    """ip 是否与 net_ip 在同一网段（按 mask）。任一参数非法则返回 False。"""
    try:
        a = int.from_bytes(bytes(_ip4(ip)), "big")
        b = int.from_bytes(bytes(_ip4(net_ip)), "big")
        m = int.from_bytes(bytes(_ip4(mask)), "big")
    except (ValueError, AttributeError):
        return False
    return (a & m) == (b & m)


def _is_any_broadcast(ip: str, net_ip: str = "", mask: str = "") -> bool:
    """是否为广播地址（受限广播 255.255.255.255，或 net_ip/mask 网段的定向广播）。

    ★ 广播地址不能参与「目的地址是否在新网段内」的判断 ——
      受限广播 255.255.255.255 是在**发送方所在链路**上广播，同网段设备都能收到；
      雷达改到哪个网段，广播就发到那个网段，不存在"跨网段发不出去"的问题。
      实测（2026-09-13 现场）：雷达 198.51.100.4 保持目的 IP=255.255.255.255，
      点云 8971 包/s 正常到达本机 198.51.100.15。
    """
    if ip == "255.255.255.255":
        return True
    try:
        a = int.from_bytes(bytes(_ip4(ip)), "big")
        b = int.from_bytes(bytes(_ip4(net_ip)), "big")
        m = int.from_bytes(bytes(_ip4(mask)), "big")
    except (ValueError, AttributeError):
        return False
    # 定向广播 = 网络号部分保留 + 主机位全 1，即 b | (~m)。
    # ★ 别写成 b | m —— 那是把**网络位**全置 1，会算出 255.255.255.4 这种废地址。
    return bool(m) and a == (b | (0xFFFFFFFF ^ m))


def _local_addr_in_subnet(net_ip: str, mask: str) -> str:
    """本机是否已有地址落在 net_ip/mask 网段内；有则返回该地址，否则返回空串。

    比 _route_source_for 更可靠：多网卡机器刚加完副 IP 时，内核路由表与源地址
    选择可能还没稳定（实测会短暂误报出口为另一张网卡）。「本机地址里已有同网段
    地址」才是改完能连上新 IP 的充分条件。
    """
    try:
        addrs = socket.gethostbyname_ex(socket.gethostname())[2]
    except OSError:
        addrs = []
    for a in addrs:
        if _in_subnet(a, net_ip, mask):
            return a
    return ""


def _dest_cannot_receive(dest_ip: str, new_ip: str, mask: str) -> bool:
    """改完自身 IP 后，这个目的地址是否**收不到**点云。

    广播地址 → False（同链路必达，与网段无关）；其余必须与新 IP 同网段。
    """
    if _is_any_broadcast(dest_ip, new_ip, mask):
        return False
    return not _in_subnet(dest_ip, new_ip, mask)


def _route_source_for(ip: str) -> str:
    """问内核：去 ip 会使用本机哪个地址出口（UDP connect 不发任何包）。

    用来在改 IP 前预判「本机网卡是否已在目标网段」——若出口地址不在目标网段，
    改完必然连不上新 IP，可以当场提示而不是等 90 秒超时。
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((ip, 9))
        return s.getsockname()[0]
    except OSError:
        return ""
    finally:
        s.close()


def _wait_reachable(host: str, port: int, timeout: float = 90.0, interval: float = 2.0) -> bool:
    """轮询 TCP 端口直到连通（雷达重启后用）。返回是否在超时内连上。"""
    import time
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = socket.socket()
        s.settimeout(2.0)
        try:
            s.connect((host, port))
            return True
        except OSError:
            time.sleep(interval)
        finally:
            try:
                s.close()
            except OSError:
                pass
    return False


def provision(host: str, new_ip: str, mask: str = None, gateway: str = None,
              dest_ip: str = None, dest_port: int = None,
              port: int = DEFAULT_TCP_PORT, archive_dir: str = "",
              wait_s: float = 90.0, assume_yes: bool = False) -> int:
    """一条命令完成：读取存档 → 改自身 IP →(可选)改目的 IP → 重启 → 等待 → 用新 IP 验证。

    这是**唯一推荐的改 IP 方式** —— 因为「改 IP」与「重启」是两条独立命令，
    分步手打很容易漏掉重启（设备仍监听旧 IP，会被误判为"没改成功"）。

    ★ mask / gateway 不传时**沿用雷达当前值**，不静默填默认值。
    ★ 目的 IP 决定点云发往哪台服务器。若它不在新网段内，改完 IP 后雷达虽然能连上，
      但点云是跨网段发的 → 服务器一个点都收不到（现象极具迷惑性）。本函数会主动告警。
    """
    import time

    # ---- 0. 前置校验（本地，不联网）
    _ip4(new_ip)                                  # 格式非法就地抛错
    for _name, _val in (("--mask", mask), ("--gateway", gateway), ("--dest-ip", dest_ip)):
        if _val is not None:
            try:
                _ip4(_val)
            except ValueError:
                raise ValueError(f"{_name} 非法：{_val}") from None
    if dest_port is not None and not (0 < dest_port < 65536):
        raise ValueError(f"--dest-port 越界：{dest_port}")
    if new_ip == host:
        print(f"[跳过] 新 IP 与当前 IP 相同（{new_ip}），无需修改。", file=sys.stderr)
        return 1

    # 前置检查：改完能不能用新 IP 连上。
    # 先看本机是否已有该网段地址（充分条件，不受路由表抖动影响），
    # 再退回问内核出口地址。mask 此时可能未传，按 /24 近似（与现场默认一致）。
    _pre_mask = mask or "255.255.255.0"
    local_same = _local_addr_in_subnet(new_ip, _pre_mask)
    src = local_same or _route_source_for(new_ip)
    if src and not _in_subnet(src, new_ip, _pre_mask):
        print(f"[前置检查] ▲ 本机去 {new_ip} 的出口地址是 {src}，不在同一网段。")
        print(f"           改完 IP 后将无法连上新地址，请先把网卡改到 {new_ip} 所在网段再执行。")
    elif local_same:
        print(f"[前置检查] √ 本机已有同网段地址 {local_same}，改完可直连 {new_ip}。")

    # ---- 1. 读取当前配置并存档（万一改坏，至少知道原值）
    print(f"[1/5] 读取当前配置 {host}:{port} …")
    with HR40Client(host, port) as c:
        cfg = c.get_config()
        try:
            inv = c.get_inventory()
        except Exception:
            inv = {}
    print(f"      当前 自身IP={cfg['ip']}  掩码={cfg['subnet_mask']}  网关={cfg['gateway']}")
    print(f"      当前 目的IP={cfg['dest_ip']}  目的端口={cfg['dest_port']}  转速={cfg['spin_rate']}")

    # ---- 掩码 / 网关：未显式指定就沿用雷达当前值（禁止静默默认值）
    if mask is None:
        mask = cfg["subnet_mask"]
        print(f"      掩码未指定 → 沿用当前值 {mask}")
    if gateway is None:
        gateway = cfg["gateway"]
        print(f"      网关未指定 → 沿用当前值 {gateway}")

    # ---- 目的 IP：点云发往哪里。不在新网段内 = 改完收不到数据
    will_set_dest = dest_ip is not None
    eff_dest_ip = dest_ip if will_set_dest else cfg.get("dest_ip", "")
    eff_dest_port = dest_port if dest_port is not None else cfg.get("dest_port")
    if not will_set_dest and _dest_cannot_receive(eff_dest_ip, new_ip, mask):
        print()
        print(f"  ▲ 雷达目的 IP = {eff_dest_ip}，不在新网段 {new_ip}/{mask} 内。")
        print( "     改完自身 IP 后，雷达会把点云发给这个地址；跨网段且网关不可达 →")
        print( "     服务器将**收不到任何点云**（但 TCP 控制口仍可连，极易误判为'改成功了'）。")
        print(f"     本机出口地址是 {src or '未知'}。若服务器就是本机，请这样指定：")
        print(f"         --dest-ip {src} --dest-port {eff_dest_port or 2368}")
        print()

    if not archive_dir:
        archive_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "..", "re", "hr40ip", "provision")
    os.makedirs(archive_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    arch = os.path.join(archive_dir, f"before_{host.replace('.', '_')}_{stamp}.txt")
    with open(arch, "w", encoding="utf-8") as f:
        f.write(f"# 雷达改 IP 前存档  {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"# 连接地址 {host}:{port}\n")
        for k, v in cfg.items():
            if not k.startswith("_"):
                f.write(f"{k} = {v}\n")
        for k, v in inv.items():
            f.write(f"inventory.{k} = {v}\n")
    print(f"      已存档 → {arch}")

    # ---- 2. 确认
    if not assume_yes:
        print()
        print(f"  即将把雷达 {host}  改为  {new_ip} / {mask} / 网关 {gateway}")
        if will_set_dest:
            print(f"  同时把点云目的地址改为 {dest_ip}:{dest_port if dest_port is not None else cfg['dest_port']}")
        print(f"  改完会立即重启，届时必须用新 IP {new_ip} 才能连上。")
        try:
            ans = input("  确认执行？输入 yes 继续：").strip().lower()
        except EOFError:
            ans = ""
        if ans != "yes":
            print("  已取消，未做任何修改。")
            return 1

    # ---- 3. 写自身网络（命令 0x21）
    print(f"[2/5] 写入自身网络配置 …")
    with HR40Client(host, port) as c:
        c.set_self_network(new_ip, mask, gateway)
    print("      写入成功（此时尚未生效，设备仍监听旧 IP）")

    # ★ 目的网络（0x20）**不能**在这里发 —— 见 [5/5] 之后的说明：
    #   紧跟 0x21 之后发 0x20，会把刚写进去的新自身 IP 冲回旧值。

    # ---- 4. 重启（命令 0x10 —— 必须单独发！）
    print(f"[3/5] 发送重启命令（发往旧 IP {host}）…")
    try:
        with HR40Client(host, port) as c:
            c.restart()
    except (OSError, ProtocolError) as e:
        # 设备可能已开始重启导致连接中断，属正常
        print(f"      （连接中断，通常说明设备已开始重启：{e}）")

    # ---- 5. 等待新 IP 起来
    if wait_s > 0:
        print(f"[4/5] 等待 {new_ip}:{port} 上线（最多 {int(wait_s)} 秒）…")
        ok = _wait_reachable(new_ip, port, timeout=wait_s)
        if not ok:
            print(f"      ▲ {int(wait_s)} 秒内未连上 {new_ip}:{port}")
            print("      请检查：网卡是否已改到新网段 / 网线 / 雷达是否还在启动 / IP 是否冲突")
            return 2
        print("      已上线")
    else:
        print("[4/5] 已跳过等待（--wait 0）")

    # ---- 6. 验证
    print(f"[5/5] 用新 IP 验证 …")
    try:
        with HR40Client(new_ip, port) as c:
            ncfg = c.get_config()
    except (OSError, ProtocolError) as e:
        print(f"      ▲ 无法连接到新 IP {new_ip}:{port}（{type(e).__name__}）")
        print("      命令已全部发送。若设备确实重启中，请稍后用以下命令手工确认：")
        print(f"          {PYEXE} tools/hr40_ip.py -H {new_ip} read")
        print(f"      原配置存档：{arch}")
        return 2
    # ★ 先只看自身网络。目的网络要等自身 IP 生效后**单独**发一条 0x20（见下）。
    self_ok = (ncfg["ip"] == new_ip and ncfg["subnet_mask"] == mask)
    print(f"      读回：自身IP={ncfg['ip']}  掩码={ncfg['subnet_mask']}  网关={ncfg['gateway']}")
    print(f"      目的IP={ncfg['dest_ip']}  目的端口={ncfg['dest_port']}"
          f"{'' if will_set_dest else '（未被改动）'}")
    if not self_ok:
        print(f"\n× 验证不一致：期望 {new_ip}/{mask}，读回 {ncfg['ip']}/{ncfg['subnet_mask']}")
        return 3

    # ---- 6b. 改目的网络（命令 0x20）—— 只在自身 IP 生效之后才发
    # ★ 实测（2026-09-13，第二台雷达）：同一次连接里先发 0x21 再发 0x20，
    #   结果是 0x20 生效、**0x21 被冲掉** —— 设备按"当前生效的配置"整体回存，
    #   把刚写入的新自身 IP 又覆盖回旧值。现场现象是"重启完 IP 根本没变、
    #   目的 IP 却改了"，极具误导性（会让人以为设备不支持改 IP 而放弃）。
    #   故拆成第二步：用**新 IP** 连上去改，改完立即回读校验。
    if will_set_dest:
        _dport = dest_port if dest_port is not None else ncfg["dest_port"]
        print(f"      改点云目的地址 → {dest_ip}:{_dport} …")
        dest_ok = False
        try:
            with HR40Client(new_ip, port) as c:
                c.set_destination(dest_ip, _dport)
                ncfg2 = c.get_config()
            dest_ok = (ncfg2["dest_ip"] == dest_ip
                       and ncfg2["dest_port"] == _dport
                       and ncfg2["ip"] == new_ip)
        except (OSError, ProtocolError) as e:
            print(f"      ▲ 改目的地址时连接失败（{type(e).__name__}）")
        if dest_ok:
            print(f"      目的地址读回：{ncfg2['dest_ip']}:{ncfg2['dest_port']}  √")
            ncfg = ncfg2
        else:
            print(f"      × 目的地址未改成（读回 {ncfg['dest_ip']}:{ncfg['dest_port']}）")
            print( "        自身 IP 已改好，可稍后单独补：")
            print(f"          {PYEXE} tools/hr40_ip.py -H {new_ip} set-dest "
                  f"--ip {dest_ip} --dest-port {_dport}")
            print(f"      原配置存档：{arch}")
            return 3

    print(f"\n√ 完成：雷达已从 {host} 改为 {new_ip}")
    if _dest_cannot_receive(ncfg["dest_ip"], new_ip, mask):
        print(f"   ▲ 但目的 IP {ncfg['dest_ip']} 仍不在 {new_ip}/{mask} 网段内，")
        print(f"      雷达的点云发不出去，服务器会收不到数据。需要时补一条：")
        print(f"          {PYEXE} tools/hr40_ip.py -H {new_ip} set-dest --ip <服务器IP> --dest-port 2368")
    print(f"   原配置存档：{arch}")
    print(f"   别忘了同步 config/channels.json 里对应通道的 source_ip → {new_ip}")
    return 0


# ---------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="HR40 雷达网络配置工具（HR40改IP.exe 的 Python 复刻）")
    ap.add_argument("-H", "--host", required=True, help="雷达当前 IP（如 192.168.1.201）")
    ap.add_argument("-p", "--port", type=int, default=DEFAULT_TCP_PORT, help="TCP 端口，默认 9347")
    sub = ap.add_subparsers(dest="action", required=True)

    sub.add_parser("read", help="读取雷达配置（自身IP/掩码/网关/目的IP/目的端口）")
    sub.add_parser("inventory", help="读取设备清单（序列号/型号/日期/MAC/固件版本）")
    sub.add_parser("angle", help="读取角度校正表(CSV)")
    sub.add_parser("restart", help="重启雷达")
    p_self = sub.add_parser("set-self", help="修改雷达自身网络（重启生效，不会自动重启）")
    p_self.add_argument("--ip", required=True)
    # 这里**故意不给默认值**：静默填 192.168.1.1 之类的网关，会让人以为改对了。
    # 想「沿用雷达当前掩码/网关」请用 provision（它会先读配置再保留原值）。
    p_self.add_argument("--mask", required=True, help="必须显式给出（想沿用原值请用 provision）")
    p_self.add_argument("--gateway", required=True, help="必须显式给出（想沿用原值请用 provision）")
    p_prov = sub.add_parser("provision",
                            help="★推荐：改自身IP + (可选)改目的IP + 自动重启 + 等待 + 验证")
    p_prov.add_argument("--ip", required=True, help="目标新 IP")
    p_prov.add_argument("--mask", default=None,
                        help="目标子网掩码；不填则沿用雷达当前值")
    p_prov.add_argument("--gateway", default=None,
                        help="目标网关；不填则沿用雷达当前值")
    p_prov.add_argument("--dest-ip", dest="dest_ip", default=None,
                        help="同时修改点云目的 IP（服务器地址）；不填则不动")
    p_prov.add_argument("--dest-port", dest="dest_port", type=int, default=None,
                        help="点云目的端口（默认沿用雷达当前值，通常 2368）")
    p_prov.add_argument("--archive-dir", default="", help="存档目录（默认 re/hr40ip/provision）")
    p_prov.add_argument("--wait", type=float, default=90.0, dest="wait_s",
                        help="重启后等待新 IP 上线的秒数，0=不等待")
    p_prov.add_argument("-y", "--yes", action="store_true", dest="assume_yes",
                        help="跳过交互确认")
    p_dest = sub.add_parser("set-dest", help="修改点云目的 IP/端口（立即生效）")
    p_dest.add_argument("--ip", "--dest-ip", dest="dest_ip", required=True,
                        help="点云目的 IP")
    p_dest.add_argument("--dest-port", dest="dest_port", type=int, default=2368,
                        help="点云目的端口（默认 2368）")
    # 注意：此处**故意不叫 --port**。顶层已有 -p/--port 表示 TCP 控制端口；
    # 若同名，argparse 会用子解析器的 default 覆盖顶层取值，导致连错端口。
    p_blk = sub.add_parser("blockage", help="遮挡检测开关")
    p_blk.add_argument("state", choices=["on", "off"])

    args = ap.parse_args(argv)

    if args.action == "provision":
        return provision(args.host, args.ip, args.mask, args.gateway,
                         dest_ip=args.dest_ip, dest_port=args.dest_port,
                         port=args.port, archive_dir=args.archive_dir,
                         wait_s=args.wait_s, assume_yes=args.assume_yes)

    with HR40Client(args.host, args.port) as c:
        if args.action == "read":
            cfg = c.get_config()
            print(f"自身IP   : {cfg['ip']}")
            print(f"子网掩码 : {cfg['subnet_mask']}")
            print(f"网关     : {cfg['gateway']}")
            print(f"目的IP   : {cfg['dest_ip']}")
            print(f"目的端口 : {cfg['dest_port']}")
            print(f"转速     : {cfg['spin_rate']} (原始 u16)")
            print(f"时钟源   : {cfg['clock_source']}  触发方式: {cfg['trigger_method']}")
            print(f"回波模式 : {cfg['return_mode']}  待机模式: {cfg['standby_mode']}")
            print(f"同步     : enable={cfg['sync_enable']} angle={cfg['sync_angle']}")
            print(f"VLAN     : flag={cfg['vlan_flag']} id={cfg['vlan_id']}")
            print(f"(payload {cfg['_payload_len']} 字节)")
        elif args.action == "inventory":
            for k, v in c.get_inventory().items():
                print(f"{k:<22}: {v}")
        elif args.action == "angle":
            sys.stdout.write(c.get_angle_csv())
        elif args.action == "restart":
            c.restart()
            print("重启命令已发送，雷达即将重启。注意：连接会断开，需等待重启完成后重连！")
        elif args.action == "set-self":
            c.set_self_network(args.ip, args.mask, args.gateway)
            print("自身网络配置修改成功！雷达将重启生效。")
            print(f"新IP：{args.ip}  新掩码：{args.mask}  新网关：{args.gateway}")
            print("▲ 请使用新IP重新连接雷达！")
        elif args.action == "set-dest":
            c.set_destination(args.dest_ip, args.dest_port)
            # ★ 必须回读校验：设备对 ReturnCode 校验很松，写 0x20 可能"报成功但没生效"。
            #   回读同时把「雷达自身地址」打出来 —— 用来证明 0x20 只动目的网络、
            #   不会改自身 IP（这是排查"改了单播后网络不通"的关键证据）。
            cfg = c.get_config()
            got_ip = cfg.get("dest_ip")
            got_port = cfg.get("dest_port")
            self_ip = cfg.get("ip")
            if got_ip == args.dest_ip and got_port == args.dest_port:
                print(f"目的配置修改成功并已回读确认：{got_ip}:{got_port}")
                print("注意：此修改通常立即生效，无需重启。")
                print(f"（回读雷达自身地址 = {self_ip}，本次修改未影响它）")
                if args.dest_ip == "255.255.255.255":
                    print("▲ 目的地址是广播 255.255.255.255 —— 多台雷达同时广播会"
                          "泛洪整个二层网段，打死同网段的百兆设备。")
            else:
                print(f"▲ 目的配置回读不一致！写入 {args.dest_ip}:{args.dest_port}"
                      f"，设备实际 {got_ip}:{got_port}")
                print("  这次写入没有生效（设备对 ReturnCode 校验很松，会静默失败）。")
                print("  请重试；若反复不一致，核对 0x20 载荷尾部是否为 00 00。")
                return 4
        elif args.action == "blockage":
            c.set_blockage_detection(args.state == "on")
            print("遮挡检测设置成功！状态: " + ("开启" if args.state == "on" else "关闭"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
