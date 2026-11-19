#!/usr/bin/env python3
"""现场网络诊断（纯标准库，麒麟/Linux 可用）——多雷达部署排障专用。

现场三大高频问题，本工具一次覆盖：

  1. 雷达到底接上网没有、在哪、是第几台  ->  scan
  2. 点云有没有真的到本机、哪几路在发     ->  traffic
  3. 是不是广播风暴把别的设备（交换机/PLC）打死了 -> traffic 的判定

用法
----
  python tools/net_diag.py scan                       # 扫 192.168.1/24 + 192.168.201/24
  python tools/net_diag.py scan 192.168.1             # 只扫一个网段
  python tools/net_diag.py traffic                    # 测 UDP 2368 点云流量（按源 IP）
  python tools/net_diag.py traffic --seconds 8
  python tools/net_diag.py traffic --bind 198.51.100.15
                                                      # 绑本机某地址测：能从"别的网卡地址"
                                                      # 收到 => 那路是广播；只能从不含它的
                                                      # 地址收到 => 那路是单播

背景结论（2026-09-13 现场实测，务必先读）
-----------------------------------------
* 雷达出厂默认 **目的 IP = 255.255.255.255（受限广播）**。
  广播帧会被交换机泛洪到整个二层网段的每个端口。
* 4 台雷达同时广播 = 约 **39 MB/s / 310 Mbps**，会把同网段的百兆设备
  （实测 198.51.100.5 / .6，保定银河电子）直接打死：ping 不通、
  拔掉雷达网线立刻恢复。这就是"广播风暴"。
* 正解：把每台雷达的目的 IP 改成 **服务器单播地址**（如 198.51.100.15），
  点云只发给服务器，不再泛洪。改完实测降到 29.6 MB/s（3 路单播）。
* 判定要点：某一路的**源 IP 不在你配置的雷达清单里** => 有未纳管设备在发广播，
  或某台雷达改 IP 后数据流源地址仍停在旧 IP。

雷达 MAC 厂商前缀：ec:9f:0d（本工具据此标出"这是雷达"）。
"""
import argparse
import concurrent.futures
import socket
import subprocess
import sys
import time

RADAR_MAC_PREFIX = "ec-9f-0d"
CLOUD_PORT = 2368
DEFAULT_PREFIXES = ("192.168.1", "192.168.201")

# 广播风暴阈值（单位 Mbps）。
# 单台雷达实测约 9.9 MB/s ≈ 79 Mbps；4 台全广播实测 306 Mbps。
# 注意：这只是"量"的分级；是否真的泛洪还取决于目的地是不是广播地址，
# 单播到服务器时速率再高也不泛洪（判定文案里已说明）。
STORM_NOTICE_MBPS = 100     # 约 1 路以上
STORM_ALERT_MBPS = 200      # 约 2 路以上
STORM_FLOOD_MBPS = 300      # 约 4 路全广播 —— 现场实测就是这一档


# --------------------------------------------------------------------------
# 纯函数（可单测）
# --------------------------------------------------------------------------
def norm_mac(mac: str) -> str:
    """统一成小写冒号分隔，便于比较。"""
    return mac.strip().lower().replace("-", ":")


def is_radar_mac(mac: str) -> bool:
    """判断是否雷达 MAC（ec:9f:0d 前缀）。"""
    return norm_mac(mac).startswith(RADAR_MAC_PREFIX.replace("-", ":"))


def parse_arp(text: str):
    """解析 `arp -a` 输出 -> [(iface, ip, mac)]。

    中文 Windows 的接口行形如 `接口: 192.168.1.15 --- 0xc`；
    数据行形如 `198.51.100.2   ec-9f-0d-02-85-15  动态`。
    对英文/其它语系用 `interface:`，此处按"含 3 个字段且首字段是 IP"兜底。
    """
    rows = []
    iface = ""
    for line in text.splitlines():
        s = line.rstrip()
        if not s.strip():
            continue
        low = s.strip().lower()
        if low.startswith("\u63a5\u53e3") or low.startswith("interface"):
            iface = s.strip()
            continue
        parts = s.split()
        if len(parts) >= 3 and parts[0].count(".") == 3:
            rows.append((iface, parts[0], parts[1]))
    return rows


def storm_level(mbps: float) -> str:
    """按速率给广播风暴分级：ok / notice / alert / flood。"""
    if mbps <= STORM_NOTICE_MBPS:
        return "ok"
    if mbps <= STORM_ALERT_MBPS:
        return "notice"
    if mbps <= STORM_FLOOD_MBPS:
        return "alert"
    return "flood"


def mbps_of(pkts: int, bytes_: int, seconds: float) -> float:
    """折算 Mbps（按 8 bit/byte）。"""
    if seconds <= 0:
        return 0.0
    return bytes_ * 8 / seconds / 1e6


# --------------------------------------------------------------------------
# scan
# --------------------------------------------------------------------------
def _ping_once(host: str) -> None:
    """发一个 ping 只为了触发 ARP（Windows/Linux 参数不同）。"""
    if sys.platform.startswith("win"):
        cmd = ["ping", "-n", "1", "-w", "400", host]
    else:
        cmd = ["ping", "-c", "1", "-W", "1", host]
    try:
        subprocess.run(cmd, capture_output=True, timeout=3)
    except Exception:
        pass


def _read_arp() -> str:
    """读 ARP 表（Windows 输出 GBK，Linux 输出 UTF-8，逐级尝试）。"""
    out = subprocess.run(["arp", "-a"], capture_output=True).stdout
    for enc in ("utf-8", "gbk"):
        try:
            return out.decode(enc)
        except UnicodeDecodeError:
            continue
    return out.decode("utf-8", "replace")


def _is_noise_ip(ip: str) -> bool:
    """多播 / 广播 / 回环 / 子网广播 —— 这些不是"设备"，别混进在线列表。

    ARP 表里永远有 `224.0.0.22`、`239.255.255.250`、`255.255.255.255`、
    `<网段>.255` 这些静态项，把它们当"在线设备"打印会淹没有用信息。
    """
    if ip.startswith(("224.", "239.", "255.", "127.")):
        return True
    return ip.endswith((".0", ".255"))


def _in_prefixes(ip: str, prefixes) -> bool:
    """ip 是否落在本次扫描的网段里。"""
    return any(ip.startswith(p + ".") for p in prefixes)


def _ip_sort_key(row):
    try:
        return tuple(int(x) for x in row[1].split("."))
    except ValueError:
        return (999, 999, 999, 999)


def cmd_scan(args) -> int:
    prefixes = args.prefixes or list(DEFAULT_PREFIXES)
    hosts = [f"{p}.{i}" for p in prefixes for i in range(1, 255)]
    print(f"扫描网段：{', '.join(p + '.0/24' for p in prefixes)}（共 {len(hosts)} 个地址）")
    with concurrent.futures.ThreadPoolExecutor(max_workers=96) as ex:
        list(ex.map(_ping_once, hosts))

    rows = parse_arp(_read_arp())
    if not rows:
        print("ARP 表为空 —— 没扫到任何设备，检查网线/网卡。")
        return 2

    # ★ 必须区分「本次扫描网段的设备」与「ARP 表里的残留项」。
    #   实测踩坑（2026-09-14）：扫描 192.168.1 时，ARP 表里仍留着上一轮
    #   198.51.100.2 的记录，本工具把它当成"在线雷达"打印，直接误导判断
    #   （当时 192.168.1.201 其实根本不应答 ARP）。
    #   只看本次 ping 过的网段才可信；其它网段的条目另列并标明可能已失效。
    in_scope = [r for r in rows
                if _in_prefixes(r[1], prefixes) and not _is_noise_ip(r[1])]
    out_scope = [r for r in rows
                 if not _in_prefixes(r[1], prefixes) and not _is_noise_ip(r[1])]

    print()
    print(f"== 在线设备（本次扫描网段，{len(in_scope)} 台）==")
    if not in_scope:
        print("  （无 —— 该网段没有任何设备应答 ARP）")
    for _iface, ip, mac in sorted(in_scope, key=_ip_sort_key):
        tag = "  <== 雷达" if is_radar_mac(mac) else ""
        print(f"  {ip:16s} {mac:20s}{tag}")

    if out_scope:
        print()
        print(f"== ARP 残留（不属于本次扫描网段，可能已失效）{len(out_scope)} 条 ==")
        for _iface, ip, mac in sorted(out_scope, key=_ip_sort_key):
            tag = "  <== 雷达" if is_radar_mac(mac) else ""
            print(f"  {ip:16s} {mac:20s}{tag}")
        if any(is_radar_mac(r[2]) for r in out_scope):
            print("  提示：上面有雷达却不在你扫的网段 —— 说明本机地址可能不在它的网段里，")
            print("        或者这是上一轮配置留下的 ARP 缓存（改完 IP 后常见）。")

    radars = [r for r in in_scope if is_radar_mac(r[2])]
    print()
    print(f"雷达数量：{len(radars)}")
    for _, ip, mac in radars:
        print(f"  {ip:16s} {mac}")
    expect = args.expect
    if expect:
        have = {r[1] for r in radars}
        missing = [e for e in expect if e not in have]
        if missing:
            print()
            print(f"▲ 配置里应有但没扫到：{', '.join(missing)}")
            print("  （该雷达未上电 / 网线没插 / 还在用旧的出厂 IP）")
    return 0 if radars else 2


# --------------------------------------------------------------------------
# traffic
# --------------------------------------------------------------------------
def cmd_traffic(args) -> int:
    bind = args.bind
    secs = args.seconds
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind((bind, CLOUD_PORT))
    except OSError as exc:
        print(f"绑定 {bind}:{CLOUD_PORT} 失败：{exc}")
        print("提示：若服务已在跑并占用了该端口，用 --bind 指定本机某个具体地址再试。")
        return 3
    s.settimeout(0.5)

    t0 = time.time()
    n = b = 0
    per = {}
    while time.time() - t0 < secs:
        try:
            data, src = s.recvfrom(65535)
        except socket.timeout:
            continue
        except OSError:
            break
        n += 1
        b += len(data)
        per[src[0]] = per.get(src[0], (0, 0))
        c, sz = per[src[0]]
        per[src[0]] = (c + 1, sz + len(data))
    dt = time.time() - t0
    s.close()

    mbps = mbps_of(n, b, dt)
    print(f"[绑定 {bind}] 收到 {n} 包 / {dt:.1f}s")
    print(f"合计 {b / dt / 1e6:8.2f} MB/s = {mbps:7.0f} Mbps , {n / dt:8.0f} pkt/s")
    print()
    print("按源 IP：")
    for ip, (c, sz) in sorted(per.items()):
        print(f"  {ip:16s} {c:8d} 包  {c / dt:8.0f} pkt/s  {sz / dt / 1e6:6.2f} MB/s")
    if not per:
        print("  （无数据 —— 点云没到本机：目的 IP 不对 / 网卡没在目标网段 / 雷达没发）")

    lvl = storm_level(mbps)
    if lvl == "ok":
        print()
        print("判定：√ 流量正常，不构成泛洪风险。")
    else:
        print()
        print(f"判定：▲ 总速率 {mbps:.0f} Mbps（{lvl}）—— 若这些都是**广播**")
        print("      目的地，会泛洪到整个二层网段，打死同网段的百兆/弱设备。")
        print("      处置：把每台雷达目的 IP 从 255.255.255.255 改成服务器单播地址：")
        print("      python tools/hr40_ip.py -H <雷达IP> set-dest --ip <服务器IP> --dest-port 2368")
    return 0


# --------------------------------------------------------------------------
# reach —— 「这台雷达 ping 不通」专用
# --------------------------------------------------------------------------
def egress_for(ip: str) -> str:
    """内核会挑本机哪个地址去找 ip。

    UDP connect 不发任何包，只触发一次路由决策 —— 这正是
    「多网卡时 ping 走错网卡」那个坑（2026-09-13 踩过，WLAN 抢了出口）
    的检测办法，而且零副作用。
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((ip, 9))
        return s.getsockname()[0]
    except OSError as exc:
        return f"(路由不到: {exc})"
    finally:
        s.close()


def same_subnet24(a: str, b: str) -> bool:
    """粗判同 /24。现场全是 255.255.255.0，够用。"""
    try:
        pa = [int(x) for x in a.split(".")]
        pb = [int(x) for x in b.split(".")]
    except ValueError:
        return False
    if len(pa) != 4 or len(pb) != 4:
        return False
    return pa[:3] == pb[:3]


def classify_tcp_error(exc: BaseException) -> str:
    """把 connect 抛出的异常折成一句人话。抽成纯函数是为了能单测 ——

    ★ 三种失败必须分开，它们指向完全不同的原因：
        拒绝(RST) = 主机活着、端口没开   -> 设备在，服务没起 / 端口填错
        超时      = 包发出去没人应       -> 回复路径不通 / 被广播风暴压住
        不可达    = 本机路由层就走不通   -> 出口网卡或路由表问题
    混成一句"连不上"等于没诊断。

    注意 **不能靠实际连一个关闭的端口来测**这个分类：Windows 对本机
    关闭端口是静默丢弃（超时），Linux 才回 RST（拒绝）—— 同一段代码
    两个平台走不同分支。所以判定逻辑与 socket 行为必须解耦。
    """
    # ConnectionRefusedError 要排在 TimeoutError 之前判：两者都是 OSError 子类，
    # 但互相不是父子关系，靠 isinstance 逐个试即可，次序只影响可读性。
    if isinstance(exc, ConnectionRefusedError):
        return "被拒绝(主机在，此端口没开)"
    if isinstance(exc, (socket.timeout, TimeoutError)):
        return "超时(包发出去了，没人应)"
    return f"不可达({exc.__class__.__name__}: {exc})"


def tcp_probe(ip: str, port: int = 9347, timeout: float = 1.5):
    """TCP 探测，返回 (通不通, 毫秒, 说明)。分类口径见 classify_tcp_error。"""
    t0 = time.time()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((ip, port))
        return True, (time.time() - t0) * 1000, "已连通"
    except OSError as exc:
        return False, (time.time() - t0) * 1000, classify_tcp_error(exc)
    finally:
        try:
            s.close()
        except OSError:
            pass


def icmp_alive(ip: str, timeout_ms: int = 800):
    """ping 一次。

    ★ 不能只看返回码 —— Windows 的 ping 收到「无法访问目标主机」时**也返回 0**。
    只有真正的应答报文才带 TTL，所以以输出里有没有 TTL= 为准。
    """
    if sys.platform.startswith("win"):
        cmd = ["ping", "-n", "1", "-w", str(int(timeout_ms)), ip]
    else:
        cmd = ["ping", "-c", "1", "-W", str(max(1, int(timeout_ms / 1000))), ip]
    try:
        p = subprocess.run(cmd, capture_output=True,
                           timeout=max(5.0, timeout_ms / 1000 + 3))
    except Exception:  # noqa: BLE001
        return False
    if p.returncode != 0:
        return False
    out = (p.stdout or b"") + (p.stderr or b"")
    for enc in ("gbk", "utf-8", "latin-1"):
        try:
            return "ttl=" in out.decode(enc).lower()
        except UnicodeDecodeError:
            continue
    return False


def reach_verdict(tcp_ok: bool, icmp_ok: bool, arp_ok: bool,
                  same_net: bool, storm_mbps: float) -> str:
    """把四项观测折成一个结论码。抽成纯函数是为了能单测。

    次序有讲究：**先判风暴**。因为风暴会把 ARP/ICMP/TCP 一起压死，
    此时去查"掩码配错没"只会白费功夫 —— 2026-09-13 现场就是拔掉
    雷达网线立刻恢复，网络配置一个字没动。
    """
    if not same_net:
        return "wrong_egress"
    if tcp_ok and icmp_ok:
        return "ok"
    if tcp_ok and not icmp_ok:
        return "ok_no_icmp"
    if storm_mbps > STORM_NOTICE_MBPS:
        return "storm_suppressed"
    if arp_ok:
        return "l2_only"
    return "unreachable"


_REACH_TEXT = {
    "ok": "网络完全正常。",
    "ok_no_icmp": "网络正常 —— 控制口通，说明回复路径是好的；"
                  "ping 不通只是这个固件不实现 ICMP，不用管、更不用重启雷达。",
    "wrong_egress": "本机出口地址与雷达不同网段 —— 包被送去了网关，"
                    "而雷达不在网关后面。先给网卡配一个同网段地址。",
    "storm_suppressed": "★ 多半是广播风暴把它压住了。雷达自己也在挨其它几台的广播，"
                        "百兆口扛不住就会把 ARP/ICMP/TCP 一起丢掉。"
                        "验证：断开其它雷达再 ping 这一台。",
    "l2_only": "二层见过它(ARP 有 MAC)但三层回不来 —— "
               "多半是雷达自己的掩码/网关配错：收得到你的包，算不出回复该发给谁。"
               "广播照发不误，正好对得上现象。",
    "unreachable": "既没 ARP 也没应答。查：网线/交换机端口、"
                   "雷达是否真在这个地址(看 traffic 里的源 IP)、本机防火墙。",
}


def cmd_reach(args) -> int:
    ip = args.host
    print(f"== 可达性诊断 {ip} ==")
    print()

    eg = egress_for(ip)
    same = same_subnet24(eg, ip) if not eg.startswith("(") else False
    print(f"[1/5] 本机出口地址 : {eg}")
    print(f"      {'√ 与雷达同网段' if same else '▲ 与雷达不同网段 —— 这本身就会导致 ping 不通'}")

    tcp_ok, ms, why = tcp_probe(ip, args.tcp_port)
    print(f"[2/5] TCP {args.tcp_port:<5}      : {'√' if tcp_ok else '×'} {why}  ({ms:.0f} ms)"
          f"   <== 以这条为准，不是 ping")

    icmp_ok = icmp_alive(ip)
    print(f"[3/5] ICMP (ping)  : {'通' if icmp_ok else '不通'}   (参考项；很多雷达固件根本不实现 ICMP)")

    rows = parse_arp(_read_arp())
    hit = [r for r in rows if r[1] == ip]
    arp_ok = bool(hit)
    if arp_ok:
        print(f"[4/5] ARP 表       : √ {hit[0][2]}"
              f"{'  <== 雷达 MAC' if is_radar_mac(hit[0][2]) else ''}")
    else:
        print("[4/5] ARP 表       : × 没有它的条目(没应答 ARP，或条目已老化)")

    storm = 0.0
    if args.storm_seconds > 0:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("0.0.0.0", CLOUD_PORT))
            s.settimeout(0.5)
            t0, nb = time.time(), 0
            while time.time() - t0 < args.storm_seconds:
                try:
                    nb += len(s.recvfrom(65535)[0])
                except socket.timeout:
                    continue
                except OSError:
                    break
            dt = time.time() - t0
            storm = nb * 8 / dt / 1e6 if dt > 0 else 0.0
            print(f"[5/5] 同时段广播量 : {storm:.0f} Mbps ({storm_level(storm)})")
        except OSError as exc:
            print(f"[5/5] 同时段广播量 : 测不了，{CLOUD_PORT} 端口被占({exc})")
            print("      ★ 两个程序抢同一个 UDP 端口在 Windows 上**不报错**，")
            print("        但包只会进其中一个 —— 这正是丢包率高的典型原因。先找出是谁：")
            print(f"        netstat -ano -p UDP | findstr :{CLOUD_PORT}")
        finally:
            s.close()
    else:
        print("[5/5] 同时段广播量 : (--storm-seconds 0，跳过)")

    v = reach_verdict(tcp_ok, icmp_ok, arp_ok, same, storm)
    print()
    print("结论：" + _REACH_TEXT[v])
    if v in ("storm_suppressed", "l2_only", "unreachable"):
        print()
        print("根治广播风暴(**立即生效，不用重启雷达**)：")
        print(f"  python tools/hr40_ip.py -H {ip} set-dest --ip <服务器IP> --dest-port {CLOUD_PORT}")
        print("  注意：这条也要走 TCP 9347。连不上就先把其它雷达断开，")
        print("        腾出带宽后再连这一台改，改完再逐台恢复。")
    print()
    print("提醒：即使控制口不通，只要雷达在**广播**点云，源 IP 分流照样命中，")
    print("      采集不受影响 —— 控制口只影响改配置。")
    return 0 if v in ("ok", "ok_no_icmp") else 2


# --------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="现场网络诊断：找雷达 / 测点云流量 / 判定广播风暴",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("用法")[-1] if __doc__ else None)
    sub = ap.add_subparsers(dest="action")

    p_scan = sub.add_parser("scan", help="扫网段找雷达（按 MAC 前缀 ec:9f:0d 识别）")
    p_scan.add_argument("prefixes", nargs="*", help="网段前缀，如 192.168.201（默认两个）")
    p_scan.add_argument("--expect", nargs="*", default=None,
                        help="配置里应有的雷达 IP，缺失时给提示")

    p_tr = sub.add_parser("traffic", help="测 UDP 2368 点云流量（按源 IP 分类 + 风暴判定）")
    p_tr.add_argument("--bind", default="0.0.0.0",
                      help="绑定的本机地址（默认 0.0.0.0；指定具体地址可区分广播/单播）")
    p_tr.add_argument("--seconds", type=float, default=6.0, help="采样秒数（默认 6）")

    p_re = sub.add_parser("reach", help="单台可达性诊断（ping 不通但点云收得到 -> 用这个）")
    p_re.add_argument("host", help="雷达 IP，如 192.168.1.201")
    p_re.add_argument("--tcp-port", type=int, default=9347)
    p_re.add_argument("--storm-seconds", type=float, default=3.0,
                      help="同时段广播量采样秒数，0 = 跳过（默认 3）")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.action == "scan":
        return cmd_scan(args)
    if args.action == "traffic":
        return cmd_traffic(args)
    if args.action == "reach":
        return cmd_reach(args)
    build_parser().print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
