#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/capture.py — 抓雷达原始报文（现场校准第一步）

现场流程通常是这三步，顺序别反：
    ① capture.py   先"把雷达的原始报文抓下来"——不解释、不猜，原样存盘
    ② probe.py     拿抓包反推「距离换算模式 / 角度标度 / z 方向」到底该取什么
    ③ run.py       配置定稿后，才启动正式服务

为什么先抓包，而不是直接 run.py？
    直接 run.py 如果配置不对，看到的现象是"点云乱七八糟"或"点云数量不足"，
    这类现象有多种可能原因，很难判断到底是哪一环。
    先抓原始报文，就有了一个**可反复复算的不变输入**：
    后面所有参数试错都在本地对同一份抓包跑，不用雷达一直在旁边开着。

输出格式（与 app/radar_probe.load_capture 的格式 A 对应）
    重复 [uint16 LE 长度][报文原始字节]
    之所以加长度前缀而不是裸拼：现场雷达的包长未必严格等于配置里的 1100B，
    带长度前缀才能忠实记录"实际收到了多长的包"，这正是校准需要的信息。

────────────────────────────────────────────────────────────────────────
用法
────────────────────────────────────────────────────────────────────────

    # 抓 3000 个包到文件（最常用）
    python3 cli/capture.py --count 3000 --out re/captures/live_20260912.bin

    # 抓 15 秒
    python3 cli/capture.py --seconds 15 --out re/captures/live.bin

    # 绑定地址/端口不在配置里时手填
    python3 cli/capture.py --bind 0.0.0.0 --port 2368 --count 3000

    # 抓完顺手尝试解码（用当前 config.json），看能不能出点云
    python3 cli/capture.py --count 2000 --decode-check

    # ★ 多雷达：4 台雷达接在同一台机器上时，按源 IP 分开存，
    #   并逐路报告包率与**估计转速**（用来定 frame_ms）
    python3 cli/capture.py --seconds 20 --split-by-ip

多雷达现场提示
    · 4 台雷达都发往同一台机器的同一个端口(2368) —— 这是正常接法，
      本工具用一个 socket 收全部，再按源 IP 分开统计/存盘。
    · --split-by-ip 会为每个源 IP 生成一个 live_<时间>_<IP>.bin，
      并打印"每路包数 / 包率 / 估计转速"。**转速是 frame_ms 的依据**：
      frame_ms 应约等于 1000 / 转速Hz（转一圈的时长）。
    · 若某个 IP 一个包都没收到 → 那台雷达的接线/目标地址/网段有问题，
      而不是程序问题（看板 hub 区也会显示它是 0）。

现场提示
    · 端口占用：报 "Address already in use" 说明 run.py 还开着，先停掉它。
    · 收不到包：确认雷达的目标 IP 指向本机、防火墙放行、bind 地址没写错
      （多网卡机器建议显式 --bind 本机在该网段的 IP）。
    · 需要 root：端口 < 1024 才需要；2368 一般不用，但麒麟上若报权限错，
      先试 sudo。
"""
from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import time
from collections import Counter, defaultdict

import _common
_common.setup_console()

from app.settings import baseline_rel, load_settings, load_calib, ensure_dir
from app.pointcloud import AngleCache
from app.radar_packet import PacketDecoder, PacketError


def _pick_bind_ip(cli_bind: str, ch) -> str:
    """选择抓包 socket 的绑定地址。

    优先级：命令行 --bind > 通道配置 lidar.bind_ip > 0.0.0.0

    ★ 不要 fallback 到 ch.lidar.ip —— 那是**雷达**的地址，本机通常并不持有它，
      bind 必然失败（WinError 10049：在其上下文中该请求的地址无效）。
      多雷达部署约定各通道 bind_ip 一律留空，由单一 socket 绑 0.0.0.0
      再按 recvfrom 的源 IP 分流（见 config/channels.json 顶部说明）。
    """
    return cli_bind or getattr(ch.lidar, "bind_ip", "") or "0.0.0.0"


def _report_loss(ip: str, revs, worst_gap_deg: float) -> None:
    """按「圈」报告丢包 —— 这是判断"这一路的数据能不能用"的唯一可靠指标。

    ★ 为什么不能看平均包率：突发丢包让少数圈缺掉一整块连续扇形，
      而平均值会把它摊平。850/901 的平均看着只少 6%，实际可能是
      「90% 的圈是满的、10% 的圈缺 1/3」—— 后者那些帧直接不能用。

    判据：以**中位数**为"应有包数"（不用最大值，避免个别超长圈把基准抬高），
    某一圈 >= 中位数 x 0.99 才算完整（与 service._COMPLETE_RATIO 同口径）。

    首尾两圈一律丢弃 —— 抓包起止时刻多半落在半圈上，不是真的缺包。
    """
    if len(revs) < 4:
        print("      | 圈数太少（%d），测不出丢包率。至少抓 3 秒。" % len(revs))
        return
    body = sorted(revs[1:-1])
    n = len(body)
    med = body[n // 2] if n % 2 else (body[n // 2 - 1] + body[n // 2]) / 2.0
    if med <= 0:
        print("      | 方位角没推进，测不出丢包率。")
        return
    full = sum(1 for v in body if v >= med * 0.99)
    ratio = full / n
    worst = min(body)
    lost = (1.0 - worst / med) * 100.0
    mark = "OK" if ratio >= 0.99 else ("!!" if ratio >= 0.90 else "XX")
    print("      | %s 完整圈 %d/%d (%.1f%%)  中位 %.0f 包/圈  最差一圈 %d 包（缺 %.0f%%）"
          % (mark, full, n, ratio * 100.0, med, worst, lost))
    if worst_gap_deg > 1.0:
        print("      |    最大连续缺口 %.1f 度（约 %d 个报文）"
              % (worst_gap_deg, int(round(worst_gap_deg / 360.0 * med))))
    if ratio < 0.99:
        print("      |    ▲ 残缺帧不参与停稳判定、也不会被存成全量帧，")
        print("      |      这一路的采集会变慢甚至采不到。先查：")
        print("      |      1) 有没有别的程序也在收 2368（Windows 下不报错，但会抢包）")
        print("      |      2) 广播风暴：tools/net_diag.py traffic")
        print("      |      3) 网卡/交换机是不是百兆口")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="capture.py", description="抓雷达原始 UDP 报文",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("用法")[0])
    ap.add_argument("--out", default="", help="输出文件（默认 re/captures/live_<时间>.bin）")
    ap.add_argument("--count", type=int, default=0, help="抓够这么多包就停")
    ap.add_argument("--seconds", type=float, default=0.0, help="抓够这么多秒就停")
    ap.add_argument("--bind", default="", help="绑定地址（默认取配置 lidar.bind_ip）")
    ap.add_argument("--port", type=int, default=0, help="UDP 端口（默认取配置 lidar.udp_port）")
    ap.add_argument("--channels", default="", help="多通道配置路径")
    ap.add_argument("--channel", default="", help="用哪条通道的雷达参数")
    ap.add_argument("--buf-mb", type=int, default=8, help="内核接收缓冲（MB）")
    ap.add_argument("--quiet", action="store_true", help="不打印滚动进度")
    ap.add_argument("--split-by-ip", action="store_true",
                    help="多雷达：按源 IP 分开存盘，并逐路报告包率与估计转速")
    ap.add_argument("--decode-check", action="store_true",
                    help="抓完用当前 config.json 试解码，报告成功率与点云量级")
    args = ap.parse_args(argv)

    st = load_settings(args.channels or "")
    chans = st.channels
    if args.channel:
        chans = [c for c in chans if c.id == args.channel] or chans
    ch = (st.enabled() or chans)[0]

    bind_ip = _pick_bind_ip(args.bind, ch)
    port = args.port or ch.lidar.udp_port or 2368

    base_dir = os.path.join(_common.ensure_root(), "re", "captures")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    if args.out:
        out = (args.out if os.path.isabs(args.out)
               else os.path.join(_common.ensure_root(), args.out))
    else:
        out = os.path.join(base_dir, f"live_{stamp}.bin")
    ensure_dir(os.path.dirname(out) or ".")

    if not args.count and not args.seconds:
        args.count = 3000            # 默认给个限度，避免忘了 Ctrl-C 抓爆磁盘
        print("（未指定 --count/--seconds，默认抓 3000 个包）")

    calib = load_calib(ch.abs_config_path)
    pkt_cfg = calib.packet
    ang_scale = float(calib.processing.angle_scale)

    print("=" * 78)
    print("雷达原始报文抓取")
    print("=" * 78)
    print(f"绑定地址   : {bind_ip}:{port}")
    if args.split_by_ip:
        print(f"输出       : 按源 IP 分文件 → {os.path.dirname(out)}/live_{stamp}_<IP>.bin")
    else:
        print(f"输出文件   : {out}")
    print(f"停止条件   : "
          + "，".join(filter(None, [f"{args.count} 个包" if args.count else "",
                                   f"{args.seconds:g} 秒" if args.seconds else ""])))
    print(f"期望包长   : {pkt_cfg.size} B（来自 config.json，仅作参照）")
    print("-" * 78)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, max(1, args.buf_mb) << 20)
    except OSError:
        pass
    try:
        sock.bind((bind_ip, port))
    except OSError as e:
        print(f"× 绑定 {bind_ip}:{port} 失败：{e}")
        print("   常见原因：① run.py 还在跑，端口被占；② 需要 sudo；"
              "③ 绑错网卡地址。")
        return 1
    sock.settimeout(1.0)

    sizes = Counter()                     # 全局包长分布
    total_bytes = 0
    n = 0
    t0 = time.time()
    t_last = t0
    n_last = 0
    stop = False

    # ── 多雷达逐路统计 ──────────────────────────────────────
    per_ip = Counter()                    # 每个源 IP 的包数
    sizes_ip = defaultdict(Counter)
    last_az = {}                          # 每个源 IP 上一个方位角（估转速用）
    rev_deg = defaultdict(float)          # 每个源 IP 累计方位角前进量（度）
    # ★ 逐圈包数 —— 丢包率必须按「圈」量，不能看平均值。
    #   突发丢包会让一圈缺掉一整块连续扇形（2026-09-14 实测最差连续缺 31 包
    #   ≈ 12.8°），而平均包率会把它摊平：850/901 看着只少 6%，
    #   实际是「大部分圈是满的、少数圈缺一大块」。后者才会毁掉那一帧点云。
    rev_pkts = defaultdict(list)          # 每个源 IP 各圈的包数
    cur_rev = Counter()                   # 当前这一圈已收到的包数
    max_gap = defaultdict(float)          # 单次方位角最大跳变（度）= 最差连续缺口
    files = {}                            # split 模式：每个源 IP 一个文件句柄

    def _fp_for(ip: str):
        fp = files.get(ip)
        if fp is None:
            safe = ip.replace(".", "_").replace(":", "_")
            fp = open(os.path.join(os.path.dirname(out),
                                   f"live_{stamp}_{safe}.bin"), "wb")
            files[ip] = fp
        return fp

    def _azimuth(data: bytes):
        """只读方位角（比完整解码便宜得多）——用于估计转速。"""
        off = pkt_cfg.angle1_offset
        if len(data) < off + 2:
            return None
        try:
            return struct.unpack_from("<H", data, off)[0] * ang_scale
        except struct.error:
            return None

    single_fp = None
    try:
        if not args.split_by_ip:
            single_fp = open(out, "wb")
        while not stop:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                # 超时也检查一下结束条件（--seconds 模式靠它推进）
                if args.seconds and (time.time() - t0) >= args.seconds:
                    break
                continue
            except KeyboardInterrupt:
                break

            ip = addr[0]
            fp = _fp_for(ip) if args.split_by_ip else single_fp
            if fp is not None:
                fp.write(len(data).to_bytes(2, "little"))
                fp.write(data)

            sizes[len(data)] += 1
            sizes_ip[ip][len(data)] += 1
            per_ip[ip] += 1
            total_bytes += len(data) + 2
            n += 1

            az = _azimuth(data)
            if az is not None:
                prev = last_az.get(ip)
                if prev is not None:
                    d = az - prev
                    if d < -180.0:
                        d += 360.0
                    elif d > 180.0:
                        d -= 360.0
                    if d > 0:
                        rev_deg[ip] += d
                        if d > max_gap[ip]:
                            max_gap[ip] = d
                    # 方位角回绕 = 这一圈结束（与 frame_mode="rotation" 同一判据）
                    if az < prev - 180.0:
                        rev_pkts[ip].append(cur_rev[ip])
                        cur_rev[ip] = 0
                cur_rev[ip] += 1
                last_az[ip] = az

            if n == 1 and not args.quiet:
                print(f"首个包来自 : {ip}:{addr[1]}，长度 {len(data)} B")

            if not args.quiet and (time.time() - t_last) >= 2.0:
                dtt = time.time() - t_last
                inst = (n - n_last) / dtt
                extra = ""
                if len(per_ip) > 1:
                    extra = "  各路: " + " ".join(
                        f"{k}({v})" for k, v in sorted(per_ip.items()))
                print(f"  已抓 {n:>7} 包  瞬时 {inst:6.1f} 包/s  "
                      f"累计 {total_bytes/1e6:6.2f} MB{extra}")
                t_last, n_last = time.time(), n

            if args.count and n >= args.count:
                stop = True
            if args.seconds and (time.time() - t0) >= args.seconds:
                stop = True
    except KeyboardInterrupt:
        print("\n（Ctrl-C）停止抓取")
    finally:
        sock.close()
        if single_fp is not None:
            single_fp.close()
        for fp in files.values():
            fp.close()

    dt = max(time.time() - t0, 1e-6)
    print("-" * 78)
    print(f"抓取完成   : {n} 包 / {total_bytes/1e6:.2f} MB / {dt:.1f}s "
          f"（平均 {n/dt:.1f} 包/s）")
    if sizes:
        print("包长分布   : " + "，".join(f"{k}B×{v}" for k, v in sizes.most_common()))
        if len(sizes) > 1:
            print("  ▲ 包长不一致 —— 可能混杂了其它端口/协议的报文，"
                  "probe.py 会用出现最多的那个长度来解析。")
        exp = pkt_cfg.size
        if exp and n != sizes.get(exp, 0):
            print(f"  ▲ 与 config.json 的 {exp}B 不一致 —— "
                  f"若 probe 提示偏移错位，先核对 packet 段的 size/偏移。")

    # ── 多雷达：逐路报告 + 转速估计（frame_ms 的依据）──────────
    if len(per_ip) > 1 or args.split_by_ip:
        print("-" * 78)
        print(f"多雷达逐路统计（{len(per_ip)} 个源 IP）")
        print("  %-16s %9s %10s %20s %14s" %
              ("来源 IP", "包数", "包/s", "估计转速", "建议 frame_ms"))
        for ip in sorted(per_ip):
            c = per_ip[ip]
            rate = c / dt
            rev_hz = (rev_deg.get(ip, 0.0) / 360.0) / dt
            if rev_hz > 0.01:
                spd = "%6.2f Hz (%6.1f rpm)" % (rev_hz, rev_hz * 60.0)
                fms = "%d ms" % int(round(min(2000.0, max(20.0, 1000.0 / rev_hz))))
                ppc = int(round(rate / rev_hz)) if rev_hz > 0 else 0
            else:
                spd = "—（包太少/方位角未推进）"
                fms = "—"
                ppc = 0
            print("  %-16s %9d %10.1f %20s %14s" % (ip, c, rate, spd, fms))
            if ppc:
                print("      | 约 %d 包/圈，%d 包/帧" % (ppc, ppc))
            _report_loss(ip, rev_pkts.get(ip, []), max_gap.get(ip, 0.0))
        print("  提示：默认的 lidar.frame_mode=\"rotation\" 按**方位角整圈**切帧，")
        print("        与转速无关，上面这一栏不用写回配置 —— 只在你把 frame_mode")
        print("        改成 \"time\" 时才需要按它设 lidar.frame_ms。")
        print("        「包/圈」倒是值得记一下：看板通道徽标显示的『整圈切帧 N 包/圈』"
              "应当与它接近。")
        want = [(c.lidar.source_ip or c.lidar.ip)
                for c in st.enabled()
                if (c.mode or "").lower() in ("live", "udp", "real")]
        silent = [ip for ip in want if ip and ip not in per_ip]
        if silent:
            print(f"  ▲ 这些配置里的雷达一个包都没收到：{', '.join(silent)}")
            print("     → 检查它的接线、目标地址、是否与本机同网段")

    if args.split_by_ip:
        print(f"已保存     : 按源 IP 分成 {len(files)} 个文件，位于 "
              f"{os.path.dirname(out)}")
    else:
        print(f"已保存     : {out}")

    rc = 0
    if args.split_by_ip:
        print("\n下一步     : 对某个源的文件跑 probe，例如 "
              + _common.PYEXE
              + " cli/probe.py --capture re/captures/live_*_192_168_201_1.bin "
                "--compare " + baseline_rel())
    elif args.decode_check and n:
        rc = _decode_check(out, ch)
    else:
        print("\n下一步     : " + _common.PYEXE + " cli/probe.py --capture " + _rel(out)
              + " --compare " + baseline_rel())
    print("=" * 78)
    return rc


def _decode_check(capture: str, ch) -> int:
    """用当前配置试解码抓包，报告成功率与点云量级 —— 相当于一次"预演"。"""
    print("-" * 78)
    print("解码预演（用当前 config.json / angle_cache.csv）")
    print("-" * 78)
    sys.path.insert(0, _common.ensure_root())
    from app.radar_probe import load_capture
    from app.radar_packet import PacketDecoder, PacketError
    from app.settings import load_calib
    import numpy as np

    calib = load_calib(ch.abs_config_path)
    ac = AngleCache(ch.abs_angle_cache).load()
    dec = PacketDecoder(calib.packet, calib.processing, ac,
                        range_mode=ch.lidar.range_mode, z_down=ch.lidar.z_down,
                        max_range_m=ch.lidar.max_range_m)
    pkts = load_capture(capture, packet_size=calib.packet.size)
    n_ok = n_bad = 0
    npts = 0
    xs = []
    err = ""
    for b in pkts:
        try:
            pkt = dec.decode(b)
            xyz, _ring, _it = dec.to_cartesian(pkt)
            n_ok += 1
            npts += len(xyz)
            if len(xyz) and len(xs) < 4000:
                xs.append(xyz)
        except PacketError as e:
            n_bad += 1
            err = str(e)
    print(f"报文       : {len(pkts)} 个，解码成功 {n_ok}，失败 {n_bad}")
    if err:
        print(f"首个失败原因: {err}")
    if n_ok:
        print(f"平均点数   : {npts/max(n_ok,1):.1f} 点/包")
    if xs:
        allp = np.concatenate(xs, axis=0)
        lo, hi = allp.min(axis=0), allp.max(axis=0)
        print(f"点云范围   : x[{lo[0]:.2f},{hi[0]:.2f}] "
              f"y[{lo[1]:.2f},{hi[1]:.2f}] z[{lo[2]:.2f},{hi[2]:.2f}]")
        zq = np.percentile(allp[:, 2], [2, 50, 98])
        print(f"z 2/50/98  : {zq[0]:.2f} / {zq[1]:.2f} / {zq[2]:.2f}"
              f"  （历史空车样本 z 中位约 -3.4～-3.8）")
        print("\n→ 若 z 方向反了（正值为主）→ 把 lidar.z_down 改 false；"
              "\n   若量级整体偏大/偏小 → 用 probe.py 定 range_mode。")
    return 0 if n_ok else 2


def _rel(p: str) -> str:
    try:
        return os.path.relpath(p, _common.ensure_root()).replace("\\", "/")
    except ValueError:
        return p


if __name__ == "__main__":
    raise SystemExit(main())
