#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/selftest.py — 无硬件全链路自检

现场到了机器上，还不确定环境是否装对、链路是否通时，先跑这个。
它把整条链路在**不需要雷达、不需要相机**的前提下从头到尾跑一遍：

    A. 配置与标定      配置能读、角度表 hash 校验通过、ROI 合理
    B. 报文编解码      仿真报文 → PacketDecoder → 点云，与几何真值比对
    C. 检测与判定      空车场景 → 「空车」；载货场景 → 「底面过高，疑似载物」
    D. 停稳检测        质心偏差度量对得上（平移/缩放实验）
    E. 服务与看板      服务能起、SSE 有事件、所有 HTTP 接口 200
    F. 落盘与格式      .xyz 写出后能读回、点数与格式一致

输出每项 PASS/FAIL 与关键数字；有 FAIL 时退出码非 0，可直接塞进巡检脚本。

用法
    python3 tools/selftest.py
    python3 tools/selftest.py --quick      # 跳过较慢的仿真环节
"""
from __future__ import annotations

import os
import sys
import json
import time
import argparse
import threading

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import numpy as np                                          # noqa: E402
from app.settings import (load_settings, load_calib, load_devices, root,  # noqa: E402
                          ensure_dir)
from app.pointcloud import AngleCache, Frame, apply_roi, save_xyz, load_xyz  # noqa: E402
from app.radar_packet import PacketDecoder                                  # noqa: E402
from app.radar_sim import RadarSimulator, make_demo_scene                    # noqa: E402
from app.detect import detect_empty, evaluate_empty, StabilityDetector       # noqa: E402
from app.service import LidarService                                         # noqa: E402

# Windows 控制台默认不是 UTF-8，中文与 √/× 会直接抛 UnicodeEncodeError 把脚本打断。
# cli/ 下由 _common.setup_console() 统一处理，tools/ 不依赖 _common，所以自己拉齐。
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")   # py3.7+
    except Exception:                                        # noqa: BLE001
        pass

RESULTS = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    RESULTS.append((name, ok, detail))
    mark = "√ PASS" if ok else "× FAIL"
    print(f"{mark}  {name}")
    if detail:
        for line in detail.splitlines():
            print(f"        {line}")
    return ok


def _frame_from_packets(packets, dec):
    xs, rs = [], []
    for b in packets:
        pkt = dec.decode(b)
        xyz, ring, _ = dec.to_cartesian(pkt)
        if len(xyz):
            xs.append(xyz)
            rs.append(ring)
    if not xs:
        return None
    return Frame(np.concatenate(xs, axis=0), np.concatenate(rs, axis=0))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="selftest.py", description="无硬件全链路自检")
    ap.add_argument("--quick", action="store_true", help="跳过较慢的仿真环节")
    ap.add_argument("--channels", default="", help="多通道配置路径")
    ap.add_argument("--port", type=int, default=0, help="看板自检端口（0=随机）")
    args = ap.parse_args(argv)

    t_all = time.time()
    print("=" * 78)
    print("载重雷达 —— 无硬件全链路自检")
    print("=" * 78)

    # ── A. 配置与标定 ────────────────────────────────────────
    print("\n【A】配置与标定")
    try:
        st = load_settings(args.channels or "")
        check("读取 channels.json", len(st.channels) > 0,
              f"{st.channels_path}\n通道: " + ", ".join(c.id for c in st.channels))
        enabled = st.enabled()
        check("至少有一条启用通道", len(enabled) > 0,
              "启用: " + ", ".join(f"{c.id}({c.mode})" for c in enabled))
        ch = enabled[0]
    except Exception as e:                                   # noqa: BLE001
        check("读取 channels.json", False, f"{type(e).__name__}: {e}")
        return _summary(t_all)

    try:
        cfg = load_calib(ch.abs_config_path)
        dev = load_devices(ch.abs_devices_path)
        check("读取 config.json", True,
              f"报文 {cfg.packet.size}B / {cfg.packet.channels}通道，"
              f"yaw_offset {cfg.processing.yaw_offset_deg}°")
        check("读取 devices.json", True,
              f"门槛 内点≥{dev.gate.min_inliers} 占比≥{dev.gate.min_inlier_ratio} "
              f"spanX≥{dev.gate.min_span_x_m} spanY≥{dev.gate.min_span_y_m}")
        lo, hi = cfg.roi
        ok_roi = (hi[0] > lo[0] and hi[1] > lo[1] and hi[2] > lo[2])
        check("ROI 区间合法", ok_roi, f"lo={lo} hi={hi}")
    except Exception as e:                                   # noqa: BLE001
        check("读取 config/devices", False, f"{type(e).__name__}: {e}")
        return _summary(t_all)

    ac = AngleCache(ch.abs_angle_cache).load()
    check("角度标定表加载", bool(ac.loaded and ac.channels_seen >= 128),
          ac.check())
    check("角度标定表校验值", bool(ac.hash_ok),
          "前 129 行（表头+128 通道）按 LF 归一化后的 SHA-256 与文件内校验行一致"
          if ac.hash_ok else f"不匹配：{ac.error}")

    # ── B. 报文编解码 ───────────────────────────────────────
    print("\n【B】报文编解码（仿真报文 → 解码 → 几何真值比对）")
    dec = PacketDecoder(cfg.packet, cfg.processing, ac,
                        range_mode=ch.lidar.range_mode, z_down=ch.lidar.z_down,
                        max_range_m=ch.lidar.max_range_m)
    check("报文布局自检", True,
          f"每 block {dec._pps} 点，每包 {dec._npoints} 点，"
          f"两块{'连续' if dec._blocks_contiguous() else '分离'}")

    sim = RadarSimulator(cfg.packet, cfg.processing, ac,
                         range_mode=ch.lidar.range_mode, z_down=ch.lidar.z_down,
                         az_step_deg=2.0, range_noise_m=0.0, dropout_prob=0.0, seed=3)
    boxes = make_demo_scene(loaded=False, z_floor=-5.0)
    truth = sim.truth_cloud(boxes, az_step=2.0)
    pkts = sim.scan(boxes)
    fr = _frame_from_packets(pkts, dec)
    if fr is None:
        check("解码产出点云", False, "没有解出任何点")
    else:
        tlo, thi = truth.min(axis=0), truth.max(axis=0)
        flo, fhi = fr.xyz.min(axis=0), fr.xyz.max(axis=0)
        err = float(np.max(np.abs(np.array([flo - tlo, fhi - thi]))))
        check("解码产出点云", fr.n > 10000, f"{fr.n} 点（真值 {len(truth)} 点）")
        check("解码几何与真值一致（bbox 最大偏差 < 0.10 m）", err < 0.10,
              f"偏差 {err:.5f} m\n真值 bbox x[{tlo[0]:.2f},{thi[0]:.2f}] "
              f"y[{tlo[1]:.2f},{thi[1]:.2f}] z[{tlo[2]:.2f},{thi[2]:.2f}]\n"
              f"解码 bbox x[{flo[0]:.2f},{fhi[0]:.2f}] "
              f"y[{flo[1]:.2f},{fhi[1]:.2f}] z[{flo[2]:.2f},{fhi[2]:.2f}]")
        # z 方向：地板应当在 -5 附近（z_down 的证据）
        floor_lo = float(fr.xyz[:, 2].min())
        check("z 方向正确（地板 ≈ -5 m，z 向下为负）",
              -6.0 < floor_lo < -3.0,
              f"最低点 z = {floor_lo:.3f} m；若在 +5 附近，把 lidar.z_down 改成 false")

    # ── C. 检测与判定 ───────────────────────────────────────
    print("\n【C】检测与判定")
    if args.quick:
        print("        （--quick：跳过）")
    else:
        for label, loaded in (("空车", False), ("载货", True)):
            sim2 = RadarSimulator(cfg.packet, cfg.processing, ac,
                                  range_mode=ch.lidar.range_mode,
                                  z_down=ch.lidar.z_down, az_step_deg=1.0,
                                  range_noise_m=0.004, dropout_prob=0.01, seed=7)
            pk = sim2.scan(make_demo_scene(loaded=loaded, z_floor=-5.0))
            f2 = _frame_from_packets(pk, dec)
            f2 = apply_roi(f2, cfg.roi) if f2 is not None else None
            if f2 is None:
                check(f"{label}场景检测", False, "没有解出点云")
                continue
            res = detect_empty(f2, cfg)
            ok, reason, dg = evaluate_empty(res, f2.n, res.cloud, dev)
            if loaded:
                good = (not ok) and reason != ""
                check(f"{label}场景应判为非空", good,
                      f"实测判定={ok}（{'空车' if ok else '非空'}）理由={reason}\n"
                      f"内点={dg.inliers} 占比={dg.ratio:.3f}")
            else:
                check(f"{label}场景应判为空车", bool(ok),
                      f"判定={ok} 理由={reason} 内点={dg.inliers} 占比={dg.ratio:.3f}\n"
                      f"（若这里失败，先看诊断：点数是否 ≥ {dev.min_save_points}）")

    # ── D. 停稳检测 ─────────────────────────────────────────
    print("\n【D】停稳检测（偏差度量 = 质心欧氏距离，已受控实验验证）")
    rng = np.random.default_rng(5)
    base = rng.normal(0, 1, (4000, 3)).astype(np.float32)
    base[:, 1] *= 4.0  # 明确可见的车辆长轴；等向点团不提供旋转停稳证据。
    det = StabilityDetector(stability_delta=0.05, vehicle_threshold=100)
    det.detect(Frame(base))
    shift = np.array([0.3, -0.2, 0.1], np.float32)
    r = det.detect(Frame(base + shift))
    expect = float(np.linalg.norm(shift))
    check("平移 0.3/-0.2/0.1 → 偏差 = 模长",
          abs(r.deviation - expect) < 1e-5,
          f"实测 {r.deviation:.8f} / 理论 {expect:.8f}")
    det.reset()
    det.detect(Frame(base))
    r2 = det.detect(Frame(base * 1.05))
    check("缩放 ×1.05 → 偏差 = 质心偏移",
          r2.deviation > 0 and r2.deviation < 0.5, f"实测 {r2.deviation:.8f}")
    d3 = StabilityDetector(stability_delta=0.05, vehicle_threshold=100)
    d3.detect(Frame(base))
    r3 = d3.detect(Frame(base))
    from unittest.mock import patch
    clock = [1000.0, 100.0]
    with patch("time.time", side_effect=lambda: clock[0]), \
            patch("time.monotonic", side_effect=lambda: clock[1]):
        d3.reset()
        for i in range(25):
            clock[:] = [1000.0 + i * .1, 100.0 + i * .1]
            continuous = d3.detect(Frame(base, seq=i + 1, ts=clock[0]))
    check("重复采集不累计停稳时间，连续新帧才判稳",
          not r3.stable and r3.deviation < 1e-9 and continuous.stable,
          f"偏差 {r3.deviation:.2e}，连续停稳 {continuous.stationary_sec:.1f}s")

    # ── E. 服务与看板 ───────────────────────────────────────
    print("\n【E】服务与看板")
    svc = None
    httpd = None
    try:
        svc = LidarService(settings=st, save_override=False, plate_mode="simulate",
                           demo=True, only=[ch.id])
        bus_events = []
        q = svc.bus.subscribe()
        svc.start()
        check("服务启动", len(svc.workers) == 1, f"{len(svc.workers)} 个通道工作线程")

        t0 = time.time()
        got_result = None
        while time.time() - t0 < 25:
            while q:
                ev = q.popleft()
                bus_events.append(ev["kind"])
                if ev["kind"] == "result" and got_result is None:
                    got_result = ev
            if got_result:
                break
            time.sleep(0.1)
        check("仿真链路产出判定结果", got_result is not None,
              (f"车牌={got_result['plate']} 判定={got_result['reason']} "
               f"点数={got_result['n_points']} 用时={got_result['elapsed_ms']}ms")
              if got_result else "25s 内没有结果（检查是否卡在'点云数量不足'）")
        check("事件总线收到多种事件", len(set(bus_events)) >= 3,
              "、".join(sorted(set(bus_events))))

        # HTTP 接口
        from app.web.server import serve
        port = args.port or _free_port()
        httpd = serve(svc, host="127.0.0.1", port=port)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        time.sleep(0.4)
        for path, want in (("/api/health", 200), ("/", 200), ("/api/state", 200),
                           ("/api/config", 200), ("/api/history?limit=5", 200),
                           ("/api/export/ply?channel=" + ch.id, 200),
                           ("/api/nonexistent", 404)):
            code, body = _http_get(port, path)
            check(f"GET {path} → {want}", code == want,
                  f"实际 {code}，{len(body)} 字节")
        code, _ = _http_post(port, "/api/demo/loaded",
                             {"channel": ch.id, "loaded": True})
        check("POST /api/demo/loaded", code == 200, f"实际 {code}")
        code, _ = _http_post(port, "/api/demo/force", {})
        check("POST /api/demo/force", code == 200, f"实际 {code}")
    except Exception as e:                                   # noqa: BLE001
        import traceback
        check("服务与看板自检", False, f"{type(e).__name__}: {e}\n{traceback.format_exc()[-500:]}")
    finally:
        if httpd is not None:
            try:
                httpd.shutdown()
            except Exception:                                # noqa: BLE001
                pass
        if svc is not None:
            svc.stop()

    # ── F. 落盘与格式 ───────────────────────────────────────
    print("\n【F】落盘与 .xyz 格式往返")
    try:
        d = ensure_dir(root("logs", "selftest"))
        f = Frame(np.random.rand(1234, 3).astype(np.float32) * 4 - 2,
                  rng.integers(0, 256, 1234).astype(np.uint16))
        p = os.path.join(d, "roundtrip.xyz")
        save_xyz(p, f)
        g = load_xyz(p)
        size = os.path.getsize(p)
        check(".xyz 往返一致", g.n == f.n and abs(size - (4 + f.n * 14)) == 0,
              f"{f.n} 点，文件 {size} 字节（期望 {4 + f.n*14}），"
              f"ring 一致={np.array_equal(f.ring, g.ring)}")
        # 故意损坏，确认能明确报错而不是静默出错
        bad = os.path.join(d, "broken.xyz")
        with open(bad, "wb") as fp:
            fp.write(b"\x10\x00\x00\x00" + b"\x00" * 40)
        try:
            load_xyz(bad)
            check("损坏文件应报错", False, "竟然读成功了")
        except Exception as e:                               # noqa: BLE001
            check("损坏文件明确报错", True, f"{type(e).__name__}: {str(e)[:90]}")
    except Exception as e:                                   # noqa: BLE001
        check(".xyz 落盘自检", False, f"{type(e).__name__}: {e}")

    return _summary(t_all)


def _free_port() -> int:
    import socket
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _http_get(port: int, path: str):
    import http.client
    try:
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
        c.request("GET", path)
        r = c.getresponse()
        return r.status, r.read()
    except Exception as e:                                   # noqa: BLE001
        return 0, str(e).encode()


def _http_post(port: int, path: str, obj):
    import http.client
    try:
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
        body = json.dumps(obj).encode()
        c.request("POST", path, body=body,
                  headers={"Content-Type": "application/json"})
        r = c.getresponse()
        return r.status, r.read()
    except Exception as e:                                   # noqa: BLE001
        return 0, str(e).encode()


def _summary(t0: float) -> int:
    npass = sum(1 for _, ok, _ in RESULTS if ok)
    nfail = len(RESULTS) - npass
    print("\n" + "=" * 78)
    print(f"自检结果：{npass} 项通过，{nfail} 项失败，耗时 {time.time()-t0:.1f}s")
    if nfail:
        print("\n失败项：")
        for name, ok, _ in RESULTS:
            if not ok:
                print(f"  × {name}")
        print("\n排查顺序建议：")
        print("  1) 先确认 README 里的依赖（numpy）装好："
              + ("python" if os.name == "nt" else "python3")
              + " -c \"import numpy;print(numpy.__version__)\"")
        print("  2) 配置类的失败 → 检查 config/ 下三个文件是否齐全、路径是否对")
        print("  3) 几何类的失败 → 大概率是 lidar.z_down / range_mode 不对，"
              "用 cli/probe.py 对真实抓包定一遍")
        print("  4) 服务类失败 → 单独跑 cli/run.py --demo，看终端报错")
    else:
        print("√ 本地无硬件自检通过，不代表现场准确性、麒麟或真实数据库已验收。"
              "接入真雷达前请完成现场标定与安全检查，参见 docs/18-连续停稳门控与验证.md。")
    print("=" * 78)
    return 0 if nfail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
