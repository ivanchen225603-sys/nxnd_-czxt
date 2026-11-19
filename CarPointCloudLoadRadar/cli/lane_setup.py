#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/lane_setup.py — 空场标定：把"地面在哪、量纲对不对、ROI 该多大"一次定死

**必须在 4 个车道全部清空、确认没有车辆的状态下运行。**
这是整套系统里唯一能可靠测出"地面在哪"的时刻 —— 一旦车停进来，
车厢会把地面完全挡住（10 份原版历史样本里，有 4 份连一个地面点都看不到）。

用法

    # ① 只看不改（默认）。--mount-height 填你**实测**的雷达吊装高度（米）
    python cli/lane_setup.py --seconds 10 --mount-height 5.0

    # ② 把标定结果写回配置（会先备份原文件）
    python cli/lane_setup.py --seconds 10 --mount-height 5.0 --write

    # ③ 只标一条通道
    python cli/lane_setup.py --channel CH2 --seconds 10 --mount-height 5.0

它做四件事
    1. 链路体检：每路包率 / 转速 / 每帧点数 / 每圈包数
    2. 测地面：取雷达正下方半径 R0 内的点（空场时这里必定是地面），
       z 直方图找峰 → 平面拟合 → 给出地面高度、倾角、平整度
    3. **量纲自检**：把实测地面高度和你给的实际吊装高度一比 ——
       差 4 倍就是 range_mode 选错了，这一步是 2026-09-13 那次事故的解药
    4. 建议 ROI，并报告"按建议 ROI 裁剪后每帧还剩多少点"

★ 为什么这一步不能省
    `detection.ground_z` 是**写死在配置里的绝对高度**，有车判据
    （`z > ground_z + vehicle_min_height_m`）全靠它。现场几何只要对不上，
    地面就会整片跑到判据线之上 → 每帧都判「有车」→ 会话永不结束 →
    **之后真来的车全部被静默跳过**，而且一路不报错。
    见 docs/03 §2.6。
"""
from __future__ import annotations

import argparse
import io
import json
import math
import os
import shutil
import time
from typing import Dict, List, Optional

import numpy as np

import _common
_common.setup_console()

from app.pointcloud import (AngleCache, Frame, normalize_roi,   # noqa: E402
                            save_xyz)
from app.radar_packet import PacketDecoder, RANGE_MODES          # noqa: E402
from app.radar_source import LiveHub, LiveSource, build_source   # noqa: E402
from app.settings import load_calib, root, ensure_dir           # noqa: E402
from app.geometry import fit_plane_pca                           # noqa: E402

# 雷达正下方这个半径内的点，空场时必定是地面（不会是墙、货架、护栏）
GROUND_RADIUS_M = 3.0
# z 直方图的分辨率：2cm 足以分辨地面与任何车体结构
GROUND_BIN_M = 0.02


# ══════════════════════════════════════════════ 地面估计

def estimate_ground(xyz: np.ndarray, radius: float = GROUND_RADIUS_M) -> Dict:
    """从**空场**点云里量出地面。只读，不改任何东西。

    做法：
      ① 只取雷达正下方半径 radius 内的点 —— 空场时这一圈必定是地面；
      ② 对 z 做 2cm 直方图取峰值，得到粗略地面高度（比用最低点稳：
         最低点会被单个噪声点带偏）；
      ③ 取峰值 ±10cm 的点做 PCA 平面拟合 → 地面高度、倾角、平整度。
    """
    out: Dict = {"ok": False, "reason": "", "n_used": 0}
    if xyz is None or len(xyz) < 200:
        out["reason"] = "点太少，先确认雷达在出数"
        return out
    q = np.asarray(xyz, np.float64)
    rho = np.hypot(q[:, 0], q[:, 1])
    near = q[rho <= radius]
    if len(near) < 200:
        out["reason"] = (f"雷达正下方 {radius} m 内只有 {len(near)} 个点 —— "
                         f"要么量纲不对（点云整体缩了），要么雷达没对着地面")
        return out

    z = near[:, 2]
    lo, hi = float(z.min()), float(z.max())
    nbins = max(8, int(math.ceil((hi - lo) / GROUND_BIN_M)))
    hist, edges = np.histogram(z, bins=nbins, range=(lo, hi))
    k = int(hist.argmax())
    peak = float((edges[k] + edges[k + 1]) / 2.0)

    band = near[np.abs(near[:, 2] - peak) <= 0.10]
    out["n_used"] = int(len(band))
    if len(band) < 100:
        out["reason"] = "峰值附近点太少，地面可能不平或有遮挡"
        return out

    pl = fit_plane_pca(band)
    if pl is None:
        out["reason"] = "地面平面拟合失败（点可能共线）"
        return out
    # 法向与 z 轴的夹角 = 地面倾角
    tilt = math.degrees(math.acos(min(1.0, abs(pl.c))))
    out.update({
        "ok": True,
        "ground_z": round(float(np.median(band[:, 2])), 4),
        "peak_z": round(peak, 4),
        "tilt_deg": round(tilt, 3),
        "rms_m": round(float(pl.rms), 4),
        "n_near": int(len(near)),
        "spread_m": round(float(band[:, 2].max() - band[:, 2].min()), 4),
    })
    return out


def check_scale(ground_z: float, mount_height: float, proc,
                ref_depth: float = 0.0) -> Dict:
    """判断距离换算模式对不对：实测地面深度 vs 一个**已知的参照深度**。

    地面在雷达正下方，所以正确解码时 |ground_z| 就等于雷达的吊装高度。
    差一个固定倍率 => range_mode 选错了（四种模式之间正好差常数倍/常数偏移）。

    参照深度从哪来（按优先级）：
      1. `--mount-height`：现场拿卷尺量的雷达吊装高度。最直接。
      2. **原版现场口径**（`config.detection.ground_z` 的绝对值）：
         这个值是原厂系统在**同一个现场**用的，10 份历史样本与它自洽
         （`cli/verify.py` 10/10 就建立在它之上）。
         所以**量不到雷达高度也能校验** —— 前提是雷达还装在原来的位置。
         雷达要是重新装过、换了高度，这条参照就失效，那才必须用第 1 条。

    两条都拿不到时返回 checked=False —— 宁可说"没校验"，也不装作能判。
    """
    out: Dict = {"checked": False}
    if not ground_z:
        return out
    cross = ""
    if mount_height and mount_height > 0:
        ref, ref_name = float(mount_height), "你实测的吊装高度"
        # ★ 两个**互相独立**的参照都在的话，先让它们自己对一下。
        #   一致 => 雷达还在原来的高度上，历史样本这条绝对参照仍然有效；
        #   不一致 => 雷达被重新装过，那 config 里的 ground_z 和 10 份历史样本
        #             就都不能再当绝对参照用了（probe.py 的置信度也会跟着掉）。
        if ref_depth and ref_depth > 0:
            dv = abs(ref - ref_depth)
            if dv / max(ref, ref_depth) <= 0.10:
                cross = (f"（与原版现场口径 {ref_depth:.2f} m 相差 {dv:.3f} m，"
                         f"互相印证 => 雷达仍在原高度，历史样本可当绝对参照）")
            else:
                cross = (f"（▲ 与原版现场口径 {ref_depth:.2f} m 相差 {dv:.2f} m —— "
                         f"雷达多半被重新装过。此时 config 的 ground_z 与 10 份历史样本"
                         f"都不能再当绝对参照，probe.py 的置信度也会偏低，以你的实测为准）")
    elif ref_depth and ref_depth > 0:
        ref, ref_name = float(ref_depth), "原版现场口径(config 的 ground_z)"
    else:
        return out

    measured = abs(float(ground_z))
    ratio = ref / measured if measured > 1e-6 else float("inf")
    out.update({"checked": True, "measured_depth_m": round(measured, 3),
                "reference_depth_m": round(ref, 3), "reference": ref_name,
                "mount_height_m": mount_height, "ratio": round(ratio, 3)})
    if 0.9 <= ratio <= 1.1:
        out["verdict"] = "ok"
        out["message"] = (f"实测地面在雷达下方 {measured:.2f} m，与{ref_name} "
                          f"{ref:.2f} m 相符 => 距离换算量纲正确" + cross)
        out["cross_check"] = cross
        return out
    out["verdict"] = "bad"
    # scale 模式 = raw*distance_scale/1000，raw_offset = (raw-offset_r)/1000
    # 两者相差约 distance_scale 倍，这正是现场最容易配反的一对
    sc = float(getattr(proc, "distance_scale", 4.0) or 4.0)
    hint = ""
    if abs(ratio - sc) / sc < 0.15:
        hint = (f"倍率 ~ distance_scale({sc:g})，典型症状是 range_mode 写成了 "
                f"raw_offset 而应为 scale 系列")
    elif abs(ratio - 1.0 / sc) * sc < 0.15:
        hint = (f"倍率 ~ 1/distance_scale，典型症状是 range_mode 写成了 scale "
                f"而应为 raw_offset")
    out["message"] = (
        f"★ 实测地面只在雷达下方 {measured:.2f} m，而{ref_name}是 "
        f"{ref:.2f} m —— 差 {ratio:.2f} 倍。"
        + (f"（{hint}）" if hint else "")
        + f" 点云量纲不对，**此时任何判定结果都不可信**。"
          f" 用 cli/capture.py + cli/probe.py 定 range_mode（可选值："
        + "/".join(sorted(RANGE_MODES)) + "）"
        + ("。雷达若确实重新装过、高度变了，请用 --mount-height 给出实测高度再判"
           if not mount_height else ""))
    return out


def suggest_roi(ground_z: float, lane_width: float, lane_span,
                cur_roi) -> Dict:
    """给出建议 ROI，并说明每条边是怎么来的。"""
    half = max(0.5, lane_width / 2.0)
    y0, y1 = lane_span
    # z 上界：保持"雷达下方 0.75m"——再往上就会打到雷达自身/吊架/顶棚
    z_hi = -0.75
    # z 下界：地面再往下留 1m 余量（地面不平、雷达俯仰有偏差时不至于把地面切掉）
    z_lo = round(ground_z - 1.0, 2)
    return {
        "x": [round(-half, 2), round(half, 2)],
        "y": [float(y0), float(y1)],
        "z": [z_lo, z_hi],
        "why": {
            "x": f"车道宽 {lane_width:g} m 取一半，雷达装在车道中线正上方",
            "y": f"沿车道的观察窗口，沿用原厂值；要改得先确定车停在哪一段",
            "z": f"下界 = 实测地面 {ground_z:.2f} - 1.0 m 余量；"
                 f"上界 -0.75 m 是雷达自身/吊架的安全距离（原厂同值）",
        },
        "current": cur_roi,
    }


# ══════════════════════════════════════════════ 采集

# ── 朝向：ROI 的长边有没有压在行车方向上 ───────────────────────
ORIENT_TOL = 200           # 「ROI 里几乎没有结构」的宽容度（点）


def check_orientation(xyz: np.ndarray, calib, span_deg: float = 180.0,
                      step: float = 1.0) -> Dict:
    """扫 yaw_offset_deg，找「ROI 内几乎只有地面」的那一段。

    ★ 为什么空场就能定朝向

    ROI 是 4 × 19 m 的长条，长边必须压在**行车方向**上。而车道是给车走的，
    空场时本来就该是空的 —— 所以**朝向正确的 ROI 里应该几乎只有地面**。
    朝向错了，长边就会横切车道、压到两侧的墙上，墙高于地面，
    于是被"有车"判据算成车：空车道恒判有车 → 地面不动 → 判已停稳 →
    会话永不结束 → 真车全漏（2026-09-13 事故的另一种成因）。

    ★ 为什么必须看「平台宽度」而不是只取最小值

    合格角度应当是**连续的一段**（本现场干净通道有 27°~31° 宽）。
    平台只有几度宽 ≠ 雷达装歪了 —— 装歪会表现为**平台一样宽、只是中心不同**；
    平台窄说明**车道里或紧邻处有障碍物**，只有很窄的一段角度能让 ROI 擦过去。
    这时候拿最小值去定 yaw，等于拿障碍物拟合 ROI，本末倒置。

    ★ 为什么 cli/verify.py 拦不住这个

    10 份历史样本是原版**已经解码完成**的 .xyz，重跑判定链根本不经过角度换算。
    朝向错成什么样，回归都照样 10/10。这一项只能在现场用真实点云定。

    返回 {"base": 当前yaw, "curve": [(yaw, 框内点, 高于地面的点)],
          "plateau": (lo, hi) 或 None, "center": 平台中心, "current": 当前值下的点数}
    """
    roi = normalize_roi(calib.roi)
    if roi is None:
        return {"ok": False, "why": "配置里没有 ROI，无从判断朝向"}
    lo, hi = roi
    det = calib.detection
    cut = float(det.ground_z) + float(det.vehicle_min_height_m)
    base = float(calib.processing.yaw_offset_deg)

    x0, y0, z = (xyz[:, 0].astype(np.float64), xyz[:, 1].astype(np.float64),
                 xyz[:, 2].astype(np.float64))
    keep = (z >= lo[2]) & (z <= hi[2])          # z 不随 yaw 变，先筛掉
    x0, y0, z = x0[keep], y0[keep], z[keep]

    curve = []
    n = int(round(span_deg / step))
    for i in range(-n // 2, n // 2 + 1):
        dd = base + i * step
        r = math.radians(dd - base)
        c, sn = math.cos(r), math.sin(r)
        x = x0 * c + y0 * sn
        y = -x0 * sn + y0 * c
        m = ((x >= lo[0]) & (x <= hi[0]) & (y >= lo[1]) & (y <= hi[1]))
        curve.append((dd, int(m.sum()), int(np.count_nonzero(z[m] > cut))))

    cur = [c for c in curve if abs(c[0] - base) < step / 2]
    cur_in, cur_hi = (cur[0][1], cur[0][2]) if cur else (0, 0)

    # 合格：高于地面的点够少，且框内总点数没塌（框跑到没回波的地方不算数）
    floor_n = 0.5 * max(cur_in, 1)
    good = [c[0] for c in curve if c[2] <= ORIENT_TOL and c[1] > floor_n]
    plateau = None
    if good:
        runs, run = [], [good[0]]
        for d in good[1:]:
            if abs(d - run[-1] - step) < step / 2:
                run.append(d)
            else:
                runs.append(run); run = [d]
        runs.append(run)
        best = max(runs, key=len)
        plateau = (best[0], best[-1])
    return {"ok": True, "base": base, "curve": curve, "plateau": plateau,
            "center": (plateau[0] + plateau[1]) / 2 if plateau else None,
            "width": (plateau[1] - plateau[0]) if plateau else 0.0,
            "current_in": cur_in, "current_above": cur_hi, "cut": cut}


def report_orientation(rows: List[Dict]) -> None:
    """把朝向结论打出来 —— 只报不改，yaw 要人来拍板。"""
    print()
    print("=" * 78)
    print("朝向核对（ROI 的长边有没有压在行车方向上）")
    print("=" * 78)
    print("%-6s %10s %12s %16s %9s" %
          ("通道", "框内点", "高于地面", "合格平台", "平台宽"))
    print("-" * 78)
    bad = []
    for r in rows:
        o = r.get("orient") or {}
        if not o.get("ok"):
            print("%-6s   %s" % (r["channel"], o.get("why", "没数据")))
            continue
        pl = o["plateau"]
        print("%-6s %10d %12d %16s %8.0f°" %
              (r["channel"], o["current_in"], o["current_above"],
               ("%.0f…%.0f°" % pl) if pl else "（找不到）", o["width"]))
        if o["current_above"] > ORIENT_TOL:
            bad.append(r["channel"])
    print("-" * 78)
    wide = [r for r in rows
            if (r.get("orient") or {}).get("plateau") and r["orient"]["width"] >= 10]
    narrow = [r for r in rows
              if (r.get("orient") or {}).get("plateau") and r["orient"]["width"] < 10]
    if bad:
        print("× 这些通道当前朝向下、空车道里有大量点被算成「车」：" + "、".join(bad))
        print("  空车道恒判有车 → 地面不动 → 判已停稳 → 会话永不结束 → 真车全漏。")
    if wide:
        cs = [r["orient"]["center"] for r in wide]
        print("√ 平台够宽的通道（朝向可信）：" +
              "、".join("%s 中心 %.0f°" % (r["channel"], r["orient"]["center"])
                        for r in wide))
        print("  建议 processing.yaw_offset_deg = %.0f" %
              (sum(cs) / len(cs)))
    if narrow:
        print("▲ 这些通道的平台很窄（< 10°）：" +
              "、".join("%s（%.0f°）" % (r["channel"], r["orient"]["width"])
                        for r in narrow))
        print("  装歪会表现为『平台一样宽、只是中心不同』；平台窄是**车道里有障碍物**，")
        print("  只有很窄一段角度能让 ROI 擦过去。先去现场看那是什么，")
        print("  **不要**拿它的最优角去定 yaw —— 那是拿障碍物拟合 ROI。")
    if wide:
        print("▲ 空场只定得出车道的**轴向**，定不出**朝哪头** —— 走廊转 180°")
        print("  长得一模一样。上面建议的角度 ±180° 同样合格，差别只来自 ROI 的")
        print("  y 窗口不对称。要定死朝向：说明车从哪头进，或停一辆车进来看车头。")
    print("  改 yaw 之后 cli/verify.py 仍会是 10/10 —— 它证明不了朝向")
    print("  （历史样本是已解码的 .xyz，不过角度换算）。只能靠现场点云定。")


def grab(ch, calib, angle, hub, seconds: float) -> Dict:
    """从一条 live 通道连续取帧（**不裁 ROI**），返回汇总。"""
    lid = ch.lidar
    dec = PacketDecoder(calib.packet, calib.processing, angle,
                        range_mode=lid.range_mode, z_down=lid.z_down,
                        max_range_m=lid.max_range_m,
                        min_range_m=float(calib.processing.min_distance_mm) / 1000.0)
    src = LiveSource(dec, udp_port=lid.udp_port,
                     roi=None,                     # ★ 关键：要全量点云才能看见地面
                     frame_ms=lid.frame_ms,
                     frame_mode=getattr(lid, "frame_mode", "rotation"),
                     name=f"{ch.id} setup", hub=hub,
                     source_ip=(lid.source_ip or lid.ip))
    frames: List[Frame] = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        f = src.read()
        if f is not None and f.n:
            frames.append(f)
        else:
            time.sleep(0.002)
    info = src.info()
    return {"frames": frames, "info": info}


def dump_raw(rows: List[Dict], ctx, out_dir: str) -> List[str]:
    """把每路**没裁过 ROI** 的整帧点云存下来。

    ★ 为什么需要这个

    正常落盘的 `.xyz` 是**先裁 ROI、再做航向旋转**之后的结果
    （radar_source._emit → detect.correct_heading → service._save）。
    于是 ROI 之外的东西**根本不在文件里** —— 想验证"车厢有没有被 ROI 切掉"，
    只能看"车厢边缘离边界多远"来**间接推断**，而这个推断在 y 方向并不可靠：
    车厢远端因掠射角会**看起来变窄**（2026-09-14 实测：主体宽 2.67 m，
    到 y=5.5 只剩 2.25 m），所以"宽度掉下去"既可能是车厢结束、
    也可能只是看不全。

    有了未裁的整帧就能**直接看到 ROI 外面**：车厢边缘之外是空的，
    就证明没被切；紧贴着还有点，就是切了。
    2026-09-14 只抓到 1 份（CH4），另外 26 份只能推断 —— 这个开关就是补这个缺口。

    车**停稳时**跑一次即可。文件与判定用的 .xyz 同格式（`cli/inspect_cloud.py`
    直接能读），旁边再写一份 .json 记下当时的 ROI / yaw / ground_z，
    这样离线复算时不用猜配置。
    """
    import json as _json
    ensure_dir(out_dir)
    written = []
    for r in rows:
        frames = r.get("_frames") or []
        if not frames:
            continue
        # 取点最多的那一帧 —— 整圈最完整，边界判断最可信
        f = max(frames, key=lambda x: x.n)
        stem = os.path.join(out_dir, "%s_%s" % (r["channel"],
                                                time.strftime("%Y%m%d_%H%M%S")))
        save_xyz(stem + ".xyz", f)
        roi = _cur_roi(ctx.calib)
        _json.dump({
            "channel": r["channel"], "points": int(f.n),
            "note": "未裁 ROI 的整帧 —— 用来直接验证车厢有没有被 ROI 切掉",
            "roi": roi,
            "yaw_offset_deg": float(ctx.calib.processing.yaw_offset_deg),
            "channel_yaw_offset_deg": getattr(
                [c for c in ctx.channels if c.id == r["channel"]][0].lidar,
                "yaw_offset_deg", None),
            "ground_z": float(ctx.calib.detection.ground_z),
            "range_mode": r.get("range_mode"),
            "captured": time.strftime("%Y-%m-%d %H:%M:%S"),
        }, io.open(stem + ".json", "w", encoding="utf-8"),
            ensure_ascii=False, indent=1)
        written.append(stem + ".xyz")
        print("  √ %s  %d 点 -> %s.xyz" % (r["channel"], f.n, stem))
    if not written:
        print("  × 一帧都没抓到，没东西可存")
    else:
        print("  ★ 这些是**未裁 ROI** 的整帧。验证车厢有没有被切：")
        print("     python3 cli/inspect_cloud.py <文件>")
        print("     或直接比对 .json 里记的 ROI 边界与点云的实际范围。")
    return written


def analyse(ch, calib, got: Dict, args) -> Dict:
    frames: List[Frame] = got["frames"]
    info = got["info"]
    r: Dict = {
        "channel": ch.id, "name": ch.name,
        "source_ip": ch.lidar.source_ip or ch.lidar.ip,
        "frames": len(frames),
        "frames_used": 0,          # analyse 里填
        "packet_rate": info.get("packet_rate", 0),
        "rpm": info.get("rpm", 0),
        "rev_packets": info.get("rev_packets", 0),
        "frames_by_timeout": info.get("frames_by_timeout", 0),
        "range_mode": ch.lidar.range_mode,
    }
    if not frames:
        r["ok"] = False
        r["reason"] = "这条通道一帧都没收到 —— 查网线 / 雷达源 IP / lidar.source_ip"
        return r

    # ★ 丢掉头两帧：刚开始收包时第一帧往往只攒到半圈，算进去会把"帧间波动"
    #   顶到 5%+，误报成丢包。现场排查最忌讳这种自己制造的假信号。
    usable = frames[2:] if len(frames) > 4 else frames
    ns = [f.n for f in usable]
    r["frames_used"] = len(usable)
    r["points_per_frame"] = [int(min(ns)), int(round(float(np.mean(ns)))), int(max(ns))]
    # 帧间点数波动：整圈切帧且不丢包时应当很小
    r["points_jitter"] = round((max(ns) - min(ns)) / max(1, max(ns)), 4)

    # 用中间那一帧量地面（前几帧可能还在攒包）
    mid = usable[len(usable) // 2]
    g = estimate_ground(mid.xyz, args.ground_radius)
    r["ground"] = g
    if not g["ok"]:
        r["ok"] = False
        r["reason"] = g["reason"]
        return r

    # 没有卷尺也能校验：拿配置里的 ground_z（原版现场口径）当参照
    ref_depth = abs(float(getattr(calib.detection, "ground_z", 0.0) or 0.0))
    r["scale"] = check_scale(g["ground_z"], args.mount_height,
                             calib.processing, ref_depth)
    r["roi"] = suggest_roi(g["ground_z"], args.lane_width,
                           args.lane_span, _cur_roi(calib))

    # 按建议 ROI 裁一刀，看还剩多少点 —— 这是"ROI 定得合不合适"的直接反馈
    from app.pointcloud import apply_roi
    roi = ((r["roi"]["x"][0], r["roi"]["y"][0], r["roi"]["z"][0]),
           (r["roi"]["x"][1], r["roi"]["y"][1], r["roi"]["z"][1]))
    kept = [apply_roi(f, roi).n for f in usable]
    r["roi_points"] = [int(min(kept)), int(round(float(np.mean(kept)))), int(max(kept))]
    r["roi_keep"] = round(float(np.mean(kept)) / max(1.0, float(np.mean(ns))), 3)
    r["ok"] = True
    return r


def _cur_roi(calib) -> Optional[List]:
    if not calib.roi:
        return None
    lo, hi = calib.roi
    return {"x": [lo[0], hi[0]], "y": [lo[1], hi[1]], "z": [lo[2], hi[2]]}


# ══════════════════════════════════════════════ 输出

def report(rows: List[Dict], args) -> None:
    print("\n" + "=" * 78)
    print("空场标定结果")
    print("=" * 78)
    for r in rows:
        print(f"\n【{r['channel']}】{r['name']}　源 IP {r['source_ip']}")
        if not r.get("ok"):
            print(f"  × {r.get('reason')}")
            continue
        pf = r["points_per_frame"]
        print(f"  链路   : {r['packet_rate']:.0f} 包/s　{r['rpm']:.0f} rpm　"
              f"{r['rev_packets']} 包/圈　兜底切帧 {r['frames_by_timeout']} 帧")
        print(f"  出帧   : {r['frames']} 帧　每帧 {pf[0]}~{pf[2]} 点（均值 {pf[1]}）　"
              f"帧间波动 {r['points_jitter']:.1%}")
        g = r["ground"]
        print(f"  地面   : z = {g['ground_z']:+.3f} m（雷达下方 {abs(g['ground_z']):.2f} m）"
              f"　倾角 {g['tilt_deg']:.2f}°　平整度 RMS {g['rms_m']*1000:.0f} mm")
        sc = r.get("scale", {})
        if sc.get("checked"):
            mark = "√" if sc["verdict"] == "ok" else "×"
            print(f"  量纲   : {mark} {sc['message']}")
        else:
            print("  量纲   : （未校验 —— 加 --mount-height 填实测吊装高度才能判）")
        roi = r["roi"]
        cur = roi.get("current")
        print(f"  建议ROI: x{roi['x']}　y{roi['y']}　z{roi['z']}")
        if cur:
            same = (cur["x"] == roi["x"] and cur["y"] == roi["y"] and cur["z"] == roi["z"])
            print(f"  当前ROI: x{cur['x']}　y{cur['y']}　z{cur['z']}"
                  + ("　（一致）" if same else "　← 不一致"))
        rp = r["roi_points"]
        print(f"  裁剪后 : 每帧 {rp[0]}~{rp[2]} 点（均值 {rp[1]}）　"
              f"ROI 保留 {r['roi_keep']:.0%}")
        _advise(r)
    print("\n" + "=" * 78)


def _advise(r: Dict) -> None:
    """把"这个数正常吗"直接写出来，别让现场自己去查表。"""
    tips = []
    sc = r.get("scale", {})
    if sc.get("checked") and sc.get("verdict") == "bad":
        tips.append("★ 量纲不对，先解决它 —— 后面所有判定都建立在它上面")
    if r.get("frames_by_timeout", 0) > 0:
        tips.append("兜底切帧 > 0：没等到方位角回绕，查 packet.angle1_offset / angle_scale")
    if r.get("points_jitter", 0) > 0.05:
        tips.append(f"帧间点数波动 {r['points_jitter']:.0%} 偏大：多半在丢包，"
                    f"查 hub 包率与队列丢弃")
    rp = r.get("roi_points", [0, 0, 0])
    if rp[1] and rp[1] < 20000:
        tips.append(f"裁剪后均值 {rp[1]} 点 < min_save_points(20000)：车来了也不会落盘，"
                    f"ROI 可能切太狠")
    g = r.get("ground", {})
    if g.get("ok") and g.get("tilt_deg", 0) > 3.0:
        tips.append(f"地面倾角 {g['tilt_deg']:.1f}°：雷达没装平，或地面确实有坡度 —— "
                    f"底面拟合的 |法向z|≥0.99 门槛会受影响")
    if r.get("roi_keep", 0) > 0.9:
        tips.append("ROI 保留率 > 90%：整个场景几乎都落进了检测框，量纲可疑")
    for t in tips:
        print(f"           ▲ {t}")


def write_back(rows: List[Dict], args, path: str = "") -> int:
    """把标定结果写回 config.json（先备份）。

    path 留空即写项目里那份共用配置；测试传临时文件进来 ——
    这样用例不必去 mock `builtins.open`（那会把**整个进程**的 open 换掉，
    一旦哪条闸没拦住就真的把现场配置覆盖了）。

    ★ 这里写的是**共用**的 config/config.json —— 四条通道读同一份。
      所以只要有一条没量到，就等于拿"量到的那条"替没量到的做主张。
      而 ground_z 写错的后果正是 2026-09-13 那次事故（地面跑到判据线之上，
      系统永远判有车、真车全漏）。宁可多问一句，也不能默默写下去。
    """
    ok = [r for r in rows if r.get("ok")]
    if not ok:
        print("× 没有一条通道标定成功，不写任何东西")
        return 2
    missing = [r["channel"] for r in rows if not r.get("ok")]
    if missing and not args.assume_same_mount:
        print(f"× 这些通道没量到：{', '.join(missing)}")
        print( "  但 config/config.json 是**四条通道共用**的 —— 写回去等于拿量到的那条")
        print( "  替没量到的做主张。它们的雷达要是装得不一样高，就会被写错，")
        print( "  而 ground_z 写错的后果就是 9-13 那次：永远判有车、真车全漏。")
        print( "  确认所有雷达**吊装高度一致**的话，加上 --assume-same-mount 再跑。")
        return 2
    if missing:
        print(f"▲ 已按 --assume-same-mount 继续：{', '.join(missing)} 没量到，"
              f"将沿用已量到通道的地面高度")
    bad = [r for r in ok if r.get("scale", {}).get("verdict") == "bad"]
    if bad:
        print("× 量纲还没定对（" + "、".join(r["channel"] for r in bad) +
              "），此时写回去等于把错的固化下来。先定 range_mode。")
        return 2
    gz = [r["ground"]["ground_z"] for r in ok]
    if max(gz) - min(gz) > 0.20:
        print(f"× 各通道实测地面高度相差 {max(gz)-min(gz):.2f} m（{min(gz):.2f}…{max(gz):.2f}）。")
        print("  它们共用同一份 config.json，写回去必然有通道被写错。")
        print("  要么把雷达装到同一高度，要么给每条通道单独一份 config"
              "（channels.json 的 config_path 支持逐通道指定）。")
        return 2

    path = path or root("config", "config.json")
    bak = path + ".bak-" + time.strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, bak)
    with open(path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    new_gz = round(float(np.median(gz)), 3)
    old_gz = cfg.get("detection", {}).get("ground_z")
    cfg.setdefault("detection", {})["ground_z"] = new_gz
    roi = ok[0]["roi"]
    old_roi = cfg.get("roi")
    cfg["roi"] = {"x": roi["x"], "y": roi["y"], "z": roi["z"]}
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print(f"√ 已写回 {path}（备份 {os.path.basename(bak)}）")
    print(f"   detection.ground_z: {old_gz} → {new_gz}")
    print(f"   roi               : {old_roi} → {cfg['roi']}")
    print("   ★ 改完必须跑一次回归：python cli/verify.py（必须仍然 10/10）")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="lane_setup.py",
        description="空场标定：测地面 / 校量纲 / 定 ROI（必须在车道全空时跑）",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    _common.add_common_args(ap)
    ap.add_argument("--seconds", type=float, default=10.0, help="每通道采集时长（秒）")
    ap.add_argument("--mount-height", type=float, default=0.0, metavar="M",
                    help="雷达吊装高度（米，雷达到地面）。**可以不填** —— 不填时"
                         "用配置里的 ground_z（原版现场口径）当参照。"
                         "只有雷达重新装过、高度变了，才必须实测后填这里")
    ap.add_argument("--lane-width", type=float, default=4.0, help="车道宽度（米），定 ROI 的 x")
    ap.add_argument("--lane-span", type=float, nargs=2, default=(-11.0, 8.0),
                    metavar=("Y0", "Y1"), help="沿车道的观察窗口（米），定 ROI 的 y")
    ap.add_argument("--ground-radius", type=float, default=GROUND_RADIUS_M,
                    help="测地面时只取雷达正下方这个半径内的点（米）")
    ap.add_argument("--check-orientation", action="store_true",
                    help="核对 ROI 的长边有没有压在行车方向上（扫 yaw_offset_deg）。"
                         "这一项 cli/verify.py 证明不了 —— 历史样本是已解码的 .xyz，"
                         "不过角度换算。只报不改，yaw 要人来拍板")
    ap.add_argument("--dump-raw", nargs="?", const="AUTO", default="", metavar="DIR",
                    help="把每路**没裁过 ROI** 的整帧点云存下来（车停稳时跑）。"
                         "落盘的 .xyz 是裁过 ROI 的，看不到框外面，"
                         "没法直接验证车厢有没有被切 —— 这个开关就是补这个缺口。"
                         "不给路径则存到 data/diag/<日期>/raw/")
    ap.add_argument("--json", default="", metavar="PATH", help="把完整结果写成 JSON")
    ap.add_argument("--write", action="store_true",
                    help="把 ground_z 与 ROI 写回 config/config.json（先备份）")
    ap.add_argument("--assume-same-mount", action="store_true",
                    help="只量到部分通道时仍然写回 —— 表示你确认**所有雷达吊装高度一致**。"
                         "config.json 是四条通道共用的，不确认就不给写")
    args = ap.parse_args(argv)

    ctx = _common.load_ctx(args)
    live = [c for c in ctx.channels if (c.mode or "").lower() in ("live", "udp", "real")]
    if not live:
        raise SystemExit("× 没有 live 通道。空场标定必须接真雷达（mode=live）。")

    print("=" * 78)
    print("空场标定　—　请再次确认：4 个车道现在都**没有车**")
    print("=" * 78)
    print(f"通道       : {', '.join(c.id for c in live)}")
    print(f"采集       : 每路 {args.seconds:g} 秒")
    ref_depth = abs(float(getattr(ctx.calib.detection, "ground_z", 0.0) or 0.0))
    print(f"量纲参照   : " + (f"你实测的吊装高度 {args.mount_height:g} m"
                              if args.mount_height else
                              f"配置里的 ground_z = {ref_depth:g} m（原版现场口径）"
                              f" —— 雷达若重新装过请用 --mount-height 给实测值"))
    print("-" * 78)

    calib = ctx.calib
    angle = ctx.angle
    hub = LiveHub(bind_ip=live[0].lidar.bind_ip or calib.network.bind_ip or "0.0.0.0",
                  udp_port=live[0].lidar.udp_port, strict=True)
    for c in live:
        hub.ensure(c.lidar.source_ip or c.lidar.ip, c.id)
    hub.start()
    rows = []
    try:
        for c in live:
            print(f"  采集 {c.id} …", flush=True)
            got = grab(c, calib, angle, hub, args.seconds)
            row = analyse(c, calib, got, args)
            if args.check_orientation and got["frames"]:
                # 取中间那一帧：头几帧可能还没同步到整圈
                f = got["frames"][len(got["frames"]) // 2]
                row["orient"] = check_orientation(f.xyz, calib)
            if args.dump_raw:
                # 只在要存盘时才把帧留住 —— 一帧十几万点，四路留一整轮很占内存
                row["_frames"] = got["frames"]
            rows.append(row)
    finally:
        hub.close()

    report(rows, args)
    if args.check_orientation:
        report_orientation(rows)
    if args.dump_raw:
        out = (args.dump_raw if args.dump_raw != "AUTO" else
               root("data", "diag", time.strftime("%Y%m%d"), "raw"))
        print()
        print("=" * 78)
        print("存未裁 ROI 的整帧（用来直接验证车厢有没有被 ROI 切掉）")
        print("=" * 78)
        dump_raw(rows, ctx, out)
    if args.json:
        with open(args.json, "w", encoding="utf-8", newline="\n") as f:
            json.dump({"time": time.strftime("%Y-%m-%d %H:%M:%S"),
                       "mount_height_m": args.mount_height,
                       "channels": rows}, f, ensure_ascii=False, indent=2)
        print(f"完整结果已写入 {args.json}")
    if args.write:
        return write_back(rows, args)
    print("（只看不改。确认无误后加 --write 写回配置）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
