#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
radar_probe.py — 报文探测与现场校准（radar_packet 的配套工具核心）

解决的问题：jt128.dll 的解码细节无法逐行还原，只能靠「用真实抓包反推」。
本模块把这件事自动化 —— 现场录 10 秒报文，跑一遍，直接得到结论。

它做四件事
    ① 报文体检：包长分布、方位角跨度与递增步长、丢包率
    ② 字节字段识别：统计点位区域每一字节的分布，指出哪个是强度、哪个是标志
    ③ 距离换算选型：把 4 种 range_mode 都跑一遍，与历史 .xyz 的分位数比对打分
    ④ 结论输出：给出建议写回 config/channels.json 的具体值

设计原则：**不猜**。每一处结论都附上支撑数字，并且明确标注置信度。
"""
from __future__ import annotations

import os
import json
import struct
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .pointcloud import load_xyz, apply_roi
from .radar_packet import PacketDecoder, PacketError, RANGE_MODES, POINT_RAW_DTYPE


# ═════════════════════════════════════════════ 抓包文件读写

def load_capture(path: str, packet_size: int = 1100) -> List[bytes]:
    """
    读抓包文件。自动识别两种格式：

        A) 带长度前缀（cli/capture.py 与 LiveSource 的输出现场格式）
           重复 [uint16 LE length][packet bytes]
        B) 裸拼接：直接是 N × packet_size 字节

    识别方式：先按 A 试解，若解出的包全部长度一致且刚好覆盖整个文件，
    则认为是 A；否则按 B 切分。
    """
    with open(path, "rb") as f:
        raw = f.read()
    if not raw:
        return []

    # 试 A
    pkts: List[bytes] = []
    off, ok = 0, True
    while off + 2 <= len(raw):
        n = struct.unpack_from("<H", raw, off)[0]
        if n <= 0 or off + 2 + n > len(raw):
            ok = False
            break
        pkts.append(raw[off + 2: off + 2 + n])
        off += 2 + n
    if ok and off == len(raw) and pkts and len({len(p) for p in pkts}) == 1:
        return pkts

    # 回退 B
    if len(raw) % packet_size == 0:
        return [raw[i:i + packet_size] for i in range(0, len(raw), packet_size)]
    # 最后再试：文件本身就是单包
    return [raw] if len(raw) >= packet_size else [raw]


# ═════════════════════════════════════════════ ① 报文体检

@dataclass
class LayoutReport:
    count: int = 0
    sizes: Dict[int, int] = field(default_factory=dict)
    az1_raw: Tuple[int, int] = (0, 0)
    az2_raw: Tuple[int, int] = (0, 0)
    az_step_min: float = 0.0
    az_step_median: float = 0.0
    az_step_max: float = 0.0
    az_scale_guess: float = 0.0
    az_scale_evidence: str = ""
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {"count": self.count, "sizes": {str(k): v for k, v in self.sizes.items()},
                "az1_raw": list(self.az1_raw), "az2_raw": list(self.az2_raw),
                "az_step": [self.az_step_min, self.az_step_median, self.az_step_max],
                "az_scale_guess": self.az_scale_guess,
                "az_scale_evidence": self.az_scale_evidence, "notes": self.notes}


def scan_layout(packets: Sequence[bytes], packet_cfg) -> LayoutReport:
    """报文体检 + 方位角标度反推。"""
    r = LayoutReport(count=len(packets))
    if not packets:
        r.notes.append("没有报文 —— 检查抓包网卡 / 端口是否选对")
        return r
    for p in packets:
        r.sizes[len(p)] = r.sizes.get(len(p), 0) + 1
    if len(r.sizes) > 1:
        r.notes.append(f"包长不唯一（{r.sizes}）—— 可能混入了别的流量，"
                       f"或 packet.size({packet_cfg.size}) 配错")

    a1, a2 = [], []
    for p in packets:
        if len(p) < max(packet_cfg.angle2_offset + 2, packet_cfg.angle1_offset + 2):
            continue
        a1.append(struct.unpack_from("<H", p, packet_cfg.angle1_offset)[0])
        a2.append(struct.unpack_from("<H", p, packet_cfg.angle2_offset)[0])
    if not a1:
        r.notes.append("报文太短，取不到方位角字段")
        return r
    A1, A2 = np.asarray(a1, np.int64), np.asarray(a2, np.int64)
    r.az1_raw = (int(A1.min()), int(A1.max()))
    r.az2_raw = (int(A2.min()), int(A2.max()))

    # 用 block1 的方位角序列推步长（同一个 block 的相邻包就是方位角步进）
    d = np.diff(A1)
    d = d[d != 0]
    if len(d):
        # 处理回绕（0xFFFF → 0）
        dd = np.where(d < 0, d + 65536, d).astype(np.float64)
        r.az_step_min = float(dd.min())
        r.az_step_median = float(np.median(dd))
        r.az_step_max = float(dd.max())

    # 标度猜测：一个完整扫描周期应该覆盖 360°，故 周期内最大值 ≈ 360/scale
    span = float(max(A1.max() - A1.min(), A2.max() - A2.min()))
    if span > 0:
        for scale, name in ((0.01, "0.01°/LSB（=9000/LSB 一圈，常见）"),
                            (1.0, "1°/LSB"),
                            (360.0 / 65536.0, "360/65536°/LSB（=0.00549°）")):
            if abs(span * scale - 360.0) < 40.0 or abs(span * scale) < 90.0:
                r.az_scale_guess = scale
                r.az_scale_evidence = (f"抓包内方位角原始跨度 {int(span)}，"
                                       f"按 {name} 换算为 {span*scale:.1f}°")
                break
        if r.az_scale_guess == 0.0:
            r.az_scale_guess = 0.01
            r.az_scale_evidence = (f"无法自动判定（原始跨度 {int(span)}），"
                                   f"沿用 config 的 angle_scale=0.01")
    if r.az_step_median and r.az_scale_guess:
        deg = r.az_step_median * r.az_scale_guess
        if deg > 5.0:
            r.notes.append(f"方位角步长中位数 {deg:.2f}°/包 —— 偏大，"
                           f"可能丢包（正常应在 0.1~1° 量级）")
        else:
            r.notes.append(f"方位角步长中位数 {deg:.3f}°/包，"
                           f"转一圈约需 {360.0/max(deg,1e-9):.0f} 个包")
    return r


# ═════════════════════════════════════════════ ② 字节字段识别

@dataclass
class FieldReport:
    """逐字节位置的分布统计，用来指认强度字节 / 标志字节。"""
    positions: List[Dict] = field(default_factory=list)
    intensity_guess: int = -1
    flag_guess: int = -1
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {"positions": self.positions, "intensity_guess": self.intensity_guess,
                "flag_guess": self.flag_guess, "notes": self.notes}


def scan_point_bytes(packets: Sequence[bytes], packet_cfg) -> FieldReport:
    """
    统计每个点位 4 字节中「每一列」的分布特征：

        距离列（r） ：uint16，动态范围大，与列高度相关
        强度列      ：uint8，动态范围大、分布接近对数正态
        标志列      ：uint8，取值集合很小（常为常量）

    判据：在所有 u8 列里，取「唯一值个数最多」的作为强度，
          「唯一值个数最少」的作为标志。
    """
    fr = FieldReport()
    dec = PacketDecoder(packet_cfg, _NullProc(), None)
    pps = dec._pps
    off1, off2 = packet_cfg.block1_data_offset, packet_cfg.block2_data_offset

    cols = {k: [[] for _ in range(packet_cfg.point_size)]
            for k in ("b1", "b2")}
    for p in packets:
        for key, off in (("b1", off1), ("b2", off2)):
            if len(p) < off + pps * packet_cfg.point_size:
                continue
            arr = np.frombuffer(p, dtype=POINT_RAW_DTYPE, count=pps, offset=off)
            cols[key][0].append(arr["r"].astype(np.int64))
            cols[key][1].append(arr["intensity"].astype(np.int64))
            cols[key][2].append(arr["flag"].astype(np.int64))
            # 第 4 字节（若 point_size==4，POINT_RAW_DTYPE 已覆盖；此处通用兜底）
            if packet_cfg.point_size > 3:
                raw = np.frombuffer(p, dtype=np.uint8,
                                    count=pps * packet_cfg.point_size, offset=off)
                raw = raw.reshape(pps, packet_cfg.point_size)
                cols[key][3].append(raw[:, 3].astype(np.int64))

    for i in range(packet_cfg.point_size):
        v = []
        for key in ("b1", "b2"):
            for a in cols[key][i]:
                v.append(a)
        if not v:
            continue
        cat = np.concatenate(v)
        if len(cat) > 400000:
            cat = cat[:: max(1, len(cat) // 400000)]
        uniq = int(len(np.unique(cat)))
        fr.positions.append({
            "byte": i,
            "is_u16_start": i == 0,
            "min": int(cat.min()), "max": int(cat.max()),
            "mean": round(float(cat.mean()), 2),
            "std": round(float(cat.std()), 2),
            "unique": uniq,
            "top_values": [[int(x), int(c)] for x, c in
                           zip(*np.unique(cat, return_counts=True))][:5] if uniq <= 8 else [],
        })

    u8cols = [p for p in fr.positions if not p["is_u16_start"]]
    if u8cols:
        by_uniq = sorted(u8cols, key=lambda p: -p["unique"])
        fr.intensity_guess = by_uniq[0]["byte"]
        if len(by_uniq) > 1:
            fr.flag_guess = by_uniq[-1]["byte"]
        if by_uniq[0]["unique"] <= 8:
            fr.notes.append("所有 u8 列的取值都很集中 —— 可能整包都是无效回波"
                            "（对着墙/玻璃，或雷达没转起来）")
    return fr


class _NullProc:
    """给 PacketDecoder 用的空 processing 配置（只做布局自检，不做换算）。"""
    offset_r = 15.640617
    distance_scale = 4.0
    angle_scale = 0.01
    deg_to_rad = 0.017453292519943295
    yaw_offset_deg = -93.0
    min_distance_mm = 100.0


# ═════════════════════════════════════════════ ③ 距离换算选型

@dataclass
class ModeScore:
    mode: str = ""
    n_points: int = 0
    n_in_roi: int = 0
    roi_ratio: float = 0.0
    r_p02: float = 0.0
    r_p50: float = 0.0
    r_p98: float = 0.0
    pct: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)
    # ROI 内的 z 分位（裁过 ROI 再算，与历史参照同口径）
    roi_pct: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)
    # ★ 主判据：ROI 内主导面（z 的中位）离"期望地面"多远，米。越小越好。
    ground_err: float = float("inf")
    ground_z_seen: float = float("nan")
    score: float = float("inf")
    error: str = ""

    def to_dict(self) -> dict:
        return {"mode": self.mode, "n_points": self.n_points, "n_in_roi": self.n_in_roi,
                "roi_ratio": round(self.roi_ratio, 4),
                "ground_z_seen": None if self.ground_z_seen != self.ground_z_seen else round(self.ground_z_seen, 3),
                "ground_err_m": None if self.ground_err == float("inf") else round(self.ground_err, 3),
                "roi_percentiles": {k: [round(x, 2) for x in v] for k, v in self.roi_pct.items()},
                "range_m": [round(self.r_p02, 2), round(self.r_p50, 2), round(self.r_p98, 2)],
                "percentiles": {k: [round(x, 2) for x in v] for k, v in self.pct.items()},
                "score": None if self.score == float("inf") else round(self.score, 3),
                "error": self.error}


def _pcts(xyz: np.ndarray) -> Dict[str, Tuple[float, float, float]]:
    if not len(xyz):
        return {}
    q = np.percentile(xyz, [2, 50, 98], axis=0)
    return {axis: tuple(float(x) for x in q[:, i])
            for i, axis in enumerate(("x", "y", "z"))}


def mode_scores(packets: Sequence[bytes], packet_cfg, proc_cfg, angle_cache,
                roi, z_down: bool = True, expect_ground: Optional[float] = None,
                reference: Optional[Dict[str, Tuple[float, float, float]]] = None,
                max_points: int = 1_500_000) -> List[ModeScore]:
    """
    把 4 种 range_mode 各跑一遍，产出统计与打分。

    reference：历史 .xyz 的 x/y/z 分位数（由 history_percentiles() 得到）。
               有它时按「分位数距离」打分；没有时退化为「ROI 命中率」打分。
    """
    out: List[ModeScore] = []
    for mode in RANGE_MODES:
        ms = ModeScore(mode=mode)
        try:
            dec = PacketDecoder(packet_cfg, proc_cfg, angle_cache, range_mode=mode,
                                z_down=z_down, max_range_m=1e9)
        except PacketError as e:
            ms.error = str(e)
            out.append(ms)
            continue

        xs: List[np.ndarray] = []
        total = 0
        r_all: List[np.ndarray] = []
        for b in packets:
            try:
                pkt = dec.decode(b)
            except PacketError as e:
                ms.error = str(e)
                break
            r_all.append(dec.to_range_m(pkt.ranges_raw))
            xyz, _ring, _it = dec.to_cartesian(pkt)
            if len(xyz):
                xs.append(xyz)
            total += len(xyz)
            if total >= max_points:
                break
        if not xs:
            if not ms.error:
                ms.error = "该换算模式下没有任何有效点（全部被距离窗过滤）"
            out.append(ms)
            continue

        xyz = np.concatenate(xs, axis=0)
        if len(xyz) > max_points:
            xyz = xyz[:: max(1, len(xyz) // max_points)]
        ms.n_points = int(len(xyz))
        r = np.concatenate(r_all)
        ms.r_p02, ms.r_p50, ms.r_p98 = (float(np.percentile(r, 2)),
                                        float(np.percentile(r, 50)),
                                        float(np.percentile(r, 98)))
        ms.pct = _pcts(xyz)

        if roi:
            (x0, y0, z0), (x1, y1, z1) = roi
            m = np.all((xyz >= np.array([x0, y0, z0], np.float32))
                       & (xyz <= np.array([x1, y1, z1], np.float32)), axis=1)
            ms.n_in_roi = int(m.sum())
            ms.roi_ratio = ms.n_in_roi / max(1, len(xyz))
            # ★ 裁过 ROI 再算一份分位数 —— 历史参照本来就是裁过的
            #   （`history_percentiles` 里 `apply_roi(f, roi)`），
            #   拿整片场景去比它是拿两个不同的点群在比，见下面的说明。
            if ms.n_in_roi >= 100:
                ms.roi_pct = _pcts(xyz[m])
        else:
            ms.n_in_roi = len(xyz)
            ms.roi_ratio = 1.0
            ms.roi_pct = ms.pct

        # ── 打分 ────────────────────────────────────────────
        #
        # ★★ 主判据是**地面落在哪**，不是分位数像不像（2026-09-18 改）。
        #
        #   原来的判据有两个问题，合起来会把人**带向错误的模式**：
        #     ① 候选侧用整片场景算分位数，而历史参照是**裁过 ROI** 的 ——
        #        等于拿"整个库房"比"ROI 小盒子里的点"。这会系统性地奖励
        #        那个把场景缩小、正好塞进 ROI 的模式。
        #     ② 就算两边都裁 ROI，抓包与历史样本**根本不是同一个场景**
        #        （抓包多是空车道，历史样本是装着货的车），跨场景比
        #        x/y/z 分位数本来就是弱证据。
        #   2026-09-18 实测：四份真实抓包、三台雷达、两个日期，旧判据
        #   一致推荐 `raw_offset`，而它把地面放在离雷达 1.3 m 处 ——
        #   那等于说雷达吊在离地 1.3 m，而车厢本身就有 4 m 高。
        #   同一批数据里 `scale_offset` 把地面放在 -5.26，与 2026-09-14
        #   实测的吊装高度（地板 z ≈ -5.2）吻合。
        #
        #   地面高度是**独立、可复核的物理量**：空场时 ROI 里绝大多数点
        #   就是地面，它离雷达多远等于吊装高度，拿卷尺就能验。
        #   所以用它当主判据，分位数降级为参考。
        if expect_ground is not None and ms.roi_pct.get("z"):
            ms.ground_z_seen = float(ms.roi_pct["z"][1])        # ROI 内 z 的中位
            ms.ground_err = abs(ms.ground_z_seen - float(expect_ground))
            ms.score = ms.ground_err
        elif reference and ms.roi_pct:
            s = 0.0
            for axis in ("x", "y", "z"):
                if axis not in reference or axis not in ms.roi_pct:
                    continue
                ref = reference[axis]
                cur = ms.roi_pct[axis]
                # 用分位数区间的「量级」做归一化，避免 z（米级）压过 x/y
                scale = max(abs(ref[2] - ref[0]), 0.5)
                s += sum(abs(a - b) for a, b in zip(cur, ref)) / scale
            ms.score = s
        else:
            # 没有参照时：ROI 命中率越高越好（取负号统一成"越小越好"）
            ms.score = 10.0 * (1.0 - ms.roi_ratio)
        out.append(ms)
    return out


# ═════════════════════════════════════════════ 历史参照

def history_percentiles(directory: str, roi=None, limit: int = 0
                        ) -> Tuple[Dict[str, Tuple[float, float, float]], int, List[str]]:
    """汇总历史 .xyz 的 x/y/z 分位数，作为解码校准的参照系。"""
    from .sample_paths import discover_samples
    found = discover_samples(directory, role="detection", limit=limit)
    files = [sample.path for sample in found.samples]
    errs: List[str] = list(found.errors)
    pts: List[np.ndarray] = []
    for p in files:
        try:
            f = load_xyz(p)
        except Exception as e:                       # noqa: BLE001
            errs.append(f"{os.path.basename(p)}: {e}")
            continue
        points = apply_roi(f, roi).xyz if roi else f.xyz
        if not len(points):
            errs.append(f"{p}: 无有效点")
            continue
        pts.append(points)
    if not pts:
        return {}, 0, errs or ["没有可用的历史样本"]
    allp = np.concatenate(pts, axis=0)
    if len(allp) > 3_000_000:
        allp = allp[:: max(1, len(allp) // 3_000_000)]
    return _pcts(allp), len(pts), errs


# ═════════════════════════════════════════════ 报告

def build_report(capture_path: str, packets: Sequence[bytes], packet_cfg, proc_cfg,
                 angle_cache, roi, z_down: bool = True,
                 reference=None, history_dir: str = "",
                 hist_count: int = 0, history_errors: Optional[List[str]] = None,
                 expect_ground: Optional[float] = None
                 ) -> Dict:
    layout = scan_layout(packets, packet_cfg)
    fields = scan_point_bytes(packets, packet_cfg)
    scores = mode_scores(packets, packet_cfg, proc_cfg, angle_cache, roi,
                         z_down=z_down, reference=reference, expect_ground=expect_ground)

    usable = [s for s in scores if s.error == "" and s.n_points > 0]
    best = min(usable, key=lambda s: s.score) if usable else None

    verdict: List[str] = []
    sugg: Dict[str, object] = {}
    if best is not None:
        sugg["range_mode"] = best.mode
        by_ground = expect_ground is not None and best.ground_err != float("inf")
        conf = "高" if by_ground else ("中" if reference and len(usable) > 1 else "低")
        verdict.append(f"建议 range_mode = \"{best.mode}\"（置信度 {conf}）")
        verdict.append(f"  该模式下量程 2/50/98 分位 = "
                       f"{best.r_p02:.2f} / {best.r_p50:.2f} / {best.r_p98:.2f} m，"
                       f"ROI 命中率 {best.roi_ratio*100:.1f}%")
        if by_ground:
            verdict.append(f"  ★ 判据：ROI 内主导面（地面）应落在 z = {expect_ground:.2f} m")
            verdict.append("    模式            ROI 内 z 中位    离期望地面   ROI 命中率")
            for s in sorted(usable, key=lambda x: x.score):
                seen = "  --  " if s.ground_z_seen != s.ground_z_seen else f"{s.ground_z_seen:6.2f}"
                err = "  --  " if s.ground_err == float("inf") else f"{s.ground_err:6.2f}"
                verdict.append(f"    {s.mode:13s} {seen:>12s} m {err:>11s} m "
                               f"{s.roi_ratio*100:9.1f}%")
            verdict.append("  ▲ 空场时 ROI 里绝大多数点就是地面，它离雷达多远 = 吊装高度，")
            verdict.append("    拿卷尺就能复核。这是独立物理量，比分位数吻合度可靠。")
            verdict.append("  ▲ ROI 命中率**不该接近 100%** —— 雷达看的是整个库房，")
            verdict.append("    ROI 只是其中一个小盒子，正常会裁掉大部分回波。")
            verdict.append("    贴近 100% 说明整个场景被压进了 ROI，多半就是换算模式错了。")
        elif reference:
            verdict.append("  与历史 .xyz 分位数比对（越接近 0 越吻合，**均已裁 ROI**）：")
            for s in sorted(usable, key=lambda x: x.score):
                verdict.append(f"    {s.mode:13s} 偏差 {s.score:7.3f}")
            verdict.append("  ▲▲ 这是**弱证据**：抓包与历史样本多半不是同一个场景")
            verdict.append("     （抓包常是空车道，历史样本是装着货的车），跨场景比分位数")
            verdict.append("     会选错。**强烈建议加 --expect-ground <实测地面 z>** 重判一次。")
        else:
            verdict.append("  未提供任何参照，仅按 ROI 命中率排序 —— 这个排序**会选错**，"
                           "务必加 --expect-ground 或 --compare 再判一次")
    else:
        verdict.append("× 四种换算模式都无法产出有效点云 —— 请检查 packet 段的偏移配置")

    if fields.intensity_guess >= 0:
        verdict.append(f"强度字节疑似在第 {fields.intensity_guess} 字节"
                       f"（唯一值 {next((p['unique'] for p in fields.positions if p['byte']==fields.intensity_guess), '?')} 个）")
    if fields.flag_guess >= 0:
        verdict.append(f"标志/保留字节疑似在第 {fields.flag_guess} 字节")

    if layout.az_scale_guess:
        sugg["angle_scale"] = layout.az_scale_guess
        verdict.append(f"方位角标度建议 angle_scale = {layout.az_scale_guess}"
                       f"（{layout.az_scale_evidence}）")

    sugg["z_down"] = z_down
    if reference:
        rz, cz = reference.get("z"), (best.pct.get("z") if best else None)
        if rz and cz:
            # ★ 判 z 方向只能看**符号**，不能比中位数。
            #   中位数取决于场景里有什么：历史样本全是**载货车**（中位 -1.75 是货堆顶），
            #   而现场标定时车道往往是**空的**（中位就是地面 -5 左右）。
            #   拿这两个数比必然"不一致"，会一路把人引到 z_down=false 这条错路上
            #   —— 而 z_down 反了的真实表现是整片点云跑到 z 正半轴，一眼就能看出来。
            same_sign = (cz[1] < 0) == (rz[1] < 0)
            verdict.append(
                f"z 方向：历史点云在 z {'负' if rz[1] < 0 else '正'}半轴，"
                f"当前在 z {'负' if cz[1] < 0 else '正'}半轴 → "
                + ("√ 一致，z_down 当前取值正确"
                   if same_sign else
                   f"▲ 方向反了，把 z_down 改成 {str(not z_down).lower()}"))
            # 深度只做**提示**，不下判断：空场 vs 载货车的深度本来就不一样。
            verdict.append(
                f"　（参考：历史 z 的 2 分位 {rz[0]:.2f} m，当前 {cz[0]:.2f} m。"
                f"车道空着时这个位置就是地面；差一个固定倍率就是 range_mode 的问题，"
                f"上面已经选过了）")

    return {"capture": capture_path, "packets": len(packets),
            "roi": roi, "z_down": z_down,
            "layout": layout.to_dict(), "fields": fields.to_dict(),
            "modes": [s.to_dict() for s in scores],
            "reference_percentiles": {k: list(v) for k, v in (reference or {}).items()},
            "history_dir": history_dir, "history_files": hist_count,
            "history_errors": history_errors or [],
            "recommended": sugg, "verdict": verdict}


def format_report(rep: Dict) -> str:
    L: List[str] = []
    add = L.append
    add("=" * 78)
    add(f"雷达报文探测报告   {os.path.basename(rep['capture'])}")
    add("=" * 78)
    lay = rep["layout"]
    add(f"报文数量        : {lay['count']}")
    if lay["sizes"]:
        add(f"包长分布        : " + ", ".join(f"{k}B×{v}" for k, v in lay["sizes"].items()))
    add(f"block1 方位角   : {lay['az1_raw'][0]} … {lay['az1_raw'][1]} (原始值)")
    add(f"block2 方位角   : {lay['az2_raw'][0]} … {lay['az2_raw'][1]} (原始值)")
    if lay["az_step"][1]:
        add(f"方位角步长      : min {lay['az_step'][0]:.0f} / "
            f"中位 {lay['az_step'][1]:.0f} / max {lay['az_step'][2]:.0f} (原始值)")
    for n in lay["notes"]:
        add(f"  · {n}")
    add("")

    add("-" * 78)
    add("点位字节分布（用于指认强度 / 标志字节）")
    add("-" * 78)
    add(f"{'字节':<6}{'最小':>8}{'最大':>8}{'均值':>10}{'标准差':>10}{'唯一值':>8}")
    for p in rep["fields"]["positions"]:
        add(f"{p['byte']:<6}{p['min']:>8}{p['max']:>8}{p['mean']:>10.1f}"
            f"{p['std']:>10.1f}{p['unique']:>8}")
    for n in rep["fields"]["notes"]:
        add(f"  · {n}")
    add("")

    add("-" * 78)
    add("距离换算模式对比")
    add("-" * 78)
    add(f"{'模式':<14}{'点数':>10}{'ROI命中':>10}{'命中率':>9}"
        f"{'量程 p2/p50/p98 (m)':>28}{'偏差':>10}")
    for m in rep["modes"]:
        if m["error"]:
            add(f"{m['mode']:<14}{'—':>10}{'—':>10}{'—':>9}{m['error'][:40]:>28}{'':>10}")
            continue
        r = m["range_m"]
        score = "—" if m["score"] is None else f"{m['score']:.3f}"
        add(f"{m['mode']:<14}{m['n_points']:>10}{m['n_in_roi']:>10}"
            f"{m['roi_ratio']*100:>8.1f}%"
            f"{f'{r[0]:.2f}/{r[1]:.2f}/{r[2]:.2f}':>28}{score:>10}")
    add("")

    if rep["reference_percentiles"]:
        add("-" * 78)
        add(f"历史参照（{rep['history_files']} 个 .xyz，截取后分位数）")
        add("-" * 78)
        for axis, v in rep["reference_percentiles"].items():
            add(f"  {axis}: p2={v[0]:>8.2f}  p50={v[1]:>8.2f}  p98={v[2]:>8.2f}")
        add("")
    for e in rep["history_errors"]:
        add(f"  ▲  历史样本读取失败: {e}")

    add("=" * 78)
    add("结论")
    add("=" * 78)
    for line in rep["verdict"]:
        add(line)
    if rep["recommended"]:
        add("")
        add("建议写回 config/channels.json → channels[].lidar：")
        add(json.dumps(rep["recommended"], ensure_ascii=False, indent=2))
    add("")
    return "\n".join(L)
