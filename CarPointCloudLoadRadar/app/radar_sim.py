#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
radar_sim.py — 报文级雷达仿真（无硬件冷启动 / 演示 / 自测）

为什么不直接用 make_synthetic_box()？
    那是「直接造点云」，跳过了解码环节。本模块是「造原始 UDP 报文」——
    先按真实几何做射线求交算出每根激光的距离，再按 config 的换算关系
    **反推**出原始整数，最后拼成一个合法报文。

    于是它构成一条闭环自检：
        仿真报文 → PacketDecoder 解码 → Frame → 检测 → 判定
    如果解码写错了（比如 z 方向搞反、方位角标度错），这条链会立刻暴露
    （点云会跑到 ROI 外面、地面不在 z≈-5、判定变成"未见厢底平面"）。

几何模型
    雷达位于原点，**吊装在车厢上方、镜头俯视**（与已确认的 z_down 约定一致）。
    场景由若干「实体盒」组成：
        车厢内腔  必填，一个空心盒（射线从内侧射出 → 命中"出射面"）
        货堆      可选，实心盒放在车厢底部（射线从外部射入 → 命中"入射面"）
    对每根激光取所有命中面里最近的正距离。

这是一份"够用就好"的仿真：不做多路径、不做反射率、不做运动模糊。
目的只有一个 —— 让离线链路和现场链路走同一套代码。
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from .radar_packet import PacketDecoder, POINT_RAW_DTYPE, RANGE_MODES


@dataclass
class Box:
    """
    轴对齐盒。lo/hi 是车体坐标下界/上界（z 向下为负）。

    hollow
        False（默认）→ 实心盒，取**入射面**(t_min)。用于货堆。
        True         → 空心盒，取**出射面**(t_max)。用于车厢内腔 ——
                       雷达吊在车厢上方透过敞开的顶部往里看，
                       看到的第一个面就是内腔的"出射面"（地板/侧壁/端壁）。
                       若当实心处理，射线会先打在"车顶外表面"上，
                       整帧点云会塌成一张 z≈车顶 的平面（这个坑踩过）。
    """
    lo: Tuple[float, float, float]
    hi: Tuple[float, float, float]
    hollow: bool = False

    def arrays(self):
        return np.asarray(self.lo, np.float64), np.asarray(self.hi, np.float64)


def box_truck(dx: float = 2.4, dy: float = 12.0, dz: float = 2.6,
              z_floor: float = -5.0, center_y: float = 0.0) -> Box:
    """车厢内腔：宽 dx、长 dy、高 dz，地板在 z_floor（负值，向下）。空心盒。"""
    return Box((-dx / 2.0, center_y - dy / 2.0, z_floor),
               (dx / 2.0, center_y + dy / 2.0, z_floor + dz),
               hollow=True)


def box_cargo(w: float = 1.8, l: float = 6.0, h: float = 1.2,
              z_floor: float = -5.0, center_y: float = 0.0) -> Box:
    """货堆：坐在车厢地板上。实心盒。"""
    return Box((-w / 2.0, center_y - l / 2.0, z_floor),
               (w / 2.0, center_y + l / 2.0, z_floor + h),
               hollow=False)


def box_ground(z_floor: float = -5.0, half: float = 30.0,
               thick: float = 0.3) -> Box:
    """
    检测位地面：一块铺开的大薄板，上表面在 z_floor。实心盒（取入射面）。

    ★ 为什么演示场景需要它

    真实车道空着的时候，雷达照样扫到地面，**每帧都有上万点回波**。
    没有地面的仿真在"车开走"后会一帧点都没有 ——
    `RadarSimSource.read()` 返回 None，`_tick` 直接 return，
    于是看板上那条通道**整个僵住不动**（状态、帧率、时间戳全不更新），
    看起来像程序挂了。

    加了地面之后：
      · 空车道仍有上万点 → 帧不为空 → 状态持续刷新，看板是"活的"；
      · 但地面点在 z=-5.0，而"有车"判据只数 z > ground_z + 0.3 = -4.7 的点
        （见 detect.StabilityDetector.vehicle_points 的注释），
        所以地面**不会**被误判成车 → 状态正确停在「空闲」。
    这两条合起来才等价于真实车道，演示才不会露馅。
    """
    return Box((-half, -half, z_floor - thick), (half, half, z_floor),
               hollow=False)


def ray_box_entry(origin: np.ndarray, dirs: np.ndarray, box: Box,
                  eps: float = 1e-12) -> np.ndarray:
    """
    射线与盒的「首次可见命中距离」。方向向量需已单位化。未命中返回 +inf。

    标准 slab 算法，注意两个必须处理对的地方：
      · **平行射线**：dir 某轴分量 ≈ 0 时，该轴不产生 t 区间约束，
        但要求原点落在该轴的 [lo,hi] 内；否则整条射线在盒外，不能命中。
        （漏了这个判断会让"从盒子上方水平掠过"的射线错误命中盒子的竖直侧面。）
      · **实心 vs 空心**：见 Box.hollow 的说明。
    """
    lo, hi = box.arrays()
    n = dirs.shape[0]
    tmin = np.full(n, -np.inf)
    tmax = np.full(n, np.inf)
    parallel_ok = np.ones(n, bool)

    for a in range(3):
        d = dirs[:, a]
        o = float(origin[a])
        par = np.abs(d) < eps
        parallel_ok &= ~par | ((o >= lo[a]) & (o <= hi[a]))
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (lo[a] - o) / d
            t2 = (hi[a] - o) / d
        l_t = np.where(par, -np.inf, np.minimum(t1, t2))
        h_t = np.where(par, np.inf, np.maximum(t1, t2))
        tmin = np.maximum(tmin, l_t)
        tmax = np.minimum(tmax, h_t)

    hit = parallel_ok & (tmax >= np.maximum(tmin, 0.0))
    t = np.where(box.hollow, tmax, np.where(tmin > 0, tmin, tmax))
    return np.where(hit & (t > 0), t, np.inf)


class RadarSimulator:
    """
    按 config 的 packet / processing / angle_cache 生成合法报文。

    用法：
        sim = RadarSimulator(calib.packet, calib.processing, angle_cache,
                             range_mode="raw_offset", z_down=True)
        packets = sim.scan([box_truck(), box_cargo()])     # 转一圈的报文
        for b in packets: decoder.decode(b) ...
    """

    def __init__(self, packet_cfg, proc_cfg, angle_cache=None,
                 range_mode: str = "raw_offset", z_down: bool = True,
                 az_step_deg: float = 0.35, range_noise_m: float = 0.01,
                 intensity_nominal: int = 120, seed: int = 0,
                 dropout_prob: float = 0.02):
        self.p = packet_cfg
        self.proc = proc_cfg
        self.angle = angle_cache
        self.range_mode = range_mode if range_mode in RANGE_MODES else "raw_offset"
        self.z_down = z_down
        self.az_step_deg = az_step_deg
        self.range_noise_m = range_noise_m
        self.intensity_nominal = intensity_nominal
        self.dropout_prob = dropout_prob
        self._dec = PacketDecoder(packet_cfg, proc_cfg, angle_cache,
                                  range_mode=range_mode, min_range_m=0.0,
                                  max_range_m=1e9, z_down=z_down)
        self._pps = self._dec._pps
        self.rng = np.random.default_rng(seed)
        self._build_rays()

    # ── 每根激光的单位方向（车体坐标）────────────────────────
    def _build_rays(self):
        ch = np.arange(self._pps)
        if self.angle is not None and getattr(self.angle, "loaded", False):
            elev = self.angle.elevation[:self._pps].astype(np.float64)
            corr = self.angle.azimuth[:self._pps].astype(np.float64)
        else:
            elev = np.linspace(0.0, 89.0, self._pps)
            corr = np.zeros(self._pps)
        self._elev = elev
        self._azim_corr = corr
        self._el = elev * self.proc.deg_to_rad
        self._az0_offset = corr + float(self.proc.yaw_offset_deg)
        self._zsign = -1.0 if self.z_down else 1.0

    def _dirs(self, az_deg: float) -> np.ndarray:
        az = (self._az0_offset + az_deg) * self.proc.deg_to_rad
        ce, se = np.cos(self._el), np.sin(self._el)
        d = np.empty((self._pps, 3), np.float64)
        d[:, 0] = ce * np.sin(az)
        d[:, 1] = ce * np.cos(az)
        d[:, 2] = self._zsign * se
        n = np.linalg.norm(d, axis=1, keepdims=True)
        return d / np.maximum(n, 1e-12)

    def _dirs_many(self, az_deg: np.ndarray) -> np.ndarray:
        """一次算出多个方位角的激光方向。az_deg 形状 (K,) → 返回 (K, pps, 3)。

        ★ 为什么要有这个"批量版"
            `_dirs` 一次只算 128 根激光，而转一圈要 360 包 × 2 块 = 720 次调用，
            每次都是十几个**小数组**numpy 运算 —— 单次只有微秒级的调用开销
            却远大于算本身。实测（本机）：
                · 逐包版  360 包 → 0.111 s/帧
                · 4 路并发时因为 GIL 被反复抢占，退化到 0.68 s/帧（6 倍）
            批量版把 720 次小调用合成 1 次大数组运算，帧耗时降到 ~0.01 s 量级，
            而且大数组运算会释放 GIL，4 路并行才真的并起来。
            演示时这就决定了"车是滑进来的"还是"一帧一帧跳进来的"。
        """
        az = (self._az0_offset[None, :] + az_deg[:, None]) * self.proc.deg_to_rad
        ce = np.cos(self._el)[None, :]
        se = np.sin(self._el)[None, :]
        d = np.empty((az.shape[0], self._pps, 3), np.float64)
        d[:, :, 0] = ce * np.sin(az)
        d[:, :, 1] = ce * np.cos(az)
        d[:, :, 2] = self._zsign * se
        n = np.linalg.norm(d, axis=2, keepdims=True)
        return d / np.maximum(n, 1e-12)

    # ── 距离 → 原始整数（to_range_m 的逆运算）────────────────
    def _encode_range(self, r_m: np.ndarray) -> np.ndarray:
        sc = float(self.proc.distance_scale)
        off = float(self.proc.offset_r)
        mm = r_m * 1000.0
        m = self.range_mode
        if m == "scale":
            raw = mm / sc
        elif m == "scale_offset":
            raw = (mm + off) / sc
        elif m == "raw_offset":
            raw = mm + off
        elif m == "scale_plus":
            raw = (mm - off) / sc
        else:
            raw = mm / sc
        return np.clip(np.rint(raw), 0, 65535).astype(np.uint16)

    # ── 单个方位角 → 一包的两块 ───────────────────────────────
    def _one_packet_with(self, boxes: Sequence[Box], az_deg: float) -> bytes:
        """用指定的场景集合生成一个报文（供运行时切换场景使用）。"""
        prev = getattr(self, "boxes", None)
        self.boxes = list(boxes)
        try:
            return self._one_packet(az_deg)
        finally:
            self.boxes = prev

    def _one_packet(self, az_deg: float) -> bytes:
        p = self.p
        buf = bytearray(p.size)
        a1 = int(round(az_deg / 0.01)) & 0xFFFF
        a2 = int(round((az_deg + self.az_step_deg / 2.0) / 0.01)) & 0xFFFF
        struct.pack_into("<H", buf, p.angle1_offset, a1)
        struct.pack_into("<H", buf, p.angle2_offset, a2)

        for off, d_az in ((p.block1_data_offset, az_deg),
                          (p.block2_data_offset, az_deg + self.az_step_deg / 2.0)):
            dirs = self._dirs(d_az)
            d = np.full(self._pps, np.inf)
            for box in self.boxes:
                d = np.minimum(d, ray_box_entry(np.zeros(3), dirs, box))
            # 未命中 / 超量程 → **无回波**：raw 写 0，解码端会按距离窗过滤掉。
            # （真实雷达不会返回一个"40 米的假点"，这点必须仿真对，
            #   否则点云会被虚假远点污染，检测结果全错。）
            bad = (~np.isfinite(d)) | (d > self.max_probe_m)
            if self.range_noise_m > 0:
                d = np.where(bad, 0.0, d + self.rng.normal(0, self.range_noise_m, self._pps))
            raw = self._encode_range(np.maximum(d, 0.0))
            raw = np.where(bad, 0, raw).astype(np.uint16)

            pts = np.zeros(self._pps, dtype=POINT_RAW_DTYPE)
            pts["r"] = raw
            inten = self.rng.normal(self.intensity_nominal, 18, self._pps)
            inten = np.clip(inten - np.where(bad, 0.0, d) * 2.0, 1, 255)  # 远处更弱
            pts["intensity"] = inten.astype(np.uint8)
            pts["flag"] = 0
            if self.dropout_prob > 0:
                drop = self.rng.random(self._pps) < self.dropout_prob
                pts["r"] = np.where(drop, 0, pts["r"])
            buf[off:off + pts.nbytes] = pts.tobytes()

        if p.imu_offset and p.imu_offset + 6 <= p.size:
            struct.pack_into("<hhh", buf, p.imu_offset, 0, 0, 2048)
        return bytes(buf)

    max_probe_m = 40.0

    # ── 整圈批量生成（快的那个）──────────────────────────────
    def scan_packets(self, boxes: Sequence[Box], az_start: float,
                     az_step: float, n_packets: int) -> List[bytes]:
        """
        一次生成 n_packets 个报文（覆盖一整圈）—— 与 _one_packet 几何等价。

        不逐包算，而是把 2×n_packets 个方位角的所有激光拉成一个大数组
        统一做射线求交（见 _dirs_many 的说明）。实测 360 包：
            逐包 0.111 s/帧  →  批量 ~0.010 s/帧（约 11 倍）

        语义与逐包版一致：未命中/超量程的激光写 raw=0（解码端按距离窗滤掉），
        不做"40 米的假回波"。
        """
        boxes = list(boxes)
        K = max(1, int(n_packets))
        step = float(az_step)
        az1 = float(az_start) + step * np.arange(K, dtype=np.float64)     # (K,)
        # 两块的方位角：第 1 块 az，第 2 块 az + step/2
        az_blk = np.empty((K, 2), np.float64)
        az_blk[:, 0] = az1
        az_blk[:, 1] = az1 + step / 2.0
        flat_az = az_blk.reshape(-1)                                      # (2K,)

        pps = self._pps
        dirs = self._dirs_many(flat_az).reshape(-1, 3)                    # (2K*pps, 3)
        origin = np.zeros(3, np.float64)
        dist = np.full(dirs.shape[0], np.inf, np.float64)
        for box in boxes:
            dist = np.minimum(dist, ray_box_entry(origin, dirs, box))

        bad = (~np.isfinite(dist)) | (dist > self.max_probe_m)
        if self.range_noise_m > 0:
            dist = np.where(bad, 0.0, dist + self.rng.normal(0, self.range_noise_m,
                                                             dist.shape[0]))
        raw = self._encode_range(np.maximum(dist, 0.0))
        raw = np.where(bad, 0, raw).astype(np.uint16)

        inten = self.rng.normal(self.intensity_nominal, 18, dist.shape[0])
        inten = np.clip(inten - np.where(bad, 0.0, dist) * 2.0, 1, 255)
        if self.dropout_prob > 0:
            drop = self.rng.random(dist.shape[0]) < self.dropout_prob
            raw = np.where(drop, 0, raw).astype(np.uint16)

        pts = np.zeros((2 * K, pps), dtype=POINT_RAW_DTYPE)
        pts["r"] = raw.reshape(2 * K, pps)
        pts["intensity"] = inten.astype(np.uint8).reshape(2 * K, pps)
        pts["flag"] = 0
        blobs = [pts[k].tobytes() for k in range(pts.shape[0])]           # 2K 个块

        p = self.p
        az_field = az1 % 360.0
        a1 = (np.rint(az_field / 0.01).astype(np.int64) & 0xFFFF)
        a2 = (np.rint((az_field + step / 2.0) / 0.01).astype(np.int64) & 0xFFFF)
        out: List[bytes] = []
        for i in range(K):
            buf = bytearray(p.size)
            struct.pack_into("<H", buf, p.angle1_offset, int(a1[i]))
            struct.pack_into("<H", buf, p.angle2_offset, int(a2[i]))
            b1 = blobs[2 * i]
            b2 = blobs[2 * i + 1]
            o1 = p.block1_data_offset
            o2 = p.block2_data_offset
            # ★ 布局配置错必须在造包时报错，而不是静默扩长报文（低严重度
            #   批次）：bytearray 切片赋值超出时会**变长**，产出与
            #   packet.size 不符的畸形包，解码端才发现且无从定位。
            if o1 + len(b1) > p.size or o2 + len(b2) > p.size:
                raise ValueError(
                    "雷达仿真布局越界：block 数据（%d+%d / %d+%d 字节）装不进 "
                    "packet.size=%d —— 检查 config.json 的 packet 段"
                    % (o1, len(b1), o2, len(b2), p.size))
            buf[o1:o1 + len(b1)] = b1
            buf[o2:o2 + len(b2)] = b2
            if p.imu_offset and p.imu_offset + 6 <= p.size:
                struct.pack_into("<hhh", buf, p.imu_offset, 0, 0, 2048)
            out.append(bytes(buf))
        return out

    # ── 转一圈 ───────────────────────────────────────────────
    def scan(self, boxes: Sequence[Box], az_start: float = 0.0,
             az_end: float = 360.0) -> List[bytes]:
        """按方位角连续扫一圈，返回报文列表。"""
        self.boxes = list(boxes)
        step = max(0.01, float(self.az_step_deg))
        n = int(math.ceil((az_end - az_start) / step))
        if n <= 0:
            return []
        return self.scan_packets(self.boxes, az_start % 360.0, step, n)

    # ── 直接给出场景点云（校验用，跳过编解码）────────────────
    def truth_cloud(self, boxes: Sequence[Box], az_step: float = 1.0
                    ) -> np.ndarray:
        self.boxes = list(boxes)
        pts = []
        az = 0.0
        while az < 360.0:
            dirs = self._dirs(az)
            d = np.full(self._pps, np.inf)
            for box in boxes:
                d = np.minimum(d, ray_box_entry(np.zeros(3), dirs, box))
            m = np.isfinite(d)
            pts.append(dirs[m] * d[m, None])
            az += az_step
        return np.concatenate(pts, axis=0).astype(np.float32) if pts else \
            np.zeros((0, 3), np.float32)


def make_demo_scene(loaded: bool = False, z_floor: float = -5.0,
                    dx: float = 2.4, dy: float = 12.0, dz: float = 2.6,
                    cargo_scale: Tuple[float, float, float] = (0.95, 0.85, 0.45)):
    """
    演示用场景：空车厢，或空车厢 + 一垛几乎铺满车厢底板的货。

    cargo_scale 是相对车厢的相对尺寸 (宽, 长, 高)。默认 (0.95, 0.85, 0.45)
    是实测调出来的 —— 见下。

    为什么不能用"半车货"？
        底面拟合的种子策略是"取 z 最低的 30% 点做 PCA"，它默认
        **最低处的密集面就是地板**。当货堆只占一部分时，会同时出现
        "地板"和"货堆顶面"两个平面，种子点集混合两者 → PCA 得到一个
        夹在中间的假平面 → 判定停在「底面内点不足」。
        只有货堆几乎铺满底板（覆盖 ≥ 约 80%）时，种子点集才几乎全是
        货堆顶面 → PCA 锁到货堆顶 → 地板被报得比最低点高 → 
        得到原版那个招牌判定「底面过高，疑似载物」。

    这个敏感性与真实数据一致：4 份真实载货样本（z≈-3.6 的"地板"、
    最低点 z≈-5.3）走的正是这条路径，真值就是「底面过高，疑似载物」。
    它同时说明：**若现场货堆较小而底面积大，算法可能判不出来** ——
    这属于原版算法的固有特性，已在 docs/03-现场调试.md 里作为
    已知限制记录，并给出缓解建议。
    """
    boxes = [box_truck(dx, dy, dz, z_floor)]
    if loaded:
        cw, cl, ch = cargo_scale
        boxes.append(box_cargo(w=dx * cw, l=dy * cl, h=dz * ch, z_floor=z_floor))
    return boxes
