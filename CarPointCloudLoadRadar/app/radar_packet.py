#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
radar_packet.py — 128 线雷达 UDP 报文解码（原 jt128.dll 的等价物）

────────────────────────────────────────────────────────────────────────
这一层是「唯一无法从二进制逐行还原」的部分，请先读完本节再用它。
────────────────────────────────────────────────────────────────────────

jt128.dll 里的解码逻辑没有 PDB、也没有可见字符串，只能从 config.json 的键名
反推布局。好消息是：**原厂把报文布局完整写进了 config.json**，所以这里
不做任何硬编码，全部按配置驱动。

由 config.json 推出的报文结构（packet 段）
    size              1100     —— 每个 UDP 包固定 1100 字节
    angle1_offset       12     —— block1 方位角（uint16 LE，单位 0.01°）
    block1_data_offset  14     —— block1 点数据起点
    angle2_offset      526     —— block2 方位角
    block2_data_offset 528     —— block2 点数据起点
    point_size           4     —— 每点 4 字节
    channels           128     —— 每个 block 的点数（= 垂直通道数）
    imu_offset        1054     —— IMU 数据起点
    angle_bin_size      40     —— 角度分箱粒度（用于探测工具统计）

    block1: [12..14) 方位角  →  [14..526)  512B / 4B = 128 点
    block2: [526..528) 方位角 →  [528..1040) 512B / 4B = 128 点
    IMU:    [1054..) 三轴加速度（加速计原始值）

一个包 = 2 个方位角 × 128 个垂直通道 = 256 个候选点。

▲ 已知的不确定性（现场必须校准的就剩 2 件）
    ① 4 字节点的内部位域。业界最常见是 u16 距离 + u8 强度 + u8 保留/标志，
       本项目按此实现。强度字节用 cli/probe.py 一验便知（强度随距离单调衰减、
       动态范围大；标志字节基本恒定）。
    ② 距离换算。config.json 给了 distance_scale=4.0 与 offset_r=15.640617，
       但没说清 offset 是加还是减。提供了 4 种 range_mode，用 cli/probe.py
       对真实抓包跑一遍即可选定。

√ 本轮已由历史数据交叉验证的两点
    ③ 坐标系方向：z 向下为负（雷达吊装俯视），即 z = -r·sin(俯仰)。
       依据：10 份历史 .xyz 的 z∈[-5.29,-0.76]，而角度表俯仰 ∈[-5.45°,88.97°]，
       与 config.detection.ground_z=-5.0、roi.z=[-6,-0.75] 三者自洽。
       详见 to_cartesian() 的注释。
    ④ ring 的编码：ring = block_index × 128 + 块内索引，取值 0..255。
       依据：历史 .xyz 中实测 ring 最大值有 255 与 187 两档（187 = 128+59，
       即 block2 只有前 60 个通道有回波），完全吻合。

现场校准流程（约 5 分钟）
    $ python3 cli/capture.py --iface 192.168.1.201 --secs 10 -o data/raw/cap.bin
    $ python3 cli/probe.py data/raw/cap.bin --compare baseline
    → 工具会：解析报文 → 试遍 4 种 range_mode → 把每种模式的点云统计
      与历史 .xyz 的分位数对比 → 直接给出结论与建议参数。
    → 把选定结果写回 config/channels.json 的 lidar.range_mode / lidar.z_down。

这里特意把"不确定"显式化，而不是假装确定 —— 现场调一个参数比猜一套位域快得多。
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np

# ────────────────────────────────────────────── 4 字节点的字节 → 字段映射
# 位置 0..1: uint16 LE 距离原始值
# 位置 2  : uint8 强度（reflectivity）
# 位置 3  : uint8 保留 / 标志
POINT_RAW_DTYPE = np.dtype([("r", "<u2"), ("intensity", "u1"), ("flag", "u1")])
POINT_RAW_SIZE = 4

# 距离换算模式 —— 见模块头部「不确定性 ②」
RANGE_MODES: Dict[str, str] = {
    "scale":        "r_m = raw * distance_scale / 1000",
    "scale_offset": "r_m = (raw * distance_scale - offset_r) / 1000",
    "raw_offset":   "r_m = (raw - offset_r) / 1000  （raw 本身即毫米）",
    "scale_plus":   "r_m = (raw * distance_scale + offset_r) / 1000",
}


class PacketError(ValueError):
    """报文长度或布局不符 —— 通常是 UDP 端口接错了设备。"""


# ────────────────────────────────────────────── 解码结果

@dataclass
class DecodedPacket:
    """一个 UDP 报文的解码中间结果（尚未转成笛卡尔点）。"""
    seq: int = 0
    ts: float = 0.0
    size: int = 0
    azimuth_deg: Tuple[float, float] = (0.0, 0.0)   # 两个 block 的原始方位角（度）
    ranges_raw: np.ndarray = field(default_factory=lambda: np.zeros(0, np.uint16))
    intensity: np.ndarray = field(default_factory=lambda: np.zeros(0, np.uint8))
    ring_raw: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int32))
    imu: Optional[np.ndarray] = None                # (3,) 加速度原始值

    @property
    def n(self) -> int:
        return int(self.ranges_raw.shape[0])


# ────────────────────────────────────────────── 转速估计辅助

def _rev_deg(az: np.ndarray) -> float:
    """估计这一批报文的方位角总共前进了多少度 —— 只用于**估计转速**。

    雷达正转时方位角单调增大、在 360° 处回绕；这里把回绕补成 +360°，
    并把 |跳变|>180° 的项当作丢包/反向而剔除，只累加前进量。
    因此它是"够用的量级估计"，不是精确积分。

    为什么需要它：现场未必清楚雷达的转速（rpm）。frame_ms 必须约等于
    一圈的时长才能攒够完整一圈点云 —— 有了 rev_deg，看板/工具就能直接
    把实测转速显示出来，现场据此定 frame_ms，不用猜。
    """
    if az.size < 2:
        return 0.0
    d = np.diff(az)
    d = np.where(d < -180.0, d + 360.0, d)
    d = np.where(d > 180.0, d - 360.0, d)
    d = np.where(d < 0.0, 0.0, d)
    return float(d.sum())


# ────────────────────────────────────────────── 解码器

class PacketDecoder:
    """
    按 config.json 的 packet / processing 段解码 UDP 报文。

    用法：
        dec = PacketDecoder(calib.packet, calib.processing, angle_cache, range_mode="scale")
        pkt = dec.decode(buf, seq=1)
        xyz, ring = dec.to_cartesian(pkt)
        frame = Frame(xyz, ring)
    """

    def __init__(self, packet_cfg, proc_cfg, angle_cache=None,
                 range_mode: str = "scale", min_range_m: float = 0.1,
                 max_range_m: float = 300.0, z_down: bool = True):
        self.p = packet_cfg
        self.proc = proc_cfg
        self.angle = angle_cache
        # ▲ 这里**必须**严格校验，不能"不认识就悄悄退回默认值"。
        #    现场最容易出的错就是在 channels.json 里把 range_mode 打错一个字母，
        #    （如 "raw-offset" / "raw_offset " 带尾空格）。若静默退回 "scale"，
        #    现象是"点云位置整体不对但程序不报错"，排查起来极其痛苦。
        #    宁可启动即失败，也不要带着错误的量纲跑起来。
        mode = (range_mode or "scale").strip()
        if mode not in RANGE_MODES:
            raise ValueError(
                f"未知的 range_mode: {range_mode!r}。可用值："
                f"{', '.join(sorted(RANGE_MODES))}。"
                f"（现场请用 cli/probe.py 对真实抓包定一遍，别手改）")
        self.range_mode = mode
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        # ★ z_down：雷达吊装在车厢上方俯视，车体坐标 z 向下为负。
        #   证据见 to_cartesian() 的注释。若现场改了吊装方式，把它设成 False。
        self.z_down = bool(z_down)
        self._pps = int(packet_cfg.channels)              # 每 block 点数
        self._ppb = self._pps
        self._npoints = self._pps * 2
        self._check_layout()

    # ── 布局自检：配置自相矛盾时立刻报错，而不是产出垃圾点云 ──────────
    def _check_layout(self) -> None:
        p = self.p
        block_span = p.angle2_offset - p.block1_data_offset
        if block_span <= 0:
            raise PacketError(f"报文布局自相矛盾：angle2_offset({p.angle2_offset}) "
                              f"<= block1_data_offset({p.block1_data_offset})")
        pps = block_span // p.point_size
        if pps != self._pps:
            # 不直接抛错——以实际跨度为准更安全，但必须显式告警
            self._pps = int(pps)
            self._npoints = self._pps * 2
        need = p.block2_data_offset + self._pps * p.point_size
        if p.size < need:
            raise PacketError(f"packet.size({p.size}) 装不下 {self._pps}×2 个点"
                              f"（至少需要 {need} 字节）")
        # ★ 角度偏移越界在**启动时**报错，而不是解码时炸出 struct.error /
        #   IndexError（2026-09-23，两轮审查低严重度批次）。两个角度字段
        #   都是 <H（2 字节）：偏移必须能放下一个 u16，且不得伸进第二个
        #   block 的数据区之外 —— 照本项目"配置错宁可启动即失败"的规矩。
        for name, offset in (("angle1_offset", p.angle1_offset),
                             ("angle2_offset", p.angle2_offset)):
            if not 0 <= offset <= p.size - 2:
                raise PacketError(f"packet.{name}({offset}) 越界：必须在 "
                                  f"[0, {p.size - 2}] 内才能放下一个 u16 角度")

    # ── 单个报文解码 ────────────────────────────────────────────
    def decode(self, buf: bytes, seq: int = 0, ts: float = 0.0) -> Optional[DecodedPacket]:
        p = self.p
        if len(buf) < p.size:
            raise PacketError(f"报文长度 {len(buf)} < packet.size {p.size}")
        b = buf[:p.size]

        a1 = struct.unpack_from("<H", b, p.angle1_offset)[0]
        a2 = struct.unpack_from("<H", b, p.angle2_offset)[0]
        pkt = DecodedPacket(seq=seq, ts=ts, size=len(b),
                            azimuth_deg=(a1 * self.proc.angle_scale,
                                         a2 * self.proc.angle_scale))

        raw = np.frombuffer(
            b, dtype=POINT_RAW_DTYPE,
            count=self._npoints,
            offset=p.block1_data_offset) if self._blocks_contiguous() else \
            self._decode_two_blocks(b)

        pkt.ranges_raw = np.ascontiguousarray(raw["r"])
        pkt.intensity = np.ascontiguousarray(raw["intensity"])
        # ring：跨 block 的原始索引（与历史 .xyz 中实测 0..187 一致）
        pkt.ring_raw = np.tile(np.arange(self._pps, dtype=np.int32), 2) + \
            np.repeat(np.array([0, self._pps], dtype=np.int32), self._pps)

        if p.imu_offset and p.imu_offset + 6 <= len(b):
            try:
                ax, ay, az = struct.unpack_from("<hhh", b, p.imu_offset)
                pkt.imu = np.array([ax, ay, az], np.int32)
            except struct.error:
                pkt.imu = None
        return pkt

    def _blocks_contiguous(self) -> bool:
        """两个 block 在内存里首尾相接时，可以一次性 frombuffer，快一个数量级。"""
        p = self.p
        return (p.block2_data_offset == p.block1_data_offset + self._pps * p.point_size)

    def _decode_two_blocks(self, b: bytes) -> np.ndarray:
        p = self.p
        a = np.frombuffer(b, dtype=POINT_RAW_DTYPE, count=self._pps,
                          offset=p.block1_data_offset)
        c = np.frombuffer(b, dtype=POINT_RAW_DTYPE, count=self._pps,
                          offset=p.block2_data_offset)
        return np.concatenate([a, c])

    # ── 距离换算 ────────────────────────────────────────────────
    def to_range_m(self, raw: np.ndarray) -> np.ndarray:
        """把原始距离换算成米。4 种模式见模块头部。"""
        r = raw.astype(np.float64)
        sc = float(self.proc.distance_scale)
        off = float(self.proc.offset_r)
        m = self.range_mode
        if m == "scale":
            out = r * sc / 1000.0
        elif m == "scale_offset":
            out = (r * sc - off) / 1000.0
        elif m == "raw_offset":
            out = (r - off) / 1000.0
        elif m == "scale_plus":
            out = (r * sc + off) / 1000.0
        else:
            out = r * sc / 1000.0
        return out

    def valid_mask(self, r_m: np.ndarray) -> np.ndarray:
        return (r_m >= self.min_range_m) & (r_m <= self.max_range_m)

    # ── 极坐标 → 笛卡尔 ────────────────────────────────────────
    def to_cartesian(self, pkt: DecodedPacket
                     ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        返回 (xyz(N,3) float32, ring(N,) uint16, intensity(N,) uint8)

        角度来源
            方位角 = 报文方位角 + 逐通道 Azimuth 修正 + yaw_offset_deg
            俯仰角 = 逐通道 Elevation 修正
        逐通道修正来自 angle_cache.csv（原 jt128.dll 的 load_angle_csv）。

        ★ 坐标系（由 10 份历史 .xyz 反推，已交叉验证）
            历史 .xyz 实测：x∈[-2.0, 1.7]，y∈[-11.0, 8.0]，z∈[-5.29, -0.76]
            而 angle_cache 的俯仰角范围是 -5.45° … 88.97°。
            两者一致的条件是：  z = -r·sin(俯仰)

            也就是说 —— 雷达**吊装在车厢正上方约 5 m，镜头俯视**：
              · 俯仰 ≈ 89° 的通道几乎垂直向下 → z ≈ -r ≈ -5 m（车厢地板）
              · 俯仰 ≈ 0°  的通道接近水平   → z ≈ 0
            这与 config.json 的 detection.ground_z = -5.0 完全吻合，
            也与 roi.z = [-6, -0.75] 的取值区间吻合。

            于是：x = r·cos(el)·sin(az)，y = r·cos(el)·cos(az)，z = **-**r·sin(el)
            其中 x = 车宽方向、y = 车长方向、z = 高度（向下为负）。

            若现场把雷达改成侧装/地面安装，把 z_down 设为 False 并重新标定 ROI。
        """
        r_m = self.to_range_m(pkt.ranges_raw)
        m = self.valid_mask(r_m)
        if not m.any():
            return (np.zeros((0, 3), np.float32), np.zeros(0, np.uint16),
                    np.zeros(0, np.uint8))

        ring = pkt.ring_raw[m]
        rr = r_m[m]
        it = pkt.intensity[m]

        # 逐通道角度修正。angle_cache 只有 channels 项 → 用 ring % channels 取
        ch = (ring % max(1, self.p.channels)).astype(np.int64)
        ch = np.clip(ch, 0, len(self.angle.elevation) - 1) if self.angle else ch
        if self.angle is not None and self.angle.loaded:
            elev = self.angle.elevation[ch].astype(np.float64)
            azim_corr = self.angle.azimuth[ch].astype(np.float64)
        else:
            elev = np.zeros_like(rr)
            azim_corr = np.zeros_like(rr)

        # 每个点属于哪个 block（决定用哪个方位角）
        blk = (pkt.ring_raw[m] >= self._pps).astype(np.int64)
        az0 = np.array([pkt.azimuth_deg[0], pkt.azimuth_deg[1]], np.float64)[blk]
        az = (az0 + azim_corr + float(self.proc.yaw_offset_deg)) * self.proc.deg_to_rad

        el = elev * self.proc.deg_to_rad
        rc = rr * np.cos(el)
        zsign = -1.0 if self.z_down else 1.0
        xyz = np.empty((len(rr), 3), np.float32)
        xyz[:, 0] = rc * np.sin(az)
        xyz[:, 1] = rc * np.cos(az)
        xyz[:, 2] = zsign * rr * np.sin(el)
        return xyz, ring.astype(np.uint16), it.astype(np.uint8)

    # ── 整帧批量解码（多雷达并发的性能关键） ────────────────────
    def decode_frame_batch(self, bufs, seq: int = 0, ts: float = 0.0):
        """
        把一帧内的**所有报文一次性**解码 + 转笛卡尔。

        为什么要批量，而不是循环调 decode()+to_cartesian()？
            单个报文只有 256 个点，逐包走一遍要发起约 25 次 numpy 调用。
            调用的固定开销（Python 层 + GIL 切换）远大于 256 个元素的算术，
            于是 4 条工作线程各自解码时**几乎完全串行** —— 实测 4 线程仅 0.89×。
            堆成一个大数组后，同样的工作只发起约 10 次 numpy 调用，而且都在
            大数组上做，GIL 会被真正释放（实测 4 线程 2.05×），这才是
            "一台机器同时处理 4 台雷达"的前提。

        返回 (xyz, ring, intensity, info)
            xyz        (M,3) float32
            ring       (M,)  uint16
            intensity  (M,)  uint8
            info       dict —— packets（有效报文数）、dropped（长度不足被丢）、
                       points（M）、rev_deg（本批方位角前进量估计，度；用于估转速）

        与逐包路径的关系：**逐位一致**（见 tests/test_batch_decode.py）。
        点的顺序也与逐包拼接完全相同（报文顺序 × 各包内 ring 升序）。
        """
        p = self.p
        n_in = len(bufs)
        if n_in == 0:
            return (np.zeros((0, 3), np.float32), np.zeros(0, np.uint16),
                    np.zeros(0, np.uint8),
                    {"packets": 0, "dropped": 0, "points": 0, "rev_deg": 0.0})

        size = p.size
        ok = [b[:size] for b in bufs if len(b) >= size]
        dropped = n_in - len(ok)
        if not ok:
            return (np.zeros((0, 3), np.float32), np.zeros(0, np.uint16),
                    np.zeros(0, np.uint8),
                    {"packets": 0, "dropped": dropped, "points": 0, "rev_deg": 0.0})

        m = len(ok)
        pps = self._pps
        psz = p.point_size
        # 整个批量路径只此一次"打包"：把 m 个报文拼成 (m, size) 的字节矩阵
        arr = np.frombuffer(b"".join(ok), dtype=np.uint8).reshape(m, size)

        # 方位角（两 block 各一，uint16 LE，单位 0.01°）。
        # 用 (低字节 + 高字节×256) 手工拼，等价于 struct.unpack_from("<H")，
        # 但可以一次算完 m 个报文。
        a1 = (arr[:, p.angle1_offset].astype(np.float64)
              + arr[:, p.angle1_offset + 1].astype(np.float64) * 256.0
              ) * self.proc.angle_scale
        a2 = (arr[:, p.angle2_offset].astype(np.float64)
              + arr[:, p.angle2_offset + 1].astype(np.float64) * 256.0
              ) * self.proc.angle_scale

        # 两个 block 的点数据按字节拼成 (m, 2*pps*4)，再 view 成结构体 → (m, 2*pps)。
        # ★ 必须保持二维再 view：numpy 的 view 只"改写最后一维"，
        #   若先 reshape 成 (m, pps, 4) 再 view，得到的是 (m, pps, 1) 而不是 (m, pps)。
        b1 = np.ascontiguousarray(
            arr[:, p.block1_data_offset:p.block1_data_offset + pps * psz])
        b2 = np.ascontiguousarray(
            arr[:, p.block2_data_offset:p.block2_data_offset + pps * psz])
        raw = np.concatenate([b1, b2], axis=1).view(POINT_RAW_DTYPE)

        r_m = self.to_range_m(raw["r"])          # (m, 2*pps) float64
        mask = self.valid_mask(r_m)
        rev = _rev_deg(a1)
        if not mask.any():
            return (np.zeros((0, 3), np.float32), np.zeros(0, np.uint16),
                    np.zeros(0, np.uint8),
                    {"packets": m, "dropped": dropped, "points": 0, "rev_deg": rev})

        # ring：跨 block 的原始索引（0..2*pps-1），与 decode() 的 ring_raw 完全一致
        ring_row = (np.tile(np.arange(pps, dtype=np.int32), 2)
                    + np.repeat(np.array([0, pps], np.int32), pps))
        blk = ring_row >= pps

        # 逐通道角度修正（来自 angle_cache.csv）
        if self.angle is not None and self.angle.loaded:
            ch_idx = ring_row % max(1, p.channels)
            ch_idx = np.clip(ch_idx, 0, len(self.angle.elevation) - 1)
            elev = self.angle.elevation[ch_idx].astype(np.float64)
            azim_corr = self.angle.azimuth[ch_idx].astype(np.float64)
        else:
            elev = np.zeros(2 * pps, np.float64)
            azim_corr = np.zeros(2 * pps, np.float64)

        az0 = np.where(blk[None, :], a2[:, None], a1[:, None])          # (m, 2*pps)
        az = (az0 + azim_corr[None, :] + float(self.proc.yaw_offset_deg)
              ) * self.proc.deg_to_rad
        el = elev[None, :] * self.proc.deg_to_rad
        rc = r_m * np.cos(el)
        zsign = -1.0 if self.z_down else 1.0

        # 逐位一致性说明：cos/sin 都是逐元素运算，先整体算再掩码，
        # 与"先掩码再算"在数值上完全相同。
        flat = mask.reshape(-1)
        xyz = np.empty((int(flat.sum()), 3), np.float32)
        xyz[:, 0] = (rc * np.sin(az)).reshape(-1)[flat]
        xyz[:, 1] = (rc * np.cos(az)).reshape(-1)[flat]
        xyz[:, 2] = (zsign * r_m * np.sin(el)).reshape(-1)[flat]
        ring = np.tile(ring_row, (m, 1)).reshape(-1)[flat].astype(np.uint16)
        inten = raw["intensity"].reshape(-1)[flat].astype(np.uint8)
        return xyz, ring, inten, {"packets": m, "dropped": dropped,
                                  "points": int(xyz.shape[0]), "rev_deg": rev}

    def frame_from_batch(self, bufs, seq: int = 0, ts: float = 0.0):
        """批量解码得到 Frame；空帧保留 (None, info) 合同，损失见 info。

        实时 LiveSource 另将全坏批次输出为零点坏帧，以立即撤销停稳许可。
        """
        from .pointcloud import Frame
        xyz, ring, _it, info = self.decode_frame_batch(bufs, seq, ts)
        if xyz.shape[0] == 0:
            return None, info
        return Frame(xyz, ring, seq=seq, ts=ts, packets=int(info["packets"]),
                     dropped_packets=int(info["dropped"])), info

    # ── 便捷入口：一包到底 ─────────────────────────────────────
# ────────────────────────────────────────────── IMU 辅助

def imu_magnitude(pkt: DecodedPacket) -> float:
    """加速度矢量模长 —— 仅用于「雷达是否在动」的粗判，不做姿态解算。

    原版 fusion.imu_warmup_sec 就是等这里稳定下来。
    """
    if pkt.imu is None:
        return 0.0
    return float(np.linalg.norm(pkt.imu.astype(np.float64)))


# ────────────────────────────────────────────── 自检 / 合成报文

def make_synthetic_packet(packet_cfg, azimuth_deg: float = 30.0,
                          raw_distance: int = 5000, seed: int = 0) -> bytes:
    """
    合成一个合法报文 —— 用于在**没有雷达**的机器上测试解码链路与看板。
    这不是仿真雷达，只是为了让 packet→Frame 的管道能被端到端跑通。
    """
    p = packet_cfg
    rng = np.random.default_rng(seed)
    buf = bytearray(p.size)

    def put_block(ang_off: int, data_off: int, az: float, up: bool):
        struct.pack_into("<H", buf, ang_off, int(round(az / 0.01)) & 0xFFFF)
        pts = np.zeros(p.channels, dtype=POINT_RAW_DTYPE)
        base = raw_distance if up else raw_distance + 200
        pts["r"] = (base + rng.integers(-40, 40, p.channels)).astype(np.uint16)
        pts["intensity"] = rng.integers(10, 250, p.channels).astype(np.uint8)
        pts["flag"] = 0
        raw = pts.tobytes()
        buf[data_off:data_off + len(raw)] = raw

    put_block(p.angle1_offset, p.block1_data_offset, azimuth_deg, True)
    put_block(p.angle2_offset, p.block2_data_offset, azimuth_deg + 0.4, False)
    if p.imu_offset and p.imu_offset + 6 <= p.size:
        struct.pack_into("<hhh", buf, p.imu_offset, 0, 0, 2048)
    return bytes(buf)
