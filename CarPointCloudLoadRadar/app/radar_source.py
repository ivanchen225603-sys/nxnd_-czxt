#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
radar_source.py — 数据源抽象层

所有源统一接口 `read() -> Optional[Frame]`，让上层 service 完全不关心
点云是从真雷达、历史文件还是合成器来的：

    LiveSource      真雷达（UDP + PacketDecoder）—— 现场用
    LiveHub         多台雷达共用一个 UDP 端口时的**唯一**接收点（按源 IP 分流）
    ReplaySource    离线回放历史 .xyz —— 无雷达时演练 / 回归测试用
    SyntheticSource 合成空车厢 —— 演示、单测、阈值演练用
    RadarSimSource  报文级仿真 —— 整条链路同构的演示 / 回归

这个抽象是「可现场调试」的关键：现场出问题时，可以把现场录到的一段报文
丢给 ReplaySource，在办公室重现。

━━━━━━━━━━━━━━━━━ 多台雷达为什么必须走 LiveHub ━━━━━━━━━━━━━━━━━

UDP 没有"多个 socket 绑同一地址、各自收一份"的语义。
4 条通道各开一个 socket 绑 0.0.0.0:2368 时，内核只会把每个包投递给
**其中一个** socket —— 实测现象是"只有第一条通道有数据，其余永远空"，
而且不报任何错（Linux 上更直接：第 2 个 bind 就 EADDRINUSE 失败）。

所以多雷达的正确做法只有一个：
    **一个 socket 收全部 → 用 recvfrom 返回的源 IP 查表 → 分流到各通道队列**
再让每条通道的工作线程从自己的队列取包、各自解码。
LiveHub 就是干这个的；LiveSource 传 hub=... 时会自动改为从队列取包。
"""
from __future__ import annotations

import os
import time
import socket
import threading
from collections import deque, Counter
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

from .pointcloud import Frame, load_xyz, apply_roi, make_synthetic_box
from .radar_packet import PacketDecoder
from .sample_paths import discover_samples, REPLAY_NOTE

# LiveSource 的切帧方式，见 LiveSource 类文档。
#   rotation —— 按方位角回绕切帧（一帧＝一整圈），与转速无关，现场默认
#   time     —— 按 frame_ms 时间窗切帧（旧行为，仅排查时用）
FRAME_MODES = ("rotation", "time")

# 单次 recvfrom 的缓冲大小。雷达报文定长 1100B，8KB 足够有余。
# （实测与 65535 在 CPU 上没有可测差异 —— 收包开销几乎全在系统调用本身，
#   所以这里取小值只是把"预期报文大小"写明白，不是性能优化。
#   即便将来遇到更大的报文被截断，解码端只取前 packet.size 字节，也不受影响。）
RECV_BUFSIZE = 8192


@dataclass
class SourceStats:
    frames: int = 0
    points: int = 0
    packets: int = 0
    dropped: int = 0
    errors: int = 0
    started: float = field(default_factory=time.time)

    @property
    def fps(self) -> float:
        dt = max(1e-6, time.time() - self.started)
        return self.frames / dt

    def to_dict(self) -> dict:
        return {"frames": self.frames, "points": self.points, "packets": self.packets,
                "dropped": self.dropped, "errors": self.errors,
                "fps": round(self.fps, 2),
                "uptime_sec": round(time.time() - self.started, 1)}


def _throttle_sleep(interval: float, t_last: float) -> None:
    """按 interval 睡满**剩余时间**（分 0.5s 小段，避免一次长睡）。

    原来的写法只睡一次 min(剩余, 0.5)：interval 配到 0.6 秒以上时
    剩余部分被直接丢弃，回放/仿真帧率悄悄高于设定值（2026-09-23，
    两轮审查低严重度批次）。当前默认 interval=0.5 不触发，但
    settings 允许现场改大 —— 节流就得真的节满。
    """
    while True:
        remain = interval - (time.time() - t_last)
        if remain <= 0:
            return
        time.sleep(min(remain, 0.5))


class SourceBase:
    """所有数据源的基类。子类只需实现 read()。"""
    kind = "base"

    def __init__(self, roi=None, name: str = ""):
        self.roi = roi
        self.name = name or self.kind
        self.stats = SourceStats()
        self.last_error = ""
        # ★ 留住**未裁 ROI** 的那一帧（默认关闭）。
        #
        #   落盘的 .xyz 是「先裁 ROI、再做航向旋转」之后的结果，ROI 之外的东西
        #   根本不在文件里 —— 想验证"车厢有没有被 ROI 切掉"，拿它只能看
        #   "边缘离边界多远"做间接推断，而这个推断在 y 方向不可靠：
        #   车厢远端因掠射角会**看起来变窄**（2026-09-14 实测：主体宽 2.67 m，
        #   到 y=5.5 只剩 2.25 m）。有了未裁的整帧才能**直接看到框外面**。
        #
        #   只存引用、不拷贝：一帧约 20 万点 ≈ 2.3 MB，四路合计约 12 MB。
        #   关闭时 _last_raw 恒为 None，一分内存都不占。
        self.keep_raw = False
        self._last_raw: Optional[Frame] = None

    def take_raw(self) -> Optional[Frame]:
        """取走上一帧的**未裁**点云（取一次就清掉，避免上层重复存同一帧）。"""
        f, self._last_raw = self._last_raw, None
        return f

    def read(self) -> Optional[Frame]:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def _emit(self, frame: Frame) -> Frame:
        """统一出口：裁 ROI + 计数。子类产出原始帧后都过这里。"""
        n_raw = frame.n
        if self.keep_raw:
            self._last_raw = frame          # 裁之前先留一份引用
        f = apply_roi(frame, self.roi)
        self.stats.frames += 1
        self.stats.points += f.n
        # ★ ROI 保留率 = 裁剪后 / 裁剪前。这是一个**量纲自检**：
        #   俯视雷达在库房里扫的是整个空间（60m 量程），而 ROI 只是其中
        #   4m × 19m × 5.25m 的一个小盒子，正常应当把大部分回波都裁掉
        #   （实测仿真链路约 39%，历史样本一帧 5~6 万点也是这个量级）。
        #   要是保留率接近 100%，说明**整个场景都被压进了 ROI 里** ——
        #   多半是距离换算(range_mode)或安装高度不对，点云整体缩了。
        #   2026-09-13 现场那次就是：每帧 15 万点，是历史样本的 2.6 倍。
        #   这个数只报不拦：车厢把 ROI 占满时保留率也会偏高，光看它下不了结论。
        self._roi_raw = n_raw
        self._roi_kept = f.n
        return f

    def info(self) -> dict:
        d = {"kind": self.kind, "name": self.name, "stats": self.stats.to_dict(),
             "error": self.last_error}
        raw = getattr(self, "_roi_raw", 0)
        if raw:
            d["roi_keep"] = round(getattr(self, "_roi_kept", 0) / raw, 3)
            d["points_raw"] = raw
        return d


# ═══════════════════════════════════════════════ 离线回放

class ReplaySource(SourceBase):
    """
    回放一个目录里的 .xyz（递归）。

    旧样本按文件名排序，新归档每包仅取末帧原观测，按采集时间排序。
    这是独立快照演练，不复现原多帧检测或原连续停稳过程。
    """

    kind = "replay"

    def __init__(self, directory: str, interval: float = 0.5, loop: bool = True,
                 roi=None, name: str = "", pattern: str = "**/*.xyz",
                 archive_channel: Optional[str] = None):
        super().__init__(roi, name)
        self.directory = directory
        self.interval = max(0.0, float(interval))
        self.loop = loop
        found = discover_samples(directory, role="observation", pattern=pattern,
                                 archive_channel=archive_channel)
        self.archive_channel = archive_channel
        self._samples = tuple(found.samples)
        self.files: List[str] = [sample.path for sample in found.samples]
        self.discovery_errors = found.errors
        self.replay_note = REPLAY_NOTE if any(sample.archived for sample in found.samples) else ""
        self.i = 0
        self._t_last = 0.0
        if not self.files:
            self.last_error = f"目录中没有 .xyz 文件: {directory}"
        if found.errors:
            self.last_error = "; ".join(found.errors)

    def read(self) -> Optional[Frame]:
        if not self.files:
            return None
        # 节流，模拟真实帧率
        now = time.time()
        if self.interval > 0 and (now - self._t_last) < self.interval:
            _throttle_sleep(self.interval, self._t_last)
        self._t_last = time.time()

        if self.i >= len(self.files):
            if not self.loop:
                return None
            self.i = 0
        path = self.files[self.i]
        sample = self._samples[self.i]
        self.i += 1
        try:
            if sample.archived:
                from .archive_reader import load_archive
                f = load_archive(sample).role("observation").frame()
            else:
                f = load_xyz(path)
        except Exception as e:                       # noqa: BLE001 —— 现场文件可能损坏
            self.stats.errors += 1
            self.last_error = f"{os.path.basename(path)}: {e}"
            return None
        f.seq = self.stats.frames + 1
        f.ts = time.time()
        return self._emit(f)

    def info(self) -> dict:
        d = super().info()
        d.update({"dir": self.directory, "total": len(self.files),
                  "index": self.i, "interval": self.interval, "loop": self.loop,
                  "discovery_errors": list(self.discovery_errors), "replay_note": self.replay_note,
                  "archive_channel": self.archive_channel})
        return d

    def rewind(self) -> None:
        self.i = 0


# ═══════════════════════════════════════════════ 合成源

class SyntheticSource(SourceBase):
    """
    合成空车厢 / 载货车点云。

    cargo=None          → 空车（判定链应当输出「空车」）
    cargo=(w,l,h,z0)    → 车厢里放一个货堆（底面内点占比会被拉低）
    swing               → 每帧给整体加一点随机平移，用来演示停稳检测的状态变化
    """

    kind = "synthetic"

    def __init__(self, roi=None, interval: float = 0.5, size=(2.4, 12.0, 2.6),
                 cargo=None, noise: float = 0.01, swing: float = 0.0,
                 n_points: int = 60000, seed: int = 0, name: str = ""):
        super().__init__(roi, name)
        self.interval = interval
        self.size = size
        self.cargo = cargo
        self.noise = noise
        self.swing = swing
        self.n_points = n_points
        self.seed = seed
        self._rng = np.random.default_rng(seed)
        self._t_last = 0.0

    def read(self) -> Optional[Frame]:
        now = time.time()
        if self.interval > 0 and (now - self._t_last) < self.interval:
            _throttle_sleep(self.interval, self._t_last)
        self._t_last = time.time()

        f = make_synthetic_box(*self.size, n=self.n_points,
                               seed=int(self._rng.integers(0, 1 << 30)),
                               cargo=self.cargo, noise=self.noise)
        # 原版 ROI 的 z 是负的（雷达装在车厢上方，z 向下为负）
        f.xyz[:, 2] -= 5.0
        if self.swing > 0:
            d = self._rng.normal(0, self.swing, 3).astype(np.float32)
            f.xyz += d
        f.seq = self.stats.frames + 1
        f.ts = time.time()
        return self._emit(f)

    def info(self) -> dict:
        d = super().info()
        d.update({"size": self.size, "cargo": self.cargo, "swing": self.swing,
                  "n_points": self.n_points})
        return d


# ═══════════════════════════════════════════════ 仿真雷达（报文级）

class RadarSimSource(SourceBase):
    """
    用 RadarSimulator 生成报文，再用**与现场完全相同的解码器**还原成帧。

    与 SyntheticSource 的区别：
        SyntheticSource  直接造点云（跳过解码）—— 只适合单测几何/判定
        RadarSimSource   造报文再解码          —— 整条链路同构，适合演示与回归

    现场没雷达时，这是"看起来像真的在采集"的唯一办法：
    方位角会一包一包推进、点云一圈一圈刷出来、停稳检测的状态机
    也会跟着正常跳变。看板和真雷达跑起来没有区别，除了数据是假的。

    支持运行时切换场景（set_loaded）—— 演示时点一下按钮就能看到
    "空车 → 载货车 → 底面过高，疑似载物" 的完整过程。
    """

    kind = "sim-radar"

    # 默认对齐**真雷达**的方位角步长：一圈 900 个方位角（450 包 x 2）
    # -> 360/900 = 0.4 度。原默认 1.0 度比真雷达粗 2.5 倍，粗采样叠加随机丢点
    # 会让静止车的中位位置跳档 —— 那是仿真不像，不是算法不行。
    # （演示路径在 build_source 里显式传 0.2，比真雷达更密，是另一回事。）
    # ▲ 本项目为"只在合成帧上验证"吃过亏：net 蠕行判据过了所有闸门，
    #   一碰真实数据就整条撤销。仿真保真度不是小事。
    def __init__(self, decoder, sim, roi=None, az_step_deg: float = 0.4,
                 packets_per_frame: int = 0, interval: float = 0.5,
                 z_floor: float = -5.0, size=(2.4, 12.0, 2.6),
                 loaded: bool = False, swing: float = 0.0, name: str = "",
                 ground: bool = False):
        super().__init__(roi, name)
        from .radar_sim import RadarSimulator, make_demo_scene   # 延迟 import
        self.dec = decoder
        self.sim: RadarSimulator = sim
        self.sim.az_step_deg = az_step_deg
        # 0 = 整圈。必须扫满一圈，否则一帧只有几百~几千点，
        # 会直接被判定链第一关挡掉（「点云数量不足」，要求 ≥20000 点）。
        self.packets_per_frame = (int(round(360.0 / max(1e-6, az_step_deg)))
                                  if packets_per_frame <= 0 else max(1, packets_per_frame))
        self.interval = interval
        self.z_floor = z_floor
        self.size = size
        self.loaded = loaded
        self.swing = swing
        # 是否铺一块地面。演示/自动编排时必须开，否则"车不在"时
        # 一帧点都没有 → read() 返回 None → 看板那条通道僵住不动。
        # 详见 radar_sim.box_ground 的说明。
        self.ground = bool(ground)
        self._ground_box = None
        if self.ground:
            from .radar_sim import box_ground
            self._ground_box = box_ground(z_floor=z_floor)
        # ★ 车辆沿 y 的位置偏移（米）。0 = 停在检测位正中。
        #   演示编排器把它从 0 推到 ±20 再拉回来，就得到"开进来 / 开出去"。
        #   故意**不**关联 generation：停稳检测本来就是比对逐帧质心，
        #   车在移动时质心自然会跑、deviation 变大 → 状态自动变「运动中」。
        #   若改成每次移动都 bump generation，反而会把参考帧一直清掉，
        #   看上去"永远刚从起点出发"，不像真车。
        self._veh_offset = 0.0
        self._az = 0.0
        self._t_last = 0.0
        self.generation = 0
        self._rng = np.random.default_rng(1234)
        # ★ 场景与代次必须**成对原子交换**。
        #   曾经把它拆成 self._boxes 和 self.generation 两个独立属性：
        #   read() 先取 boxes、后取 generation，而生成一帧要点时间（~0.12s），
        #   切换恰好夹在中间时，就会得到"旧场景数据 + 新代次"的组合 →
        #   上层以为这是新场景而重置参考帧，下一帧又变成新场景 → 偏差恒 0.7m。
        self._scene = (0, make_demo_scene(loaded=loaded, z_floor=z_floor,
                                          dx=size[0], dy=size[1], dz=size[2]))

    @property
    def boxes(self):
        return self._scene[1]

    def set_vehicle(self, loaded=None, y_offset=None) -> bool:
        """
        运行时控制"这辆车"：换车（空车/载货）和/或挪动位置。

        这是自动演示（demo_director）的基础动作：
            set_vehicle(y_offset=+20)   → 车已经开出去了
            set_vehicle(y_offset=0)     → 车停在检测位
            set_vehicle(loaded=True)    → 换成一车货（换车，会 bump generation）

        两种参数语义**故意不同**：
          · loaded 变了 = 换了一辆车，场景语义变了 → bump generation，
            让停稳检测重建参考帧（否则新旧车的质心差会被当成"还在动"）；
          · y_offset 变了 = 同一辆车在动 → **不动** generation，
            交给停稳检测按质心自己去判"运动中"，这才是真实物理过程。
        返回场景是否实际变化，供控制层即时撤销旧停稳许可；重复同姿态不清窗。
        """
        previous = (self.generation, self._veh_offset)
        if loaded is not None and bool(loaded) != bool(self.loaded):
            self.set_loaded(bool(loaded))
        if y_offset is not None:
            self._veh_offset = float(y_offset)
        return previous != (self.generation, self._veh_offset)

    def set_loaded(self, loaded: bool) -> None:
        """
        运行时切换"空车 / 载货"场景。

        自增 generation（与场景打包成元组原子替换）——
        上层（service）据此判断"这一帧属于新场景"，再去重置停稳检测器。
        **不能在这里直接重置**：切换瞬间可能还有一帧旧场景在路上，
        先重置会把旧场景的质心当成新参考帧，接着新场景的质心差 0.7m
        → 判定永远停在"运动中"。（这个坑实测踩过两次，见 service._tick。）
        """
        from .radar_sim import make_demo_scene
        self.loaded = bool(loaded)
        self.generation += 1
        self._scene = (self.generation,
                       make_demo_scene(loaded=self.loaded, z_floor=self.z_floor,
                                       dx=self.size[0], dy=self.size[1], dz=self.size[2]))

    def read(self) -> Optional[Frame]:
        now = time.time()
        if self.interval > 0 and (now - self._t_last) < self.interval:
            _throttle_sleep(self.interval, self._t_last)
        self._t_last = time.time()

        # ★ 原子取"场景 + 代次"：见 __init__ 的说明
        gen, boxes_raw = self._scene
        boxes = list(boxes_raw)
        off = self._veh_offset
        if self.swing > 0:
            # 让整个场景沿 y 轻微漂移，模拟车辆仍在移动 —— 停稳检测会显示"运动中"
            off = off + float(self._rng.normal(0, self.swing))
        if off != 0.0:
            # 车辆整体平移（车厢 + 货堆一起动）——
            # 移出 ROI 后高于地面的点数会掉到阈值以下，停稳检测自然回到「空闲」。
            from .radar_sim import Box
            boxes = [Box((b.lo[0], b.lo[1] + off, b.lo[2]),
                         (b.hi[0], b.hi[1] + off, b.hi[2]),
                         b.hollow) for b in boxes]
        # 地面最后加，且**不参与平移** —— 地面是固定的，车在它上面跑
        if self._ground_box is not None:
            boxes.append(self._ground_box)

        # ★ 整圈批量生成 + 整帧批量解码。
        #   逐包版（`_one_packet_with` + `decode`）在 4 路并发时会被 GIL 拖到
        #   1/6 速度（实测 360 包 0.111s → 0.68s），车看上去是一帧一帧跳的。
        #   批量版把 720 个方位角的所有激光合成一次大数组运算（大数组会释放
        #   GIL，4 路才真的并行），解码走 decode_frame_batch（与 live 路径同一个
        #   函数，逐位一致）。详见 radar_sim._dirs_many 与 radar_packet.decode_frame_batch。
        step = self.sim.az_step_deg
        az0 = self._az % 360.0
        bufs = self.sim.scan_packets(boxes, az0, step, self.packets_per_frame)
        self._az += step * len(bufs)
        if not bufs:
            return None
        self.stats.packets += len(bufs)
        xyz, ring, _it, info = self.dec.decode_frame_batch(bufs, self.stats.frames + 1,
                                                           time.time())
        if info.get("dropped"):
            self.stats.dropped += info["dropped"]
            self.last_error = f"本帧 {info['dropped']} 个报文长度不足，已丢弃"
        if xyz.shape[0] == 0 and not info.get("dropped"):
            return None
        f = Frame(xyz, ring, seq=self.stats.frames + 1, ts=time.time(), gen=gen,
                  dropped_packets=int(info.get("dropped", 0)))
        return self._emit(f)

    def info(self) -> dict:
        d = super().info()
        d.update({"loaded": self.loaded, "az_deg": round(self._az % 360.0, 1),
                  "packets_per_frame": self.packets_per_frame,
                  "swing": self.swing, "size": self.size,
                  "generation": self.generation,
                  "ground": self.ground,
                  "veh_offset": round(self._veh_offset, 3)})
        return d


# ═══════════════════════════════════════════════ 工厂

def build_source(ch, calib, angle_cache, mode: Optional[str] = None,
                 demo_loaded: bool = False, hub: Optional["LiveHub"] = None):
    """
    按通道配置造对应的数据源。service 与 CLI 都走这里，保证口径一致。

    mode 优先级：显式入参 > ch.mode
        replay   → ReplaySource(ch.abs_replay_dir or ch.abs_output_dir)
        live     → LiveSource（hub 传入时走共享套接字 + 源 IP 分流）
        simulate → RadarSimSource（无硬件演示）
        synthetic→ SyntheticSource（纯点云，跳过解码）

    hub：多台雷达必须传入同一个 LiveHub。传 None 时 LiveSource 自开 socket
         （单台雷达 / 老配置兼容）。详见 LiveHub 的说明。
    """
    from .radar_packet import PacketDecoder
    m = (mode or ch.mode or "replay").lower()
    roi = calib.roi
    lid = ch.lidar

    # ★ 逐通道方位角微调：四条车道并不平行，共用的 processing.yaw_offset_deg
    #   只能对齐其中一条。lidar.yaw_offset_deg 给这一路单独叠加一个角度。
    #   留空（None）= 不叠加，行为与从前完全一致。
    #   为什么必须补：2026-09-14 实测 CH3 的车一致斜停 4.25°~6.00°，
    #   车厢最近只差 0.01 m 就被 ROI 切掉，而车厢被切会直接少算体积。
    #   注意**只改这一路解码用的副本**，共用的 calib 一个字不动 ——
    #   否则四路会互相踩。
    dyaw = getattr(lid, "yaw_offset_deg", None)
    if dyaw is not None:
        import copy as _copy
        calib = _copy.copy(calib)
        calib.processing = _copy.copy(calib.processing)
        calib.processing.yaw_offset_deg = float(dyaw)
        roi = calib.roi

    if m in ("live", "udp", "real"):
        dec = PacketDecoder(calib.packet, calib.processing, angle_cache,
                            range_mode=lid.range_mode, z_down=lid.z_down,
                            max_range_m=lid.max_range_m,
                            # 原厂 config.json 里 processing.min_distance_mm 就是近场盲区，
                            # 之前没接线、一直用默认的 0.1m（恰好等于 100mm，纯属巧合）。
                            min_range_m=float(calib.processing.min_distance_mm) / 1000.0)
        # source_ip = 该通道雷达的源地址（多雷达共用端口时按它分流）。
        # 兼容旧配置：没填 source_ip 就退回 lidar.ip。
        src = getattr(lid, "source_ip", "") or lid.ip or ""
        return LiveSource(dec, bind_ip=lid.bind_ip or calib.network.bind_ip,
                          udp_port=lid.udp_port, frame_ms=lid.frame_ms, roi=roi,
                          name=f"{ch.id} live@{src}", hub=hub, source_ip=src,
                          frame_mode=getattr(lid, "frame_mode", "rotation"))

    if m in ("simulate", "sim", "sim-radar"):
        from .radar_sim import RadarSimulator
        dec = PacketDecoder(calib.packet, calib.processing, angle_cache,
                            range_mode=lid.range_mode, z_down=lid.z_down,
                            max_range_m=lid.max_range_m,
                            # 原厂 config.json 里 processing.min_distance_mm 就是近场盲区，
                            # 之前没接线、一直用默认的 0.1m（恰好等于 100mm，纯属巧合）。
                            min_range_m=float(calib.processing.min_distance_mm) / 1000.0)
        # 演示加密射线，避免粗采样叠加随机丢点让静止车的中位位置跳档。
        # 保留测距噪声和丢点；此处仅为仿真采样，不代表现场雷达参数。
        sim = RadarSimulator(calib.packet, calib.processing, angle_cache,
                             range_mode=lid.range_mode, z_down=lid.z_down,
                             az_step_deg=0.2, range_noise_m=0.004,
                             dropout_prob=0.01, seed=7)
        # ★ ground=True：铺一块地面。
        #   没有它，"车开走"后一帧点都没有 → read() 返回 None → 看板那条通道
        #   整个僵住不动（像挂了）。有地面后空车道仍有上万点 → 帧不为空 →
        #   状态持续刷新；而地面点在 z=-5.0，被"有车"判据（z > -4.7）排除，
        #   所以状态正确停在「空闲」。详见 radar_sim.box_ground。
        return RadarSimSource(dec, sim, roi=roi, loaded=demo_loaded,
                              az_step_deg=sim.az_step_deg,
                              interval=calib.detection.detect_interval,
                              ground=True,
                              name=f"{ch.id} 仿真雷达")

    if m in ("synthetic", "syn"):
        # ★ 第四个分量是货堆在**箱体坐标**里的起高（地板 z=0），不是世界坐标。
        #   写 -5.0 的话：make_synthetic_box 先把货堆放到 z∈[-5.03, -3.86]，
        #   read() 再整体 z -= 5.0 -> 最终 z∈[-10.03, -8.86]，而 ROI 的 z 窗口
        #   是 [-6, -0.75] —— **5 万个货堆点全被裁掉**，演示的"载货"车道
        #   必然判成空车（实测 ROI 内 60 000/110 000 点，正好少了货堆那一份）。
        #   取 0.0 = 货堆坐在地板上，平移后 z∈[-5.04, -3.86]，全部落在 ROI 内。
        return SyntheticSource(roi=roi, interval=calib.detection.detect_interval,
                               cargo=(2.28, 10.2, 1.17, 0.0) if demo_loaded else None,
                               name=f"{ch.id} 合成点云")

    d = ch.abs_replay_dir or ch.abs_output_dir
    return ReplaySource(d, interval=ch.replay_interval, loop=ch.replay_loop,
                        roi=roi, name=f"{ch.id} 回放", archive_channel=ch.id)


# ═══════════════════════════════════════════════ 多雷达分流（单套接字）

class ChannelQueue:
    """
    一条通道的报文队列 —— LiveHub（生产者）与 ChannelWorker（消费者）的唯一交接点。

    为什么用 Condition 而不是忙轮询？
        现场"4 个车道可能同时来车"，但更多时候只有 1~2 路有车。
        空闲通道若用 while + sleep 空转，4 条线程会白白抢 CPU，
        反而拖慢正在做六面拟合的那一路。这里消费者没有报文时阻塞在 wait()，
        CPU 占用为 0，来包时由 notify() 立刻唤醒。

    溢出策略：丢**最旧**的。worker 卡在一次检测里（六面拟合可能几百 ms）时
    队列会积压，保留最新的那一段才有意义（旧数据会拖出一帧重影）。
    """

    def __init__(self, maxlen: int = 8192):
        self.maxlen = int(maxlen)
        self._dq: Deque[Tuple[bytes, float]] = deque()
        self._cv = threading.Condition()
        self.pushed = 0
        self.dropped = 0

    def push(self, item: Tuple[bytes, float]) -> None:
        with self._cv:
            if self.maxlen and len(self._dq) >= self.maxlen:
                self._dq.popleft()
                self.dropped += 1
            self._dq.append(item)
            self.pushed += 1
            self._cv.notify()

    def drain(self, timeout: float = 0.05) -> List[Tuple[bytes, float]]:
        """取走当前所有报文；队列为空时最多阻塞 timeout 秒再返回空表。"""
        with self._cv:
            if not self._dq:
                self._cv.wait(timeout)
            if not self._dq:
                return []
            items = list(self._dq)
            self._dq.clear()
            return items

    def qsize(self) -> int:
        with self._cv:
            return len(self._dq)


class RateWindow:
    """滑动窗口速率估计 —— 给「包率 / 转速」这类**现场诊断**用。

    ★ 为什么不能用「总数 / 运行时长」

    累计平均对现场排障是**有害**的，而且恰恰在最需要它的时候骗人：
    通道掉线一段时间再恢复，那段空窗会永久压低平均值 ——
    看板上一条明明已经满速的链路，会持续显示成只有两三成包率，
    看起来就像在丢包。2026-09-14 现场就这样误判过一次
    （三路恢复后报 2 096 包/秒、转速 139 rpm，实测其实是 8 980 / 600）。

    做法：每次取值时记一个 (时刻, 累计数) 样本，只留最近 window 秒，
    用「最老的样本到现在」算速率。样本只在 info() 被调用时产生，
    接收线程一个字都不多做（见 LiveHub 的说明：它刻意只做查表 + 入队）。
    """

    __slots__ = ("window", "min_span", "_pts", "_lock")

    def __init__(self, window: float = 10.0, min_span: float = 1.0):
        self.window = float(window)
        self.min_span = float(min_span)
        self._pts = deque()                  # [(t, 累计值), …]
        self._lock = threading.Lock()

    def rate(self, total: float, started: float,
             now: Optional[float] = None) -> float:
        """喂进当前累计值，取回速率（单位/秒）。

        样本还不够一个 min_span 时退回「总数 / 运行时长」——
        刚启动的头一两秒本来也只有这个口径可用。
        """
        # 不能写 `now or time.time()` —— now=0.0 是合法时刻却是假值，
        # 会悄悄混进真实时钟（单测里就这么炸过）。
        if now is None:
            now = time.time()
        with self._lock:
            self._pts.append((now, float(total)))
            cut = now - self.window
            # 逐出老样本，但**必须留够 min_span 的跨度**：
            # 只剩两个挨得很近的样本时会退回累计口径（见下），
            # 而那正是这个类要修掉的东西 —— 流停了反而报出历史平均。
            keep = now - self.min_span
            while (len(self._pts) > 2 and self._pts[0][0] < cut
                   and self._pts[1][0] <= keep):
                self._pts.popleft()
            t0, v0 = self._pts[0]
            span = now - t0
            if span >= self.min_span:
                return max(0.0, (float(total) - v0) / span)
        up = max(1e-6, now - started)
        return float(total) / up



class LiveHub:
    """
    多台雷达共用一个 UDP 端口时的**唯一**接收点：一个 socket + 按源 IP 分流。

    ── 为什么非这样不可 ────────────────────────────────────────────
    UDP 没有"多个 socket 绑同一地址、各自收一份"的语义。若每条通道各开一个
    socket 绑 0.0.0.0:2368：
        · Windows（SO_REUSEADDR 让 bind 都成功）→ 所有包只落到其中一个 socket，
          现象是"只有 CH1 有数据，其余永远空"，**不报错**；
        · Linux → 第 2 个 bind 直接 EADDRINUSE 失败。
    两条路都不通。唯一正确的做法就是本类：一个 socket 收全部，
    再用 recvfrom 返回的**发送方 IP** 查表分流。

    ── 线程模型 ────────────────────────────────────────────────────
        1 个接收线程（本类）
            ↓ 只做"查表 + 入队"，纯 dict/deque 操作（实测 1190 万次/秒）
        N 个 ChannelQueue（按 source_ip 划分）
            ↓
        N 个 ChannelWorker（各自解码 + 检测 + 判定，互不阻塞）

    接收线程刻意**不做解码**：解码才是重活，必须分散到 N 条通道线程上去，
    否则又变回单核串行。这样设计后，4 路同时来车也是各自线程并行处理。

    ── 未登记的源 ──────────────────────────────────────────────────
        strict=True（默认）：丢弃并计数（防串扰，看板上能看出来谁在乱发）
        strict=False       ：交给 register() 时传入的默认队列
          —— 但多雷达场景建议保持 strict，否则无法判断是否接错网段。
    """

    def __init__(self, bind_ip: str = "0.0.0.0", udp_port: int = 2368,
                 recv_buf: int = 32 << 20, queue_max: int = 8192,
                 strict: bool = True, name: str = "live-hub"):
        self.bind_ip = bind_ip or "0.0.0.0"
        self.udp_port = int(udp_port)
        self.recv_buf = int(recv_buf)
        self.queue_max = int(queue_max)
        self.strict = bool(strict)
        self.name = name
        self.sock: Optional[socket.socket] = None
        self.last_error = ""
        # ★ 接收线程"进程内部挂了"的标记，与"网络没数据"必须分开 ——
        #   这是全链路唯一的接收点，它一死四路同时变哑，而看板上的表现
        #   与拔网线、与雷达断电完全一样。见 docs/52 第二节。
        self.dead = False
        self._queues: Dict[str, ChannelQueue] = {}
        self._owners: Dict[str, str] = {}          # source_ip -> 通道名（仅用于报错/展示）
        self._default_q: Optional[ChannelQueue] = None
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.started = 0.0
        # 统计（看板 / 现场排查用）
        # ★ 速率一律走滑动窗口，不用「总数 / 运行时长」：
        #   通道掉线再恢复时，累计平均会被那段空窗永久压低，
        #   把一条已经满速的链路显示成在丢包。见 RateWindow。
        self._rate_all = RateWindow()
        self._rate_ip: Dict[str, RateWindow] = {}
        self.recv_count = 0
        self.nbytes = 0
        self.unknown = 0
        self.errors = 0
        self.unknown_ips: Counter = Counter()
        self.per_ip: Counter = Counter()

    # ── 注册通道 ────────────────────────────────────────────────
    def register(self, source_ip: str, channel: str = "",
                 default: bool = False) -> ChannelQueue:
        """给某台雷达（按源 IP）注册一个队列。IP 重复会直接报错 —— 那是配置错误。"""
        ip = (source_ip or "").strip()
        if not ip:
            raise ValueError(
                "LiveHub 需要每台雷达的 source_ip —— 请在 config/channels.json 的 "
                "lidar.source_ip（或 lidar.ip）里填上该通道雷达的 IP，"
                "例如 198.51.100.1。")
        with self._lock:
            if ip in self._queues:
                raise ValueError(
                    f"源 IP {ip} 被重复注册：通道 {self._owners.get(ip)!r} 与 {channel!r} "
                    f"用了同一个 source_ip。多雷达共用端口时必须一机一 IP。")
            q = ChannelQueue(self.queue_max)
            self._queues[ip] = q
            self._owners[ip] = channel or ip
            if default:
                self._default_q = q
            return q

    def ensure(self, source_ip: str, channel: str = "",
               default: bool = False) -> ChannelQueue:
        """幂等注册：已登记过就返回原队列。

        用途：service 会**先**替所有 live 通道登记（这样某台雷达没上线时，
        看板上能明确显示"它的队列一个包都没有"，而不是整条通道凭空消失）；
        随后 LiveSource 再认领时走的就是这里，不会因"重复注册"报错。
        """
        ip = (source_ip or "").strip()
        with self._lock:
            if ip in self._queues:
                return self._queues[ip]
        return self.register(ip, channel, default)

    # ── 生命周期 ────────────────────────────────────────────────
    def open(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.recv_buf)
        except OSError:
            pass
        try:
            s.bind((self.bind_ip, self.udp_port))
        except OSError as e:
            # 显式关掉：CPython 引用计数会兜底，但别赌它 —— 端口占用排查
            # 期间反复重试 open，多挂一个句柄就多一分干扰（低严重度批次）。
            s.close()
            self.last_error = (
                f"LiveHub 无法绑定 {self.bind_ip}:{self.udp_port} —— {e}。"
                f"检查端口是否被占用（另一个 run.py / capture.py 还开着？）"
                f"或是否有权限（<1024 需 root）。")
            raise
        s.settimeout(0.05)
        self.sock = s

    def start(self) -> None:
        # ★ 重入保护（2026-09-23，低严重度批次）：二次 start 会起第二个
        #   _run 线程，两个线程各自 recvfrom 同一 socket，局部 recv_count
        #   互相覆盖（received/pkt_rate 少报一半）、共享 Counter 全量累加
        #   （per_ip 又是全量）—— 两个口径对不上，排障会被误导。
        thread = self._thread
        if thread is not None and thread.is_alive():
            raise RuntimeError("LiveHub 接收线程已在运行，禁止重复 start()")
        if self.sock is None:
            self.open()
        self.started = time.time()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name=self.name)
        self._thread.start()

    def _run(self) -> None:
        """接收线程入口：只负责**兜住** `_recv_loop` 漏出来的任何异常。

        为什么要这一层：`_recv_loop` 只兜 `socket.timeout` 与 `OSError`，
        其余任何异常（MemoryError、队列对象被换掉后的 AttributeError…）
        都会让线程**静默退出**。而这是全链路唯一的接收点 —— 它一死，
        四条通道同时不再收到任何包，看板上却只显示"等待雷达数据…"，
        与拔网线、与雷达断电**完全无法区分**。

        `app/` 里 plate / demo_director / result_finalizer / load_radar_bridge /
        service 的线程都有 `is_alive()` 检查，唯独这里没有。见 docs/52 第二节。

        ▲ 只加在**循环外面**：Python 3.11 的零开销异常让未触发的 try 不产生
          运行时代价，而这个循环每秒要跑 3.6 万次，任何每轮开销都不可接受。
        """
        try:
            self._recv_loop()
        except BaseException as exc:                          # noqa: BLE001
            # 记下来就好，不再抛 —— 这是 daemon 线程，抛出去只会打到
            # stderr 然后消失，反而不如留在快照里让看板/api 能读到。
            self.dead = True
            self.last_error = ("接收线程异常退出: %s: %s —— 四条通道都不会再收到包，"
                               "必须重启服务。" % (type(exc).__name__,
                                                 " ".join(str(exc).split())[:160]))

    def _recv_loop(self) -> None:
        """收包线程：**只做查表 + 入队**，解码交给各通道线程（见类说明第 1 条）。

        ★ 这个循环是整条链路唯一的串行瓶颈
          4 台雷达 × 约 9 000 包/秒 = **每秒 3.6 万次**循环，而解码线程同时在
          抢 GIL。热路径上每一次属性查找都要乘以 3.6 万 —— 所以下面把
          self.xxx / 全局名字全部在循环外绑成局部变量（局部变量取值是
          LOAD_FAST，比 LOAD_ATTR 的字典查找快得多），并且把计数改成
          局部累加、只在每包末尾回写一次。

          2026-09-14 现场：四路同时在线时丢了约 28%（每圈包数从 900 掉到
          490~645），而 queue_dropped 与 errors 全是 0 —— 说明包是在**进到
          我们的队列之前**就被内核丢掉的，即这个循环来不及排空套接字缓冲。
          光靠加大 SO_RCVBUF 只能推迟溢出，排空速度才是根本。
        """
        sock = self.sock
        recvfrom = sock.recvfrom
        bufsize = RECV_BUFSIZE
        queues = self._queues
        stop_set = self._stop.is_set
        now = time.time
        timeout_exc = socket.timeout
        strict = self.strict
        recv_count = self.recv_count
        nbytes = self.nbytes
        per_ip = self.per_ip
        while not stop_set():
            try:
                data, addr = recvfrom(bufsize)
            except timeout_exc:
                # 空闲即回写：小流量下（单测常常只发几个包）不能让计数卡在
                # 攒够 128 个才露面。超时是 50ms 一次，足够及时。
                if self.recv_count != recv_count:
                    self.recv_count, self.nbytes = recv_count, nbytes
                continue
            except OSError as e:
                if stop_set():
                    break
                self.errors += 1
                self.last_error = f"recvfrom 失败: {e}"
                time.sleep(0.1)
                continue
            ip = addr[0]
            q = queues.get(ip)
            if q is None:
                # 冷路径：未登记的源。这里怎么慢都无所谓，别拖累上面的热路径。
                self.recv_count, self.nbytes = recv_count, nbytes
                if self.last_error:
                    self.last_error = ""
                self.unknown += 1
                if ip in self.unknown_ips or len(self.unknown_ips) < 64:
                    self.unknown_ips[ip] += 1
                if strict:
                    continue
                q = self._default_q
                if q is None:
                    continue
            q.push((data, now()))
            recv_count += 1
            nbytes += len(data)
            per_ip[ip] += 1
            # 统计只在整百包时回写：看板取值是 2Hz，这点粒度看不出来，
            # 但省掉了每包三次属性写。
            if not recv_count % 128:
                self.recv_count, self.nbytes = recv_count, nbytes
                if self.last_error:
                    self.last_error = ""   # 收到包了就说明恢复了
        self.recv_count, self.nbytes = recv_count, nbytes

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    # ── 查询 ────────────────────────────────────────────────────
    def alive(self) -> bool:
        """接收线程是不是还在跑。没 start() 过、已 close()、或异常退出都是 False。"""
        thread = self._thread
        return bool(thread is not None and thread.is_alive() and not self.dead)

    def source_ips(self) -> List[str]:
        return list(self._queues.keys())

    def to_dict(self) -> Dict:
        return {
            "bind": f"{self.bind_ip}:{self.udp_port}",
            "strict": self.strict,
            # ★ "线程还在不在"必须与"有没有包"分开看：两者都表现为收不到数据，
            #   但一个要重启服务、一个要查网线。alive=False 时 error 里有原因。
            "alive": self.alive(),
            "received": self.recv_count,
            "pkt_rate": round(self._rate_all.rate(self.recv_count, self.started), 1),
            "mb": round(self.nbytes / 1e6, 2),
            "unknown": self.unknown,
            "unknown_ips": dict(self.unknown_ips.most_common(8)),
            "errors": self.errors,
            "error": self.last_error,
            "sources": {
                ip: {
                    "channel": self._owners.get(ip, ip),
                    "pkts": int(self.per_ip.get(ip, 0)),
                    "pkt_rate": round(self._rate_ip.setdefault(ip, RateWindow())
                                      .rate(int(self.per_ip.get(ip, 0)), self.started), 1),
                    "queue_pending": q.qsize(),
                    "queued_total": q.pushed,
                    "queue_dropped": q.dropped,
                }
                for ip, q in self._queues.items()
            },
        }


# ═══════════════════════════════════════════════ 实时 UDP

class LiveSource(SourceBase):
    """
    真雷达数据源：取 UDP 报文 → 解码 → 按时间窗聚合成一帧。

    两种取包方式（由是否传 hub 决定）
        hub=None（单台雷达）  自己开一个 socket 收包。原行为，保持兼容。
        hub=LiveHub（多台雷达）从 hub 分给自己的队列取包 —— **不能自己开 socket**，
                             否则 4 条通道会一起抢 0.0.0.0:2368（见模块头部的说明）。

    聚合策略（frame_mode）
        "rotation"（默认，**现场应当用这个**）
            按**方位角回绕**切帧：报文里的方位角从 359.x° 跳回 0.x° 就是转完一圈，
            这一圈的报文合成一帧。不需要知道转速，雷达调速也不用改配置。
        "time"（旧行为，排查时才用）
            每 `frame_ms` 毫秒把这段时间收到的报文合并成一帧。
            ★ 此时 frame_ms 必须约等于**转一圈的时长**（＝ 1000 / 转速Hz）。

        ★ 为什么默认改成按整圈切帧（2026-09 现场问题）
          按时间切帧时，时间窗和转圈**不同步**，于是：
            · frame_ms 比一圈短 → 每帧只有一片扇面，而且扇面位置每帧都在转，
              看板上点云"变化极快、根本看不清"；
            · 每帧的点集完全不同 → 质心跟着乱跳 → StabilityDetector 的
              dev（‖质心差‖）忽大忽小 → 状态在"运动中/已停稳"之间每帧乱翻；
            · 点数也跟着忽多忽少，可能一直够不到 min_save_points 而不落盘。
          按整圈切帧后，每帧覆盖的方位角完全相同，静止场景的帧间质心差
          只剩测量噪声（约 1e-4 m 量级），上面三个症状一起消失。
          历史样本每帧 5~6 万点（ROI 内），可作为"一帧是否就是一圈"的现场对照。

        兜底：`rotation` 模式下若连续 `rotation_timeout_ms`（默认 1000ms）都没
        检测到回绕（方位角字段解析不对 / 雷达没转），自动退回按时间切帧并计数，
        info() 的 frames_by_timeout 会一直涨 —— 现场看到它涨就知道该查方位角了。

    丢包防护
        worker 卡在一次检测里时（六面拟合可能几百 ms），队列会积压好几圈报文。
        _flush_window() 只取「最近 frame_ms×1.25」窗口内的报文，把更早的丢掉 ——
        否则多圈会叠进同一帧，点云出现**重影**。

    绑定地址（仅 hub=None 时有意义）
        bind_ip 留空 → 绑 0.0.0.0，收所有。多雷达请统一交给 LiveHub。
    """

    kind = "live"

    def __init__(self, decoder: PacketDecoder, bind_ip: str = "0.0.0.0",
                 udp_port: int = 2368, frame_ms: int = 100, roi=None,
                 recv_buf: int = 8 << 20, name: str = "", save_raw_dir: str = "",
                 hub: Optional["LiveHub"] = None, source_ip: str = "",
                 frame_mode: str = "rotation", rotation_timeout_ms: int = 1000,
                 max_buffer_packets: int = 8192):
        super().__init__(roi, name)
        self.decoder = decoder
        self.bind_ip = bind_ip or "0.0.0.0"
        self.udp_port = int(udp_port)
        self.frame_ms = max(10, int(frame_ms))
        # 不认识的模式**启动即报错**，不静默退回 —— 与 range_mode 同一套规矩：
        # 打错一个字母就悄悄换一种切帧方式，现象是"点云怪但不报错"，最难查。
        mode = (frame_mode or "rotation").strip()
        if mode not in FRAME_MODES:
            raise ValueError(f"未知的 frame_mode: {frame_mode!r}。可用值："
                             f"{', '.join(sorted(FRAME_MODES))}")
        self.frame_mode = mode
        self.rotation_timeout_ms = max(self.frame_ms, int(rotation_timeout_ms))
        self.max_buffer_packets = max(64, int(max_buffer_packets))
        self.save_raw_dir = save_raw_dir
        self.hub = hub
        self.source_ip = (source_ip or "").strip()
        self.sock: Optional[socket.socket] = None
        self.queue: Optional[ChannelQueue] = None
        self._buf: List[bytes] = []
        self._buf_ts: List[float] = []
        self._t_window = time.time()
        self._seq = 0
        self._raw_fp = None
        self._rev_deg = 0.0                      # 累计方位角前进量（估转速用）
        # 同上：包率与转速都走滑动窗口
        self._rate_pkts = RateWindow()
        self._rate_rev = RateWindow()
        # ── 按整圈切帧用的状态 ──
        self._az: List[int] = []                 # 与 _buf 等长：每包的原始方位角（0.01°）
        self._prev_az: Optional[int] = None      # 上一帧末尾那包的方位角，跨帧判回绕
        self._rev_packets = 0                    # 最近一帧包含多少个报文（≈一圈的包数）
        self._by_rotation = 0                    # 按回绕切出来的帧数
        self._by_timeout = 0                     # 兜底按时间切出来的帧数
        self._last_timeout_t = 0.0               # 最近一次走兜底的时刻
        self._trimmed = 0                        # 缓冲区超限被丢掉的报文数
        self._last_emit = time.time()
        self._emit_dt = 0.0                     # 最近一次出帧间隔（秒）= 一圈的时长
        # ── 队列溢出观察（两轮审查 M7）：溢出意味着消费者（通道线程）停了 ——
        #    解码层的丢包会写 last_error，队列溢出原来一个字不出，而它恰恰是
        #    "worker 卡住了"最该被看见的信号。只记"最近还在涨"的时刻。
        self._queue_dropped_seen = 0
        self._queue_drop_t = 0.0

        if hub is not None:
            # 多雷达：从共享 hub 领一个只属于本通道的队列，不再自己 bind
            self.queue = hub.ensure(self.source_ip, name or self.source_ip)
            if save_raw_dir:
                self.last_error = ("hub 模式下不做原始存盘（4 路共用 socket）；"
                                   "现场抓包请用 cli/capture.py --split-by-ip")
        else:
            self.open()

    def open(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        except OSError:
            pass
        try:
            s.bind((self.bind_ip, self.udp_port))
        except OSError as e:
            s.close()                         # 同 LiveHub.open：失败即关，不赌 GC
            self.last_error = (f"无法绑定 {self.bind_ip}:{self.udp_port} —— {e}。"
                               f"检查端口是否被占用 / 是否有权限（<1024 需 root）。"
                               f"若这是多雷达场景，请确认走的是 LiveHub（不应各自 bind）。")
            raise
        s.settimeout(0.05)                     # 短超时，保证 read() 能被 Ctrl-C 打断
        self.sock = s
        if self.save_raw_dir:
            os.makedirs(self.save_raw_dir, exist_ok=True)
            p = os.path.join(self.save_raw_dir, time.strftime("raw_%Y%m%d_%H%M%S.bin"))
            self._raw_fp = open(p, "wb")

    # ── 取包 ────────────────────────────────────────────────
    def _collect(self) -> None:
        """把"新到的"报文放进 _buf/_buf_ts。两种模式在这里分叉。"""
        if self.queue is not None:
            items = self.queue.drain(0.05)
            if items:
                for data, ts in items:
                    self._buf.append(data)
                    self._buf_ts.append(ts)
                self.stats.packets += len(items)
            return

        if self.sock is None:
            return
        got = 0
        while got < 4096:
            if self.frame_mode == "rotation":
                # 攒够一小批就回去判回绕。不能沿用下面的"时间窗满了就停"——
                # rotation 模式下 _t_window 要等出帧才推进，那个条件会退化成
                # "每次 read() 只收一个包"。128 包在 4500 包/s 下约 28ms，
                # 既不会一次吞掉好几圈，也不会把 read() 调用次数放大。
                if got >= 128:
                    break
            elif got and (time.time() - self._t_window) * 1000.0 >= self.frame_ms:
                # 窗口已满就停手，避免一次 read() 把好几圈都吞进来
                break
            try:
                data, _addr = self.sock.recvfrom(4096)
            except socket.timeout:
                break
            except OSError as e:
                self.stats.errors += 1
                self.last_error = f"recvfrom 失败: {e}"
                break
            self.stats.packets += 1
            if self._raw_fp is not None:
                self._raw_fp.write(len(data).to_bytes(2, "little") + data)
            self._buf.append(data)
            self._buf_ts.append(time.time())
            got += 1

    # ── 方位角（按整圈切帧用） ──────────────────────────────
    def _sync_az(self) -> None:
        """给 _buf 里新到的报文补上方位角（原始单位 0.01°，不转成度）。

        只读 2 个字节，用整数比较判回绕 —— 不做浮点换算，也不解包点数据。
        实测 ~0.5 μs/包；单路 4500 包/s 时开销约 2 ms/s，可以忽略。
        """
        off = self.decoder.p.angle1_offset
        need = off + 2
        for b in self._buf[len(self._az):]:
            self._az.append(int.from_bytes(b[off:need], "little")
                            if len(b) >= need else -1)

    def _find_wraps(self) -> List[int]:
        """返回所有"方位角回绕"的位置 i（第 i 包属于**新的一圈**）。

        判据：方位角比上一包小了超过 180°（18000 个 0.01° 单位）。
        用"小了很多"而不是"小了就算"，是为了容忍同一圈内因丢包/乱序
        造成的小幅倒退 —— 那不是回绕。
        """
        wraps: List[int] = []
        prev = self._prev_az
        for i, a in enumerate(self._az):
            if a < 0:                       # 长度不足的坏包：不参与回绕判定
                continue
            if prev is not None and a < prev - 18000:
                wraps.append(i)
            prev = a
        return wraps

    def _drop_front(self, n: int) -> None:
        if n <= 0:
            return
        del self._buf[:n]
        del self._buf_ts[:n]
        del self._az[:n]

    def read(self) -> Optional[Frame]:
        self._collect()
        if self.frame_mode == "rotation":
            self._sync_az()
            return self._read_rotation()
        return self._read_time()

    # ── 按整圈切帧（默认） ──────────────────────────────────
    def _read_rotation(self) -> Optional[Frame]:
        now = time.time()
        if not self._buf:
            self._t_window = now            # 断流时把窗口推到当前，理由见 _read_time
            return None

        # 缓冲区上限：方位角一直不回绕时（字段解析错 / 雷达没转）别把内存吃光
        if len(self._buf) > self.max_buffer_packets:
            over = len(self._buf) - self.max_buffer_packets
            self._trimmed += over
            # 被丢掉的最后一包的方位角要留下来当"上一包"，否则回绕正好发生在
            # 丢弃边界上时会被漏判 —— 那一圈就被并进下一圈，出一帧重影。
            last = self._az[over - 1] if over <= len(self._az) else -1
            self._prev_az = last if last >= 0 else self._prev_az
            self._drop_front(over)

        wraps = self._find_wraps()
        if not wraps:
            # 兜底：整整 rotation_timeout_ms 都没见到回绕 → 当作"按整圈切不了"，
            # 退回按时间切帧。计数会体现在 info().frames_by_timeout 上。
            if (now - self._t_window) * 1000.0 >= self.rotation_timeout_ms:
                self._by_timeout += 1
                self._last_timeout_t = now
                return self._flush_window()
            return None

        # 取**最后一整圈**：wraps[-1] 是最新一圈的起点，它之前那一圈才是完整的。
        # worker 卡在检测里时会积压好几圈，更早的直接丢掉 —— 与旧逻辑的防重影同理，
        # 而且这里是按圈丢，不会切出半圈。
        end = wraps[-1]
        start = wraps[-2] if len(wraps) >= 2 else 0
        self._drop_front(start)
        end -= start
        bufs = self._buf[:end]
        tss = self._buf_ts[:end]
        self._drop_front(end)
        # 下一圈的第一包现在是 _buf[0]。它的"上一包"是刚出帧那一圈的最后一包，
        # 那次回绕已经消费掉了 —— 若把它留在 _prev_az 里，下一次 _find_wraps
        # 会在 index 0 上再报一次回绕，等于每圈都被切成"空的一帧 + 一圈"。
        # 所以置空：下一圈的回绕从 _buf 内部自己比就能判出来。
        self._prev_az = None
        self._by_rotation += 1
        return self._emit_batch(bufs, tss, now)

    # ── 按时间窗切帧（旧行为） ──────────────────────────────
    def _read_time(self) -> Optional[Frame]:
        now = time.time()
        if not self._buf:
            # ★ 没收到包时也要把时间窗推到当前 —— 否则雷达断流一段时间后，
            #   _t_window 还停在很久以前，来的**第一个包**就立刻满足
            #   "已过 frame_ms" 而被单独 flush 成一帧，产出一条扇面的残片点云。
            #   残片帧会让停稳检测反复在 IDLE/BASELINE 之间跳，白白打断会话。
            if (now - self._t_window) * 1000.0 >= self.frame_ms:
                self._t_window = now
            return None
        if (now - self._t_window) * 1000.0 < self.frame_ms:
            return None
        return self._flush_window()

    def _flush_window(self) -> Optional[Frame]:
        """把缓冲区按"最近 frame_ms×1.25"的时间窗切出一帧（time 模式 / 兜底）。"""
        bufs, tss = self._buf, self._buf_ts
        self._buf, self._buf_ts, self._az = [], [], []
        self._prev_az = None
        now = time.time()
        # 只保留窗口内的报文：worker 卡在检测里时队列会积压好几圈，
        # 全部塞进一帧会把多个旋转周期叠在一起 → 点云重影（见类文档）。
        # ★ 窗口以"最新那一包的时刻"为基准，不是以 now —— 否则 worker 被占住
        #   （或 rotation 兜底等了一整个 rotation_timeout_ms）之后，缓冲区里
        #   全部报文都比 now-1.25×frame_ms 还老，会被一次性丢光、出不了帧，
        #   表现为"雷达明明在出包，通道却一帧都不出"。
        ref = tss[-1] if tss else now
        cutoff = ref - self.frame_ms / 1000.0 * 1.25
        lo = len(bufs)
        for i, t in enumerate(tss):
            if t >= cutoff:
                lo = i
                break
        bufs, tss = bufs[lo:], tss[lo:]
        return self._emit_batch(bufs, tss, now)

    # 兼容旧名字（外部脚本/测试可能还在用）
    def _emit_batch(self, bufs, tss, now: float) -> Optional[Frame]:
        self._t_window = now
        self._seq += 1
        self._rev_packets = len(bufs)
        self._emit_dt = now - self._last_emit
        self._last_emit = now
        if not bufs:
            return None

        ts_last = tss[-1] if tss else now
        # ★ 整帧批量解码（不是逐包）—— 4 路并行时这才不会把 GIL 攥死。
        #   逐位一致性与性能见 tests/test_batch_decode.py、docs/04。
        xyz, ring, _it, info = self.decoder.decode_frame_batch(bufs, self._seq, ts_last)
        self.stats.dropped += info["dropped"]
        if info["dropped"]:
            self.last_error = f"本帧 {info['dropped']} 个报文长度不足，已丢弃"
        self._rev_deg += float(info.get("rev_deg", 0.0))
        if xyz.shape[0] == 0 and not info["dropped"]:
            return None
        # ★ 成功出帧就把上一次的瞬时错误清掉。
        #   之前 last_error 只写不清，于是"本帧 N 个报文长度不足"这种一次性抖动
        #   会永久挂在看板上，现场排查时会一直被它带偏。
        if not info["dropped"]:
            self.last_error = ""
        # packets：这一圈**实际解出点来**的报文数。停稳判据靠它识别"残缺帧"
        # —— 突发丢包会让一圈缺掉一整块扇形（实测最差连续缺 31 包 ≈ 12.8°），
        # 整帧质心随之偏 0.1~0.2 m，看起来就像车又动了。
        #
        # ▲▲ 这里**必须用 info["packets"]（有效数），不能用 len(bufs)（收到数）**。
        #    两者差的正是 `info["dropped"]` —— 长度不足、解不出点的报文。
        #    用 len(bufs) 的话，解码失败的报文仍被算进"这一圈有多少包"，于是
        #    半数报文解不开的帧照样声明满包 -> 过 `_frame_is_complete` ->
        #    `trusted=True` -> **拿到测量许可**。2026-09-19 实测：送进 901 包、
        #    451 包因长度不足被丢，帧仍声明 901，完整性检查照过。
        # 已知坏包单独传递；冷启动不能用持续损失后的低包数自建完整参考。
        # 全部解码失败也输出零点坏帧，让上层立即撤销旧停稳许可。
        f = Frame(xyz, ring, seq=self._seq, ts=ts_last, packets=int(info["packets"]),
                  dropped_packets=int(info["dropped"]))
        return self._emit(f)

    def close(self) -> None:
        # hub 是所有通道共享的，由 service 统一关闭，这里绝不能关它
        if self.hub is None and self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        if self._raw_fp is not None:
            self._raw_fp.close()
            self._raw_fp = None

    def info(self) -> dict:
        d = super().info()
        # 圈/分：按最近一段窗口算，不是开机以来的平均（见 RateWindow）
        rpm = self._rate_rev.rate(self._rev_deg / 360.0, self.stats.started) * 60.0
        d.update({
            "bind": (f"hub {self.hub.bind_ip}:{self.hub.udp_port}"
                     if self.hub is not None else f"{self.bind_ip}:{self.udp_port}"),
            "frame_mode": self.frame_mode,
            "frame_ms": self.frame_ms,
            "pending_packets": len(self._buf),
            "range_mode": self.decoder.range_mode,
            "source_ip": self.source_ip,
            "packet_rate": round(
                self._rate_pkts.rate(self.stats.packets, self.stats.started), 1),
            "rpm": round(rpm, 2),
            "rev_hz": round(rpm / 60.0, 3),
            "frame_seconds": round(self.frame_ms / 1000.0, 3),
            # 按整圈切帧的现场诊断：
            #   rev_packets     最近一帧的报文数 ≈ 一圈的包数（该稳定在某个值附近）
            #   frames_by_rotation / frames_by_timeout
            #                   分别是"按回绕切出来的帧"和"没等到回绕、兜底按时间切的帧"。
            #                   后者一直涨 = 方位角没解析对或雷达没转，点云会缺扇面。
            "rev_packets": self._rev_packets,
            "frames_by_rotation": self._by_rotation,
            "frames_by_timeout": self._by_timeout,
            "buffer_trimmed": self._trimmed,
            "frame_dt_ms": round(self._emit_dt * 1000.0, 1),
        })
        # ★ 只报**最近 30 秒内**还在发生的兜底切帧。
        #   这个计数是累计值，若无条件挂出去，开机瞬间或一次丢包抖动留下的 1 帧
        #   会让看板永远挂着红字 —— 现场很快就学会无视它，那这条告警就白做了。
        if (self.frame_mode == "rotation" and self._by_timeout > 0
                and time.time() - self._last_timeout_t < 30.0 and not d.get("error")):
            d["error"] = (f"最近 30s 内有帧没等到方位角回绕（累计 {self._by_timeout} 帧，"
                          f"已按时间兜底切帧）—— 检查 config.json 的 packet.angle1_offset / "
                          f"processing.angle_scale，或用 cli/capture.py 抓包核对方位角字段")
        if self.queue is not None:
            d["queue"] = {"pending": self.queue.qsize(), "pushed": self.queue.pushed,
                          "dropped": self.queue.dropped}
            # ★ 溢出红字只挂"最近 30 秒仍在发生"的（与上面兜底切帧同一条纪律）：
            #   瞬时积压（六面拟合几百 ms）不该报警，持续顶着上限才是
            #   通道线程卡死了 —— 那时才值得红字，否则现场学会无视就白做了。
            if self.queue.dropped > self._queue_dropped_seen:
                self._queue_dropped_seen = self.queue.dropped
                self._queue_drop_t = time.time()
            near_full = self.queue.maxlen and self.queue.qsize() >= self.queue.maxlen * 0.9
            if (time.time() - self._queue_drop_t < 30.0 and near_full
                    and not d.get("error")):
                d["error"] = (f"通道队列最近 30s 仍在溢出（累计丢 {self.queue.dropped} 包、"
                              f"当前积压 {self.queue.qsize()}）—— 通道线程大概率卡住了，"
                              f"看 /api/state 的 state 与最近的 result 事件定位卡点")
            # hub 是 4 条通道共用的收包线程：它 recvfrom 失败时，各通道只会表现为
            # "没有数据"。把 hub 的错误透到通道 info 里，现场才看得见真正的原因。
            if self.hub is not None and self.hub.last_error and not d.get("error"):
                d["error"] = f"LiveHub: {self.hub.last_error}"
        return d
