"""未裁观测与检测输入的精确配对；合帧不抹掉来源身份，不按坐标猜 ring。"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np

from .pointcloud import Frame, normalize_roi


@dataclass(frozen=True)
class FrameSnapshot:
    xyz_bytes: bytes
    ring_bytes: Optional[bytes]
    seq: int
    ts: float
    gen: int
    packets: int
    dropped_packets: int = 0

    @classmethod
    def freeze(cls, frame: Frame) -> "FrameSnapshot":
        if (type(frame.seq) is not int or frame.seq <= 0
                or not math.isfinite(frame.ts) or frame.ts <= 0
                or type(frame.gen) is not int or frame.gen < 0
                or type(frame.packets) is not int or frame.packets < 0
                or type(frame.dropped_packets) is not int or frame.dropped_packets < 0):
            raise ValueError("观测缺少有效 seq/ts/gen/packets 身份")
        if not np.isfinite(frame.xyz).all():
            raise ValueError("观测包含非有限坐标")
        return cls(frame.xyz.astype("<f4", copy=False).tobytes(),
                   frame.ring.astype("<u2", copy=False).tobytes() if frame.ring is not None else None,
                   frame.seq, float(frame.ts), frame.gen, frame.packets, frame.dropped_packets)

    def frame(self) -> Frame:
        # 从不可变 bytes 建视图；调用方不能通过 setflags 恢复写权限。
        xyz = np.frombuffer(self.xyz_bytes, dtype="<f4").reshape(-1, 3)
        ring = np.frombuffer(self.ring_bytes, dtype="<u2") if self.ring_bytes is not None else None
        return Frame(xyz, ring, self.seq, self.ts, self.gen, self.packets, self.dropped_packets)

    def identity(self) -> dict:
        return dict(seq=self.seq, ts=self.ts, gen=self.gen, packets=self.packets)


@dataclass(frozen=True)
class ObservationPair:
    channel: str
    epoch: int
    received_monotonic: float
    complete: bool
    raw: FrameSnapshot
    cropped: FrameSnapshot
    index_bytes: bytes

    @classmethod
    def freeze(cls, *, channel: str, epoch: int, received_monotonic: float,
               complete: bool, raw: Optional[Frame], cropped: Frame, roi) -> "ObservationPair":
        if raw is None:
            raise ValueError("缺少同次未裁观测，不能用检测输入冒充全观测")
        if (not isinstance(channel, str) or not channel or type(epoch) is not int or epoch < 0
                or type(complete) is not bool or not math.isfinite(received_monotonic)
                or received_monotonic < 0):
            raise ValueError("观测上下文或完整性无效")
        first, second = FrameSnapshot.freeze(raw), FrameSnapshot.freeze(cropped)
        if (first.identity() != second.identity()
                or first.dropped_packets != second.dropped_packets):
            raise ValueError("未裁观测与检测输入 seq/ts/gen/packets 不一致")
        if complete and first.dropped_packets:
            raise ValueError("已知解码损失不能声明为完整观测")
        bounds = normalize_roi(roi)
        indices = (np.arange(raw.n, dtype=np.int64) if bounds is None else
                   np.flatnonzero(np.all((raw.xyz >= bounds[0]) & (raw.xyz <= bounds[1]), axis=1)))
        expected = raw._mask(indices)
        if (not np.array_equal(expected.xyz, cropped.xyz)
                or (expected.ring is None) != (cropped.ring is None)
                or expected.ring is not None and not np.array_equal(expected.ring, cropped.ring)):
            raise ValueError("检测输入不是本次原观测的精确 ROI 行子集")
        return cls(channel, epoch, float(received_monotonic), complete, first, second,
                   indices.astype("<i8", copy=False).tobytes())

    def indices(self) -> np.ndarray:
        return np.frombuffer(self.index_bytes, dtype="<i8")


@dataclass(frozen=True)
class ObservationWindow:
    pairs: Tuple[ObservationPair, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.pairs, tuple) or not self.pairs:
            raise ValueError("测量窗口必须为非空观测元组")
        first = self.pairs[0]
        previous = None
        for pair in self.pairs:
            if (pair.channel, pair.epoch, pair.raw.gen) != (first.channel, first.epoch, first.raw.gen):
                raise ValueError("不同通道、活动代次或数据源代次不能合帧")
            if not pair.complete or pair.raw.dropped_packets or pair.cropped.dropped_packets:
                raise ValueError("残缺观测不能进入测量合帧")
            if previous is not None and (pair.raw.seq <= previous.raw.seq
                    or pair.raw.ts <= previous.raw.ts
                    or pair.received_monotonic <= previous.received_monotonic):
                raise ValueError("重复、乱序或时间不连续的来源不能合帧")
            previous = pair

    def input_frame(self) -> Frame:
        if len(self.pairs) == 1:
            return self.pairs[0].cropped.frame()
        frames = [pair.cropped.frame() for pair in self.pairs]
        last = frames[-1]
        # packets=0 明确不是单圈；每圈真实报文数在 segments 中记录，不伪造合计圈数。
        return Frame(np.concatenate([f.xyz for f in frames]),
                     np.concatenate([f.ring for f in frames]) if all(f.ring is not None for f in frames) else None,
                     last.seq, last.ts, last.gen, 0)

    def segments(self) -> list:
        segments, raw_start, input_start = [], 0, 0
        for index, pair in enumerate(self.pairs):
            raw, cropped = pair.raw.frame(), pair.cropped.frame()
            segments.append(dict(member=index, channel=pair.channel, epoch=pair.epoch,
                                 frame=pair.raw.identity(), complete=pair.complete,
                                 received_monotonic=pair.received_monotonic,
                                 raw_rows=[raw_start, raw_start + raw.n],
                                 input_rows=[input_start, input_start + cropped.n],
                                 raw_ring_present=raw.ring is not None,
                                 input_ring_present=cropped.ring is not None))
            raw_start += raw.n
            input_start += cropped.n
        return segments

    def detection_lineage(self, input_indices: np.ndarray) -> np.ndarray:
        """每行输出 (原成员号, 原始行号)，包括同坐标不同 ring 的重复点。"""
        selected = np.asarray(input_indices)
        total = sum(len(pair.indices()) for pair in self.pairs)
        if (selected.ndim != 1 or selected.dtype.kind not in "iu" or
                len(selected) and (selected.min() < 0 or selected.max() >= total
                                  or np.any(selected[1:] <= selected[:-1]))):
            raise ValueError("检测索引须为输入范围内有序唯一的整数行号")
        rows = []
        for index, pair in enumerate(self.pairs):
            raw_indices = pair.indices()
            rows.append(np.column_stack((np.full(len(raw_indices), index, np.int64), raw_indices)))
        return np.concatenate(rows)[selected]
