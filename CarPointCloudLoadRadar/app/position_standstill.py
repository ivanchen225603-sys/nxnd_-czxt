# -*- coding: utf-8 -*-
"""CH4 双雷达尺寸采集的"位置停稳"判定：只看车辆是否挪动。

与 JT128 原判稳（StabilityDetector）完全分开：不读取、不过滤其 triggers，
不改变其锚点、输出、留样或体积。原判稳的位置锚点会被形变判据反复重置，
所以这里用自己的起点：

· 位置：车辆点水平中位位置相对起点的位移；
· 航向：航向变化 × 车辆半径，换算成位移（与原检测器 yaw_drift 同一口径）；
· 任一超过 drift_m 就以本帧为新起点重新计时；连续满足 min_sec 即为位置停稳；
· 帧不可用（残缺、不新鲜、无车）或车次变化即清零。

门槛复用 config.detection.stationary_drift_m / stationary_min_sec，不新增配置。
"""
from __future__ import annotations
import math


class PositionStandstill:
    def __init__(self, drift_m: float, min_sec: float, max_gap_sec: float | None = None):
        self.drift_m = float(drift_m)
        self.min_sec = float(min_sec)
        self.max_gap_sec = max_gap_sec
        self.reset()

    def reset(self) -> None:
        self._key = None
        self._origin = None
        self._since = None
        self._last_ts = None
        self._last_seq = None
        self.break_reason = None

    def _moved(self, xy, heading, radius) -> bool:
        origin_xy, origin_heading = self._origin
        if math.hypot(float(xy[0]) - origin_xy[0], float(xy[1]) - origin_xy[1]) > self.drift_m:
            return True
        if heading is None or origin_heading is None:
            # 航向不可观测（主轴接近等向）时只按位置判断；可观测性变化视为重新起算。
            return (heading is None) != (origin_heading is None)
        return abs(float(heading) - origin_heading) * float(radius) > self.drift_m

    def update(self, key, ts: float, pose, usable: bool, seq: int | None = None) -> bool:
        """喂入一帧，返回此刻是否位置停稳。

        key: 车次编号（None 表示没有车次）；ts: 本帧采集时刻（秒，本机时钟）；
        pose: StabilityDetector.last_pose() 的 (位置, 航向, 半径)；usable: 本帧完整、新鲜且车辆在场。
        """
        self.break_reason = None
        if not usable or pose is None or key is None or not math.isfinite(ts):
            self.reset()
            self.break_reason = "observation_unavailable"
            return False
        if key != self._key:
            self.reset()
            self._key = key
        xy, heading, radius = pose
        if not all(math.isfinite(v) for v in (xy[0], xy[1], radius)) or (heading is not None and not math.isfinite(heading)):
            self.reset()
            self.break_reason = "observation_unavailable"
            return False
        if self._last_ts is not None and (ts <= self._last_ts
                or (self.max_gap_sec is not None and ts - self._last_ts > self.max_gap_sec)
                or (seq is not None and self._last_seq is not None and seq != self._last_seq + 1)):
            self._origin = None
            self.break_reason = "observation_gap"
        self._last_ts, self._last_seq = ts, seq
        if self._origin is None or self._moved(xy, heading, radius):
            if self._origin is not None:
                self.break_reason = "movement"
            self._origin = ((float(xy[0]), float(xy[1])), None if heading is None else float(heading))
            self._since = float(ts)
            return False
        return float(ts) - self._since + 1e-9 >= self.min_sec
