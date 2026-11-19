# -*- coding: utf-8 -*-
"""时间序列测试辅助：显式新采集身份和虚拟时钟，不关闭任何停稳门控。"""
from unittest.mock import patch

from app.pointcloud import Frame


class ObservationClock:
    def __init__(self, test):
        self.wall, self.mono, self.seq = 1000.0, 100.0, 0
        for name, getter in (("time.time", lambda: self.wall),
                             ("time.monotonic", lambda: self.mono)):
            patched = patch(name, side_effect=getter)
            patched.start()
            test.addCleanup(patched.stop)

    def advance(self, seconds):
        self.wall += seconds
        self.mono += seconds

    def frame(self, original, dt=0.5):
        self.advance(dt)
        self.seq += 1
        return Frame(original.xyz, original.ring, seq=self.seq, ts=self.wall,
                     gen=original.gen, packets=original.packets,
                     dropped_packets=original.dropped_packets)

    def detect(self, detector, original, trusted=True, dt=0.5):
        return detector.detect(self.frame(original, dt), trusted=trusted)

    def hold(self, detector, original, steps=7, dt=0.5):
        result = None
        for _ in range(steps):
            result = self.detect(detector, original, dt=dt)
        return result

    def tick(self, worker, original, dt=0.1):
        frame = self.frame(original, dt) if original is not None else None
        if original is None:
            self.advance(dt)
        worker.source.read = lambda: frame
        if frame is not None and worker.source.keep_raw:
            worker.source._last_raw = frame
        return worker._tick()

    def settle(self, worker, original, limit=200):
        """推进到真实沉降入口，不预先消耗此次测量。"""
        for _ in range(limit):
            self.tick(worker, original)
            if worker.st.stability.get("settling"):
                return
        raise AssertionError("连续新静止帧未能进入沉降窗口")
