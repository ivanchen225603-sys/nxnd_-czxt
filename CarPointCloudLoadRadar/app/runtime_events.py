# -*- coding: utf-8 -*-
"""近期运行过程只读记录；不参与采集许可、身份关联或计算。"""
from collections import deque
import math
import threading
import time
import uuid


class RuntimeEvents:
    """每通道最多一百条，源进程重启后以新编号开始。"""

    def __init__(self, channel, source):
        self.channel, self.source = str(channel), str(source)
        self._epoch, self._seq = uuid.uuid4().hex, 0
        self._rows = deque(maxlen=100)
        self._last = {}
        self._lock = threading.RLock()
        self._session = 0
        self._stable = self._settling = self._gap = False
        self._forced_ts = None

    def record(self, kind, message, session, ts=None, category=None, identity=None):
        with self._lock:
            signature = (session, kind, message, identity)
            category = category or kind
            if self._last.get(category) == signature:
                return None
            stamp = float(ts) if isinstance(ts, (int, float)) and not isinstance(ts, bool) else time.time()
            if not math.isfinite(stamp) or stamp <= 0:
                stamp = time.time()
            self._last[category] = signature
            self._seq += 1
            event = dict(event_id=self._epoch + ":" + str(self._seq), channel=self.channel,
                         session=max(0, int(session)), ts=stamp, kind=kind,
                         message=message, source=self.source)
            self._rows.append(event)
            return dict(event)

    def observe(self, session, stability, ts, forced=False):
        """仅消费检测器已经认可的完整新观测，不从浏览器或旧快照推断过车。"""
        if stability.get("fresh") is not True or stability.get("degraded") is True:
            return []
        state = stability.get("state")
        if isinstance(state, bool) or state not in (0, 1, 2):
            return []
        events = []

        def emit(kind, message, number=None):
            row = self.record(kind, message, self._session if number is None else number, ts,
                              category=kind, identity=self._seq)
            if row is not None:
                events.append(row)

        with self._lock:
            if self._gap:
                emit("observation_resumed", "已恢复新帧，重新检查车辆停稳状态")
                self._gap = False
            if state == 0:
                if self._session:
                    emit("vehicle_departed", "已观测到车辆离开，车道恢复空闲")
                self._session = 0
                self._stable = self._settling = False
                self._forced_ts = None
                return events
            if session <= 0:
                return events
            if self._session != session:
                self._session = session
                self._stable = self._settling = False
                self._forced_ts = ts if forced else None
                emit("observation_resumed" if forced else "vehicle_detected",
                     "测量会话已重开，车辆仍在场" if forced else "检测到车辆，等待连续停稳")
            # 强制重开当拍的稳定字典可能属于重置前；日志同样等待下一份新观测。
            if self._forced_ts is not None:
                if ts <= self._forced_ts:
                    return events
                self._forced_ts = None
            if state == 2 and stability.get("stable") is True:
                if not self._stable:
                    emit("vehicle_stable", "车辆已停稳")
                self._stable = True
            elif state == 1 and self._stable:
                emit("vehicle_moving", "车辆重新移动，重新等待停稳")
                self._stable = False
            settling = stability.get("settling") is True
            if settling and not self._settling:
                emit("settling", "开始沉降观测，等待连续完整新帧")
            self._settling = settling
        return events

    def gap(self):
        with self._lock:
            if self._gap:
                return None
            self._gap = True
            self._stable = self._settling = False
            return self.record("observation_gap", "新帧超时，暂停停稳与检测", self._session,
                               category="observation_gap", identity=self._seq)

    def snapshot(self):
        with self._lock:
            return [dict(row) for row in self._rows]
