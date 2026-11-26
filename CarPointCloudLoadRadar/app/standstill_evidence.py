"""CH4 连续静止段的累计证据。网络错误时同一序号重发原文。"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import logging
import math
import threading
import time
import uuid

from .load_radar_transport import HttpError

logger = logging.getLogger(__name__)


def utc_stamp(ts: float) -> str:
    return datetime.fromtimestamp(ts, timezone.utc).isoformat()


@dataclass
class StillSegment:
    passage: str
    id: str
    start: float
    last: float
    broken: float | None = None
    reason: str | None = None
    published_until: float | None = None

    @classmethod
    def begin(cls, passage: str, ts: float) -> StillSegment:
        return cls(passage, uuid.uuid4().hex, ts, ts)

    def seal(self, observed_at: float, reason: str) -> None:
        if self.broken is None:
            self.broken = max(observed_at if math.isfinite(observed_at) else time.time(), self.last)
            self.reason = reason

    def snapshot(self, node: str, channel: str) -> dict:
        return dict(nodeId=node, channelId=channel, passageId=self.passage, segmentId=self.id,
                    stoppedAt=utc_stamp(self.start), lastStillAt=utc_stamp(self.last),
                    brokenAt=utc_stamp(self.broken) if self.broken is not None else None, reason=self.reason)


@dataclass
class _Delivery:
    latest: dict
    expires: float
    acknowledged: dict | None = None
    in_flight: dict | None = None
    sequence: int = 0
    next_attempt: float = 0.0


class StillEvidenceSender:
    """只有发送前才分配序号。超时可能已经提交，下一次继续发送同一份快照。"""
    def __init__(self, client, retry_sec: float, timeout_sec: float):
        self.client = client
        self.retry_sec = retry_sec
        self.timeout_sec = timeout_sec
        self._jobs: dict[str, _Delivery] = {}
        self._condition = threading.Condition()
        self._thread = None
        self._stopped = False

    def publish(self, payload: dict) -> None:
        with self._condition:
            if self._stopped:
                return
            key = payload["segmentId"]
            job = self._jobs.get(key)
            if job is None:
                if len(self._jobs) >= 128:
                    logger.warning("静止证据发送缓冲已满，旧段停止续报")
                    del self._jobs[next(iter(self._jobs))]
                job = self._jobs[key] = _Delivery(payload, time.monotonic() + 60)
            else:
                job.latest = payload
                job.expires = time.monotonic() + 60
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="load-radar-still-evidence", daemon=True)
                self._thread.start()
            self._condition.notify_all()

    def _run(self) -> None:
        while True:
            with self._condition:
                if self._stopped:
                    return
                now = time.monotonic()
                for key, job in list(self._jobs.items()):
                    if now >= job.expires:
                        if job.latest != job.acknowledged:
                            logger.warning("静止证据回执超时 passageId=%s segmentId=%s", job.latest["passageId"], key)
                        del self._jobs[key]
                    elif job.latest != job.acknowledged and now >= job.next_attempt:
                        if job.in_flight is None:
                            job.in_flight = dict(job.latest, sequence=job.sequence + 1)
                        break
                else:
                    due = [job.expires if job.latest == job.acknowledged else min(job.expires, job.next_attempt)
                           for job in self._jobs.values()]
                    self._condition.wait(max(0, min(due) - now) if due else None)
                    continue
                payload = job.in_flight
            try:
                self.client.report_standstill_state(payload)
            except Exception as exc:
                with self._condition:
                    if isinstance(exc, HttpError) and not exc.retryable and exc.status != 409:
                        logger.warning("静止证据被拒绝 passageId=%s segmentId=%s status=%s",
                                       payload["passageId"], key, exc.status)
                        job.next_attempt = job.expires
                    else:
                        job.next_attempt = time.monotonic() + self.retry_sec
                continue
            with self._condition:
                job.sequence = payload["sequence"]
                job.acknowledged = {k: v for k, v in payload.items() if k != "sequence"}
                job.in_flight = None
                job.next_attempt = 0

    def close(self) -> None:
        with self._condition:
            self._stopped = True
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=self.timeout_sec + 2)
