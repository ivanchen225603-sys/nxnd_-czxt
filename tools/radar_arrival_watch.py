"""从 LoadRadar 的只读状态接口记录疑似漏登，供对照到磅、抓拍或现场视频。"""

import argparse
import json
import sqlite3
import time
from contextlib import closing
from datetime import datetime, timedelta, timezone
from pathlib import Path
from urllib.request import ProxyHandler, build_opener


SHANGHAI = timezone(timedelta(hours=8))
STATE_PATH = "/api/state?history=0&frame_integrity=1"
SCHEMA = """
CREATE TABLE IF NOT EXISTS observations(
    channel TEXT NOT NULL, reason TEXT NOT NULL, started REAL NOT NULL,
    last_seen REAL NOT NULL, samples INTEGER NOT NULL, peak_points INTEGER NOT NULL,
    threshold INTEGER NOT NULL, last_complete REAL, instance TEXT NOT NULL,
    lane_status TEXT NOT NULL, passage_id TEXT,
    PRIMARY KEY(channel, reason, started));
CREATE TABLE IF NOT EXISTS watch_state(
    id INTEGER PRIMARY KEY CHECK(id=1), checked_at REAL NOT NULL,
    last_success REAL, error TEXT);
"""


def stamp(epoch):
    return datetime.fromtimestamp(epoch, SHANGHAI).isoformat(timespec="seconds") if epoch is not None else None


class ArrivalWatch:
    def __init__(self, threshold, low_min):
        self.threshold = threshold
        self.low_min = low_min
        self.channels = {}
        self.service_boot = None

    def update(self, snapshot):
        now = snapshot["service"]["now"]
        boot = now - snapshot["service"]["uptime"]
        restarted = self.service_boot is not None and abs(boot - self.service_boot) > 2
        self.service_boot = boot
        observations = []
        lanes = {lane["channelId"]: lane for lane in snapshot["integration"]["lanes"]}
        for channel in snapshot["channels"]:
            name = channel["id"]
            if name not in lanes or name not in self.threshold:
                continue
            threshold = self.threshold[name]
            lane = lanes[name]
            integrity = channel["frame_integrity"]
            instance = integrity["capture_instance_id"]
            previous = self.channels.get(name)
            new_instance = previous is None or previous["instance"] != instance
            if new_instance or restarted:
                startup = restarted or snapshot["service"]["uptime"] <= 60
                previous = {"instance": instance, "startup": startup, "segment": None, "seq": None}
                self.channels[name] = previous

            frames = integrity["frames"]
            latest = frames[-1] if frames else None
            last_complete = next((frame["decision_at"] for frame in reversed(frames) if frame["complete"]), None)
            last_observed = last_complete if last_complete is not None else boot
            gap = max(0, now - last_observed)
            status = lane["status"]
            passage = lane.get("passageId")
            if status in ("idle", "occupied"):
                previous["startup"] = False
            stability = channel["stability"]
            points = stability.get("n_vehicle_points") or 0
            new_frame = latest is not None and latest["seq"] != previous["seq"]
            if latest is not None:
                previous["seq"] = latest["seq"]
            trusted = bool(latest and latest["complete"] and stability.get("fresh") and
                           not stability.get("degraded") and channel["online"] and
                           channel["frame_age_sec"] is not None and channel["frame_age_sec"] <= 1)
            reason = None
            if status == "quarantined" and trusted and points >= threshold and previous["startup"]:
                reason = "restart_occupied"
            elif status == "quarantined" and gap > 5:
                reason = "observation_gap"
            elif status == "quarantined" and trusted and points >= threshold:
                reason = "quarantined_occupied"
            elif status == "idle" and not passage and trusted and self.low_min <= points < threshold:
                reason = "below_arrival_threshold"

            segment = previous["segment"]
            if reason is None:
                if segment and passage and segment["samples"] >= segment["required"]:
                    segment.update(passage_id=passage, lane_status=status)
                    observations.append(segment.copy())
                previous["segment"] = None
                continue
            if segment is None or reason != segment["reason"]:
                segment = dict(channel=name, reason=reason, started=now, samples=0, peak_points=0,
                               threshold=threshold, instance=instance,
                               required=3 if reason == "below_arrival_threshold" else 1)
                previous["segment"] = segment
            if new_frame or reason == "observation_gap":
                segment.update(samples=segment["samples"] + 1,
                               peak_points=max(segment["peak_points"], points if trusted else 0),
                               last_seen=now, last_complete=last_complete, lane_status=status,
                               passage_id=passage)
                if segment["samples"] >= segment["required"]:
                    observations.append(segment.copy())
        return observations


def run(args, store):
    watch = ArrivalWatch(dict(args.threshold), args.low_min)
    read = build_opener(ProxyHandler({}))
    last_success = store.execute("SELECT last_success FROM watch_state WHERE id=1").fetchone()
    last_success = last_success[0] if last_success else None
    while True:
        checked = time.time()
        try:
            with read.open(args.url.rstrip("/") + STATE_PATH, timeout=3) as response:
                snapshot = json.load(response)
            rows = watch.update(snapshot)
            last_success, error = checked, None
        except (OSError, ValueError, KeyError, TypeError) as exc:
            watch.channels.clear()
            rows, error = [], str(exc)[:200]
        with store:
            store.executemany("""
                INSERT OR REPLACE INTO observations VALUES (
                  :channel,:reason,:started,:last_seen,:samples,:peak_points,
                  :threshold,:last_complete,:instance,:lane_status,:passage_id)
                """, rows)
            store.execute("INSERT OR REPLACE INTO watch_state VALUES (1,?,?,?)", (checked, last_success, error))
        time.sleep(args.interval)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--db", type=Path, required=True)
    sub = cli.add_subparsers(dest="command", required=True)
    capture = sub.add_parser("run")
    capture.add_argument("--url", default="http://127.0.0.1:8080")
    capture.add_argument("--interval", type=float, default=2)
    capture.add_argument("--low-min", type=int, default=1500)
    capture.add_argument("--threshold", action="append", nargs=2, metavar=("CHANNEL", "POINTS"), required=True)
    report = sub.add_parser("report")
    report.add_argument("--channel")
    args = cli.parse_args()
    if args.command == "run":
        args.threshold = [(name, int(threshold)) for name, threshold in args.threshold]
        if args.interval < 1 or args.low_min < 0 or any(points <= args.low_min for _, points in args.threshold):
            cli.error("采样间隔至少一秒，低点数记录下限应低于各车道入场阈值")
        args.db.parent.mkdir(parents=True, exist_ok=True)
        with closing(sqlite3.connect(args.db, timeout=5)) as store:
            store.executescript(SCHEMA)
            run(args, store)
        return
    with closing(sqlite3.connect(args.db.resolve().as_uri() + "?mode=ro", uri=True, timeout=5)) as store:
        store.row_factory = sqlite3.Row
        state = store.execute("SELECT checked_at,last_success,error FROM watch_state WHERE id=1").fetchone()
        if state:
            print(json.dumps(dict(checked_at=stamp(state[0]), last_success=stamp(state[1]), error=state[2]),
                             ensure_ascii=False))
        for row in store.execute("SELECT * FROM observations WHERE (? IS NULL OR channel=?) ORDER BY started",
                                 (args.channel, args.channel)):
            event = dict(row)
            for name in ("started", "last_seen", "last_complete"):
                event[name] = stamp(event[name])
            print(json.dumps(event, ensure_ascii=False))


if __name__ == "__main__":
    main()
