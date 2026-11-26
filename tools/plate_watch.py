"""持续读取 DM8 已登记的车次和绑定事实，供现场逐通道核对。"""
import argparse
from contextlib import closing
from datetime import datetime, timedelta
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import time


STORE = Path("/data/carpointcloud/service-state/plate-watch/monitor.sqlite")
SQL = """SELECT a.Id,a.SessionId,a.CameraIdentifier,a.ArrivalTime,a.PlateStatus,a.LicensePlate,
                a.PlateSource,j.Status,j.AttemptCount,j.LastError,j.UpdatedAt,
                p.CaptureTime,p.EnterId,p.CreatedAt
         FROM CARPOINTCLOUD.VehicleArrival a
         LEFT JOIN CARPOINTCLOUD.PlateAcquisitionJob j ON j.WorkflowId=a.SessionId
         LEFT JOIN CARPOINTCLOUD.LicensePlateRecord p ON p.WorkflowId=a.SessionId AND p.IsSuccess=1
              AND p.CameraIdentifier=a.CameraIdentifier AND p.LicensePlate=a.LicensePlate
         WHERE a.DeviceId='LOAD_RADAR' AND """

SCHEMA = """
PRAGMA journal_mode=WAL;
CREATE TABLE IF NOT EXISTS trips(
    workflow TEXT PRIMARY KEY, arrival TEXT NOT NULL, channel TEXT NOT NULL,
    bound INTEGER NOT NULL, pending INTEGER NOT NULL, payload TEXT NOT NULL);
CREATE INDEX IF NOT EXISTS trips_channel_arrival ON trips(channel,arrival);
CREATE INDEX IF NOT EXISTS trips_pending ON trips(pending);
CREATE TABLE IF NOT EXISTS events(
    id INTEGER PRIMARY KEY AUTOINCREMENT, channel TEXT NOT NULL, arrival TEXT NOT NULL,
    observed TEXT NOT NULL, payload TEXT NOT NULL);
CREATE INDEX IF NOT EXISTS events_arrival ON events(arrival);
CREATE TABLE IF NOT EXISTS meta(key TEXT PRIMARY KEY,value TEXT NOT NULL);
"""


def dt(value):
    return value.strftime("%Y-%m-%d %H:%M:%S") if value is not None else None


def meta(store, key):
    row = store.execute("SELECT value FROM meta WHERE key=?", (key,)).fetchone()
    return row[0] if row else None


def put(store, key, value):
    store.execute("INSERT OR REPLACE INTO meta VALUES(?,?)", (key, value))


def source():
    import dmPython
    raw = subprocess.check_output(["systemctl", "show", "-p", "WorkingDirectory", "--value",
                                   "carpointcloudwebapi.service"], universal_newlines=True)
    cfg = json.loads((Path(raw.strip()) / "appsettings.json").read_text(encoding="utf-8-sig"))
    lanes = {ch["CameraIdentifier"]: ch["ChannelId"] for n in cfg["LoadRadar"]["Nodes"] for ch in n["Channels"]}
    keys = {k.lower().strip(): v.strip() for k, v in (part.split("=", 1) for part in
            cfg["ConnectionStrings"]["CarPointCloudDb"].split(";") if "=" in part)}
    if keys["schema"].upper() != "CARPOINTCLOUD" or keys["server"] not in ("localhost", "127.0.0.1"):
        raise ValueError("unexpected DM8 target")
    db = dmPython.connect(user=keys["userid"], password=keys["pwd"], server=keys["server"], port=int(keys["port"]))
    return db, lanes


def snapshot(cur, row, lane):
    aid, wf, camera, at, status, plate, origin, job, attempts, error, updated, capture, eid, bound = row
    if eid is not None and not isinstance(eid, str):
        eid = eid.read() if hasattr(eid, "read") else str(eid)
    if isinstance(eid, bytes):
        eid = eid.decode("utf-8")
    first = None
    if eid:
        digest = hashlib.sha256((camera + "\0" + eid).encode()).hexdigest().upper()
        cur.execute("SELECT FirstSeenAt FROM CARPOINTCLOUD.PartnerPlateObservation WHERE ObservationKey=?", (digest,))
        obs = cur.fetchone()
        first = obs[0] if obs else None
    decided = (bound or updated) if status == "SUCCEEDED" else updated if job == "FAILED" else None
    return {"id": aid, "workflow": wf, "channel": lane, "camera": camera, "arrival": dt(at),
            "status": status, "plate": plate if status == "SUCCEEDED" else None, "source": origin,
            "job": job, "attempts": attempts, "capture": dt(capture), "enterID": eid,
            "firstSeen": dt(first), "decision": dt(decided),
            "matchSeconds": round((decided-at).total_seconds(), 3) if decided else None, "reason": error}


def write_trip(store, record, observed):
    wf, at, channel = record["workflow"], record["arrival"], record["channel"]
    packed = json.dumps(record, separators=(",", ":"), ensure_ascii=False)
    old = store.execute("SELECT payload FROM trips WHERE workflow=?", (wf,)).fetchone()
    if old and old[0] == packed:
        return
    store.execute("INSERT OR REPLACE INTO trips VALUES(?,?,?,?,?,?)",
                  (wf, at, channel, record["status"] == "SUCCEEDED", record["status"] == "PENDING", packed))
    store.execute("INSERT INTO events(channel,arrival,observed,payload) VALUES(?,?,?,?)",
                  (channel, at, observed, packed))


def poll(db, store, lanes, start):
    now = datetime.now()
    end = dt(now)
    keep = max(start, now - timedelta(days=30))
    reconcile = meta(store, "reconcile")
    full = not reconcile or now - datetime.fromisoformat(reconcile) >= timedelta(hours=1)
    last = meta(store, "lastOK")
    # 每小时从业务事实补齐迟提交的历史行。普通轮次重读时间边界，并跟进所有尚未收口的车次。
    since = keep if full or not last else max(keep, min(datetime.fromisoformat(last), now) - timedelta(minutes=2))
    cur = db.cursor()
    try:
        def collect(predicate, params):
            cur.execute(SQL + predicate + " ORDER BY a.Id", tuple(params))
            for row in cur.fetchall():
                if row[2] in lanes:
                    write_trip(store, snapshot(cur, row, lanes[row[2]]), end)
        with store:
            collect("a.CreatedAt>=?", [since])
            pending = list(store.execute("SELECT workflow FROM trips WHERE pending=1 AND arrival<?", (dt(since),)))
            for i in range(0, len(pending), 64):
                ids = [r[0] for r in pending[i:i + 64]]
                collect("a.SessionId IN (" + ",".join("?" for _ in ids) + ")", ids)
            if full:
                expiry = dt(now - timedelta(days=30))
                store.execute("DELETE FROM events WHERE arrival<?", (expiry,))
                store.execute("DELETE FROM trips WHERE arrival<?", (expiry,))
                put(store, "reconcile", end)
            put(store, "lanes", json.dumps(sorted(lanes.values())))
            put(store, "lastOK", end)
    finally:
        cur.close()
        db.rollback()  # 结束仅含 SELECT 的事务，下轮重新读取现场状态。


def run(path, start):
    os.umask(0o077)
    path.parent.mkdir(mode=0o700, exist_ok=True)
    with closing(sqlite3.connect(str(path), timeout=5)) as store:
        store.executescript(SCHEMA)
        db = None
        while True:
            try:
                if db is None:
                    db, lanes = source()
                poll(db, store, lanes, start)
            except Exception as error:
                print("plate-watch: poll failed:", type(error).__name__, flush=True)
                if db is not None:
                    try:
                        db.close()
                    except Exception:
                        pass
                    db = None
            time.sleep(5)


def output(record):
    plate = record["plate"] or ("未匹配" if record["status"] == "FAILED" else "等待中")
    print(f'{record["channel"]}  到磅 {record["arrival"]}  {record["status"]}  {plate}'
          f'  尝试 {record["attempts"]}  抓拍 {record["capture"] or "—"}  首次收到 {record["firstSeen"] or "—"}'
          f'  决定 {record["decision"] or "—"}  耗时秒 {record["matchSeconds"]}  enterID {record["enterID"] or "—"}'
          f'  {record["reason"] or ""}  workflow {record["workflow"]}', flush=True)


def view(path, args):
    with closing(sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)) as store:
        where = " WHERE channel=?" if args.channel else ""
        params = [args.channel] if args.channel else []
        # 同一快照内取得显示基线与事件游标，避免进入持续显示时漏过刚发生的变化。
        store.execute("BEGIN")
        cursor = store.execute("SELECT COALESCE(MAX(id),0) FROM events").fetchone()[0]
        print("最近成功同步", meta(store, "lastOK") or "等待首次同步", flush=True)
        if args.status:
            for channel in json.loads(meta(store, "lanes") or "[]"):
                n, bound, pending = store.execute(
                    "SELECT COUNT(*),COALESCE(SUM(bound),0),COALESCE(SUM(pending),0) FROM trips WHERE channel=?",
                    (channel,)).fetchone()
                print(f"{channel}  车次 {n}  已匹配 {bound}  待定 {pending}  失败 {n-bound-pending}")
        else:
            rows = list(store.execute("SELECT payload FROM trips" + where + " ORDER BY arrival DESC,rowid DESC LIMIT ?",
                                      params + [args.limit]))
            for row in reversed(rows):
                output(json.loads(row[0]))
        store.commit()
        while args.follow:
            cond = " AND channel=?" if args.channel else ""
            for eid, payload in store.execute("SELECT id,payload FROM events WHERE id>?" + cond + " ORDER BY id",
                                              [cursor] + params).fetchall():
                output(json.loads(payload))
                cursor = eid
            time.sleep(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    reader = commands.add_parser("view")
    reader.add_argument("--channel")
    reader.add_argument("--limit", type=int, default=20)
    reader.add_argument("--follow", action="store_true")
    reader.add_argument("--status", action="store_true")
    watch = commands.add_parser("run")
    watch.add_argument("--since", required=True, type=datetime.fromisoformat)
    options = parser.parse_args()
    if options.command == "run":
        run(STORE, options.since)
    else:
        view(STORE, options)
