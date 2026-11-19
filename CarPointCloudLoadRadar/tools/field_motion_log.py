# -*- coding: utf-8 -*-
"""现场运动量记录器 —— 有车时把停稳判据的各个量逐帧写 CSV。

为什么需要它（2026-09-21 现场）：
    停稳层卡住时，看板和日志只给出"拦在 shape_speed x173"这样的**计数**，
    拿不到**数值分布**，而定门槛必须要分布。这个脚本把 drift / speed /
    shape_speed / yaw_speed 逐帧记下来，离线就能算 p50/p90/p99 与重叠度。

用法：
    python tools/field_motion_log.py                 # 默认写 data/field-log/
    python tools/field_motion_log.py --out 某目录
"""
import argparse, csv, json, os, sys, time, urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIELDS = ["t", "ch", "session", "state", "n_veh", "stationary_sec", "speed", "instant_speed",
          "drift", "shape_speed", "yaw_speed", "settling", "stable", "triggers", "saved", "reason"]


def main(argv=None):
    ap = argparse.ArgumentParser(description="现场运动量记录器")
    ap.add_argument("--url", default="http://127.0.0.1:8080/api/state")
    ap.add_argument("--out", default=os.path.join(ROOT, "data", "field-log"))
    ap.add_argument("--min-points", type=int, default=5000, help="车辆点少于它就不记，避免空闲刷盘")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, "motion_%s.csv" % time.strftime("%Y%m%d"))
    new = not os.path.exists(path)
    f = open(path, "a", newline="", encoding="utf-8-sig")
    w = csv.writer(f)
    if new:
        w.writerow(FIELDS)
    print("记录到 %s" % path, flush=True)
    last = {}
    while True:
        try:
            d = json.load(urllib.request.urlopen(a.url, timeout=3))
        except Exception:
            time.sleep(1.0)
            continue
        for c in d.get("channels", []):
            s = c.get("stability") or {}
            n = s.get("n_vehicle_points", 0)
            if n < a.min_points and last.get(c["id"], 0) < a.min_points:
                last[c["id"]] = n
                continue
            last[c["id"]] = n
            tg = s.get("triggers") or []
            tg = " ".join(str(x) for x in tg)
            w.writerow([time.strftime("%H:%M:%S"), c["id"], c.get("session"), s.get("state_cn"), n,
                        round(s.get("stationary_sec", 0), 2), s.get("speed_mps"), s.get("instant_speed_mps"),
                        round(s.get("drift_m", 0) or 0, 4), round(s.get("shape_speed_mps", 0) or 0, 4),
                        round(s.get("yaw_speed_mps", 0) or 0, 4), s.get("settling"), s.get("stable"),
                        tg, c.get("saved_count"), c.get("last_reason", "")])
        f.flush()
        time.sleep(0.1)


if __name__ == "__main__":
    sys.exit(main())
