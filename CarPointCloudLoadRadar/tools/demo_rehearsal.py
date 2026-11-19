# -*- coding: utf-8 -*-
"""
demo_rehearsal.py — 演示前的"彩排"：不开窗口、不起看板，
直接起一遍 4 路仿真 + 自动编排，把每一秒的状态打成一条时间线。

为什么要有这个（而不是直接双击演示版看一眼）
    演示现场没有第二次机会。真到领导面前才发现"某一路一直不动"
    或者"四路车牌一模一样"，就来不及了。
    这里把整条链路跑一遍并给出可核对的结论：
        · 每一路都真的产出过 result（不是只有画面在动）
        · 空车/载货两种判定都出现过
        · 四路车牌各不相同（错开起牌是否生效）
        · 停稳偏差在车停稳后回落（说明参考帧机制正常）
        · 没有任何 error 事件

用法
    python tools/demo_rehearsal.py                 # 120 秒，4 路，人类可读
    python tools/demo_rehearsal.py --sec 45        # 跑短一点
    python tools/demo_rehearsal.py --json out.json # 额外落一份机器可读结果
    python tools/demo_rehearsal.py --quiet         # 只打印结论

退出码：0 = 彩排通过；1 = 有通道没出结果 / 出现过 error。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="demo_rehearsal.py",
        description="4 路仿真雷达 + 自动编排的演示彩排（无窗口、无看板）",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("--sec", type=float, default=120.0, help="彩排时长（秒，默认 120）")
    p.add_argument("--tick", type=float, default=1.0, help="状态采样间隔（秒，默认 1.0）")
    p.add_argument("--json", dest="json_path", default="", metavar="PATH",
                   help="把结论另存为 JSON")
    p.add_argument("--quiet", action="store_true", help="不逐秒打印，只给结论")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    from app.settings import load_settings
    from app.service import LidarService

    st = load_settings()
    chans = st.enabled()
    if not chans:
        print("× config/channels.json 里没有启用的通道")
        return 2
    ids = [c.id for c in chans]
    print("=" * 78)
    print(" 演示彩排 · 4 路仿真雷达 + 自动编排")
    print("=" * 78)
    print(" 通道     : %s" % ", ".join(ids))
    print(" 时长     : %.0f s" % args.sec)
    from app.settings import baseline_rel
    print(" 落盘     : 关闭（save_override=False，不污染 %s）" % baseline_rel())
    print("-" * 78)

    svc = LidarService(settings=st, save_override=False,
                       plate_mode="simulate", demo=True, auto_demo=True)
    q = svc.bus.subscribe()

    per = {cid: {"results": [], "plates": [], "reasons": [],
                 "max_dev": 0.0, "min_dev_after_stable": None,
                 "phases": set(), "state_codes": set()} for cid in ids}
    errors = []
    warns = []
    dir_status = {}

    svc.start()
    t0 = time.time()
    next_sample = t0
    try:
        while time.time() - t0 < args.sec:
            while q:
                ev = q.popleft()
                k = ev.get("kind")
                cid = ev.get("channel") or ""
                if k == "result" and cid in per:
                    d = per[cid]
                    d["results"].append({"t": round(ev["ts"] - t0, 1),
                                         "plate": ev.get("plate"),
                                         "empty": bool(ev.get("empty")),
                                         "reason": ev.get("reason"),
                                         "n_points": ev.get("n_points"),
                                         "deviation": ev.get("deviation")})
                    if ev.get("plate"):
                        d["plates"].append(ev["plate"])
                    if ev.get("reason"):
                        d["reasons"].append(ev["reason"])
                elif k == "error":
                    errors.append({"t": round(ev.get("ts", 0) - t0, 1),
                                   "channel": cid, "error": ev.get("error")})
                elif k == "warn":
                    warns.append(ev.get("message", ""))
                elif k == "demo_phase" and cid in per:
                    per[cid]["phases"].add(ev.get("phase", ""))

            now = time.time()
            if now >= next_sample:
                next_sample = now + args.tick
                snap = svc.snapshot()
                dir_status = snap.get("director") or {}
                states = {}
                for c in snap["channels"]:
                    cid = c["id"]
                    stb = c.get("stability") or {}
                    dmo = c.get("demo") or {}
                    dev = float(stb.get("deviation") or 0.0)
                    if cid in per:
                        per[cid]["max_dev"] = max(per[cid]["max_dev"], dev)
                        if str(stb.get("state_cn")) == "已停稳":
                            m = per[cid]["min_dev_after_stable"]
                            per[cid]["min_dev_after_stable"] = dev if m is None else min(m, dev)
                        per[cid]["state_codes"].add(int(stb.get("state", 0) or 0))
                    states[cid] = "%-6s 位%5.1fm 车点%6d 偏差%7.4f %-4s fps%4.1f 帧%5d" % (
                        dmo.get("phase", "-"), float(dmo.get("y_offset") or 0.0),
                        int(stb.get("n_vehicle_points") or 0), dev,
                        (stb.get("state_cn") or "-")[:3],
                        float(c.get("fps") or 0.0), int(c.get("frames") or 0))
                if not args.quiet:
                    line = "t=%5.1fs | %s | 结果 %d" % (
                        now - t0,
                        " | ".join("%s %s" % (cid, states.get(cid, "?")) for cid in ids),
                        sum(len(v["results"]) for v in per.values()))
                    print(line)
            # ★ 必须真的睡一下。忙等（while 空转）会一直攥着 GIL，
            #   把 4 条工作线程饿到几乎跑不动 —— 实测帧数会从 2.1 fps 掉到 0.3 fps，
            #   看起来就像"编排器把车开走了、画面还停在原地"。
            #   这个坑是彩排脚本自己制造的，不能在脚本里重演。
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n（被中断，先给已有结论）")
    finally:
        svc.stop()

    # ── 结论 ────────────────────────────────────────────────
    print("-" * 78)
    print("结论")
    print("-" * 78)
    ok = True
    all_plates = set()
    for cid in ids:
        d = per[cid]
        rs = d["results"]
        kinds = set(("空车" if r["empty"] else "载货") for r in rs)
        all_plates.update(d["plates"])
        line = (" %s : 出结论 %d 次 | 判定 %s | 车牌 %s"
                % (cid, len(rs),
                   ", ".join(sorted(kinds)) if kinds else "（无）",
                   ", ".join(sorted(set(d["plates"]))) if d["plates"] else "（无）"))
        print(line)
        if not rs:
            ok = False
            print("        × 这一路一次结论都没出 —— 演示时它就是一张静止的卡片，"
                  "请检查 ROI / vehicle_threshold / 编排器是否被暂停")
        else:
            if len(kinds) < 2:
                print("        ▲ 只出现过一种判定（%s）；"
                      "交替空车/载货没生效？" % ", ".join(kinds))
            if d["phases"] and len(d["phases"]) < 3:
                print("        ▲ 只走了 %d 个阶段：%s"
                      % (len(d["phases"]), ", ".join(sorted(d["phases"]))))
    print(" 四路车牌合计 : %d 张 → %s" % (len(all_plates), ", ".join(sorted(all_plates))))
    if len(all_plates) < len(ids):
        print("      ▲ 车牌数少于通道数，说明各路起牌没有错开（看板会四张卡同时显示同一张牌）")
    print(" error 事件   : %d" % len(errors))
    for e in errors[:5]:
        print("      t=%.1fs %s %s" % (e["t"], e["channel"], e["error"]))
        ok = False
    if warns:
        print(" warn 事件    : %d 条（前 3 条）" % len(warns))
        for w in warns[:3]:
            print("      " + str(w)[:110])
    if dir_status:
        print(" 编排器       : 周期 %.0fs，auto=%s，阶段 %s"
              % (dir_status.get("cycle_sec", 0), dir_status.get("auto"),
                 " → ".join(p["name"] for p in dir_status.get("phases", []))))
    print("=" * 78)
    print(" 彩排%s" % ("通过 √" if ok else "未通过 ×"))

    if args.json_path:
        outp = {"ok": ok, "sec": args.sec, "channels": ids,
                "per_channel": {k: {"results": v["results"],
                                    "plates": sorted(set(v["plates"])),
                                    "phases": sorted(v["phases"]),
                                    "max_dev": round(v["max_dev"], 5),
                                    "min_dev_after_stable": v["min_dev_after_stable"]}
                                for k, v in per.items()},
                "errors": errors, "warns": warns[:20], "director": dir_status}
        with open(args.json_path, "w", encoding="utf-8") as f:
            json.dump(outp, f, ensure_ascii=False, indent=2)
        print(" 结论已写入 : %s" % args.json_path)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
