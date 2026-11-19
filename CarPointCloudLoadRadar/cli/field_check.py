#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/field_check.py —— 现场自查：`docs/41` 的清单，今天答掉了几条？

采集过程中随时跑。它只做两件事：
  ① 读正在跑的服务状态（HTTP `/api/state`）—— 帧率、残缺帧占比、整帧被挤掉多少；
  ② 扫成套归档 `data/field/<CH>/archives-v1/` —— 今天测了几辆、结论分布、
     哪些项还缺样本。

**它不替你下结论**，只回答"还差什么、现在能不能收工"。
拿不到的项会如实说"离线答不了"，不会假装通过。

用法
    python3 cli/field_check.py                      # 默认读 127.0.0.1:8080 + data/field
    python3 cli/field_check.py --url http://10.0.0.5:8080
    python3 cli/field_check.py --no-live            # 只看归档，不连服务
"""
from __future__ import annotations

import argparse
import collections
import io
import json
import os
import sys
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common                                                   # noqa: E402
_common.setup_console()
from app.settings import load_settings                           # noqa: E402

LANES = ("CH2", "CH3", "CH4")
OK, BAD, WAIT = "[Y]", "[X]", "[ ]"


def fetch_state(url):
    try:
        with urllib.request.urlopen(url.rstrip("/") + "/api/state", timeout=3) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception as error:                                   # noqa: BLE001
        return {"_error": "%s: %s" % (type(error).__name__, error)}


def scan_archives(root):
    """扫归档里的 detection.xyz.meta.json，按通道汇总今天测到了什么。"""
    out = collections.defaultdict(list)
    for lane in LANES:
        base = os.path.join(root, lane, "archives-v1")
        if not os.path.isdir(base):
            continue
        for dirpath, _dirs, files in os.walk(base):
            if "detection.xyz.meta.json" not in files:
                continue
            try:
                with io.open(os.path.join(dirpath, "detection.xyz.meta.json"),
                             encoding="utf-8") as handle:
                    meta = json.load(handle)
            except Exception:                                    # noqa: BLE001
                continue
            meta["_dir"] = dirpath
            meta["_raw"] = sum(1 for name in files
                               if name.startswith("raw-") and name.endswith(".xyz"))
            out[meta.get("channel") or lane].append(meta)
    return out


def gate_row(meta, step):
    for row in meta.get("gates") or []:
        if row.get("step") == step:
            return row
    return {}


def main():
    parser = argparse.ArgumentParser(description="现场自查：docs/41 清单还差什么")
    parser.add_argument("--url", default="http://127.0.0.1:8080", help="看板地址")
    parser.add_argument("--root", default="", help="输出根（默认 data/field）")
    parser.add_argument("--no-live", action="store_true", help="不连服务，只看归档")
    args = parser.parse_args()

    settings = load_settings("")
    root = args.root or "data/field"
    state = {} if args.no_live else fetch_state(args.url)
    chans = {c.get("id"): c for c in (state.get("channels") or [])}
    arch = scan_archives(root)

    print("=" * 74)
    print("现场自查 —— 归档根 %s" % os.path.abspath(root))
    if state.get("_error"):
        print("  ! 连不上服务（%s）—— 实时项全部跳过，只报归档能答的" % state["_error"])
    print("=" * 74)

    if chans:
        print()
        print("【实时】0 源IP / 0b 残缺帧 / 0c 帧率 / 整帧是否被挤掉")
        print("  %-5s %6s %8s %7s %12s %8s  %s" % (
            "车道", "有数据", "累计帧", "帧率", "残缺帧占比", "整帧丢弃", "健康横幅"))
        for lane in LANES:
            chan = chans.get(lane)
            if chan is None:
                print("  %-5s %6s  快照里没有这条通道" % (lane, BAD))
                continue
            ratio = float(chan.get("incomplete_ratio") or 0.)
            # ▲▲ **不能看 online** —— 源 IP 配错的通道 `online` 一样是 True，
            #   只是 fps/frames 恒为 0 且不报错（CLAUDE.md 设计第 1 条那个
            #   "卡片长得和车道空着一模一样"的失效形态）。只有"收到过帧"作数。
            frames = int(chan.get("frames") or 0)
            print("  %-5s %6s %8d %7.1f %9.1f%% %s %8d  %s" % (
                lane, OK if frames > 0 else BAD, frames, float(chan.get("fps") or 0.),
                ratio * 100, OK if ratio < .03 else BAD,
                int(chan.get("raw_dropped") or 0), (chan.get("health") or "")[:26] or "-"))
        print("    判读：**累计帧=0 就是这一路没数据**（源 IP 多半配错，而它不报错）；")
        print("          残缺帧占比 <=3% 才算过；整帧丢弃 >0 —— 那几辆的『框外面』查不了了。")

    print()
    print("【归档】今天各车道测到了什么")
    print("  %-5s %6s %6s %8s %8s %8s %8s" % (
        "车道", "测量数", "整帧", "判空车", "判有货", "证据不足", "有车牌"))
    total = collections.Counter()
    for lane in LANES:
        rows = arch.get(lane, [])
        verdicts = collections.Counter(r.get("verdict") or "?" for r in rows)
        raw_ok = sum(1 for r in rows if r["_raw"] >= 3)
        plate = sum(1 for r in rows if (r.get("plate") or "").strip()
                    and r.get("plate") != "未知车牌")
        total.update(verdicts)
        total["n"] += len(rows)
        total["plate"] += plate
        print("  %-5s %6d %6d %8d %8d %8d %8d" % (
            lane, len(rows), raw_ok, verdicts.get("PASS_EMPTY", 0),
            verdicts.get("REJECT_LOADED", 0), verdicts.get("NO_EVIDENCE", 0), plate))

    rows = [r for lane in LANES for r in arch.get(lane, [])]

    def line(tag, done, text):
        print("  %s %-24s %s" % (OK if done else WAIT, tag, text))

    print()
    print("【docs/41 逐条】")
    got = {lane: int(chans.get(lane, {}).get("frames") or 0) for lane in LANES}
    line("0  源 IP", bool(chans) and all(got[lane] > 0 for lane in LANES),
         ("累计帧 " + " / ".join("%s=%d" % (lane, got[lane]) for lane in LANES)
          + ("" if all(got.values()) else "  <- 为 0 的那一路没收到包，先查源 IP"))
         if chans else "需连上服务")
    ratios = [float(chans[l].get("incomplete_ratio") or 0.) for l in LANES if l in chans]
    line("0b 残缺帧占比", bool(ratios) and max(ratios) < .03,
         ("最高 %.1f%%" % (max(ratios) * 100)) if ratios else "需连上服务")
    fps = [float(chans[l].get("fps") or 0.) for l in LANES if l in chans]
    line("0c(1) 实际帧率", bool(fps) and min(fps) > 0,
         ("%s Hz" % "/".join("%.1f" % x for x in fps)) if fps else "需连上服务")
    line("0c(2) 相邻帧位移", False,
         "离线答不了：归档整帧间隔 1.5 s，而 immediate_speed 量的是相邻帧（约 100 ms）")
    line("0d 形变门槛", False, "同上，需要 100 ms 的连续帧")
    inliers = [gate_row(r, 6).get("inliers") for r in rows]
    inliers = sorted(x for x in inliers if isinstance(x, int))
    line("0e 内点门槛", len(inliers) >= 30,
         ("已收 %d 份内点数（中位 %d），>=30 份才够定门槛" % (len(inliers), inliers[len(inliers) // 2]))
         if inliers else "还没有样本")
    empties = [r for r in rows if r.get("verdict") == "PASS_EMPTY"]
    line("0f 空车真值", len(empties) >= 5,
         "今天判空车 %d 辆 —— 每辆都要现场确认并打标（全项目原有仅 2 份）" % len(empties))
    high = [r for r in rows if "底面过高" in (r.get("reason") or "")]
    line("0g 底面过高定性", not high,
         ("今天 %d 辆报『底面过高，疑似载物』—— 要看一眼车：空挂车还是装了一半" % len(high))
         if high else "今天还没遇到")
    line("6  车牌接口", total["plate"] > 0, "带车牌的测量 %d 份" % total["plate"])
    npz = [x for x in os.listdir("data/in") if x.endswith(".npz")] \
        if os.path.isdir("data/in") else []
    line("7  空车基线", bool(npz), "data/in/*.npz：%d 份" % len(npz))

    print()
    print("【收工前必做】")
    print("  1. 当班实际车次 vs 上面『测量数』—— 差值就是**静默漏测**，不比对看不出来")
    print("  2. 判空车的 %d 辆逐辆确认 + python3 cli/label_truth.py 打标" % len(empties))
    print("  3. 记下三路当时的 yaw（归档包**没记**解码 yaw，事后改了对不上）：")
    for chan in settings.channels:
        if chan.id in LANES:
            print("       %s = %s" % (chan.id, chan.lidar.yaw_offset_deg))
    print("  4. python3 cli/roi_audit.py --dir %s/<CH>/archives-v1" % root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
