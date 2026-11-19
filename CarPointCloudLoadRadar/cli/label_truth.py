# -*- coding: utf-8 -*-
"""给落盘样本打「空车 / 载货」的人工标注 —— 判定链唯一的真值来源。

为什么需要这个工具
------------------
全项目到 2026-09-16 为止，**空车真值只有 2 份**（2026-09-15 下午的两辆基线车）。
而判定链最后一步的门槛（`floor_gate_z_above_min_max_m = 0.08`）正是靠它标定的，
余量只有 1.3~1.4 倍。样本荒是「是否空车」这件事最大的短板，不是算法。

`cli/verify.py` 的 10 份基线也**不是真值** —— 那是原厂的输出，
而其中 4 份本身就是空车、被原厂误判成了载货（见 docs/12 §4.1）。
所以"攒真值"这件事没有捷径，只能一辆一辆由人确认。

成本几乎为零：现场标定空车基线本来就要停车，顺手在这里敲一个字母。

标注存在哪
----------
`data/truth/labels.csv`，一行一份样本：

    sample,label,by,at,note
    未知车牌/20260915_161309.xyz,empty,张三,2026-09-16 10:12,宁A81530M 空车基线

* `sample` 是相对 `--dir`（默认 `data/field`）的路径，**跨机器可搬**；
* 点云本身一个字节都不动，标注与数据分离；
* 已有标注不会被静默覆盖，要改得加 `--force`。

用法
----
    python3 cli/label_truth.py                    # 列出待标注的（服务标了 needs_truth_label 的优先）
    python3 cli/label_truth.py --all              # 把所有样本都列出来
    python3 cli/label_truth.py --stats            # 只看已标注的统计
    python3 cli/label_truth.py --set <样本> empty  # 直接给某一份打标
    python3 cli/label_truth.py --interactive      # 逐份看诊断数字再敲标签

标签取值
--------
    empty    空车
    loaded   载货
    unsure   拿不准（人看了也说不清，比如车厢里有一层残料）

▲ `unsure` 不是逃避 —— 它恰恰是最该记下来的：
  "残料到什么程度算空"属于**业主要定的口径**，不是技术问题（docs/12 §7 第 5 条）。
  攒一批 unsure 拿去问，比技术侧自己拍一个门槛靠谱。
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cli import _common                      # noqa: E402
from app.settings import (config_fingerprint, config_mismatch,   # noqa: E402
                          load_calib, load_devices, load_settings)
from app.sample_paths import discover_samples

LABELS = ("empty", "loaded", "unsure")
# (过期份数, 样本目录) —— mark_stale 填，cmd_stats 读
_STALE = [0, ""]
LABEL_CN = {"empty": "空车", "loaded": "载货", "unsure": "拿不准"}
CSV_NAME = os.path.join("truth", "labels.csv")
FIELDS = ("sample", "label", "by", "at", "note")


def csv_path(data_root: str) -> str:
    return os.path.join(data_root, CSV_NAME)


def load_labels(data_root: str) -> dict:
    """读已有标注。文件不存在就返回空 dict（第一次跑很正常）。"""
    p = csv_path(data_root)
    if not os.path.exists(p):
        return {}
    out = {}
    with io.open(p, encoding="utf-8-sig", newline="") as f:
        for row in csv.DictReader(f):
            k = (row.get("sample") or "").strip()
            if k:
                out[k] = {x: (row.get(x) or "").strip() for x in FIELDS}
    return out


def save_labels(data_root: str, labels: dict) -> str:
    p = csv_path(data_root)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with io.open(p, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for k in sorted(labels):
            row = dict(labels[k])
            row["sample"] = k
            w.writerow({x: row.get(x, "") for x in FIELDS})
    return p


def scan(sample_dir: str) -> list:
    """每个新归档只标注判定云；旧样本仍读旁边的 meta，排除 raw。"""
    out = []
    found = discover_samples(sample_dir, role="detection", exclude_raw=True)
    found.report()
    for sample in found.samples:
        f = sample.path
        meta = {}
        mp = f + ".meta.json"
        if os.path.exists(mp):
            try:
                with io.open(mp, encoding="utf-8") as fp:
                    meta = json.load(fp)
            except Exception:                 # noqa: BLE001
                meta = {}
        out.append((f, meta))
    return out


def mark_stale(items, data_root: str = "") -> int:
    """给每份样本标上「落盘时的配置与现在是否一致」。返回过期的份数。

    为什么要标（docs/14 §1.8）：meta 里的 `reason` / `empty` 是**快照**，
    配置一改就可能过期。2026-09-17 实测 62 份现场样本里有 **7 份（11%）**
    的 reason 与用当前配置重算的结果不一致，其中 2 份现在是「空车」
    而 meta 记的是「底面过高，疑似载物」——
    照过期的结论打标，等于把错的真值攒进 labels.csv。
    """
    try:
        st = load_settings("")
    except Exception:                                 # noqa: BLE001
        return 0
    cur = {}
    for c in st.channels:
        try:
            cur[c.id] = config_fingerprint(
                load_calib(c.abs_config_path), load_devices(c.abs_devices_path),
                getattr(c.lidar, "yaw_offset_deg", None))
        except Exception:                             # noqa: BLE001
            continue
    n = 0
    for _f, meta in items:
        cid = meta.get("channel")
        if cid not in cur:
            continue
        why = config_mismatch(meta.get("config"), cur[cid])
        if why:
            meta["_stale"] = why
            n += 1
    return n


def _brief(meta: dict) -> str:
    """一行诊断摘要 —— 让打标的人有依据，而不是凭印象。"""
    dg = meta.get("diagnostics") or {}
    bits = [str(meta.get("reason") or "?")]
    if meta.get("step"):
        bits.append("第%s步" % meta["step"])
    for k, label, fmt in (("ratio", "占比", "%.3f"), ("floor_z", "底面z", "%.2f"),
                          ("freeboard", "余高", "%.2f"),
                          ("floor_tilt_deg", "倾角", "%.1f度"),
                          ("z_above_used", "z_above", "%.3f")):
        v = dg.get(k)
        if isinstance(v, (int, float)):
            bits.append(("%s=" + fmt) % (label, v))
    if meta.get("frame_degraded"):
        bits.append("**残缺帧硬测**")
    if meta.get("duplicate_of"):
        bits.append("重复")
    if meta.get("_stale"):
        bits.append("**配置已变，结论可能过期**")
    return "  ".join(bits)


def cmd_stats(labels: dict, items: list) -> int:
    n = {k: 0 for k in LABELS}
    for v in labels.values():
        if v.get("label") in n:
            n[v["label"]] += 1
    print("已标注 %d 份 / 共扫到 %d 份样本" % (len(labels), len(items)))
    for k in LABELS:
        print("   %-8s %-6s %d 份" % (k, LABEL_CN[k], n[k]))
    print()
    # docs/12 §7 第 1 条：空车正样本 >= 20 份才谈得上标定门槛
    need = 20
    if n["empty"] >= need:
        print("√ 空车正样本 %d 份（>= %d）—— 可以重新标定第 10 步的门槛了，"
              "见 docs/12 §7" % (n["empty"], need))
    else:
        print("× 空车正样本只有 %d 份，离可以标定门槛的 %d 份还差 %d 份。"
              % (n["empty"], need, need - n["empty"]))
        print("  现场标定空车基线时顺手打标，成本几乎为零（docs/12 §7 第 1 条）。")
    if _STALE[0]:
        print()
        print("▲ 有 %d 份样本的落盘配置与当前配置**不一致**，它们记的 reason/empty"
              % _STALE[0])
        print("  可能已经过期。照过期结论打标 = 把错的真值攒进 labels.csv。")
        print("  先跑 `python3 cli/replay.py --dir %s` 用当前配置重算一遍再打标。"
              % _STALE[1])
    if n["unsure"]:
        print("▲ 有 %d 份拿不准 —— 这些正是要拿去问业主"
              "「残料到什么程度算空」的素材。" % n["unsure"])
    return 0


def main(argv=None):
    _common.setup_console()
    ap = argparse.ArgumentParser(description="给落盘样本打空车/载货的人工标注")
    ap.add_argument("--dir", default="data/field", help="样本目录（默认 data/field）")
    ap.add_argument("--data-root", default="data", help="labels.csv 放哪（默认 data/）")
    ap.add_argument("--all", action="store_true", help="列出全部，不只是待标注的")
    ap.add_argument("--stats", action="store_true", help="只看统计")
    ap.add_argument("--set", nargs=2, metavar=("样本", "标签"),
                    help="直接打标，标签取 empty / loaded / unsure")
    ap.add_argument("--by", default="", help="标注人")
    ap.add_argument("--note", default="", help="备注（比如车牌）")
    ap.add_argument("--force", action="store_true", help="允许覆盖已有标注")
    ap.add_argument("--interactive", action="store_true", help="逐份询问")
    args = ap.parse_args(argv)
    _common.ensure_root()

    sample_dir = args.dir
    labels = load_labels(args.data_root)
    items = scan(sample_dir)
    _STALE[0] = mark_stale(items)
    _STALE[1] = sample_dir
    if not items:
        print("在 %s 下没扫到 .xyz 样本。" % sample_dir)
        return 2

    if args.stats:
        return cmd_stats(labels, items)

    if args.set:
        key, lab = args.set
        lab = lab.strip().lower()
        if lab not in LABELS:
            print("标签只能是 %s，当前是 %r" % (" / ".join(LABELS), lab))
            return 2
        key = key.replace("\\", "/").strip()
        known = {os.path.relpath(f, sample_dir).replace("\\", "/") for f, _ in items}
        if key not in known:
            hit = [k for k in known if key in k]
            if len(hit) != 1:
                print("找不到样本 %r（或不唯一）。用不带 --set 的形式先列一下。" % key)
                return 2
            key = hit[0]
        if key in labels and not args.force:
            print("%s 已标注为 %r（%s）。要改加 --force。"
                  % (key, labels[key].get("label"), labels[key].get("by") or "?"))
            return 1
        import time as _t
        labels[key] = {"sample": key, "label": lab, "by": args.by,
                       "at": _t.strftime("%Y-%m-%d %H:%M"), "note": args.note}
        print("%s -> %s（%s）" % (key, lab, LABEL_CN[lab]))
        print("已写入 %s" % save_labels(args.data_root, labels))
        return 0

    rows = []
    for f, meta in items:
        rel = os.path.relpath(f, sample_dir).replace("\\", "/")
        have = labels.get(rel, {}).get("label", "")
        want = bool(meta.get("needs_truth_label"))
        if not args.all and (have or not want):
            continue
        rows.append((rel, have, want, meta))

    if not rows:
        print("没有待标注的样本。")
        print("（服务只把**信息量最大**的挑出来：判成空车的，"
              "以及第 10 步差一点点被拦的。用 --all 看全部。）")
        return cmd_stats(labels, items)

    print("=" * 96)
    print("待标注 %d 份（共扫到 %d 份，已标注 %d 份）"
          % (len(rows), len(items), len(labels)))
    print("=" * 96)
    for i, (rel, have, want, meta) in enumerate(rows, 1):
        print("%3d. %s%s" % (i, rel, "   <== 服务建议打标" if want else ""))
        print("     %s" % _brief(meta))
        if have:
            print("     已标注：%s" % have)

    if not args.interactive:
        print()
        print("打标：python3 cli/label_truth.py --set <样本> empty --by 你的名字")
        print("逐份问：python3 cli/label_truth.py --interactive")
        return 0

    import time as _t
    changed = 0
    for rel, have, _want, meta in rows:
        print()
        print("-" * 96)
        print(rel)
        print("   %s" % _brief(meta))
        if have:
            print("   已标注：%s（回车跳过，输入新标签覆盖）" % have)
        try:
            ans = input("   e=空车  l=载货  u=拿不准  回车=跳过  q=退出 > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n中断。")
            break
        if ans in ("q", "quit"):
            break
        lab = {"e": "empty", "l": "loaded", "u": "unsure"}.get(ans)
        if not lab:
            continue
        labels[rel] = {"sample": rel, "label": lab, "by": args.by,
                       "at": _t.strftime("%Y-%m-%d %H:%M"), "note": args.note}
        changed += 1
        print("   -> %s" % LABEL_CN[lab])
    if changed:
        print()
        print("标注了 %d 份，已写入 %s" % (changed, save_labels(args.data_root, labels)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
