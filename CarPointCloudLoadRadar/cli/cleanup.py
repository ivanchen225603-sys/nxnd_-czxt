#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/cleanup.py — 看看落盘占了多少，按需清理过期样本

用法

    # ① 只看占用（**什么都不删**，默认就是这个）
    python cli/cleanup.py

    # ② 看看"删 90 天前的"会删掉哪些（仍然什么都不删）
    python cli/cleanup.py --days 90
    python cli/cleanup.py --auto-days          # 天数取配置里的 service.keep_days

    # ③ 真删（唯一会删东西的写法）
    python cli/cleanup.py --days 90 --apply

    # ④ 只处理某条通道
    python cli/cleanup.py --channel CH1 --days 30 --apply

桌面版（打包成 exe，现场没有 Python）用的是同一套逻辑：

    载重雷达检测系统.exe --cleanup
    载重雷达检测系统.exe --cleanup --days 90 --apply
    或双击《清理过期样本.bat》

为什么要有这个脚本
    `service.keep_days` 在配置里写着 90，但程序**不会自动清理** —— 这是刻意的：
    检测记录该留多久是业务决定，不该由程序替人删。

安全边界（三条硬规矩，实现在 app/storage.py）
    1. **默认不删**。不写 `--apply` 就只打印。
    2. 只动各通道自己的 `output_dir`，且**拒绝**碰 `baseline`
       —— 那是 cli/verify.py 的回归基线（10 份原版样本），删了回归闸门会变成
       "0 个样本全部通过"而静默失效。
    3. 只删 `<时间>.xyz` 和它的 `.reason.txt` / `.meta.json`，三件一起删；
       其它文件一律不碰。清完顺手删掉空的车牌目录。
"""
from __future__ import annotations

import argparse
import os
import time
from typing import Dict, List

import _common
_common.setup_console()

from app.storage import (dir_usage, human, is_baseline,                    # noqa: E402
                         prune_empty_dirs, remove_sample, sample_is_empty, scan, summarize)

__all__ = ["dir_usage", "human", "is_baseline", "prune_empty_dirs", "sample_is_empty",
           "remove_sample", "scan", "summarize", "report", "main"]

_FMT = "%Y-%m-%d %H:%M"


def report(label: str, out_dir: str, days: float, apply: bool) -> Dict:
    """打印一个目录的盘点结果；apply=True 时才真删。"""
    info = summarize(out_dir, days)

    print(f"\n{label}")
    print(f"  目录     : {out_dir}" + ("" if info["exists"] else "   （还不存在）"))
    print(f"  样本     : {info['samples']} 份，占用 {human(info['bytes'])}")
    if info["samples"]:
        print(f"  时间跨度 : {time.strftime(_FMT, time.localtime(info['oldest']))}"
              f" … {time.strftime(_FMT, time.localtime(info['newest']))}")
        if info["per_day"]:
            print(f"  增长     : 约 {info['per_day']:.0f} 份/天，"
                  f"{human(info['bytes_per_day'])}/天")

    # ★ 空车样本**不按天数删**（2026-09-18）。
    #   现场要求是"最新的空车车厢永久保存、载重云图只留最近 N 天"，
    #   而按 mtime 一刀切会把空车一起删掉 —— 空车点云是基线的原始材料，
    #   删了就再也补不回来（那辆车得重新开进来标定一次）。
    #   判据读 meta 的 `empty`（判定链结论，不是 fit_ok），读不出来时保留。
    kept_empty = 0
    if days > 0 and info["old"]:
        survivors = []
        for item in info["old_items"]:
            if sample_is_empty(item[0]):
                kept_empty += 1
            else:
                survivors.append(item)
        info = dict(info, old_items=survivors, old=len(survivors),
                    old_bytes=sum(item[2] for item in survivors))

    removed_files = removed_bytes = removed_dirs = 0
    if days <= 0:
        print("  清理     : 未指定 --days，仅报告")
    else:
        if kept_empty:
            print(f"  空车保护 : {kept_empty} 份判为空车的样本不参与清理（永久保留）")
        if not info["old"]:
            print(f"  清理     : 没有超过 {days:g} 天的**载货**样本")
        elif not apply:
            print(f"  清理     : 将删除 {info['old']} 份（{human(info['old_bytes'])}）"
                  f" —— 这是**预演**，加 --apply 才真删")
            for p, mt, _s in sorted(info["old_items"], key=lambda x: x[1])[:5]:
                print(f"             {time.strftime(_FMT, time.localtime(mt))}  "
                      f"{os.path.relpath(p, out_dir)}")
            if info["old"] > 5:
                print(f"             …… 共 {info['old']} 份")
        else:
            for p, _mt, _s in info["old_items"]:
                n, b = remove_sample(p)
                removed_files += n
                removed_bytes += b
            removed_dirs = prune_empty_dirs(out_dir) if removed_files else 0
            print(f"  清理     : 已删除 {info['old']} 份样本 / {removed_files} 个文件，"
                  f"释放 {human(removed_bytes)}"
                  + (f"，顺带清掉 {removed_dirs} 个空目录" if removed_dirs else ""))

    archive = report_archive(out_dir, days, apply)

    return {"label": label, "samples": info["samples"], "bytes": info["bytes"],
            "old": info["old"], "old_bytes": info["old_bytes"], "kept_empty": kept_empty,
            "removed_files": removed_files, "removed_bytes": removed_bytes,
            "removed_dirs": removed_dirs, "archive": archive}


def report_archive(out_dir: str, days: float, apply: bool) -> Dict:
    """成套归档单独清理 —— `storage.scan` 看到 archives-v1 一律跳过。

    ★ 这是 2026-09-18 补上的：逐 XYZ 的清理**绝不能拆成套包**（那是对的，
      包是不可变的整体），于是归档库从来没有任何删除路径，**只涨不删**。
      现场要的是"载重留 N 天、空车永久"，所以按同一个天数清，
      但空车、本通道最新空车、以及还没落定（pending）的包一律不删。
    """
    directory = os.path.join(out_dir, "archives-v1")
    if not os.path.isdir(directory):
        return {"exists": False}
    from app.cloud_archive import ArchiveStore
    store = None
    try:
        store = ArchiveStore(out_dir)
        info = store.prune(time.time() - days * 86400.0, apply=bool(apply and days > 0))
    except Exception as exc:                                 # noqa: BLE001
        print(f"  成套归档 : 打不开（{type(exc).__name__}），跳过 —— 服务在跑时先停服务再清理")
        return {"exists": True, "error": type(exc).__name__}
    finally:
        if store is not None:
            try:
                store.close()
            except Exception:                                # noqa: BLE001
                pass
    print(f"  成套归档 : {info['scanned']} 个包，"
          f"空车保留 {info['kept_empty']}、未落定保留 {info['kept_pending']}、"
          f"未到期保留 {info['kept_recent']}")
    if days <= 0:
        print("             未指定 --days，仅报告")
    elif not info["removable"]:
        print(f"             没有超过 {days:g} 天可清的载货包")
    elif info["applied"]:
        print(f"             已删除 {info['removed']} 个包，释放 {human(info['removed_bytes'])}")
    else:
        print(f"             将删除 {len(info['removable'])} 个包（{human(info['removed_bytes'])}）"
              f" —— **预演**，加 --apply 才真删")
    return info


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="cleanup.py", description="查看落盘占用 / 清理过期样本（默认只看不删）",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    _common.add_common_args(ap)
    ap.add_argument("--days", type=float, default=0.0,
                    help="保留天数；超过这个天数的样本会被列出（不加 --apply 不会删）。"
                         "留空或 0 = 只报告占用；用 --auto-days 可直接取配置里的 keep_days")
    ap.add_argument("--auto-days", action="store_true",
                    help="用 config/channels.json 里的 service.keep_days 作为保留天数")
    ap.add_argument("--apply", action="store_true",
                    help="★ 真的删除（不加这个就只是预演）")
    args = ap.parse_args(argv)

    ctx = _common.load_ctx(args, need_calib=False)
    days = float(args.days or 0.0)
    if args.auto_days:
        days = float(getattr(ctx.settings.service, "keep_days", 0) or 0)
        print(f"按配置的 service.keep_days = {days:g} 天")

    print("=" * 78)
    print("落盘占用 / 清理" + ("（预演，不会删任何东西）" if not args.apply else "（★ 实删）"))
    print("=" * 78)

    # ★ 多条通道常常共用同一个 output_dir（本部署 4 路都写 data/field）。
    #   必须按目录去重，否则同一批文件会被数 4 遍、"占用"直接虚报 4 倍，
    #   --apply 时还会对同一个目录重复扫描。
    groups: Dict[str, List[str]] = {}
    shown: Dict[str, str] = {}          # 规范化路径 → 原样路径（打印用，别让盘符变小写）
    for c in ctx.channels:
        key = os.path.normcase(os.path.abspath(c.abs_output_dir))
        groups.setdefault(key, []).append(f"{c.id} {c.name}")
        shown.setdefault(key, os.path.abspath(c.abs_output_dir))

    rows = []
    for key, owners in groups.items():
        out_dir = shown[key]
        label = "【" + " / ".join(owners) + "】"
        if is_baseline(out_dir):
            print(f"\n{label}")
            print(f"  × 拒绝处理 {out_dir}")
            print("    这是回归基线目录（cli/verify.py 的 10 份原版样本）。")
            print("    把 config/channels.json 里这些通道的 output_dir 改成别的目录"
                  "（例如 data/field）再来。")
            return 2
        rows.append(report(label, out_dir, days, args.apply))

    # ── 其它占用：磁盘真正被谁吃掉，往往不是点云 ──
    from app.settings import root as _root
    print("\n" + "-" * 78)
    print("其它占用（本脚本**不删**，需要时手工清理）")
    for d, note in ((_root("logs"), "运行日志 / 导出 / 浏览器窗口用户数据"),
                    (_root("data"), "含上面的样本目录与空车基线")):
        u = dir_usage(d)
        if not u["exists"]:
            continue
        print(f"  {os.path.basename(d):<6} {human(u['bytes']):>10}"
              f"  （{u['files']} 个文件）  {note}")
        detail = "，".join(f"{k} {human(v)}" for k, v in u["top"])
        if detail:
            print(f"         大头：{detail}")
    print("  提示：logs/desktop-profile 是浏览器窗口的用户数据目录，删掉会自动重建；")
    print("        logs/export 是每次点『导出 PLY/XYZ』留下的，也可以随时删。")

    print("-" * 78)
    tot = sum(r["bytes"] for r in rows)
    print(f"合计 {sum(r['samples'] for r in rows)} 份样本，占用 {human(tot)}")
    if days > 0 and not args.apply:
        n_old = sum(r["old"] for r in rows)
        if n_old:
            print(f"加 --apply 可删除 {n_old} 份、释放 "
                  f"{human(sum(r['old_bytes'] for r in rows))}")
    if args.apply:
        print(f"本次释放 {human(sum(r['removed_bytes'] for r in rows))}")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
