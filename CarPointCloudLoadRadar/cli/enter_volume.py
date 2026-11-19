#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用进场帧补车厢两端，离线对比一份载货归档补与不补的体积（只读，不写任何文件）。

2026-10-24 起服务已接入进场帧（service.enter_volume，docs/92），并默认开遮挡上限补洞；
本工具按同一口径（fill_bound=True）离线对比单份归档补与不补，--no-fill-bound 退回旧松弛补洞。

做的事：
  1. 读归档（停稳帧 + 沉降窗口的额外货面帧），按服务同样的方式做载货拟合；
  2. 在配准前核对进场帧身份、完整性、坐标配置和文件内容，用主货面同一个旋转对齐；
  3. 相邻帧链式配准 + 逐帧扣高度（register_enter_frames），打印每帧位移与校核；
  4. compute_volume_auto 各算一遍：不补 / 只补停稳帧没测到的格。

▲ 体积的其余参数按服务的默认口径给（没有体积参数文件、没有独立底板证据），
  与服务里那一份测量的数可能有细微差别（服务用的是测量当时的那一份配置）。

用法
----
    python3 cli/enter_volume.py --archive data/field/archives-v1/<id>/<id>/<id> \\
        --plate 宁A05922F --enter data/field/enter/CH4/20260928_171809
    python3 cli/enter_volume.py --archive ... --baseline data/in/宁A05922F.baseline.npz --enter ...
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path

from _common import ensure_root, setup_console

ROOT = ensure_root()


def _frames(prefix: str):
    p = Path(prefix)
    files = list(p.parent.glob(p.name + "_*of*.xyz"))
    pat = re.compile(re.escape(p.name) + r"_(\d+)of(\d+)\.xyz$")
    keyed = sorted((int(m.group(1)), f) for f in files for m in [pat.search(f.name)] if m)
    return [f for _, f in keyed]


def main(argv=None) -> int:
    setup_console()
    ap = argparse.ArgumentParser(description="进场帧补两端：离线对比载货体积（只读）")
    ap.add_argument("--archive", required=True, help="单份载货归档目录（含 manifest.json）")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--plate", help="车牌：用 data/in/<车牌>.baseline.npz")
    g.add_argument("--baseline", help="空车基线 npz 路径")
    ap.add_argument("--enter", required=True,
                    help="进场帧前缀，如 data/field/enter/CH4/20260928_171809（不含 _<i>of<N>.xyz）")
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "config.json"))
    ap.add_argument("--actual-plate", help="自定义 --baseline 文件名不能表达真实车牌时显式指定")
    ap.add_argument("--no-fill-bound", action="store_true", help="不做遮挡上限补洞（旧松弛补洞口径）")
    ap.add_argument("--history-confirmation", help="历史弱证据回放的显式对应确认 JSON；不会豁免已知字段冲突")
    args = ap.parse_args(argv)

    from app import detect as D
    from app.archive_io import ArchiveReadLimits
    from app.archive_reader import load_archive
    from app.enter_validation import validate_enter_frames
    from app.plate import BASELINE_SUFFIX, baseline_file
    from app.sample_paths import _read_package
    from app.settings import load_calib
    from app.volume_inputs import align_extra_surfaces

    files = _frames(args.enter)
    if len(files) < 2:
        print("进场帧不足 2 帧：%s_*of*.xyz" % args.enter, file=sys.stderr)
        return 2
    base_path = args.baseline or baseline_file(os.path.join(ROOT, "data", "in"), args.plate)
    actual_plate = args.plate or args.actual_plate
    if not actual_plate and Path(base_path).name.endswith(BASELINE_SUFFIX):
        actual_plate = Path(base_path).name[:-len(BASELINE_SUFFIX)]
    try:
        if not actual_plate:
            raise ValueError("自定义基线文件须用 --actual-plate 指明实际车牌")
        archive = load_archive(_read_package(Path(os.path.abspath(args.archive)), "observation",
                                             ArchiveReadLimits()))
        confirmation = (json.loads(Path(args.history_confirmation).read_text(encoding="utf-8"))
                        if args.history_confirmation else None)
        verified = validate_enter_frames(archive, files, base_path, actual_plate, confirmation)
        baseline = D.EmptyBaseline.load(base_path)
        if baseline is None:
            raise ValueError("指定空车基线不存在")
        calib = load_calib(args.config)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print("输入校验失败：%s" % error, file=sys.stderr)
        return 2
    print("输入校验：" + ("历史弱证据（显式确认）" if verified.historical else "身份、配置及内容通过"))
    for note in verified.notes:
        print("  " + note)
    loaded = D.detect_loaded(archive.window().input_frame(), calib)
    if loaded is None or loaded.source_rotation is None:
        print("载货拟合没成功，取不到货面", file=sys.stderr)
        return 2
    extras = align_extra_surfaces(loaded.source_rotation,
                                  [s.frame().xyz for s in archive.volume_extra_inputs()])
    enter = align_extra_surfaces(loaded.source_rotation, verified.clouds)

    reg = D.register_enter_frames(loaded, enter)
    print("进场帧：%d 帧（%s）" % (len(files), files[0].parent))
    print("配准：%s（%s）" % (reg.status, reg.reason))
    if reg.shifts:
        print("  各帧位移（m，时间顺序）：" + " ".join("%+.2f" % s for s in reg.shifts))

    kw = dict(loaded_extra=extras or None, fill_bound=not args.no_fill_bound)
    plain = D.compute_volume_auto(baseline, loaded, **kw)
    print("\n不补进场帧：%.3f m^3" % plain.volume_m3)
    print("  " + plain.text)
    if reg.status != "resolved":
        print("\n进场帧没配上，不补。")
        return 1
    filled = D.compute_volume_auto(baseline, loaded, enter_extra=reg.clouds, **kw)
    print("\n补进场帧：%.3f m^3（%+.2f%%）" % (filled.volume_m3,
                                           (filled.volume_m3 / plain.volume_m3 - 1) * 100 if plain.volume_m3 else 0))
    print("  " + filled.text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
