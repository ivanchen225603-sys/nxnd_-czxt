#!/usr/bin/env python3
"""显式目录容量盘点；不打开数据库、不读取配置、不执行清理。"""
from __future__ import annotations

import argparse
import json
import ntpath
import os
from pathlib import Path
import sys

if __package__:
    from . import _common
else:
    import _common
_common.setup_console()

from app.archive_capacity import CapacityScanLimits, scan_capacity_files
from app.archive_io import _path_guard
from app.storage import is_baseline


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dir", action="append", required=True, help="显式根目录，可重复；不默认查找私有存储")
    parser.add_argument("--max-entries", type=int, default=100000, help="最多访问的目录项数")
    parser.add_argument("--seconds", type=float, default=5., help="协作式扫描时间预算，不是I/O硬超时")
    parser.add_argument("--json", default="", help="可选报告文件，必须位于输入目录之外且尚不存在")
    args = parser.parse_args(argv)
    try:
        limits = CapacityScanLimits(entries=args.max_entries, seconds=args.seconds)
    except ValueError as exc:
        parser.error(str(exc))
    if args.json:
        # Windows ADS可附着在目录本身，不能用 commonpath 判断它是否为子文件。
        # 同时拒绝设备命名空间，报告只允许普通的新文件。
        windows_path = args.json.replace("/", "\\")
        if ":" in ntpath.splitdrive(windows_path)[1] or windows_path.startswith(("\\\\?\\", "\\\\.\\")):
            parser.error("报告不允许备用数据流或设备命名空间")
        if os.name == "nt":
            reserved = {"CON", "PRN", "AUX", "NUL", *("COM%d" % i for i in range(1, 10)),
                        *("LPT%d" % i for i in range(1, 10))}
            if any(part.endswith((".", " ")) or part.split(".")[0].upper() in reserved
                   for part in ntpath.splitdrive(windows_path)[1].split("\\") if part):
                parser.error("报告路径含Windows保留名称或不规范后缀")
        target = os.path.normcase(os.path.realpath(args.json))
        protected = os.path.normcase(os.path.realpath(Path(__file__).resolve().parents[1] / "re"))
        if is_baseline(target) or target == protected or target.startswith(protected + os.sep):
            parser.error("禁止向回归基线或只读取证目录写报告")
        try:
            _path_guard(Path(os.path.abspath(args.json)).parent)
        except (OSError, ValueError):
            parser.error("报告父目录不可用或穿过链接")
        for directory in args.dir:
            source = os.path.normcase(os.path.realpath(directory))
            try:
                if os.path.commonpath((source, target)) == source:
                    parser.error("报告不能写入输入目录")
            except ValueError:
                pass
        if os.path.lexists(args.json):
            parser.error("报告文件已存在，不覆盖")
    try:
        result = scan_capacity_files(args.dir, limits=limits)
    except ValueError as exc:
        parser.error(str(exc))
    print("文件盘点：%s；%d 个独立文件，表观 %d 字节。" %
          (result["status"], result["files"], result["apparent_bytes"]))
    print("数据库引用和运行时未提交状态未检查；不是可回收空间，不能据此删除。")
    rendered = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    print(rendered)
    if args.json:
        try:
            _path_guard(Path(os.path.abspath(args.json)).parent)
            with open(args.json, "x", encoding="utf-8") as stream:
                stream.write(rendered + "\n")
        except (OSError, ValueError) as exc:
            print("报告保存失败：" + type(exc).__name__, file=sys.stderr)
            return 2
    return 0 if result["status"] == "complete" else 2


if __name__ == "__main__":
    sys.exit(main())
