#!/usr/bin/env python3
"""独立单包体积复算：默认只输出报告，不连接后台或写回归档。"""
import argparse
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.archive_io import ArchiveReadLimits
from app.archive_reader import load_archive
from app.sample_paths import _read_package
from app.volume_replay import recompute


def main(argv=None):
    parser = argparse.ArgumentParser(description="按原资产和参数离线复算单份测量归档")
    parser.add_argument("--archive", required=True, help="含manifest.json的单份归档目录")
    parser.add_argument("--json", help="另存到不存在的独立报告文件；禁止写入归档")
    args = parser.parse_args(argv)
    try:
        directory = Path(args.archive).resolve(strict=True)
        # 原输入路径也须经链接检查，不能用resolve掩盖显式链接。
        sample = _read_package(Path(os.path.abspath(args.archive)), "observation", ArchiveReadLimits())
        report = recompute(load_archive(sample))
        text = json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
        if args.json:
            target = Path(args.json).resolve()
            if target == directory or directory in target.parents:
                raise ValueError("报告不能写入原归档")
            with target.open("x", encoding="utf-8") as stream:
                stream.write(text + "\n")
        print(text)
        return 0 if report.get("matched") is True else 2
    except (OSError, ValueError, KeyError, TypeError) as error:
        print("体积复算失败: " + str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
