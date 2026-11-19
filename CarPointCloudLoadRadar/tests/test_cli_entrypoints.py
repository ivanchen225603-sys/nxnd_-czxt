#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_cli_entrypoints.py — 每个命令行入口都要能起来

为什么需要这组用例：`cli/` 下的脚本是**现场唯一的操作界面**，而它们
一行单元测试都没覆盖到。实际踩过的坑：给 `cli/run.py` 加了个 `--quiet`，
但 `_common.add_common_args` 里已经有一个同名参数 ——
argparse 直接抛 `conflicting option string`，**脚本连启动都启动不了**，
而全套 428 个用例照样全绿（没有一个用例去构造过这些解析器）。

这里用 `--help` 把每个脚本拉起来一次：能覆盖
    · 模块级 import 写错 / 缺依赖
    · argparse 参数重名、互斥组写错
    · 模块级代码在导入时就炸（比如 cli/inspect.py 遮蔽标准库 inspect 那次）
跑一次约几百毫秒，换的是"现场脚本起不来"这类问题当场就红。
"""
from __future__ import annotations

import os
import subprocess
import sys
import unittest

from tests import _helpers as H                # noqa: F401  （把项目根加进 sys.path）
from app.settings import root

CLI_DIR = root("cli")
TOOLS = [root("tools", "configure_radars.py"), root("tools", "pc_tool.py"),
         root("desktop", "app.py")]      # 桌面版入口也有一套参数，同样会撞名


def _scripts():
    out = []
    for f in sorted(os.listdir(CLI_DIR)):
        if f.endswith(".py") and not f.startswith("_"):
            out.append(os.path.join(CLI_DIR, f))
    out += [p for p in TOOLS if os.path.exists(p)]
    return out


def _run_help(path: str):
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"           # tools/ 下的脚本不调 setup_console
    env["PYTHONPATH"] = root()
    return subprocess.run([sys.executable, path, "--help"],
                          capture_output=True, text=True, encoding="utf-8",
                          errors="replace", timeout=120, env=env, cwd=root())


class TestCliEntrypoints(unittest.TestCase):

    def test_every_script_has_a_working_help(self):
        bad = []
        for p in _scripts():
            r = _run_help(p)
            if r.returncode != 0:
                tail = (r.stderr or r.stdout or "").strip().splitlines()[-3:]
                bad.append(f"{os.path.basename(p)} → 退出码 {r.returncode}: "
                           + " | ".join(tail))
        self.assertFalse(bad, "这些命令行入口起不来：\n  " + "\n  ".join(bad))

    def test_help_mentions_the_script_name(self):
        """顺带确认 --help 真的是这个脚本的用法，而不是别人的（epilog 复制粘贴错）。"""
        for p in _scripts():
            name = os.path.basename(p)
            r = _run_help(p)
            self.assertIn(name.split(".")[0], (r.stdout or "") + (r.stderr or ""),
                          f"{name} 的 --help 里没出现自己的名字")


if __name__ == "__main__":
    unittest.main()
