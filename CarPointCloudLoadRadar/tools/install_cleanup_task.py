#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/install_cleanup_task.py — 把「按天清理」挂成每天自动执行的计划任务

用法

    python tools/install_cleanup_task.py              # 只看要装什么（**不写**）
    python tools/install_cleanup_task.py --apply      # 真装
    python tools/install_cleanup_task.py --remove --apply   # 卸掉
    python tools/install_cleanup_task.py --at 03:30 --apply # 改执行时刻

为什么需要这个脚本
    现场要求是「载重云图只保留最近 N 天」，而 `cli/cleanup.py` **默认只报不删**，
    非要 `--days N --apply` 才动手 —— 这是刻意的设计（"留多久是业务决定，
    不该由程序替人删"），不打算改。
    但"只保留 N 天"作为一条要求，是要**有人/有东西定期去执行**的。
    与其把自动删除塞进采集服务（删除混在采集进程里，出问题最难查），
    不如把它挂成操作系统的计划任务：**删除仍然是一个显式、可审计、可单独停掉
    的动作**，日志也单独一份。

    天数取 `config/channels.json` 的 `service.keep_days`（现为 7），
    用的是 `--auto-days`，所以以后改配置不用重装任务。

▲ 空车不受影响：`cli/cleanup.py` 会跳过判为空车的样本与归档，
  基线本体 `data/in/*.npz` 更是不在清理范围内。见 CLAUDE.md 的「保留策略」。
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TASK_NAME = "LoadRadarCleanup"


def _cleanup_argv() -> list:
    """真正要跑的那条命令。打包版走 exe，源码版走 python。"""
    exe = os.path.join(ROOT, "载重雷达检测系统.exe")
    if os.path.isfile(exe):
        return [exe, "--cleanup", "--auto-days", "--apply"]
    return [sys.executable, os.path.join(ROOT, "cli", "cleanup.py"), "--auto-days", "--apply"]


def _quote(argv: list) -> str:
    return " ".join(('"%s"' % a) if (" " in a or "\\" in a) else a for a in argv)


def _run(argv: list, apply: bool) -> int:
    print("    " + _quote(argv))
    if not apply:
        return 0
    try:
        done = subprocess.run(argv, capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.SubprocessError) as exc:
        print("    × 执行失败：%s: %s" % (type(exc).__name__, exc))
        return 1
    out = (done.stdout or "") + (done.stderr or "")
    for line in out.splitlines():
        if line.strip():
            print("    | " + line.strip())
    return done.returncode


RUNNER = "清理过期样本-自动.bat"


def _write_runner(apply: bool) -> str:
    """生成计划任务真正调用的那个 .bat。

    ★ 为什么要多这一层，而不是把命令直接塞进 `schtasks /TR`：
      `/TR` 收的是**一个字符串**，而我们的命令里 python 路径和脚本路径
      都带空格、都必须自己带引号 —— 于是 `/TR` 里出现嵌套双引号，
      schtasks 解析出来是残的，任务注册"成功"但一跑就失败（而且失败
      只写在任务计划程序的历史里，现场根本不会去看）。
      指向一个 .bat 就只有一层引号，顺带让现场能直接双击验证、
      能看懂到底会跑什么。

    ▲ 与交互版 `清理过期样本.bat` 是两个文件：那个要人工确认两次才删，
      这个是给计划任务用的**非交互**版本，故意分开，免得有人手滑双击。
    """
    path = os.path.join(ROOT, RUNNER)
    lines = [
        "@echo off",
        "rem 由 tools/install_cleanup_task.py 生成 —— 计划任务专用（非交互）。",
        "rem 天数取 config/channels.json 的 service.keep_days，改配置不用重装任务。",
        "rem 空车样本与空车归档不参与清理；基线 npz 更不在范围内。",
        'cd /d "%~dp0"',
        _quote(_cleanup_argv()),
        "exit /b %errorlevel%",
    ]
    print("  计划任务会调用：%s" % path)
    print("  它的内容是：")
    for line in lines:
        print("      " + line)
    # .bat 必须是 CRLF + GBK：现场是中文 Windows，写成 LF/UTF-8 的 bat
    # 在 cmd 里会把中文注释和路径吐成乱码，甚至把 rem 行当命令执行。
    body = ("\r\n".join(lines) + "\r\n").encode("gbk", "replace")
    if apply:
        with open(path, "wb") as handle:
            handle.write(body)
    return path


def windows(at: str, apply: bool, remove: bool) -> int:
    if remove:
        print("卸载 Windows 计划任务 %s：" % TASK_NAME)
        code = _run(["schtasks", "/Delete", "/TN", TASK_NAME, "/F"], apply)
        path = os.path.join(ROOT, RUNNER)
        if apply and os.path.isfile(path):
            try:
                os.remove(path)
                print("    已删除 " + RUNNER)
            except OSError as exc:
                print("    ▲ %s 没删掉（%s），可手工删" % (RUNNER, type(exc).__name__))
        return code
    print("安装 Windows 计划任务 %s（每天 %s）：" % (TASK_NAME, at))
    path = _write_runner(apply)
    print("  注册命令：")
    # /RL LIMITED：清理只动 data 下的文件，不需要管理员权限。
    return _run(["schtasks", "/Create", "/SC", "DAILY", "/TN", TASK_NAME,
                 "/TR", path, "/ST", at, "/RL", "LIMITED", "/F"], apply)


def posix(at: str, apply: bool, remove: bool) -> int:
    """麒麟 / Linux：不代改 crontab，把该加的那一行打出来。

    现场的 crontab 往往还有别的任务，脚本去改它风险大于收益 ——
    这一行贴进去就行，也方便留在交接文档里。
    """
    hour, _, minute = at.partition(":")
    line = "%s %s * * * cd %s && %s >> %s 2>&1" % (
        minute or "0", hour, ROOT, _quote(_cleanup_argv()),
        os.path.join(ROOT, "logs", "cleanup.log"))
    if remove:
        print("麒麟 / Linux：请从 `crontab -e` 里删掉这一行：")
    else:
        print("麒麟 / Linux：把下面这一行加进 `crontab -e`（每天 %s）：" % at)
    print()
    print("    " + line)
    print()
    print("  （本脚本不代改 crontab —— 现场那份往往还有别的任务，自动改的风险大于收益）")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="把按天清理挂成每天自动执行的计划任务")
    p.add_argument("--at", default="03:30", help="每天几点执行，HH:MM，默认 03:30（避开白天作业）")
    p.add_argument("--apply", action="store_true", help="真装/真卸；不加只打印要做什么")
    p.add_argument("--remove", action="store_true", help="卸载而不是安装")
    args = p.parse_args(argv)

    hour, sep, minute = args.at.partition(":")
    if not (sep and hour.isdigit() and minute.isdigit()
            and 0 <= int(hour) <= 23 and 0 <= int(minute) <= 59):
        print("× --at 必须是 HH:MM，例如 03:30")
        return 2
    at = "%02d:%02d" % (int(hour), int(minute))

    print("=" * 78)
    print("载重雷达 —— 过期样本自动清理")
    print("=" * 78)
    print("保留天数   : 取自 config/channels.json 的 service.keep_days（--auto-days）")
    print("空车        : 判为空车的样本与归档**不参与清理**，基线 data/in/*.npz 更不在范围内")
    print("项目根      : " + ROOT)
    if not args.apply:
        print("模式        : **预演**，什么都不写。确认无误后加 --apply")
    print()

    code = (windows if os.name == "nt" else posix)(at, args.apply, args.remove)

    print()
    if not args.apply:
        print("以上是预演。加 --apply 才真正生效。")
    elif code == 0:
        print("√ 完成。建议先手工跑一次确认：python cli/cleanup.py --auto-days")
    else:
        print("× 没有成功，退出码 %d。上面有命令原文，可以手工执行排查。" % code)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
