# -*- coding: utf-8 -*-
"""把 desktop/launchers/ 下的启动脚本模板转码安装到目标目录（默认项目根）。

用法：
    python tools/install_launchers.py                    # 装到项目根
    python tools/install_launchers.py --dest dist/载重雷达检测系统
    python tools/install_launchers.py --check            # 只检查，不写文件

为什么要"转码"这一步（而不是直接把 .bat 放到根目录）：
  · Windows 的 cmd.exe 与 WSH 默认按**系统代码页**（中文系统 = CP936/GBK）
    解析 .bat / .vbs 的**内容**；
  · 用 UTF-8 保存含中文的 .bat/.vbs，双击后会乱码甚至解析失败；
  · 换行必须是 CRLF，否则 cmd 在含括号块 / 标签跳转时可能解析出错。
  · 而模板用 UTF-8 保存才好维护（编辑器、diff、代码审查都友好），
    所以约定：**模板在 desktop/launchers/ 用 UTF-8，安装产物是 GBK+CRLF**。
    改了模板一定要重跑本脚本，否则根目录的脚本不会更新。
"""
from __future__ import annotations

import argparse
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC_DIR = os.path.join(ROOT, "desktop", "launchers")

# 模板名 -> 安装到目标目录后的中文名
JOBS = [
    ("launch_console.bat", "启动桌面版（带日志）.bat"),
    ("launch_silent.vbs", "启动桌面版.vbs"),
    ("launch_demo.vbs", "启动演示版（4路仿真）.vbs"),
    ("check_env.bat", "环境自检.bat"),
    ("configure_radars.bat", "配置雷达IP.bat"),
]

TARGET_ENCODING = "gbk"          # 中文 Windows 的系统代码页


def unencodable(text: str) -> dict:
    """返回 {字符: 次数} —— GBK 编不出来的那些。

    模板是 UTF-8，很容易顺手写进 U+2705 / U+26A0 / U+274C 之类的符号；
    装到根目录时它们会变成 '?'，菜单上就成了乱码问号，而脚本本身还能跑
    —— 属于最难发现的那类问题。所以这里必须先查。
    """
    out: dict = {}
    for ch in text:
        if ord(ch) < 128:
            continue
        try:
            ch.encode(TARGET_ENCODING)
        except UnicodeEncodeError:
            out[ch] = out.get(ch, 0) + 1
    return out


def render(src_path: str) -> bytes:
    with open(src_path, encoding="utf-8") as f:
        text = f.read()
    # 统一换行为 CRLF
    text = text.replace("\r\n", "\n").replace("\r", "\n").replace("\n", "\r\n")
    # 用 strict：万一漏检就当场炸，绝不静默写成 '?'
    return text.encode(TARGET_ENCODING, errors="strict")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="安装 Windows 启动脚本（转 GBK+CRLF）")
    ap.add_argument("--dest", default=ROOT, help="目标目录（默认项目根）")
    ap.add_argument("--check", action="store_true", help="只校验能转码，不写文件")
    args = ap.parse_args(argv)

    dest = os.path.abspath(args.dest)
    print(f"模板目录 : {SRC_DIR}")
    print(f"目标目录 : {dest}")
    print(f"目标编码 : {TARGET_ENCODING.upper()} + CRLF")
    print()

    if not os.path.isdir(SRC_DIR):
        print(f"× 找不到模板目录 {SRC_DIR}")
        return 2
    if not args.check:
        os.makedirs(dest, exist_ok=True)

    rc = 0
    for src, out_name in JOBS:
        sp = os.path.join(SRC_DIR, src)
        if not os.path.isfile(sp):
            print(f"  × 缺模板 {src}")
            rc = 3
            continue

        # 先查模板里有没有 GBK 表达不了的字符 —— 有就直接拒绝装，别装出问号
        with open(sp, encoding="utf-8") as f:
            bad = unencodable(f.read())
        if bad:
            print(f"  × {src} 含 GBK 编不出的字符，装出去会变成 '?'：")
            for c, n in bad.items():
                print(f"      U+{ord(c):04X} {c!r} x{n}")
            print("      改成 GBK 安全等价字符（▲ √ × → └ · 等）后重跑。")
            rc = 5
            continue

        try:
            data = render(sp)
        except Exception as e:                                # noqa: BLE001
            print(f"  × {src} 转码失败: {type(e).__name__}: {e}")
            rc = 4
            continue

        if args.check:
            print(f"  √ {src} → {out_name}  ({len(data)} B, 可转码)")
        else:
            dp = os.path.join(dest, out_name)
            with open(dp, "wb") as f:
                f.write(data)
            print(f"  √ {out_name}  ({len(data)} B)")

    print()
    print("完成。" if rc == 0 else "有失败项。")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
