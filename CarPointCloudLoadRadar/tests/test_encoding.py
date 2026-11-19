# -*- coding: utf-8 -*-
"""源码与控制台输出编码守卫。

为什么要有这个测试（真实事故，不是假想）：
    现场机器是中文 Windows，控制台/管道代码页是 GBK(CP936)。
    程序里 print 一个 GBK 表达不了的字符（U+2705 / U+26A0 / U+274C / U+1F449 …）
    时，Python 会抛 UnicodeEncodeError: 'gbk' codec can't encode character，
    而**异常发生在 print 那一刻**，也就是：
        · 前面已打印的内容只到崩的那一行为止，后面全丢；
        · 崩溃点在工具函数里，堆栈看起来跟"现场体检"毫无关系；
        · 开发机上装了 PYTHONUTF8=1 / PYTHONIOENCODING=utf-8 时完全复现不了。

    实际踩到的：`tools/configure_radars.py` 的 preflight() 里一句带 U+26A0 的
    提示，让整个"配置雷达 IP"命令在真机上一跑就挂，且没有任何输出。

所以规则是硬性的：
    仓库里任何会打到控制台的 .py / .bat / .vbs，
    其**字面量文本**必须全部能被 GBK 编码。

注意本文件自身也受这条规则约束 —— 所以下面所有不安全的符号
（包括 SUGGEST 表的键值、测试用例里的样本）一律写成 \\uXXXX 转义，
不写字面量 emoji。改这个文件时请保持这个习惯。

不用背符号表，改完代码跑一遍这个测试即可（tests/run_all.py 已纳入）。
失败信息会直接告诉你：哪个文件、哪一行、哪个字符、建议换成什么。
"""
from __future__ import annotations

import os
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 源码目录（desktop/launchers 里的模板也在内：它会被转成 GBK 装到根目录）
# desktop/ 不在本项目：目标平台是麒麟，不做 Windows 桌面打包。
SCAN_DIRS = ("tools", "cli", "app", "tests")
SCAN_EXTS = (".py", ".bat", ".vbs")

CONSOLE_ENCODING = "gbk"

# 常用替换建议：左边是踩过坑的字符，右边是 GBK 安全等价物
# （键值一律用转义写，这样本文件自己也符合 GBK 安全规则）
SUGGEST = {
    "\u2705": "\u221a",      # U+2705 -> U+221A
    "\u274c": "\u00d7",      # U+274C -> U+00D7
    "\u26a0": "\u25b2",      # U+26A0 -> U+25B2
    "\u2714": "\u221a",      # U+2714 -> U+221A
    "\u2713": "\u221a",      # U+2713 -> U+221A  （这个也编不进 GBK，别想当然）
    "\u2716": "\u00d7",      # U+2716 -> U+00D7
    "\u2717": "\u00d7",      # U+2717 -> U+00D7  （同上）
    "\U0001F449": "\u2192",  # U+1F449 -> U+2192
    "\u21b3": "\u2514",      # U+21B3 -> U+2514
    "\u27f7": "<->",         # U+27F7 -> <->
    "\u2194": "<->",         # U+2194 -> <->   （实测编不进 GBK）
    "\u21d2": "=>",          # U+21D2 -> =>
    "\u2022": "\u00b7",      # U+2022 -> U+00B7
    "\u2212": "-",           # U+2212 -> -
    "\u00b3": "^3",          # U+00B3 -> ^3
    "\u00b2": "^2",          # U+00B2 -> ^2
    "\u00b5": "\u03bc",      # U+00B5 微符号 -> U+03BC 希腊 mu（长得一样，GBK 编得进）
    "\ufe0f": "(删除)",       # 变体选择符 VS16，直接删
}

# 已确认**可以**被 GBK 编码的符号，用来钉住"建议值不能比原值更糟"
GBK_SAFE_SAMPLES = ("\u221a", "\u00d7", "\u25b2", "\u2192", "\u2514",
                    "\u00b7", "\u2261", "\u2260", "\u2264", "\u2265",
                    "\u25cf", "\u25cb", "\u2605", "\u203b",
                    "\u25a0", "\u25a1", "\u00b0", "\u2103", "\u221e")

# 已确认**不能**被 GBK 编码的符号，钉住黑名单
GBK_BAD_SAMPLES = ("\u2705", "\u274c", "\u26a0", "\u2713", "\u2714",
                   "\u2716", "\u2717", "\U0001F449", "\u21d2", "\u2194",
                   "\u27f7", "\u21b3", "\u00b5")


def gbk_ok(ch: str) -> bool:
    try:
        ch.encode(CONSOLE_ENCODING)
        return True
    except UnicodeEncodeError:
        return False


def bad_lines(text: str):
    """产出 (行号, 字符) —— 只针对 GBK 编不出来的字符。"""
    for no, line in enumerate(text.splitlines(), 1):
        for ch in line:
            if ord(ch) >= 128 and not gbk_ok(ch):
                yield no, ch


def iter_sources():
    for d in SCAN_DIRS:
        base = os.path.join(ROOT, d)
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [x for x in dirnames
                           if x not in ("__pycache__", ".pytest_cache")]
            for fn in sorted(filenames):
                if fn.endswith(SCAN_EXTS):
                    yield os.path.join(dirpath, fn)


def fmt_rel(path: str) -> str:
    return os.path.relpath(path, ROOT).replace("\\", "/")


class TestSourceIsGbkSafe(unittest.TestCase):
    """仓库源码里不允许出现 GBK 编不出的字符。"""

    def test_all_sources_gbk_encodable(self):
        offenders = []
        for path in iter_sources():
            try:
                with open(path, encoding="utf-8") as f:
                    text = f.read()
            except UnicodeDecodeError:
                # 已是 GBK 的产物（例如安装好的 .bat），跳过 —— 另有专门测试
                continue
            seen = {}
            for no, ch in bad_lines(text):
                seen.setdefault(ch, []).append(no)
            if seen:
                rel = fmt_rel(path)
                for ch, lines in sorted(seen.items()):
                    hit = ", ".join(str(n) for n in lines[:6])
                    more = "" if len(lines) <= 6 else f" 等 {len(lines)} 处"
                    hint = SUGGEST.get(ch, "见本文件 SUGGEST 表")
                    offenders.append(
                        f"  {rel}:{hit}{more}  U+{ord(ch):04X} {ch!r}"
                        f" -> 建议 {hint}")

        if offenders:
            self.fail(
                "以下字符无法用 GBK 编码，会在中文 Windows 控制台抛 "
                "UnicodeEncodeError 并把程序打断：\n"
                + "\n".join(offenders)
                + "\n\n处理：换成上面建议的等价字符，"
                  "或跑 python logs/_fix_gbk.py --apply 批量替换。")

    def test_launcher_templates_follow_convention(self):
        """desktop/launchers/ 的模板必须能被 install_launchers 无损转 GBK。

        ▲ `desktop/` 不在本项目（目标平台是麒麟，不做 Windows 桌面打包），
          所以模板不在时跳过。要做桌面版就得把 `desktop/` 一起带上，
          这条用例会自动重新生效。
        """
        from tools.install_launchers import JOBS, SRC_DIR, unencodable

        if not os.path.isdir(SRC_DIR):
            self.skipTest("本项目不含 desktop/launchers/：" + str(SRC_DIR))
        self.assertTrue(JOBS, "JOBS 不该为空")
        for src, out_name in JOBS:
            sp = os.path.join(SRC_DIR, src)
            self.assertTrue(os.path.isfile(sp), f"缺模板 {src}")
            with open(sp, encoding="utf-8") as f:
                bad = unencodable(f.read())
            self.assertEqual(
                bad, {},
                f"{src} -> {out_name} 转 GBK 会变成 '?'：{bad}")


class TestInstalledLaunchers(unittest.TestCase):
    """装到项目根的启动脚本必须是 GBK + CRLF，否则双击就乱码。"""

    # 根目录只保证这几个存在；缺失时跳过（测试可以脱离安装产物单跑）
    NAMES = (
        "\u542f\u52a8\u684c\u9762\u7248\uff08\u5e26\u65e5\u5fd7\uff09.bat",
        "\u542f\u52a8\u684c\u9762\u7248.vbs",
        "\u73af\u5883\u81ea\u68c0.bat",
        "\u914d\u7f6e\u96f7\u8fbeIP.bat",
    )

    def test_installed_are_gbk_and_crlf(self):
        found = 0
        for name in self.NAMES:
            p = os.path.join(ROOT, name)
            if not os.path.isfile(p):
                continue
            found += 1
            with open(p, "rb") as f:
                raw = f.read()

            self.assertNotIn(
                b"\xef\xbf\xbd", raw,
                f"{name} 里有 U+FFFD 替换字符，说明转码丢字了")

            # 必须能按 GBK 解出来，且不能是 UTF-8 中文被硬塞进来
            text = raw.decode(CONSOLE_ENCODING)          # 解不出会抛，测试即失败

            # CRLF 约定：不允许出现裸 LF
            bare_lf = raw.replace(b"\r\n", b"").count(b"\n")
            self.assertEqual(bare_lf, 0, f"{name} 有 {bare_lf} 处裸 LF，应统一 CRLF")

            # 装出来的脚本里不该再出现 GBK 编不出的字符（理想态；有则说明
            # 是手工塞进去的，不会被本测试放过）
            for ch in text:
                if ord(ch) < 128:
                    continue
                self.assertTrue(
                    gbk_ok(ch),
                    f"{name} 含 U+{ord(ch):04X} {ch!r}，GBK 控制台解析会出错")

        if found == 0:
            self.skipTest("根目录没有安装产物，跳过（先跑 tools/install_launchers.py）")


class TestUnencodableHelper(unittest.TestCase):
    """tools/install_launchers.unencodable 本身的行为。"""

    def test_detects_emoji(self):
        from tools.install_launchers import unencodable
        self.assertEqual(unencodable("hello world"), {})
        self.assertEqual(unencodable("\u221a \u00d7 \u25b2"), {})   # 这些是 GBK 安全的
        self.assertEqual(unencodable("\u2705"), {"\u2705": 1})
        self.assertEqual(unencodable("\U0001F449\u26a0\ufe0f"),
                         {"\U0001F449": 1, "\u26a0": 1, "\ufe0f": 1})

    def test_render_is_strict(self):
        """render 用 strict：漏检时当场抛，不静默写 '?'。"""
        import tempfile
        from tools.install_launchers import render

        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "t.bat")
            with open(p, "w", encoding="utf-8") as f:
                f.write("echo ok\n")
            self.assertEqual(render(p), b"echo ok\r\n")

            with open(p, "w", encoding="utf-8") as f:
                f.write("echo \u2705\n")
            with self.assertRaises(UnicodeEncodeError):
                render(p)


class TestSuggestTableItself(unittest.TestCase):
    """SUGGEST 的右值必须自己也是 GBK 安全的，否则修一个坏一个。"""

    def test_replacements_are_gbk_safe(self):
        for k, v in SUGGEST.items():
            if v == "(删除)":
                continue
            for ch in v:
                if ord(ch) < 128:
                    continue
                self.assertTrue(
                    gbk_ok(ch),
                    f"建议把 U+{ord(k):04X} {k!r} 换成 {v!r}，"
                    f"但 {v!r} 里的 U+{ord(ch):04X} {ch!r} 本身也编不进 GBK")

    def test_blacklist_symbols_all_have_a_suggestion(self):
        """黑名单字符都应该有建议 —— 免得下次又要现查符号表。"""
        for ch in GBK_BAD_SAMPLES:
            self.assertIn(ch, SUGGEST, f"U+{ord(ch):04X} {ch!r} 缺建议")

    def test_gbk_safe_samples_really_are_safe(self):
        """钉住符号表结论，防止换机器/换 Python 后认知漂移。"""
        for ch in GBK_SAFE_SAMPLES:
            self.assertTrue(gbk_ok(ch), f"U+{ord(ch):04X} {ch!r} 本应可编 GBK")

    def test_gbk_bad_samples_really_are_bad(self):
        for ch in GBK_BAD_SAMPLES:
            self.assertFalse(gbk_ok(ch), f"U+{ord(ch):04X} {ch!r} 本应编不进 GBK")


if __name__ == "__main__":
    unittest.main(verbosity=2)
