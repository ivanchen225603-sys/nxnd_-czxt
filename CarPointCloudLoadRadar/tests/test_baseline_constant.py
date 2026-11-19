# -*- coding: utf-8 -*-
"""回归基线目录必须只有一个定义处。

为什么要钉住这件事：

    `cli/verify.py` 的全部效力都来自基线目录下那 10 份原版点云 +
    同名 `.reason.txt` + `MANIFEST.json`。在 2026-09-22 收口之前，
    `"data/loaded"` 这个字面量**散在 14 处代码里**
    （storage / verify / replay / pc_tool / fix_plate_dirs 与若干用例），
    而且没有任何一处是权威定义。

    后果不是"改起来麻烦"，是**闸门会静默失效**：改名或迁移时漏掉
    `storage.baseline_dir()` 那一处，清理就不再认识基线目录；
    漏掉 `verify.py --dir` 那一处，闸门就去比一个空目录或者现场落盘目录
    —— 两种都照样打印"全部通过"。

    所以现在唯一的定义在 `app/settings.BASELINE_SUBDIR`，
    其余全部从它派生；本模块守住这个约定。
"""
from __future__ import annotations

import ast
import io
import os
import re
import unittest

from app import settings, storage

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 运行时代码目录。tests/ 不扫：用例里出现这个字面量往往是在**造**一个
# 假目录结构做断言，属于正当用法。
SCAN_DIRS = ("app", "cli", "tools")
ALLOWED = {os.path.normcase(os.path.join(ROOT, "app", "settings.py"))}


def _looks_like_url(value: str) -> bool:
    """看板路由（`/api/baseline/capture`）长得像路径但不是文件路径。

    目录名是 `baseline` 这种短常用词，不排除这类就会一直误报，
    而误报多了守卫就会被关掉。
    """
    return value.startswith("/api/") or "://" in value


def _code_literals(path: str):
    """模块里**非文档字符串**的字符串常量。

    用 ast 而不是正则：注释与 docstring 里提到 `data/loaded` 是正常的
    （本项目的注释密度很高），只有真正参与运算的字面量才算违规。
    """
    try:
        with io.open(path, encoding="utf-8") as stream:
            tree = ast.parse(stream.read(), filename=path)
    except (SyntaxError, UnicodeDecodeError):
        return []
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            body = getattr(node, "body", None)
            if body and isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant) \
                    and isinstance(body[0].value.value, str):
                docstrings.add(id(body[0].value))
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docstrings:
            out.append((node.lineno, node.value))
    return out


class TestBaselineIsSingleSourced(unittest.TestCase):

    def test_no_runtime_module_hardcodes_the_path(self):
        """运行时代码里不许再出现**路径形状**的基线目录字面量。

        ▲▲ 判据是「**带路径分隔符**」，不是「出现这个词」。两次放宽都有实测依据：
          · 按裸词匹配 -> `baseline_cache_unverified` 这类正当用词误报 **114 处**；
          · 连整串相等也算 -> `call["baseline"]` `receipt.get("baseline")`
            这类 **dict 键名**误报 **9 处**。
          目录名短且是常用词，只有带分隔符的形态能确定是路径。

        ▲ 代价说清楚：`root("baseline")` 这种单段写法**测不出来**。
          兜底是本模块的 `test_storage_follows_the_constant` ——
          `storage.baseline_dir()` 是唯一的绝对路径出口，它被行为钉死。
        """
        literal = settings.baseline_rel()
        sep = "[/" + chr(92) * 2 + "]"          # 路径分隔符字符类（/ 或 \）
        # 必须带分隔符：`baseline/x`、`x/baseline`、`x/baseline/y`
        pattern = re.compile("(?:^|%s)%s%s|%s%s(?:%s|$)"
                             % (sep, re.escape(literal), sep,
                                sep, re.escape(literal), sep))
        offenders = []
        for folder in SCAN_DIRS:
            for dirpath, dirnames, filenames in os.walk(os.path.join(ROOT, folder)):
                dirnames[:] = [d for d in dirnames if d != "__pycache__"]
                for name in filenames:
                    if not name.endswith(".py"):
                        continue
                    path = os.path.join(dirpath, name)
                    if os.path.normcase(path) in ALLOWED:
                        continue
                    for lineno, value in _code_literals(path):
                        if _looks_like_url(value):
                            continue          # HTTP 路由不是文件路径
                        if pattern.search(value):
                            offenders.append("%s:%d  %r" % (
                                os.path.relpath(path, ROOT), lineno, value))
        self.assertEqual(offenders, [],
                         "这些地方又把基线目录写死了，请改用 settings.BASELINE_SUBDIR / "
                         "baseline_rel()：\n  " + "\n  ".join(offenders))

    def test_storage_follows_the_constant(self):
        """改常量，`storage.baseline_dir()` 必须跟着变 —— 它是唯一的绝对路径出口。"""
        original = settings.BASELINE_SUBDIR
        try:
            settings.BASELINE_SUBDIR = ("data", "somewhere-else")
            self.assertTrue(storage.baseline_dir().endswith(
                os.path.normcase(os.path.join("data", "somewhere-else"))),
                "baseline_dir() 没有跟着常量走")
        finally:
            settings.BASELINE_SUBDIR = original
        self.assertTrue(storage.baseline_dir().endswith(
            os.path.normcase(os.path.join(*original))))

    def test_relative_not_absolute(self):
        """★ 必须是相对分段：打包成 exe 后 PROJECT_ROOT 会被改写，
        存绝对路径会停在 _MEIPASS 上，`is_baseline()` 那道闸门等于没有。"""
        for part in settings.BASELINE_SUBDIR:
            self.assertFalse(os.path.isabs(part), "基线常量不能存绝对路径")
        self.assertFalse(os.path.isabs(settings.baseline_rel()))

    def test_the_real_baseline_is_there(self):
        """本仓库里这个目录必须真的存在且非空 —— 否则闸门在比空气。"""
        directory = storage.baseline_dir()
        self.assertTrue(os.path.isdir(directory), "基线目录不存在：" + directory)
        found = []
        for dirpath, _dirs, files in os.walk(directory):
            found += [f for f in files if f.endswith(".xyz")]
        self.assertGreaterEqual(len(found), 10, "基线点云少于 10 份")


if __name__ == "__main__":
    unittest.main()
