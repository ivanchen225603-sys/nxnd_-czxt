#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_cleanup.py — cli/cleanup.py（落盘占用 / 过期清理）

这是全项目**唯一会删文件**的脚本，所以用例的重点全在"别删错"：
    · 不加 --apply 绝不删任何东西；
    · 只删超过保留天数的，没到期的一个都不能少；
    · 点云 + reason + meta 三件一起删，不留半拉记录；
    · 认不出的文件（现场手放的 .txt / .bak / .zip）一律不碰；
    · 目标是 data/loaded（回归基线）时**直接拒绝**并退出码 2。
最后一条尤其重要：删掉基线之后 cli/verify.py 会变成"0 个样本全部通过"，
回归闸门静默失效，而且不会有任何人发现。
"""
from __future__ import annotations

import os
import shutil
import sys
import tempfile
import time
import unittest

from contextlib import redirect_stdout

from tests import _helpers as H                # noqa: F401  （把项目根加进 sys.path）
from app.settings import BASELINE_SUBDIR, baseline_rel, root

sys.path.insert(0, root("cli"))
import cleanup                                  # noqa: E402  （命令行外壳）
from app import storage                         # noqa: E402  （真正的逻辑）


def _make_sample(d: str, name: str, age_days: float, size: int = 128) -> str:
    os.makedirs(d, exist_ok=True)
    xyz = os.path.join(d, name + ".xyz")
    with open(xyz, "wb") as f:
        f.write(b"\x00" * size)
    with open(xyz + ".reason.txt", "w", encoding="utf-8") as f:
        f.write("空车")
    with open(xyz + ".meta.json", "w", encoding="utf-8") as f:
        f.write("{}")
    t = time.time() - age_days * 86400.0
    for p in (xyz, xyz + ".reason.txt", xyz + ".meta.json"):
        os.utime(p, (t, t))
    return xyz


def _quiet_report(*a, **kw):
    """跑 report() 但别把报告打进测试输出里（它是给人看的，不是给日志看的）。"""
    import io as _io
    buf = _io.StringIO()
    with redirect_stdout(buf):
        r = cleanup.report(*a, **kw)
    return r


class TestScanAndRemove(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="lidar-clean-")
        self.old = _make_sample(os.path.join(self.tmp, "宁A88635W"), "20260101_010101", 100)
        self.new = _make_sample(os.path.join(self.tmp, "宁A88635W"), "20260901_010101", 1)
        # 现场常见的"手放进来的文件" —— 一律不该被动
        with open(os.path.join(self.tmp, "现场记录.txt"), "w", encoding="utf-8") as f:
            f.write("x")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_scan_counts_the_trio_as_one_sample(self):
        items = storage.scan(self.tmp)
        self.assertEqual(len(items), 2, "两份样本（每份三个文件）应当算成 2")
        sizes = {os.path.basename(p): sz for p, _t, sz in items}
        for name, sz in sizes.items():
            self.assertGreater(sz, 128, f"{name} 的占用应当含 reason/meta")

    def test_dry_run_deletes_nothing(self):
        before = sorted(os.listdir(os.path.join(self.tmp, "宁A88635W")))
        _quiet_report("【测试】", self.tmp, days=30, apply=False)
        after = sorted(os.listdir(os.path.join(self.tmp, "宁A88635W")))
        self.assertEqual(before, after, "预演模式动了文件")

    def test_apply_removes_only_expired(self):
        r = _quiet_report("【测试】", self.tmp, days=30, apply=True)
        self.assertEqual(r["old"], 1)
        self.assertEqual(r["removed_files"], 3, "点云+理由+元数据要一起删")
        self.assertFalse(os.path.exists(self.old))
        self.assertFalse(os.path.exists(self.old + ".reason.txt"))
        self.assertFalse(os.path.exists(self.old + ".meta.json"))
        self.assertTrue(os.path.exists(self.new), "没到期的样本被删了")
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "现场记录.txt")),
                        "不认识的文件不该动")

    def test_empty_plate_dirs_are_pruned(self):
        d = os.path.join(self.tmp, "蒙C88593X")
        _make_sample(d, "20260101_020202", 100)
        _quiet_report("【测试】", self.tmp, days=30, apply=True)
        self.assertFalse(os.path.isdir(d), "清空后的车牌目录应当删掉")
        self.assertTrue(os.path.isdir(self.tmp), "输出目录本身不能删")

    def test_zero_days_reports_only(self):
        r = _quiet_report("【测试】", self.tmp, days=0, apply=True)
        self.assertEqual(r["removed_files"], 0, "没给保留天数时不该删任何东西")
        self.assertTrue(os.path.exists(self.old))


class TestStoragePurge(unittest.TestCase):
    """app/storage.purge() —— 命令行与桌面版共用的那一个删除入口。"""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="lidar-purge-")
        self.old = _make_sample(os.path.join(self.tmp, "宁A88635W"), "20260101_010101", 100)
        self.new = _make_sample(os.path.join(self.tmp, "宁A88635W"), "20260901_010101", 1)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_purge_refuses_without_days(self):
        r = storage.purge(self.tmp, days=0)
        self.assertEqual(r["removed_files"], 0)
        self.assertTrue(r["refused"], "days<=0 必须拒绝，而不是把目录删空")
        self.assertTrue(os.path.exists(self.old))

    def test_purge_removes_only_expired(self):
        r = storage.purge(self.tmp, days=30)
        self.assertEqual(r["removed"], 1)
        self.assertEqual(r["removed_files"], 3)
        self.assertFalse(os.path.exists(self.old))
        self.assertTrue(os.path.exists(self.new))

    def test_summarize_is_read_only(self):
        before = sorted(os.listdir(os.path.join(self.tmp, "宁A88635W")))
        info = storage.summarize(self.tmp, days=30)
        self.assertEqual(info["samples"], 2)
        self.assertEqual(info["old"], 1)
        self.assertEqual(sorted(os.listdir(os.path.join(self.tmp, "宁A88635W"))), before)


class TestDirUsage(unittest.TestCase):
    """磁盘占用盘点 —— 现场的大头往往不是点云（浏览器用户数据能到 500MB+）。"""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="lidar-usage-")
        for sub, n in (("big", 40), ("small", 3), ("", 1)):
            d = os.path.join(self.tmp, sub) if sub else self.tmp
            os.makedirs(d, exist_ok=True)
            for i in range(n):
                with open(os.path.join(d, f"f{i}.bin"), "wb") as f:
                    f.write(b"x" * 1024)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_counts_and_ranks(self):
        u = storage.dir_usage(self.tmp, top=2)
        self.assertTrue(u["exists"])
        self.assertEqual(u["files"], 44)
        self.assertEqual(u["bytes"], 44 * 1024)
        self.assertEqual([k for k, _v in u["top"]], ["big", "small"],
                         "要按占用从大到小排，现场一眼看出谁在吃盘")

    def test_missing_dir_is_not_an_error(self):
        u = storage.dir_usage(os.path.join(self.tmp, "不存在"))
        self.assertFalse(u["exists"])
        self.assertEqual(u["bytes"], 0)


class TestBaselineGuard(unittest.TestCase):
    """回归基线目录必须被硬挡住。"""

    def test_baseline_follows_patched_project_root(self):
        """★ 打包版会在启动时把 app.settings.PROJECT_ROOT 换成 exe 目录。

        基线目录若在 import 时算成常量，就会停在解包目录上 ——
        "别碰 data/loaded" 这道闸门在打包版里等于没有。
        """
        from app import settings as app_settings
        orig = app_settings.PROJECT_ROOT
        fake = tempfile.mkdtemp(prefix="lidar-fakeroot-")
        try:
            app_settings.PROJECT_ROOT = fake
            self.assertTrue(storage.is_baseline(os.path.join(fake, *BASELINE_SUBDIR)),
                            "换根之后没跟着走 —— 基线闸门失效")
            self.assertFalse(storage.is_baseline(os.path.join(fake, "data", "field")))
        finally:
            app_settings.PROJECT_ROOT = orig
            shutil.rmtree(fake, ignore_errors=True)

    def test_is_baseline_detects_dir_and_children(self):
        self.assertTrue(storage.is_baseline(root(*BASELINE_SUBDIR)))
        self.assertTrue(storage.is_baseline(root(*BASELINE_SUBDIR, "TEST001")))
        self.assertFalse(storage.is_baseline(root("data", "field")))
        # 前缀相同但不是同一个目录，不能误判成基线
        self.assertFalse(storage.is_baseline(root(*BASELINE_SUBDIR[:-1],
                                                  BASELINE_SUBDIR[-1] + "_field")))

    def test_main_refuses_baseline_output_dir(self):
        import io as _io
        import json
        from contextlib import redirect_stdout
        tmp = tempfile.mkdtemp(prefix="lidar-cfgtest-")
        try:
            cfg = {
                "service": {"host": "127.0.0.1", "port": 8080, "data_root": "data",
                            "log_dir": "logs"},
                "channels": [{
                    "id": "CH1", "name": "1号通道", "enabled": True, "mode": "replay",
                    "lidar": {"ip": "198.51.100.1", "source_ip": "198.51.100.1"},
                    # ← 故意指到回归基线，验证清理会拒绝
                    "output_dir": baseline_rel(),
                }],
            }
            p = os.path.join(tmp, "channels.json")
            with open(p, "w", encoding="utf-8") as f:
                json.dump(cfg, f, ensure_ascii=False)
            buf = _io.StringIO()
            with redirect_stdout(buf):
                rc = cleanup.main(["--channels", p, "--days", "1", "--apply"])
            self.assertEqual(rc, 2, "指向回归基线时必须拒绝执行")
            self.assertIn("拒绝处理", buf.getvalue())
            # 基线一个文件都不能少
            n = len([f for f in os.listdir(root(*BASELINE_SUBDIR))])
            self.assertGreater(n, 0)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()

def _mark_verdict(xyz: str, empty) -> None:
    """按判定链结论写 meta；`empty=None` 模拟老样本（没这个字段）。"""
    import json as _json
    meta = {} if empty is None else {"empty": empty, "fit_ok": not empty}
    stat = os.stat(xyz)
    with open(xyz + ".meta.json", "w", encoding="utf-8") as handle:
        handle.write(_json.dumps(meta, ensure_ascii=False))
    os.utime(xyz + ".meta.json", (stat.st_atime, stat.st_mtime))


class TestEmptySamplesAreNeverAged(unittest.TestCase):
    """★ 现场要求：最新的空车车厢**永久保存**，载重云图只留最近 N 天。

    按天数清理是按 mtime 一刀切的，会把空车一起删掉 —— 而空车点云是
    基线的原始材料，删了补不回来（那辆车得重新开进来标定一次）。

    判据读 meta 的 `empty`（**判定链结论**）。注意不能读 `fit_ok`：
    那是"六面拟合成功与否"，2026-09-17 查实 137 份现场样本里 115 份
    两者相反，照 fit_ok 挑会把载货样本当空车留下、把空车删掉。
    """

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="cleanup-empty-")
        self.addCleanup(self.temp.cleanup)
        self.out = os.path.join(self.temp.name, "field")

    def test_old_empty_sample_survives_and_old_loaded_sample_goes(self):
        """★ 本条就是这次改动的目标。"""
        empty = _make_sample(self.out, "20260101_000000", 30.)
        loaded = _make_sample(self.out, "20260101_000001", 30.)
        _mark_verdict(empty, True)
        _mark_verdict(loaded, False)
        result = _quiet_report("测试", self.out, 7., True)
        self.assertTrue(os.path.exists(empty), "空车样本被按天数删掉了")
        self.assertFalse(os.path.exists(loaded), "过期载货样本没被删")
        self.assertEqual(1, result["kept_empty"])

    def test_sample_without_verdict_still_follows_the_age_policy(self):
        """判不出来的按"不是空车"处理 —— 否则老样本全部永久豁免，
        7 天策略对现存的历史数据等于没有。

        敢这么定是因为空车的**底本不在这里**：`data/in/*.npz` 在
        `output_dir` 之外，成套归档另按 manifest 保护。"""
        old = _make_sample(self.out, "20260101_000002", 30.)
        _mark_verdict(old, None)
        self.assertFalse(storage.sample_is_empty(old))
        _quiet_report("测试", self.out, 7., True)
        self.assertFalse(os.path.exists(old))

    def test_unreadable_meta_follows_the_age_policy(self):
        old = _make_sample(self.out, "20260101_000003", 30.)
        with open(old + ".meta.json", "wb") as handle:
            handle.write(bytes([0xFF, 0xFE]) + b" not json")
        self.assertFalse(storage.sample_is_empty(old))
        _quiet_report("测试", self.out, 7., True)
        self.assertFalse(os.path.exists(old))

    def test_recent_loaded_sample_is_untouched(self):
        fresh = _make_sample(self.out, "20260101_000004", 1.)
        _mark_verdict(fresh, False)
        _quiet_report("测试", self.out, 7., True)
        self.assertTrue(os.path.exists(fresh))

    def test_dry_run_deletes_nothing(self):
        loaded = _make_sample(self.out, "20260101_000005", 30.)
        _mark_verdict(loaded, False)
        result = _quiet_report("测试", self.out, 7., False)
        self.assertTrue(os.path.exists(loaded), "预演竟然真删了")
        self.assertEqual(1, result["old"])
        self.assertEqual(0, result["removed_files"])



class TestCleanupSchedule(unittest.TestCase):
    """计划任务安装器：默认不写任何东西，生成的 bat 必须能被 cmd 读懂。

    "只保留 N 天"作为现场要求是要**定期执行**的，而 cli/cleanup.py 刻意
    默认只报不删。这个缺口靠操作系统的计划任务补，而不是把自动删除塞进
    采集服务 —— 删除混在采集进程里，出问题最难查。
    """

    def setUp(self):
        sys.path.insert(0, os.path.join(root(), "tools"))
        self.addCleanup(lambda: sys.path.remove(os.path.join(root(), "tools")))
        import install_cleanup_task
        self.mod = install_cleanup_task

    def _run(self, argv):
        import io as _io
        buf = _io.StringIO()
        with redirect_stdout(buf):
            code = self.mod.main(argv)
        return code, buf.getvalue()

    def test_dry_run_writes_nothing(self):
        """★ 预演不许落任何文件 —— 与 cleanup.py 同一套规矩。"""
        path = os.path.join(root(), self.mod.RUNNER)
        existed = os.path.exists(path)
        code, out = self._run([])
        self.assertEqual(0, code)
        self.assertIn("预演", out)
        self.assertEqual(existed, os.path.exists(path), "预演竟然写了文件")

    def test_rejects_bad_time(self):
        for bad in ("25:00", "3:60", "0330", "abc", ""):
            code, _ = self._run(["--at", bad])
            self.assertEqual(2, code, "非法时刻 %r 被接受了" % bad)

    def test_command_never_deletes_the_regression_baseline(self):
        """清理命令走的是 cleanup.py，而它对 data/loaded 有硬保护；
        这里只钉住"任务跑的确实是那个脚本"，不能悄悄换成 rm。"""
        argv = self.mod._cleanup_argv()
        self.assertTrue(any("cleanup" in str(a) for a in argv), argv)
        self.assertIn("--auto-days", argv, "天数应取配置，改配置不该重装任务")
        self.assertIn("--apply", argv, "计划任务不真删就没意义")

    def test_generated_bat_is_gbk_crlf(self):
        """.bat 必须 CRLF + GBK：中文 Windows 上 LF/UTF-8 的 bat
        会把中文注释吐成乱码，甚至把 rem 行当命令执行。"""
        import io as _io
        buf = _io.StringIO()
        with redirect_stdout(buf):
            with tempfile.TemporaryDirectory() as tmp:
                real = self.mod.ROOT
                try:
                    self.mod.ROOT = tmp
                    path = self.mod._write_runner(True)
                    raw = open(path, "rb").read()
                finally:
                    self.mod.ROOT = real
        self.assertIn(b"\r\n", raw)
        # 每个 LF 都必须有 CR 在前，一个裸 LF 都不许有
        self.assertEqual(raw.count(b"\n"), raw.count(b"\r\n"), "混进了裸 LF")
        raw.decode("gbk")
        self.assertIn(b"--auto-days", raw)
