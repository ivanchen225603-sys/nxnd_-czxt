# -*- coding: utf-8 -*-
"""docs/12 第 1 层那四条改进的测试。

四条都是**围着判定链做的辅助**，判定链本身一个字没动：
  1.1 判定帧也做完整性检查（残缺就推迟测量，超时照测但打标）
  1.2 判定链逐步的实测/门槛/余量落进结果与 meta（gate_report）
  1.3 值得人工打标的样本自动挑出来 + cli/label_truth.py
  1.4 看板显示"走到第几步被拦"（数据就是 1.2 的 gate_report）
"""
from __future__ import annotations

import io
import json
import os
import sys
import unittest

import numpy as np

from app.settings import baseline_rel

from ._helpers import ctx                      # noqa: F401
from app.settings import load_calib, load_devices, load_settings
from app.pointcloud import Frame, load_xyz, apply_roi
from app.detect import (detect_empty, evaluate_empty, gate_report,
                        R_PASS_EMPTY, R_FLOOR_LOW_RATIO)

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "cli")):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _sample(rel):
    """基线里的一份样本 —— 目录名走常量，不写死。"""
    return baseline_rel() + "/" + rel


def _one(rel):
    calib = load_calib("config/config.json")
    dev = load_devices("config/devices.json")
    fr = apply_roi(load_xyz(os.path.join(_ROOT, rel)), calib.roi)
    res = detect_empty(fr, calib)
    ok, why, dg = evaluate_empty(res, fr.n, res.cloud if res else fr.xyz, dev)
    return ok, why, dg, dev


class TestGateReport(unittest.TestCase):
    """1.2 —— 把 11 步摊成表：每步量什么、实测多少、门槛多少、余量多少。

    为什么需要：现场只能看到一句理由，**看不出离通过差多少**。
    2026-09-15 两辆空车的 z_above 是 0.318/0.349、门槛 0.08 ——
    差 4 倍还是差 1%，处理方式完全不同，而原来的输出分不出来。
    """

    def test_step_is_recorded(self):
        _ok, why, dg, _dev = _one(_sample("TEST001/20260721_160519.xyz"))
        self.assertEqual(why, R_FLOOR_LOW_RATIO)
        self.assertEqual(dg.step, 7, "「底面内点占比过低」是第 7 步")

    def test_pass_is_step_11(self):
        _ok, why, dg, _dev = _one(_sample("瀹丄09692D/20260721_190013.xyz"))
        if why != R_PASS_EMPTY:
            self.skipTest("当前 z_above_mode 下这份不判空车")
        self.assertEqual(dg.step, 11)

    def test_report_marks_exactly_one_blocker(self):
        _ok, _why, dg, dev = _one(_sample("TEST001/20260721_160519.xyz"))
        rows = gate_report(dg, dev)
        self.assertEqual(len(rows), 11, "11 步一步都不能少")
        blocked = [r for r in rows if r["state"] == "block"]
        self.assertEqual(len(blocked), 1, "只能有一步是「就是它拦下的」")
        self.assertEqual(blocked[0]["step"], dg.step)

    def test_steps_after_the_blocker_are_skipped(self):
        """短路之后的步骤必须标成 skip —— 否则会让人以为它们"过了"。"""
        _ok, _why, dg, dev = _one(_sample("TEST001/20260721_160519.xyz"))
        for r in gate_report(dg, dev):
            if r["step"] > dg.step:
                self.assertEqual(r["state"], "skip", f"第 {r['step']} 步应当是 skip")
            elif r["step"] < dg.step:
                self.assertEqual(r["state"], "pass", f"第 {r['step']} 步应当是 pass")

    def test_margin_sign_matches_pass_fail(self):
        """余量的符号必须与过没过一致 —— 正数 = 过了并且富余这么多。"""
        _ok, _why, dg, dev = _one(_sample("TEST001/20260721_160519.xyz"))
        for r in gate_report(dg, dev):
            if r["margin"] is None:
                continue
            if r["state"] == "block":
                self.assertLess(r["margin"], 0, f"第 {r['step']} 步拦下了，余量该是负的")
            elif r["state"] == "pass":
                self.assertGreaterEqual(r["margin"], 0,
                                        f"第 {r['step']} 步过了，余量该是非负的")

    def test_margin_direction_respects_the_operator(self):
        """">=" 与 "<=" 两种门槛的余量算法不一样，别算反。"""
        _ok, _why, dg, dev = _one(_sample("瀹丄09692D/20260721_190013.xyz"))
        rows = {r["step"]: r for r in gate_report(dg, dev)}
        r7 = rows[7]
        self.assertEqual(r7["op"], ">=")
        self.assertAlmostEqual(r7["margin"], r7["value"] - r7["limit"], places=4)
        r10 = rows[10]
        if r10["value"] is not None:
            self.assertEqual(r10["op"], "<=")
            self.assertAlmostEqual(r10["margin"], r10["limit"] - r10["value"], places=4)


class TestExtraDiagnostics(unittest.TestCase):
    """1.2 附带的两个纯诊断量：底面倾角与余高。判定链一个字都不看它们。"""

    def test_tilt_is_reported_for_the_tilted_empty_bed(self):
        """空载自卸车底板向车头翘 3.8~4.1 度 —— 正是它让原厂把空车判成载货。"""
        p = os.path.join(_ROOT, "data", "field", "未知车牌", "20260915_161309.xyz")
        if not os.path.exists(p):
            self.skipTest("现场样本不在")
        _ok, _why, dg, _dev = _one(os.path.relpath(p, _ROOT))
        self.assertGreater(dg.floor_tilt_deg, 3.0)
        self.assertLess(dg.floor_tilt_deg, 6.0)

    def test_freeboard_available_even_when_blocked_early(self):
        """★ 在第 6/7 步就被拦的样本也要能看到余高 —— 否则最需要它的时候没有。"""
        _ok, _why, dg, _dev = _one(_sample("TEST001/20260721_160519.xyz"))
        self.assertEqual(dg.step, 7)
        self.assertEqual(dg.freeboard, dg.freeboard, "余高不该是 nan")
        self.assertLess(dg.freeboard, 1.5, "满载车的余高应当很小")

    def test_diag_dict_carries_them(self):
        _ok, _why, dg, _dev = _one(_sample("TEST001/20260721_160519.xyz"))
        d = dg.to_dict()
        for k in ("step", "floor_tilt_deg", "freeboard", "z_above_used", "z_above_mode"):
            self.assertIn(k, d, f"看板/落盘要用的 {k} 没进 to_dict")


class TestWorthLabeling(unittest.TestCase):
    """1.3 —— 挑出**信息量最大**的样本让人工打标。

    全项目空车真值只有 2 份，而第 10 步的门槛正是靠它标的，余量只有 1.3~1.4 倍。
    样本荒是空车判定最大的短板（docs/12 §7 第 1 条）。
    """

    class _DG:
        def __init__(self, step, z=float("nan")):
            self.step = step
            self.z_above_used = z

    def _worker(self):
        from app.service import EventBus, ChannelWorker
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        return ChannelWorker(ch, EventBus(), st.plate, save_override=False)

    def test_pass_is_always_worth_labeling(self):
        w = self._worker()
        self.assertTrue(w._worth_labeling(self._DG(11)),
                        "判成空车的是正样本，一份都不能漏")

    def test_near_miss_on_step10_is_worth_labeling(self):
        w = self._worker()
        lim = w.dev.gate.z_above_min_max_m
        self.assertTrue(w._worth_labeling(self._DG(10, lim * 1.5)),
                        "第 10 步差一点点被拦的，正是门槛准不准的关键区")

    def test_far_miss_on_step10_is_not(self):
        w = self._worker()
        lim = w.dev.gate.z_above_min_max_m
        self.assertFalse(w._worth_labeling(self._DG(10, lim * 50)),
                         "差出几十倍的不用人看")

    def test_other_steps_are_not(self):
        w = self._worker()
        for step in (1, 3, 6, 7, 8, 9):
            self.assertFalse(w._worth_labeling(self._DG(step)),
                             f"第 {step} 步被拦的信息量不大")


class TestLabelTruthTool(unittest.TestCase):
    """1.3 的工具：cli/label_truth.py —— 标注与点云分离，存 csv。"""

    def setUp(self):
        import shutil
        import tempfile
        self.d = tempfile.mkdtemp(prefix="truth_")
        self.addCleanup(shutil.rmtree, self.d, ignore_errors=True)

    def test_roundtrip(self):
        import label_truth as LT
        labels = {"a/b.xyz": {"sample": "a/b.xyz", "label": "empty",
                              "by": "张三", "at": "2026-09-16 10:00", "note": "宁A"}}
        p = LT.save_labels(self.d, labels)
        self.assertTrue(os.path.exists(p))
        back = LT.load_labels(self.d)
        self.assertEqual(back["a/b.xyz"]["label"], "empty")
        self.assertEqual(back["a/b.xyz"]["by"], "张三", "中文标注人不能写坏")

    def test_missing_file_is_not_an_error(self):
        """第一次跑时 labels.csv 还不存在 —— 不许炸。"""
        import label_truth as LT
        self.assertEqual(LT.load_labels(self.d), {})

    def test_labels_are_the_documented_three(self):
        import label_truth as LT
        self.assertEqual(set(LT.LABELS), {"empty", "loaded", "unsure"})

    def test_scan_skips_raw_frames(self):
        """未裁 ROI 的整帧不参与打标 —— 它们没有判定结果，打了也没用。"""
        import label_truth as LT
        from app.pointcloud import save_xyz
        a = np.zeros((4, 3), dtype=np.float32)
        os.makedirs(os.path.join(self.d, "raw", "CH1"), exist_ok=True)
        os.makedirs(os.path.join(self.d, "未知车牌"), exist_ok=True)
        save_xyz(os.path.join(self.d, "raw", "CH1", "x.xyz"), Frame(a))
        save_xyz(os.path.join(self.d, "未知车牌", "y.xyz"), Frame(a))
        got = [os.path.basename(f) for f, _m in LT.scan(self.d)]
        self.assertIn("y.xyz", got)
        self.assertNotIn("x.xyz", got, "raw/ 下的整帧不该被扫进来")


class TestDegradedFrameGatesMeasurement(unittest.TestCase):
    """残帧撤销停稳窗口；超时不能把缺失证据变成测量许可。"""

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    def _worker(self, tmp=None):
        from app.service import EventBus, ChannelWorker
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        if tmp:
            ch.abs_output_dir = tmp
        w = ChannelWorker(ch, EventBus(), st.plate, save_override=bool(tmp), plate_mode="null")
        w.settle_sec = .2
        w.raw_frames = 0
        return w

    @staticmethod
    def _truck(n=40000, packets=450):
        r = np.random.default_rng(5)
        ground = np.stack([r.uniform(-2, 2, 8000), r.uniform(-11, 8, 8000),
                           -5.0 + r.normal(0, .01, 8000)], 1)
        car = np.stack([r.uniform(-1.2, 1.2, n), r.uniform(-6, 6, n),
                        r.uniform(-4.2, -2.5, n)], 1)
        return Frame(np.concatenate([ground, car]).astype(np.float32), packets=packets)

    def _to_settled(self, w):
        self.clock.settle(w, self._truck())
        self.assertEqual(w._detected_this_session, 0)

    def test_degraded_frame_postpones_measurement(self):
        w = self._worker()
        self._to_settled(w)
        self.clock.tick(w, self._truck(packets=200))
        self.assertEqual(w._detected_this_session, 0)
        self.assertTrue(w.st.stability["degraded"])
        self.assertFalse(w.st.stability["stable"])
        self.assertIsNone(w._settle_t0)

    def test_complete_frames_must_reestablish_stop_before_measurement(self):
        w = self._worker()
        self._to_settled(w)
        self.clock.tick(w, self._truck(packets=200))
        self.clock.tick(w, self._truck())
        self.assertEqual(w._detected_this_session, 0, "恢复首帧不能沿用旧窗口")
        for _ in range(25):
            self.clock.tick(w, self._truck())
        self.assertEqual(w._detected_this_session, 1)
        self.assertFalse(w._degraded_measure)

    def test_sustained_loss_never_forces_a_measurement(self):
        w = self._worker()
        self._to_settled(w)
        for _ in range(100):
            self.clock.tick(w, self._truck(packets=200))
        self.assertEqual(w._detected_this_session, 0)
        self.assertFalse(w.st.stability["stable"])
        self.assertEqual(len(w._raw_shots), 0)

    def test_only_recovered_complete_measurement_is_saved_with_gates(self):
        import tempfile
        import glob
        with tempfile.TemporaryDirectory(prefix="degr_") as tmp:
            w = self._worker(tmp)
            try:
                self._to_settled(w)
                for _ in range(100):
                    self.clock.tick(w, self._truck(packets=200))
                pattern = os.path.join(tmp, "archives-v1", "**", "detection.xyz.meta.json")
                self.assertEqual(glob.glob(pattern, recursive=True), [])
                self.assertEqual(w._detected_this_session, 0)
                for _ in range(30):
                    self.clock.tick(w, self._truck())
                from tests.test_result_finalizer import await_result
                await_result(w)
                metas = glob.glob(pattern, recursive=True)
                self.assertEqual(len(metas), 1)
                with io.open(metas[0], encoding="utf-8") as fp:
                    meta = json.load(fp)
                self.assertFalse(meta.get("frame_degraded"))
                self.assertEqual(len(meta["gates"]), 11)
            finally:
                w.stop()
                self.assertTrue(w.close_results(10.))
                w.source.close()


if __name__ == "__main__":
    unittest.main()


class TestAnalyzeLoad(unittest.TestCase):
    """3.2 双平面（底板 + 货面）—— 纯诊断，判定链一个字都不看它。

    ▲ 先说清楚它解决不了什么：**俯视单视角下，装满的车与底板更高的空车
      在几何上无法区分**。货把底板盖住之后雷达只看到一个平面，
      它是底板还是货面，单靠这一片点云说不出来。这是传感器布置的固有极限。

    它真正要抓的是**半装载**（底板露一部分、货占一部分）。
    ▲ **手上一份真实的半装载样本都没有**（见下面 test_no_real_half_load_sample
      的说明），所以正样本只能用合成的。这一条必须写在脸上，别拿它当已验证。
    """

    @staticmethod
    def _bed(nx=2.2, ny=10.0, z=-3.7, n=60000, seed=0, tilt=0.0):
        """一片底板。tilt 是沿 y 的倾角（度）—— 空载自卸车实测 3.8~4.1 度。"""
        r = np.random.default_rng(seed)
        x = r.uniform(-nx / 2, nx / 2, n)
        y = r.uniform(-ny / 2, ny / 2, n)
        return np.column_stack([x, y, z + y * np.tan(np.deg2rad(tilt))
                                + r.normal(0, 0.004, n)])

    @staticmethod
    def _walls(nx=2.2, ny=10.0, z0=-3.7, h=2.3, n=20000, seed=1):
        """两侧厢壁：从底板一直到壁顶，整个高度上都有点（掠射扫到的）。"""
        r = np.random.default_rng(seed)
        k = n // 2
        out = []
        for xv in (-nx / 2, nx / 2):
            out.append(np.column_stack([
                np.full(k, xv) + r.normal(0, 0.01, k),
                r.uniform(-ny / 2, ny / 2, k),
                r.uniform(z0, z0 + h, k)]))
        return np.concatenate(out)

    def test_empty_bed_reads_as_no_cargo(self):
        from app.detect import analyze_load
        a = np.concatenate([self._bed(), self._walls()])
        lp = analyze_load(a)
        self.assertTrue(lp.ok, lp.message)
        self.assertLess(lp.cargo_ratio, 0.05, "空车厢不该报出货")

    def test_tilted_empty_bed_still_reads_as_no_cargo(self):
        """★ 底板倾斜不许被当成"一半是货"。

        空载自卸车底板向车头翘 3.8~4.1 度、沿 y 落差 0.68~0.84 m。
        第一版按**绝对高度**切格子，把斜着的底板自己切成了"底板 + 货"——
        确认空车的 cargo_ratio 报成 85%（2026-09-16 踩过）。
        这与判定链第 10 步栽的是同一个跟头。
        """
        from app.detect import analyze_load
        a = np.concatenate([self._bed(tilt=4.0), self._walls()])
        lp = analyze_load(a)
        self.assertTrue(lp.ok, lp.message)
        self.assertLess(lp.cargo_ratio, 0.05,
                        "底板斜 4 度被当成货了 —— 度量没做到倾斜无关")

    def test_transverse_rib_is_not_cargo(self):
        """★ 横梁 / 前挡板是**一条线**，不是货。

        实测 `CH4_110959` 高出底板的格子全在 y=+0.78 一排、横跨整个车宽、
        高度恒定离地 4.02 m（挂车前挡板）；`CH4_161309` 是 y=-2.14/+1.36/+4.36
        三排横梁。不剔掉它们，空车的 cargo_ratio 会报成 8%~9%。
        """
        from app.detect import analyze_load
        r = np.random.default_rng(7)
        rib = np.column_stack([r.uniform(-1.1, 1.1, 3000),
                               r.uniform(0.7, 0.9, 3000),      # 只有 0.2 m 厚的一条
                               np.full(3000, -1.2) + r.normal(0, 0.01, 3000)])
        a = np.concatenate([self._bed(), self._walls(), rib])
        lp = analyze_load(a)
        self.assertLess(lp.cargo_ratio, 0.05,
                        "一格厚的横梁被当成货了 —— 少了「货是一片不是一条线」这条")

    def test_real_half_load_is_detected(self):
        """★ 正样本（合成）：车厢后半露底板、前半装到 1.5 m 高。"""
        from app.detect import analyze_load
        r = np.random.default_rng(11)
        bed = self._bed(ny=10.0)
        bed = bed[bed[:, 1] < 0.0]                       # 只留后半的底板
        k = 30000
        cargo = np.column_stack([r.uniform(-1.1, 1.1, k), r.uniform(0.2, 5.0, k),
                                 np.full(k, -3.7 + 1.5) + r.normal(0, 0.03, k)])
        a = np.concatenate([bed, cargo, self._walls()])
        lp = analyze_load(a)
        self.assertTrue(lp.ok, lp.message)
        self.assertGreater(lp.cargo_ratio, 0.25,
                           "半装载没认出来 —— 这正是双平面要解决的那一种")
        self.assertLess(lp.cargo_ratio, 0.85, "也不该报成基本装满")
        self.assertGreater(lp.cargo_depth, 1.0, "货面该比底板高一米多")
        self.assertIn("半装载", lp.message)

    def test_full_load_reads_as_one_surface(self):
        """装满时只看得到一个面 —— 如实报"一个面"，不要假装分得出来。"""
        from app.detect import analyze_load
        a = np.concatenate([self._bed(z=-2.2), self._walls(z0=-2.2, h=0.4)])
        lp = analyze_load(a)
        self.assertTrue(lp.ok, lp.message)
        self.assertLess(lp.cargo_ratio, 0.05)

    def test_too_few_points_is_reported_not_crashed(self):
        from app.detect import analyze_load
        lp = analyze_load(np.zeros((10, 3)))
        self.assertFalse(lp.ok)
        self.assertTrue(lp.message)

    def test_no_real_half_load_sample(self):
        """★ 把"没有真实正样本"这件事钉在测试里，别让它被悄悄忘掉。

        2026-09-16 逐份查过手上全部 65 份样本（10 份基线 + 53 份现场 + 2 辆确认空车）：
        **没有一份是半装载**。cargo_ratio 除 `CH4_121030`（37%，货堆表面不平、
        余高只有 0.48 说明它是满的）之外全是 0。
        docs/12 §4.4 原本把 `CH4_110959` 当成半装载，那是**误读** ——
        它高出底板的部分是挂车前挡板（一条线），不是货。

        所以：双平面的**正样本目前只有合成的**。这一条测试的存在就是提醒：
        哪天现场采到真的半装载，把它加进来，并把这条改掉。
        """
        import glob as _g
        root = os.path.join(_ROOT, "data", "truth")
        real = []
        if os.path.exists(os.path.join(root, "labels.csv")):
            import csv as _csv
            with io.open(os.path.join(root, "labels.csv"),
                         encoding="utf-8-sig", newline="") as f:
                real = [r for r in _csv.DictReader(f)
                        if (r.get("label") or "").strip() == "half"]
        self.assertEqual(real, [],
                         "现场采到真实半装载样本了 —— 把它加进 TestAnalyzeLoad，"
                         "并更新 docs/12 §4.4 与本条测试")
        _ = _g


class TestVolumeUsesBedPlane(unittest.TestCase):
    """基线平面及有独立裸露证据的底板修正。两件事：

      ① 基线**没覆盖到**的格子，按底板**平面**逐格求值，而不是拿常数 floor_z 糊。
         "车厢底板本来就是平的"只对了一半 —— 它是平面没错，但它是**斜的**：
         空载自卸车向车头翘 3.8~4.1 度，沿 y 落差 0.68~0.84 m。
         而基线覆盖率实测只有 79%（多帧合并 85~92%，docs/08 §10.11）。
      ② 用独立确认裸露的底板高度校正基线。基线是从另一辆车采的，
         而各车底板高度实测跨度 0.35 m（1.31~1.66 m）。

    单独analyze_load不能授权②：薄煤面也会被拟成底板，门槛内同样可能抹零。
    """

    NX, NY, TILT, H = 2.2, 10.0, 4.0, 1.0

    @property
    def tan(self):
        return float(np.tan(np.deg2rad(self.TILT)))

    @property
    def truth(self):
        return self.NX * self.NY * self.H

    def _bed(self, z0=-3.7, n=120000, ycut=None, seed=0):
        r = np.random.default_rng(seed)
        x = r.uniform(-self.NX / 2, self.NX / 2, n)
        y = r.uniform(-self.NY / 2, self.NY / 2, n)
        if ycut is not None:
            m = y < ycut
            x, y = x[m], y[m]
        return np.column_stack([x, y, z0 + y * self.tan + r.normal(0, .004, len(x))])

    def _walls(self, z0=-3.7, h=2.4, n=30000, seed=2):
        r = np.random.default_rng(seed)
        k, out = n // 2, []
        for xv in (-self.NX / 2, self.NX / 2):
            yy = r.uniform(-self.NY / 2, self.NY / 2, k)
            out.append(np.column_stack([np.full(k, xv), yy,
                                        z0 + yy * self.tan + r.uniform(0, h, k)]))
        return np.concatenate(out)

    def _baseline(self, ycut=None, with_plane=True):
        from app.detect import EmptyBaseline
        c = np.concatenate([self._bed(ycut=ycut), self._walls()]).astype(np.float32)
        lo, hi = c.min(axis=0), c.max(axis=0)
        fz = -3.7
        bp = (0.0, -self.tan, 1.0, 3.7) if with_plane else None
        return EmptyBaseline(fz, (float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]),
                                  fz, float(hi[2])), c, len(c), "t", bed_plane=bp)

    def _loaded(self, z0=-3.7, n=120000, seed=1):
        from app.detect import LoadedResult
        r = np.random.default_rng(seed)
        x = r.uniform(-self.NX / 2, self.NX / 2, n)
        y = r.uniform(-self.NY / 2, self.NY / 2, n)
        return LoadedResult(cloud=np.column_stack(
            [x, y, z0 + y * self.tan + self.H + r.normal(0, .01, n)]).astype(np.float32))

    def _err(self, v):
        return 100.0 * (v - self.truth) / self.truth

    def test_plane_fallback_removes_the_coverage_bias(self):
        """★ 基线覆盖不全时，常数兜底会偏，平面兜底不偏。"""
        from app.detect import compute_volume
        full = compute_volume(self._baseline(), self._loaded()).volume_m3
        for ycut in (2.0, 0.0):                    # 约 70% / 50% 覆盖
            const = compute_volume(self._baseline(ycut, False), self._loaded()).volume_m3
            plane = compute_volume(self._baseline(ycut, True), self._loaded()).volume_m3
            self.assertGreater(abs(self._err(const)), 3.5,
                               "前提：常数兜底在覆盖不全时确实会偏，否则本用例空转")
            self.assertLess(abs(self._err(plane)), abs(self._err(const)),
                            f"ycut={ycut}: 平面兜底应当比常数兜底更接近真值")
            self.assertLess(abs(plane - full), 0.2,
                            f"ycut={ycut}: 平面兜底应当与满覆盖基本一致")

    def test_old_baseline_without_plane_behaves_exactly_as_before(self):
        """★ 老 npz 没有 bed_plane —— 必须与从前**逐位一致**，不许悄悄改行为。"""
        from app.detect import compute_volume
        b = self._baseline(with_plane=False)
        self.assertIsNone(b.bed_plane)
        v = compute_volume(b, self._loaded()).volume_m3
        self.assertGreater(v, 0.0)
        # bed_z_at 在没有平面时必须退回常数
        z = b.bed_z_at(np.array([0.0, 1.0, -3.0]), np.array([-4.0, 0.0, 4.0]))
        self.assertTrue(np.allclose(z, b.floor_z))

    def test_bed_z_corrects_for_a_different_vehicle(self):
        """★ 被测车底板比基线车高 0.2 m —— 不校正会多算一大块。"""
        from app.detect import compute_volume
        from tests._bed_reference import reference
        base, lr = self._baseline(), self._loaded(z0=-3.5)
        raw = compute_volume(base, lr).volume_m3
        unverified = compute_volume(base, lr, bed_z=-3.5)
        self.assertEqual(raw, unverified.volume_m3)
        self.assertEqual("unverified", unverified.bed_reference_status)
        fixed = compute_volume(base, lr, **reference(base, lr, -3.5)).volume_m3
        self.assertGreater(self._err(raw), 10.0, "前提：不校正确实会多算")
        self.assertLess(abs(self._err(fixed)), 5.0, "校正后应当回到几个百分点以内")

    def test_full_truck_bed_z_is_not_adopted(self):
        """★ 保命：车装满时量到的是货面，照它平移会把体积算成 0。"""
        from app.detect import compute_volume
        base, lr = self._baseline(), self._loaded()
        plain = compute_volume(base, lr)
        bogus = compute_volume(base, lr, bed_z=-3.7 + 1.8)   # 货面，比底板高 1.8 m
        self.assertAlmostEqual(bogus.volume_m3, plain.volume_m3, places=6,
                               msg="超出门槛的 bed_z 必须不采纳")
        self.assertIn("未采纳", bogus.text)

    def test_shift_is_bounded_by_the_gate(self):
        """校正量本身被门槛限住上界 —— 不会出现"校正了 2 米"这种事。"""
        from app.detect import compute_volume
        from tests._bed_reference import reference
        base, lr = self._baseline(), self._loaded()
        for d in (0.49, 0.51):
            r = compute_volume(base, lr, dy=0., bed_shift_max_m=0.5, **reference(base, lr, -3.7 + d))
            if d < 0.5:
                self.assertIn("校正", r.text)
                self.assertEqual("applied", r.bed_reference_status)
                self.assertAlmostEqual(d, r.bed_shift_applied_m)
            else:
                self.assertIn("未采纳", r.text)
                self.assertEqual("rejected", r.bed_reference_status)
                self.assertEqual(0., r.bed_shift_applied_m)

    def test_no_bed_z_is_the_old_path(self):
        from app.detect import compute_volume
        base, lr = self._baseline(), self._loaded()
        a = compute_volume(base, lr).volume_m3
        b = compute_volume(base, lr, bed_z=None).volume_m3
        self.assertEqual(a, b)

    def test_baseline_capture_stores_the_bed_plane(self):
        """标定基线时要把底板平面一起存下来，并且能 round-trip。"""
        import shutil
        import tempfile
        from app.detect import EmptyBaseline, detect_empty
        from app.pointcloud import Frame
        calib = load_calib("config/config.json")
        cloud = np.concatenate([self._bed(z0=-4.0), self._walls(z0=-4.0)]).astype(np.float32)
        res = detect_empty(Frame(cloud), calib)
        self.assertIsNotNone(res)
        b = EmptyBaseline.from_empty_result(res, "t")
        self.assertIsNotNone(b)
        self.assertIsNotNone(b.bed_plane, "标定时应当把底板平面存下来")
        d = tempfile.mkdtemp(prefix="base_")
        try:
            p = os.path.join(d, "CH1.baseline.npz")
            b.save(p)
            back = EmptyBaseline.load(p)
            self.assertIsNotNone(back.bed_plane)
            for a_, b_ in zip(b.bed_plane, back.bed_plane):
                self.assertAlmostEqual(a_, b_, places=9)
        finally:
            shutil.rmtree(d, ignore_errors=True)


class TestVolumeFillHoles(unittest.TestCase):
    """给 compute_volume 补洞 —— 实时路径原来系统性低估 30%。

    2026-09-16 拿两辆有磅单的车实测（净重 53.60 t / 48.64 t）：

        补洞前  39.13 / 34.52 m^3  ->  密度 1.37 / 1.41 t/m^3
        补洞后  53.32 / 47.41 m^3  ->  密度 1.01 / 1.03 t/m^3

    **松散煤的堆积密度是 0.8~1.0 t/m^3，1.37~1.41 物理上不存在** ——
    这不是"两个实现略有出入"，是实时路径明确算错了。

    根因：`h = np.where(load_hit, ..., 0.0)` —— 载货侧没数据的格子直接记 0。
    而掠射角下车厢前端只剩 26 条 ring 够得着，0.10 m 格子下**载货侧只覆盖
    45%~60%**，一半格子被当成"没有货"。缝两边都有货，缝里当然也有货。

    ▲ 补洞不能单独做，三件事是一个整体（见 compute_volume 里的说明）：
      不限范围会渗到车厢外（实测冲到 87 m^3、密度 0.56，比不补还离谱）；
      车厢盒不圈准则厢壁那几格的参考面是壁顶，货高少算 1.5 m 以上。
    """

    def _case(self):
        return TestVolumeUsesBedPlane("test_no_bed_z_is_the_old_path")

    def test_off_reproduces_the_old_numbers(self):
        """★ fill_holes=False 必须完全退回 2026-09-16 之前的行为。"""
        from app.detect import compute_volume
        t = self._case()
        base, lr = t._baseline(), t._loaded()
        a = compute_volume(base, lr, fill_holes=False).volume_m3
        self.assertGreater(a, 0.0)
        # 关掉时不该有"补了 N 个空洞格"的字样
        self.assertNotIn("空洞", compute_volume(base, lr, fill_holes=False).text)

    def test_filling_is_reported(self):
        from app.detect import compute_volume
        t = self._case()
        r = compute_volume(t._baseline(), t._loaded())
        self.assertTrue(r.ok)

    def test_holes_are_actually_filled(self):
        """★ 人为在载货云上挖一条缝，补洞后体积要基本补回来。"""
        from app.detect import compute_volume
        t = self._case()
        base, lr = t._baseline(), t._loaded()
        full = compute_volume(base, lr).volume_m3
        c = np.asarray(lr.cloud, np.float64)
        # ★ 缝要挖得够宽。挖 0.6 m 时缺的体积只占 6%，正好卡在下面 8% 的容差里，
        #   把补洞换成空操作用例照样绿（当场踩过）。1.4 m 缺 14%，糊弄不过去。
        holed = c[~((c[:, 1] > 0.6) & (c[:, 1] < 2.0))]
        import copy as _c
        lr2 = _c.copy(lr)
        lr2.cloud = holed.astype(np.float32)
        with_fill = compute_volume(base, lr2).volume_m3
        without = compute_volume(base, lr2, fill_holes=False).volume_m3
        self.assertLess(without, full * 0.95, "前提：挖了缝之后不补洞确实会少算")
        self.assertGreater(with_fill, without, "补洞应当把缝里的货补回来")
        self.assertLess(abs(with_fill - full) / full, 0.06,
                        "补洞后应当基本回到没挖缝时的体积")
        self.assertGreater((full - without) / full, 0.10,
                           "前提：挖掉的这条缝必须足够大，否则容差能盖住它，用例空转")

    def test_result_is_stable_across_cell_sizes(self):
        """★ 补洞之后体积应当基本不随格子尺寸变 —— 这是"补对了"的标志。

        改之前车厢盒会跟着格子一起缩（0.10 m 时 10.80x2.50，0.30 m 时 9.90x2.10，
        面积 -23%），体积跟着掉 21%。根因是格子里取 max：粗格只要沾到一点厢壁，
        整格就被判成"壁"。改成按**点**定盒之后，真车实测收到 ±4%。
        """
        from app.detect import compute_volume
        t = self._case()
        base, lr = t._baseline(), t._loaded()
        vs = [compute_volume(base, lr, voxel=c).volume_m3 for c in (0.10, 0.15, 0.20)]
        spread = (max(vs) - min(vs)) / max(1e-9, float(np.mean(vs)))
        self.assertLess(spread, 0.12, f"体积随格子变化 {spread:.0%}，太大了：{vs}")

    def test_box_is_not_inflated_by_rounding(self):
        """★ 车厢盒按**格心**取，不能用 floor/ceil 往外各撑一格。

        0.10 m 格子下，2.2 x 10.0 m 的车厢会被撑到 2.4 x 10.2（面积 +11%），
        再叠上补洞把货高外推进去，体积就系统性偏大（当场踩过：
        同车对照从 -3.3% 变成 +6.0%）。
        """
        from app.detect import compute_volume
        t = self._case()
        v = compute_volume(t._baseline(), t._loaded()).volume_m3
        self.assertLess(abs(v - t.truth) / t.truth, 0.06,
                        f"同车对照应当在几个百分点内，实际 {v:.2f} vs 真值 {t.truth:.2f}")

    def test_no_baseline_box_means_no_fill(self):
        """圈不出车厢盒时**不补** —— 无界松弛比不补还糟（实测密度 0.56）。"""
        from app.detect import EmptyBaseline, compute_volume
        t = self._case()
        b = t._baseline()
        thin = EmptyBaseline(b.floor_z, b.box, b.cloud[:30], 30, "", bed_plane=b.bed_plane)
        r = compute_volume(thin, t._loaded())
        self.assertNotIn("空洞", r.text)
        self.assertFalse(r.ok, "无有效积分区不能退回全网格成功值")
        self.assertIsNone(r.to_dict()["volume_m3"])


class TestVolumeWallCellsAreBlind(unittest.TestCase):
    """★ 厢壁格是**盲区**，不能只换参考面而把载货侧照旧用实测。

    那一列被厢壁挡着，看不到里面有没有货。若把参考面换成底板平面、
    而载货侧仍用实测高度（= **壁顶**），h 就变成整个厢壁的高度 ——
    凭空造出货来。合成场景实测：240 个壁格造出 **3.06 m^3**，
    而那辆车厢里什么都没有。

    正确做法是把壁格也当成空洞按邻居补：空车时邻居是底板（h 约 0），
    载货时邻居是货面，两边都对。
    """

    def test_empty_carriage_with_tall_walls_is_near_zero(self):
        from app.detect import EmptyBaseline, compute_volume, detect_empty, detect_loaded
        from app.pointcloud import Frame
        calib = load_calib("config/config.json")
        t = TestVolumeUsesBedPlane("test_no_bed_z_is_the_old_path")
        # 空车厢：底板 + 2.4 m 高的厢壁，车厢里什么都没有
        cloud = np.concatenate([t._bed(), t._walls()]).astype(np.float32)
        base = EmptyBaseline.from_empty_result(detect_empty(Frame(cloud), calib), "t")
        self.assertIsNotNone(base)
        lr = detect_loaded(Frame(cloud), calib)
        v = compute_volume(base, lr)
        self.assertTrue(v.ok, v.text)
        self.assertTrue(np.isfinite(v.volume_m3))
        self.assertLess(v.volume_m3, 1.0,
                        f"车厢空的却算出 {v.volume_m3:.2f} m3 —— 多半是把厢壁高度当成货了")

    def test_walls_are_counted_as_holes_not_as_cargo(self):
        """壁格应当进"补洞"的账，而不是直接拿壁顶去减底板。"""
        from app.detect import EmptyBaseline, compute_volume, detect_empty, detect_loaded
        from app.pointcloud import Frame
        calib = load_calib("config/config.json")
        t = TestVolumeUsesBedPlane("test_no_bed_z_is_the_old_path")
        cloud = np.concatenate([t._bed(), t._walls()]).astype(np.float32)
        base = EmptyBaseline.from_empty_result(detect_empty(Frame(cloud), calib), "t")
        lr = detect_loaded(Frame(cloud), calib)
        r = compute_volume(base, lr)
        self.assertIn("空洞", r.text, "壁格应当计入补洞格数")


class TestVolumeRegistration(unittest.TestCase):
    """纵向配准：空车与载货是两次停车，位置不一样。

    2026-09-15 实测 CH4 21 辆车，车厢后沿 y0 的 std 0.864 m、极差 4.00 m。
    基线没对齐就整体错位，车厢中段的货高会被减到厢底以外去。

    实测灵敏度（两辆有磅单的车）：dy 从 -1.5 扫到 +2.0，体积差 **18%** ——
    配准值不值得做不是问题，问题是**估得准不准**。

    ▲ 两个走不通的估法（都真试过）：
      · 基线用**底板**点、载货用**货面**点找后沿 —— 量的不是同一个特征
        （货可能堆不到车厢最后头），估出 +1.88/+1.98；
      · 最大化两个足迹的 IoU —— 被 +y 的驾驶室/门架拽偏，估出 -0.90，
        **连符号都反了**。
    ▲ 还有一个纯粹的 bug：门槛取"最大行数的百分之几"。俯视雷达天底那一行
      点数是别处的几十倍（一行 2 万点），比例门槛被抬到 400，
      把真正的后沿（一行两三百点）整段跳过，估出 +2.03。**必须用绝对门槛。**

    最后留下的那一个估法（两边都取高出底板的点 + 绝对门槛找后沿）
    在两辆车上给 +1.03 / +1.07，与 `cli/volume_calc.py` 独立算出的 +1.00 吻合。
    """

    def _case(self):
        return TestVolumeUsesBedPlane("test_no_bed_z_is_the_old_path")

    def test_recovers_a_known_shift(self):
        """★ 把载货云整体挪已知的一段，估出来的 dy 要对得上。"""
        from app.detect import estimate_dy
        import copy as _c
        t = self._case()
        base = t._baseline()
        for truth in (0.0, 0.5, 1.0, -0.75):
            lr = t._loaded()
            c = np.asarray(lr.cloud, np.float64).copy()
            c[:, 1] += truth
            lr2 = _c.copy(lr)
            lr2.cloud = c.astype(np.float32)
            d, note = estimate_dy(base, lr2)
            self.assertLess(abs(d - truth), 0.35,
                            f"真值 {truth:+.2f} 估成 {d:+.2f}（{note}）")

    def test_absolute_threshold_survives_a_dense_row(self):
        """★ 天底那一行点数极大时，估计不能被它带偏（比例门槛就会）。"""
        from app.detect import estimate_dy
        import copy as _c
        t = self._case()
        base = t._baseline()
        lr = t._loaded()
        c = np.asarray(lr.cloud, np.float64)
        r = np.random.default_rng(5)
        # 在 y=0 附近塞一坨极密的点，模拟天底
        dense = np.column_stack([r.uniform(-1.0, 1.0, 60000),
                                 r.uniform(-0.1, 0.1, 60000),
                                 np.full(60000, -2.7) + r.normal(0, .01, 60000)])
        lr2 = _c.copy(lr)
        lr2.cloud = np.concatenate([c, dense]).astype(np.float32)
        d, _n = estimate_dy(base, lr2)
        self.assertLess(abs(d), 0.35, f"被天底那坨点带偏了：dy={d:+.2f}")

    def test_out_of_range_is_refused(self):
        """估出离谱的值就不配准 —— 配反了比不配更糟（实测差 18%）。"""
        from app.detect import estimate_dy
        import copy as _c
        t = self._case()
        lr = t._loaded()
        c = np.asarray(lr.cloud, np.float64).copy()
        c[:, 1] += 6.0
        lr2 = _c.copy(lr)
        lr2.cloud = c.astype(np.float32)
        d, note = estimate_dy(t._baseline(), lr2, max_dy_m=2.5)
        self.assertEqual(d, 0.0)
        self.assertIn("不配准", note)

    def test_explicit_zero_disables(self):
        """dy=0.0 明确表示"不配准"，不能被自动估计覆盖。"""
        from app.detect import compute_volume
        t = self._case()
        base, lr = t._baseline(), t._loaded()
        r = compute_volume(base, lr, dy=0.0)
        self.assertNotIn("配准", r.text)

    def test_registration_fixes_a_misaligned_baseline(self):
        """★ 基线与载货车错位 1 m 时，配准要把体积拉回来。"""
        from app.detect import compute_volume
        import copy as _c
        t = self._case()
        base = t._baseline()
        lr = t._loaded()
        c = np.asarray(lr.cloud, np.float64).copy()
        c[:, 1] += 1.0                                  # 载货车停得靠前 1 m
        lr2 = _c.copy(lr)
        lr2.cloud = c.astype(np.float32)
        off = compute_volume(base, lr2, dy=0.0).volume_m3
        on = compute_volume(base, lr2).volume_m3
        aligned = compute_volume(base, t._loaded()).volume_m3
        self.assertLess(off, aligned * 0.97, "前提：错位确实会让体积变差")
        self.assertLess(abs(on - aligned) / aligned, 0.05,
                        f"配准后应当回到对齐时的水平：{on:.2f} vs {aligned:.2f}")


class _CarriageScene:
    """合成车厢（带厚度的壁顶 + 内侧面），横向/姿态两组配准用例共用。"""

    W, L, WALL, HW, H, TILT = 2.4, 10.0, 0.05, 1.6, 1.0, 2.0   # 内宽/长/壁厚/壁高/货高/底板坡度°

    def _z(self, y):
        return -3.7 + y * float(np.tan(np.deg2rad(self.TILT)))

    def _walls(self, r, xc, z_lo, n=6000):
        """两侧壁：内面从 z_lo 到壁顶的回波 + 厚 WALL 的壁顶。"""
        out = []
        for s in (-1.0, 1.0):
            y = r.uniform(-self.L / 2, self.L / 2, n)
            out.append(np.column_stack([np.full(n, xc + s * self.W / 2), y,
                                        self._z(y) + r.uniform(z_lo, self.HW, n)]))
            y = r.uniform(-self.L / 2, self.L / 2, n)
            out.append(np.column_stack([xc + s * (self.W / 2 + r.uniform(0, self.WALL, n)), y,
                                        self._z(y) + self.HW + r.normal(0, .004, n)]))
        return out

    def _surface(self, r, xc, h, sd, n=60000):
        x = r.uniform(xc - self.W / 2, xc + self.W / 2, n)
        y = r.uniform(-self.L / 2, self.L / 2, n)
        return np.column_stack([x, y, self._z(y) + h + r.normal(0, sd, n)])

    def _baseline(self, xc=0.0, seed=0):
        from app.detect import EmptyBaseline
        r = np.random.default_rng(seed)
        c = np.concatenate([self._surface(r, xc, 0.0, .004)] + self._walls(r, xc, 0.0)).astype(np.float32)
        lo, hi = c.min(axis=0), c.max(axis=0)
        tan = float(np.tan(np.deg2rad(self.TILT)))
        return EmptyBaseline(-3.7, (float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]),
                                    -3.7, float(hi[2])), c, len(c), "t", bed_plane=(0.0, -tan, 1.0, 3.7))

    def _loaded(self, xc=0.0, seed=1, clip_below=None, sd=0.01):
        from app.detect import LoadedResult
        r = np.random.default_rng(seed)
        c = np.concatenate([self._surface(r, xc, self.H, sd)] + self._walls(r, xc, self.H))
        if clip_below is not None:                     # 模拟一侧壁被 ROI 截掉
            c = c[c[:, 0] >= clip_below]
        return LoadedResult(cloud=c.astype(np.float32))


class TestVolumeLateralRegistration(_CarriageScene, unittest.TestCase):
    """横向配准（2026-10-01）：空车与载货两次停车左右位置不同，跨车道时坐标更不同。

    2026-09-28 宁A66912A（载货 CH3、空车 CH4）横向错开 0.30 m：体积 35.99 m^3，
    挪齐后 30.30 —— 多算 19%。厢壁格按**基线**的厢壁位置划，载货车的壁顶错开后
    落进基线的底板格，整列壁顶被当成煤。下面的合成车厢复现同一机理。
    """

    def test_recovers_a_known_offset(self):
        """★ 载货车整体横移已知距离，估出来要对得上（两种估法都得过）。"""
        from app.detect import estimate_lateral_registration
        # 最后一组是跨道最坏情形：空车标定道与载货道的车厢中心相差约 0.65 m。
        for base_xc, load_xc in ((0.0, 0.0), (0.0, 0.12), (0.0, -0.30), (0.0, 0.45), (-0.30, 0.35)):
            truth = load_xc - base_xc
            d = estimate_lateral_registration(self._baseline(xc=base_xc), self._loaded(xc=load_xc))
            self.assertEqual("resolved", d.status, d.reason)
            self.assertLess(abs(d.dx_m - truth), 0.02, f"真值 {truth:+.2f} 估成 {d.dx_m:+.3f}（{d.reason}）")

    def test_registration_removes_the_wall_column_inflation(self):
        """★ 错开 0.30 m 不对齐会多算（壁顶进了底板格）；自动对齐后回到对齐时的水平。"""
        from app.detect import compute_volume
        base, lr = self._baseline(), self._loaded(xc=-0.30)
        aligned = compute_volume(base, self._loaded(), dy=0.0, dx=0.0).volume_m3
        off = compute_volume(base, lr, dy=0.0, dx=0.0).volume_m3
        on = compute_volume(base, lr, dy=0.0)
        self.assertGreater(off, aligned * 1.05, f"前提：横向错开确实会多算：{off:.2f} vs {aligned:.2f}")
        self.assertLess(abs(on.volume_m3 - aligned) / aligned, 0.02,
                        f"对齐后应当回到对齐时的水平：{on.volume_m3:.2f} vs {aligned:.2f}")
        self.assertIn("横向对齐 -0.30", on.text)

    def test_offset_beyond_limit_is_not_applied(self):
        """估出超过上限就不挪 —— 等于旧行为，不会比原来更糟。"""
        from app.detect import compute_volume, estimate_lateral_registration
        base, lr = self._baseline(xc=-0.45), self._loaded(xc=0.45)    # 相对 0.90 m，两边都没出 ROI
        d = estimate_lateral_registration(base, lr)
        self.assertEqual(("unresolved", 0.0), (d.status, d.dx_m))
        self.assertIn("超过上限", d.reason)
        auto = compute_volume(base, lr, dy=0.0)
        # 旧行为 = 不横移、不转、旧积分盒：左右位置没对上时偏航和内腔积分域也跳过
        # （错开 0.9 m 时体积本来就乱，一点剪切都会晃它；内侧面摆在没对齐的地方更糟）。
        self.assertEqual(compute_volume(base, lr, dy=0.0, dx=0.0, yaw=0.0, inner=False).volume_m3, auto.volume_m3)
        self.assertIn("横向未对齐", auto.text)
        self.assertIn("偏航未对齐：左右位置没对上", auto.text)
        self.assertIn("内腔未用：左右位置没对上", auto.text)

    def test_the_two_estimates_must_agree(self):
        """车厢盒校核与侧壁外包对不上就不挪。"""
        from unittest import mock
        import app.detect as D
        real, calls = D.carriage_box, []

        def shifted_second(xyz, **kw):
            box = real(xyz, **kw)
            calls.append(1)
            if len(calls) == 2:                        # 第二次切的是载货
                box = dict(box, x0=box["x0"] + 0.10, x1=box["x1"] + 0.10)
            return box
        with mock.patch.object(D, "carriage_box", side_effect=shifted_second):
            d = D.estimate_lateral_registration(self._baseline(), self._loaded(xc=-0.30))
        self.assertEqual(("unresolved", 0.0), (d.status, d.dx_m))
        self.assertIn("两种估法不一致", d.reason)

    def test_clipped_side_wall_is_refused(self):
        """★ 一侧壁被截掉时两种估法会**一起**偏，只有外包宽度看得出来。"""
        from app.detect import carriage_box, estimate_lateral_registration
        lr = self._loaded(xc=-0.30, clip_below=-1.35)
        self.assertIsNotNone(carriage_box(np.asarray(lr.cloud, np.float64), x_lim=2.0),
                             "前提：截掉一侧后车厢仍切得出来，否则本用例空转")
        d = estimate_lateral_registration(self._baseline(), lr)
        self.assertEqual(("unresolved", 0.0), (d.status, d.dx_m))
        self.assertIn("宽度差", d.reason)

    def test_explicit_zero_disables(self):
        """dx=0.0 明确表示"不做横向配准"，不能被自动估计覆盖。"""
        from app.detect import compute_volume
        r = compute_volume(self._baseline(), self._loaded(xc=-0.30), dy=0.0, dx=0.0)
        self.assertNotIn("横向", r.text)

    def test_auto_estimates_once_for_all_cells(self):
        """compute_volume_auto 只估一次、各档共用；结果与直接调用一致。"""
        from unittest import mock
        import app.detect as D
        base, lr = self._baseline(), self._loaded(xc=-0.30)
        with mock.patch.object(D, "estimate_lateral_registration",
                               wraps=D.estimate_lateral_registration) as est:
            v = D.compute_volume_auto(base, lr, dy=0.0)
        self.assertEqual(1, est.call_count)
        direct = D.compute_volume(base, lr, voxel=v.voxel_size, dy=0.0)
        self.assertEqual(direct.volume_m3, v.volume_m3)
        self.assertIn("横向对齐 -0.30", v.text)

    def test_shift_keeps_the_bed_plane_in_place(self):
        """平移基线：点云 x、车厢盒 x 跟着挪，底板平面在挪过的位置取原来的高度。"""
        from app.detect import EmptyBaseline, _shift_baseline_x
        cloud = np.random.default_rng(3).uniform(-1, 1, (400, 3)).astype(np.float32)
        b = EmptyBaseline(-3.7, (-1.25, 1.25, -5.0, 5.0, -3.7, -2.0), cloud, 400, "t",
                          bed_plane=(0.05, -0.054, 0.997, 3.67))
        s = _shift_baseline_x(b, -0.3)
        x, y = np.array([-1.0, 0.0, 0.8]), np.array([-4.0, 0.0, 3.0])
        np.testing.assert_allclose(s.bed_z_at(x - 0.3, y), b.bed_z_at(x, y))
        np.testing.assert_allclose(s.cloud[:, 0], cloud[:, 0].astype(np.float64) - 0.3)
        np.testing.assert_allclose(s.box, (-1.55, 0.95, -5.0, 5.0, -3.7, -2.0))
        self.assertEqual((s.floor_z, s.n), (b.floor_z, b.n))
        np.testing.assert_array_equal(b.cloud, cloud)          # 原基线不许被改

    def test_adopted_bed_reference_keeps_the_original_frame(self):
        """已采纳的独立底板参考按原基线坐标核定：自动估的不挪，显式给的拒绝。"""
        from app.detect import compute_volume
        from tests._bed_reference import reference
        base, lr = self._baseline(), self._loaded(xc=-0.30)
        r = compute_volume(base, lr, dy=0.0, **reference(base, lr, -3.65))
        self.assertEqual("applied", r.bed_reference_status)
        self.assertIn("横向未对齐：已采纳的独立底板参考", r.text)
        explicit = compute_volume(base, lr, dy=0.0, dx=-0.30, **reference(base, lr, -3.65))
        self.assertFalse(explicit.ok)
        self.assertIn("横向偏移不能与已采纳的独立底板参考同时使用", explicit.text)


class TestVolumePoseRegistration(_CarriageScene, unittest.TestCase):
    """姿态对齐（2026-10-01）：用两次都看得见的壁顶，估载货车厢相对空车的高差、横滚、俯仰。

    底板被煤盖住看不见，体积拿空车的底板当参考。2026-09-28 宁A05922F 同一条道载货比空车
    低 4~5 cm（满载压下悬挂和轮胎），不修就按空车的底板高度算煤厚，少算一个下沉量。
    """

    def _shaped(self, dz=0.0, roll=0.0, pitch=0.0, xc=0.0, field=None, seed=1):
        """整个载货车（货面 + 两侧壁）按 dz + x*tan(横滚) + y*tan(俯仰) 抬/压，模拟车厢整体姿态变化。"""
        from app.detect import LoadedResult
        c = np.asarray(self._loaded(xc=xc, seed=seed).cloud, np.float64).copy()
        c[:, 2] += (dz + (c[:, 0] - xc) * np.tan(np.deg2rad(roll)) + c[:, 1] * np.tan(np.deg2rad(pitch)))
        if field is not None:
            c[:, 2] += field(c)
        return LoadedResult(cloud=c.astype(np.float32))

    def test_recovers_height_roll_and_pitch(self):
        """★ 已知的高差 / 横滚 / 俯仰要估得回来。"""
        from app.detect import estimate_pose_registration
        base = self._baseline()
        for dz, roll, pitch in ((-0.05, 0.0, 0.0), (-0.03, 3.0, -2.0), (0.04, -2.0, 1.0)):
            d = estimate_pose_registration(base, self._shaped(dz, roll, pitch))
            self.assertEqual("resolved", d.status, d.reason)
            xm = (d.x_left + d.x_right) / 2
            c0 = float(d.at(xm, d.y_mid))
            want = dz + xm * np.tan(np.deg2rad(roll)) + d.y_mid * np.tan(np.deg2rad(pitch))   # 估计点处的真值
            got_roll = np.degrees(np.arctan((d.right[0] - d.left[0]) / (d.x_right - d.x_left)))
            got_pitch = np.degrees(np.arctan((d.left[1] + d.right[1]) / 2))
            self.assertLess(abs(c0 - want), 0.003, f"高差 {want:+.3f} 估成 {c0:+.3f}（{d.reason}）")
            self.assertLess(abs(got_roll - roll), 0.3, f"横滚 {roll:+.1f}° 估成 {got_roll:+.2f}°")
            self.assertLess(abs(got_pitch - pitch), 0.2, f"俯仰 {pitch:+.1f}° 估成 {got_pitch:+.2f}°")

    def test_sag_correction_restores_the_volume(self):
        """★ 载货车整体低 5 cm：不修少算约 5%（煤厚 1 m），修了回到没下沉时的水平。"""
        from app.detect import compute_volume
        base = self._baseline()
        truth = compute_volume(base, self._loaded(), dy=0.0, dx=0.0, pose=False).volume_m3
        lr = self._shaped(dz=-0.05)
        off = compute_volume(base, lr, dy=0.0, dx=0.0, pose=False).volume_m3
        on = compute_volume(base, lr, dy=0.0, dx=0.0)
        self.assertLess(off, truth * 0.97, f"前提：下沉确实会少算：{off:.2f} vs {truth:.2f}")
        self.assertLess(abs(on.volume_m3 - truth) / truth, 0.01,
                        f"修正后应当回到没下沉时的水平：{on.volume_m3:.2f} vs {truth:.2f}")
        self.assertIn("姿态对齐：中心高差 -5.0", on.text)

    def test_lateral_and_pose_together(self):
        """横向错开 0.30 m 又整体低 4 cm：两步都做才回到对齐时的水平。"""
        from app.detect import compute_volume
        base = self._baseline()
        truth = compute_volume(base, self._loaded(), dy=0.0, dx=0.0, pose=False).volume_m3
        r = compute_volume(base, self._shaped(dz=-0.04, xc=-0.30), dy=0.0)
        self.assertLess(abs(r.volume_m3 - truth) / truth, 0.01, f"{r.volume_m3:.2f} vs {truth:.2f}（{r.text}）")
        self.assertIn("横向对齐 -0.30", r.text)
        self.assertIn("姿态对齐：中心高差 -4.0", r.text)

    def test_local_board_change_does_not_drag_the_estimate(self):
        """★ 右壁后段边板装货时比空车高 25 cm（宁A66912A 见过）：当坏段剔掉，不许把高差带偏。"""
        from app.detect import estimate_pose_registration

        def raise_right_rear(c):
            top = (c[:, 0] > self.W / 2 + 0.01) & (c[:, 1] < -2.0)
            return np.where(top, 0.25, 0.0)
        d = estimate_pose_registration(self._baseline(), self._shaped(dz=-0.04, field=raise_right_rear))
        self.assertEqual("resolved", d.status, d.reason)
        c0 = float(d.at((d.x_left + d.x_right) / 2, d.y_mid))
        self.assertLess(abs(c0 + 0.04), 0.01, f"高差应仍是 -0.04，估成 {c0:+.3f}（{d.reason}）")

    def test_twisted_walls_are_refused(self):
        """两侧壁前后倾相反（不像同一个车厢的整体移动）就不修。"""
        from app.detect import estimate_pose_registration

        def twist(c):
            return np.where(c[:, 0] < 0, 1.0, -1.0) * c[:, 1] * np.tan(np.deg2rad(3.0))
        d = estimate_pose_registration(self._baseline(), self._shaped(field=twist))
        self.assertEqual("unresolved", d.status)
        self.assertIn("不像同一个车厢", d.reason)

    def test_missing_wall_is_not_corrected(self):
        """一侧壁整段看不见（被截掉）就不修 —— 退回旧行为。"""
        from app.detect import compute_volume, estimate_pose_registration
        from app.detect import LoadedResult
        c = np.asarray(self._shaped(dz=-0.05).cloud, np.float64)
        lr = LoadedResult(cloud=c[c[:, 0] < self.W / 2 - 0.01].astype(np.float32))
        base = self._baseline()
        d = estimate_pose_registration(base, lr)
        self.assertEqual("unresolved", d.status)
        on = compute_volume(base, lr, dy=0.0, dx=0.0)
        self.assertEqual(compute_volume(base, lr, dy=0.0, dx=0.0, pose=False).volume_m3, on.volume_m3)
        self.assertIn("姿态未对齐", on.text)

    def test_explicit_off(self):
        """pose=False 明确不做，结果与旧行为一致、说明里不出现"姿态"。"""
        from app.detect import compute_volume
        r = compute_volume(self._baseline(), self._shaped(dz=-0.05), dy=0.0, dx=0.0, pose=False)
        self.assertNotIn("姿态", r.text)

    def test_auto_estimates_once_and_rejects_a_stale_decision(self):
        """compute_volume_auto 只估一次；传进来的估计若是按别的 dy 算的，就重估。"""
        from unittest import mock
        import app.detect as D
        base, lr = self._baseline(), self._shaped(dz=-0.05)
        with mock.patch.object(D, "estimate_pose_registration", wraps=D.estimate_pose_registration) as est:
            v = D.compute_volume_auto(base, lr, dy=0.0, dx=0.0)
        # 5 档候选共用 1 次；误差估计按 dy ±0.10 重算时姿态跟着重估各 1 次（见 estimate_volume_uncertainty）。
        self.assertEqual(3, est.call_count)
        self.assertEqual({0.0, -D.VOLUME_ERR_DY_M, D.VOLUME_ERR_DY_M},
                         {round(call.args[3], 9) for call in est.call_args_list})
        self.assertEqual(D.compute_volume(base, lr, voxel=v.voxel_size, dy=0.0, dx=0.0).volume_m3, v.volume_m3)
        stale = D.estimate_pose_registration(base, lr, 0.0, 0.5)
        with mock.patch.object(D, "estimate_pose_registration", wraps=D.estimate_pose_registration) as est:
            r = D.compute_volume(base, lr, voxel=v.voxel_size, dy=0.0, dx=0.0, pose=stale)
        self.assertEqual(1, est.call_count, "偏移对不上的现成估计必须重估")
        self.assertEqual(v.volume_m3, r.volume_m3)

    def test_adopted_bed_reference_skips_pose(self):
        """已采纳的独立底板参考就是这一车实测的底板高度，再叠壁顶高差会重复修正。"""
        from app.detect import compute_volume
        from tests._bed_reference import reference
        base, lr = self._baseline(), self._shaped(dz=-0.05)
        r = compute_volume(base, lr, dy=0.0, dx=0.0, **reference(base, lr, -3.75))
        self.assertEqual("applied", r.bed_reference_status)
        self.assertIn("姿态未对齐：已采纳的独立底板参考", r.text)


class TestVolumeCrossCheck(_CarriageScene, unittest.TestCase):
    """逐车交叉校核（2026-10-01）：只用这一车两次扫描的相对高度另算一遍体积，
    抓主算法特有的粗错（配准没配上、积分范围圈错）。超 10% 只标"需复核"，不改体积。"""

    def test_flat_load_matches_the_truth(self):
        """煤厚 1 m、内宽 2.4 m、内长 10 m 的合成车厢：校核值约 24 m^3。

        煤面噪声取小：每格取最高点，噪声 1 cm、每格上百点时会系统性偏高约 2.5 cm
        （主算法同样按格取最高，两者相比时抵消）；这里只验几何对不对。"""
        from app.detect import cross_check_volume
        cc = cross_check_volume(self._baseline(), self._loaded(sd=0.002))
        self.assertEqual("done", cc.status, cc.reason)
        self.assertAlmostEqual(cc.mean_depth_m, self.H, delta=0.01)
        self.assertLess(abs(cc.volume_m3 - self.W * self.L * self.H) / (self.W * self.L * self.H), 0.02,
                        f"{cc.volume_m3:.2f}（{cc.reason}）")

    def test_flags_a_misaligned_main_result(self):
        """★ 横向错开 0.30 m 又不让主算法对齐（旧口径）：主算法多算，交叉校核不跟着错 -> 标"需复核"。"""
        from app.detect import compute_volume_auto
        base, lr = self._baseline(), self._loaded(xc=-0.30)
        old = compute_volume_auto(base, lr, dy=0.0, dx=0.0, pose=False, inner=False)
        self.assertGreater(old.volume_m3, old.cross_check.volume_m3 * 1.10, "前提：旧口径确实多算 10% 以上")
        self.assertIn("需复核", old.text)
        new = compute_volume_auto(base, lr, dy=0.0)
        self.assertIn("交叉校核", new.text)
        self.assertNotIn("需复核", new.text)

    def test_unaffected_by_sag(self):
        """载货车整体低 5 cm（满载下沉）：校核是相对本车壁顶算的，不跟着变。"""
        from app.detect import LoadedResult, cross_check_volume
        base = self._baseline()
        c = np.asarray(self._loaded().cloud, np.float64).copy()
        c[:, 2] -= 0.05
        a = cross_check_volume(base, self._loaded()).volume_m3
        b = cross_check_volume(base, LoadedResult(cloud=c.astype(np.float32))).volume_m3
        self.assertLess(abs(a - b) / a, 0.01, f"{a:.2f} vs {b:.2f}")

    def test_unavailable_when_a_wall_is_missing(self):
        """一侧壁整段看不见：校核做不了，说明里写明，体积照常给。"""
        from app.detect import LoadedResult, compute_volume_auto, cross_check_volume
        c = np.asarray(self._loaded().cloud, np.float64)
        lr = LoadedResult(cloud=c[c[:, 0] < self.W / 2 - 0.01].astype(np.float32))
        base = self._baseline()
        self.assertEqual("unavailable", cross_check_volume(base, lr).status)
        v = compute_volume_auto(base, lr, dy=0.0)
        self.assertTrue(v.ok, v.text)
        self.assertIn("交叉校核不可用", v.text)

    def test_never_changes_the_volume_or_the_payload(self):
        """校核只写说明、挂在 cross_check 上：体积与直接计算一致，to_dict 不多出键（旧归档按键集合校验）。"""
        from app.detect import VolumeResult, compute_volume, compute_volume_auto
        base, lr = self._baseline(), self._loaded(xc=-0.30)
        v = compute_volume_auto(base, lr, dy=0.0)
        self.assertIsNotNone(v.cross_check)
        self.assertEqual(compute_volume(base, lr, voxel=v.voxel_size, dy=0.0).volume_m3, v.volume_m3)
        self.assertEqual(set(VolumeResult().to_dict(raw=True)), set(v.to_dict(raw=True)))


def _shift_pair(base, lr, sx, sy):
    """空车基线与载货点云一起平移 (sx, sy)：点云、车厢盒、底板平面（d' = d - a*sx - b*sy）。"""
    from dataclasses import replace
    from app.detect import LoadedResult
    c = np.asarray(base.cloud, np.float64).copy()
    c[:, 0] += sx
    c[:, 1] += sy
    x0, x1, y0, y1, z0, z1 = base.box
    a, b, cz, d = base.bed_plane
    nb = replace(base, cloud=c.astype(np.float32), box=(x0 + sx, x1 + sx, y0 + sy, y1 + sy, z0, z1),
                 bed_plane=(a, b, cz, d - a * sx - b * sy))
    lc = np.asarray(lr.cloud, np.float64).copy()
    lc[:, 0] += sx
    lc[:, 1] += sy
    return nb, LoadedResult(cloud=lc.astype(np.float32))


class _EndWallScene(_CarriageScene):
    """在 _CarriageScene 上加前后挡板：后板竖直；前板往里倾（根部在 +L/2，顶部往里 LEAN）。
    载货时挡板在煤面以上那一截（板面 + 板顶）也看得见 —— 正是按格取最高会当成煤顶的那部分。"""

    LEAN = 0.12

    def _front_y(self, h):
        return self.L / 2 - self.LEAN * np.asarray(h) / self.HW

    def _end_walls(self, r, xc, z_lo, n=4000):
        out = []
        for front in (False, True):
            x = r.uniform(xc - self.W / 2, xc + self.W / 2, n)
            h = r.uniform(z_lo, self.HW, n)
            y = self._front_y(h) if front else np.full(n, -self.L / 2)
            out.append(np.column_stack([x, y, self._z(y) + h]))
            x = r.uniform(xc - self.W / 2, xc + self.W / 2, n)
            y = (self._front_y(self.HW) + r.uniform(0, self.WALL, n) if front
                 else -self.L / 2 - r.uniform(0, self.WALL, n))
            out.append(np.column_stack([x, y, self._z(y) + self.HW + r.normal(0, .004, n)]))
        return out

    def _baseline(self, xc=0.0, seed=0):
        from app.detect import EmptyBaseline
        r = np.random.default_rng(seed)
        c = np.concatenate([self._surface(r, xc, 0.0, .004)] + self._walls(r, xc, 0.0)
                           + self._end_walls(r, xc, 0.0)).astype(np.float32)
        lo, hi = c.min(axis=0), c.max(axis=0)
        tan = float(np.tan(np.deg2rad(self.TILT)))
        return EmptyBaseline(-3.7, (float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]),
                                    -3.7, float(hi[2])), c, len(c), "t", bed_plane=(0.0, -tan, 1.0, 3.7))

    def _loaded(self, xc=0.0, seed=1, clip_below=None, sd=0.002, surface=None):
        """surface(r, xc) 可替换煤面（默认平的 H，到前板为止）。"""
        from app.detect import LoadedResult
        r = np.random.default_rng(seed)
        s = self._surface(r, xc, self.H, sd) if surface is None else surface(r, xc)
        s = s[s[:, 1] <= self._front_y(s[:, 2] - self._z(s[:, 1]))]
        c = np.concatenate([s] + self._walls(r, xc, self.H) + self._end_walls(r, xc, self.H))
        if clip_below is not None:
            c = c[c[:, 0] >= clip_below]
        return LoadedResult(cloud=c.astype(np.float32))

    def _truth(self):
        """平煤面 H、前板往里倾时的真体积。"""
        return self.W * (self.L * self.H - self.LEAN * self.H ** 2 / (2 * self.HW))


class TestInnerVolumeDomain(_EndWallScene, unittest.TestCase):
    """内腔积分域（2026-10-02）：积分只在两侧壁内侧面之间、两端底板之间做，贴壁和端板上的载货点不用。

    9-28 两对：旧积分盒宽 2.58 / 2.61 m，内侧面内宽 2.40 / 2.36 m，多出来的厢壁占地多算 8%；
    端板顶往里倾、最高处在积分域里，按格取最高又多算约 3%；网格原点挪几厘米体积变 3.7%~6.2%。"""

    def test_measures_the_inner_faces_and_the_floor_length(self):
        from app.detect import estimate_inner_domain
        d = estimate_inner_domain(self._baseline())
        self.assertEqual("resolved", d.status, d.reason)
        xl, xr = d.edges(np.array([0.0]))
        self.assertAlmostEqual(float(xr[0] - xl[0]), self.W, delta=0.01)
        self.assertAlmostEqual(d.y1 - d.y0, self.L, delta=0.06)
        self.assertIn("后、前端板已圈出", d.reason)

    def test_flat_load_matches_the_truth(self):
        """★ 平煤面：内腔积分接近真值；旧积分盒把壁的占地也算进去，偏大。"""
        from app.detect import compute_volume
        base, lr = self._baseline(), self._loaded()
        v = compute_volume(base, lr, dy=0.0)
        old = compute_volume(base, lr, dy=0.0, inner=False)
        truth = self._truth()
        self.assertIn("内腔积分", v.text)
        self.assertLess(abs(v.volume_m3 - truth) / truth, 0.015, v.text)
        self.assertLess(abs(v.volume_m3 - truth), abs(old.volume_m3 - truth),
                        f"内腔 {v.volume_m3:.2f} / 旧盒 {old.volume_m3:.2f} / 真值 {truth:.2f}")

    def test_end_wall_tops_are_not_taken_as_coal(self):
        """★ 纵向配错 12 cm：载货车后板落进基线的底板格，没有端板掩模时板顶被当成煤顶。"""
        from dataclasses import replace
        from app.detect import compute_volume, estimate_inner_domain
        from app.detect import LoadedResult
        base = self._baseline()
        c = np.asarray(self._loaded().cloud, np.float64).copy()
        c[:, 1] += 0.12
        c[:, 2] += 0.12 * np.tan(np.deg2rad(self.TILT))         # 跟着底板坡度走，整车平移
        lr = LoadedResult(cloud=c.astype(np.float32))
        dom = estimate_inner_domain(base)
        kw = dict(dy=0.0, dx=0.0, yaw=0.0, pose=False)
        masked = compute_volume(base, lr, inner=dom, **kw).volume_m3
        bare = compute_volume(base, lr, inner=replace(dom, ends=None), **kw).volume_m3
        truth = self._truth()
        self.assertGreater(bare - masked, 0.005 * truth, f"无掩模 {bare:.2f} / 有掩模 {masked:.2f}")
        self.assertLess(abs(masked - truth), abs(bare - truth))

    def test_small_registration_errors_barely_move_the_volume(self):
        """横向 ±3 cm、纵向 ±0.25 m 的配准误差：体积变化在 2% 以内（贴壁留空 + 端板掩模膨胀吸收掉）。"""
        from app.detect import compute_volume
        base, lr = self._baseline(), self._loaded()
        kw = dict(yaw=0.0, pose=False)
        v0 = compute_volume(base, lr, dy=0.0, dx=0.0, **kw).volume_m3
        for dx, dy, tol in ((0.03, 0.0, 0.01), (-0.03, 0.0, 0.01), (0.0, 0.25, 0.02), (0.0, -0.25, 0.02)):
            v = compute_volume(base, lr, dy=dy, dx=dx, **kw).volume_m3
            self.assertLess(abs(v - v0) / v0, tol, f"dx {dx:+.2f} dy {dy:+.2f}: {v:.2f} vs {v0:.2f}")

    def test_grid_phase_does_not_move_the_volume(self):
        """网格原点换相位（载货点云里远处多一个杂点，网格就从它开始划）：边界格按面积计权，体积几乎不动。"""
        from app.detect import LoadedResult, compute_volume
        base = self._baseline()
        c = np.asarray(self._loaded().cloud, np.float64)
        kw = dict(dy=0.0, dx=0.0, yaw=0.0, pose=False)
        vols = []
        for sx, sy in ((0.0, 0.0), (0.037, 0.053), (0.071, 0.021)):
            stray = np.array([[-1.93 - sx, -5.6 - sy, self._z(-5.6 - sy)]])
            vols.append(compute_volume(base, LoadedResult(cloud=np.vstack([c, stray]).astype(np.float32)),
                                       **kw).volume_m3)
        self.assertLess((max(vols) - min(vols)) / min(vols), 0.005, " / ".join("%.3f" % v for v in vols))

    def test_wall_and_end_strips_count_as_structural_not_holes(self):
        """贴壁/端板剔掉点的格是结构盲区（壁格），不是空洞 —— 否则 hole_ratio 会结构性涨十几个点。"""
        from app.detect import VolumeResult, compute_volume
        v = compute_volume(self._baseline(), self._loaded(), dy=0.0)
        self.assertLess(v.hole_ratio, 0.01, v.text)
        self.assertGreater(v.wall_ratio, 0.05, v.text)
        self.assertEqual("inner", v.domain.kind)
        self.assertEqual(set(VolumeResult().to_dict(raw=True)), set(v.to_dict(raw=True)))

    def test_falls_back_to_the_old_box_without_side_walls(self):
        """空车基线里没有侧壁内侧面：量不出内腔，退回旧积分盒（= 旧行为）并写明。"""
        from app.detect import EmptyBaseline, compute_volume
        r = np.random.default_rng(0)
        floor = self._surface(r, 0.0, 0.0, .004).astype(np.float32)
        lo, hi = floor.min(axis=0), floor.max(axis=0)
        tan = float(np.tan(np.deg2rad(self.TILT)))
        base = EmptyBaseline(-3.7, (float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]), -3.7, -2.5),
                             floor, len(floor), "t", bed_plane=(0.0, -tan, 1.0, 3.7))
        lr = self._loaded()
        v = compute_volume(base, lr, dy=0.0, dx=0.0, yaw=0.0, pose=False)
        self.assertIn("内腔未用", v.text)
        self.assertEqual(compute_volume(base, lr, dy=0.0, dx=0.0, yaw=0.0, pose=False, inner=False).volume_m3,
                         v.volume_m3)
        self.assertEqual("box", v.domain.kind)


class TestVolumeYawRegistration(_EndWallScene, unittest.TestCase):
    """偏航配准（2026-10-02）：空车、载货各自做航向矫正，两次差半度左右（9-28 两对 +0.49° / +0.71°）。"""

    def _rotated(self, deg, left_only=False):
        from app.detect import LoadedResult
        c = np.asarray(self._loaded().cloud, np.float64).copy()
        t = np.deg2rad(deg)
        sel = c[:, 0] < -self.W / 2 + 0.2 if left_only else np.ones(len(c), bool)
        x, y = c[sel, 0].copy(), c[sel, 1].copy()
        if left_only:
            c[sel, 0] = x + np.tan(t) * y                      # 只有左壁斜：车厢变形，不是整车转
        else:
            c[sel, 0] = x * np.cos(t) - y * np.sin(t)
            c[sel, 1] = x * np.sin(t) + y * np.cos(t)
        return LoadedResult(cloud=c.astype(np.float32))

    def test_recovers_a_known_yaw(self):
        from app.detect import estimate_yaw_registration
        base = self._baseline()
        for deg in (0.6, -1.0):
            d = estimate_yaw_registration(base, self._rotated(deg))
            self.assertEqual("resolved", d.status, d.reason)
            # 载货车绕原点逆时针转 deg：壁的 x 随 y 减小，基线要加的斜率是 -tan(deg)
            self.assertAlmostEqual(d.slope, -np.tan(np.deg2rad(deg)), delta=0.001, msg=d.reason)

    def test_the_two_sides_must_agree(self):
        from app.detect import estimate_yaw_registration
        d = estimate_yaw_registration(self._baseline(), self._rotated(1.0, left_only=True))
        self.assertEqual("unresolved", d.status)
        self.assertIn("两侧壁不一致", d.reason)

    def test_beyond_the_limit_is_not_applied(self):
        from app.detect import estimate_yaw_registration
        d = estimate_yaw_registration(self._baseline(), self._rotated(3.0))
        self.assertEqual("unresolved", d.status)
        self.assertIn("超过上限", d.reason)

    def test_rotated_truck_volume_is_still_right(self):
        """转了 1.5° 的载货车：横向对齐 + 偏航补上之后，内腔积分仍接近真值；yaw=0 关掉时说明里不提偏航。"""
        from app.detect import compute_volume
        base, lr = self._baseline(), self._rotated(1.5)
        v = compute_volume(base, lr, dy=0.0)
        self.assertIn("偏航对齐", v.text)
        self.assertLess(abs(v.volume_m3 - self._truth()) / self._truth(), 0.02, v.text)
        off = compute_volume(base, lr, dy=0.0, yaw=0.0)
        self.assertNotIn("偏航", off.text)


class TestHeightMapMedian(_EndWallScene, unittest.TestCase):
    """逐格取中位（2026-10-02，对照用）：取最高点对粗糙煤面系统性偏高，取中位不偏；默认仍取最高。"""

    def test_cell_statistics(self):
        from app.detect import _height_map
        pts = np.array([[0.05, 0.05, 1.0], [0.06, 0.04, 2.0], [0.02, 0.08, 10.0],
                        [0.15, 0.05, 1.0], [0.12, 0.02, 2.0], [0.18, 0.09, 3.0], [0.11, 0.01, 4.0]])
        zmax, hit = _height_map(pts, 0.0, 0.0, 3, 1, 0.1)
        zmed, hit2 = _height_map(pts, 0.0, 0.0, 3, 1, 0.1, "median")
        np.testing.assert_array_equal(hit, hit2)
        np.testing.assert_allclose(zmax[0, :2], [10.0, 4.0])
        np.testing.assert_allclose(zmed[0, :2], [2.0, 2.5])
        self.assertTrue(np.isnan(zmed[0, 2]))

    def test_median_is_unbiased_on_a_noisy_surface(self):
        """煤面噪声 2 cm：按格取最高偏大约 3%，取中位接近真值。"""
        from app.detect import compute_volume
        base, lr = self._baseline(), self._loaded(sd=0.02)
        kw = dict(dy=0.0, dx=0.0, yaw=0.0, pose=False)
        top = compute_volume(base, lr, **kw).volume_m3
        med = compute_volume(base, lr, surface_stat="median", **kw).volume_m3
        truth = self._truth()
        self.assertGreater(top, truth * 1.015)
        self.assertLess(abs(med - truth), abs(top - truth), f"最高 {top:.2f} / 中位 {med:.2f} / 真值 {truth:.2f}")

    def test_rejects_an_unknown_statistic(self):
        from app.detect import compute_volume
        v = compute_volume(self._baseline(), self._loaded(), dy=0.0, surface_stat="mean")
        self.assertFalse(v.ok)
        self.assertIn("高度图取值方式非法", v.text)


class TestVolumeUncertainty(_EndWallScene, unittest.TestCase):
    """逐车误差估计（2026-10-02）：前后、左右按配准误差扰动重算，高低按姿态估计的标准误差，
    外推分空洞/贴壁/端板检验；只写说明、挂在 uncertainty 上，不改体积、不进 to_dict。"""

    def test_attached_and_written_without_touching_the_volume(self):
        import math
        from app.detect import VolumeResult, compute_volume, compute_volume_auto
        base, lr = self._baseline(), self._loaded()
        v = compute_volume_auto(base, lr, dy=0.0)
        u = v.uncertainty
        self.assertIsNotNone(u)
        parts = [p for p in (u.dy_m3, u.dx_m3, u.pose_m3, u.fill_m3) if math.isfinite(p)]
        self.assertTrue(math.isfinite(u.dy_m3) and math.isfinite(u.dx_m3), v.text)
        self.assertAlmostEqual(u.total_m3, math.sqrt(sum(p * p for p in parts)), places=9)
        self.assertLess(u.total_m3 / v.volume_m3, 0.03, v.text)
        self.assertIn("误差估计", v.text)
        self.assertIn("逐格取中位", v.text)
        self.assertEqual(compute_volume(base, lr, voxel=v.voxel_size, dy=0.0).volume_m3, v.volume_m3)
        self.assertEqual(set(VolumeResult().to_dict(raw=True)), set(v.to_dict(raw=True)))

    def test_lateral_term_is_skipped_when_the_lateral_position_is_unknown(self):
        import math
        from app.detect import compute_volume_auto
        base, lr = self._baseline(xc=-0.45), self._loaded(xc=0.45)    # 相对 0.90 m，横向估不出
        v = compute_volume_auto(base, lr, dy=0.0)
        self.assertTrue(math.isnan(v.uncertainty.dx_m3), v.text)
        self.assertIn("左右 未估", v.text)

    def test_fill_term_follows_the_extrapolated_ends(self):
        """后端缺测给非零误差；普通空洞全覆盖时该分项为0，贴壁和端板仍单独检验。"""
        from app.detect import compute_volume_auto

        def sloped(r, xc, n=60000):
            x = r.uniform(xc - self.W / 2, xc + self.W / 2, n)
            y = r.uniform(-self.L / 2 + 1.0, self.L / 2, n)
            h = np.clip(0.4 + 0.12 * (y + self.L / 2), 0.4, self.H)
            return np.column_stack([x, y, self._z(y) + h + r.normal(0, .002, n)])
        base = self._baseline()
        v = compute_volume_auto(base, self._loaded(surface=sloped), dy=0.0, pose=False)
        self.assertGreater(v.hole_ratio, 0.01, v.text)
        self.assertGreater(v.uncertainty.fill_m3, 0.0, v.text)
        full = compute_volume_auto(base, self._loaded(), dy=0.0, pose=False)
        parts = {part.kind: part for part in full.domain.fill_parts}
        self.assertEqual(0.0, parts["hole"].error_m3)
        self.assertTrue(np.isfinite(parts["side"].error_m3))
        self.assertTrue(np.isfinite(parts["end"].error_m3))


class TestCrossCheckRowColumnEffects(_EndWallScene, unittest.TestCase):
    """脊形的横向效应随纵向改变，不能因为总体积碰巧接近就声明加性校核可靠。
    2026-10-03 增加联合拟合与留出检验后，此例应拒绝校核；主积分仍须贴近真值。"""

    def test_ridged_load_with_sparse_ends(self):
        from app.detect import compute_volume, cross_check_volume

        def height(x, y, xc=0.0):
            ridge = 0.6 * (1 - (2 * (x - xc) / self.W) ** 2)
            ends = np.clip((self.L / 2 - np.abs(y)) / 2.0, 0.3, 1.0)
            return 0.5 + ridge * ends

        def ridged(r, xc, n=80000):
            x = r.uniform(xc - self.W / 2, xc + self.W / 2, n)
            y = r.uniform(-self.L / 2, self.L / 2, n)
            keep = (np.abs(y) < self.L / 2 - 1.5) | (r.uniform(0, 1, n) < 0.25)     # 两端 1.5 m 只剩 1/4 的点
            x, y = x[keep], y[keep]
            return np.column_stack([x, y, self._z(y) + height(x, y, xc) + r.normal(0, .002, len(x))])
        g = np.linspace(-self.W / 2, self.W / 2, 241)
        yy = np.linspace(-self.L / 2, self.L / 2, 1001)
        truth = float(np.trapz(np.trapz(height(g[None, :], yy[:, None]), g, axis=1), yy))
        base, lr = self._baseline(), self._loaded(surface=ridged)
        cc = cross_check_volume(base, lr)
        self.assertEqual("unavailable", cc.status, cc.reason)
        self.assertIn("留出检查", cc.reason)
        self.assertGreater(cc.diagnostics["holdout_worst_relative"], cc.diagnostics["holdout_relative_limit"])
        v = compute_volume(base, lr, dy=0.0)
        self.assertLess(abs(v.volume_m3 - truth) / truth, 0.03, f"{v.volume_m3:.2f} vs 真值 {truth:.2f}")


class TestEnterFrameFill(_EndWallScene, unittest.TestCase):
    """进场帧补两端（2026-10-02，可选、默认不用）：相邻帧链式配准 + 逐帧扣高度，只补停稳帧没测到的格。"""

    SHIFTS = (2.0, 1.6, 1.2, 0.8, 0.4, 0.05)      # 时间顺序：最早（离停车点最远）在前

    def _height(self, x, y):
        """前 3 m 往前板方向从 H 降到 0.4 m —— 按邻居补会把前端补高；再叠一层块煤尺度的起伏当纹理。
        纹理要够碎：9-28 两车相邻帧配准的尖锐度 2.4~3.6，平缓起伏的合成煤面只有 2 左右，
        会被 ENTER_STEP_SHARP 当成"看不出位移"拦掉。"""
        base = np.where(y < 2.0, self.H, self.H - (self.H - 0.4) * (y - 2.0) / 3.0)
        return (base + 0.03 * np.sin(2 * np.pi * y / 0.45 + 1.3 * np.sin(2 * np.pi * x / 0.8))
                + 0.02 * np.sin(2 * np.pi * x / 0.37 + 2 * np.pi * y / 0.61))

    def _surface_points(self, r, n=60000):
        x = r.uniform(-self.W / 2, self.W / 2, n)
        y = r.uniform(-self.L / 2, self.L / 2, n)
        return np.column_stack([x, y, self._z(y) + self._height(x, y) + r.normal(0, .002, n)])

    def _truth(self):
        x = np.linspace(-self.W / 2, self.W / 2, 241)
        y = np.linspace(-self.L / 2, self.L / 2, 4001)
        return float(np.trapz(np.trapz(self._height(x[None, :], y[:, None]), x, axis=1), y))

    def _truck(self, seed, shift=0.0, heave=0.0):
        """整辆载货车往后退 shift（y 减 shift）、整体低 heave 的一帧（雷达坐标）。"""
        c = np.asarray(self._loaded(seed=seed, surface=lambda rr, xc: self._surface_points(rr)).cloud,
                       np.float64).copy()
        c[:, 1] -= shift
        c[:, 2] += heave
        return c

    def _stationary(self):
        """停稳帧：前端 y > 3 的煤面没扫到（掠射）。"""
        from app.detect import LoadedResult
        c = self._truck(1)
        coal = np.abs(c[:, 0]) < self.W / 2 - 0.01
        return LoadedResult(cloud=c[~coal | (c[:, 1] <= 3.0)].astype(np.float32))

    def _frames(self):
        return [self._truck(20 + i, s, heave=-0.03 * s) for i, s in enumerate(self.SHIFTS)]

    def test_recovers_the_shifts(self):
        from app.detect import register_enter_frames
        reg = register_enter_frames(self._stationary(), self._frames())
        self.assertEqual("resolved", reg.status, reg.reason)
        self.assertEqual(len(self.SHIFTS), reg.used, reg.reason)
        np.testing.assert_allclose(reg.shifts, self.SHIFTS, atol=0.03)

    def test_fills_the_unseen_front_end(self):
        """★ 停稳帧看不到前 2 m：按邻居补偏大；补上进场帧（扣过高度）后接近真值。"""
        from app.detect import compute_volume, register_enter_frames
        base, stat = self._baseline(), self._stationary()
        reg = register_enter_frames(stat, self._frames())
        truth = self._truth()
        kw = dict(dy=0.0, dx=0.0, yaw=0.0, pose=False)
        alone = compute_volume(base, stat, **kw)
        filled = compute_volume(base, stat, enter_extra=reg.clouds, **kw)
        self.assertGreater(alone.volume_m3, truth * 1.02, alone.text)
        self.assertLess(abs(filled.volume_m3 - truth) / truth, 0.015, filled.text)
        self.assertIn("进场帧补了", filled.text)
        self.assertLess(filled.hole_ratio, alone.hole_ratio)

    def test_a_broken_link_drops_the_earlier_frames(self):
        """链中间混进一帧对不上的：从它往前（更早）的帧一律不用，后面的照用。"""
        from app.detect import register_enter_frames
        frames = self._frames()
        r = np.random.default_rng(5)
        frames[2] = np.column_stack([r.uniform(-1.2, 1.2, 30000), r.uniform(-6, 4, 30000),
                                     r.uniform(-4, -2, 30000)])
        reg = register_enter_frames(self._stationary(), frames)
        self.assertEqual("resolved", reg.status, reg.reason)
        self.assertEqual(3, reg.used, reg.reason)
        np.testing.assert_allclose(reg.shifts, self.SHIFTS[3:], atol=0.03)

    def test_never_touches_measured_cells_and_is_off_by_default(self):
        """停稳帧全覆盖时进场帧（哪怕高度全错）一格都不补；不传就与原来逐位相同。"""
        from app.detect import compute_volume
        base, lr = self._baseline(), self._loaded()
        kw = dict(dy=0.0, dx=0.0, yaw=0.0, pose=False)
        wrong = np.asarray(lr.cloud, np.float64).copy()
        wrong[:, 2] += 0.5
        v0 = compute_volume(base, lr, **kw).volume_m3
        self.assertEqual(v0, compute_volume(base, lr, enter_extra=[wrong], **kw).volume_m3)
        self.assertEqual(v0, compute_volume(base, lr, enter_extra=None, **kw).volume_m3)

    def test_unrelated_frames_are_rejected(self):
        from app.detect import register_enter_frames
        r = np.random.default_rng(5)
        noise = np.column_stack([r.uniform(-1.2, 1.2, 20000), r.uniform(-5, 5, 20000), r.uniform(-4, -2, 20000)])
        reg = register_enter_frames(self._stationary(), [noise, noise + [0, .3, 0]])
        self.assertEqual("unresolved", reg.status)

    def test_not_used_on_the_old_box(self):
        from app.detect import compute_volume
        base, lr = self._baseline(), self._loaded()
        v = compute_volume(base, lr, dy=0.0, dx=0.0, yaw=0.0, pose=False, inner=False,
                           enter_extra=[np.asarray(lr.cloud, np.float64)])
        self.assertIn("进场帧未用", v.text)
