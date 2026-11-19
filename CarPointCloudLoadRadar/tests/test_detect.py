# -*- coding: utf-8 -*-
"""
判定链测试 —— 本套件里最重要的一组。

最重要的一条：test_history_regression
    把 data/loaded 里每一份历史样本重新跑一遍本项目，
    结果必须与原版当年写下的 .reason.txt **逐字一致**。
    这是"移植没有走样"的唯一硬证据，也是每次改阈值/顺序后的必跑项。

其余用例按 evaluate_empty 的 11 步逐条构造最小输入，钉住每一处的行为，
这样万一哪天某一步的条件写反了，能精确指出是哪一步。
"""
from __future__ import annotations

import io
import os
import unittest

import numpy as np

from tests import _helpers as H
from app.settings import BASELINE_SUBDIR
from app.geometry import Plane
from app.pointcloud import Frame, load_xyz, apply_roi, load_reason
from app.detect import (EmptyResult, PLANE_ORDER, detect_empty, evaluate_empty,
                        reason_detail, gate_report,
                        fit_floor, check_dims, StabilityDetector, StabilityState,
                        compute_volume, carriage_box, carriage_confidence,
                        carriage_agreement, CARRIAGE_JITTER,
                        R_INSUFFICIENT_POINTS, R_EMPTY_FAILED,
                        R_FLOOR_LOW_RATIO, R_FLOOR_TOO_HIGH, R_FLOOR_FEW_INLIERS,
                        R_FLOOR_NOT_LEVEL, R_PASS_EMPTY, R_NO_FLOOR)


def _res(cloud, floor: Plane, dims=(2.4, 12.0, 2.6)) -> EmptyResult:
    planes = [Plane() for _ in range(6)]
    planes[PLANE_ORDER.index("floor")] = floor
    return EmptyResult(planes=planes, dims=dims, cloud=cloud, yaw=0.0, ok=True)


def _flat_cloud(n=40000, z=-5.0, jitter=0.004, span_x=1.2, span_y=6.0, seed=0):
    """一片贴合某水平面的点云（内点占比≈1，span 足够大）。"""
    rng = np.random.default_rng(seed)
    x = rng.uniform(-span_x, span_x, n)
    y = rng.uniform(-span_y, span_y, n)
    zz = z + rng.normal(0, jitter, n)
    return np.stack([x, y, zz], 1).astype(np.float32)


class TestHistoryRegression(unittest.TestCase):
    """★ 与原版 reason.txt 比对 —— 但**分两档**（2026-09-16 起）。

    10 份基线的"标准答案"是**原厂的输出**，不是"车到底空不空"。
    查实其中 4 份（原版报「底面过高，疑似载物」的那 4 份）**本身就是空车**：
    它们的底面平面落在**底板**高度 1.51~1.64 m，而 2026-09-15 两辆现场确认的
    空车是 1.52~1.55 m；报「底面内点占比过低」的 6 份则落在**货面** 3.30~3.55 m，
    离壁顶只剩 0.4 m。

    所以：
      · `z_above_mode="origin"`（原厂度量）-> 10 份逐字一致，**这是不许破的底线**；
      · `z_above_mode="plane"`（倾斜无关）-> 那 4 份变成「空车」，
        属于**故意的分歧**，由 `cli/verify.py::EXPECTED_DIVERGENCE_PLANE` 白名单化，
        且只放行一种跃迁（底面过高 -> 空车）。

    两档都要绿，所以下面逐条把两种模式都跑一遍。详见 docs/12 §4.1。
    """

    @staticmethod
    def _modes(dev):
        """返回 [(模式名, dev 副本), ...] —— 两档都要验。"""
        import copy as _copy
        out = []
        for m in ("origin", "plane"):
            d = _copy.deepcopy(dev)
            d.gate.z_above_mode = m
            out.append((m, d))
        return out

    @staticmethod
    def _want(mode, expected):
        """这份样本在该模式下**应当**给出什么理由。"""
        if mode == "plane" and expected == R_FLOOR_TOO_HIGH:
            return R_PASS_EMPTY
        return expected

    def test_all_history_samples_match(self):
        _, _, calib, dev, _ = H.ctx()
        samples = H.history_samples()
        if not samples:
            self.skipTest("data/loaded 下没有历史样本")
        for mode, d in self._modes(dev):
            bad = []
            for path, expected in samples:
                fr = apply_roi(load_xyz(path), calib.roi)
                res = detect_empty(fr, calib)
                cloud = res.cloud if res is not None else fr.xyz
                ok, reason, dg = evaluate_empty(res, fr.n, cloud, d)
                want = self._want(mode, expected)
                if reason != want:
                    bad.append(f"{path}\n    应当={want!r} 实际={reason!r} "
                               f"({dg.inliers} 内点 / {dg.ratio:.4f})")
            self.assertEqual(bad, [],
                             f"[mode={mode}] {len(bad)}/{len(samples)} 条对不上：\n  "
                             + "\n  ".join(bad))

    def test_history_verdict_ok_flag_consistent(self):
        """ok 标志必须与理由一致：理由='空车' => ok=True，其它理由 => ok=False。"""
        _, _, calib, dev, _ = H.ctx()
        for mode, d in self._modes(dev):
            for path, expected in H.history_samples():
                fr = apply_roi(load_xyz(path), calib.roi)
                res = detect_empty(fr, calib)
                ok, reason, _ = evaluate_empty(res, fr.n,
                                               res.cloud if res else fr.xyz, d)
                self.assertEqual(ok, (reason == R_PASS_EMPTY),
                                 f"[{mode}] {path}: ok={ok} 但理由={reason!r}")
                self.assertEqual(reason == R_PASS_EMPTY,
                                 self._want(mode, expected) == R_PASS_EMPTY,
                                 f"[{mode}] {path}: 空车与否与应当值矛盾")

    def test_origin_mode_reproduces_the_factory_verdicts(self):
        """★ 底线：原厂度量下必须**逐字**还原 10 份。

        换句话说 —— 新度量是**可选**的，关掉它就得完全回到原厂行为。
        这一条如果红了，说明改度量的时候顺手动了别的东西。
        """
        import copy as _copy
        _, _, calib, dev, _ = H.ctx()
        d = _copy.deepcopy(dev)
        d.gate.z_above_mode = "origin"
        n = 0
        for path, expected in H.history_samples():
            fr = apply_roi(load_xyz(path), calib.roi)
            res = detect_empty(fr, calib)
            _ok, reason, _dg = evaluate_empty(res, fr.n, res.cloud, d)
            self.assertEqual(reason, expected, f"{path} 与原厂不一致")
            n += 1
        self.assertGreaterEqual(n, 10, "基线样本少了")

    def test_plane_mode_only_flips_the_four_known_ones(self):
        """★ 换度量**只许**影响那 4 份，6 份满载一个字都不许动。

        6 份满载在第 7 步「底面内点占比过低」就被拦下，根本走不到第 10 步，
        所以换第 10 步的度量对它们必须毫无影响。这一条防的是
        "改度量顺手改坏了别的步骤"。
        """
        import copy as _copy
        _, _, calib, dev, _ = H.ctx()
        do = _copy.deepcopy(dev)
        do.gate.z_above_mode = "origin"
        dp = _copy.deepcopy(dev)
        dp.gate.z_above_mode = "plane"
        flipped, kept = [], []
        for path, expected in H.history_samples():
            fr = apply_roi(load_xyz(path), calib.roi)
            res = detect_empty(fr, calib)
            r0 = evaluate_empty(res, fr.n, res.cloud, do)[1]
            r1 = evaluate_empty(res, fr.n, res.cloud, dp)[1]
            (flipped if r0 != r1 else kept).append((path, r0, r1))
        self.assertEqual(len(flipped), 4,
                         f"换度量影响了 {len(flipped)} 份，应当正好 4 份："
                         + "; ".join(f"{p}: {a}->{b}" for p, a, b in flipped))
        for _p, r0, r1 in flipped:
            self.assertEqual(r0, R_FLOOR_TOO_HIGH, "只许从「底面过高」翻")
            self.assertEqual(r1, R_PASS_EMPTY, "只许翻成「空车」")
        for _p, r0, _r1 in kept:
            self.assertEqual(r0, R_FLOOR_LOW_RATIO,
                             "没翻的那 6 份应当都是在第 7 步被拦的满载样本")

    def test_verify_whitelist_matches_the_flipped_set(self):
        """★ `cli/verify.py` 的白名单必须与"实际会翻的那几份"完全一致。

        白名单是手写的，一旦基线增删或度量再改，它就会与现实脱节 ——
        脱节的方向若是"白名单更宽"，闸门就悄悄变松了。这条钉住它。
        """
        import copy as _copy
        import os as _os
        import sys as _sys
        _b = _os.path.join(_os.path.dirname(_os.path.dirname(
            _os.path.abspath(__file__))), "cli")
        if _b not in _sys.path:
            _sys.path.insert(0, _b)
        from verify import EXPECTED_DIVERGENCE_PLANE, EXPECTED_FROM, EXPECTED_TO
        self.assertEqual(EXPECTED_FROM, R_FLOOR_TOO_HIGH)
        self.assertEqual(EXPECTED_TO, R_PASS_EMPTY)

        _, _, calib, dev, _ = H.ctx()
        root = _os.path.join(_os.path.dirname(_b), *BASELINE_SUBDIR)
        do = _copy.deepcopy(dev)
        do.gate.z_above_mode = "origin"
        dp = _copy.deepcopy(dev)
        dp.gate.z_above_mode = "plane"
        flipped = set()
        for path, _expected in H.history_samples():
            fr = apply_roi(load_xyz(path), calib.roi)
            res = detect_empty(fr, calib)
            if (evaluate_empty(res, fr.n, res.cloud, do)[1]
                    != evaluate_empty(res, fr.n, res.cloud, dp)[1]):
                flipped.add(_os.path.relpath(path, root).replace("\\", "/"))
        self.assertEqual(flipped, set(EXPECTED_DIVERGENCE_PLANE),
                         "verify.py 的白名单与实际会翻的样本对不上了")


class TestVerdictSteps(unittest.TestCase):
    """按 11 步逐条构造最小输入，钉住每一步。"""

    def setUp(self):
        _, _, self.calib, self.dev, _ = H.ctx()

    def test_step1_insufficient_points(self):
        cloud = _flat_cloud(n=100)                    # 远少于 min_save_points
        ok, reason, dg = evaluate_empty(None, len(cloud), cloud, self.dev)
        self.assertFalse(ok)
        self.assertEqual(reason, R_INSUFFICIENT_POINTS)

    def test_step2_no_result(self):
        cloud = _flat_cloud(n=self.dev.min_save_points + 10)
        ok, reason, _ = evaluate_empty(None, len(cloud), cloud, self.dev)
        self.assertFalse(ok)
        self.assertEqual(reason, R_EMPTY_FAILED)

    def test_step3_no_valid_floor(self):
        n = self.dev.min_save_points + 10
        cloud = _flat_cloud(n=n)
        fl = Plane(0, 0, 1, 5.0, valid=False)         # valid=False
        ok, reason, _ = evaluate_empty(_res(cloud, fl), len(cloud), cloud, self.dev)
        self.assertEqual(reason, R_NO_FLOOR)

    def test_step4_floor_not_level(self):
        n = self.dev.min_save_points + 10
        cloud = _flat_cloud(n=n)
        # |c| = 0.5 < 0.99 → 底面不够水平
        fl = Plane(0.866, 0, 0.5, -2.5, valid=True)
        ok, reason, dg = evaluate_empty(_res(cloud, fl), len(cloud), cloud, self.dev)
        self.assertEqual(reason, R_FLOOR_NOT_LEVEL)
        self.assertLess(dg.normal_z, self.dev.gate.normal_z_min)

    def test_step6_few_inliers(self):
        n = self.dev.min_save_points + 10
        # 点云铺在 z=-1，但底面拟在 z=0 → 距离 1m，几乎无内点
        cloud = _flat_cloud(n=n, z=-1.0)
        fl = Plane(0, 0, 1, 0.0, valid=True)
        ok, reason, dg = evaluate_empty(_res(cloud, fl), len(cloud), cloud, self.dev)
        self.assertIn(reason, (R_FLOOR_FEW_INLIERS, R_FLOOR_LOW_RATIO))
        self.assertLess(dg.inliers, self.dev.gate.min_inliers)

    def test_step7_low_inlier_ratio(self):
        """内点数够（≥600）但占比不足（<15%）→ 第 7 步。"""
        rng = np.random.default_rng(2)
        # 只有 1000 个点在底面上（≥ min_inliers=600，确保走到第 7 步）
        on_plane = _flat_cloud(n=1000, z=0.0, jitter=0.001, seed=1)
        # 十万个点远离底面 → 把占比压到 ~1%
        far = np.stack([rng.uniform(-1.2, 1.2, 100000),
                        rng.uniform(-6, 6, 100000),
                        rng.uniform(1.0, 2.0, 100000)], 1).astype(np.float32)
        cloud = np.concatenate([on_plane, far], 0)
        fl = Plane(0, 0, 1, 0.0, valid=True)          # z = 0
        ok, reason, dg = evaluate_empty(_res(cloud, fl), len(cloud), cloud, self.dev)
        self.assertGreaterEqual(dg.inliers, self.dev.gate.min_inliers)
        self.assertLess(dg.ratio, self.dev.gate.min_inlier_ratio)
        self.assertEqual(reason, R_FLOOR_LOW_RATIO)

    def test_step10_floor_too_high(self):
        """底面明显高于最低点 → 「底面过高，疑似载物」（真实载货样本的路径）。"""
        n = 60000
        rng = np.random.default_rng(3)
        # 大部分点在货堆顶面 z=-3.8；少量点落到 z=-5.3 的真实底板
        top = _flat_cloud(n=n, z=-3.8, jitter=0.002, seed=3)
        deep = np.stack([rng.uniform(-1.2, 1.2, n // 20),
                         rng.uniform(-6, 6, n // 20),
                         np.full(n // 20, -5.3)], 1).astype(np.float32)
        cloud = np.concatenate([top, deep], 0)
        fl = Plane(0, 0, 1, 3.8, valid=True)          # z = -3.8
        fl.inliers = int((fl.abs_distance(cloud) <= self.dev.gate.distance_m).sum())
        ok, reason, dg = evaluate_empty(_res(cloud, fl), len(cloud), cloud, self.dev)
        self.assertFalse(ok)
        self.assertEqual(reason, R_FLOOR_TOO_HIGH)
        self.assertGreater(dg.z_above, self.dev.gate.z_above_min_max_m)

    def test_step11_pass_empty(self):
        cloud = _flat_cloud(n=60000, z=-5.0, jitter=0.002, span_x=1.3,
                            span_y=6.5, seed=4)
        fl = Plane(0, 0, 1, 5.0, valid=True)          # z = -5.0
        fl.inliers = int((fl.abs_distance(cloud) <= self.dev.gate.distance_m).sum())
        ok, reason, dg = evaluate_empty(_res(cloud, fl), len(cloud), cloud, self.dev)
        self.assertTrue(ok, f"应当判空车，实际 {reason}（内点 {dg.inliers} "
                            f"占比 {dg.ratio:.4f}）")
        self.assertEqual(reason, R_PASS_EMPTY)

    def test_gate_override_via_min_save_points(self):
        """dev 也可以只传 GateCfg —— 此时最少点数走 min_save_points 参数。"""
        cloud = _flat_cloud(n=1000, z=-5.0, jitter=0.002, seed=5)
        fl = Plane(0, 0, 1, 5.0, valid=True)
        ok, reason, _ = evaluate_empty(_res(cloud, fl), 1000, cloud,
                                       self.dev.gate, min_save_points=500)
        self.assertEqual(reason, R_PASS_EMPTY)
        assert ok


class TestFloorHeightGuard(unittest.TestCase):
    """第 10 步的平面离地上限：刮平的满载煤面不能被判成空车（2026-09-24 现场）。

    现场 CH2 那辆满载车：平面离地 3.03 m、内点占比 0.205、z_above 0.047 ——
    只因内点 X 跨度差 4.8 mm 被第 8 步拦下。这里构造"跨度够了"的同一种情形：
    平面下面没有东西（z_above 量不出来），能拦它的只有平面本身的离地高度。
    """

    GROUND = -5.23

    def setUp(self):
        import copy
        _, _, _, dev, _ = H.ctx()
        self.dev = copy.deepcopy(dev)
        self.dev.gate.z_above_mode = "plane"
        self.dev.gate.floor_height_max_m = 2.3

    def _judge(self, height, ground_z=GROUND, **gate):
        for key, value in gate.items():
            setattr(self.dev.gate, key, value)
        z = self.GROUND + height
        cloud = _flat_cloud(n=60000, z=z, jitter=0.002, span_x=1.3, span_y=6.5, seed=4)
        fl = Plane(0, 0, 1, -z, valid=True)
        fl.inliers = int((fl.abs_distance(cloud) <= self.dev.gate.distance_m).sum())
        return evaluate_empty(_res(cloud, fl), len(cloud), cloud, self.dev, ground_z=ground_z)

    def test_flat_cargo_top_is_loaded(self):
        ok, reason, dg = self._judge(3.03)
        self.assertFalse(ok)
        self.assertEqual(R_FLOOR_TOO_HIGH, reason)
        self.assertEqual(10, dg.step)
        self.assertTrue(dg.floor_height_blocked)
        self.assertAlmostEqual(3.03, dg.floor_height, places=2)
        self.assertLessEqual(dg.z_above, self.dev.gate.z_above_min_max_m,
                             "这正是 z_above 量不出来的情形 —— 平面下面没有东西")

    def test_without_the_guard_the_same_cloud_passes_as_empty(self):
        """钉住漏洞本身：关掉这道闸（0），同一片刮平的货面会被判成空车。"""
        ok, reason, _dg = self._judge(3.03, floor_height_max_m=0.0)
        self.assertTrue(ok)
        self.assertEqual(R_PASS_EMPTY, reason)

    def test_empty_bed_heights_still_pass(self):
        for height in (1.51, 1.64, 1.9):
            ok, reason, dg = self._judge(height)
            self.assertEqual(R_PASS_EMPTY, reason, "底板离地 %.2f m 的空车被拦了" % height)
            self.assertFalse(dg.floor_height_blocked)
            self.assertAlmostEqual(height, dg.floor_height, places=2)

    def test_needs_ground_z_and_plane_mode(self):
        """拿不到 ground_z、或 origin 档（原厂行为）时这道闸不参与，行为逐位不变。"""
        self.assertEqual(R_PASS_EMPTY, self._judge(3.03, ground_z=None)[1])
        self.assertEqual(R_PASS_EMPTY, self._judge(3.03, z_above_mode="origin")[1])

    def test_gate_report_shows_the_row_that_blocked(self):
        _ok, _reason, dg = self._judge(3.03)
        rows = gate_report(dg, self.dev)
        self.assertEqual(11, len(rows), "11 步一步都不能少")
        blocked = [r for r in rows if r["state"] == "block"]
        self.assertEqual(1, len(blocked))
        self.assertEqual("底面离地高度", blocked[0]["name"])
        self.assertEqual(10, blocked[0]["step"])
        self.assertAlmostEqual(3.03, blocked[0]["value"], places=2)
        self.assertLess(blocked[0]["margin"], 0)
        text = reason_detail(dg, self.dev)
        self.assertIn("底面离地高度", text)
        self.assertIn("刮平", text)

    def test_shipped_devices_config_enables_it(self):
        from app.settings import load_devices
        dev = load_devices(os.path.join(H.ROOT, "config", "devices.json"))
        self.assertEqual("plane", dev.gate.z_above_mode)
        self.assertAlmostEqual(2.3, dev.gate.floor_height_max_m)


class TestOfflineToolsUseGroundZ(unittest.TestCase):
    """离线工具调判定链必须传 ground_z，与服务、cli/verify.py 同口径（2026-09-24）。

    第 10 步的兜底和"平面离地上限"都只在拿得到 ground_z 时生效。原来
    `cli/replay.py` / `cli/inspect_cloud.py` 没传 —— 同一帧在服务里判"底面过高"，
    拿这两个工具复核却可能判"空车"，现场会以为服务错了。
    """

    def _sample(self):
        import glob
        files = sorted(glob.glob(os.path.join(H.ROOT, *BASELINE_SUBDIR, "**", "*.xyz"), recursive=True))
        if not files:
            self.skipTest("没有回归基线样本")
        return files[0]

    def _spy(self, module):
        from unittest.mock import patch
        seen = []
        real = module.evaluate_empty

        def spy(*args, **kwargs):
            seen.append(kwargs.get("ground_z"))
            return real(*args, **kwargs)
        return patch.object(module, "evaluate_empty", side_effect=spy), seen

    def test_replay_passes_ground_z(self):
        from cli import replay
        _, _, calib, dev, _ = H.ctx()
        patcher, seen = self._spy(replay)
        with patcher:
            replay.detect_one(self._sample(), calib, dev, use_roi=True)
        self.assertEqual([calib.detection.ground_z], seen)

    def test_inspect_cloud_passes_ground_z(self):
        import contextlib
        import io
        import sys
        cli_dir = os.path.join(H.ROOT, "cli")
        if cli_dir not in sys.path:                 # inspect_cloud 按脚本方式 `import _common`
            sys.path.insert(0, cli_dir)
        from cli import inspect_cloud
        from app.settings import load_calib, load_settings
        patcher, seen = self._spy(inspect_cloud)
        with patcher, contextlib.redirect_stdout(io.StringIO()):
            inspect_cloud.main([self._sample(), "-q"])
        st = load_settings("")
        ch = (st.enabled() or st.channels)[0]
        self.assertEqual([load_calib(ch.abs_config_path).detection.ground_z], seen)


class TestFloorFit(unittest.TestCase):
    """底面拟合：必须用「最低处种子 + PCA」，不能靠 RANSAC 选内点最多的面。"""

    def setUp(self):
        _, _, self.calib, self.dev, _ = H.ctx()
        # fit_floor(xyz, cfg) 里的 cfg 就是 CalibCfg（它自带 .floor 子配置）
        self.cfg = self.calib

    def test_recovers_low_floor(self):
        cloud = _flat_cloud(n=20000, z=-5.0, jitter=0.001, seed=6)
        p = fit_floor(cloud, self.cfg)
        self.assertIsNotNone(p)
        self.assertTrue(p.valid)
        self.assertAlmostEqual(abs(p.intercept(2)), 5.0, delta=0.02)

    def test_ignores_high_dense_surface(self):
        """
        回归那条踩过的坑：车厢点云很"薄"时，RANSAC 会挑出内点更多的倾斜面，
        导致 |c|<0.99 → 底面 invalid → 判定链在第 2 步就报「空车检测失败」，
        而真值是第 7 步的「底面内点占比过低」。
        正确的种子策略应始终锁到最低处的面。
        """
        rng = np.random.default_rng(7)
        n = 40000
        floor = _flat_cloud(n=n // 4, z=-5.0, jitter=0.002, seed=7)
        # 上方一大片倾斜面（内点会更多，是干扰项）
        x = rng.uniform(-1.2, 1.2, n)
        y = rng.uniform(-6, 6, n)
        zz = -3.0 + 0.3 * x
        noise = np.stack([x, y, zz], 1).astype(np.float32)
        cloud = np.concatenate([floor, noise], 0)
        p = fit_floor(cloud, self.cfg)
        self.assertIsNotNone(p)
        # 应当锁到 z≈-5 的底板，而不是 z≈-3 的倾斜面
        self.assertAlmostEqual(abs(p.intercept(2)), 5.0, delta=0.25)

    def test_too_few_points_returns_none(self):
        self.assertIsNone(fit_floor(np.zeros((2, 3), np.float32), self.cfg))


class TestCheckDims(unittest.TestCase):

    def setUp(self):
        _, _, _, self.dev, _ = H.ctx()

    def test_pass(self):
        cloud = _flat_cloud(n=1000)
        ok, msg = check_dims(_res(cloud, Plane(), dims=(2.4, 12.0, 2.6)), self.dev)
        self.assertTrue(ok)
        self.assertEqual(msg, "")

    def test_fail_lists_reasons(self):
        cloud = _flat_cloud(n=1000)
        ok, msg = check_dims(_res(cloud, Plane(), dims=(1.0, 3.0, 0.5)), self.dev)
        self.assertFalse(ok)
        self.assertIn("尺寸不达标", msg)


class TestStabilityDetector(unittest.TestCase):
    """停稳检测：dev = ‖质心差‖，已用受控实验验证到 1e-7。"""

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    def test_translation_deviation_equals_norm(self):
        rng = np.random.default_rng(8)
        base = rng.normal(0, 1, (5000, 3)).astype(np.float32)
        d = StabilityDetector(stability_delta=0.05, vehicle_threshold=100)
        self.clock.detect(d, Frame(base))
        shift = np.array([0.3, -0.2, 0.1], np.float32)
        r = self.clock.detect(d, Frame(base + shift))
        self.assertAlmostEqual(r.deviation, float(np.linalg.norm(shift)), places=6)

    def test_identical_new_frames_need_continuous_time(self):
        rng = np.random.default_rng(9)
        base = rng.normal(0, 1, (5000, 3)).astype(np.float32)
        base[:, 1] *= 4  # 有可见长轴的车辆；等向点团不能证明旋转停止。
        d = StabilityDetector(stability_delta=0.05, vehicle_threshold=100)
        self.clock.detect(d, Frame(base))
        r = self.clock.detect(d, Frame(base))
        self.assertLess(r.deviation, 1e-9)
        self.assertFalse(r.stable, "两次采集时间不足，不能证明停稳")
        r = self.clock.hold(d, Frame(base))
        self.assertTrue(r.stable)
        self.assertEqual(r.state, StabilityState.STABLE)

    def test_moving_frame_is_not_stable(self):
        rng = np.random.default_rng(10)
        base = rng.normal(0, 1, (5000, 3)).astype(np.float32)
        d = StabilityDetector(stability_delta=0.05, vehicle_threshold=100)
        self.clock.detect(d, Frame(base))
        r = self.clock.detect(d, Frame(base + np.array([1.0, 0, 0], np.float32)))
        self.assertFalse(r.stable)
        self.assertEqual(r.state, StabilityState.BASELINE)

    def test_too_few_points_is_idle(self):
        d = StabilityDetector(stability_delta=0.05, vehicle_threshold=3000)
        r = self.clock.detect(d, Frame(np.zeros((10, 3), np.float32)))
        self.assertEqual(r.state, StabilityState.IDLE)

    def test_reset_clears_reference(self):
        rng = np.random.default_rng(12)
        base = rng.normal(0, 1, (5000, 3)).astype(np.float32)
        d = StabilityDetector(stability_delta=0.05, vehicle_threshold=100)
        self.clock.detect(d, Frame(base))
        d.reset()
        # 重置后第一帧只建立参考，不应立刻判稳
        r = self.clock.detect(d, Frame(base + 5.0))
        self.assertEqual(r.state, StabilityState.BASELINE)


class TestVehicleThresholdBrackets(unittest.TestCase):
    """`vehicle_threshold` 必须夹在「空车道」和「空车厢」之间。

    这条判据数的是"高于地面 0.3 m 的点"（`vehicle_points()`），
    是**本项目自己的度量**，跟原厂那个数总点数的 `vehicle_threshold=3000`
    不是一回事 —— 所以阈值只能拿这个量自己的数据定，不能照抄原厂。

    两侧都会静默失效，方向相反，所以两侧都要钉：

    · 定低了 → 空车道被判成有车 → 地面不动 → 判"已停稳" → 会话永不结束
      → **之后真来的车全部被静默跳过**（2026-09-13 事故，见 docs/03 §2.6）；
    · 定高了 → **空车厢根本不触发会话** → 一份记录都不留，看板上什么都不显示。
      本系统要判的正是空车，这一侧更容易被忽略：2026-09-14 按"空车道 vs 载货车"
      取中点算出 25 000，两端看着都有一倍余量，实际会把空车全漏掉。

    空车道那一侧的数字是 2026-09-14 现场实测（三路 9 878 / 10 320 / 2 892，
    帧间波动 < 1%）；空车厢这一侧用仿真算，因为 10 份历史样本里**没有空车**
    （CLAUDE.md「已知限制」：无"通过"正样本）。
    """

    # 2026-09-14 现场实测的空车道车点数上界（CH4，车道里还有障碍物）。
    # 同日早些时候量到的 10 374 是**朝向错的**那一版：yaw_offset_deg 当时是
    # -93°，ROI 的长边横切车道、压在两侧墙上，于是墙被当成了"车"。
    # 改成 -183°（正好 90°）之后三路降到 2 / 0 / 621。
    EMPTY_LANE_MEASURED = 621

    def _vehicle_points(self, boxes):
        from app.settings import load_settings, load_calib
        from app.pointcloud import AngleCache, roi_arrays
        from app.radar_packet import PacketDecoder
        from app.radar_sim import RadarSimulator
        ch = load_settings("").channels[0]
        calib = load_calib(ch.abs_config_path)
        angle = AngleCache(ch.abs_angle_cache).load()
        lid = ch.lidar
        sim = RadarSimulator(calib.packet, calib.processing, angle,
                             range_mode=lid.range_mode, z_down=lid.z_down,
                             az_step_deg=0.8, range_noise_m=0.002,
                             dropout_prob=0.0, seed=3)
        dec = PacketDecoder(calib.packet, calib.processing, angle,
                            range_mode=lid.range_mode, z_down=lid.z_down,
                            max_range_m=lid.max_range_m,
                            min_range_m=calib.processing.min_distance_mm / 1000.0)
        xyz, _ring, _inten, _info = dec.decode_frame_batch(sim.scan(boxes))
        lo, hi = roi_arrays(calib.roi)
        xyz = xyz[np.all((xyz >= lo) & (xyz <= hi), axis=1)]
        d = calib.detection
        n = int(np.count_nonzero(xyz[:, 2] > d.ground_z + d.vehicle_min_height_m))
        return n, d

    def test_empty_truck_bed_still_triggers(self):
        """★ 空车厢必须够得着阈值 —— 够不着就等于这套系统不判空车了。"""
        from app.radar_sim import box_ground, box_truck
        from app.settings import load_settings, load_calib
        gz = load_calib(load_settings("").channels[0].abs_config_path).detection.ground_z
        n, d = self._vehicle_points([box_ground(z_floor=gz),
                                     box_truck(dx=2.4, dy=12.0, dz=2.6, z_floor=gz)])
        self.assertGreater(
            n, d.vehicle_threshold,
            f"一节空车厢只有 {n} 个车点，够不着阈值 {d.vehicle_threshold} —— "
            f"空车会**完全不触发会话**，一份记录都不留。"
            f"阈值必须低于空车厢的点数，见 docs/08 §1.3。")

    def test_threshold_clears_the_measured_empty_lane(self):
        """空车道那一侧：阈值（含回差）必须明显高于实测底数。"""
        from app.settings import load_settings, load_calib
        d = load_calib(load_settings("").channels[0].abs_config_path).detection
        self.assertGreater(
            d.vehicle_threshold, self.EMPTY_LANE_MEASURED * 1.2,
            f"阈值 {d.vehicle_threshold} 离实测空车道底数 "
            f"{self.EMPTY_LANE_MEASURED} 太近 —— 空车道会被判成有车，"
            f"然后地面不动→判已停稳→会话永不结束→真车全漏（9-13 事故）。")
        release = d.vehicle_threshold * d.vehicle_release_ratio
        self.assertGreater(
            release, self.EMPTY_LANE_MEASURED,
            f"回差线 {release:.0f} 低于实测空车道底数 {self.EMPTY_LANE_MEASURED} —— "
            f"车走了以后点数掉不回回差线以下，会话结束不了。")

    def test_empty_lane_scene_stays_idle(self):
        """只有地面时车点数必须远低于阈值（仿真侧的同一条性质）。"""
        from app.radar_sim import box_ground
        from app.settings import load_settings, load_calib
        gz = load_calib(load_settings("").channels[0].abs_config_path).detection.ground_z
        n, d = self._vehicle_points([box_ground(z_floor=gz)])
        self.assertLess(n, d.vehicle_threshold * d.vehicle_release_ratio,
                        f"空地面竟然有 {n} 个点被算成车")


class TestDegradedFrameIsSkipped(unittest.TestCase):
    """残缺帧不参与"车动没动"的判定。

    dev 是**整帧质心**的位移，而质心取决于"这一帧有哪些点"。突发丢包会让
    一圈缺掉一整块连续扇形 —— 2026-09-14 现场实测：一圈 901 个报文，最差
    时连续缺 31 个（≈12.8° 的楔块），dev 随之冲到 0.13~0.22 m，而车一动没动。

    后果是现场报的那个症状：**车停稳了，系统还说在动** ——
    沉降窗口被作废、重等 3 秒，如此反复。而且短帧是成串出现的
    （实测 111 帧里有 28 帧偏少），`_over` 很容易攒够 release_frames。

    修法不是放宽阈值（那会让真正的蠕行也被当成静止），而是**把残缺帧整帧跳过**：
    不更新参考帧、不累计 _over、不改状态，于是下一个完整帧和**上一个完整帧**比。
    度量本身一个字没动 —— 改的只是哪些帧有资格参与比较。
    """

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    def _det(self):
        return StabilityDetector(stability_delta=0.05, vehicle_threshold=1,
                                 ground_z=-10.0, vehicle_min_height_m=0.0)

    def _frame(self, shift=0.0, n=5000, seed=1):
        r = np.random.default_rng(seed)
        a = r.normal(0, 1, (n, 3)).astype(np.float32)
        a[:, 1] *= 4
        a[:, 0] += shift
        return Frame(a)

    def test_degraded_frame_clears_stop_not_presence(self):
        d = self._det()
        self.clock.detect(d, self._frame())                      # 建参考帧
        r = self.clock.hold(d, self._frame())                    # 连续停稳
        self.assertTrue(r.stable)
        # 残缺帧：质心差 0.5 m，远超阈值，但不该被当成"车动了"
        r = self.clock.detect(d, self._frame(shift=0.5), trusted=False)
        self.assertFalse(r.stable, "残缺帧必须撤销测量许可")
        self.assertTrue(r.degraded)
        self.assertEqual(r.state, StabilityState.BASELINE)

    def test_next_good_frame_compares_against_last_good_frame(self):
        """★ 跳过之后，参考帧必须还停在**上一个完整帧**上。

        若残缺帧污染了参考帧，下一个完整帧就会相对那个歪掉的质心去比，
        等于把同一次丢包的影响算了两遍（CLAUDE.md 第 7 条：掉一帧会产生
        **两次**超阈值 —— 偏出去一次、偏回来一次）。
        """
        d = self._det()
        self.clock.detect(d, self._frame())
        self.clock.detect(d, self._frame())
        self.clock.detect(d, self._frame(shift=0.5), trusted=False)
        r = self.clock.detect(d, self._frame())                  # 完整帧，和最初那帧一样
        self.assertLess(r.deviation, 1e-6,
                        f"参考帧被残缺帧污染了，dev={r.deviation:.4f}")
        self.assertFalse(r.stable, "恢复首帧不能沿用中断前的停稳时间")
        self.assertTrue(self.clock.hold(d, self._frame()).stable)

    def test_a_run_of_degraded_frames_does_not_accumulate(self):
        """短帧是成串来的 —— 连续残缺也不能攒够 release_frames 把稳态打掉。"""
        d = self._det()
        self.clock.detect(d, self._frame())
        self.assertTrue(self.clock.hold(d, self._frame()).stable)
        for i in range(8):
            r = self.clock.detect(d, self._frame(shift=0.3 + 0.05 * i), trusted=False)
            self.assertFalse(r.stable, f"第 {i+1} 个残缺帧仍有测量许可")
            self.assertNotEqual(r.state, StabilityState.IDLE)

    def test_real_motion_still_breaks_stable(self):
        """反向：完整帧上的真实位移必须照旧判成"又动了"，别把判据废掉。"""
        d = self._det()
        self.clock.detect(d, self._frame())
        self.assertTrue(self.clock.hold(d, self._frame()).stable)
        # 单帧大位移（>= delta × break_ratio = 0.30）立刻退出
        r = self.clock.detect(d, self._frame(shift=0.6))
        self.assertFalse(r.stable)
        self.assertEqual(r.state, StabilityState.BASELINE)

    def test_small_real_motion_still_breaks_after_release_frames(self):
        d = self._det()
        self.clock.detect(d, self._frame())
        self.assertTrue(self.clock.hold(d, self._frame()).stable)
        last = None
        for i in range(4):                            # 每帧都挪一点点
            last = self.clock.detect(d, self._frame(shift=0.08 * (i + 1)))
        self.assertFalse(last.stable, "连续小位移应当判成又动了")


class TestEnteringVehicleIsNotStable(unittest.TestCase):
    """车还在进场时，不管质心偏差多小都不能判"已停稳"。

    ★ 现场症状（2026-09-14，CH4 实测，每一帧都记了下来）

        t=182.91  车点 15 605  —— 刚越过阈值，建参考帧
        t=183.02  车点 16 737  偏差 0.0413 → 判「已停稳」，还开了 3 秒沉降窗口
        t=183.30  车点 21 143  偏差 0.0541 → 抖动 1
        t=183.52  车点 24 434  偏差 0.0709 → 翻回「运动中」
        t=185.34  车点 83 919  偏差 0.0309 → 又判「已停稳」
        …车一直在往里开，状态却来回跳。这就是现场报的
        "车辆行驶状态每秒都在变"。

    ★ 为什么质心偏差判不出来

        dev 是**整帧**质心的位移，而整帧里绝大部分是地面 —— 车往里开一截，
        整帧质心只挪几厘米，落在 stability_delta(0.05) 以下。

    ★ 点数是干净得多的信号（同一次实测）

        进场中  每帧变化 1% ~ 6.8%
        真停稳  每帧变化 ≤ 0.11%

        差 50 倍。所以再加一条独立门控：点数还在明显变化就不算停稳。
        **质心度量一个字没动**（仍是验证到 1e-7 的 ‖质心差‖），
        改的只是"什么时候允许判停稳"。

    ★ 为什么要和 5 帧前比、而不是只和上一帧比

        进场途中会有短暂顿挫，单帧比较会被骗过去。按实测数据回放：
            只比上一帧  首次判停稳比真停稳早 3.6s，状态翻转 5 次
            比 5 帧前    早 0.5s，翻转 1 次      ← 取这个
            旧逻辑       早 6.1s，翻转 5 次
    """

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    def _det(self, ratio=0.005):
        return StabilityDetector(stability_delta=0.05, vehicle_threshold=1,
                                 ground_z=-10.0, vehicle_min_height_m=0.0,
                                 count_ratio=ratio)

    def _frame(self, n, seed=1):
        """点数为 n 的一帧，质心固定在原点 —— 排除质心的影响，只看点数门控。"""
        r = np.random.default_rng(seed)
        a = r.normal(0, 1, (n, 3)).astype(np.float32)
        a[:, 1] *= 4
        a -= a.mean(0)
        return Frame(a)

    def _frame_shift(self, n, dx, seed=1):
        """同上，但质心平移 dx —— 用来制造一次"真实位移"。"""
        f = self._frame(n, seed)
        f.xyz[:, 0] += np.float32(dx)
        return f

    def test_growing_point_count_is_not_stable(self):
        """★ 质心完全不动，但点数每帧涨 7% —— 必须判成还在动。"""
        d = self._det()
        self.clock.detect(d, self._frame(15000))
        n = 15000
        for _ in range(8):
            n = int(n * 1.07)
            r = self.clock.detect(d, self._frame(n))
            self.assertFalse(r.stable,
                             f"点数涨到 {n}（+7%/帧）却判成了已停稳")
            self.assertEqual(r.state, StabilityState.BASELINE)

    def test_steady_point_count_is_stable(self):
        """点数稳住之后要能正常判停稳，别把判据卡死。"""
        d = self._det()
        self.clock.detect(d, self._frame(118000))
        last = None
        for _ in range(8):
            last = self.clock.detect(d, self._frame(118000))
        self.assertTrue(last.stable, "点数稳定却判不出停稳")

    def test_field_measured_parked_jitter_is_tolerated(self):
        """实测停稳时点数波动 ±0.11%，不能因此判成在动。"""
        d = self._det()
        counts = [117937, 118112, 117956, 118066, 117968, 118090, 117979,
                  118028, 117940, 118057]
        self.clock.detect(d, self._frame(counts[0]))
        last = None
        for n in counts[1:]:
            last = self.clock.detect(d, self._frame(n))
        self.assertTrue(last.stable,
                        f"实测的停稳抖动（{min(counts)}~{max(counts)}）被误判成在动")

    def test_brief_pause_during_entry_does_not_fool_it(self):
        """★ 进场途中顿一下：和 5 帧前比，仍然看得出在动。"""
        d = self._det()
        # 前 5 帧快速增长，第 6 帧几乎不动（顿挫）
        seq = [20000, 24000, 28000, 33000, 38000, 38100]
        self.clock.detect(d, self._frame(seq[0]))
        last = None
        for n in seq[1:]:
            last = self.clock.detect(d, self._frame(n))
        self.assertFalse(last.stable,
                         "进场途中顿了一帧就被判成停稳 —— 说明只和上一帧比了")

    def test_count_change_revokes_measurement_and_recovers(self):
        """点数显著变化也可能是进出场，不能保留历史测量许可。"""
        d = self._det()
        self.assertTrue(self.clock.hold(d, self._frame(118000)).stable)
        r = self.clock.detect(d, self._frame(113300))
        self.assertFalse(r.stable)
        self.assertTrue(self.clock.hold(d, self._frame(118000), steps=20).stable)

    def test_vehicle_driving_away_does_break_stable(self):
        """★ 但车真的开走时必须打断 —— 那是点数**极大**变化（> 阈值 × break_ratio）。

        2026-09-14 实测离场：点数 80 141 → 13 978，而质心偏差只有 0.0478，
        旧判据一路显示「已停稳」直到车都快出画面。
        打断阈值 0.03 × 6 = 0.18，在 1390 帧真·停稳数据上零误报（最差 0.0396）。
        """
        d = self._det()
        self.assertTrue(self.clock.hold(d, self._frame(118000)).stable)
        for _ in range(8):
            self.assertTrue(self.clock.detect(d, self._frame(118000)).stable)
        n = 118000
        broke = False
        for _ in range(6):
            n = int(n * 0.75)                  # 车往外开，点数骤降
            if not self.clock.detect(d, self._frame(n)).stable:
                broke = True; break
        self.assertTrue(broke, "车都开走了还一直显示已停稳")

    def test_gate_still_blocks_entry_after_a_break(self):
        """被真实位移打断之后，门控要重新生效（否则进场那半段就白拦了）。"""
        d = self._det()
        self.clock.detect(d, self._frame(118000))
        self.assertTrue(self.clock.hold(d, self._frame(118000)).stable)
        # 用一次大位移把稳态打掉
        r = self.clock.detect(d, self._frame_shift(118000, 0.6))
        self.assertFalse(r.stable)
        # 此后点数猛涨（车在进场）必须重新被拦住
        n = 118000
        for _ in range(6):
            n = int(n * 1.07)
            self.assertFalse(self.clock.detect(d, self._frame(n)).stable,
                             "打断之后门控没有重新生效")

    def test_ratio_zero_disables_the_gate(self):
        """count_ratio=0 退回旧行为，留一条后路。"""
        d = self._det(ratio=0.0)
        self.clock.detect(d, self._frame(15000))
        r = self.clock.detect(d, self._frame(30000))          # 点数翻倍
        self.assertFalse(r.stable, "关闭点数门控不能关闭连续时间门控")
        self.assertTrue(self.clock.hold(d, self._frame(30000)).stable)

    def test_leaving_vehicle_is_also_caught(self):
        """车往外开时点数骤降 —— 同样不算停稳。"""
        d = self._det()
        self.clock.detect(d, self._frame(118000))
        n = 118000
        for _ in range(5):
            n = int(n * 0.9)
            r = self.clock.detect(d, self._frame(n))
            self.assertFalse(r.stable, f"点数掉到 {n} 却判成已停稳")


class TestVolume(unittest.TestCase):

    def test_no_baseline_skips(self):
        """没有空车基线时，体积分支必须安全跳过而不是给出错误数字。

        现场从未生成过空车基线，所以原版的这段分支实际上从未跑过 ——
        这也是它最容易藏着未验证行为的地方，务必保证"没基线就明确跳过"。
        """
        from app.detect import LoadedResult
        v = compute_volume(None, LoadedResult())
        self.assertFalse(v.ok)
        self.assertEqual(v.volume_m3, 0.0)
        self.assertIn("空车基线", v.message)


if __name__ == "__main__":
    unittest.main()


def _carriage_scene(y0=-5.0, y1=5.0, w=2.45, bed=1.2, wall=2.8,
                    cab=True, ground=True, z_shift=0.0, seed=7):
    """造一个「地面 + 车厢 + 驾驶室」的场景，坐标系与现场一致（z 向下为负）。

    车厢：内宽 w，底板在 bed，壁顶在 wall（都相对地面）。
    驾驶室：接在车厢 +y 端，中间留 0.6 m 缝，顶比壁顶低 1.4 m，宽 2.5（同量级）。
    地面：占满 ROI 的 x[-2,2] —— 宽 4 m，宽度判据必须把它甩掉。
    z_shift 用来验证「不依赖 ground_z」：整片平移 z，答案不许变。
    """
    rng = np.random.default_rng(seed)
    g = -5.23 + z_shift
    parts = []
    if ground:
        # 地面要给够密度：现场一帧 ROI 内 5~6 万点，地面占大头。
        # ★ 车所在的那一段 y 上，ROI 内**一点地面都看不到** —— 不只是车底下。
        #   雷达吊在 5.2 m，车厢高 4.0 m、半宽 1.25 m，地面阴影一直伸到
        #   |x| = 1.25 x 5.2 / (5.2 - 4.0) = 5.4 m，远超 ROI 的 |x| <= 2。
        #   宽度判据之所以能成立，靠的正是这个遮挡：车厢那几格只有车厢自己
        #   （宽 2.4~2.6），空车道那几格是宽 4 m 的整片地面。
        #   造景漏了它的话，车厢格会量出 ~3.0 m 的宽度，一份都切不出来。
        n = 30000
        gx = rng.uniform(-2.0, 2.0, n)
        gy = rng.uniform(-11.0, 8.0, n)
        shadow = (gy >= y0) & (gy <= y1)
        if cab:
            shadow |= (gy >= y1 + 0.6) & (gy <= y1 + 3.0)
        gx, gy = gx[~shadow], gy[~shadow]
        parts.append(np.c_[gx, gy, np.full(len(gx), g) + rng.normal(0, 0.01, len(gx))])
    n = 12000
    yy = rng.uniform(y0, y1, n)
    parts.append(np.c_[rng.uniform(-w / 2, w / 2, n), yy,
                       np.full(n, g + bed) + rng.normal(0, 0.01, n)])
    for side in (-1, 1):
        n = 4000
        parts.append(np.c_[np.full(n, side * w / 2) + rng.normal(0, 0.01, n),
                           rng.uniform(y0, y1, n),
                           rng.uniform(g + bed, g + wall, n)])
    if cab:
        n = 5000
        parts.append(np.c_[rng.uniform(-1.25, 1.25, n),
                           rng.uniform(y1 + 0.6, y1 + 3.0, n),
                           np.full(n, g + wall - 1.4) + rng.normal(0, 0.01, n)])
    return np.vstack(parts).astype(np.float64)


class TestCarriageBox(unittest.TestCase):
    """carriage_box —— 按横截面把车厢切出来。

    ▲ 每条用例都要能在功能被拿掉时真的失败，别写成「反正都过」的空用例。
    """

    def test_finds_carriage_and_rejects_ground_and_cab(self):
        a = _carriage_scene(y0=-5.0, y1=5.0, w=2.45)
        b = carriage_box(a)
        self.assertIsNotNone(b)
        self.assertAlmostEqual(b["y0"], -5.0, delta=0.35)
        self.assertAlmostEqual(b["y1"], 5.0, delta=0.35)
        self.assertAlmostEqual(b["width"], 2.45, delta=0.12)
        # 空用例防线：朴素做法（整片点云的 y 范围）会被地面撑到 -11..8，
        # 所以「切出来 != 朴素」这一条必须成立，否则说明判据根本没起作用
        self.assertLess(b["y1"] - b["y0"], 0.7 * (a[:, 1].max() - a[:, 1].min()))

    def test_ignores_ground_only_scene(self):
        """只有地面（宽 4 m）时必须返回 None —— 宽度判据的直接后果。"""
        a = _carriage_scene(ground=True, cab=False, y0=0.0, y1=0.2)
        a = a[a[:, 2] < -5.0]
        self.assertIsNone(carriage_box(a))

    def test_does_not_depend_on_ground_z(self):
        """整片点云平移 z，结果不许变 —— 本函数刻意不挂 ground_z。"""
        base = carriage_box(_carriage_scene(z_shift=0.0))
        for dz in (-1.5, +1.5, +3.0):
            got = carriage_box(_carriage_scene(z_shift=dz))
            self.assertIsNotNone(got)
            self.assertAlmostEqual(got["y0"], base["y0"], delta=0.06)
            self.assertAlmostEqual(got["y1"], base["y1"], delta=0.06)

    def test_cab_is_excluded_by_the_height_jump(self):
        """驾驶室顶比厢顶低 1.4 m，jump_m 必须把它断开。

        把 jump_m 放大到 3.0（等于关掉这道判据）时前沿必须明显往 +y 跑 ——
        跑不动就说明这道判据本来就没起作用。
        """
        a = _carriage_scene(y0=-5.0, y1=5.0)
        tight = carriage_box(a, jump_m=0.8)
        loose = carriage_box(a, jump_m=3.0, gap_m=1.0)
        self.assertIsNotNone(tight)
        self.assertIsNotNone(loose)
        self.assertLess(tight["y1"], 5.5)
        self.assertGreater(loose["y1"] - tight["y1"], 1.0)

    def test_short_carriage(self):
        a = _carriage_scene(y0=-4.5, y1=1.5, w=2.40)
        b = carriage_box(a)
        self.assertIsNotNone(b)
        self.assertAlmostEqual(b["length"], 6.0, delta=0.6)

    def test_too_few_points_returns_none(self):
        self.assertIsNone(carriage_box(np.zeros((10, 3))))


    def test_nadir_waist_does_not_split_the_carriage(self):
        """天底腰不许把车厢劈成两半 —— 宽度判据的回差就是为这个加的。

        场景：一条正常车厢，但正中间 0.5 m 的宽度掉到 w_lo 以下一点点
        （现场实测雷达正下方只打到货面顶、扫不到厢壁顶，62 份里 55 份
        天底更窄，中位窄 0.062 m、p90 窄 0.138 m）。
        关掉回差（w_slack=0）时必然被劈短 —— 这一条同时钉住回差有没有生效。
        """
        a = _carriage_scene(y0=-5.5, y1=5.5, w=2.30, cab=False)
        # 腰做到 2.12（比 w_lo=2.15 低 0.03）—— 照现场实测的深度来：
        # 62 份里天底比偏开处窄，中位 0.062 m、p90 0.138 m，都在 w_slack 之内。
        # ▲ 要**把壁点整体缩进来**，不能把壁点删掉：删掉只剩地板点均匀分布，
        #   p2..p98 再削掉尾巴就变成 2.01（低过 w_slack），那是造景失真。
        #   现场的天底腰是壁点仍在、只是量出来略窄。
        waist = np.abs(a[:, 1]) < 0.35
        a = a.copy()
        a[waist, 0] *= 2.12 / 2.33      # 2.33 是这个场景量出来的实际宽度
        wide = carriage_box(a, w_slack=0.15)
        tight = carriage_box(a, w_slack=0.0)
        self.assertIsNotNone(wide)
        self.assertIsNotNone(tight)
        # 带回差：整条都在
        self.assertGreater(wide["length"], 9.0)
        # 不带回差：被腰劈短
        self.assertLess(tight["length"], wide["length"] - 2.0)

    def test_slack_does_not_drag_the_endpoint_outwards(self):
        """松格只许桥接，不许把端点往外拽 —— 否则回差会让车厢越切越长。"""
        a = _carriage_scene(y0=-5.0, y1=5.0)
        base = carriage_box(a, w_slack=0.0)
        loose = carriage_box(a, w_slack=0.15)
        self.assertIsNotNone(base)
        self.assertIsNotNone(loose)
        self.assertAlmostEqual(loose["y0"], base["y0"], delta=0.30)
        self.assertAlmostEqual(loose["y1"], base["y1"], delta=0.30)


class TestCarriageConfidence(unittest.TestCase):
    """自举置信度 —— 没有它就会把「可复现的偏差」当成「准」。"""

    def test_clean_scene_is_confident(self):
        r = carriage_confidence(_carriage_scene(y0=-5.0, y1=5.0))
        self.assertTrue(r["ok"])
        self.assertLessEqual(r["sway"], 0.25)
        self.assertEqual(r["n"], len(CARRIAGE_JITTER))

    def test_ambiguous_scene_is_flagged(self):
        """驾驶室顶做到与厢顶只差 0.3 m、缝也几乎没有 —— 这时「车厢到哪结束」
        在数据里本来就说不清，必须报不可信，而不是硬给一个数。"""
        rng = np.random.default_rng(3)
        a = _carriage_scene(y0=-5.0, y1=5.0, cab=False)
        g = -5.23
        n = 6000
        blur = np.c_[rng.uniform(-1.22, 1.22, n), rng.uniform(5.02, 8.0, n),
                     np.full(n, g + 2.5) + rng.normal(0, 0.05, n)]
        r = carriage_confidence(np.vstack([a, blur]))
        self.assertFalse(r["ok"])
        self.assertGreater(r["sway"], 0.25)

    def test_confidence_still_returns_a_box_when_not_ok(self):
        rng = np.random.default_rng(5)
        a = _carriage_scene(y0=-5.0, y1=5.0, cab=False)
        n = 6000
        blur = np.c_[rng.uniform(-1.22, 1.22, n), rng.uniform(5.02, 8.0, n),
                     np.full(n, -5.23 + 2.5) + rng.normal(0, 0.05, n)]
        r = carriage_confidence(np.vstack([a, blur]))
        self.assertIsNotNone(r["box"])


class TestCarriageAgreement(unittest.TestCase):
    """多帧一致性 —— 判「这次切得可不可信」唯一靠得住的办法。

    ▲ 现场实测它多半会说"不可信"（62 份里只有 8 份过），这是**对的**：
      同一辆停着的车三连帧之间端点本来就摆 1 米量级（docs/08 §17）。
      用例钉的是"它有没有如实反映一致性"，不是"它是不是总说可信"。
    """

    def test_same_scene_repeated_agrees(self):
        cl = [_carriage_scene(y0=-5.0, y1=5.0, seed=s) for s in (1, 2, 3)]
        r = carriage_agreement(cl)
        self.assertTrue(r["ok"], r)
        self.assertEqual(r["n"], 3)
        self.assertLessEqual(r["sway"], 0.30)
        self.assertFalse(r["moved"])       # 车没动，不许报成"车在动"

    def test_shifted_scene_is_flagged_as_moved(self):
        """把车整体挪 1.2 m —— 必须报不一致，**并且认出是车在动**。

        这两件事要分开：端点对不上有两种原因，算法不稳 vs 车真的动了。
        混在一起报的话，逐辆核对时会把 5 组"车在动"的当成算法缺陷去查。
        """
        cl = [_carriage_scene(y0=-5.0, y1=5.0, seed=1),
              _carriage_scene(y0=-3.8, y1=6.2, seed=2)]
        r = carriage_agreement(cl)
        self.assertFalse(r["ok"])
        self.assertGreater(r["sway"], 1.0)
        self.assertTrue(r["moved"])
        self.assertIn("车在动", r["why"])

    def test_single_frame_cannot_judge(self):
        """单帧判不了一致性 —— 必须说出来，不能假装可信。"""
        r = carriage_agreement([_carriage_scene()])
        self.assertFalse(r["ok"])
        self.assertIn("量不了", r["why"])

    def test_empty_input(self):
        r = carriage_agreement([])
        self.assertFalse(r["ok"])
        self.assertEqual(r["n"], 0)

    def test_uncuttable_frame_is_reported(self):
        cl = [_carriage_scene(), np.zeros((10, 3))]
        r = carriage_agreement(cl)
        self.assertFalse(r["ok"])
        self.assertIn("切不出", r["why"])

class TestBaselineManifest(unittest.TestCase):
    """回归基线的校验和清单 —— `docs/14` §2.3。

    这道闸门的全部效力都来自 `data/loaded` 里那 10 份样本：
    点云是输入、`.reason.txt` 是标准答案。**任何一方被无声改动，
    闸门就悄悄变松，而它照样报「全部通过」。**

    实际会发生的几种走法（都在下面钉住了）：
      · 编辑器打开 reason.txt 顺手把换行改成 CRLF；
      · `tools/fix_plate_dirs.py --apply` 被误跑（CLAUDE.md 专门警告过）；
      · 有人"顺手"往基线里塞一份恰好通过的样本 —— 分母变大、难度变低；
      · 某一份被删掉。

    所有用例都在 `data/loaded` 的**临时副本**上做，绝不碰真基线。
    """

    @staticmethod
    def _verify_mod():
        import importlib
        import sys as _sys
        cli_dir = os.path.join(H.ROOT, "cli")
        if cli_dir not in _sys.path:
            _sys.path.insert(0, cli_dir)
        return importlib.import_module("verify")

    def _copy(self):
        import shutil
        import tempfile
        d = os.path.join(tempfile.mkdtemp(), "loaded")
        shutil.copytree(os.path.join(H.ROOT, *BASELINE_SUBDIR), d)
        return d

    def _write_manifest(self, d):
        import json as _json
        V = self._verify_mod()
        with io.open(os.path.join(d, V.MANIFEST_NAME), "w", encoding="utf-8") as f:
            f.write(_json.dumps(V.build_manifest(d), ensure_ascii=False, indent=2))

    def test_manifest_covers_clouds_and_answers(self):
        """点云和标准答案**都**要在清单里 —— 只保一边等于没保。"""
        V = self._verify_mod()
        man = V.build_manifest(os.path.join(H.ROOT, *BASELINE_SUBDIR))
        names = list(man["files"])
        self.assertTrue(any(n.endswith(".xyz") for n in names))
        self.assertTrue(any(n.endswith(".reason.txt") for n in names),
                        "清单里没有 .reason.txt —— 标准答案被改了会察觉不到")
        self.assertEqual(man["count"], len(names))

    def test_real_baseline_matches_its_manifest(self):
        """★ 真基线必须与仓库里那份清单一致。

        这条红了有两种可能：基线被动过，或者改动是有意的但忘了
        `cli/verify.py --update-manifest`。两种都该停下来看一眼。
        """
        V = self._verify_mod()
        d = os.path.join(H.ROOT, *BASELINE_SUBDIR)
        if not os.path.exists(os.path.join(d, V.MANIFEST_NAME)):
            self.skipTest("仓库里还没有 MANIFEST.json")
        state, lines = V.check_manifest(d)
        self.assertEqual(state, "ok", chr(10).join(lines))

    def test_missing_manifest_is_not_a_failure(self):
        """还没生成清单时只警告，不失败 —— 不能因为一个以前不存在的文件罚人。"""
        V = self._verify_mod()
        d = self._copy()
        mp = os.path.join(d, V.MANIFEST_NAME)
        if os.path.exists(mp):
            os.remove(mp)
        state, lines = V.check_manifest(d)
        self.assertEqual(state, "missing")
        self.assertTrue(any("没有被校验和保护" in x for x in lines))

    def test_modified_answer_is_caught(self):
        """改标准答案 —— 最阴险的一种，闸门会照常报通过。"""
        V = self._verify_mod()
        d = self._copy()
        self._write_manifest(d)
        hit = None
        for root_, _dirs, fs in os.walk(d):
            for f in fs:
                if f.endswith(".reason.txt"):
                    hit = os.path.join(root_, f)
                    break
            if hit:
                break
        self.assertIsNotNone(hit)
        with io.open(hit, encoding="utf-8") as f:
            t = f.read()
        with io.open(hit, "w", encoding="utf-8") as f:
            f.write(t + "x")
        state, lines = V.check_manifest(d)
        self.assertEqual(state, "bad")
        self.assertTrue(any("内容被改" in x for x in lines), lines)

    def test_crlf_rewrite_is_caught(self):
        """编辑器把换行改成 CRLF —— 内容"看起来没变"，但闸门的输入变了。"""
        V = self._verify_mod()
        d = self._copy()
        self._write_manifest(d)
        hit = None
        for root_, _dirs, fs in os.walk(d):
            for f in fs:
                if f.endswith(".reason.txt"):
                    hit = os.path.join(root_, f)
                    break
            if hit:
                break
        with open(hit, "rb") as f:
            b = f.read()
        with open(hit, "wb") as f:
            f.write(b.replace(b"\n", b"\r\n") if b"\n" in b else b + b"\r\n")
        state, _lines = V.check_manifest(d)
        self.assertEqual(state, "bad")

    def test_deleted_sample_is_caught(self):
        """删一份 = 分母变小 = 闸门变松。"""
        V = self._verify_mod()
        d = self._copy()
        self._write_manifest(d)
        for root_, _dirs, fs in os.walk(d):
            gone = [f for f in fs if f.endswith(".xyz")]
            if gone:
                os.remove(os.path.join(root_, gone[0]))
                break
        state, lines = V.check_manifest(d)
        self.assertEqual(state, "bad")
        self.assertTrue(any("少了" in x for x in lines), lines)

    def test_added_sample_is_caught(self):
        """往基线里塞一份恰好通过的样本，同样是变松。"""
        import shutil
        V = self._verify_mod()
        d = self._copy()
        self._write_manifest(d)
        for root_, _dirs, fs in os.walk(d):
            src = [f for f in fs if f.endswith(".xyz")]
            if src:
                a = os.path.join(root_, src[0])
                shutil.copy(a, a[:-4] + "_ZZZ.xyz")
                shutil.copy(a + ".reason.txt", a[:-4] + "_ZZZ.xyz.reason.txt")
                break
        state, lines = V.check_manifest(d)
        self.assertEqual(state, "bad")
        self.assertTrue(any("多了" in x for x in lines), lines)

    def test_untouched_copy_is_ok(self):
        V = self._verify_mod()
        d = self._copy()
        self._write_manifest(d)
        state, _lines = V.check_manifest(d)
        self.assertEqual(state, "ok")

def _dev_gate():
    from app.settings import load_settings, load_devices
    st = load_settings("")
    return load_devices(st.enabled()[0].abs_devices_path)


def _diag(step, **kw):
    """造一份"除了指定项之外全部合格"的诊断，用来定点触发某一步。"""
    from app.detect import Diagnostics
    g = _dev_gate().gate
    d = Diagnostics()
    d.n_points = 60000
    d.normal_z = 0.999
    d.yaw_deg = 0.3
    d.inliers = g.min_inliers + 100
    d.ratio = g.min_inlier_ratio + 0.05
    d.span_x = g.min_span_x_m + 1.0
    d.span_y = g.min_span_y_m + 1.0
    d.z_above_used = 0.01
    d.z_above_mode = "plane"
    for k, v in kw.items():
        setattr(d, k, v)
    d.step = step
    return d


class TestReasonDetail(unittest.TestCase):
    """§3.1 —— 第 8 步与第 9 步返回**同一个** reason 字符串。

    「底面覆盖范围不足」既可能是 x 跨度不够（车停歪 / ROI 横向偏），
    也可能是 y 跨度不够（车厢太短 / 被 ROI 切掉一截）——
    现场处置完全不同，而看 reason 的人分不出来。

    ▲ reason 字符串是红线（与原厂逐字一致，改了破 verify.py），
      所以只能在诊断侧补 `reason_detail()`，落盘的 reason.txt 一个字节不动。
    """

    def test_step8_and_step9_are_distinguishable(self):
        """★ 这条就是本次修复的目标。"""
        from app.detect import reason_detail
        dev = _dev_gate()
        d8 = reason_detail(_diag(8, span_x=1.20), dev)
        d9 = reason_detail(_diag(9, span_y=3.10), dev)
        self.assertNotEqual(d8, d9, "第 8/9 步的说明一模一样 —— 等于没补")
        self.assertIn("第 8 步", d8)
        self.assertIn("X", d8.upper())
        self.assertIn("第 9 步", d9)
        self.assertIn("Y", d9.upper())

    def test_reason_string_itself_is_unchanged(self):
        """★ 补说明**不许**动 reason 本身 —— 动了直接破回归闸门。"""
        from app.detect import R_FLOOR_COVERAGE
        self.assertEqual(R_FLOOR_COVERAGE, "底面覆盖范围不足")

    def test_pass_case(self):
        from app.detect import reason_detail
        self.assertIn("空车", reason_detail(_diag(11), _dev_gate()))

    def test_boolean_gate_says_not_passed(self):
        """没有数值门槛的那几步，要明说「未通过」——
        只写条目名（"厢底平面有效"）读起来像是通过了。"""
        from app.detect import reason_detail
        txt = reason_detail(_diag(3, normal_z=0.5), _dev_gate())
        self.assertIn("第 3 步", txt)
        self.assertTrue("未通过" in txt or "<" in txt, txt)

    def test_numeric_detail_has_value_and_limit(self):
        from app.detect import reason_detail
        txt = reason_detail(_diag(7, ratio=0.01), _dev_gate())
        self.assertIn("0.01", txt)
        self.assertIn("0.15", txt)

    def test_none_is_empty(self):
        from app.detect import reason_detail
        self.assertEqual(reason_detail(None, _dev_gate()), "")


class TestFitOkAlias(unittest.TestCase):
    """§3.4 —— `EmptyResult.ok` 的含义是"六面拟合成功"，不是"空车"。

    这个名字已经咬过一次：落盘 `meta.empty` 写的就是 `res.ok`，
    137 份现场样本里 **115 份**标着 empty=True 而实际是载货车（docs/13 §0）。
    """

    def test_alias_matches(self):
        from app.detect import EmptyResult
        for v in (True, False):
            r = EmptyResult()
            r.ok = v
            self.assertEqual(r.fit_ok, v)

    def test_to_dict_has_both(self):
        """`ok` 保留（看板契约钉着），`fit_ok` 并排给出。"""
        from app.detect import EmptyResult
        r = EmptyResult()
        r.ok = True
        d = r.to_dict()
        self.assertIn("ok", d)
        self.assertIn("fit_ok", d)
        self.assertEqual(d["ok"], d["fit_ok"])


class TestVolumeRatioDecomposition(unittest.TestCase):
    """§3.2 —— `filled_ratio` 与 `hole_ratio` 名字太近、用途完全不同。

    补 `wall_ratio` 之后关系变成显式的：**filled = wall + hole**。
    · wall（厢壁盲区）两侧都有实测货面，插值有依据，且**每车都有**
      -> 这个数恒在 20~35%，不能当判据；
    · hole（真空洞）周围也没点，填多少纯属假设 -> 这才是判据。

    2026-09-17 我把 `confidence()` 的门槛挂在 filled_ratio 上，
    结果把两辆有磅单真值的好车全标成低置信（docs/14 §1.3）。
    """

    def _vol(self, drop_ends=0):
        from app.detect import compute_volume
        from tests.test_field_fixes import _bed_and_cargo, _mk
        empty, loaded, _ = _bed_and_cargo(drop_ends=drop_ends, seed=4)
        b, l = _mk(empty, loaded)
        return compute_volume(b, l)

    def test_filled_equals_wall_plus_hole(self):
        """★ 关系必须精确成立，否则这个分解就是装饰。"""
        v = self._vol(drop_ends=8)
        self.assertAlmostEqual(v.filled_ratio, v.wall_ratio + v.hole_ratio, places=9)
        self.assertEqual(v.filled_cells, v.wall_cells + v.hole_cells)

    def test_wall_exists_even_with_perfect_coverage(self):
        """覆盖完好时 hole 该很小，而 wall 仍然存在 ——
        这正是"别拿 filled_ratio 当判据"的原因。"""
        v = self._vol(drop_ends=0)
        self.assertGreater(v.wall_ratio, 0.0)
        self.assertLess(v.hole_ratio, v.wall_ratio,
                        "覆盖完好时空洞占比竟然不小于壁格占比")

    def test_holes_move_hole_ratio_not_wall_ratio(self):
        """挖掉两端只该抬高 hole_ratio，wall_ratio 不该跟着暴涨。"""
        good = self._vol(drop_ends=0)
        bad = self._vol(drop_ends=12)
        self.assertGreater(bad.hole_ratio, good.hole_ratio + 0.03)

    def test_to_dict_exposes_all_three(self):
        d = self._vol(drop_ends=4).to_dict()
        for k in ("filled_ratio", "wall_ratio", "hole_ratio",
                  "filled_cells", "wall_cells", "hole_cells"):
            self.assertIn(k, d)


class TestEmptyEvidence(unittest.TestCase):
    """★ 自动挑空车基线的闸门（2026-09-18）。

    现场车太多，"每辆车人工点一次激活"不可行，基线必须能自动挑。
    而失败是**静默且永久**的 —— 把载货点云存成空车基线，这辆车之后的
    体积恒为接近 0 且不报错。所以判据取保守方向，且必须多条联合。

    ▲▲ 这组用例同时钉住一条**已经踩过的坑**：docs/13 §12.1 那个
      "z 厚度 1.3 m 干净间隙"是 CH4 下午 8 份子集的假象。
      124 份全量实测 z 厚度是连续谱（1.2~2.4 之间压着 43 份，35%），
      底面离地单独用也有 31% 落在模糊带 —— **单条判据不能用**。
    """

    @staticmethod
    def _carriage(thickness=2.7, floor_above=1.3, length=11.0, width=2.45,
                  ground_z=-5.2, seed=3, n=40000):
        """造一个车厢，按**实测的点分布**来，不能想当然。

        2026-09-18 量了真实的两份（`|x| < 1.1` 带内沿 z 的直方图）：
          · 空车 161309：大头压在 z 1.23~1.72（**底板**），
            往上到 4.16 是一条**稀疏的尾巴**（厢壁内面、顶沿、结构件）；
          · 载货 152838：大头在 3.18~3.82（**货面**，整个抬高了），
            往下只有零星点。
        所以 `carriage_profile` 量到的"厚度"是 **底板到那条尾巴顶端**。

        ▲ 第一版夹具把厢壁放在 `x = +-width/2 = +-1.225`，而
          `carriage_profile` 只取 `|x| < 1.1` —— 壁整个被排除，
          厚度量出 **0.000**，四条用例全绿得莫名其妙。
          内面回波实际落在 |x| 1.00~1.09（p99 = 1.08~1.09 实测）。
        """
        rng = np.random.default_rng(seed)
        z0 = ground_z + floor_above
        half = width / 2.0
        bed = np.column_stack((rng.uniform(-half + .05, half - .05, n),
                               rng.uniform(-length / 2, length / 2, n),
                               np.full(n, z0) + rng.normal(0, .01, n)))
        m = max(64, n // 8)                       # 内面回波稀疏，但铺满整个高度
        inner = np.column_stack((
            np.where(rng.random(m) < .5, -1.0, 1.0) * rng.uniform(1.00, 1.09, m),
            rng.uniform(-length / 2, length / 2, m),
            z0 + rng.uniform(0, thickness, m)))
        edge = np.column_stack((                  # 定内宽用，落在 x_lim 之外
            np.where(rng.random(m) < .5, -half, half),
            rng.uniform(-length / 2, length / 2, m),
            z0 + rng.uniform(0, thickness, m)))
        return np.concatenate((bed, inner, edge))

    def _half_loaded(self, covered=0.6, cargo_top_m=2.1, **kw):
        """半装载：前 `covered` 段被煤盖住，尾段露底。

        ★ 这是旧判据（z 厚度 + 底面离地）**漏掉**的那一类：尾段露底
          仍然给出空车的厚度与底面读数。实测现场三份就是这个样子
          （20260914_165712 等，总露出 12%~14%，剖面 100/19/0/0/0/0/0/0）。
        """
        base = self._carriage(**kw)
        length = kw.get("length", 11.0)
        ground_z = kw.get("ground_z", -5.2)
        y_cut = -length / 2 + length * (1.0 - covered)
        # ▲ 只把**被盖住那段的底板**换成煤；**厢壁要留着** ——
        #   真实半装载车的壁还在。第一版连壁一起删了，于是
        #   `carriage_box` 按截面宽度把车厢切成了 4.85 m（煤堆宽 2.10
        #   低于 w_lo=2.15），整段载货区被排除在车厢之外，露出量成 96%。
        is_bed = np.abs(base[:, 0]) < 1.1
        drop = is_bed & (base[:, 1] > y_cut)
        rng = np.random.default_rng(11)
        n = 30000
        coal = np.column_stack((rng.uniform(-1.05, 1.05, n),
                                rng.uniform(y_cut, length / 2, n),
                                np.full(n, ground_z + 1.3 + cargo_top_m) + rng.normal(0, .04, n)))
        return np.concatenate((base[~drop], coal))

    def _evidence(self, cloud=None, frames=2, **kw):
        from app.detect import empty_evidence
        cloud = self._carriage() if cloud is None else cloud
        clouds = [cloud + np.array([0., 0., 1e-4 * i]) for i in range(frames)] if frames else None
        return empty_evidence(cloud, -5.2, clouds=clouds, **kw)

    def _names(self, evidence):
        return {g["name"]: g["passed"] for g in evidence["gates"]}

    def test_a_clean_empty_carriage_passes(self):
        ev = self._evidence()
        self.assertTrue(ev["ok"], ev["reason"])

    def test_a_loaded_carriage_is_rejected(self):
        """★ 最要命的那条：载货绝不能被当成空车基线。

        实测载货的 z 厚度 0.63~1.16 m、底面离地 2.62~3.31 m
        （货把底板盖住了，只剩一层货面）。
        """
        ev = self._evidence(self._carriage(thickness=0.8, floor_above=2.9))
        self.assertFalse(ev["ok"])
        self.assertFalse(self._names(ev)["底板在整个车厢内露出"])
        self.assertEqual(0.0, ev["profile"]["bed_exposure"],
                         "货把底板盖住了，逐格最低回波不该落在底板高度带里")

    def test_implausible_floor_height_is_rejected_even_when_thick(self):
        """★ 底面必须是一个**带**，不能只设上界。

        2026-09-13 ROI/yaw 配错那天的数据：厚度 2.38（够）、
        底面离地 **0.22 m**（自卸车底板离地 1.2~1.5，物理上不可能），
        而它的 meta 理由恰恰写着"空车" —— 只设上界会直接中招。
        """
        ev = self._evidence(self._carriage(thickness=2.7, floor_above=0.22))
        self.assertFalse(ev["ok"])
        self.assertFalse(self._names(ev)["底板在整个车厢内露出"])
        # ▲ 它的低面反而**比真空车还平**（实测残差 0.013 对 0.021/0.025）——
        #   因为那是一片被误切进来的地面。平面度这一条拦不住它。
        self.assertTrue(self._names(ev)["低面确实是个平面"],
                        "这一份本来就是平的；靠底板露出那条拦，不是靠平面度")

    def test_single_frame_cannot_authorise_a_baseline(self):
        """单帧判不了重复性 —— 场景静不静止是必须回答的问题。"""
        ev = self._evidence(frames=1)
        self.assertFalse(ev["ok"])
        self.assertFalse(self._names(ev)["多帧场景静止"])

    def test_moving_scene_is_rejected(self):
        """实测 2026-09-15 两次空车采集 r 只有 0.799/0.865（车还在动），
        拿它建基线体积会一直偏约 4% 且事后看不出来。"""
        from app.detect import empty_evidence
        cloud = self._carriage()
        moved = cloud + np.array([0., 2.5, 0.])        # 整体挪 2.5 m = 还在走
        ev = empty_evidence(cloud, -5.2, clouds=[cloud, moved])
        self.assertFalse(ev["ok"])
        self.assertFalse(self._names(ev)["多帧场景静止"])

    def test_a_different_carriage_cannot_replace_the_baseline(self):
        """★ 牵引车换了挂车、车牌没变 —— 不能把新车厢悄悄当成同一辆车的基线。"""
        from app.detect import EmptyBaseline
        old = self._carriage(width=2.45, length=11.0)
        base = EmptyBaseline(floor_z=-3.9, box=(-1.3, 1.3, -5.5, 5.5, -3.9, -1.5),
                             cloud=old.astype(np.float32), n=len(old))
        same = self._evidence(self._carriage(width=2.45, length=11.0, seed=9), baseline=base)
        self.assertTrue(same["ok"], same["reason"])
        other = self._evidence(self._carriage(width=2.20, length=6.5, seed=9), baseline=base)
        self.assertFalse(other["ok"])
        self.assertFalse(self._names(other)["与现有基线同一车厢"])

    def test_baseline_may_only_get_emptier(self):
        """★ 单调性：新候选比旧基线更满就不许替换。

        这条是真正的安全网 —— 哪怕空车判据出错，只要新的比旧的满就拦住。
        """
        from app.detect import EmptyBaseline
        old = self._carriage()
        base = EmptyBaseline(floor_z=-3.9, box=(-1.3, 1.3, -5.5, 5.5, -3.9, -1.3),
                             cloud=old.astype(np.float32), n=len(old))
        ev = self._evidence(self._half_loaded(), baseline=base)
        self.assertFalse(ev["ok"])
        self.assertFalse(self._names(ev)["不比现有基线更满"])

    def test_first_baseline_has_nothing_to_compare_against(self):
        ev = self._evidence(baseline=None)
        self.assertTrue(ev["ok"], ev["reason"])
        note = [g["note"] for g in ev["gates"] if g["name"] == "与现有基线一致"]
        self.assertTrue(any("first_baseline" in n for n in note), note)

    def test_degraded_or_duplicate_is_rejected(self):
        self.assertFalse(self._evidence(degraded=True)["ok"])
        self.assertFalse(self._evidence(duplicate=True)["ok"])

    def test_uncuttable_cloud_reports_instead_of_guessing(self):
        rng = np.random.default_rng(5)
        flat = np.column_stack((rng.uniform(-6, 6, 8000), rng.uniform(-6, 6, 8000),
                                np.full(8000, -5.2)))
        ev = self._evidence(flat)
        self.assertFalse(ev["ok"])
        self.assertIsNone(ev["profile"])
        self.assertIn("切不出车厢", ev["reason"])

    def test_every_gate_reports_value_and_limit_not_just_pass_fail(self):
        """后台要能看出"差多少"，也要能在攒够真值后回头复核这些常数。"""
        ev = self._evidence(self._carriage(thickness=0.8, floor_above=2.9))
        for gate in ev["gates"]:
            self.assertIn("name", gate)
            self.assertIn("passed", gate)
            self.assertIn("value", gate)
            self.assertIn("limit", gate)
        from app.detect import EMPTY_BED_EXPOSURE_MIN
        bed = next(g for g in ev["gates"] if g["name"] == "底板在整个车厢内露出")
        self.assertEqual(EMPTY_BED_EXPOSURE_MIN, bed["limit"])
        self.assertLess(bed["value"], EMPTY_BED_EXPOSURE_MIN)

    def test_payload_shaping_drops_non_finite(self):
        """上报形状：NaN 必须变 null，不能混进 JSON。"""
        import json as _json
        from app.load_radar_contract import baseline_candidate_payload
        ev = self._evidence()
        ev["gates"].append({"name": "假的", "passed": True, "value": float("nan"),
                            "limit": float("inf"), "note": ""})
        payload = baseline_candidate_payload(ev)
        _json.dumps(payload, allow_nan=False)
        bad = next(g for g in payload["gates"] if g["name"] == "假的")
        self.assertIsNone(bad["value"])
        self.assertIsNone(bad["limit"])

    def test_payload_is_not_wired_into_the_measurement_yet(self):
        """★ 钉住一条会全局打挂的事：`_same_measurement()` 是遍历 payload
        每一个键要求后台回执逐一相等的，而后台是强类型记录、不回显未知字段。
        往载荷里加字段 -> 比对必失败 -> 每一次上报都被 mark_blocked。
        后台加上属性并回显之前，这个字段不许出现在载荷里。"""
        from app.load_radar_contract import build_measurement
        source = {"channel": "CH1", "ts": 1704067200.0, "empty": False, "reason": "载货",
                  "frame_degraded": False, "n_points": 60000, "cloud": [[1., 2., -3.]],
                  "volume": {}, "diagnostics": {"step": 7}, "step": 7,
                  "empty_evidence": self._evidence()}
        payload = build_measurement(source, measurement_id="m-1", node_id="n-1",
            passage_id="p-1", calibration_id="c-1", algorithm_version="v1",
            configuration_hash="a" * 64, source_mode="live")
        self.assertNotIn("baselineCandidate", payload,
                         "载荷里出现了后台不认识的字段 —— 会把整条上报链打挂")
    def test_partial_load_is_rejected(self):
        """半装载既不是空车也不是满车，不能拿去当基线。

        现场实测三份（20260914_165712 / 171018、20260915_113311）：
        总露出 11.9% / 14.4% / 13.0%，剖面都是"尾段露底、往前全被盖住"。

        ▲ **不要声称"旧的 z 厚度判据漏掉了这三份"** —— 实测它也拦得住
          （厚度 1.59~1.81 < 2.2），因为这几车的货顶低于壁顶、
          厚度跟着降了。露出判据真正被证实的好处是另外两条：
          模糊带从 35% 降到 3.2%（见类文档），以及剖面能指出货在哪一段。
        """
        ev = self._evidence(self._half_loaded())
        self.assertFalse(ev["ok"])
        self.assertFalse(self._names(ev)["底板在整个车厢内露出"])
        exposure = ev["profile"]["bed_exposure"]
        self.assertLess(exposure, 0.9)
        self.assertGreater(exposure, 0.0, "尾段应当还露着底")

    def test_cargo_heaped_to_the_wall_top_still_fails_exposure(self):
        """★ 露出判据**不依赖货顶有多高**，而厚度判据依赖。

        造一辆货堆到壁顶的半装载：这时 z 厚度会回到空车的读数
        （p1 是尾段露出的底板、p99 是货顶≈壁顶），厚度判据放行；
        而底板露出仍然只有尾段那一点。
        ▲ 现场样本里**没有**这样一份，所以这是合成场景 —— 它证明的是
          两条判据的**依赖关系不同**，不是"现场发生过"。
        """
        from app.detect import EMPTY_THICKNESS_MIN
        heaped = self._half_loaded(cargo_top_m=2.6)
        ev = self._evidence(heaped)
        self.assertGreaterEqual(ev["profile"]["thickness_m"], EMPTY_THICKNESS_MIN,
                                "合成场景没造对：厚度应当回到空车读数")
        self.assertFalse(ev["ok"])
        self.assertFalse(self._names(ev)["底板在整个车厢内露出"])

    def test_exposure_profile_shows_where_the_cargo_sits(self):
        """剖面要能指出货堆在哪一段 —— 总比例看不出这件事。"""
        ev = self._evidence(self._half_loaded(covered=0.6))
        prof = [p for p in ev["profile"]["bed_exposure_profile"] if p is not None]
        self.assertEqual(8, len(ev["profile"]["bed_exposure_profile"]))
        self.assertGreater(prof[0], 0.8, "尾段应当基本露底")
        self.assertLess(prof[-1], 0.2, "前段应当被货盖住")


class TestVerdictAndDiagnosis(unittest.TestCase):
    """★ 「没看清」与「看清了、有货」必须分得开（2026-09-19）。

    判定链第 1~10 步**全部** `return False`，于是这两件事压成了同一个答案，
    现场无从判断该**重测**还是该**拦截**。实测 196 帧里判 False 的 186 帧中，
    真正证据不足只有 **12 帧（6.5%）**。

    另外两条同批查实的（都只改诊断，不动判定）：
    · `z_above` 以前写在第 10 步的分支里 -> 在第 6/7 步被拦的帧上**永远是 NaN**
      （实测第 7 步 161/161 全 NaN），而补算出来中位 **0.377 m**、门槛 0.08，
      **99% 的帧在第 10 步照样会被拦** —— 现场因此会误以为"放宽占比能救空车"；
    · 第 7 步等效要求约 15000 个内点，第 6 步只要求 600，**差约 25 倍**，
      第 6 步实际被架空。不摆出来，现场会去调错的那一个。
    """

    def sample(self):
        """取一份真实基线（它们停在第 7 步）。"""
        _, _, calib, dev, _ = H.ctx()
        import glob
        path = sorted(glob.glob(os.path.join(H.ROOT, *BASELINE_SUBDIR, "**", "*.xyz"),
                                recursive=True))[0]
        frame = apply_roi(load_xyz(path), calib.roi)
        res = detect_empty(frame, calib)
        ok, reason, dg = evaluate_empty(res, frame.n,
                                        res.cloud if res else frame.xyz, dev)
        return ok, reason, dg, dev

    def test_verdict_is_derived_from_step(self):
        from app.detect import Diagnostics, NO_EVIDENCE_STEPS
        self.assertEqual((1, 2, 3, 4, 5), NO_EVIDENCE_STEPS,
                         "必须与 load_radar_contract 的 early_failure 取同一集合")
        for step, want in ((0, ""), (1, "NO_EVIDENCE"), (3, "NO_EVIDENCE"),
                           (4, "NO_EVIDENCE"), (5, "NO_EVIDENCE"),
                           (6, "REJECT_LOADED"), (7, "REJECT_LOADED"),
                           (10, "REJECT_LOADED"), (11, "PASS_EMPTY")):
            self.assertEqual(want, Diagnostics(step=step).verdict, "第 %d 步" % step)

    def test_z_above_is_computed_even_when_blocked_early(self):
        ok, reason, dg, dev = self.sample()
        self.assertFalse(ok)
        self.assertEqual(7, dg.step, "这份基线应当停在第 7 步")
        self.assertEqual("REJECT_LOADED", dg.verdict)
        self.assertEqual(dg.z_above_used, dg.z_above_used, "z_above 不该还是 NaN")
        self.assertGreater(dg.z_above_used, dev.gate.z_above_min_max_m,
                           "这份即便放宽第 7 步，也会卡在第 10 步 —— "
                           "这正是必须把它摆出来的原因")

    def test_reason_detail_carries_the_physical_quantities(self):
        _ok, _reason, dg, dev = self.sample()
        text = reason_detail(dg, dev)
        self.assertIn("第 7 步", text)
        self.assertIn("余高", text)
        self.assertIn("z_above", text)
        self.assertIn("第 6 步只要求", text, "必须把两步的严格度摆在一起")

    def test_gate_report_exposes_both_inlier_gates_in_one_unit(self):
        _ok, _reason, dg, dev = self.sample()
        rows = {r["step"]: r for r in gate_report(dg, dev)}
        self.assertEqual(dg.inliers, rows[6]["inliers"])
        self.assertEqual(dg.inliers, rows[7]["inliers"])
        self.assertGreater(rows[7]["equiv_inliers"], 10 * rows[6]["limit"],
                           "第 7 步等效门槛应当远高于第 6 步，这是它独自把关的证据")

    def test_reason_string_is_untouched(self):
        """reason 与原厂逐字一致，是 verify.py 的全部效力所在 —— 一个字都不许动。"""
        _ok, reason, _dg, _dev = self.sample()
        self.assertEqual("底面内点占比过低", reason)


class TestEchoModeSensitivity(unittest.TestCase):
    """★ 第 7 步的门槛量纲随**回波模式**漂移（2026-09-19 钉住）。

    `floor_gate_min_inlier_ratio = 0.15` 是照**单回波**点数（ROI 后 5~6 万）
    标定的。现场是双回波，ROI 后 11~13 万点 —— 第二回波多来自**别的面**
    （不落在厢底平面上），于是分母涨、分子不涨，**同一块底板的占比被腰斩**。

    实测后果：196 帧里内点**绝对数**达标 94%，内点**占比**达标只有 12%。

    ▲ 这条用例**不主张哪个门槛对**，只钉住"两者对点数的敏感性根本不同"这个事实 ——
      将来若把第 7 步改成绝对数判据（`docs/41` §0e），它应当仍然绿。
    """

    def sample(self):
        import glob
        _, _, calib, dev, _ = H.ctx()
        path = sorted(glob.glob(os.path.join(H.ROOT, *BASELINE_SUBDIR, "**", "*.xyz"),
                                recursive=True))[0]
        frame = apply_roi(load_xyz(path), calib.roi)
        res = detect_empty(frame, calib)
        self.assertIsNotNone(res, "夹具样本应当能拟合出六面")
        return frame, res, dev

    def second_echo(self, cloud, rng):
        """造第二回波：与首回波同样多，但**沿 z 偏开**，不落在厢底平面上。

        真实双回波的第二个返回来自更远的面（货堆内部缝隙、厢壁、地面），
        所以它加的是"不在底面上的点"。这里偏 0.5~2.0 m，远超
        `floor_gate_distance_m = 0.015` 的内点带宽。
        """
        extra = cloud.copy()
        extra[:, 2] += rng.uniform(0.5, 2.0, len(extra)).astype(cloud.dtype)
        return np.vstack([cloud, extra])

    def test_ratio_halves_while_absolute_inliers_hold(self):
        frame, res, dev = self.sample()
        cloud = res.cloud
        _ok1, _r1, one = evaluate_empty(res, frame.n, cloud, dev)
        rng = np.random.default_rng(7)
        doubled = self.second_echo(cloud, rng)
        _ok2, _r2, two = evaluate_empty(res, frame.n * 2, doubled, dev)

        self.assertAlmostEqual(one.inliers, two.inliers, delta=max(5, one.inliers * 0.01),
                               msg="第二回波不落在底面上，内点绝对数不该变")
        self.assertAlmostEqual(one.ratio / 2.0, two.ratio, delta=one.ratio * 0.05,
                               msg="分母翻倍、分子不变 -> 占比应当腰斩")

    def test_the_two_gates_disagree_under_double_echo(self):
        """同一块底板：绝对数判据说"够"，占比判据说"不够"。"""
        frame, res, dev = self.sample()
        rng = np.random.default_rng(7)
        doubled = self.second_echo(res.cloud, rng)
        _ok, _reason, dg = evaluate_empty(res, frame.n * 2, doubled, dev)
        gate = dev.gate
        self.assertGreaterEqual(dg.inliers, gate.min_inliers,
                                "绝对内点数仍然达标 —— 底板没有变少")
        self.assertLess(dg.ratio, gate.min_inlier_ratio,
                        "而占比不达标 —— 差别全来自那个与空满无关的分母")
        # 等效门槛：第 7 步折算成绝对内点数，应当远高于第 6 步
        self.assertGreater(dg.ratio_equiv_inliers, 10 * gate.min_inliers)

class TestStepTenSelfReferentialFallback(unittest.TestCase):
    """★★ 第 10 步的自参考兜底：底板平整度 + 车厢 z 厚度。

    为什么不用绝对高度：`bed_exposure` 靠"最低回波落不落在离地 [0.9, 1.9]"判，
    而**每辆车的车厢高度不固定** —— 62 份实测里已有 7 份载货的最低面落进带内
    （1.828~1.9），上界压在载货群里；带太窄又会漏掉高底盘的空车。

    改用两条这辆车自己量出来的量（62 份全量都零重叠）：
        ① 底板平整度 空车 0.0189~0.0244，其余 58 份最小 0.0421
        ② 车厢 z 厚度 空车 2.423~2.957，其余 58 份最大 1.692

    四条边界必须钉住：
      · 0 = 关闭，行为逐位不变；
      · 只在 plane 档生效（origin 档要与原厂逐字一致）；
      · 拿不到 ground_z 不放行；
      · **两条必须同时满足** —— 少任何一条都有洞。
    """

    ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    EMPTY = ("110959", "111107")
    LOADED = ("152838", "153502")

    def _verdict(self, stamp, flat=0.032, thick=2.2, mode="plane", pass_ground=True):
        import copy
        from app.settings import load_settings, load_calib, load_devices
        from app.pointcloud import apply_roi, load_xyz
        from app.detect import detect_empty, evaluate_empty
        path = os.path.join(self.ROOT, "data", "raw-62", "CH4",
                            "CH4_20260915_%s_3of3.xyz" % stamp)
        if not os.path.isfile(path):
            self.skipTest("没有 data/raw-62 现场样本")
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH4"][0]
        calib = load_calib(ch.abs_config_path)
        dev = copy.deepcopy(load_devices(ch.abs_devices_path))
        dev.gate.bed_flatness_max = flat
        dev.gate.bed_thickness_min = thick
        dev.gate.bed_exposure_min = 0.0
        dev.gate.z_above_mode = mode
        fr = apply_roi(load_xyz(path), calib.roi)
        res = detect_empty(fr, calib)
        gz = calib.detection.ground_z if pass_ground else None
        _ok, _reason, dg = evaluate_empty(res, fr.n, res.cloud, dev, ground_z=gz)
        return dg

    def test_disabled_keeps_the_old_verdict(self):
        """两条都给 0 = 关闭：仍停在第 10 步，与从前逐位一致。"""
        for stamp in self.EMPTY:
            dg = self._verdict(stamp, flat=0.0, thick=0.0)
            self.assertEqual(dg.step, 10, stamp + " 关闭时应仍停在第 10 步")

    def test_enabled_rescues_the_empty_truths(self):
        """开启后：真空车走到第 11 步，且两条证据都算出来了。"""
        for stamp in self.EMPTY:
            dg = self._verdict(stamp)
            self.assertEqual(dg.step, 11, stamp + " 是空车，应判到第 11 步")
            self.assertLessEqual(dg.bed_flatness, 0.032)
            self.assertGreaterEqual(dg.bed_thickness, 2.2)

    def test_loaded_is_never_rescued(self):
        """载货在第 7 步就被拦下，根本走不到第 10 步。"""
        for stamp in self.LOADED:
            self.assertEqual(self._verdict(stamp).step, 7)

    def test_origin_mode_is_untouched(self):
        """★ origin 档必须与原厂逐字一致 —— 兜底在这一档不生效。"""
        for stamp in self.EMPTY:
            self.assertEqual(self._verdict(stamp, mode="origin").step, 10)

    def test_without_ground_z_it_does_not_pass(self):
        """拿不到 ground_z 不放行 —— 宁可漏判成载货。"""
        for stamp in self.EMPTY:
            self.assertEqual(self._verdict(stamp, pass_ground=False).step, 10)

    def test_both_conditions_are_required(self):
        """▲▲ 两条缺一不可。

        只留平整度：铺平的非煤货物（砂石/板材）货顶也很平 —— 合成载货场景的
        货顶就是完美平面，残差约 0，光看平整度必放行。
        只留厚度：124 份全量实测厚度是连续谱，1.2~2.4 之间压着 43 份模糊带。
        """
        for stamp in self.EMPTY:                  # 真空车两条都过，单开也该过
            self.assertEqual(self._verdict(stamp, thick=0.0).step, 11)
            self.assertEqual(self._verdict(stamp, flat=0.0).step, 11)

    def test_thin_flat_load_is_caught_by_thickness(self):
        """薄而平的载货：平整度骗得过，**厚度拦得住** —— 这就是要两条的原因。

        底板 1.2 m 的车铺 0.3 m 平料，货面 1.5 m：
          · 货面很平 -> 平整度过得去；
          · 但壁顶到货面只剩约 1.6 m -> 厚度拦下（门槛 2.2）。
        而"把载货判成空车"是最危险的方向（会被存成空车基线，之后体积恒为
        接近 0 且不报错）。
        """
        import numpy as np
        from app.detect import carriage_profile
        from app.settings import load_settings, load_calib
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH4"][0]
        gz = load_calib(ch.abs_config_path).detection.ground_z
        rng = np.random.default_rng(7)
        n = 40000
        cloud = np.vstack([
            np.stack([rng.uniform(-1.1, 1.1, n), rng.uniform(-4., 4., n),
                      gz + 1.5 + rng.normal(0, .005, n)], 1),          # 平整货面
            np.stack([np.where(rng.random(6000) < .5, -1.2, 1.2),
                      rng.uniform(-4., 4., 6000),
                      gz + rng.uniform(1.5, 3.1, 6000)], 1)])          # 厢壁到 3.1
        prof = carriage_profile(cloud, gz)
        if prof is None:
            self.skipTest("合成云切不出车厢")
        self.assertLess(prof["bed_plane_rms_m"], 0.032, "平整的货面骗得过平整度")
        self.assertLess(prof["thickness_m"], 2.2, "但厚度拦得住它")


class TestCargoTopAndLowSeverityFixes(unittest.TestCase):
    """★ 2026-09-23 修复批次：_cargo_top 倾角筛选 + gate_report step0 + fill_holes 转发。"""

    def _cfg(self):
        # config.json 的 detect_empty 段被 _fill 摊平进 CalibCfg 顶层
        #（heading/floor/roof/wall 都是它的直接字段）。
        _, _, calib, _, _ = H.ctx()
        return calib

    def test_cargo_top_rejects_vertical_walls(self):
        """★ 货顶必须是**够水平**的面（两轮审查 M2）。

        RANSAC 只按内点数选面 —— y 中段的左右壁是贯穿整段的竖直强平面，
        起伏的货面大概率争不过它；胜出的"货顶"其实是厢壁，近竖直面
        （|n_z|≈0）的竖向间距会算出 NaN 并原样进 JSON。
        """
        from app.detect import _cargo_top
        import numpy as np
        rng = np.random.default_rng(11)
        wall = []
        for x_at in (-1.2, 1.2):                    # 两片竖直厢壁，无水平顶面
            wall.append(np.stack([np.full(8000, x_at),
                                  rng.uniform(-6, 6, 8000),
                                  rng.uniform(-4.2, -2, 8000)], 1))
        xyz = np.concatenate(wall).astype(np.float32)
        p = _cargo_top(xyz, self._cfg())
        self.assertIsNone(p, "竖直墙不该冒充货顶 —— 修复前它以最大内点面胜出")
        # 对照：顶面点数占优时必须仍能找到货顶（倾角筛选不得误伤真货顶）
        roof = np.stack([rng.uniform(-1.1, 1.1, 12000),
                         rng.uniform(-6, 6, 12000),
                         np.full(12000, -3.4)], 1)
        thin = np.concatenate(wall[:1] + [np.stack([np.full(2000, 1.2),
                          rng.uniform(-6, 6, 2000),
                          rng.uniform(-4.2, -2, 2000)], 1)]).astype(np.float32)
        p2 = _cargo_top(np.concatenate([thin, roof]).astype(np.float32), self._cfg())
        self.assertIsNotNone(p2, "水平货面占优，就该找到货顶")
        self.assertGreater(abs(float(p2.normal()[2])), 0.9)

    def test_gate_report_marks_step_zero_as_unknown(self):
        """step==0（没走到任何一步）不得展示成"全部通过"。"""
        from app.detect import Diagnostics, gate_report
        _, _, _, dev, _ = H.ctx()
        rows = gate_report(Diagnostics(step=0), dev)
        self.assertTrue(rows)
        for r in rows:
            self.assertEqual("unknown", r["state"], "第 %d 步被误标成 pass" % r["step"])

    def test_volume_auto_accepts_fill_holes_override(self):
        """**kw 里的 fill_holes 不再撞 TypeError（低严重度批次）。"""
        from app.detect import compute_volume_auto
        result = compute_volume_auto(None, None, cells=(0.2,), fill_holes=False)
        self.assertFalse(result.estimate_ok()[0], "无基线本就不该给出有效估值")
