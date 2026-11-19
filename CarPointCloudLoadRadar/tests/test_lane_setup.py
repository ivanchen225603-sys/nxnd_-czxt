#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_lane_setup.py — 空场标定（cli/lane_setup.py）

这个工具解决的是 2026-09-13 那次事故的根因：
`detection.ground_z` 是**写死在配置里的绝对高度**，而有车判据全靠它。
现场几何只要对不上，地面就会整片跑到判据线之上 →
每帧都判「有车」→ 会话永不结束 → **真来的车全部被静默跳过**，且一路不报错。

空场（4 个车道都没有车）是唯一能可靠测出"地面在哪"的时刻 ——
车一停进来就把地面挡住了（10 份原版样本里有 4 份连一个地面点都没有）。

用例覆盖三件事：
    ① 能不能从空场点云里把地面量准（含倾斜地面）；
    ② 量纲错了能不能**指名道姓**地报出来（这是工具存在的全部意义）；
    ③ ROI 建议是不是跟着实测地面走，而不是抄死值。
"""
from __future__ import annotations

import io
import math
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

from tests import _helpers as H                # noqa: F401  （把项目根加进 sys.path）
from app.settings import root, load_calib
from app.pointcloud import Frame

sys.path.insert(0, root("cli"))
import lane_setup                               # noqa: E402


def _ground_cloud(depth=5.0, n=40000, tilt_deg=0.0, noise=0.005, seed=0,
                  radius=12.0):
    """造一片"雷达吊在 depth 米高、俯视一块地面"的点云。

    极坐标均匀撒点再转笛卡尔 —— 这样点密度和真雷达一样是**中间密、四周疏**，
    否则直方图找峰会失真。
    """
    rng = np.random.default_rng(seed)
    rho = radius * np.sqrt(rng.uniform(0.0004, 1.0, n))   # 面积均匀
    th = rng.uniform(0, 2 * np.pi, n)
    x, y = rho * np.cos(th), rho * np.sin(th)
    z = np.full(n, -depth) + rng.normal(0, noise, n)
    if tilt_deg:
        z = z + y * math.tan(math.radians(tilt_deg))
    return np.stack([x, y, z], 1).astype(np.float32)


class TestEstimateGround(unittest.TestCase):

    def test_flat_ground(self):
        g = lane_setup.estimate_ground(_ground_cloud(depth=5.0))
        self.assertTrue(g["ok"], g.get("reason"))
        self.assertAlmostEqual(g["ground_z"], -5.0, delta=0.02)
        self.assertLess(g["tilt_deg"], 0.5)
        self.assertLess(g["rms_m"], 0.02)

    def test_other_mount_height(self):
        """换个吊装高度也要量得准 —— 不能对 -5.0 有任何偏好。"""
        for d in (3.2, 4.5, 6.8):
            g = lane_setup.estimate_ground(_ground_cloud(depth=d, seed=int(d * 10)))
            self.assertTrue(g["ok"])
            self.assertAlmostEqual(g["ground_z"], -d, delta=0.03,
                                   msg=f"吊装 {d} m 时量成了 {g['ground_z']}")

    def test_tilted_ground_is_reported(self):
        """地面有坡度要如实报出来 —— 底面拟合的 |法向z|≥0.99 门槛会受它影响。"""
        g = lane_setup.estimate_ground(_ground_cloud(tilt_deg=4.0, seed=7))
        self.assertTrue(g["ok"])
        self.assertGreater(g["tilt_deg"], 2.0, "4° 的坡当成平地了")

    def test_too_few_points_is_refused(self):
        g = lane_setup.estimate_ground(np.zeros((10, 3), np.float32))
        self.assertFalse(g["ok"])
        self.assertIn("点太少", g["reason"])

    def test_nothing_under_the_radar_is_refused(self):
        """正下方没点（量纲缩了 / 雷达没对着地面）必须拒绝，不能瞎给一个值。"""
        far = _ground_cloud(depth=5.0, radius=30.0, n=3000)
        far = far[np.hypot(far[:, 0], far[:, 1]) > 10.0]      # 把近处掏空
        g = lane_setup.estimate_ground(far, radius=3.0)
        self.assertFalse(g["ok"])
        self.assertIn("正下方", g["reason"])


class TestScaleCheck(unittest.TestCase):
    """★ 工具存在的全部意义：量纲错了要指名道姓。"""

    def setUp(self):
        self.proc = load_calib(root("config", "config.json")).processing

    def test_correct_scale_passes(self):
        r = lane_setup.check_scale(-5.0, 5.0, self.proc)
        self.assertEqual(r["verdict"], "ok")

    def test_four_times_off_is_caught_and_named(self):
        """2026-09-13 的原样复现：实测地面 -1.25，实际吊装 5 m。"""
        r = lane_setup.check_scale(-1.25, 5.0, self.proc)
        self.assertEqual(r["verdict"], "bad")
        self.assertAlmostEqual(r["ratio"], 4.0, delta=0.05)
        self.assertIn("range_mode", r["message"])
        self.assertIn("raw_offset", r["message"],
                      "要直接点出是哪个值配错了，不能只说『量纲不对』")
        self.assertIn("不可信", r["message"])

    def test_inverse_error_is_named_too(self):
        """反过来配错（scale 当成 raw_offset）也要认得出。"""
        r = lane_setup.check_scale(-20.0, 5.0, self.proc)
        self.assertEqual(r["verdict"], "bad")
        self.assertIn("scale", r["message"])

    def test_small_deviation_is_tolerated(self):
        """吊装高度是人拿卷尺量的，±10% 不该报错。"""
        for h in (4.6, 5.4):
            self.assertEqual(lane_setup.check_scale(-5.0, h, self.proc)["verdict"], "ok")

    def test_no_reference_at_all_means_no_verdict(self):
        """两个参照都没有就别装作能判 —— 宁可说"没校验"。"""
        self.assertFalse(lane_setup.check_scale(-5.0, 0.0, self.proc).get("checked"))

    # ── 没有卷尺时走的那条路：拿原版现场口径当参照 ──────────────
    def test_config_ground_z_works_as_reference(self):
        """★ 现场量不到雷达吊装高度时，用 config 的 ground_z 当参照。

        它是原厂系统在**同一个现场**用的值，10 份历史样本与它自洽
        （verify.py 10/10 就建立在它之上）。
        """
        r = lane_setup.check_scale(-5.0, 0.0, self.proc, ref_depth=5.0)
        self.assertTrue(r["checked"])
        self.assertEqual(r["verdict"], "ok")
        self.assertIn("原版现场口径", r["reference"])

    def test_config_reference_catches_the_4x_error(self):
        """9-13 那次：地面测出来 -1.25，而原版口径是 5.0 —— 不用卷尺也该抓住。"""
        r = lane_setup.check_scale(-1.25, 0.0, self.proc, ref_depth=5.0)
        self.assertEqual(r["verdict"], "bad")
        self.assertAlmostEqual(r["ratio"], 4.0, delta=0.05)
        self.assertIn("raw_offset", r["message"])
        self.assertIn("--mount-height", r["message"],
                      "用的是间接参照，就得提醒一句『雷达若重新装过要用实测值』")

    def test_two_references_corroborate(self):
        """★ 实测 5.095 m 与原版口径 5.0 m 相差 1.9% —— 应当报"互相印证"。

        这条印证本身是有价值的结论：说明雷达还在原来的高度上没被动过，
        于是"拿 10 份历史样本当绝对参照"这条零测量路径的前提就成立了。
        """
        r = lane_setup.check_scale(-5.09, 5.095, self.proc, ref_depth=5.0)
        self.assertEqual(r["verdict"], "ok")
        self.assertIn("互相印证", r["message"])
        self.assertIn("历史样本可当绝对参照", r["message"])

    def test_two_references_disagreeing_is_flagged(self):
        """两个参照打架 = 雷达被重新装过 —— 必须说清楚历史样本此时不能用了。"""
        r = lane_setup.check_scale(-3.2, 3.2, self.proc, ref_depth=5.0)
        self.assertEqual(r["verdict"], "ok", "以实测为准，量纲本身是对的")
        self.assertIn("重新装过", r["message"])
        self.assertIn("不能再当绝对参照", r["message"])

    def test_mount_height_wins_over_config(self):
        """给了实测吊装高度就以它为准 —— 雷达重新装过时 config 的值已经失效。"""
        r = lane_setup.check_scale(-3.0, 3.0, self.proc, ref_depth=5.0)
        self.assertEqual(r["verdict"], "ok", "实测 3m、测出 3m，该判通过")
        self.assertIn("实测", r["reference"])


class TestRoiSuggestion(unittest.TestCase):

    def test_z_follows_measured_ground(self):
        """ROI 的 z 下界必须跟着**实测**地面走，不能抄死值。"""
        for gz in (-5.0, -3.4, -6.2):
            roi = lane_setup.suggest_roi(gz, 4.0, (-11.0, 8.0), None)
            self.assertAlmostEqual(roi["z"][0], gz - 1.0, places=2)
            self.assertEqual(roi["z"][1], -0.75, "上界是雷达自身的安全距离，不该动")

    def test_x_follows_lane_width(self):
        roi = lane_setup.suggest_roi(-5.0, 3.0, (-11.0, 8.0), None)
        self.assertEqual(roi["x"], [-1.5, 1.5])

    def test_matches_factory_roi_on_factory_geometry(self):
        """★ 用原厂的几何（地面 -5.0、车道 4m）应当**复现原厂 ROI**。

        这是对整套推导的交叉验证：10 份原版样本的实际占用范围正好顶到
        x[-2,2] / y[-11,8] / z[-6,-0.75]，说明那就是原厂现场用的 ROI。
        """
        roi = lane_setup.suggest_roi(-5.0, 4.0, (-11.0, 8.0), None)
        self.assertEqual(roi["x"], [-2.0, 2.0])
        self.assertEqual(roi["y"], [-11.0, 8.0])
        self.assertEqual(roi["z"], [-6.0, -0.75])

    def test_every_edge_has_a_reason(self):
        roi = lane_setup.suggest_roi(-5.0, 4.0, (-11.0, 8.0), None)
        for axis in ("x", "y", "z"):
            self.assertGreater(len(roi["why"][axis]), 10,
                               f"{axis} 轴的取值依据没写清楚")


def _lane_scene(ground_z, wall_x=3.4, n_ground=40000, n_wall=6000, seed=1):
    """造一条「沿 y 的车道」：满地地面 + 两侧各一道墙。

    朝向正确时，ROI（x[-2,2] × y[-11,8]）应当只框到地面，墙在框外。
    """
    r = np.random.default_rng(seed)
    g = np.stack([r.uniform(-12, 12, n_ground), r.uniform(-14, 14, n_ground),
                  np.full(n_ground, ground_z)], 1)
    w = []
    for sx in (-1, 1):
        w.append(np.stack([
            sx * r.uniform(wall_x, wall_x + 0.6, n_wall),
            r.uniform(-13, 11, n_wall),
            r.uniform(ground_z + 0.8, ground_z + 3.5, n_wall)], 1))
    return np.concatenate([g] + w).astype(np.float64)


def _spin(xyz, deg):
    """按 check_orientation 的同一套约定把场景转 deg 度。"""
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    out = xyz.copy()
    out[:, 0] = xyz[:, 0] * c + xyz[:, 1] * s
    out[:, 1] = -xyz[:, 0] * s + xyz[:, 1] * c
    return out


class TestCheckOrientation(unittest.TestCase):
    """ROI 的长边有没有压在行车方向上 —— cli/verify.py 证明不了的那一项。

    10 份历史样本是原版**已经解码完成**的 .xyz，重跑判定链不经过角度换算，
    所以朝向错成什么样回归都是 10/10。这组用例是这一项唯一的自动防线。

    2026-09-14 现场：原厂的 yaw_offset_deg=-93° 把 ROI 的 19 m 长边
    横切在车道上、压住两侧的墙 —— 空车道被判成有车（9 878 / 10 320 / 2 892 点）。
    改成 -183°（正好 90°）后降到 2 / 0 / 621。
    """

    def setUp(self):
        self.calib = load_calib(root("config", "config.json"))
        self.gz = self.calib.detection.ground_z

    def test_correct_orientation_sees_only_ground(self):
        """朝向对的时候，ROI 里高于地面的点应当≈0，且框内点数不塌。"""
        o = lane_setup.check_orientation(_lane_scene(self.gz), self.calib)
        self.assertTrue(o["ok"])
        self.assertLessEqual(o["current_above"], lane_setup.ORIENT_TOL)
        self.assertGreater(o["current_in"], 1000, "框内点数塌了，说明场景没造对")

    def test_ninety_degrees_off_is_caught(self):
        """转 90°（ROI 横切车道）必须被抓出来 —— 这正是现场那个 bug。"""
        o = lane_setup.check_orientation(_spin(_lane_scene(self.gz), 90.0),
                                         self.calib)
        self.assertGreater(
            o["current_above"], lane_setup.ORIENT_TOL * 5,
            "ROI 横切车道、压在墙上，却没被判成朝向不对")

    def test_plateau_points_back_at_the_right_axis(self):
        """场景转了 φ，合格平台的中心就该挪到 base - φ —— **模 180°**。

        见 test_180_degrees_is_inherently_ambiguous：空场只能定轴向，定不了朝向。
        """
        base = float(self.calib.processing.yaw_offset_deg)
        for phi in (-40.0, -90.0, 25.0):
            o = lane_setup.check_orientation(_spin(_lane_scene(self.gz), phi),
                                             self.calib, span_deg=200.0)
            self.assertIsNotNone(o["plateau"], f"φ={phi} 时找不到平台")
            err = abs((o["center"] - (base - phi) + 90.0) % 180.0 - 90.0)
            self.assertLessEqual(
                err, 6.0,
                f"φ={phi}: 平台中心 {o['center']} 与 {base - phi} 差了 {err:.0f}°"
                f"（已按模 180° 计）")

    def test_180_degrees_is_inherently_ambiguous(self):
        """★ 空场定得出车道的**轴向**，定不出**朝哪头**。

        一条走廊转 180° 长得一模一样 —— 空车道里没有任何东西能区分车头方向。
        所以 -183° 和 -3° 在空场数据上都合格（2026-09-14 现场实测：
        CH2 分别是 0 和 66 点）。差别只来自 ROI 的 y 窗口不对称（-11…+8）。

        要定死朝向只有两条路：现场说明车从哪头进，或者停一辆车进来看车头在哪。
        本用例把这个**局限**钉住，免得有人以为扫一遍就万事大吉。
        """
        xyz = _lane_scene(self.gz)
        a = lane_setup.check_orientation(xyz, self.calib)
        b = lane_setup.check_orientation(_spin(xyz, 180.0), self.calib)
        self.assertLessEqual(a["current_above"], lane_setup.ORIENT_TOL)
        self.assertLessEqual(
            b["current_above"], lane_setup.ORIENT_TOL,
            "转 180° 之后本应同样合格 —— 若这里失败，说明场景不再对称，用例失效了")

    def test_clean_lane_gives_a_wide_plateau(self):
        """没有障碍物的车道，合格角度是**连续的一大段**（现场实测 26°~30°）。"""
        o = lane_setup.check_orientation(_lane_scene(self.gz), self.calib)
        self.assertGreaterEqual(
            o["width"], 10.0,
            "干净车道的平台应当很宽；平台窄意味着车道里有东西挡着")

    def test_obstacle_in_lane_narrows_the_plateau(self):
        """★ 平台窄 ≠ 装歪了，而是车道里有障碍物 —— 两者必须能分开。

        装歪：平台一样宽，只是中心不同（上一条用例覆盖）。
        有障碍物：平台被挤窄。此时**不能**拿它的最优角去定 yaw，
        那等于拿障碍物拟合 ROI。2026-09-14 的 CH4 就是这一种（平台仅 3°）。
        """
        xyz = _lane_scene(self.gz)
        r = np.random.default_rng(7)
        n = 4000
        blob = np.stack([r.uniform(-1.6, -0.6, n), r.uniform(-2.0, 2.0, n),
                         r.uniform(self.gz + 0.6, self.gz + 2.4, n)], 1)
        o = lane_setup.check_orientation(
            np.concatenate([xyz, blob]).astype(np.float64), self.calib)
        self.assertLess(o["width"], 10.0,
                        "车道里杵着一块东西，平台却还是很宽")

    def test_roi_over_empty_space_does_not_count_as_good(self):
        """框跑到没有回波的地方时「高于地面的点=0」，但不该算合格。"""
        r = np.random.default_rng(3)
        n = 30000
        # 只在远处有一圈点，ROI 附近什么都没有
        th = r.uniform(0, 2 * math.pi, n)
        rad = r.uniform(20, 24, n)
        far = np.stack([rad * np.cos(th), rad * np.sin(th),
                        np.full(n, self.gz)], 1)
        o = lane_setup.check_orientation(far, self.calib)
        self.assertEqual(o["current_in"], 0)
        self.assertIsNone(o["plateau"],
                          "框内一个点都没有，不该被当成『朝向正确』")


class TestDumpRaw(unittest.TestCase):
    """--dump-raw：把**没裁过 ROI** 的整帧存下来。

    为什么需要：正常落盘的 .xyz 是先裁 ROI、再做航向旋转之后的结果，
    ROI 之外的东西根本不在文件里 —— 想验证"车厢有没有被 ROI 切掉"，
    只能靠"边缘离边界多远"间接推断，而这个推断在 y 方向并不可靠
    （车厢远端因掠射角会看起来变窄：2026-09-14 实测主体宽 2.67 m，
    到 y=5.5 只剩 2.25 m）。有了未裁的整帧才能**直接看到框外面**。
    """

    class _Ctx:
        pass

    def _ctx(self):
        from app.settings import load_settings
        c = self._Ctx()
        c.calib = load_calib(root("config", "config.json"))
        c.channels = load_settings("").channels
        return c

    def _frame(self, n, seed=0):
        r = np.random.default_rng(seed)
        xyz = r.uniform(-8, 8, (n, 3)).astype(np.float32)
        ring = r.integers(0, 256, n).astype(np.uint16)
        return Frame(xyz, ring)

    def _row(self, cid, sizes):
        return {"channel": cid, "range_mode": "scale_offset",
                "_frames": [self._frame(n, seed=n) for n in sizes]}

    def test_writes_xyz_and_json(self):
        import json as _json
        d = tempfile.mkdtemp(); self.addCleanup(shutil.rmtree, d, True)
        rows = [self._row("CH1", [500, 900, 700])]
        got = lane_setup.dump_raw(rows, self._ctx(), d)
        self.assertEqual(len(got), 1)
        self.assertTrue(os.path.exists(got[0]))
        meta = got[0][:-4] + ".json"
        self.assertTrue(os.path.exists(meta), "缺少 .json 旁注 —— 离线复算要靠它知道当时的 ROI")
        j = _json.load(io.open(meta, encoding="utf-8"))
        for k in ("roi", "yaw_offset_deg", "ground_z", "channel"):
            self.assertIn(k, j, f"旁注里少了 {k}")
        self.assertEqual(j["channel"], "CH1")

    def test_picks_the_densest_frame(self):
        """取点最多的那一帧 —— 整圈最完整，边界判断才可信。"""
        d = tempfile.mkdtemp(); self.addCleanup(shutil.rmtree, d, True)
        rows = [self._row("CH2", [300, 1200, 800])]
        got = lane_setup.dump_raw(rows, self._ctx(), d)
        from app.pointcloud import load_xyz
        self.assertEqual(load_xyz(got[0]).n, 1200)

    def test_saved_cloud_is_not_roi_cropped(self):
        """★ 存下来的必须是**原样**，不能顺手裁一刀 —— 裁了就失去意义。"""
        d = tempfile.mkdtemp(); self.addCleanup(shutil.rmtree, d, True)
        rows = [self._row("CH3", [1000])]
        got = lane_setup.dump_raw(rows, self._ctx(), d)
        from app.pointcloud import load_xyz
        a = load_xyz(got[0]).xyz
        roi = lane_setup._cur_roi(self._ctx().calib)
        self.assertIsNotNone(roi)
        outside = int(((a[:, 0] < roi["x"][0]) | (a[:, 0] > roi["x"][1])).sum())
        self.assertGreater(outside, 0,
                           "存出来的点全在 ROI 内 —— 说明被裁过了，这个开关就白加了")

    def test_no_frames_is_not_an_error(self):
        d = tempfile.mkdtemp(); self.addCleanup(shutil.rmtree, d, True)
        self.assertEqual(lane_setup.dump_raw([{"channel": "CH1"}], self._ctx(), d), [])

    def test_each_channel_gets_its_own_file(self):
        d = tempfile.mkdtemp(); self.addCleanup(shutil.rmtree, d, True)
        rows = [self._row("CH1", [600]), self._row("CH2", [700]),
                self._row("CH3", [800]), self._row("CH4", [900])]
        got = lane_setup.dump_raw(rows, self._ctx(), d)
        self.assertEqual(len(got), 4)
        self.assertEqual(len({os.path.basename(x)[:3] for x in got}), 4)


class TestWriteBackGuards(unittest.TestCase):
    """写回 config.json 的三道闸 —— 写错 ground_z 就是 9-13 那次事故。"""

    class _Args:
        assume_same_mount = False

    def _row(self, cid, gz, ok=True, verdict="ok"):
        return {"channel": cid, "ok": ok,
                "ground": {"ground_z": gz},
                "scale": {"checked": True, "verdict": verdict},
                "roi": {"x": [-2.0, 2.0], "y": [-11.0, 8.0], "z": [gz - 1.0, -0.75]}}

    def test_refuses_when_a_channel_was_not_measured(self):
        """config.json 是四条通道共用的，只量到一部分就写=替别人做主张。"""
        rows = [self._row("CH1", -5.0), self._row("CH2", 0.0, ok=False)]
        rc = lane_setup.write_back(rows, self._Args())
        self.assertEqual(rc, 2)

    def test_allows_it_when_operator_confirms_same_mount(self):
        """放行之后**真的写一次盘** —— 但写到临时文件里。

        原先这条用例是靠 mock 掉 builtins.open / json.dump 来"假装写"。
        那样做有两个毛病：① 闸门之后的代码等于没跑过，写坏了也测不出来；
        ② `builtins.open` 是**全进程**的，万一哪天闸门逻辑变了、提前走到写盘，
        mock 又没盖住，倒霉的是现场那份 config/config.json。
        直接给一个临时路径，两个毛病一起没有。
        """
        rows = [self._row("CH1", -5.0), self._row("CH2", 0.0, ok=False)]
        args = self._Args()
        args.assume_same_mount = True
        import json as _json
        import shutil as _shutil
        d = tempfile.mkdtemp()
        self.addCleanup(_shutil.rmtree, d, True)
        tmp = os.path.join(d, "config.json")
        _shutil.copy2(root("config", "config.json"), tmp)

        rc = lane_setup.write_back(rows, args, path=tmp)
        self.assertEqual(rc, 0)

        cfg = _json.load(io.open(tmp, encoding="utf-8"))
        self.assertAlmostEqual(cfg["detection"]["ground_z"], -5.0, places=6)
        self.assertEqual(cfg["roi"]["z"], [-6.0, -0.75])
        # 现场那份必须一个字没动
        live = _json.load(io.open(root("config", "config.json"), encoding="utf-8"))
        self.assertNotEqual(live["detection"]["ground_z"], -5.0,
                            "write_back 写到了项目里那份共用配置")


    def test_refuses_when_scale_is_still_wrong(self):
        """量纲没定对就写回去 = 把错的固化下来。"""
        rows = [self._row("CH1", -1.25, verdict="bad")]
        self.assertEqual(lane_setup.write_back(rows, self._Args()), 2)

    def test_refuses_when_channels_disagree_on_ground(self):
        """各通道地面高度差太多 —— 共用一份配置必然写错一部分。"""
        rows = [self._row("CH1", -5.0), self._row("CH2", -3.2)]
        self.assertEqual(lane_setup.write_back(rows, self._Args()), 2)

    def test_refuses_when_nothing_succeeded(self):
        rows = [self._row("CH1", 0.0, ok=False)]
        self.assertEqual(lane_setup.write_back(rows, self._Args()), 2)


if __name__ == "__main__":
    unittest.main()
