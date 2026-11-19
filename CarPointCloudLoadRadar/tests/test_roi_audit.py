# -*- coding: utf-8 -*-
"""全量帧 ROI 审查工具的测试。

这个工具的结论会直接决定"要不要改 ROI"，而改 ROI 是**红线**
（`config.json` 的 roi 一动就得跑 verify.py）。所以它宁可说"存疑"，
也不能给出错误的肯定结论。

2026-09-15 实测中连踩三个坑，每个都钉一条测试：

  1. **按点数密度找车** —— 俯视雷达天底附近角分辨率极高，正下方一片能有
     2 万点、6 m 外只剩几百。按点数找到的是"雷达看得最密的地方"。
     量出车厢 5 m 长（实际 11.5 m）。
  2. **用 x 百分位跨度当宽度** —— 一片里有两个物体时，百分位跨的是两者之和。
     把"左边碎片 + 右边门架"算成 4.4 m 宽的车厢。
  3. **用格子边界当宽度** —— 向外多算最多 0.5 m，真车厢 2.4 m 量成 2.75，
     与"并进了墙"的 3.25 挤在一起，宽度上限没法定。
"""
from __future__ import annotations

import io
import contextlib
import json
import os
from pathlib import Path
import sys
import tempfile
from datetime import datetime
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "cli")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from roi_audit import (BOX_W_HI, captured_yaw, check, find_boxes,   # noqa: E402
                       load_frame, main_box, rotate_yaw)
import roi_audit as audit

GROUND = -5.23


def slab(x0, x1, y0, y1, z, n=6000, seed=0):
    """造一片水平面（模拟雷达只看得见的顶面）。"""
    rng = np.random.default_rng(seed)
    return np.column_stack([
        rng.uniform(x0, x1, n),
        rng.uniform(y0, y1, n),
        np.full(n, GROUND + z) + rng.normal(0, 0.01, n),
    ])


class TestFindBoxes(unittest.TestCase):

    def test_finds_a_single_box(self):
        a = slab(-1.2, 1.2, -5.0, 5.0, 3.4)
        b = find_boxes(a, GROUND)
        self.assertEqual(len(b), 1)
        self.assertAlmostEqual(b[0]["x0"], -1.2, delta=0.15)
        self.assertAlmostEqual(b[0]["x1"], 1.2, delta=0.15)
        self.assertAlmostEqual(b[0]["y1"] - b[0]["y0"], 10.0, delta=0.6)

    def test_ignores_the_ground(self):
        """地面点必须被高度带挡掉，否则整个车道都会被当成车厢。"""
        a = np.vstack([slab(-1.2, 1.2, -5, 5, 3.4),
                       slab(-3, 3, -15, 15, 0.0, n=40000, seed=1)])
        b = find_boxes(a, GROUND)
        self.assertEqual(len(b), 1)
        self.assertLess(b[0]["y1"] - b[0]["y0"], 12.0)

    def test_ignores_points_above_the_lidar(self):
        """天花板/门架在雷达平面附近甚至更高，必须被上界挡掉。"""
        a = np.vstack([slab(-1.2, 1.2, -5, 5, 3.4),
                       slab(-1.2, 1.2, 7, 12, 5.4, n=20000, seed=2)])
        b = find_boxes(a, GROUND)
        self.assertEqual(len(b), 1)
        self.assertLess(b[0]["y1"], 6.0)

    def test_density_spike_at_nadir_does_not_shrink_the_box(self):
        """★ 坑 1：天底附近点密度极高，不能因此把车厢量短。

        车厢均匀 10 m 长，但 y 在 [-1,1] 处再塞 5 倍的点（模拟扫描几何）。
        按点数找"密集区"会量成 2 m；按几何应当仍是 10 m。
        """
        a = np.vstack([slab(-1.2, 1.2, -5, 5, 3.4, n=4000),
                       slab(-1.2, 1.2, -1, 1, 3.4, n=20000, seed=3)])
        b = find_boxes(a, GROUND)
        self.assertEqual(len(b), 1)
        self.assertGreater(b[0]["y1"] - b[0]["y0"], 8.0,
                           "被天底密度带偏了 —— 判据不能用点数")

    def test_two_narrow_objects_are_not_merged_into_one_wide_box(self):
        """★ 坑 2：一片里有两个窄物体时，不能把它们的跨度当成一个物体的宽度。

        左 x=-1.4 一条窄带、右 x=2.9 一条窄带，跨度 4.3 m。
        若用百分位就会得到一个"4.3 m 宽的车厢"。取单个连续簇则两条都太窄，
        不构成车厢。
        """
        a = np.vstack([slab(-1.5, -1.3, -5, 5, 3.9, n=8000),
                       slab(2.8, 3.0, -5, 5, 3.9, n=8000, seed=4)])
        for b in find_boxes(a, GROUND):
            self.assertLessEqual(b["x1"] - b["x0"], BOX_W_HI)

    def test_box_width_is_not_inflated_by_grid_quantization(self):
        """★ 坑 3：宽度要用簇内实际点的跨度，不是格子边界。

        真宽 2.40 m。若按 0.25 m 的格子边界量会变成 2.50~2.75。
        """
        a = slab(-1.2, 1.2, -5, 5, 3.4, n=20000, seed=5)
        b = find_boxes(a, GROUND)[0]
        self.assertLess(b["x1"] - b["x0"], 2.55,
                        "宽度被格子量化撑大了")

    def test_a_wall_beside_the_truck_does_not_extend_the_box(self):
        """墙与车横向中心差一两米，中心连续性判据应当把它们断开。"""
        a = np.vstack([slab(-1.2, 1.2, -5, 2, 3.4),
                       slab(-2.9, -0.6, 2.5, 8, 3.4, n=8000, seed=6)])
        b = find_boxes(a, GROUND)
        self.assertGreaterEqual(len(b), 2, "墙被并进车厢了")

    def test_short_fragments_are_not_boxes(self):
        a = slab(-1.2, 1.2, 0.0, 1.5, 3.4)      # 只有 1.5 m 长
        self.assertEqual(find_boxes(a, GROUND), [])


class TestMainBox(unittest.TestCase):
    """磅秤在雷达正下方，被测车必然跨过 y=0；其余是过路车辆。"""

    def _b(self, y0, y1):
        return dict(x0=-1.2, x1=1.2, y0=y0, y1=y1, z_top=3.4, slices=40)

    def test_picks_the_one_spanning_zero(self):
        boxes = [self._b(4.0, 9.0), self._b(-5.0, 3.0)]
        self.assertEqual(main_box(boxes), 1)

    def test_none_when_empty(self):
        self.assertIsNone(main_box([]))

    def test_falls_back_to_nearest_when_none_spans_zero(self):
        boxes = [self._b(6.0, 11.0), self._b(-9.0, -2.0)]
        self.assertEqual(main_box(boxes), 1)

    def test_a_passing_vehicle_is_not_chosen(self):
        """过路车辆即使更大也不能被选成被测车。"""
        boxes = [self._b(3.0, 15.0), self._b(-4.0, 2.0)]
        self.assertEqual(main_box(boxes), 1)


class TestCheck(unittest.TestCase):

    ROI = ((-2.0, 2.0), (-11.0, 8.0))

    def _b(self, x0, x1, y0, y1):
        return dict(x0=x0, x1=x1, y0=y0, y1=y1, z_top=3.4, slices=40)

    def test_inside(self):
        ok, bad, margin = check([self._b(-1.2, 1.2, -5, 5)], self.ROI)
        self.assertTrue(ok)
        self.assertEqual(bad, [])
        self.assertAlmostEqual(margin, 0.8, places=6)

    def test_reports_which_side(self):
        ok, bad, _ = check([self._b(-1.2, 2.3, -5, 5)], self.ROI)
        self.assertFalse(ok)
        self.assertIn("右", bad[0])

    def test_front_overrun_is_reported(self):
        ok, bad, _ = check([self._b(-1.2, 1.2, -5, 9.0)], self.ROI)
        self.assertFalse(ok)
        self.assertIn("前", bad[0])

    def test_margin_is_the_worst_over_all_boxes(self):
        _ok, _bad, margin = check(
            [self._b(-1.2, 1.2, -5, 5), self._b(-1.9, 1.2, -5, 5)], self.ROI)
        self.assertAlmostEqual(margin, 0.1, places=6)

    def test_empty_gives_no_margin(self):
        ok, bad, margin = check([], self.ROI)
        self.assertTrue(ok)
        self.assertEqual(bad, [])
        self.assertIsNone(margin)


class TestYawRecompute(unittest.TestCase):
    """按当前配置的 yaw 复算整帧 —— 没有它，本工具改完配置验证不了配置。

    整帧存的是**已经解码过的笛卡尔坐标**，不是原始报文。改了
    `channels.json` 的 `lidar.yaw_offset_deg` 之后直接重跑，读到的仍是
    旧角度下的点云，余量一个数都不会变 —— 看的人却会以为"新配置也没改善"。

    这里钉住两件事：
      1. `rotate_yaw(d)` 与「解码时把 yaw_offset_deg 加 d」**逐点等价**；
      2. 存盘角度与当前配置一致、或没有旁注 json 时，**一个点都不许动**。

    2026-09-16 用真解码器（PacketDecoder，同一个报文两个 yaw）比对过，
    最大逐点误差 2.6e-06 m —— 那是 float32 的舍入，不是算法差异。
    """

    def test_matches_the_real_decoder(self):
        """★ 与 app/radar_packet.py 的解码结果逐点比对，不是自己跟自己比。"""
        import copy
        from app.pointcloud import AngleCache
        from app.radar_packet import PacketDecoder
        from app.settings import load_calib

        calib = load_calib(os.path.join(_ROOT, "config", "config.json"))
        ac = AngleCache().load(os.path.join(_ROOT, "config", "angle_cache.csv"))
        pk = calib.packet
        rng = np.random.default_rng(7)
        buf = bytearray(rng.integers(1, 250, size=pk.size, dtype=np.uint8).tobytes())
        for off, deg in ((pk.angle1_offset, 137.25), (pk.angle2_offset, 291.80)):
            buf[off:off + 2] = int(round(deg / calib.processing.angle_scale))                 .to_bytes(2, "little")

        def decode(yaw):
            c = copy.copy(calib)
            c.processing = copy.copy(calib.processing)
            c.processing.yaw_offset_deg = float(yaw)
            dec = PacketDecoder(c.packet, c.processing, ac,
                                range_mode="scale_offset", z_down=True,
                                max_range_m=60.0)
            xyz, _r, _i, _info = dec.decode_frame_batch([bytes(buf)], 1, 0.0)
            return np.asarray(xyz, dtype=np.float64)

        base = decode(-183.0)
        self.assertGreater(len(base), 0, "样例报文没解出点，测试本身失效了")
        for d in (3.0, -3.0, 5.25):
            want = decode(-183.0 + d)
            got = rotate_yaw(base, d)
            self.assertLess(float(np.abs(got - want).max()), 1e-4,
                            "rotate_yaw(%+.2f) 与真解码对不上" % d)

    def test_zero_delta_touches_nothing(self):
        a = np.array([[1.0, 2.0, -5.0], [-0.5, 7.0, -3.0]])
        self.assertIs(rotate_yaw(a, 0.0), a)

    def test_rotation_preserves_range_and_height(self):
        """只绕 z 转：水平距离与 z 都不能变。"""
        rng = np.random.default_rng(3)
        a = rng.uniform(-8, 8, size=(500, 3))
        b = rotate_yaw(a, -3.25)
        self.assertLess(float(np.abs(np.hypot(b[:, 0], b[:, 1])
                                     - np.hypot(a[:, 0], a[:, 1])).max()), 1e-9)
        self.assertLess(float(np.abs(b[:, 2] - a[:, 2]).max()), 1e-12)

    def test_aligns_a_tilted_carriage(self):
        """把斜停 3 度的车厢转正之后，x 宽度必须明显收窄。"""
        rng = np.random.default_rng(11)
        y = rng.uniform(-6.0, 5.0, 9000)
        k = np.tan(np.deg2rad(-3.0))
        x = k * y + rng.uniform(-1.25, 1.25, 9000)
        a = np.column_stack([x, y, np.full(9000, GROUND + 2.0)])
        w0 = a[:, 0].max() - a[:, 0].min()
        w1 = rotate_yaw(a, 3.0)[:, 0].max() - rotate_yaw(a, 3.0)[:, 0].min()
        self.assertLess(w1, w0 - 0.4,
                        "补 +3 度没把 -3 度的斜停转正，符号搞反了")
        # 反着补会把倾斜加倍 —— 这是 2026-09-16 差点踩进去的坑
        w2 = rotate_yaw(a, -3.0)[:, 0].max() - rotate_yaw(a, -3.0)[:, 0].min()
        self.assertGreater(w2, w0, "反向补偿理应更宽")


class TestLoadFrame(unittest.TestCase):
    """load_frame 的取舍：**不知道存盘角度时不许猜**。"""

    def setUp(self):
        import tempfile
        from app.pointcloud import Frame, save_xyz
        self.d = tempfile.mkdtemp(prefix="roiaudit_")
        self.f = os.path.join(self.d, "CH9_20260916_120000_1of1.xyz")
        a = np.array([[1.0, 2.0, -5.0], [-0.5, 7.0, -3.0]], dtype=np.float32)
        save_xyz(self.f, Frame(a, None))

    def tearDown(self):
        import shutil
        shutil.rmtree(self.d, ignore_errors=True)

    def _meta(self, yaw):
        import json
        with io.open(self.f + ".meta.json", "w", encoding="utf-8") as fp:
            json.dump({"effective_yaw_offset_deg": yaw}, fp)

    def test_no_meta_means_no_guessing(self):
        """没有旁注 json 就原样返回 —— 猜错了会得出完全相反的越界结论。"""
        _a, d = load_frame(self.f, -180.0, recompute=True)
        self.assertEqual(d, 0.0)
        self.assertIsNone(captured_yaw(self.f))

    def test_same_yaw_means_no_rotation(self):
        self._meta(-180.0)
        _a, d = load_frame(self.f, -180.0, recompute=True)
        self.assertEqual(d, 0.0)

    def test_delta_is_current_minus_captured(self):
        self._meta(-183.0)
        _a, d = load_frame(self.f, -180.0, recompute=True)
        self.assertAlmostEqual(d, 3.0, places=9)

    def test_as_captured_disables_it(self):
        self._meta(-183.0)
        a, d = load_frame(self.f, -180.0, recompute=False)
        self.assertEqual(d, 0.0)
        self.assertAlmostEqual(float(a[0, 0]), 1.0, places=5)

    def test_invalid_yaw_is_unknown_not_a_rotation(self):
        for value in (True, float("nan"), float("inf"), "-180"):
            self._meta(value)
            self.assertIsNone(captured_yaw(self.f))
            _, delta = load_frame(self.f, -180.)
            self.assertEqual(0., delta)


class TestArchiveAudit(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="roi-archive-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.channels = [SimpleNamespace(id=cid, abs_output_dir=str(self.root), abs_config_path=cid,
                                        lidar=SimpleNamespace(yaw_offset_deg=20.)) for cid in ("CH1", "CH2")]
        self.settings = SimpleNamespace(channels=self.channels)
        self.info = dict(roi=((-2., 2.), (-6., 4.)), ground_z=GROUND, yaw=20.)

    def package(self, name="package", *, channel="CH1", measurement="m1", fronts=(3., 5., 3., 3.),
                provenance=True, diagnostic=True, start=None, empty_points=False):
        from app.cloud_archive import freeze_bundle
        from app.cloud_observation import ObservationPair, ObservationWindow
        from app.pointcloud import Frame, apply_roi
        from tests.test_sample_paths import publish
        if start is None:
            start = datetime(2026, 9, 18, 12).timestamp()
        pairs = []
        crop = ((-1., -4., -5.), (1., 4., 0.))
        for index, front in enumerate(fronts):
            points = slab(-1.2, 1.2, -5., front, 3.4, n=10000, seed=index)
            if empty_points:
                points[:, 2] = GROUND
            else:
                self.assertTrue(find_boxes(points, GROUND), "正例必须真实识别出候选")
            frame = Frame(points, None, index + 1, start + index, 3, 450)
            pairs.append(ObservationPair.freeze(channel=channel, epoch=1, received_monotonic=100. + index,
                                                complete=True, raw=frame, cropped=apply_roi(frame, crop), roi=crop))
        primary, support = ObservationWindow(tuple(pairs[-2:])), ObservationWindow(tuple(pairs))
        detection = primary.input_frame()
        bundle = freeze_bundle(measurement_id=measurement, channel=channel, passage_id="p1",
                               observation=pairs[-1].raw.frame(), detection=detection,
                               vehicle_indices=np.arange(pairs[-1].raw.frame().n),
                               carriage_indices=np.arange(pairs[-1].raw.frame().n),
                               result={"empty": False, "config": {"yaw_offset_deg": 90.}},
                               roi={"carriage_status": "partial"}, configuration_hash="d" * 64,
                               algorithm_version="test-v1", complete=True, measurable=True,
                               extra_frames=(pairs[-1].raw.frame(),) if diagnostic else (),
                               observation_window=primary if provenance else None,
                               detection_indices=np.arange(detection.n) if provenance else None,
                               detection_rotation=np.eye(3) if provenance else None,
                               volume_window=support if provenance else None,
                               volume_extra_members=tuple(range(len(pairs) - 1)) if provenance else None)
        directory = self.root / "archives-v1" / name
        publish(directory, bundle)
        return directory

    def run_cli(self, args=()):
        report = self.root / "report.json"
        def calib(cid):
            return SimpleNamespace(roi=((-2., -6., -10.), (2., 4. if cid == "CH1" else 8., 1.)),
                                   detection=SimpleNamespace(ground_z=GROUND), processing=SimpleNamespace(yaw_offset_deg=0.))
        stream = io.StringIO()
        with patch.object(audit, "load_settings", return_value=self.settings), patch.object(audit, "load_calib", side_effect=calib), \
                contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
            status = audit.main([*args, "--json", str(report)])
        return status, json.loads(report.read_text(encoding="utf-8")), stream.getvalue()

    def test_default_output_finds_archive_without_legacy_raw(self):
        self.package(diagnostic=False)
        status, report, _ = self.run_cli()
        self.assertEqual(0, status)
        self.assertEqual(1, len(report["rows"]))
        row = report["rows"][0]
        self.assertEqual(("primary", 2), (row["window_kind"], row["frames"]))
        self.assertEqual("inside_visible", row["status"])
        self.assertFalse(row["physical_boundary_verified"])

    def test_first_primary_raw_overrun_not_hidden_by_last_or_cropped_input(self):
        self.package(fronts=(3., 3., 5., 3.))
        status, report, _ = self.run_cli()
        self.assertEqual(0, status)
        row = report["rows"][0]
        self.assertLess(row["margin"], 0.)
        self.assertGreater(row["members"][-1]["margin"], 0.)
        self.assertEqual("uncertain", row["status"])
        self.assertEqual(2, row["frames"])

    def test_volume_primary_and_diagnostic_roles_do_not_inflate_frames(self):
        self.package()
        _, primary, _ = self.run_cli()
        status, volume, _ = self.run_cli(["--window", "volume"])
        self.assertEqual(0, status)
        self.assertEqual(2, primary["rows"][0]["frames"])
        row = volume["rows"][0]
        self.assertEqual(4, row["frames"])
        self.assertEqual(["primary", "extra"], row["members"][2]["usage"])
        self.assertEqual(["primary"], row["members"][3]["usage"])

    def test_shared_root_and_same_measurement_use_each_channel_roi(self):
        self.package("one", fronts=(5.,) * 4)
        self.package("two", channel="CH2", fronts=(5.,) * 4)
        status, report, _ = self.run_cli()
        self.assertEqual(0, status)
        self.assertEqual(["outside_visible", "inside_visible"], [r["status"] for r in report["rows"]])
        status, report, _ = self.run_cli(["--channel", "CH2"])
        self.assertEqual(0, status)
        self.assertEqual(["CH2"], [r["channel"] for r in report["rows"]])

    def test_valid_plus_broken_or_partial_package_returns_error(self):
        self.package()
        broken = self.package("broken", measurement="broken")
        (broken / "source-000.xyz").write_bytes(b"broken")
        partial = self.package("partial", measurement="partial")
        (partial / "manifest.json").unlink()
        status, report, _ = self.run_cli()
        self.assertEqual(2, status)
        self.assertEqual(1, len(report["rows"]))
        self.assertEqual(2, len(report["errors"]))

    def test_conflicting_identity_is_not_counted(self):
        self.package()
        self.package("conflict", fronts=(5.,) * 4)
        status, report, _ = self.run_cli()
        self.assertEqual(2, status)
        self.assertEqual([], report["rows"])
        self.assertTrue(report["errors"])

    def test_date_uses_last_acquisition_keeps_cross_midnight_window(self):
        package = self.package(start=datetime(2026, 9, 17, 23, 59, 58).timestamp())
        os.utime(package / "manifest.json", (1000., 1000.))
        status, report, _ = self.run_cli(["--date", "20260918"])
        self.assertEqual(0, status)
        self.assertEqual(2, report["rows"][0]["frames"])
        status, report, _ = self.run_cli(["--date", "20260917"])
        self.assertEqual(2, status)
        self.assertEqual([], report["rows"])

    def test_hidden_front_positive_margin_and_no_candidate_never_prove_complete(self):
        self.package("hidden", fronts=(3.,) * 4)
        self.package("none", measurement="none", empty_points=True)
        status, report, text = self.run_cli()
        self.assertEqual(0, status)
        self.assertEqual(1, report["summary"]["inside_visible"])
        self.assertEqual(1, report["summary"]["unknown"])
        self.assertIn("可见候选在内 1", text)
        for row in report["rows"]:
            self.assertFalse(row["physical_boundary_verified"])

    def test_unknown_archive_yaw_never_rotated_even_if_config_fingerprint_exists(self):
        self.package()
        groups, errors = audit.discover_audit_groups(self.settings)
        self.assertEqual([], errors)
        with patch.object(audit, "rotate_yaw", side_effect=AssertionError("不能猜实际 yaw")):
            normal = audit.audit_group(groups[0], self.info)
            captured = audit.audit_group(groups[0], self.info, as_captured=True)
        self.assertEqual(normal["margin"], captured["margin"])
        self.assertTrue(all("yaw_unknown" in m["yaw_status"] for m in normal["members"]))

    def test_no_provenance_is_single_observation_not_multiframe(self):
        self.package(provenance=False)
        status, report, _ = self.run_cli()
        self.assertEqual(0, status)
        self.assertEqual(("single_observation", 1), (report["rows"][0]["window_kind"], report["rows"][0]["frames"]))
        status, report, _ = self.run_cli(["--window", "volume"])
        self.assertEqual(2, status)
        self.assertTrue(report["errors"])

    def test_legacy_raw_group_date_and_member_yaw_remain_distinct(self):
        from app.pointcloud import Frame, save_xyz
        directory = self.root / "raw" / "CH1"
        directory.mkdir(parents=True)
        for index in (1, 2):
            path = directory / ("CH1_20260918_120000_%dof2.xyz" % index)
            save_xyz(str(path), Frame(slab(-1.2, 1.2, -5., 3., 3.4)))
            if index == 1:
                Path(str(path) + ".meta.json").write_text('{"effective_yaw_offset_deg":20}', encoding="utf-8")
        status, report, _ = self.run_cli(["--date", "20260918"])
        self.assertEqual(0, status)
        row = report["rows"][0]
        self.assertEqual(("legacy_raw", 2), (row["window_kind"], row["frames"]))
        self.assertIn("current_yaw_delta", row["members"][0]["yaw_status"])
        self.assertIn("yaw_unknown", row["members"][1]["yaw_status"])
        status, report, _ = self.run_cli(["--dir", str(directory), "--date", "20260917"])
        self.assertEqual(2, status)
        self.assertEqual([], report["rows"])

    def test_legacy_groups_never_join_across_parent_directories(self):
        from app.pointcloud import Frame, save_xyz
        nested = self.root / "CH1"
        nested.mkdir()
        cloud = Frame(slab(-1.2, 1.2, -5., 3., 3.4))
        basename = "CH1_20260918_120000_%dof2.xyz"
        save_xyz(str(self.root / (basename % 1)), cloud)
        save_xyz(str(nested / (basename % 2)), cloud)
        groups, errors = audit.discover_audit_groups(self.settings, str(self.root))
        self.assertEqual([], groups)
        self.assertEqual(2, len(errors))
        save_xyz(str(self.root / (basename % 2)), cloud)
        save_xyz(str(nested / (basename % 1)), cloud)
        groups, errors = audit.discover_audit_groups(self.settings, str(self.root))
        self.assertEqual([], errors)
        self.assertEqual(2, len(groups))
        self.assertNotEqual(groups[0].key, groups[1].key)
        for group in groups:
            self.assertEqual(1, len({Path(path).parent for path in group.files}))


if __name__ == "__main__":
    unittest.main()
