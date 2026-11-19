# -*- coding: utf-8 -*-
"""进场帧接入服务体积 + 遮挡上限补洞（2026-10-24，docs/92）。

合成场景用**射线追踪**生成：雷达在原点，逐条射线找第一个交点（煤面 / 侧壁 / 端壁 / 地面）。
旧的车厢合成场景是"按区域删点"造缺测，删掉的区域里照样留着壁面回波 —— 那种点云在物理上
不可能出现（真实射线会先打到煤），拿来检验视线上限会得出错误结论。
"""
import hashlib
import math
import unittest

import numpy as np

W, L, HW, FLOOR = 2.4, 11.0, 1.85, -3.6


def _heaps(x, y):
    """三个 35 度坡的煤堆，堆间低谷背向雷达 —— 停稳帧看不到的阴影就在这里。"""
    h = np.zeros(np.broadcast(x, y).shape)
    for cy, top in ((-3.2, 1.5), (0.4, 1.7), (4.2, 1.6)):
        h = np.maximum(h, top - math.tan(math.radians(35)) * np.abs(y - cy))
    return np.clip(h + 0.02 * np.sin(7 * x) * np.cos(5 * y), 0, None)


def _flat(x, y):
    return np.full(np.broadcast(x, y).shape, 1.2) + 0.02 * np.sin(7 * np.asarray(x)) * np.cos(5 * np.asarray(y))


def _empty(x, y):
    return np.zeros(np.broadcast(x, y).shape)


def _trace(height, d_pol=0.6, d_az=0.5, max_pol=70.0, seed=0):
    pol = np.deg2rad(np.arange(0.2, max_pol, d_pol))
    az = np.deg2rad(np.arange(0, 360, d_az))
    p, a = np.meshgrid(pol, az)
    d = np.column_stack([np.sin(p).ravel() * np.sin(a).ravel(), np.sin(p).ravel() * np.cos(a).ravel(),
                         -np.cos(p).ravel()])
    hit = np.full((len(d), 3), np.nan)
    alive = np.ones(len(d), bool)
    t = np.full(len(d), 1.5)
    while alive.any():
        idx = np.flatnonzero(alive)
        q = d[idx] * t[idx, None]
        x, y, z = q.T
        inside = (np.abs(x) < W / 2) & (np.abs(y) < L / 2)
        surface = np.where(inside, FLOOR + height(x, y), FLOOR - 10)
        wall = (((np.abs(x) >= W / 2) & (np.abs(x) < W / 2 + 0.06) & (np.abs(y) < L / 2 + 0.06))
                | ((np.abs(y) >= L / 2) & (np.abs(y) < L / 2 + 0.06) & (np.abs(x) < W / 2 + 0.06)))
        h = (inside & (z <= surface)) | (wall & (z <= FLOOR + HW)) | (z <= FLOOR - 1.5)
        hit[idx[h]] = q[h]
        alive[idx[h]] = False
        alive[idx[((np.abs(x) > 3) | (np.abs(y) > 9)) & ~h]] = False
        t[idx] += 0.015
    pts = hit[np.isfinite(hit).all(1)]
    pts = pts[pts[:, 2] > FLOOR - 1.0]
    return pts + np.random.default_rng(seed).normal(0, 0.004, pts.shape)


def _truth(height):
    x = np.linspace(-W / 2, W / 2, 241)
    y = np.linspace(-L / 2, L / 2, 2201)
    return float(np.trapz(np.trapz(height(x[None, :], y[:, None]), x, axis=1), y))


class _RayScene:
    _cache = {}

    @classmethod
    def cloud(cls, name):
        if name not in cls._cache:
            cls._cache[name] = _trace({"empty": _empty, "heaps": _heaps, "flat": _flat}[name])
        return cls._cache[name]

    @classmethod
    def baseline(cls):
        from app.detect import EmptyBaseline
        c = cls.cloud("empty")
        lo, hi = c.min(0), c.max(0)
        return EmptyBaseline(FLOOR, (float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]), FLOOR, float(hi[2])),
                             c.astype(np.float32), len(c), "t", bed_plane=(0.0, 0.0, 1.0, -FLOOR))

    @classmethod
    def loaded(cls, name):
        from app.detect import LoadedResult
        return LoadedResult(cloud=cls.cloud(name).astype(np.float32))


KW = dict(dy=0.0, dx=0.0, yaw=0.0, pose=False)


class TestOcclusionBoundFill(unittest.TestCase):
    def test_shadowed_valleys_are_no_longer_filled_up_to_the_grazing_ray(self):
        """★ 煤堆间的低谷：松弛把脊线和远处可见面直接连起来（贴着掠射视线），多算约 25%；
        按堆积角背坡估之后回到真值附近，且上下界包络罩住真值。"""
        from app.detect import compute_volume
        base, lr, truth = _RayScene.baseline(), _RayScene.loaded("heaps"), _truth(_heaps)
        old = compute_volume(base, lr, **KW)
        new = compute_volume(base, lr, fill_bound=True, **KW)
        self.assertGreater(old.volume_m3 / truth - 1, 0.15, "前提：旧补洞在阴影区明显多算")
        self.assertLess(abs(new.volume_m3 / truth - 1), 0.05, new.text)
        occ = new.occlusion
        self.assertEqual("applied", occ.status)
        self.assertGreater(occ.shadow_cells, 100)
        self.assertLessEqual(occ.volume_lo_m3, truth)
        self.assertGreaterEqual(occ.volume_hi_m3, truth)
        self.assertIn("遮挡上限补洞", new.text)

    def test_flat_coal_is_left_alone(self):
        """平煤面没有阴影：贴壁格（有回波、只是被剔掉）不能被当成阴影压低。"""
        from app.detect import compute_volume
        base, lr = _RayScene.baseline(), _RayScene.loaded("flat")
        old = compute_volume(base, lr, **KW)
        new = compute_volume(base, lr, fill_bound=True, **KW)
        self.assertAlmostEqual(new.volume_m3, old.volume_m3, delta=0.001 * old.volume_m3)
        self.assertEqual(0, new.occlusion.raised_cells)

    def test_default_off_is_unchanged(self):
        """compute_volume 默认不开：结果与不传参数逐位相同，也不挂诊断。"""
        from app.detect import compute_volume
        base, lr = _RayScene.baseline(), _RayScene.loaded("heaps")
        a = compute_volume(base, lr, **KW)
        b = compute_volume(base, lr, fill_bound=False, **KW)
        self.assertEqual(a.to_dict(raw=True), b.to_dict(raw=True))
        self.assertIsNone(a.occlusion)

    def test_to_dict_keys_do_not_change(self):
        """诊断不进归档合同：to_dict 键集合与不开时相同。"""
        from app.detect import compute_volume
        base, lr = _RayScene.baseline(), _RayScene.loaded("heaps")
        self.assertEqual(set(compute_volume(base, lr, **KW).to_dict(raw=True)),
                         set(compute_volume(base, lr, fill_bound=True, **KW).to_dict(raw=True)))

    def test_ray_ceiling_is_never_below_the_true_surface(self):
        """★ 物理不变量：真实射线不可能穿过煤，所以视线上限（含余量）不能低于格内真实最高点。"""
        from app.occlusion_fill import ray_ceiling
        cell, x0, y0, nx, ny = 0.1, -1.5, -6.0, 30, 120
        gx = x0 + (np.arange(nx) + .5) * cell
        gy = y0 + (np.arange(ny) + .5) * cell
        ceiling = ray_ceiling(_RayScene.cloud("heaps"), x0, y0, nx, ny, cell, np.full((ny, nx), FLOOR))
        offsets = np.linspace(-0.5, 0.5, 6) * cell
        true_max = np.max([_heaps(gx[None, :] + a, gy[:, None] + b) for a in offsets for b in offsets], axis=0)
        inside = (np.abs(gx)[None, :] < W / 2 - 0.1) & (np.abs(gy)[:, None] < L / 2 - 0.1) & np.isfinite(ceiling)
        self.assertGreater(int(inside.sum()), 1000)
        self.assertGreaterEqual(float(np.min(ceiling[inside] - true_max[inside])), 0.0)

    def test_measured_cells_are_never_changed(self):
        from app.occlusion_fill import bound_unknown_cells
        r = np.random.default_rng(1)
        measured = r.uniform(0.5, 1.5, (20, 10))
        measured[5:12, 3:7] = np.nan
        estimate = np.where(np.isfinite(measured), measured, 9.0)
        domain = np.ones_like(measured, bool)
        out, lo, hi, counts = bound_unknown_cells(estimate, measured, domain, None, 0.1)
        known = np.isfinite(measured)
        np.testing.assert_array_equal(out[known], measured[known])
        self.assertTrue((out[~known] < 9.0).all(), "上包络必须压住离谱的估值")
        self.assertEqual(counts["unknown_cells"], int((~known).sum()))
        self.assertTrue((lo[~known] <= out[~known]).all() and (out[~known] <= hi[~known]).all())

    def test_narrow_scan_gaps_are_not_shadows(self):
        """扫描线之间一格宽的窄缝不算阴影（背坡试探点紧挨着命中点时总会被它挡住）。"""
        from app.occlusion_fill import _deep_hole_components
        holes = np.zeros((40, 10), bool)
        holes[10, :] = True                      # 一格宽的缝
        holes[20:30, 2:8] = True                 # 成片的洞
        keep = _deep_hole_components(holes, 0.1)
        self.assertFalse(keep[10].any())
        self.assertTrue(keep[20:30, 2:8].all())

    def test_ceiling_conflict_lets_the_observation_win(self):
        """视线上限比下包络还低时（地形陡于假设坡度），下界让给观测。"""
        from app.occlusion_fill import bound_unknown_cells
        measured = np.full((5, 5), np.nan)
        measured[0, :] = 1.0
        ceiling = np.full((5, 5), 0.2)
        out, lo, hi, _ = bound_unknown_cells(np.full((5, 5), 1.0), measured, np.ones((5, 5), bool), ceiling, 0.1)
        self.assertTrue((out[1:] <= 0.2 + 1e-12).all())
        self.assertTrue((lo[1:] <= hi[1:]).all())


class TestEnterVolumeRecord(unittest.TestCase):
    def test_declared_model_truthfully_excludes_entry_and_occlusion(self):
        from unittest.mock import patch
        from app.detect import LoadedResult, compute_volume
        from tests._volume_context import conditions
        from tests.test_volume_validity import scene
        base, full = scene(.5)
        loaded = LoadedResult(cloud=full.cloud[full.cloud[:, 1] >= 2])
        options = conditions(base, loaded)
        extra = full.cloud[full.cloud[:, 1] < 2].copy()
        extra[:, 2] = 1.5
        plain = compute_volume(base, loaded, dy=0., **options)
        with patch("app.occlusion_fill.bound_unknown_cells", side_effect=AssertionError("声明模型不可补洞")):
            result = compute_volume(base, loaded, dy=0., enter_extra=(extra,), fill_bound=True, **options)
        self.assertEqual(plain.volume_m3, result.volume_m3)
        self.assertEqual(plain.unknown_area_m2, result.unknown_area_m2)
        self.assertTrue(result.declared_model_used)
        self.assertIn("进/离场帧和遮挡上限补洞未参与计算", result.text)

    def test_no_inner_domain_with_fill_bound_returns_skipped(self):
        from app.detect import compute_volume
        from tests.test_volume_validity import scene
        base, loaded = scene(.5)
        result = compute_volume(base, loaded, dy=0., dx=0., yaw=0., pose=False,
                                inner=False, fill_holes=False, fill_bound=True)
        self.assertTrue(result.ok)
        self.assertEqual("skipped", result.occlusion.status)

    def _frames(self, n=3, points=400):
        from app.cloud_observation import FrameSnapshot
        r = np.random.default_rng(5)
        return tuple(FrameSnapshot(r.normal(0, 1, (points, 3)).astype("<f4").tobytes(), None,
                                   seq=10 + i, ts=100.0 + i, gen=0, packets=900) for i in range(n))

    def test_bytes_round_trip_and_tamper_is_rejected(self):
        from app.enter_volume import ENTER_VOLUME_SCHEMA, frame_record, frames_bytes, restore_frames
        frames = self._frames()
        record = dict(schema=ENTER_VOLUME_SCHEMA, status="unresolved", reason="x", used=0, shifts=[],
                      frames=[frame_record(f) for f in frames])
        data = frames_bytes(frames)
        self.assertEqual(restore_frames(record, data), frames)
        bad = bytearray(data)
        bad[100] ^= 1
        with self.assertRaises(ValueError):
            restore_frames(record, bytes(bad))
        with self.assertRaises(ValueError):
            restore_frames(record, data[:-12])

    def test_frames_must_be_ordered_complete_and_same_generation(self):
        from dataclasses import replace
        from app.enter_volume import check_frames
        a, b, c = self._frames()
        with self.assertRaises(ValueError):
            check_frames((b, a))
        with self.assertRaises(ValueError):
            check_frames((a, replace(b, dropped_packets=1)))
        with self.assertRaises(ValueError):
            check_frames((a, replace(b, gen=1)))
        self.assertEqual(check_frames((a, b, c)), (a, b, c))

    def test_single_frame_is_recorded_but_not_used(self):
        from app.enter_volume import register_for_volume
        from app.detect import LoadedResult
        loaded = LoadedResult(cloud=np.zeros((500, 3), np.float32), source_rotation=np.eye(3))
        clouds, record = register_for_volume(loaded, self._frames(1))
        self.assertIsNone(clouds)
        self.assertEqual("unresolved", record["status"])
        self.assertIn("不足 2 帧", record["reason"])

    def test_no_frames_means_no_record(self):
        from app.enter_volume import register_for_volume
        self.assertEqual((None, None), register_for_volume(None, ()))

    def test_frame_record_digest_is_of_xyz_bytes(self):
        from app.enter_volume import frame_record
        f = self._frames(1)[0]
        self.assertEqual(frame_record(f)["sha256"], hashlib.sha256(f.xyz_bytes).hexdigest())
        self.assertEqual(frame_record(f)["points"], 400)


class TestEnterBufferThinning(unittest.TestCase):
    """缓冲满了按位移抽稀，最早（看清车厢前端）的帧不被最先挤掉。"""

    def _worker(self, **kw):
        import copy
        import tempfile
        from unittest.mock import patch
        from app.radar_source import build_source
        from app.service import ChannelWorker, EventBus
        from tests import _helpers as H
        settings, ch, _, _, _ = H.ctx()
        ch = copy.deepcopy(ch)
        ch.id, ch.mode = "CH1", "synthetic"
        ch.abs_output_dir = tempfile.mkdtemp()

        def syn(*a, **k):
            return build_source(*a, **dict(k, mode="synthetic"))
        with patch("app.application.channel_worker.build_source", side_effect=syn):
            return ChannelWorker(ch, EventBus(), settings.plate, save_override=True, plate_mode="null", **kw)

    def _feed(self, worker, offsets):
        from types import SimpleNamespace
        from app.pointcloud import Frame
        stab = SimpleNamespace(n_vehicle_points=worker.stability.vehicle_threshold + 1)
        for seq, dy in enumerate(offsets, 1):
            points = np.zeros((500, 3), np.float32)
            points[:, 1] = np.linspace(-1., 1., 500) + dy
            points[:, 2] = -1.
            shot = SimpleNamespace(frame=lambda points=points, seq=seq: Frame(points, seq=seq, ts=float(seq)))
            worker._current_pair = SimpleNamespace(raw=shot, cropped=shot)
            worker._grab_enter(stab, True)

    def test_earliest_frame_survives_a_full_buffer(self):
        """8 帧缓冲、每帧 0.15 m 走 2.25 m：旧环形缓冲只剩最后 8 帧（1.05 m），抽稀后最早一帧还在。"""
        w = self._worker(enter_frames=8, enter_step_m=0.10, enter_crop_roi=True)
        self._feed(w, [0.15 * i for i in range(16)])
        seqs = [s.seq for s in w._enter_shots]
        self.assertEqual(seqs[0], 1, "最早一帧被挤掉了：%s" % seqs)
        self.assertLessEqual(len(seqs), 8)
        self.assertAlmostEqual(w._enter_step_eff, 0.2)
        self.assertTrue(all(0.15 * (b - a) <= 0.5 + 1e-9 for a, b in zip(seqs, seqs[1:])), seqs)

    def test_thinning_stops_before_the_chain_step_limit(self):
        """抽稀不超过 ENTER_THIN_MAX_STEP_M；之后退回丢最早帧，相邻帧位移仍在链式配准范围内。"""
        from app.service import ENTER_THIN_MAX_STEP_M
        w = self._worker(enter_frames=4, enter_step_m=0.10, enter_crop_roi=True)
        self._feed(w, [0.15 * i for i in range(60)])
        self.assertLessEqual(w._enter_step_eff, ENTER_THIN_MAX_STEP_M)
        seqs = [s.seq for s in w._enter_shots]
        self.assertTrue(all(0.15 * (b - a) <= 0.6 + 1e-9 for a, b in zip(seqs, seqs[1:])), seqs)

    def test_flush_freezes_volume_frames_and_reset_clears_them(self):
        w = self._worker(enter_frames=8, enter_step_m=0.10, enter_crop_roi=True)
        self._feed(w, [0.15 * i for i in range(5)])
        w._flush_enter()
        self.assertEqual(5, len(w._enter_volume))
        self.assertTrue(all(s.ring_bytes is None for s in w._enter_volume))
        w._enter_reset()
        self.assertEqual((), w._enter_volume)
        self.assertAlmostEqual(w._enter_step_eff, 0.10)

    def test_switch_off_keeps_capture_but_not_volume(self):
        w = self._worker(enter_frames=8, enter_step_m=0.10, enter_crop_roi=True, enter_volume=False)
        self._feed(w, [0.15 * i for i in range(5)])
        w._flush_enter()
        self.assertEqual((), w._enter_volume)


class TestExitVolumeFreeze(unittest.TestCase):
    """离场帧体积冻结：缓冲满 / 落盘时冻 ROI 快照；exit_volume=false 只采不冻。"""

    def _worker(self, **kw):
        import copy
        import tempfile
        from unittest.mock import patch
        from app.radar_source import build_source
        from app.service import ChannelWorker, EventBus
        from tests import _helpers as H
        settings, ch, _, _, _ = H.ctx()
        ch = copy.deepcopy(ch)
        ch.id, ch.mode = "CH1", "synthetic"
        ch.abs_output_dir = tempfile.mkdtemp()

        def syn(*a, **k):
            return build_source(*a, **dict(k, mode="synthetic"))
        with patch("app.application.channel_worker.build_source", side_effect=syn):
            worker = ChannelWorker(ch, EventBus(), settings.plate, save_override=True,
                                   plate_mode="null", **kw)
        worker._local_passage = "test-exit-volume"
        return worker

    def _feed_exit(self, worker, offsets):
        from types import SimpleNamespace
        from app.pointcloud import Frame
        stab = SimpleNamespace(n_vehicle_points=worker.stability.vehicle_threshold + 1)
        worker._mark_exit_eligible("mid-exit")
        for seq, dy in enumerate(offsets, 1):
            points = np.zeros((500, 3), np.float32)
            points[:, 1] = np.linspace(-1., 1., 500) + dy
            points[:, 2] = -1.
            shot = SimpleNamespace(frame=lambda points=points, seq=seq: Frame(points, seq=seq, ts=float(seq)))
            worker._current_pair = SimpleNamespace(raw=shot, cropped=shot)
            worker._grab_exit(stab, True)

    def test_buffer_full_freezes_volume_frames(self):
        w = self._worker(exit_frames=3, exit_step_m=0.10, exit_crop_roi=True, exit_volume=True)
        self._feed_exit(w, [0., .15, .30])
        self.assertEqual(3, len(w._exit_volume))
        self.assertTrue(all(s.ring_bytes is None for s in w._exit_volume))

    def test_switch_off_keeps_capture_but_not_volume(self):
        w = self._worker(exit_frames=3, exit_step_m=0.10, exit_crop_roi=True, exit_volume=False)
        self._feed_exit(w, [0., .15, .30])
        self.assertEqual(3, len(w._exit_shots))
        self.assertEqual((), w._exit_volume)

    def test_register_exit_reverses_for_anchor(self):
        """采集近端在前；配准记录 frames 仍采集序，shifts 与采集序对齐。"""
        from app.cloud_observation import FrameSnapshot
        from app.detect import LoadedResult
        from app.enter_volume import _register_checked, register_exit_for_volume
        pts = np.zeros((400, 3), np.float64)
        pts[:, 1] = np.linspace(-2., 2., 400)
        pts[:, 2] = 1.
        loaded = LoadedResult(cloud=pts.copy(), source_rotation=np.eye(3))
        frames = tuple(
            FrameSnapshot((pts + [0., .2 * k, 0.]).astype("<f4").tobytes(), None, k + 1, float(k), 0, 0)
            for k in range(3))
        # reverse 后与直接对 reverse 元组配准，身份记录应按采集序写出。
        _, record = register_exit_for_volume(loaded, frames)
        _, direct = _register_checked(loaded, tuple(reversed(frames)), direction=-1)
        self.assertEqual([f.seq for f in frames], [item["seq"] for item in record["frames"]])
        self.assertEqual(list(reversed(direct["shifts"])), record["shifts"])
        self.assertEqual(direct["status"], record["status"])

    @staticmethod
    def _textured_coal():
        x, y = np.meshgrid(np.arange(-1.1875, 1.2, .025), np.arange(-4.9875, 5, .025))
        z = .9 + .3 * np.sin(1.7 * y) + .16 * np.cos(3.3 * y) + .015 * np.sin(8 * x)
        return np.column_stack((x.ravel(), y.ravel(), z.ravel()))

    def test_forward_departure_exit_frames_register_and_fill_the_rear(self):
        """2026-11-02 现场确认载货车向前离场：离场帧在停稳位置 +y 一侧，配准位移为负，补后端。"""
        from app.cloud_observation import FrameSnapshot
        from app.detect import EmptyBaseline, InnerDomain, LoadedResult, compute_volume
        from app.enter_volume import register_exit_for_volume, register_for_volume
        surface = self._textured_coal()
        stop = surface[np.abs(surface[:, 1]) < 3].copy()
        loaded = LoadedResult(cloud=stop, source_rotation=np.eye(3))

        def shots(body, offsets):
            return tuple(FrameSnapshot((body + [0., d, 0.]).astype("<f4").tobytes(), None, i + 1, 100. + i, 0, 1000)
                         for i, d in enumerate(offsets))
        rear_body = surface[surface[:, 1] < 3]
        clouds, record = register_exit_for_volume(loaded, shots(rear_body, [.2, .4, .6]))
        self.assertEqual("resolved", record["status"], record["reason"])
        self.assertEqual(3, record["used"])
        # shifts 按采集序：近端先，位移为负且接近真值
        for got, want in zip(record["shifts"], [-.2, -.4, -.6]):
            self.assertAlmostEqual(want, got, delta=0.05)
        # 进场帧方向不变
        entry, er = register_for_volume(loaded, shots(surface[surface[:, 1] > -3], [-.6, -.4, -.2]))
        self.assertEqual("resolved", er["status"], er["reason"])

        floor = surface.copy()
        floor[:, 2] = 0
        base = EmptyBaseline(0., (-1.2, 1.2, -5., 5., 0., 2.), floor, len(floor), "analytic", bed_plane=(0., 0., 1., 0.))
        domain = InnerDomain("resolved", "analytic", left=(-1.2, 0.), right=(1.2, 0.), y0=-5., y1=5.)
        truth = float(surface[:, 2].mean() * 24.)

        def volume(extra):
            return compute_volume(base, loaded, dy=0., dx=0., yaw=0., pose=False, inner=domain,
                                  enter_extra=extra, fill_bound=False).volume_m3
        with_entry = volume(tuple(entry))
        with_both = volume(tuple(entry) + tuple(clouds))
        # 补上后端后，与解析参考的误差明显减小
        self.assertLess(abs(with_both - truth), abs(with_entry - truth))
        self.assertLess(abs(with_both - truth) / truth, 0.03)


if __name__ == "__main__":
    unittest.main()
