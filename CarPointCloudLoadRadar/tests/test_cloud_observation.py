"""精确原始行映射、不可变快照、合帧来源及实际 Service 配对路径。"""
import copy
import hashlib
import io
import json
from dataclasses import replace
import unittest
from unittest.mock import patch

import numpy as np

from app.cloud_observation import FrameSnapshot, ObservationPair, ObservationWindow
from app.cloud_archive import FrozenBundle, freeze_bundle
from app.pointcloud import Frame, apply_roi
from app.service import ChannelWorker, EventBus
from app.settings import load_settings
from tests._observations import ObservationClock


ROI = ((-2., -6., -4.), (2., 6., 4.))


def pair(seq=1, **kwargs):
    raw = Frame([[0., 0., 1.], [0., 0., 1.], [1., 2., 2.], [5., 0., 0.]],
                [11, 22, 33, 44], seq=seq, ts=1000. + seq, gen=3, packets=450 + seq)
    args = dict(channel="CH1", epoch=2, received_monotonic=100. + seq,
                complete=True, raw=raw, cropped=apply_roi(raw, ROI), roi=ROI)
    args.update(kwargs)
    return ObservationPair.freeze(**args)


class TestObservationPair(unittest.TestCase):
    def test_known_decode_loss_survives_snapshot_and_cannot_be_declared_complete(self):
        raw = pair().raw.frame()
        raw.dropped_packets = 7
        cropped = apply_roi(raw, ROI)
        snapshot = FrameSnapshot.freeze(cropped)
        self.assertEqual(7, snapshot.frame().dropped_packets)
        with self.assertRaisesRegex(ValueError, "解码损失"):
            pair(raw=raw, cropped=cropped)
        incomplete = pair(raw=raw, cropped=cropped, complete=False)
        with self.assertRaisesRegex(ValueError, "残缺"):
            ObservationWindow((replace(incomplete, complete=True),))

    def test_crop_preserves_exact_original_rows_not_coordinate_lookup(self):
        p = pair()
        np.testing.assert_array_equal(p.indices(), [0, 1, 2])
        np.testing.assert_array_equal(p.cropped.frame().ring, [11, 22, 33])
        self.assertEqual(p.raw.identity(), p.cropped.identity())

    def test_missing_raw_is_not_replaced_by_cropped_frame(self):
        with self.assertRaisesRegex(ValueError, "缺少同次"):
            pair(raw=None)

    def test_each_identity_field_must_match(self):
        for field, value in (("seq", 99), ("ts", 2000.), ("gen", 4), ("packets", 500)):
            with self.subTest(field=field):
                cropped = pair().cropped.frame()
                setattr(cropped, field, value)
                with self.assertRaisesRegex(ValueError, "不一致"):
                    pair(cropped=cropped)

    def test_ring_and_order_mismatch_rejected_even_with_equal_coordinates(self):
        p = pair()
        cropped = p.cropped.frame()
        cropped.ring = np.array([22, 11, 33], np.uint16)
        with self.assertRaisesRegex(ValueError, "精确 ROI"):
            pair(cropped=cropped)

    def test_snapshot_owns_bytes_and_cannot_be_reenabled_writable(self):
        raw = pair().raw.frame()
        raw = Frame(raw.xyz.copy(), raw.ring.copy(), raw.seq, raw.ts, raw.gen, raw.packets)
        p = pair(raw=raw)
        raw.xyz[:] = 9
        raw.ring[:] = 0
        self.assertEqual(p.raw.frame().xyz[0, 0], 0)
        self.assertEqual(p.raw.frame().ring[0], 11)
        with self.assertRaises(ValueError):
            p.raw.frame().xyz.setflags(write=True)
        with self.assertRaises(ValueError):
            p.indices().setflags(write=True)
        exposed = p.raw.frame()
        exposed.seq = 999
        self.assertEqual(1, p.raw.seq)

    def test_invalid_identity_quality_and_nonfinite_rejected(self):
        for kwargs in (dict(complete=1), dict(epoch=-1), dict(received_monotonic=float("nan"))):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                pair(**kwargs)
        for frame in (Frame([[0, 0, 0]]), Frame([[float("nan"), 0, 0]], seq=1, ts=1000)):
            with self.assertRaises(ValueError):
                FrameSnapshot.freeze(frame)

    def test_merged_segments_can_trace_every_input_row(self):
        window = ObservationWindow((pair(1), pair(2), pair(3)))
        merged = window.input_frame()
        self.assertEqual((merged.seq, merged.ts, merged.gen, merged.packets), (3, 1003., 3, 0))
        segments = window.segments()
        self.assertEqual([[0, 4], [4, 8], [8, 12]], [s["raw_rows"] for s in segments])
        self.assertEqual([[0, 3], [3, 6], [6, 9]], [s["input_rows"] for s in segments])
        self.assertEqual([451, 452, 453], [s["frame"]["packets"] for s in segments])
        lineage = window.detection_lineage(np.array([0, 1, 4, 8]))
        np.testing.assert_array_equal(lineage, [[0, 0], [0, 1], [1, 1], [2, 2]])
        for row, (member, original) in enumerate(window.detection_lineage(np.arange(merged.n))):
            raw = window.pairs[member].raw.frame()
            np.testing.assert_array_equal(merged.xyz[row], raw.xyz[original])
            self.assertEqual(merged.ring[row], raw.ring[original])

    def test_trimmed_window_restarts_offsets_without_old_raw(self):
        window = ObservationWindow((pair(2), pair(3)))
        self.assertEqual([0, 4], window.segments()[0]["raw_rows"])
        self.assertEqual(2, window.segments()[0]["frame"]["seq"])

    def test_mixed_epoch_channel_generation_incomplete_and_order_rejected(self):
        p, q = pair(1), pair(2)
        for bad in (replace(q, channel="CH2"), replace(q, epoch=4), replace(q, complete=False),
                    replace(q, raw=replace(q.raw, gen=4)), p,
                    replace(q, received_monotonic=p.received_monotonic),
                    replace(q, raw=replace(q.raw, ts=p.raw.ts))):
            with self.subTest(bad=bad.channel), self.assertRaises(ValueError):
                ObservationWindow((p, bad))
        for indices in (np.array([-1]), np.array([3]), np.array([1, 1]), np.array([1., 2.])):
            with self.assertRaises(ValueError):
                ObservationWindow((p,)).detection_lineage(indices)


class TestServiceObservationPair(unittest.TestCase):
    def setUp(self):
        self.clock = ObservationClock(self)
        st = load_settings("")
        ch = copy.deepcopy(st.channels[0])
        ch.mode = "synthetic"
        with patch("app.application.channel_worker.EmptyBaseline.load", return_value=None):
            self.worker = ChannelWorker(ch, EventBus(), st.plate, save_override=True,
                                        plate_mode="null", raw_frames=0, state_push_hz=0)
        self.worker.source.roi = ROI
        self.worker.stability.ground_z = 0.
        self.addCleanup(self.worker.stop)

    def test_save_keeps_full_primary_observation_with_zero_extra_raw(self):
        w = self.worker
        w.stability.vehicle_threshold = 1
        self.assertTrue(w.source.keep_raw)
        raw = pair().raw.frame()
        cropped = apply_roi(raw, ROI)
        w.source.read = lambda: cropped
        w.source._last_raw = raw
        with patch.object(w.source, "take_raw", wraps=w.source.take_raw) as take:
            w._tick()
            self.assertEqual(1, take.call_count)
            w._grab_raw(0)
            self.assertEqual(1, take.call_count)
        self.assertEqual(4, w._current_pair.raw.frame().n)
        self.assertEqual(3, w._current_pair.cropped.frame().n)
        self.assertEqual([], w._raw_shots)

    def test_diagnostic_sampling_does_not_consume_raw_a_second_time(self):
        w = self.worker
        w.raw_frames = 3
        raw = pair().raw.frame()
        w.source._last_raw = raw
        with patch.object(w.source, "take_raw", wraps=w.source.take_raw) as take:
            w._pair_observation(apply_roi(raw, ROI), True)
            w._grab_raw(0)
            w._grab_raw(1)
            self.assertEqual(1, take.call_count)
        self.assertEqual(raw.n, w._raw_shots[0][1].n)

    def test_missing_or_mismatched_raw_reports_failure_not_full_observation(self):
        w = self.worker
        p = pair()
        w._pair_observation(p.cropped.frame(), True)
        self.assertIsNone(w._current_pair)
        self.assertIn("缺少同次", w._pair_error)
        wrong = p.raw.frame()
        wrong.seq += 1
        w.source._last_raw = wrong
        w._pair_observation(p.cropped.frame(), True)
        self.assertIsNone(w._current_pair)
        self.assertIn("不一致", w._pair_error)

    def test_no_frame_clears_previous_pair_without_reading_stale_raw(self):
        w = self.worker
        w._current_pair = pair()
        w.source.read = lambda: None
        with patch.object(w.source, "take_raw", wraps=w.source.take_raw) as take:
            self.assertFalse(w._tick())
            self.assertEqual(0, take.call_count)
        self.assertIsNone(w._current_pair)

    def test_real_settle_freezes_detection_and_volume_source_windows_before_clear(self):
        w = self.worker
        rng = np.random.default_rng(331)
        xyz = np.column_stack((rng.uniform(-1., 1., 20000), rng.uniform(-5., 5., 20000),
                               rng.uniform(1., 2., 20000)))
        original = Frame(xyz, np.arange(len(xyz)))
        w.settle_sec = 1.
        w.settle_max_frames = 4
        snapshots = []

        def capture(frame, stab):
            window, support = w._measurement_window, w._volume_observation_window
            # ★ H1（2026-09-23）：检测执行时窗口必须**还活着** ——
            #   window_evidence / empty_evidence 读的正是 _settle_frames。
            #   旧时序先清后测，两个诊断在主链路上恒读到空列表；
            #   检测结束后才由 finally 收口（下面 224-225 行验证）。
            self.assertGreaterEqual(len(w._settle_pairs), 1)
            self.assertGreaterEqual(len(w._settle_frames), 1)
            self.assertEqual(4 if w.settle_accumulate else 3, len(support.pairs))
            self.assertEqual(len(w._vol_extra), len(w._volume_extra_members))
            for points, index in zip(w._vol_extra, w._volume_extra_members):
                member = support.pairs[index]
                np.testing.assert_array_equal(points, member.cropped.frame().xyz)
            np.testing.assert_array_equal(frame.xyz, window.input_frame().xyz)
            snapshots.append((window, support))
            w._detected_this_session += 1

        w._run_detection = capture
        for accumulate in (False, True):
            w.force_new_session()
            w.settle_accumulate = accumulate
            before = len(snapshots)
            for _ in range(45):
                self.clock.tick(w, original, dt=.1)
            self.assertEqual(before + 1, len(snapshots))
            window, support = snapshots[-1]
            self.assertEqual(4 if accumulate else 1, len(window.pairs))
            self.assertIsNone(w._measurement_window)
            self.assertIsNone(w._volume_observation_window)
            self.assertEqual(support.pairs[-1].raw.seq, window.pairs[-1].raw.seq)


class TestArchivedObservationLineage(unittest.TestCase):
    @staticmethod
    def bundle(**override):
        window = ObservationWindow((pair(1), pair(2)))
        source = window.input_frame()
        selected = np.array([0, 1, 4, 5])
        angle = .05
        matrix = np.array([[np.cos(angle), -np.sin(angle), 0.],
                           [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        detection = Frame(source.xyz.astype(np.float64)[selected] @ matrix.T, source.ring[selected],
                          source.seq, source.ts, source.gen, source.packets)
        args = dict(measurement_id="m-trace", channel="CH1", passage_id="p-trace",
                    observation=window.pairs[-1].raw.frame(), detection=detection,
                    vehicle_indices=np.array([0, 1, 2]), carriage_indices=np.array([0, 1, 2]),
                    result={"empty": True}, roi={"carriage_status": "partial"},
                    configuration_hash="b" * 64, algorithm_version="test-v1", complete=True, measurable=True,
                    observation_window=window, detection_indices=selected, detection_rotation=matrix)
        args.update(override)
        return freeze_bundle(**args)

    def test_serialized_multi_frame_sources_and_lineage_validate(self):
        bundle = self.bundle()
        meta = bundle.validate()
        self.assertEqual("concatenate", meta["provenance"]["aggregation"])
        self.assertEqual(6, meta["provenance"]["input_points"])
        self.assertEqual([451, 452], [s["frame"]["packets"] for s in meta["provenance"]["segments"]])
        mapping = np.load(io.BytesIO(dict(bundle.files)["detection.lineage.npy"]), allow_pickle=False)
        np.testing.assert_array_equal(mapping, [[0, 0], [0, 1], [1, 1], [1, 2]])

    def test_missing_transform_or_wrong_primary_cannot_claim_lineage(self):
        for override in (dict(detection_rotation=None), dict(observation=pair(1).raw.frame()),
                         dict(detection_rotation=np.eye(3)), dict(channel="CH2")):
            with self.subTest(keys=list(override)), self.assertRaises(ValueError):
                self.bundle(**override)

    def test_volume_support_is_independent_of_detection_aggregation(self):
        window = ObservationWindow((pair(1), pair(2)))
        bundle = self.bundle(volume_window=window, volume_extra_members=(0,))
        meta = bundle.validate()
        self.assertEqual(2, len(meta["volume_support"]["segments"]))
        self.assertIn("volume-source-000.xyz", dict(bundle.files))
        self.assertEqual([0, 1], meta["volume_support"]["primary_members"])
        self.assertEqual([0], meta["volume_support"]["extra_members"])
        with self.assertRaisesRegex(ValueError, "遗漏"):
            self.bundle(volume_window=ObservationWindow((pair(1),)), volume_extra_members=())

    def test_volume_and_detection_windows_cannot_cross_epoch_or_reception(self):
        p, q = pair(1), pair(2)
        for support in ((replace(p, epoch=99), replace(q, epoch=99)),
                        (p, replace(q, received_monotonic=q.received_monotonic + .1))):
            with self.assertRaisesRegex(ValueError, "活动观测窗口"):
                self.bundle(volume_window=ObservationWindow(support), volume_extra_members=(0,))
        altered_raw = p.raw.frame()
        altered_raw = Frame(altered_raw.xyz, altered_raw.ring + 1, altered_raw.seq,
                            altered_raw.ts, altered_raw.gen, altered_raw.packets)
        changed = pair(1, raw=altered_raw, cropped=apply_roi(altered_raw, ROI))
        with self.assertRaisesRegex(ValueError, "重复成员"):
            self.bundle(volume_window=ObservationWindow((changed, q)), volume_extra_members=(0,))

    def test_volume_member_roles_are_explicit_and_cannot_omit_merged_primary(self):
        window = ObservationWindow((pair(1), pair(2)))
        for extra in (None, (2,), (0, 0), (True,)):
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                self.bundle(volume_window=window, volume_extra_members=extra)
        bundle = self.bundle(volume_window=window, volume_extra_members=(0,))
        meta = bundle.metadata
        meta["volume_support"]["primary_members"] = [1]
        tampered = FrozenBundle(json.dumps(meta, ensure_ascii=False, sort_keys=True, separators=(",", ":"),
                                           allow_nan=False).encode("utf-8"), bundle.files)
        with self.assertRaisesRegex(ValueError, "主输入"):
            tampered.validate()

    def test_modified_provenance_rejected_even_after_rehashing_manifest(self):
        bundle = self.bundle()
        for member in ("raw_rows", "input_rows", "complete", "channel", "frame"):
            meta = bundle.metadata
            segment = meta["provenance"]["segments"][0]
            segment[member] = {"raw_rows": [0, 99], "input_rows": [0, 99], "complete": False,
                               "channel": "CH2", "frame": dict(seq=100, ts=1001., gen=3, packets=451)}[member]
            tampered = FrozenBundle(json.dumps(meta, ensure_ascii=False, sort_keys=True, separators=(",", ":"),
                                               allow_nan=False).encode("utf-8"), bundle.files)
            with self.subTest(member=member), self.assertRaises(ValueError):
                tampered.validate()

    def test_modified_original_ring_with_valid_hash_still_breaks_detection_lineage(self):
        bundle = self.bundle()
        meta, files = bundle.metadata, dict(bundle.files)
        data = bytearray(files["source-000.xyz"])
        data[16:18] = b"\x00\x00"
        files["source-000.xyz"] = bytes(data)
        meta["files"]["source-000.xyz"]["sha256"] = hashlib.sha256(data).hexdigest()
        tampered = FrozenBundle(json.dumps(meta, ensure_ascii=False, sort_keys=True, separators=(",", ":"),
                                           allow_nan=False).encode("utf-8"), tuple(sorted(files.items())))
        with self.assertRaisesRegex(ValueError, "ring"):
            tampered.validate()

    def test_actual_detector_y_trim_and_rotation_have_exact_input_indices(self):
        from app.detect import detect_empty
        from tests import _helpers as H
        cfg = copy.deepcopy(H.ctx()[2])
        cfg.floor.y_trim_gap_m = 1.
        xyz = H.grid_plane(n=35, extent=5, z=-3.)
        frame = Frame(xyz, np.arange(len(xyz)), seq=1, ts=1000.)
        with patch("app.detect.carriage_y_window", return_value=(-3., 3.)):
            result = detect_empty(frame, cfg)
        self.assertIsNotNone(result)
        self.assertGreater(len(result.source_indices), 0)
        self.assertLess(len(result.source_indices), frame.n)
        expected = frame.xyz.astype(np.float64)[result.source_indices] @ result.source_rotation.T
        np.testing.assert_array_equal(expected, result.cloud)


if __name__ == "__main__":
    unittest.main()
