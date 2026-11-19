"""从不可信文件重建原输入，含重复坐标、缺 ring、混合用途及重哈希攻击。"""
import contextlib
from dataclasses import replace
import hashlib
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from app import archive_reader as reader
from app.cloud_archive import FrozenBundle, freeze_bundle
from app.cloud_observation import FrameSnapshot, ObservationPair, ObservationWindow
from app.pointcloud import Frame, apply_roi
from app.sample_paths import Sample, discover_samples
from tests.test_cloud_archive import example_bundle
from tests.test_sample_paths import publish


def make_inputs(count=3, rings=None, *, volume=True, extra=None):
    pairs = []
    roi = ((-2., -6., -4.), (2., 6., 4.))
    for index in range(count):
        ring = [11, 22, 33, 44] if rings is None else rings[index]
        raw = Frame([[0., 0., 1.], [0., 0., 1.], [1., index, 2.], [5., 0., 0.]],
                    ring, seq=index + 1, ts=1001. + index, gen=3, packets=451 + index)
        pairs.append(ObservationPair.freeze(channel="CH1", epoch=2, received_monotonic=100. + index,
                                            complete=True, raw=raw, cropped=apply_roi(raw, roi), roi=roi))
    window = ObservationWindow(tuple(pairs))
    source = window.input_frame()
    selected = np.arange(0, source.n, 2)
    angle = .05
    matrix = np.array([[np.cos(angle), -np.sin(angle), 0.],
                       [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
    detection = Frame(source.xyz.astype(np.float64)[selected] @ matrix.T,
                      None if source.ring is None else source.ring[selected],
                      source.seq, source.ts, source.gen, source.packets)
    bundle = freeze_bundle(measurement_id="m-input", channel="CH1", passage_id="p-input",
                           observation=pairs[-1].raw.frame(), detection=detection,
                           vehicle_indices=np.array([0, 1, 2]), carriage_indices=np.array([0, 1]),
                           result={"empty": True}, roi={"carriage_status": "partial"},
                           configuration_hash="c" * 64, algorithm_version="test-v1", complete=True, measurable=True,
                           observation_window=window, detection_indices=selected, detection_rotation=matrix,
                           volume_window=window if volume else None,
                           volume_extra_members=(tuple(range(count - 1)) if extra is None else extra) if volume else None)
    return bundle, window


def mutate(bundle, *, name=None, data=None, change=None):
    meta, files = bundle.metadata, dict(bundle.files)
    if name is not None:
        files[name] = data
        meta["files"][name].update(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
    if change:
        change(meta)
    manifest = json.dumps(meta, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return FrozenBundle(manifest, tuple(sorted(files.items())))


def npy(value):
    out = io.BytesIO()
    np.save(out, value, allow_pickle=False)
    return out.getvalue()


class TestArchiveInputs(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="archive-inputs-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.serial = 0

    def sample(self, bundle):
        self.serial += 1
        directory = self.root / str(self.serial)
        publish(directory, bundle)
        meta = bundle.metadata
        return Sample(str(directory / "observation.xyz"), "observation", meta["channel"], meta["measurement_id"],
                      meta["acquired_at"], hashlib.sha256(bundle.manifest).hexdigest())

    def test_single_and_multiframe_primary_bytes_and_identity(self):
        for count in (1, 3):
            bundle, window = make_inputs(count)
            sample = self.sample(bundle)
            discovered = discover_samples(str(Path(sample.path).parent)).samples
            self.assertEqual(1, len(discovered))
            loaded = reader.load_archive(discovered[0])
            self.assertEqual(window, loaded.window())
            self.assertEqual(FrameSnapshot.freeze(window.input_frame()), loaded.primary_input())
            self.assertEqual(451 if count == 1 else 0, loaded.primary_input().packets)
            self.assertEqual(loaded, reader.load_archive(discovered[0]))

    def test_duplicate_coordinates_keep_original_rows_and_rings(self):
        bundle, _ = make_inputs()
        loaded = reader.load_archive(self.sample(bundle))
        frame = loaded.primary_input().frame()
        np.testing.assert_array_equal(frame.xyz[0], frame.xyz[1])
        np.testing.assert_array_equal(frame.ring, [11, 22, 33] * 3)
        self.assertEqual(9, frame.n)

    def test_missing_mixed_and_true_zero_rings_distinct(self):
        for rings in ([None] * 3, [None, [0] * 4, [0] * 4], [[0] * 4] * 3):
            bundle, window = make_inputs(rings=rings)
            loaded = reader.load_archive(self.sample(bundle))
            self.assertEqual(window, loaded.window())
            self.assertEqual(FrameSnapshot.freeze(window.input_frame()), loaded.primary_input())
            self.assertEqual(rings[-1] is None, loaded.role("observation").ring_bytes is None)

    def test_primary_extra_role_overlap_preserves_sequence(self):
        bundle, window = make_inputs(extra=(0, 2))
        loaded = reader.load_archive(self.sample(bundle))
        self.assertEqual((window.pairs[0].cropped, window.pairs[2].cropped), loaded.volume_extra_inputs())
        empty, _ = make_inputs(extra=())
        self.assertEqual((), reader.load_archive(self.sample(empty)).volume_extra_inputs())

    def test_detection_is_not_primary_or_original_loaded_cloud(self):
        bundle, window = make_inputs()
        loaded = reader.load_archive(self.sample(bundle))
        self.assertNotEqual(loaded.role("detection").xyz_bytes, loaded.primary_input().xyz_bytes)
        self.assertFalse(loaded.capabilities["original_loaded_cloud"])
        self.assertFalse(loaded.capabilities["original_parameters"])
        self.assertFalse(loaded.capabilities["standstill_timeline"])
        self.assertEqual(9, window.input_frame().n)
        self.assertEqual(5, loaded.role("detection").frame().n)

    def test_missing_provenance_or_support_cannot_claim_exact_inputs(self):
        loaded = reader.load_archive(self.sample(example_bundle()))
        self.assertEqual(4, loaded.role("observation").frame().n)
        self.assertFalse(loaded.capabilities["primary_input"])
        with self.assertRaisesRegex(ValueError, "provenance"):
            loaded.primary_input()
        bundle, _ = make_inputs(volume=False)
        loaded = reader.load_archive(self.sample(bundle))
        self.assertTrue(loaded.capabilities["primary_input"])
        self.assertFalse(loaded.capabilities["volume_extra_inputs"])
        with self.assertRaisesRegex(ValueError, "volume_support"):
            loaded.volume_extra_inputs()

    def test_snapshots_and_metadata_do_not_mutate_verified_bytes(self):
        loaded = reader.load_archive(self.sample(make_inputs()[0]))
        meta = loaded.metadata
        meta["channel"] = "CH9"
        self.assertEqual("CH1", loaded.metadata["channel"])
        for array in (loaded.primary_input().frame().xyz, loaded.role("vehicle").frame().ring,
                      loaded.window().pairs[0].indices()):
            with self.assertRaises(ValueError):
                array.setflags(write=True)
        frame = loaded.primary_input().frame()
        frame.seq = 999
        self.assertEqual(3, loaded.primary_input().seq)

    def test_rehashed_semantic_corruption_rejected(self):
        bundle, _ = make_inputs()
        cases = [
            mutate(bundle, name="vehicle.indices.npy", data=npy(np.array([999999]))),
            mutate(bundle, name="source-000.input-indices.npy", data=dict(bundle.files)["source-000.input-indices.npy"] + b"tail"),
            mutate(bundle, name="carriage.indices.npy", data=npy(np.array([1, 0]))),
            mutate(bundle, name="detection.lineage.npy", data=npy(np.zeros((5, 2), dtype=np.int64))),
            mutate(bundle, change=lambda m: m["files"]["source-000.xyz"].update(ring_present="yes")),
            mutate(bundle, change=lambda m: m["files"]["source-000.xyz"].update(coordinates="other")),
            mutate(bundle, change=lambda m: m["files"]["source-000.xyz"].update(role="diagnostic_observation")),
            mutate(bundle, change=lambda m: m["provenance"]["segments"][0].update(channel="CH2")),
            mutate(bundle, change=lambda m: m["provenance"]["segments"][0].update(epoch=True)),
            mutate(bundle, change=lambda m: m["provenance"]["segments"][1]["frame"].update(gen=9)),
            mutate(bundle, change=lambda m: m["provenance"]["segments"][1].update(received_monotonic=100.)),
            mutate(bundle, change=lambda m: m.update(empty_observation=1)),
        ]
        for index, candidate in enumerate(cases):
            with self.subTest(index=index), self.assertRaises(ValueError):
                reader.load_archive(self.sample(candidate))

    def test_rehashed_role_subsets_and_ring_presence_must_agree(self):
        bundle, _ = make_inputs(rings=[[0] * 4] * 3)
        for candidate in (
            mutate(bundle, name="vehicle.indices.npy", data=npy(np.array([0, 1]))),
            mutate(bundle, change=lambda m: m["files"]["observation.xyz"].update(ring_present=False)),
            mutate(bundle, change=lambda m: m["files"]["source-002.xyz"].update(ring_present=False)),
            mutate(bundle, change=lambda m: m["files"]["vehicle.xyz"].update(ring_present=False)),
        ):
            with self.assertRaises(ValueError):
                reader.load_archive(self.sample(candidate))

    def test_discovery_then_manifest_or_member_swap_rejected(self):
        bundle, _ = make_inputs()
        sample = self.sample(bundle)
        directory = Path(sample.path).parent
        (directory / "manifest.json").write_bytes(mutate(bundle, change=lambda m: m.update(algorithm_version="other")).manifest)
        with self.assertRaisesRegex(ValueError, "发现后"):
            reader.load_archive(sample)
        (directory / "manifest.json").write_bytes(bundle.manifest)
        (directory / "source-000.xyz").write_bytes(dict(bundle.files)["source-001.xyz"])
        with self.assertRaisesRegex(ValueError, "摘要"):
            reader.load_archive(sample)

    def test_manifest_changed_during_load_rejected(self):
        sample = self.sample(make_inputs()[0])
        original = reader._read_regular
        def change_after_member(path, *args, **kwargs):
            data = original(path, *args, **kwargs)
            if path.name == "source-000.xyz":
                (path.parent / "manifest.json").write_bytes(b"{}")
            return data
        with patch.object(reader, "_read_regular", side_effect=change_after_member), self.assertRaises(ValueError):
            reader.load_archive(sample)

    def test_identity_and_wrong_role_binding_rejected(self):
        sample = self.sample(make_inputs()[0])
        for candidate in (replace(sample, channel="CH2"), replace(sample, measurement_id="other"),
                          replace(sample, acquired_at=1.), replace(sample, role="detection"),
                          replace(sample, manifest_sha256="")):
            with self.assertRaises(ValueError):
                reader.load_archive(candidate)

    def test_budget_refuses_before_numpy_load_or_reading_members(self):
        sample = self.sample(make_inputs()[0])
        for field in ("file_bytes", "total_bytes", "members", "segments", "points", "working_bytes"):
            with self.subTest(field=field), patch("numpy.load", side_effect=AssertionError("不应进入解码")):
                with self.assertRaisesRegex(ValueError, "预算"):
                    reader.load_archive(sample, limits=replace(reader.ArchiveReadLimits(), **{field: 1}))
        with self.assertRaises(ValueError):
            reader.ArchiveReadLimits(points=True)

    def test_npy_shape_object_tail_and_truncation_fail_before_numpy_load(self):
        bundle, _ = make_inputs()
        head = b"{'descr': '<i8', 'fortran_order': False, 'shape': (99999999999999999999,), }\n"
        hostile = b"\x93NUMPY\x01\x00" + len(head).to_bytes(2, "little") + head
        for data in (hostile, npy(np.array([1.])), npy(np.array([1]))[:-1],
                     npy(np.array([1])) + b"tail", b"\x93NUMPY\x01\x00"):
            with patch("numpy.load", side_effect=AssertionError("不应进入解码")), self.assertRaises(ValueError):
                reader.load_archive(self.sample(mutate(bundle, name="vehicle.indices.npy", data=data)))

    def test_discovery_budget_checked_before_opening_any_member(self):
        from app import sample_paths
        bundle, _ = make_inputs()
        changed = mutate(bundle, change=lambda m: m["files"]["carriage.xyz"].update(points=20_000_000, bytes=280_000_004))
        sample = self.sample(changed)
        original = sample_paths._read_regular
        opened = []
        def only_manifest(path, *args, **kwargs):
            opened.append(path.name)
            self.assertEqual("manifest.json", path.name)
            return original(path, *args, **kwargs)
        with patch.object(sample_paths, "_read_regular", side_effect=only_manifest):
            found = discover_samples(str(Path(sample.path).parent))
        self.assertEqual(["manifest.json"], opened)
        self.assertEqual([], found.samples)
        self.assertIn("预算", found.errors[0])

    def test_result_json_and_available_identity_fields_always_checked(self):
        bundle, _ = make_inputs()
        for data in (b"not JSON", b"[]", b'{"channel":"CH9","empty":true}',
                     b'{"empty":true,"seq":9}', b'{"empty":1}', b'{"reason":"different"}'):
            candidate = mutate(bundle, name="detection.xyz.meta.json", data=data,
                               change=lambda m: m.update(empty_observation=False))
            with self.subTest(data=data), self.assertRaises(ValueError):
                reader.load_archive(self.sample(candidate))

    def test_actual_service_generation_field_must_match_manifest(self):
        bundle, _ = make_inputs()
        for value in (999, True, 3.0, "3"):
            data = json.dumps({"empty": True, "source_generation": value}, sort_keys=True, separators=(",", ":")).encode()
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "代次"):
                reader.load_archive(self.sample(mutate(bundle, name="detection.xyz.meta.json", data=data)))
        data = b'{"empty":true,"gen":3,"source_generation":3}'
        loaded = reader.load_archive(self.sample(mutate(bundle, name="detection.xyz.meta.json", data=data)))
        self.assertEqual(3, loaded.primary_input().gen)

    def test_json_working_depth_and_node_budgets_before_parse(self):
        from app import archive_io
        cases = [(b'{"padding":' + b"[" * 100 + b"0" + b"]" * 100 + b"}", reader.ArchiveReadLimits()),
                 (b'{"padding":' + b"9" * 500 + b"}", reader.ArchiveReadLimits()),
                 (b'{"padding":[' + b"{}," * 30000 + b"{}]}", replace(reader.ArchiveReadLimits(), working_bytes=256 * 1024)),
                 (b'{"padding":[{},{},{}]}', replace(reader.ArchiveReadLimits(), json_nodes=2))]
        for data, limits in cases:
            with patch.object(archive_io.json, "loads", side_effect=AssertionError("不应解析超预算 JSON")):
                with self.assertRaisesRegex(ValueError, "预算"):
                    archive_io.bounded_json(data, limits, manifest=True)
        for encoding in ("utf-16", "utf-16le", "utf-16be", "utf-32", "utf-32le", "utf-8-sig"):
            data = ('{"padding":' + "9" * 500 + "}").encode(encoding)
            with self.subTest(encoding=encoding), patch.object(archive_io.json, "loads", side_effect=AssertionError("不应自动检测编码")):
                with self.assertRaises(ValueError):
                    archive_io.bounded_json(data, reader.ArchiveReadLimits(), manifest=True)
        bundle, _ = make_inputs()
        data = b'{"padding":[' + b"{}," * 30000 + b"{}]}"
        sample = self.sample(mutate(bundle, name="detection.xyz.meta.json", data=data))
        with self.assertRaisesRegex(ValueError, "预算"):
            reader.load_archive(sample, limits=replace(reader.ArchiveReadLimits(), working_bytes=1024 * 1024))

    def test_hardlinks_and_nonregular_members_rejected(self):
        sample = self.sample(make_inputs()[0])
        path = Path(sample.path)
        os.link(str(path), str(path.parent / "alias.xyz"))
        with self.assertRaisesRegex(ValueError, "单链接"):
            reader.load_archive(sample)
        sample = self.sample(make_inputs()[0])
        path = Path(sample.path)
        path.unlink()
        path.mkdir()
        with self.assertRaisesRegex(ValueError, "普通"):
            reader.load_archive(sample)

    def test_symlink_rejected(self):
        sample = self.sample(make_inputs()[0])
        link = self.root / "link"
        try:
            link.symlink_to(Path(sample.path).parent, target_is_directory=True)
        except OSError as exc:
            self.skipTest("本机无创建符号链接权限: " + str(exc))
        with self.assertRaisesRegex(ValueError, "链接"):
            reader.load_archive(replace(sample, path=str(link / "observation.xyz")))

    def test_replay_source_loads_verified_bytes_preserves_missing_ring(self):
        from app.radar_source import ReplaySource
        bundle, _ = make_inputs(rings=[None] * 3)
        sample = self.sample(bundle)
        source = ReplaySource(str(Path(sample.path).parent), interval=0, loop=False)
        self.assertIsNone(source.read().ring)
        source.rewind()
        (Path(sample.path).parent / "source-000.xyz").write_bytes(b"broken")
        self.assertIsNone(source.read())
        self.assertEqual(1, source.stats.errors)

    def test_explicit_input_replay_no_second_crop_no_false_regression(self):
        from cli import replay
        bundle, window = make_inputs()
        sample = self.sample(bundle)
        report = self.root / "report.json"
        def check_input(frame, calib, dev, use_roi):
            self.assertFalse(use_roi)
            self.assertEqual(FrameSnapshot.freeze(window.input_frame()), FrameSnapshot.freeze(frame))
            return True, "诊断", None, frame, None, ""
        with patch.object(replay, "detect_one", side_effect=check_input) as detect, contextlib.redirect_stdout(io.StringIO()):
            result = replay.main(["--dir", str(Path(sample.path).parent), "--archive-input", "--json", str(report)])
        self.assertEqual(0, result)
        detect.assert_called_once()
        data = json.loads(report.read_text(encoding="utf-8"))
        self.assertEqual("not_run", data["comparison_status"])
        self.assertEqual("original_primary_roi", data["rows"][0]["input_kind"])
        self.assertEqual(3, data["rows"][0]["archive"]["primary_members"])

    def test_input_replay_missing_lineage_is_error_not_last_frame_fallback(self):
        from cli import replay
        sample = self.sample(example_bundle())
        with patch.object(replay, "detect_one") as detect, contextlib.redirect_stdout(io.StringIO()):
            result = replay.main(["--dir", str(Path(sample.path).parent), "--archive-input"])
        self.assertEqual(1, result)
        detect.assert_not_called()


if __name__ == "__main__":
    unittest.main()
