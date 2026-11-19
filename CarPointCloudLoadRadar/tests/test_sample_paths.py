"""多角色归档只计一次；混合格式、破包及真实消费者路径，全部在临时目录。"""
import contextlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from app.settings import BASELINE_SUBDIR
from app.cloud_archive import freeze_bundle
from app.cloud_observation import ObservationWindow
from app.pointcloud import Frame, load_xyz, save_xyz
from app.sample_paths import discover_samples, REPLAY_NOTE
from tests.test_cloud_archive import example_bundle
from tests.test_cloud_observation import pair


def publish(directory, bundle):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for name, data in bundle.files:
        (directory / name).write_bytes(data)
    (directory / "manifest.json").write_bytes(bundle.manifest)
    return directory


def rewrite_manifest(directory, mutate):
    path = directory / "manifest.json"
    meta = json.loads(path.read_bytes())
    mutate(meta)
    path.write_text(json.dumps(meta, ensure_ascii=False, sort_keys=True, separators=(",", ":")), encoding="utf-8")


class _SampleFixture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="sample-paths-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def package(self, name="p1", measurement="m1", **kwargs):
        return publish(self.root / "archives-v1" / name, example_bundle(measurement, **kwargs))

    def legacy(self, relative="legacy/sample.xyz"):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        save_xyz(str(path), Frame(np.zeros((3, 3)), np.array([1, 2, 3])))
        return path


class TestSamplePaths(_SampleFixture):

    def test_mixed_two_packages_legacy_and_half_is_three_logical_samples(self):
        legacy = self.legacy()
        late = self.package("a", "later", ts=2000.)
        early = self.package("z", "earlier", ts=1000.)
        half = self.package("half", "half")
        (half / "manifest.json").unlink()
        found = discover_samples(str(self.root))
        self.assertEqual([str(legacy), str(early / "detection.xyz"), str(late / "detection.xyz")],
                         [sample.path for sample in found.samples])
        self.assertEqual(1, len(found.errors))
        self.assertIn("未发布", found.errors[0])

    def test_root_archive_and_package_entry_select_same_role(self):
        package = self.package()
        for root in (self.root, self.root / "archives-v1", package):
            for role in ("observation", "detection", "vehicle", "carriage"):
                found = discover_samples(str(root), role=role)
                self.assertFalse(found.errors)
                self.assertEqual([str(package / (role + ".xyz"))], [sample.path for sample in found.samples])

    def test_limit_is_after_logical_selection_and_time_sort(self):
        self.package("a", "late", ts=2000.)
        early = self.package("z", "early")
        found = discover_samples(str(self.root), role="observation", limit=1)
        self.assertEqual([str(early / "observation.xyz")], [sample.path for sample in found.samples])
        with self.assertRaises(ValueError):
            discover_samples(str(self.root), limit=-1)

    def test_all_declared_roles_are_verified_not_just_selected_xyz(self):
        package = self.package()
        path = package / "carriage.xyz"
        original = path.read_bytes()
        path.write_bytes(original[:-1] + bytes([original[-1] ^ 1]))
        found = discover_samples(str(self.root), role="observation")
        self.assertEqual([], found.samples)
        self.assertIn("摘要", found.errors[0])

    def test_missing_reason_or_corrupt_manifest_does_not_fall_back(self):
        package = self.package()
        (package / "detection.xyz.reason.txt").unlink()
        self.assertFalse(discover_samples(str(self.root)).samples)
        (package / "manifest.json").write_bytes(b"{")
        found = discover_samples(str(self.root))
        self.assertFalse(found.samples)
        self.assertEqual(1, len(found.errors))

    def test_copied_package_outside_reserved_tree_and_half_package(self):
        package = publish(self.root / "copied", example_bundle())
        self.assertEqual(1, len(discover_samples(str(self.root)).samples))
        (package / "manifest.json").unlink()
        self.assertFalse(discover_samples(str(self.root)).samples)

    def test_copied_partial_single_role_with_archive_markers_never_falls_back(self):
        package = publish(self.root / "copied", example_bundle())
        for name in ("manifest.json", "vehicle.xyz", "carriage.xyz", "detection.xyz"):
            (package / name).unlink()
        found = discover_samples(str(self.root))
        self.assertFalse(found.samples)
        self.assertIn("未发布", found.errors[0])

    def test_role_coordinate_and_header_mismatch_rejected(self):
        package = self.package()
        rewrite_manifest(package, lambda meta: meta["files"]["observation.xyz"].update(coordinates="detection"))
        self.assertFalse(discover_samples(str(self.root)).samples)
        publish(package, example_bundle("m1"))
        rewrite_manifest(package, lambda meta: meta["files"]["observation.xyz"].update(points=999))
        self.assertFalse(discover_samples(str(self.root)).samples)

    def test_traversal_filename_rejected_before_external_read(self):
        package = self.package()
        rewrite_manifest(package, lambda meta: meta["files"].update({"../outside": {"bytes": 0, "sha256": "a" * 64}}))
        found = discover_samples(str(self.root))
        self.assertFalse(found.samples)
        self.assertIn("文件名", found.errors[0])

    def test_duplicate_copy_counts_once_conflicting_identity_excludes_both(self):
        first = self.package("a")
        shutil.copytree(first, self.root / "archives-v1" / "b")
        self.assertEqual(1, len(discover_samples(str(self.root)).samples))
        self.package("b", reason="不同结果")
        found = discover_samples(str(self.root))
        self.assertFalse(found.samples)
        self.assertIn("冲突", found.errors[0])

    def test_legacy_filename_order_and_raw_exclusion_preserved(self):
        z = self.legacy("z/x.xyz")
        a = self.legacy("a/x.xyz")
        raw = self.legacy("raw/CH1/x.xyz")
        self.assertEqual([str(a), str(raw), str(z)], [s.path for s in discover_samples(str(self.root)).samples])
        found = discover_samples(str(self.root), exclude_raw=True)
        self.assertEqual([str(a), str(z)], [s.path for s in found.samples])

    def test_direct_raw_directory_entry_still_excluded_from_labels(self):
        from cli import label_truth
        self.legacy("raw/CH1/x.xyz")
        for directory in (self.root, self.root / "raw", self.root / "raw" / "CH1"):
            self.assertEqual([], label_truth.scan(str(directory)))

    def test_service_replay_channel_filter_before_limit_preserves_legacy(self):
        from app.radar_source import build_source
        from app.settings import ChannelCfg, load_calib
        from tests._helpers import first_channel
        self.package("a", "m2", channel="CH2", ts=900.)
        first = self.package("z", "m1", channel="CH1", ts=1000.)
        found = discover_samples(str(self.root), role="observation", archive_channel="CH1", limit=1)
        self.assertEqual([str(first / "observation.xyz")], [sample.path for sample in found.samples])
        _, configured = first_channel()
        calib = load_calib(configured.abs_config_path)
        ch = ChannelCfg(id="CH1", mode="replay")
        ch.abs_replay_dir, ch.replay_interval, ch.replay_loop = str(self.root), 0, False
        source = build_source(ch, calib, None, mode="replay")
        self.assertEqual([str(first / "observation.xyz")], source.files)
        self.assertEqual("CH1", source.info()["archive_channel"])
        source.roi = None
        np.testing.assert_array_equal(load_xyz(str(first / "observation.xyz")).xyz, source.read().xyz)
        self.assertIsNone(source.read())
        legacy = self.legacy()
        source = build_source(ch, calib, None, mode="replay")
        self.assertEqual([str(legacy), str(first / "observation.xyz")], source.files)

    def test_pattern_cannot_promote_extra_role(self):
        self.package()
        self.assertFalse(discover_samples(str(self.root), role="observation", pattern="**/carriage.xyz").samples)
        with self.assertRaises(ValueError):
            discover_samples(str(self.root), pattern="../*.xyz")

    def test_case_variant_archive_name_does_not_expose_partial_xyz(self):
        directory = self.root / "ARCHIVES-V1" / "half"
        directory.mkdir(parents=True)
        save_xyz(str(directory / "observation.xyz"), Frame(np.zeros((2, 3))))
        found = discover_samples(str(directory.parent))
        self.assertEqual([], found.samples)
        self.assertIn("未发布", found.errors[0])

    def test_custom_pattern_selects_duplicate_copy_in_named_directory(self):
        first = self.package("a")
        copied = self.root / "archives-v1" / "b"
        shutil.copytree(first, copied)
        found = discover_samples(str(self.root), pattern="**/b/detection.xyz")
        self.assertFalse(found.errors)
        self.assertEqual([str(copied / "detection.xyz")], [sample.path for sample in found.samples])

    def test_glob_pattern_legacy_depth_and_hidden_behavior_preserved(self):
        import glob
        for relative in ("root.xyz", "a/x.xyz", "a/b/y.xyz", ".hidden/z.xyz", "a/.secret.xyz"):
            self.legacy(relative)
        for pattern in ("**/*.xyz", "*.xyz", "a/*.xyz", "a/**/*.xyz", ".hidden/*.xyz"):
            with self.subTest(pattern=pattern):
                expected = sorted(glob.glob(str(self.root / pattern), recursive=True))
                found = discover_samples(str(self.root), pattern=pattern)
                self.assertEqual(expected, [sample.path for sample in found.samples])

    def test_original_regression_directory_still_discovers_ten(self):
        directory = Path(__file__).resolve().parents[1].joinpath(*BASELINE_SUBDIR)
        self.assertEqual(10, len(discover_samples(str(directory)).samples))

    def test_stdlib_discovery_does_not_import_numpy(self):
        script = "import sys; from app.sample_paths import discover_samples; discover_samples(sys.argv[1]); assert 'numpy' not in sys.modules"
        self.package()
        proc = subprocess.run([sys.executable, "-S", "-c", script, str(self.root)],
                              cwd=Path(__file__).resolve().parents[1], capture_output=True, timeout=30)
        self.assertEqual(0, proc.returncode, proc.stderr.decode("utf-8", "replace"))

    def test_symlink_member_not_followed(self):
        package = self.package()
        path = package / "carriage.xyz"
        # 外部目标必须在发现根之外；否则它本身会被合法识别为一个 legacy 样本。
        outside = tempfile.TemporaryDirectory(prefix="sample-paths-external-")
        self.addCleanup(outside.cleanup)
        target = Path(outside.name) / "external.xyz"
        target.write_bytes(path.read_bytes())
        path.unlink()
        try:
            path.symlink_to(target)
        except OSError:
            self.skipTest("当前 Windows 账号没有创建符号链接权限")
        found = discover_samples(str(self.root))
        self.assertFalse(found.samples)
        self.assertIn("链接", found.errors[0])

    def test_actual_multiframe_package_has_one_observation_and_visible_limit(self):
        window = ObservationWindow((pair(1), pair(2)))
        raw = window.pairs[-1].raw.frame()
        frame = window.input_frame()
        bundle = freeze_bundle(measurement_id="multi", channel="CH1", passage_id="p1", observation=raw,
                               detection=frame, vehicle_indices=np.arange(raw.n), carriage_indices=np.arange(raw.n),
                               result={"empty": False, "reason": "载货"}, roi={"carriage_status": "partial"},
                               configuration_hash="a" * 64, algorithm_version="test", complete=True, measurable=True,
                               extra_frames=(raw,), observation_window=window, detection_indices=np.arange(frame.n),
                               detection_rotation=np.eye(3), volume_window=window, volume_extra_members=())
        package = publish(self.root / "archives-v1" / "multi", bundle)
        from app.radar_source import ReplaySource
        source = ReplaySource(str(self.root), interval=0, loop=False)
        self.assertEqual([str(package / "observation.xyz")], source.files)
        self.assertEqual(REPLAY_NOTE, source.info()["replay_note"])
        self.assertEqual(2, discover_samples(str(self.root)).samples[0].window_members)
        observed = source.read()
        self.assertEqual(raw.n, observed.n)
        self.assertNotEqual(frame.n, observed.n)
        self.assertIsNone(source.read())

    def test_label_and_percentiles_consume_detection_only(self):
        from cli import label_truth
        from app.radar_probe import history_percentiles, _pcts
        package = self.package()
        self.legacy("raw/CH1/x.xyz")
        items = label_truth.scan(str(self.root))
        self.assertEqual([str(package / "detection.xyz")], [item[0] for item in items])
        stats, count, errors = history_percentiles(str(package), limit=1)
        self.assertEqual(1, count)
        self.assertFalse(errors)
        self.assertEqual(_pcts(load_xyz(str(package / "detection.xyz")).xyz), stats)

    def test_pc_tool_batch_counts_package_once_and_runs_without_numpy(self):
        self.package()
        self.package("p2", "m2")
        script = Path(__file__).resolve().parents[1] / "tools" / "pc_tool.py"
        proc = subprocess.run([sys.executable, "-S", str(script), "batch", str(self.root)],
                              capture_output=True, timeout=30)
        self.assertEqual(0, proc.returncode, proc.stderr.decode("utf-8", "replace"))
        self.assertIn("共 2 个样本", proc.stdout.decode("utf-8"))

    def test_discovery_errors_exposed_in_source_and_statistics(self):
        from app.radar_source import ReplaySource
        from app.radar_probe import history_percentiles
        package = self.package()
        (package / "manifest.json").unlink()
        source = ReplaySource(str(self.root))
        self.assertEqual([], source.files)
        self.assertTrue(source.info()["discovery_errors"])
        self.assertTrue(history_percentiles(str(self.root))[2])


class TestReplayConsumers(_SampleFixture):
    def run_replay(self, args):
        from cli import replay
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
            status = replay.main(args)
        return status, stream.getvalue()

    def test_replay_explicit_role_remains_available(self):
        from cli import replay
        package = self.package()
        path = str(package / "carriage.xyz")
        self.assertEqual([path], replay.collect(path, str(self.root)))
        self.assertEqual([str(package / "observation.xyz")], replay.collect("", str(self.root)))
        with self.assertRaises(ValueError):
            replay.collect_samples(path, str(self.root), limit=-1)

    def test_replay_outputs_unique_across_packages_and_runs_no_false_regression(self):
        from cli import replay
        first = self.package()
        self.package("p2", "m2")
        report1, report2 = self.root / "r1.json", self.root / "r2.json"
        output = self.root / "output"

        def detect(path, *args):
            frame = path if isinstance(path, Frame) else load_xyz(path)
            indices = np.array([1, 3])
            return True, "空车", None, frame, SimpleNamespace(cloud=frame.xyz[indices], source_indices=indices), ""

        with patch.object(replay, "detect_one", side_effect=detect):
            status, text = self.run_replay(["--dir", str(self.root / "archives-v1"), "--out", str(output), "--json", str(report1)])
            self.assertEqual(0, status)
            self.assertIn("未执行理由回归", text)
            self.assertIn(REPLAY_NOTE, text)
            data = json.loads(report1.read_text(encoding="utf-8"))
            self.assertEqual((2, 0, "not_run"), (data["frames"], data["compared"], data["comparison_status"]))
            saved = {row["output"]: Path(row["output"]).read_bytes() for row in data["rows"]}
            self.assertEqual(2, len(saved))
            for path in saved:
                np.testing.assert_array_equal(load_xyz(path).ring, [11, 13])
            status, _ = self.run_replay(["--dir", str(self.root / "archives-v1"), "--out", str(output), "--json", str(report2)])
            self.assertEqual(0, status)
            second = json.loads(report2.read_text(encoding="utf-8"))
            self.assertNotEqual(data["output_run"], second["output_run"])
            for path, content in saved.items():
                self.assertEqual(content, Path(path).read_bytes())
            self.assertEqual(4, len(list(output.rglob("*.xyz"))))
            self.assertEqual(dict(example_bundle("m1").files)["observation.xyz"], (first / "observation.xyz").read_bytes())

    def test_invalid_package_and_detection_failure_return_nonzero(self):
        from cli import replay
        self.package()
        broken = self.package("broken", "broken")
        (broken / "manifest.json").unlink()
        with patch.object(replay, "detect_one", return_value=(None, "", None, None, None, "单测故障")):
            status, text = self.run_replay(["--dir", str(self.root)])
        self.assertEqual(1, status)
        self.assertIn("单测故障", text)
        self.assertIn("未发布", text)

    def test_legacy_reason_comparison_preserved(self):
        from cli import replay
        path = self.legacy()
        Path(str(path) + ".reason.txt").write_text("空车", encoding="utf-8")
        frame = load_xyz(str(path))
        with patch.object(replay, "detect_one", return_value=(True, "空车", None, frame, None, "")):
            status, text = self.run_replay(["--dir", str(self.root)])
        self.assertEqual(0, status)
        self.assertIn("已比较 1 帧", text)

    def test_unknown_archive_channel_not_recomputed_with_first_channel(self):
        from cli import replay
        self.package(channel="NO-CONFIG")
        with patch.object(replay, "detect_one") as detect:
            status, text = self.run_replay(["--dir", str(self.root)])
        self.assertEqual(1, status)
        detect.assert_not_called()
        self.assertIn("没有当前配置", text)

    def test_output_input_trees_cannot_overlap_or_feed_back(self):
        from cli import replay
        input_dir = self.root / "inputs"
        publish(input_dir / "archives-v1" / "p1", example_bundle())
        for output in (input_dir, input_dir / "replayed", self.root):
            with self.subTest(output=output), patch.object(replay, "detect_one") as detect:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                    replay.main(["--dir", str(input_dir), "--out", str(output), "--no-roi"])
                self.assertEqual(2, raised.exception.code)
                detect.assert_not_called()
        self.assertFalse((input_dir / "replayed").exists())
        self.assertEqual(4, len(list(input_dir.rglob("*.xyz"))))
