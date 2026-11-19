"""进场帧先核身份/坐标/内容；旧旁注只能显式、可追溯地离线回放。"""
import copy
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from app.enter_validation import (HISTORY_SCHEMA, MOTION_SCHEMA, coordinate_evidence,
                                  file_sha256, validate_enter_frames)
from app.load_radar_contract import effective_configuration_bytes
from app.pointcloud import AngleCache, Frame, save_xyz
from tests import _helpers as H


class TestEnterValidation(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        settings, channel, calib, devices, _ = H.ctx()
        channel = copy.deepcopy(channel)
        channel.mode = "live"
        config = effective_configuration_bytes(calib, devices, channel, AngleCache(channel.abs_angle_cache).load())
        self.config = json.loads(config)
        self.evidence = coordinate_evidence(config)
        self.plate = "宁A05922F"
        self.base = self.root / (self.plate + ".baseline.npz")
        self.base.write_bytes(b"baseline-bound-to-test")
        self.meta = dict(configuration_hash=self.evidence["configuration_sha256"], channel=channel.id,
                         passage_id="passage-test", source_mode="live", frame=dict(gen=0),
                         provenance=dict(segments=[dict(frame=dict(seq=10, ts=1010.0))]),
                         volume_support=dict(segments=[dict(frame=dict(seq=8, ts=1008.0))]))
        self.gate = dict(capture_instance="instance-test")
        self.result = dict(plate=self.plate, plate_ok=True, session=3)
        self.assets = {"effective-configuration.json": config}
        self.archive = SimpleNamespace(metadata=self.meta, manifest_sha256="a" * 64,
                                       bundle=SimpleNamespace(files=self.assets))
        self.paths = []
        roi = np.asarray(calib.roi)
        points = np.tile((roi[0] + roi[1]) / 2, (300, 1)).astype("f4")
        lidar = self.config["channel"]["lidar"]
        yaw = lidar["yaw_offset_deg"]
        for index in (1, 2):
            path = self.root / ("batch_%dof2.xyz" % index)
            save_xyz(str(path), Frame(points))
            item = dict(metadata_schema=MOTION_SCHEMA, motion_kind="enter", channel=channel.id,
                passage_id="passage-test", capture_instance_id="instance-test", source_mode="live",
                gen=0, session_id=3, range_mode=lidar["range_mode"], complete_gate_passed=True,
                dropped_packets=0, packets=901, cropped_to_roi=True,
                roi=dict(zip("xyz", zip(*calib.roi))), ground_z=calib.detection.ground_z,
                yaw_offset_deg=calib.processing.yaw_offset_deg,
                channel_yaw_offset_deg=yaw,
                effective_yaw_offset_deg=yaw if yaw is not None else calib.processing.yaw_offset_deg,
                seq=index, ts=1000.0 + index, index=index, frames=2, points=300,
                file_sha256=file_sha256(path), **self.evidence)
            self._write(path, item)
            self.paths.append(path)

    def _write(self, path, item):
        Path(str(path) + ".meta.json").write_text(json.dumps(item, ensure_ascii=False), encoding="utf-8")

    def _read(self, path):
        return json.loads(Path(str(path) + ".meta.json").read_text(encoding="utf-8"))

    def _sync(self):
        self.assets.update({"gate-summary.json": json.dumps(self.gate).encode(),
                            "detection.xyz.meta.json": json.dumps(self.result).encode()})

    def _validate(self, confirmation=None):
        self._sync()
        return validate_enter_frames(self.archive, self.paths, self.base, self.plate, confirmation)

    def _legacy(self):
        for path in self.paths:
            item = self._read(path)
            for key in ("metadata_schema", "motion_kind", "configuration_sha256", "coordinate_config", "file_sha256"):
                item.pop(key)
            self._write(path, item)

    def _confirmation(self):
        return dict(schema=HISTORY_SCHEMA, archive_manifest_sha256=self.archive.manifest_sha256,
                    channel=self.meta["channel"], passage_id="passage-test", capture_instance_id="instance-test",
                    archive_plate=self.result["plate"], actual_plate=self.plate,
                    baseline_sha256=file_sha256(self.base), reason="现场记录及用户确认的同车对应",
                    enter_files_sha256={path.name: file_sha256(path) for path in self.paths})

    def test_strict_accepts_verified_frames(self):
        value = self._validate()
        self.assertFalse(value.historical)
        self.assertEqual([300, 300], [len(x) for x in value.clouds])

    def test_identity_and_coordinate_conflicts_reject(self):
        path = self.paths[0]
        original = self._read(path)
        changes = dict(channel="CH99", passage_id="other", capture_instance_id="other", source_mode="synthetic",
                       gen=1, session_id=0, range_mode="raw_offset", effective_yaw_offset_deg=12., ground_z=10.,
                       complete_gate_passed=False, dropped_packets=1, packets=0)
        for key, value in changes.items():
            with self.subTest(key=key):
                self._write(path, dict(original, **{key: value}))
                with self.assertRaises(ValueError):
                    self._validate()
        self._write(path, original)

    def test_declared_roi_is_not_the_geometric_manifest_roi(self):
        self.meta["roi"] = {"carriage_status": "unsupported"}
        self.assertEqual(2, len(self._validate().clouds))

    def test_same_passage_can_span_automatic_measurement_sessions(self):
        self.result["session"] = 4
        value = self._validate()
        self.assertEqual(2, len(value.clouds))
        self.assertIn("跨测量会话", value.notes[0])

    def test_content_hash_point_count_and_finite_coordinates_checked(self):
        path = self.paths[0]
        original = self._read(path)
        self._write(path, dict(original, points=299))
        with self.assertRaisesRegex(ValueError, "点数"):
            self._validate()
        self._write(path, dict(original, file_sha256="0" * 64))
        with self.assertRaisesRegex(ValueError, "摘要"):
            self._validate()
        points = np.zeros((300, 3), "f4")
        points[0, 0] = float("nan")
        save_xyz(str(path), Frame(points))
        self._write(path, dict(original, file_sha256=file_sha256(path)))
        with self.assertRaisesRegex(ValueError, "非有限"):
            self._validate()

    def test_partial_new_metadata_cannot_downgrade_to_history(self):
        item = self._read(self.paths[0])
        item.pop("coordinate_config")
        self._write(self.paths[0], item)
        with self.assertRaisesRegex(ValueError, "coordinate_config"):
            self._validate(self._confirmation())

    def test_full_configuration_and_angle_identity_checked(self):
        path = self.paths[0]
        original = self._read(path)
        for key in ("configuration_sha256", "coordinate_config"):
            item = copy.deepcopy(original)
            if key == "configuration_sha256":
                item[key] = "0" * 64
            else:
                item[key]["angle_calibration_sha256"] = "0" * 64
            self._write(path, item)
            with self.assertRaisesRegex(ValueError, "配置证据"):
                self._validate()

    def test_frame_sequence_and_time_precede_stopped_window(self):
        path = self.paths[1]
        original = self._read(path)
        for changes in (dict(seq=1), dict(ts=1001.), dict(seq=8), dict(ts=1008.), dict(ts=float("nan"))):
            self._write(path, dict(original, **changes))
            with self.assertRaises(ValueError):
                self._validate()
        self._write(path, dict(original, seq=7, ts=1002.))
        self.assertEqual(2, len(self._validate().clouds))    # 按位移采样允许跳序号和较长间隔

    def test_missing_frame_and_duplicate_index_reject(self):
        path = self.paths[1]
        self._write(path, dict(self._read(path), index=1))
        with self.assertRaisesRegex(ValueError, "编号"):
            self._validate()

    def test_legacy_requires_explicit_bound_confirmation(self):
        self._legacy()
        with self.assertRaisesRegex(ValueError, "history-confirmation"):
            self._validate()
        self.result["plate"] = "宁ARCB1Q"
        confirmation = self._confirmation()
        value = self._validate(confirmation)
        self.assertTrue(value.historical)
        self.assertIn("未验证", value.notes[0])
        self.assertIn("宁ARCB1Q", value.notes[1])
        confirmation["baseline_sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "baseline_sha256"):
            self._validate(confirmation)

    def test_history_never_waives_known_conflicts_or_changed_content(self):
        self._legacy()
        confirmation = self._confirmation()
        path = self.paths[0]
        original = self._read(path)
        self._write(path, dict(original, source_mode="synthetic"))
        with self.assertRaisesRegex(ValueError, "source_mode"):
            self._validate(confirmation)
        self._write(path, original)
        data = bytearray(path.read_bytes())
        data[-1] ^= 1
        path.write_bytes(data)
        with self.assertRaisesRegex(ValueError, "历史确认清单"):
            self._validate(confirmation)

    def test_cli_rejects_before_fitting_or_registration(self):
        self._legacy()
        self._sync()
        cli = Path(__file__).resolve().parents[1] / "cli"
        with patch.object(sys, "path", [str(cli)] + sys.path):
            spec = importlib.util.spec_from_file_location("enter_volume_test", cli / "enter_volume.py")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        with patch.object(module, "setup_console"), patch("app.archive_reader.load_archive", return_value=self.archive), \
             patch("app.sample_paths._read_package"), patch("app.detect.detect_loaded") as fit, \
             patch("app.detect.register_enter_frames") as register, patch("sys.stderr", new_callable=io.StringIO):
            code = module.main(["--archive", str(self.root), "--baseline", str(self.base),
                                "--enter", str(self.root / "batch")])
        self.assertEqual(2, code)
        fit.assert_not_called()
        register.assert_not_called()

    def test_cli_strict_metadata_reaches_calculation_and_conflict_stops_it(self):
        """入口联测：算法用替身计次，重点钉住实际 CLI 校验在拟合与配准之前。"""
        self._sync()
        self.archive.window = lambda: SimpleNamespace(input_frame=lambda: Frame(np.zeros((300, 3), "f4")))
        self.archive.volume_extra_inputs = lambda: ()
        cli = Path(__file__).resolve().parents[1] / "cli"
        with patch.object(sys, "path", [str(cli)] + sys.path):
            spec = importlib.util.spec_from_file_location("enter_volume_strict_test", cli / "enter_volume.py")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        for wrong in (False, True):
            if wrong:
                self._write(self.paths[0], dict(self._read(self.paths[0]), channel="wrong-channel"))
            with self.subTest(conflicting_channel=wrong), patch.object(module, "setup_console"), \
                 patch("app.archive_reader.load_archive", return_value=self.archive), patch("app.sample_paths._read_package"), \
                 patch("app.detect.EmptyBaseline.load", return_value=object()), \
                 patch("app.detect.detect_loaded", return_value=SimpleNamespace(source_rotation=np.eye(3))) as fit, \
                 patch("app.detect.register_enter_frames", return_value=SimpleNamespace(status="resolved", reason="入口用例", \
                       shifts=(0.,), clouds=())) as register, \
                 patch("app.detect.compute_volume_auto", return_value=SimpleNamespace(volume_m3=1., text="入口用例")), \
                 patch("sys.stdout", new_callable=io.StringIO) as output, patch("sys.stderr", new_callable=io.StringIO):
                code = module.main(["--archive", str(self.root), "--baseline", str(self.base),
                                    "--enter", str(self.root / "batch")])
                self.assertEqual(2 if wrong else 0, code)
                self.assertEqual(0 if wrong else 1, fit.call_count)
                self.assertEqual(0 if wrong else 1, register.call_count)
                if not wrong:
                    self.assertIn("身份、配置及内容通过", output.getvalue())
