"""CH4同牌基线、几何对齐及正式回退；只用解析点云和独占临时目录。"""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from app import detect
from app.load_radar_contract import build_measurement
from app.load_radar_transport import SQLiteOutbox, command_body
from app.service import ChannelWorker
from tests.test_empty_audit import _EndWallScene
from tests.test_load_radar_contract import _result
from tests.test_volume_relative_height import scene
from tests._volume_context import conditions


def contract_payload(channel, *, aligned=False):
    """由实际积分生成跨语言载荷；固定时间和身份仅用于隔离合同夹具。"""
    base, loaded, options = scene()
    if aligned:
        observed = _EndWallScene()
        base, loaded = observed._baseline(), observed._loaded()
        cloud = loaded.cloud.copy()
        cloud[:, 0] += .3 + .004 * cloud[:, 1]
        cloud[:, 1] += .5
        cloud[:, 2] += .03
        volume = detect.compute_volume_auto(base, replace(loaded, cloud=cloud), cells=(.1,),
            fill_check=False, shared_baseline=True).to_dict(raw=True)
    else:
        options = dict(options, dx=None, dy=None, yaw=None, pose=None)
        volume = detect.compute_volume(base, loaded, **options, shared_baseline=True).to_dict(raw=True)
    result = dict(_result(), channel=channel, ts=1704067200., volume=volume)
    result["baseline_use"] = dict(sourceChannelId="CH4", sourceCalibrationId="cal1",
        sourceConfigurationHash="c" * 64, **volume["baseline_registration"])
    return build_measurement(result, measurement_id="shared-" + channel, node_id="n1",
        passage_id="passage-" + channel, calibration_id="cal-" + channel,
        algorithm_version="ch4-contract-v1", configuration_hash="d" * 64,
        source_mode="live", baseline_version_id="ch4-original")


class TestSharedGeometry(_EndWallScene, unittest.TestCase):
    def test_actual_wall_registration_preserves_volume_for_four_lane_positions(self):
        base, loaded = self._baseline(), self._loaded()
        original = base.cloud.copy()
        reference = detect.compute_volume_auto(base, loaded, cells=(.1,), fill_check=False, shared_baseline=True)
        for channel, dx, dy in (("CH1", .3, .5), ("CH2", -.2, -.5), ("CH3", .15, 1.), ("CH4", 0., 0.)):
            with self.subTest(channel=channel):
                cloud = loaded.cloud.copy()
                cloud[:, 0] += dx + .004 * cloud[:, 1]
                cloud[:, 1] += dy
                cloud[:, 2] += .03 + .004 * cloud[:, 1] + .005 * cloud[:, 0]
                result = detect.compute_volume_auto(base, replace(loaded, cloud=cloud), cells=(.1,),
                    fill_check=False, shared_baseline=True)
                registration = result.baseline_registration
                self.assertEqual("resolved", registration["registrationStatus"], registration)
                self.assertAlmostEqual(dx, registration["dxM"], delta=.01)
                self.assertAlmostEqual(dy, registration["dyM"], delta=.01)
                self.assertAlmostEqual(.004, registration["yawSlope"], delta=.001)
                self.assertAlmostEqual(reference.volume_m3, result.volume_m3, delta=.01)
        np.testing.assert_array_equal(original, base.cloud)

    def test_failed_registration_keeps_original_reference_and_can_publish(self):
        base, loaded = self._baseline(), self._loaded()
        with patch.object(detect, "estimate_registration", return_value=detect.RegistrationDecision(0., "unresolved", "后沿不可见")), \
                patch.object(detect, "estimate_lateral_registration", return_value=detect.LateralDecision(0., "unresolved", "侧壁不可见")), \
                patch.object(detect, "estimate_pose_registration", return_value=detect.PoseDecision("unresolved", "壁顶不可见")), \
                patch.object(detect, "estimate_yaw_registration", return_value=detect.YawDecision("unresolved", "特征不足")):
            result = detect.compute_volume_auto(base, loaded, cells=(.1,), fill_check=False, shared_baseline=True)
        expected = detect.compute_volume(base, loaded, dx=0., dy=0., yaw=0., pose=False, inner=False, fill_check=False)
        self.assertAlmostEqual(expected.volume_m3, result.volume_m3, places=10)
        info = result.baseline_registration
        self.assertEqual("baseline_fallback", info["registrationStatus"])
        self.assertEqual((0., 0., 0., None), (info["dxM"], info["dyM"], info["yawSlope"], info["pose"]))
        self.assertIn("后沿不可见", info["registrationReason"])
        raw = result.to_dict(raw=True)
        self.assertFalse(raw["low_confidence"], result.text)
        self.assertFalse(raw["range_ok"])
        self.assertIsNone(raw["bound_spread"])

    def test_declared_model_also_applies_lateral_yaw_and_pose(self):
        base, loaded = self._baseline(), self._loaded()
        original = loaded.cloud.copy()
        context = conditions(base, loaded, domain=(-1.2, 1.2, -5., 5.), origin=(-1.2, -5.))
        reference = detect.compute_volume(base, loaded, dy=0., shared_baseline=True, **context)
        cloud = loaded.cloud.copy()
        cloud[:, 0] += .3 + .004 * cloud[:, 1]
        cloud[:, 2] += .04
        moved = replace(loaded, cloud=cloud)
        context = conditions(base, moved, domain=(-1.2, 1.2, -5., 5.), origin=(-1.2, -5.))
        result = detect.compute_volume(base, moved, dy=0., shared_baseline=True, **context)
        self.assertTrue(result.declared_model_used, result.text)
        self.assertEqual("resolved", result.baseline_registration["registrationStatus"])
        self.assertAlmostEqual(reference.volume_m3, result.volume_m3, delta=.03)
        np.testing.assert_array_equal(original, loaded.cloud)


class TestSharedContract(unittest.TestCase):
    def test_local_source_metadata_and_latest_ch4_file_survive_restart(self):
        import os
        from app.load_radar_contract import _read_baseline
        with tempfile.TemporaryDirectory() as folder:
            worker = object.__new__(ChannelWorker)
            worker._baseline_dir, worker._baseline_cache = folder, {}
            path = worker.baseline_path_for("宁A25771")
            base, _, _ = scene()
            base.source_channel, base.created = "CH4", "正式CH4来源元数据"
            base.save(path)
            self.assertEqual("CH4", _read_baseline(Path(path).read_bytes()).source_channel)
            first, _ = worker._resolve_baseline("宁A25771")
            base.floor_z -= .1
            before = Path(path).stat().st_mtime
            base.save(path)
            os.utime(path, (before + 1, before + 1))
            current, _ = worker._resolve_baseline("宁A25771")
            self.assertNotEqual(first.floor_z, current.floor_z)
            worker._baseline_cache.clear()
            cold, _ = worker._resolve_baseline("宁A25771")
            self.assertEqual(current.floor_z, cold.floor_z)
            base.source_channel = "CH1"
            base.save(path)
            worker._baseline_cache.clear()
            self.assertIsNone(worker._resolve_baseline("宁A25771")[0])

    def test_all_lanes_keep_ch4_source_and_durable_payload_after_restart(self):
        for channel in ("CH1", "CH2", "CH3", "CH4"):
            with self.subTest(channel=channel), tempfile.TemporaryDirectory() as folder:
                payload = contract_payload(channel)
                self.assertEqual(channel, payload["channelId"])
                self.assertEqual("CH4", payload["baselineUse"]["sourceChannelId"])
                self.assertEqual("success", payload["volumeStatus"])
                self.assertIsNone(payload["boundSpread"])
                outbox = SQLiteOutbox(folder)
                outbox.save(payload)
                before = outbox.get(payload["measurementId"]).payload
                outbox.close()
                outbox = SQLiteOutbox(folder)
                try:
                    self.assertEqual(before, outbox.get(payload["measurementId"]).payload)
                finally:
                    outbox.close()

    def test_cross_language_fixture_is_generated_from_current_python(self):
        path = Path(__file__).parents[2] / "CarPointCloudWebApi/tests/CarPointCloudWebApi.Tests/Fixtures/load-radar-ch4-shared-v1.json"
        actual = {channel: contract_payload(channel) for channel in ("CH1", "CH2", "CH3", "CH4")}
        actual["aligned"] = contract_payload("CH1", aligned=True)
        self.assertEqual(actual, json.loads(path.read_text(encoding="utf-8")))

    def test_non_ch4_capture_is_rejected_before_observation_or_file_access(self):
        from types import SimpleNamespace
        for channel in ("CH1", "CH2", "CH3"):
            body = dict(requestId="capture", channelId=channel, passageId="passage", kind="baseline-capture", confirmedEmpty=True)
            with self.subTest(channel=channel), self.assertRaisesRegex(ValueError, "CH4"):
                command_body(body)
            ok, reason = ChannelWorker.capture_baseline(SimpleNamespace(ch=SimpleNamespace(id=channel)))
            self.assertFalse(ok)
            self.assertIn("CH4", reason)
