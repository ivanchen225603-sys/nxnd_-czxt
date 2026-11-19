# -*- coding: utf-8 -*-
"""后台上报与独立基线缓存合同测试；不访问雷达、HTTP 或现场数据。"""
from __future__ import annotations

import hashlib
import io
import json
import os
import tempfile
import time
import unittest
import warnings
import zipfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from app.detect import (EmptyBaseline, EmptyResult, PLANE_ORDER, R_EMPTY_FAILED,
                        R_FLOOR_TOO_HIGH, VolumeResult)
from app.settings import baseline_rel
from app.geometry import Plane
from app.load_radar_contract import (BaselineCache, build_measurement,
                                     effective_configuration_hash, make_candidate)
from app.pointcloud import AngleCache, Frame
from app.settings import CalibCfg, ChannelCfg, DevicesCfg, PROJECT_ROOT


def _result():
    return {"channel": "CH1", "ts": time.time(), "empty": False, "step": 10,
            "reason": R_FLOOR_TOO_HIGH, "n_points": 100,
            "frame_degraded": False, "cloud": [[1.25, 2, -3]],
            "volume": {"ok": True, "volume_m3": 45.1234, "hole_ratio": 0.1,
                       "bound_spread": 0.02, "filled_ratio": 0.4}}


def _build(result=None, **kwargs):
    options = dict(measurement_id="m1", node_id="node1", passage_id="p1",
                   calibration_id="cal1", algorithm_version="jt128-v1",
                   configuration_hash="c" * 64, source_mode="live", baseline_version_id="b1")
    options.update(kwargs)
    return build_measurement(_result() if result is None else result, **options)


def _arrays():
    rng = np.random.default_rng(17)
    cloud = rng.uniform([-1, -4, -3], [1, 4, -1], (60, 3)).astype(np.float32)
    return {"floor_z": np.array(-3.0, np.float64),
            "box": np.array([-1, 1, -4, 4, -3, -1], np.float64), "cloud": cloud,
            "created": np.array("2026-09-17T00:00:00Z"),
            "bed_plane": np.array([0, 0, 1, 3], np.float64)}


def _npz(arrays=None):
    output = io.BytesIO()
    np.savez_compressed(output, **(_arrays() if arrays is None else arrays))
    return output.getvalue()


def _version(data=None, **changes):
    version = {"versionId": "b1", "scopeKey": "a" * 64, "nodeId": "node1", "channelId": "CH4",
               "calibrationId": "cal1", "vehicleIdentity": "挂车001", "licensePlate": "宁A25771",
               "measurementId": "m1", "assetSha256": hashlib.sha256(_npz() if data is None else data).hexdigest(),
               "actor": "管理员", "reason": "已确认实车空车", "createdAt": "2026-09-17T00:00:00Z",
               "active": True, "activatedAt": "2026-09-17T00:00:01Z"}
    version.update(changes)
    return version


class TestBuildMeasurement(unittest.TestCase):
    def test_baseline_reader_accepts_numpy_122_function_signatures(self):
        import app.load_radar_contract as contract
        old_header = np.lib.format.read_array_header_1_0
        old_load = np.load
        def legacy_header(stream):
            return old_header(stream)
        def legacy_load(stream, allow_pickle=False):
            return old_load(stream, allow_pickle=allow_pickle)
        data = _npz()
        with patch.object(np.lib.format, "read_array_header_1_0", legacy_header), patch.object(np, "load", legacy_load):
            self.assertEqual(contract._read_baseline(data).n, 60)

    def test_camel_case_contract_success_preserves_volume_and_utc(self):
        result = _build()
        self.assertEqual(result["verdict"], "loaded")
        self.assertEqual(result["volumeStatus"], "success")
        self.assertEqual(result["volumeM3"], 45.1234)
        self.assertEqual(result["filledRatio"], 0.4)  # 厢壁补点占比不是低置信门槛。
        self.assertTrue(result["measuredAt"].endswith("Z"))
        self.assertEqual(result["points"], [[1.25, 2.0, -3.0]])
        self.assertEqual(len(result), 21)
        json.dumps(result, allow_nan=False)

    def test_actual_measurement_interval_is_optional_and_preserved(self):
        source = _result()
        self.assertNotIn("measurementStartedAt", _build(source))
        source["measurement_started_at"] = source["ts"] - 3
        source["measurement_ended_at"] = source["ts"]
        payload = _build(source)
        self.assertIn("measurementStartedAt", payload)
        self.assertIn("measurementEndedAt", payload)
        from app.load_radar_bridge import LoadRadarBridge
        receipt = dict(measurement=dict(payload))
        for key in ("measurementStartedAt", "measurementEndedAt"):
            receipt["measurement"][key] = receipt["measurement"][key].replace("Z", "+00:00")
        self.assertTrue(LoadRadarBridge._same_measurement(receipt, payload))
        receipt["measurement"]["measurementEndedAt"] = payload["measurementStartedAt"]
        self.assertFalse(LoadRadarBridge._same_measurement(receipt, payload))

    def test_actual_measurement_interval_rejects_missing_or_reversed_times(self):
        source = _result()
        source["measurement_started_at"] = source["ts"]
        with self.assertRaises(ValueError):
            _build(source)
        source["measurement_ended_at"] = source["ts"] - 1
        with self.assertRaises(ValueError):
            _build(source)

    def test_all_volume_failure_or_missing_baseline_values_are_null(self):
        for volume in ({}, {"ok": False, "volume_m3": 0}, {"ok": True, "volume_m3": float("nan")}, None):
            with self.subTest(volume=volume):
                source = _result()
                source["volume"] = volume
                result = _build(source)
                self.assertEqual(result["volumeStatus"], "unavailable")
                self.assertIsNone(result["volumeM3"])
                json.dumps(result, allow_nan=False)
        self.assertIsNone(_build(baseline_version_id=None)["volumeM3"])

    def test_computed_volume_publishes_despite_quality_diagnostics(self):
        # ▲ `bound_spread` 不在这张表里了：2026-09-18 现场决定它不当闸门
        #   （`detect.VOLUME_SPREAD_MAX = None`，质量筛选放到下游第三方系统做）。
        #   见 test_wide_range_no_longer_blocks_success。
        for changes in ({"hole_ratio": 0.151}, {"hole_ratio": None},
                        {"status": "low_confidence"}, {"low_confidence": True}):
            with self.subTest(changes=changes):
                source = _result()
                source["volume"].update(changes)
                result = _build(source)
                self.assertEqual(result["volumeStatus"], "success")
                self.assertEqual(result["volumeM3"], 45.1234)
                json.dumps(result, allow_nan=False)
        source = _result()
        source["frame_degraded"] = True
        self.assertEqual(_build(source)["volumeStatus"], "success")

    def test_exact_quality_thresholds_are_allowed(self):
        source = _result()
        source["volume"].update(hole_ratio=0.15, bound_spread=0.03)
        self.assertEqual(_build(source)["volumeStatus"], "success")

    def test_wide_range_no_longer_blocks_success(self):
        """★ 2026-09-18 现场决定：范围宽度不再拦 `volumeStatus`。

        理由是现场给的：「用户没时间看这个数字，最终会进入第三方系统，
        他们会去处理」—— 质量筛选放到下游，源端只管把数和质量指标都报出去。

        ▲ `boundSpread` **照常上报**，一个字段没少；变的只是它不参与判定。
        ▲ `holeRatio` 仍然是闸门 —— 它量的是"这一次扫得全不全"，
          随扫描质量真的在变（实测 3.2% ~ 14.8%）；而 `boundSpread` 量的是
          "这套雷达装配有多少看不见"，每辆车都是 34~39%，不随测量变化。
        ▲▲ **后台还有两处硬编码的 0.03**（`LoadRadarService.cs` /
          `LoadRadarRepository.cs`），源端放行了它照样拒发煤量。
        """
        from app.detect import VOLUME_SPREAD_MAX
        self.assertIsNone(VOLUME_SPREAD_MAX, "门槛应为 None = 不设")
        for spread in (0.031, 0.55, 0.99, float("inf"), None):
            with self.subTest(spread=spread):
                source = _result()
                source["volume"]["bound_spread"] = spread
                result = _build(source)
                self.assertEqual(result["volumeStatus"], "success")
                self.assertEqual(result["volumeM3"], 45.1234)
                json.dumps(result, allow_nan=False)
        # 空洞补算占比也仅作诊断，不阻止已计算的体积发布。
        source = _result()
        source["volume"].update(bound_spread=0.55, hole_ratio=0.16)
        self.assertEqual(_build(source)["volumeStatus"], "success")

    def test_historical_quality_flag_does_not_block_computed_volume(self):
        # 只留空洞占比那一档：范围宽度已不当闸门（见 test_wide_range_no_longer_blocks_success）。
        for hole, spread in ((0.15000001, 0.02),):
            volume = VolumeResult(volume_m3=45.123456, ok=True,
                                  coverage=.9, bound_lo=44., bound_hi=46.,
                                  hole_ratio=hole, bound_spread=spread,
                                  range_kind="conditional_cell_mean", range_status="conditional")
            self.assertEqual([], volume.validity_errors(), "测试必须只因原精度超门槛而低质量")
            source = _result()
            source["volume"] = volume.to_dict()
            source["volume"]["low_confidence"] = not volume.confidence()[0]
            result = _build(source)
            self.assertEqual(result["volumeStatus"], "success")
            # 沿用数值舍入口径，原质量标记不改变计算值。
            self.assertEqual(result["volumeM3"], 45.1235)

    def test_explicit_volume_failure_cannot_be_success(self):
        source = _result()
        source["volume"]["status"] = "failed"
        result = _build(source)
        self.assertEqual(result["volumeStatus"], "unavailable")
        self.assertIsNone(result["volumeM3"])

    def test_ten_thousand_valid_points_are_not_truncated(self):
        source = _result()
        source["cloud"] = [[1.0, 2.0, 3.0]] * 10000
        source["n_points"] = 10000
        self.assertEqual(len(_build(source)["points"]), 10000)

    def test_early_detection_failures_are_unknown_even_with_volume(self):
        for step in range(1, 6):
            source = _result()
            source["step"] = step
            self.assertEqual(_build(source)["verdict"], "unknown")
            self.assertNotEqual(_build(source)["volumeStatus"], "success")
        source = _result()
        source["reason"] = R_EMPTY_FAILED
        self.assertEqual(_build(source)["verdict"], "unknown")

    def test_late_nonempty_result_stays_loaded_without_volume(self):
        for step in range(6, 11):
            source = _result()
            source["step"] = step
            result = _build(source, baseline_version_id=None)
            self.assertEqual(result["verdict"], "loaded")
            self.assertEqual(result["volumeStatus"], "unavailable")

    def test_empty_is_official_result_not_legacy_metadata(self):
        source = _result()
        source["empty"] = True
        source["meta"] = {"empty": False}
        self.assertEqual(_build(source)["verdict"], "empty")
        self.assertNotEqual(_build(source)["volumeStatus"], "success")
        source.pop("empty")
        source["meta"]["empty"] = True
        self.assertEqual(_build(source)["verdict"], "unknown")

    def test_unknown_unproven_false_result_is_not_loaded(self):
        source = _result()
        source.pop("step")
        self.assertEqual(_build(source, baseline_version_id=None)["verdict"], "unknown")

    def test_source_modes_are_explicit(self):
        for mode in ("simulate", "synthetic", "syn", "sim", "sim-radar"):
            self.assertEqual(_build(source_mode=mode)["sourceMode"], "demo")
        for mode in ("udp", "real", "LIVE"):
            self.assertEqual(_build(source_mode=mode)["sourceMode"], "live")
        self.assertEqual(_build(source_mode="replay")["sourceMode"], "replay")
        with self.assertRaises(ValueError):
            _build(source_mode="auto")

    def test_invalid_points_are_rejected_without_dropping_or_repairing(self):
        for cloud in ([[float("nan"), 0, 0]], [[float("inf"), 0, 0]], [[1e39, 0, 0]],
                      [[1, 2]], [1, 2, 3], [["1", "2", "3"]], [[True, False, True]], [[0, 0, 0]] * 10001):
            source = _result()
            source["cloud"] = cloud
            source["n_points"] = 20000
            with self.subTest(cloud_type=type(cloud)), self.assertRaises(ValueError):
                _build(source)

    def test_invalid_identifiers_count_time_and_metrics_fail(self):
        for changes in ({"measurement_id": "../m"}, {"configuration_hash": "c" * 12},
                        {"baseline_candidate_sha256": "A" * 64}, {"node_id": "节点"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                _build(**changes)
        for changes in ({"n_points": 0}, {"n_points": True}, {"frame_degraded": None},
                        {"ts": 0}, {"ts": float("nan")}, {"reason": ""}):
            source = _result()
            source.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                _build(source)
        source = _result()
        source["volume"]["hole_ratio"] = 1.1
        with self.assertRaises(ValueError):
            _build(source)


class TestEffectiveConfigurationHash(unittest.TestCase):
    def setUp(self):
        self.calib, self.dev, self.ch, self.angle = CalibCfg(), DevicesCfg(), ChannelCfg(), AngleCache()

    def digest(self):
        return effective_configuration_hash(self.calib, self.dev, self.ch, self.angle)

    def test_full_digest_uses_defaults_and_all_effective_inputs(self):
        initial = self.digest()
        self.assertRegex(initial, r"^[a-f0-9]{64}$")
        for target, field, value in ((self.calib.packet, "angle_bins", 901),
                                     (self.calib.detection, "settle_sec", 4),
                                     (self.dev.gate, "min_inliers", 999),
                                     (self.ch.lidar, "range_mode", "raw"),
                                     (self.ch.lidar, "yaw_offset_deg", -90),
                                     (self.ch, "replay_interval", 1)):
            old = getattr(target, field)
            setattr(target, field, value)
            self.assertNotEqual(initial, self.digest(), field)
            setattr(target, field, old)
        self.angle.elevation[1] = 1
        self.assertNotEqual(initial, self.digest())

    def test_path_and_raw_do_not_mask_effective_values_or_leak(self):
        initial = self.digest()
        self.calib.path = "private/config.json"
        self.calib.raw = {"secret": "not-runtime-configuration"}
        self.dev.loaded_output_dir = "private/output"
        self.ch.abs_angle_cache = "private/angle.csv"
        self.ch.lidar.angle_cache = "private/angle.csv"
        self.angle.path = "private/angle.csv"
        self.angle.error = "private failure"
        self.assertEqual(initial, self.digest())
        self.assertNotIn("private", self.digest())

    def test_missing_angle_and_nonfinite_effective_config_fail(self):
        with self.assertRaises(ValueError):
            effective_configuration_hash(self.calib, self.dev, self.ch)
        self.calib.floor.z_percentile = float("nan")
        with self.assertRaises(ValueError):
            self.digest()


class TestBaselineCache(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.cache = BaselineCache(os.path.join(self.temp.name, "managed"))

    def tearDown(self):
        self.temp.cleanup()

    def test_install_is_content_addressed_and_does_not_activate_legacy(self):
        data = _npz()
        version = _version(data)
        result = self.cache.install(version, data)
        self.assertIsInstance(result, EmptyBaseline)
        self.assertEqual(result.n, 60)
        self.assertEqual(result.bed_plane, (0, 0, 1, 3))
        np.testing.assert_array_equal(result.cloud, _arrays()["cloud"])
        files = list(Path(self.temp.name).rglob("*"))
        self.assertEqual([p.name for p in files if p.is_file()], [version["assetSha256"] + ".npz"])
        self.assertEqual(self.cache.install(version, data).n, 60)

    def test_hash_and_required_metadata_checked_before_writes(self):
        data = _npz()
        for version in (_version(data, assetSha256="b" * 64), _version(data, versionId="../b"),
                        _version(data, active="true"), _version(data, activatedAt=None),
                        _version(data, createdAt="2026-09-17"), _version(data, licensePlate="UNKNOWN")):
            with self.subTest(version=version["versionId"]), self.assertRaises(ValueError):
                self.cache.install(version, data)
        for field in ("vehicleIdentity", "calibrationId", "scopeKey", "createdAt", "actor"):
            version = _version(data)
            version.pop(field)
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.cache.install(version, data)
        self.assertFalse(self.cache.storage_dir.exists())

    def test_invalid_npz_array_shapes_types_values_and_extra_files_fail(self):
        bad_arrays = []
        for name, value in (("cloud", np.zeros((49, 3), np.float32)),
                            ("cloud", np.zeros((50, 4), np.float32)),
                            ("cloud", np.zeros((50, 3), np.int32)),
                            ("floor_z", np.array([0.0])), ("box", np.zeros(6)),
                            ("bed_plane", np.array([0, 0, 0, 3.0])),
                            ("created", np.array("", dtype=object)),
                            ("ring", np.zeros(60, np.uint16))):
            arrays = _arrays()
            arrays[name] = value
            bad_arrays.append(arrays)
        arrays = _arrays()
        arrays["cloud"][0, 0] = np.nan
        bad_arrays.append(arrays)
        arrays = _arrays()
        arrays.pop("created")
        bad_arrays.append(arrays)
        for arrays in bad_arrays:
            data = _npz(arrays)
            with self.subTest(keys=list(arrays)), self.assertRaises(ValueError):
                self.cache.install(_version(data), data)

    def test_zip_duplicate_members_and_expanded_limit_fail(self):
        original = _npz()
        output = io.BytesIO()
        with zipfile.ZipFile(io.BytesIO(original)) as source, zipfile.ZipFile(output, "w") as target:
            for entry in source.infolist():
                target.writestr(entry.filename, source.read(entry))
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", UserWarning)
                target.writestr("floor_z.npy", source.read("floor_z.npy"))
        data = output.getvalue()
        with self.assertRaises(ValueError):
            self.cache.install(_version(data), data)
        with patch("app.load_radar_contract.MAX_EXPANDED_BYTES", 100):
            with self.assertRaises(ValueError):
                self.cache.install(_version(original), original)
        with patch("app.load_radar_contract.MAX_ASSET_BYTES", 10):
            with self.assertRaises(ValueError):
                self.cache.install(_version(original), original)

    def test_npy_declared_large_shape_cannot_trigger_array_allocation(self):
        output = io.BytesIO()
        with zipfile.ZipFile(io.BytesIO(_npz())) as source, zipfile.ZipFile(output, "w") as target:
            for entry in source.infolist():
                value = source.read(entry)
                if entry.filename == "cloud.npy":
                    value = value.replace(b"(60, 3)", b"(99, 3)")
                target.writestr(entry.filename, value)
        data = output.getvalue()
        with self.assertRaises(ValueError):
            self.cache.install(_version(data), data)

    def test_selection_requires_exact_explicit_identity_and_single_active(self):
        binding = {"vehicleIdentity": "挂车001", "licensePlate": "宁A25771"}
        version = _version()
        self.assertEqual(self.cache.select([version], binding, "node1", "CH1", "cal1"), version)
        self.assertIsNone(self.cache.select([version], None, "node1", "CH1", "cal1"))
        for changes in ({"nodeId": "node2"}, {"channelId": "CH2"}, {"licensePlate": "宁A64259"}, {"active": False}):
            other = dict(version, **changes)
            self.assertIsNone(self.cache.select([other], binding, "node1", "CH1", "cal1"))
        other = dict(version, calibrationId="ch4-new-cal", vehicleIdentity="plate:宁A25771")
        self.assertEqual(other, self.cache.select([other], binding, "node1", "CH3", "ch3-cal"))
        with self.assertRaises(ValueError):
            self.cache.select([version, dict(version, versionId="b2")], binding, "node1", "CH1", "cal1")
        with self.assertRaises(ValueError):
            self.cache.select([version], {"licensePlate": "宁A25771"}, "node1", "CH1", "cal1")

    def test_protected_paths_and_reparse_points_are_rejected(self):
        for relative in (baseline_rel() + "/cache", "data/in/cache", "re/cache"):
            with self.subTest(relative=relative), self.assertRaises(ValueError):
                BaselineCache(os.path.join(PROJECT_ROOT, relative))
        linked = Path(self.temp.name) / "link"
        real = Path(self.temp.name) / "real"
        real.mkdir()
        try:
            linked.symlink_to(real, target_is_directory=True)
        except OSError:
            self.skipTest("当前账号无符号链接权限")
        with self.assertRaises(ValueError):
            BaselineCache(str(linked / "cache"))

    def test_windows_reparse_attribute_fails_without_link_privilege(self):
        fake = SimpleNamespace(st_mode=0, st_file_attributes=0x400)
        with patch.object(Path, "lstat", return_value=fake):
            with self.assertRaises(ValueError):
                BaselineCache(os.path.join(self.temp.name, "junction", "cache"))

    def test_failed_atomic_replace_does_not_change_cached_asset(self):
        data = _npz()
        self.cache.install(_version(data), data)
        path = self.cache.storage_dir / (hashlib.sha256(data).hexdigest() + ".npz")
        with patch("app.load_radar_contract.os.replace", side_effect=OSError("测试故障")):
            with self.assertRaises(OSError):
                self.cache.install(_version(data), data)
        self.assertEqual(path.read_bytes(), data)
        self.assertEqual(len(list(self.cache.storage_dir.iterdir())), 1)


class TestMakeCandidate(unittest.TestCase):
    def fixtures(self):
        cloud = _arrays()["cloud"]
        planes = [Plane() for _ in range(6)]
        planes[PLANE_ORDER.index("floor")] = Plane(0, 0, 1, 3, True)
        return Frame(cloud, ts=time.time()), EmptyResult(planes=planes, cloud=cloud, ok=True)

    def test_requires_explicit_confirmation_and_valid_floor(self):
        frame, result = self.fixtures()
        for confirmation in (False, None, 1, "true"):
            with self.subTest(confirmation=confirmation), self.assertRaises(ValueError):
                make_candidate(frame, result, confirmation)
        result.plane("floor").valid = False
        with self.assertRaises(ValueError):
            make_candidate(frame, result, True)

    def test_candidate_is_npz_bytes_without_files_or_ring_metadata(self):
        frame, result = self.fixtures()
        frame.ring = np.arange(frame.n, dtype=np.uint16)
        with patch.object(EmptyBaseline, "save", side_effect=AssertionError("禁止旧基线写盘")):
            data = make_candidate(frame, result, True)
        with np.load(io.BytesIO(data), allow_pickle=False) as baseline:
            self.assertEqual(set(baseline.files), {"floor_z", "box", "cloud", "created", "bed_plane"})
            np.testing.assert_array_equal(baseline["cloud"], result.cloud)
        with tempfile.TemporaryDirectory() as folder:
            self.assertEqual(BaselineCache(folder).install(_version(data), data).n, len(result.cloud))

    def test_unknown_time_and_nonfinite_frame_rejected(self):
        frame, result = self.fixtures()
        frame.ts = 0
        with self.assertRaises(ValueError):
            make_candidate(frame, result, True)
        frame.ts = time.time()
        frame.xyz[0, 0] = np.nan
        with self.assertRaises(ValueError):
            make_candidate(frame, result, True)


if __name__ == "__main__":
    unittest.main()
