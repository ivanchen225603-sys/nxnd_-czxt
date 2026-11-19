"""离场资格跟随本车已完成测量，不能跟随会话超时，也不能泄露给下一辆车。"""
import json
import os
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from app.enter_validation import coordinate_evidence, file_sha256, MOTION_SCHEMA
from app.pointcloud import Frame
from app.service import MeasurementRejected
from tests._observations import ObservationClock
from tests import test_service as service_cases


class TestExitQualification(unittest.TestCase):
    def _worker(self, **options):
        self.helper = service_cases.TestExitFrames()
        worker = self.helper._worker(exit_frames=10, exit_step_m=.10, **options)
        self.addCleanup(worker.stop)
        self.addCleanup(worker.source.close)
        return worker

    def _scene(self, worker):
        r = np.random.default_rng(7)
        n = worker.stability.vehicle_threshold * 2
        return np.column_stack([r.uniform(-1.2, 1.2, n), r.uniform(-6., 6., n),
                                r.uniform(-4.2, -2.5, n)]).astype("f4")

    def test_timeout_preserves_physical_passage_before_remeasurement(self):
        worker = self._worker()
        clock = ObservationClock(self)
        cloud = self._scene(worker)
        clock.tick(worker, Frame(cloud, packets=900))
        worker._detected_this_session = 1
        worker._mark_exit_eligible("measurement-before-timeout")
        original = worker._exit_qualification
        worker._session_t0 = time.monotonic() - worker.session_timeout - 1
        clock.tick(worker, Frame(cloud, packets=900))
        self.assertTrue(worker._session_forced)
        self.assertEqual(0, worker._detected_this_session)
        self.assertEqual(original, worker._exit_qualification)
        for step in range(1, 8):
            clock.tick(worker, Frame((cloud + [0., .4 * step, 0.]).astype("f4"), packets=900))
        self.assertGreater(len(worker._exit_shots), 3)
        empty = np.column_stack([np.zeros(100), np.zeros(100), np.full(100, -5.2)]).astype("f4")
        clock.tick(worker, Frame(empty, packets=900))
        self.assertIsNone(worker._exit_qualification)
        files = self.helper._wait_files(os.path.join(self.helper._tmp, "exit", "CH1", "*.xyz.meta.json"))
        self.assertTrue(files)
        # 旁注晚于 XYZ 写入；完整读取后核对内容摘要和配置证据。
        for _ in range(100):
            try:
                meta = json.loads(Path(files[0]).read_text(encoding="utf-8"))
                break
            except json.JSONDecodeError:
                time.sleep(.01)
        self.assertEqual("measurement-before-timeout", meta["measurement_id"])
        self.assertEqual(MOTION_SCHEMA, meta["metadata_schema"])
        self.assertEqual(file_sha256(files[0][:-10]), meta["file_sha256"])
        for key, value in coordinate_evidence(worker._effective_configuration).items():
            self.assertEqual(value, meta[key])
        clock.tick(worker, Frame(cloud, packets=900))
        self.assertIsNone(worker._exit_qualification)
        self.assertEqual([], worker._exit_shots)

    def test_manual_reset_generation_change_and_changed_passage_revoke(self):
        for action in ("manual", "generation", "passage"):
            with self.subTest(action=action):
                worker = self._worker()
                worker._mark_exit_eligible("measurement")
                if action == "manual":
                    worker.force_new_session()
                elif action == "generation":
                    worker._gen += 1
                else:
                    worker._local_passage = "different-vehicle"
                self.helper._shot(worker, 0., 1)
                worker._grab_exit(self.helper._stab(worker), True)
                self.assertIsNone(worker._exit_qualification)
                self.assertEqual([], worker._exit_shots)

    def test_only_successful_measurement_grants_qualification(self):
        worker = self._worker(save_override=False)
        frame = Frame(np.zeros((10, 3), "f4"))
        with patch.object(worker, "_calculate_detection", side_effect=MeasurementRejected("取消或拒绝")):
            with self.assertRaises(MeasurementRejected):
                worker._run_detection(frame, None)
        self.assertIsNone(worker._exit_qualification)
        with patch.object(worker, "_calculate_detection", return_value="measurement"):
            worker._run_detection(frame, None)
        self.assertIsNotNone(worker._exit_qualification)
        worker._mark_exit_eligible("late-old-measurement", "another-passage")
        self.assertEqual("measurement", worker._exit_measurement_id)

    def test_remeasurement_keeps_existing_exit_buffer(self):
        worker = self._worker()
        worker._mark_exit_eligible("first-measurement")
        for i in range(3):
            self.helper._shot(worker, .2 * i, i)
            worker._grab_exit(self.helper._stab(worker), True)
        worker._mark_exit_eligible("same-car-remeasurement")
        self.assertEqual(3, len(worker._exit_shots))
        self.assertEqual("first-measurement", worker._exit_measurement_id)

    def test_enter_writer_records_config_and_content(self):
        worker = self._worker(enter_frames=5)
        for i in range(3):
            self.helper._shot(worker, .2 * i, i)
            worker._grab_enter(self.helper._stab(worker), True)
        worker._flush_enter()
        files = self.helper._wait_files(os.path.join(self.helper._tmp, "enter", "CH1", "*.xyz.meta.json"))
        self.assertTrue(files)
        for _ in range(100):
            try:
                meta = json.loads(Path(files[0]).read_text(encoding="utf-8"))
                break
            except json.JSONDecodeError:
                time.sleep(.01)
        self.assertEqual(MOTION_SCHEMA, meta["metadata_schema"])
        self.assertEqual("enter", meta["motion_kind"])
        self.assertEqual(file_sha256(files[0][:-10]), meta["file_sha256"])
        self.assertEqual(coordinate_evidence(worker._effective_configuration)["coordinate_config"], meta["coordinate_config"])
