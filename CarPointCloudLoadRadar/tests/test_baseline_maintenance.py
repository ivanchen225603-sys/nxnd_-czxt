"""维护工具只推进原确认操作；模拟后台，不访问真实节点或数据库。"""
import copy
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from tools.maintain_webapi_baseline import maintain, main, operation_file


class MaintenanceApi:
    def __init__(self):
        self.calls = []
        self.versions = []
        self.running = False
        self.active = False
        self.fail_after_activation = False
        self.competing = False
        self.wrong_candidate = False
        self.row = dict(measurement=dict(measurementId="original", nodeId="node1", channelId="CH4",
            passageId="p1", calibrationId="cal1", configurationHash="c" * 64,
            sourceMode="live", verdict="empty", frameDegraded=False),
            binding=dict(passageKey="k1", workflowId="w1", licensePlate="宁A25771",
                cameraIdentifier="CAM_003", vehicleIdentity="plate:宁A25771"))

    def __call__(self, method, path, body=None):
        self.calls.append((method, path, copy.deepcopy(body)))
        if path == "/state":
            return dict(lanes=[dict(nodeId="node1", channelId="CH4", online=True,
                state=dict(mode="live", frame_age_sec=0.1, current_result=dict(measurement_id="original")))])
        if path == "/measurements/original":
            return copy.deepcopy(self.row)
        if path == "/measurements/candidate":
            row = copy.deepcopy(self.row)
            row["measurement"].update(measurementId="candidate", baselineCandidateSha256="a" * 64,
                passageId="other" if self.wrong_candidate else "p1")
            return row
        if path.endswith("/commands"):
            return dict(requestId=body["requestId"], kind=body["kind"], channelId="CH4", passageId="p1",
                status="running" if self.running else "succeeded", measurementId=None if self.running else "candidate")
        if path == "/baselines" and method == "GET":
            return self.versions
        if path == "/baselines" and method == "POST":
            return dict(versionId="version1", measurementId="candidate", assetSha256="a" * 64, active=self.active)
        if path.endswith("/activate"):
            if self.competing:
                raise ValueError("模拟并发基线已变化")
            self.active = True
            if self.fail_after_activation:
                self.fail_after_activation = False
                raise OSError("模拟激活成功但回执丢失")
            return dict(versionId="version1", active=True)
        raise AssertionError((method, path))


class TestBaselineMaintenance(unittest.TestCase):
    def setUp(self):
        self.api, self.saved = MaintenanceApi(), None

    def save(self, value):
        self.saved = copy.deepcopy(value)

    def run_once(self):
        return maintain(self.api, "node1", "CH4", "人工确认空车并维护", self.saved, self.save)

    def test_one_confirmation_records_id_before_capture_and_finishes(self):
        original = self.api.__call__
        def api(method, path, body=None):
            if method == "POST":
                self.assertIsNotNone(self.saved)
            return original(method, path, body)
        result = maintain(api, "node1", "CH4", "人工确认空车并维护", None, self.save)
        self.assertTrue(result["completed"])
        self.assertTrue(self.api.active)
        self.assertEqual([p for m, p, _ in self.api.calls if m == "POST"],
            ["/measurements/original/commands", "/baselines", "/baselines/version1/activate"])

    def test_running_resumes_same_command_id(self):
        self.api.running = True
        self.run_once()
        original = self.saved["requestId"]
        self.api.running = False
        self.run_once()
        commands = [b for _, p, b in self.api.calls if p.endswith("/commands")]
        self.assertEqual([b["requestId"] for b in commands], [original, original])
        self.assertTrue(self.saved["completed"])

    def test_ch4_calibration_update_uses_same_plate_previous_head(self):
        self.api.versions = [dict(active=True, nodeId="node1", channelId="CH4", calibrationId="old-calibration",
            vehicleIdentity="old-manual-identity", licensePlate=self.api.row["binding"]["licensePlate"], versionId="previous-version")]
        self.run_once()
        self.assertEqual("previous-version", self.saved["expectedVersionId"])
        activations = [body for method, path, body in self.api.calls if path.endswith("/activate")]
        self.assertEqual("previous-version", activations[0]["expectedVersionId"])

    def test_non_ch4_maintenance_is_rejected_before_remote_calls(self):
        with self.assertRaisesRegex(ValueError, "CH4"):
            maintain(self.api, "node1", "CH1", "人工确认", None, self.save)
        self.assertEqual([], self.api.calls)

    def test_activation_lost_reply_does_not_repeat_capture_or_overwrite_new_head(self):
        self.api.fail_after_activation = True
        with self.assertRaises(OSError):
            self.run_once()
        self.assertNotIn("completed", self.saved)
        self.run_once()
        self.assertTrue(self.saved["completed"])
        self.assertEqual(1, sum(p.endswith("/commands") for _, p, _ in self.api.calls))
        self.assertEqual(1, sum(p.endswith("/activate") for _, p, _ in self.api.calls))

    def test_changed_vehicle_or_competing_activation_fails_closed(self):
        self.api.wrong_candidate = True
        with self.assertRaises(ValueError):
            self.run_once()
        self.assertFalse(self.api.active)
        self.assertNotIn("candidate", self.saved)
        self.api.wrong_candidate, self.api.competing = False, True
        with self.assertRaises(ValueError):
            self.run_once()
        self.assertNotIn("completed", self.saved)

    def test_confirmation_and_durable_record_are_required_before_any_write(self):
        with patch("tools.maintain_webapi_baseline.AdminApi") as api:
            with self.assertRaises(SystemExit):
                main(["--url", "http://localhost", "--node", "node1", "--channel", "CH4",
                    "--reason", "维护", "--operation-file", "unused.json"])
            api.assert_not_called()
        def fail_save(value):
            raise OSError("模拟维护记录盘满")
        with self.assertRaises(OSError):
            maintain(self.api, "node1", "CH4", "人工确认", None, fail_save)
        self.assertFalse(any(m == "POST" for m, _, _ in self.api.calls))

    def test_operation_file_survives_reopen_without_token(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "maint.json"
            with operation_file(path) as (saved, persist):
                self.assertIsNone(saved)
                persist(dict(requestId="original-command"))
            with operation_file(path) as (saved, _):
                self.assertEqual(saved, dict(requestId="original-command"))
            self.assertNotIn("token", path.read_text(encoding="utf-8"))

    def test_directory_sync_happens_after_replace_before_post(self):
        from tools import maintain_webapi_baseline as module
        events = []
        replace = module._replace_journal
        original_api = self.api
        def api(method, path, body=None):
            if method == "POST":
                events.append("post")
                self.assertEqual(events[-4:], ["file-sync", "replace", "directory-sync", "post"])
                raise OSError("只检查第一次命令前的顺序")
            return original_api(method, path, body)
        def replace_record(source, destination):
            events.append("replace")
            replace(source, destination)
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(module, "_sync_directory", side_effect=lambda _: events.append("directory-sync")):
                with operation_file(Path(folder) / "maint.json") as (_, persist):
                    with patch.object(module.os, "fsync", side_effect=lambda _: events.append("file-sync")), \
                            patch.object(module, "_replace_journal", side_effect=replace_record):
                        with self.assertRaises(OSError):
                            maintain(api, "node1", "CH4", "人工确认", None, persist)
        self.assertIn("post", events)

    def test_directory_sync_failure_never_posts(self):
        with tempfile.TemporaryDirectory() as folder:
            with operation_file(Path(folder) / "maint.json") as (_, persist):
                with patch("tools.maintain_webapi_baseline._sync_directory", side_effect=OSError("目录同步失败")):
                    with self.assertRaises(OSError):
                        maintain(self.api, "node1", "CH4", "人工确认", None, persist)
            self.assertFalse(any(m == "POST" for m, _, _ in self.api.calls))

    def test_missing_directory_is_not_created_during_maintenance(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "missing" / "maint.json"
            with self.assertRaises(ValueError):
                with operation_file(path):
                    pass
            self.assertFalse(path.parent.exists())
