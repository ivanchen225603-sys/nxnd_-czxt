"""Windows 无符号链接权限时，用真实目录联接覆盖永久目录/归档边界。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from app.archive_capacity import scan_capacity_files
from app.load_radar_contract import BaselineCache
from app.load_radar_transport import BridgeConfig
from app.pending_measurements import PendingTaskStore


@unittest.skipUnless(os.name == "nt", "Windows 目录联接专用")
class TestWindowsJunctionBoundaries(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(prefix="radar-junction-test-")
        self.root = Path(self.folder.name)
        self.target = self.root / "permanent"
        self.target.mkdir()
        self.sentinel = self.target / "empty-baseline.npz"
        self.sentinel.write_bytes(b"permanent-baseline-must-survive")
        self.link = self.root / "ordinary" / "link"
        self.link.parent.mkdir()
        # New-Item Junction 不需要创建符号链接权限；仅在本用例临时目录创建。
        subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command",
            "$ErrorActionPreference='Stop'; New-Item -ItemType Junction "
            "-Path $env:RADAR_TEST_LINK -Target $env:RADAR_TEST_TARGET | Out-Null"],
            env={**os.environ, "RADAR_TEST_LINK": str(self.link), "RADAR_TEST_TARGET": str(self.target)},
            check=True, capture_output=True)
        self.addCleanup(self.cleanup)

    def cleanup(self):
        # rmdir 只解除联接本身，不遍历永久目录。
        os.rmdir(self.link)
        self.assertEqual(b"permanent-baseline-must-survive", self.sentinel.read_bytes())
        self.folder.cleanup()

    def test_pending_tasks_reject_actual_junction(self):
        with self.assertRaises(ValueError):
            PendingTaskStore(self.link / "queue", ("CH4",))

    def test_baseline_cache_rejects_actual_junction(self):
        with self.assertRaises(ValueError):
            BaselineCache(str(self.link / "cache"))

    def test_outbox_rejects_actual_junction(self):
        with self.assertRaises(ValueError):
            BridgeConfig(node_id="test", base_url="http://127.0.0.1:1",
                agent_key="isolated-test-key-not-production-1234", storage_dir=self.link / "outbox",
                algorithm_version="test", channels={"CH4": {"calibration_id": "test"}})

    def test_capacity_does_not_descend_into_permanent_target(self):
        report = scan_capacity_files([self.link.parent])
        self.assertEqual("partial", report["status"])
        self.assertEqual(0, report["apparent_bytes"])
