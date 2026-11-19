"""双轨验证入口只覆盖内存，不修改现场配置或历史答案。"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app.settings import BASELINE_SUBDIR   # noqa: E402


class TestVerify(unittest.TestCase):
    root = Path(__file__).resolve().parents[1]

    def run_verify(self, *args):
        return subprocess.run([sys.executable, str(self.root / "cli/verify.py"), *args],
                              cwd=str(self.root), env=dict(os.environ, PYTHONIOENCODING="utf-8"),
                              capture_output=True, text=True, encoding="utf-8", timeout=120)

    def test_origin_strict_has_ten_exact_matches_without_file_changes(self):
        paths = list((self.root / "config").glob("*.json"))
        baseline = self.root.joinpath(*BASELINE_SUBDIR)
        paths += list(baseline.rglob("*.xyz"))
        paths += list(baseline.rglob("*.reason.txt"))
        before = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / "verify.json"
            result = self.run_verify("--strict", "--gate-mode", "origin", "--quiet", "--json", str(report))
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            data = json.loads(report.read_text(encoding="utf-8"))
            self.assertEqual((data["compared"], data["passed"], data["failed"]), (10, 10, 0))
            self.assertEqual(data["gate_mode"], "origin")
            self.assertTrue(data["strict"])
            self.assertTrue(all(row["match"] is True for row in data["rows"]))
        self.assertEqual(before, {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})

    def test_invalid_mode_is_rejected(self):
        result = self.run_verify("--gate-mode", "typo")
        self.assertEqual(result.returncode, 2)
        self.assertIn("invalid choice", result.stderr)

    def test_help_explains_process_only_override(self):
        result = self.run_verify("--help")
        self.assertEqual(result.returncode, 0)
        self.assertIn("--gate-mode", result.stdout)
        self.assertIn("不改配置", result.stdout)
