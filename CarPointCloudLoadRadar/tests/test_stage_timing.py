# -*- coding: utf-8 -*-
"""异步留样分阶段耗时：只写 stderr，任何输入都不能打断留样/计算/提交。"""
import contextlib
import io
import time
import unittest

from app.deferred_measurements import _stage_timing


class TestStageTiming(unittest.TestCase):
    def test_writes_one_greppable_line_relative_to_frozen_time(self):
        out = io.StringIO()
        with contextlib.redirect_stderr(out):
            _stage_timing("CH4", "mid-1", "calculated", {"measured_at": time.time() - 2.0},
                          calculationMs=12.5)
        line = out.getvalue()
        self.assertTrue(line.startswith("[stage-timing] "))
        self.assertIn("channel=CH4 measurementId=mid-1 stage=calculated", line)
        self.assertIn("calculationMs=12.5", line)
        since = float(line.split("sinceFrozen=")[1].split("s")[0])
        self.assertGreaterEqual(since, 2.0)

    def test_missing_or_bad_frozen_time_is_silently_skipped(self):
        out = io.StringIO()
        with contextlib.redirect_stderr(out):
            _stage_timing("CH1", "mid-2", "persisted", {})
            _stage_timing("CH1", "mid-2", "persisted", None)
            _stage_timing("CH1", "mid-2", "persisted", {"measured_at": "bad"})
        self.assertEqual(out.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
