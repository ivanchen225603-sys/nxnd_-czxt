# -*- coding: utf-8 -*-
"""正式 HTTP/SSE 快照不得含裸 NaN/Infinity，且不改诊断内存对象。"""
import io
import json
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from app.web.server import DashboardHandler, _json_dumps


class TestWebJson(unittest.TestCase):
    def test_frame_integrity_query_is_explicit_and_preserves_light_request(self):
        handler = object.__new__(DashboardHandler)
        handler.service = SimpleNamespace(snapshot=Mock(return_value={'channels': []}))
        handler._json = lambda value: value
        handler.path = '/api/state?history=0&frame_integrity=1'
        self.assertEqual(handler.do_GET(), {'channels': []})
        handler.service.snapshot.assert_called_once_with(
            include_history=False, include_frame_integrity=True)
        handler.service.snapshot.reset_mock()
        handler.path = '/api/state?history=0'
        handler.do_GET()
        handler.service.snapshot.assert_called_once_with(include_history=False)

    def test_nested_native_and_numpy_missing_values_are_null(self):
        source = {"diag": {"nan": float("nan"), "inf": float("inf")},
                  "points": np.array([[1.25, -np.inf, np.nan]]),
                  "number": np.float32(2.5), "count": np.int64(7)}
        result = json.loads(_json_dumps(source), parse_constant=lambda value: self.fail(value))
        self.assertEqual({"diag": {"nan": None, "inf": None},
                          "points": [[1.25, None, None]], "number": 2.5, "count": 7}, result)
        self.assertTrue(np.isnan(source["diag"]["nan"]))

    def test_sse_emits_same_strict_json(self):
        handler = object.__new__(DashboardHandler)
        handler.wfile = io.BytesIO()
        handler._sse_write("state", {"missing": float("nan"), "zero": 0.0})
        wire = handler.wfile.getvalue().decode("utf-8")
        self.assertIn('event: state\n', wire)
        self.assertEqual({"missing": None, "zero": 0.0}, json.loads(wire.split('data: ', 1)[1]))
