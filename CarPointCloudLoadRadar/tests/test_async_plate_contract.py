# -*- coding: utf-8 -*-
"""历史基线查询的纯合同测试；不启动 HTTP，不连接数据库。"""
from copy import deepcopy
from datetime import datetime, timedelta, timezone
from urllib.parse import parse_qs, urlsplit
import unittest
from unittest.mock import Mock

from app.load_radar_contract import BaselineCache, validate_baseline_as_of
from app.load_radar_transport import HttpClient, _json_bytes
from tests.test_load_radar_contract import _build, _result, _version


_AT = datetime(2026, 9, 17, 0, 1, 2, 123456, tzinfo=timezone.utc)
_STAMP = _AT.isoformat().replace("+00:00", "Z")
_BINDING = {"vehicleIdentity": "挂车001", "licensePlate": "宁A25771"}


class TestHistoricalPrepareTransport(unittest.TestCase):
    def setUp(self):
        self.client = object.__new__(HttpClient)
        self.client._call = Mock(return_value={"status": "awaiting_plate"})
        self.payload = dict(nodeId="node1", channelId="CH1", passageId="p1", calibrationId="cal1",
                            configurationHash="a" * 64, arrivedAt="2026-09-17T00:00:00Z", sourceMode="live")

    def test_query_preserves_exact_body_and_matches_measurement_time(self):
        original = deepcopy(self.payload)
        self.client.prepare_passage(self.payload)
        old_args = self.client._call.call_args.args
        self.client.prepare_passage(self.payload, measured_at=_AT.timestamp())
        new_args = self.client._call.call_args.args
        result = _result()
        result["ts"] = _AT.timestamp()
        measured = _build(result)["measuredAt"]
        self.assertEqual(old_args, ("POST", "/passages/prepare", _json_bytes(original), "application/json"))
        self.assertEqual(new_args[0], "POST")
        self.assertEqual(new_args[2:], old_args[2:])
        self.assertEqual(parse_qs(urlsplit(new_args[1]).query), {"measuredAt": [measured]})
        self.assertEqual(self.payload, original)

    def test_none_keeps_legacy_route(self):
        self.client.prepare_passage(self.payload, measured_at=None)
        self.assertEqual(self.client._call.call_args.args[1], "/passages/prepare")

    def test_retries_keep_original_query_and_body(self):
        for _ in range(3):
            self.client.prepare_passage(self.payload, measured_at=_AT.timestamp())
        self.assertEqual(len(set(call.args for call in self.client._call.call_args_list)), 1)

    def test_bad_frozen_time_never_calls_transport(self):
        for value in (True, "2026-09-17T00:01:02Z", float("nan"), float("inf"),
                      -float("inf"), 0, (datetime.now(timezone.utc) + timedelta(days=1)).timestamp()):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.client.prepare_passage(self.payload, measured_at=value)
        self.client._call.assert_not_called()


class TestExactBaselineAsOf(unittest.TestCase):
    def test_equivalent_offsets_and_dotnet_zero_precision_are_exact(self):
        for value in (_STAMP, "2026-09-17T00:01:02.1234560Z", "2026-09-17T08:01:02.123456+08:00"):
            with self.subTest(value=value):
                self.assertEqual(validate_baseline_as_of(_AT.timestamp(), value), _AT)

    def test_missing_bad_or_changed_echo_is_rejected(self):
        for value in (None, "", True, 0, "2026-09-17T00:01:02.123456", "2026-09-17T00:01:02.123457Z",
                      "2026-09-17T00:01:02.1234561Z", "2026-09-17T00:01:02.12345601Z",
                      "2026-09-17T00:01:02.123456+00:00:01", "2026-09-17T00:01:02.123456Z "):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_baseline_as_of(_AT.timestamp(), value)

    def test_invalid_frame_time_is_rejected_even_with_valid_echo(self):
        for value in (True, "1", None, float("nan"), float("inf"), 0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_baseline_as_of(value, _STAMP)


class TestHistoricalBaselineSelection(unittest.TestCase):
    def select(self, versions, binding=_BINDING, **changes):
        options = dict(measured_at=_AT.timestamp(), baseline_as_of=_STAMP)
        options.update(changes)
        return BaselineCache.select_historical(versions, binding, "node1", "CH1", "cal1", **options)

    def test_inactive_historical_version_does_not_change_legacy_selection_or_metadata(self):
        version = _version(active=False)
        original = deepcopy(version)
        selected = self.select([version])
        self.assertEqual(selected, original)
        self.assertIs(selected["active"], False)
        self.assertIsNone(BaselineCache.select([version], _BINDING, "node1", "CH1", "cal1"))
        self.assertEqual(version, original)

    def test_currently_active_historical_version_is_accepted(self):
        version = _version()
        self.assertEqual(self.select([version]), version)
        self.assertEqual(BaselineCache.select([version], _BINDING, "node1", "CH1", "cal1"), version)

    def test_all_baseline_identity_dimensions_must_match(self):
        for key, value in (("nodeId", "node2"), ("channelId", "CH2"), ("licensePlate", "宁A64259")):
            with self.subTest(key=key):
                self.assertIsNone(self.select([_version(active=False, **{key: value})]))

    def test_same_plate_ch4_history_is_shared_across_calibration_and_legacy_identity(self):
        # CH4来源及车牌是新契约；保留源标定和旧挂车描述，不要求它们等于载货车道。
        for key, value in (("calibrationId", "cal2"), ("vehicleIdentity", "另一挂车")):
            with self.subTest(key=key):
                version = _version(active=False, **{key: value})
                self.assertEqual(version, self.select([version]))

    def test_binding_is_explicit_and_unknown_never_selects(self):
        self.assertIsNone(self.select([_version()], binding=None))
        self.assertIsNone(self.select([_version()], binding=dict(_BINDING, licensePlate="UNKNOWN")))
        for value in ({}, [], "", dict(_BINDING, vehicleIdentity=""), dict(_BINDING, licensePlate=" x")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.select([_version()], binding=value)

    def test_historical_authority_cannot_be_omitted_or_changed(self):
        for value in (None, "2026-09-17T00:01:03.123456Z"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.select([_version(active=False)], baseline_as_of=value)

    def test_first_activation_cannot_be_missing_later_or_rounded(self):
        for value in (None, "2026-09-17T00:01:02.123457Z", "2026-09-17T00:01:02.1234561Z"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.select([_version(active=False, activatedAt=value)])
        self.assertIsNotNone(self.select([_version(active=False, activatedAt=_STAMP)]))

    def test_dotnet_activation_precision_before_frame_remains_valid(self):
        for value in ("2026-09-17T00:01:02.1234559Z", "2026-09-17T08:01:01.9999999+08:00"):
            with self.subTest(value=value):
                version = _version(active=False, activatedAt=value)
                self.assertEqual(self.select([version]), version)

    def test_conflicting_history_is_not_guessed(self):
        with self.assertRaises(ValueError):
            self.select([_version(active=False), _version(versionId="b2", active=False)])

    def test_existing_version_validation_is_not_relaxed(self):
        for change in ({"active": "false"}, {"assetSha256": "invalid"}, {"scopeKey": "invalid"},
                       {"activatedAt": "2026-09-16T00:00:00Z"}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.select([_version(**change)])


if __name__ == "__main__":
    unittest.main()
