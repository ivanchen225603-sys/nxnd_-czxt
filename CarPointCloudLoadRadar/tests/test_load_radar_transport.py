"""平台传输使用本机假 HTTP 服务；持久队列只写临时目录，不连接现场或达梦。"""
import concurrent.futures
import hashlib
import http.client
import json
import os
import sqlite3
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock

from app import load_radar_transport as transport
from app.load_radar_transport import BridgeConfig, ChannelConfig, HttpClient, HttpError, SQLiteOutbox
from app.settings import baseline_rel


_KEY = "unit-test-node-key-never-for-production-0123456789"


def _configuration(directory, **changes):
    values = dict(node_id="node-1", base_url="http://127.0.0.1:8888", agent_key=_KEY,
                  storage_dir=Path(directory) / "outbox", algorithm_version="clean-room-v1",
                  channels={"CH1": ChannelConfig("calibration-1")})
    values.update(changes)
    return BridgeConfig(**values)


def _measurement(identifier="measurement-1", **changes):
    result = {"measurementId": identifier, "nodeId": "node-1", "channelId": "CH1",
              "verdict": "unknown", "points": [[1.0, 2.0, 3.0]], "reason": "协议测试"}
    result.update(changes)
    return result


class TestBridgeConfig(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="load-transport-")
        self.directory = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def _file(self, **changes):
        values = dict(node_id="node-1", base_url="https://platform.example", agent_key=_KEY,
                      storage_dir="outbox", algorithm_version="clean-room-v1",
                      channels={"CH1": {"calibration_id": "calibration-1"}})
        values.update(changes)
        path = self.directory / "bridge.json"
        path.write_text(json.dumps(values), encoding="utf-8")
        return path

    def test_disabled_by_default_relative_storage_and_redacted_repr(self):
        config = BridgeConfig.load(self._file())
        self.assertFalse(config.enabled)
        self.assertEqual(self.directory / "outbox", config.storage_dir)
        self.assertEqual("calibration-1", config.channels["CH1"].calibration_id)
        self.assertNotIn(_KEY, repr(config))
        self.assertFalse(config.storage_dir.exists())

    def test_unknown_missing_duplicate_or_misspelled_fields_fail_closed(self):
        for changes in ({"retries": 3}, {"channels": {"CH1": {"calibration_id": "c", "wrong": True}}},
                        {"channels": {}}, {"channels": {"CH1": {}}}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                BridgeConfig.load(self._file(**changes))
        path = self._file()
        path.write_text('{"node_id":"a","node_id":"b"}', encoding="utf-8")
        with self.assertRaises(ValueError):
            BridgeConfig.load(path)
        path.write_text('{}', encoding="utf-8")
        with self.assertRaises(ValueError):
            BridgeConfig.load(path)

    def test_rejects_invalid_limits_and_non_boolean_flags_without_secrets(self):
        for changes in ({"timeout_sec": 0}, {"timeout_sec": 31}, {"timeout_sec": True},
                        {"retry_sec": 0}, {"retry_sec": 301}, {"retry_sec": float("nan")},
                        {"enabled": "false"}, {"allow_insecure_http": 1}, {"agent_key": "short"},
                        {"agent_key": _KEY + "\r\nInjected: yes"}, {"node_id": "node\n1"},
                        {"algorithm_version": "x" * 65}, {"algorithm_version": " version"},
                        {"algorithm_version": "version\u0080"}, {"channels": {"CH1": {"calibration_id": "x" * 65}}},
                        {"channels": {"CH1": {"calibration_id": "标定一"}}},
                        {"channels": {"CH%d" % n: ChannelConfig("c") for n in range(17)}}):
            with self.subTest(changes=list(changes)):
                with self.assertRaises(ValueError) as caught:
                    _configuration(self.directory, **changes)
                self.assertNotIn(_KEY, str(caught.exception))

    def test_requires_https_except_literal_loopback_or_explicit_override(self):
        for address in ("http://localhost:8888", "http://127.0.0.1:8888", "http://[::1]:8888", "https://platform.example/base/"):
            self.assertEqual(address.rstrip("/"), _configuration(self.directory, base_url=address).base_url)
        for address in ("http://platform.example", "http://localhost.example", "ftp://localhost",
                        "https://user:password@platform.example", "https://platform.example?", "https://platform.example#",
                        "https://platform.example/?key=" + _KEY, "https://platform.example:0", "https://platform.example:99999",
                        "https://platform.example\n", "https://platform.example\\evil"):
            with self.subTest(address=address), self.assertRaises(ValueError) as caught:
                _configuration(self.directory, base_url=address)
            self.assertNotIn(_KEY, str(caught.exception))
        config = _configuration(self.directory, base_url="http://platform.example", allow_insecure_http=True)
        self.assertTrue(config.allow_insecure_http)

    def test_rejects_protected_paths(self):
        baseline = baseline_rel()
        for relative in (baseline, baseline + "/child", "re", "re/child"):
            with self.subTest(relative=relative), self.assertRaises(ValueError):
                SQLiteOutbox(transport._REPOSITORY_ROOT / relative)

    def test_rejects_symlink_target(self):
        destination = self.directory / "real"
        destination.mkdir()
        link = self.directory / "link"
        try:
            link.symlink_to(destination, target_is_directory=True)
        except OSError:
            self.skipTest("本机不具备创建符号链接权限")
        with self.assertRaises(ValueError):
            SQLiteOutbox(link / "child")


class TestSQLiteOutbox(unittest.TestCase):
    @staticmethod
    def command(**changes):
        body = dict(requestId="request-1", channelId="CH1", passageId="passage-1",
                    kind="remeasure", confirmedEmpty=False)
        body.update(changes)
        return body

    def test_command_and_measurement_commit_and_replay_together(self):
        payload = _measurement(passageId="passage-1")
        body = self.command()
        self.outbox.save(payload, command=body)
        self.outbox.note_retry(payload["measurementId"], HttpError(503))
        self.outbox.save(payload, command=dict(reversed(list(body.items()))))
        receipt = self.outbox.command_result("request-1")
        self.assertEqual(receipt["command"], body)
        self.assertEqual(receipt["measurement"].payload, payload)
        self.assertEqual(receipt["measurement"].attempts, 1)
        with SQLiteOutbox(self.directory) as reopened:
            self.assertEqual(reopened.command_result("request-1")["measurement"].payload, payload)

    def test_command_insert_failure_rolls_back_measurement_and_candidate(self):
        asset = b"candidate"
        body = self.command(channelId="CH4", kind="baseline-capture", confirmedEmpty=True)
        payload = _measurement(channelId="CH4", passageId="passage-1", sourceMode="live", verdict="empty",
                               frameDegraded=False, baselineCandidateSha256=hashlib.sha256(asset).hexdigest())
        self.outbox._connection.execute("""CREATE TEMP TRIGGER fail_receipt BEFORE INSERT ON command_results
            BEGIN SELECT RAISE(ABORT, 'injected'); END""")
        with self.assertRaises(sqlite3.IntegrityError):
            self.outbox.save(payload, asset, command=body)
        self.assertIsNone(self.outbox.get(payload["measurementId"]))
        self.assertIsNone(self.outbox.command_result("request-1"))
        self.outbox._connection.execute("DROP TRIGGER fail_receipt")
        self.outbox.save(payload, asset, command=body)
        self.assertEqual(self.outbox.command_result("request-1")["measurement"].asset, asset)

    def test_command_rejects_changed_identity_and_retroactive_association(self):
        body = self.command()
        payload = _measurement(passageId="passage-1")
        for changes in (dict(channelId="CH2"), dict(passageId="other")):
            with self.assertRaises(ValueError):
                self.outbox.save(payload, command=self.command(**changes))
        self.outbox.save(payload)
        with self.assertRaises(ValueError):
            self.outbox.save(payload, command=body)
        self.assertIsNone(self.outbox.command_result("request-1"))
        original = dict(payload, measurementId="command-result")
        self.outbox.save(original, command=body)
        for changed_body, changed_payload in ((dict(body, confirmedEmpty=True), original),
                                             (body, dict(original, measurementId="other-result"))):
            with self.assertRaises(ValueError):
                self.outbox.save(changed_payload, command=changed_body)
        self.assertIsNone(self.outbox.get("other-result"))
        self.assertEqual(self.outbox.command_result("request-1")["command"], body)

    def test_candidate_command_requires_asset_and_complete_empty_measurement(self):
        body = self.command(kind="baseline-capture", confirmedEmpty=True)
        with self.assertRaises(ValueError):
            self.outbox.save(_measurement(passageId="passage-1"), command=body)
        self.assertEqual(self.outbox.stats()["total"], 0)

    def test_corrupt_receipt_and_read_failure_are_not_missing(self):
        self.outbox.save(_measurement(passageId="passage-1"), command=self.command())
        self.outbox._connection.execute("DELETE FROM outbox")
        with self.assertRaises(RuntimeError):
            self.outbox.command_result("request-1")
        self.outbox.close()
        with self.assertRaises(sqlite3.ProgrammingError):
            self.outbox.command_result("unknown")

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="load-outbox-")
        self.directory = Path(self.temporary.name) / "outbox"
        self.outbox = SQLiteOutbox(self.directory)

    def tearDown(self):
        self.outbox.close()
        self.temporary.cleanup()

    def test_wal_full_and_empty_stats(self):
        self.assertEqual("wal", self.outbox._connection.execute("PRAGMA journal_mode").fetchone()[0])
        self.assertEqual(2, self.outbox._connection.execute("PRAGMA synchronous").fetchone()[0])
        self.assertEqual({"pending": 0, "sent": 0, "blocked": 0, "total": 0}, self.outbox.stats())
        self.assertIsNone(self.outbox.get("missing"))

    def test_asset_and_immutable_payload_survive_restart(self):
        asset = b"candidate-npz-content"
        payload = _measurement(baselineCandidateSha256=hashlib.sha256(asset).hexdigest())
        identifier = self.outbox.save(payload, asset)
        self.assertEqual(payload["measurementId"], identifier)
        payload["reason"] = "不能改变数据库原始值"
        self.outbox.close()
        self.outbox = SQLiteOutbox(self.directory)
        item = self.outbox.get(identifier)
        self.assertEqual("协议测试", item.payload["reason"])
        self.assertEqual(asset, item.asset)
        self.assertEqual("pending", item.status)
        item.payload["reason"] = "返回值也不得修改数据库"
        self.assertEqual("协议测试", self.outbox.get(identifier).payload["reason"])

    def test_same_id_same_payload_is_idempotent_and_changed_payload_fails(self):
        payload = _measurement()
        self.outbox.save(payload)
        self.outbox.note_retry(payload["measurementId"], HttpError(503))
        self.outbox.save(dict(reversed(list(payload.items()))))
        self.assertEqual(1, self.outbox.get(payload["measurementId"]).attempts)
        with self.assertRaises(ValueError):
            self.outbox.save(_measurement(reason="同 ID 不同载荷"))
        self.assertEqual(1, self.outbox.stats()["total"])
        self.assertEqual(payload, self.outbox.pending()[0].payload)

    def test_asset_validation_is_atomic_and_never_replaces_first_asset(self):
        asset = b"candidate"
        digest = hashlib.sha256(asset).hexdigest()
        payload = _measurement(baselineCandidateSha256=digest)
        with self.assertRaises(ValueError):
            self.outbox.save(payload, b"changed")
        with self.assertRaises(ValueError):
            self.outbox.save(_measurement(), asset)
        self.assertEqual(0, self.outbox.stats()["total"])
        self.outbox.save(payload, asset)
        self.outbox.save(payload)
        self.assertEqual(asset, self.outbox.get(payload["measurementId"]).asset)
        other = _measurement("no-asset", baselineCandidateSha256=digest)
        self.outbox.save(other)
        with self.assertRaises(ValueError):
            self.outbox.save(other, asset)
        self.assertIsNone(self.outbox.get("no-asset").asset)

    def test_invalid_json_and_nonfinite_values_are_not_persisted(self):
        for payload in (_measurement(volumeM3=float("nan")), _measurement(volumeM3=float("inf")),
                        _measurement(measurementId="../unsafe"), _measurement(channelId=""), []):
            with self.subTest(payload=type(payload)), self.assertRaises(ValueError):
                self.outbox.save(payload)
        self.assertEqual(0, self.outbox.stats()["total"])

    def test_retry_blocked_and_sent_records_are_retained_and_errors_redacted(self):
        self.outbox.save(_measurement("pending"))
        self.outbox.save(_measurement("blocked"))
        self.outbox.save(_measurement("sent"))
        self.outbox.note_retry("pending", RuntimeError("https://server?key=" + _KEY))
        self.outbox.mark_blocked("blocked", HttpError(409))
        self.outbox.mark_sent("sent")
        self.outbox.mark_sent("sent")
        self.assertEqual({"pending": 1, "sent": 1, "blocked": 1, "total": 3}, self.outbox.stats())
        self.assertEqual(["pending"], [item.measurement_id for item in self.outbox.pending()])
        self.assertNotIn(_KEY, self.outbox.get("pending").last_error)
        self.assertIn("409", self.outbox.get("blocked").last_error)
        self.assertEqual(1, self.outbox.get("sent").attempts)
        with self.assertRaises(ValueError):
            self.outbox.mark_sent("blocked")
        with self.assertRaises(ValueError):
            self.outbox.note_retry("sent", "不能倒退状态")
        with self.assertRaises(KeyError):
            self.outbox.mark_sent("missing")

    def test_latest_sent_is_channel_scoped_and_newest_first(self):
        for identifier, channel in (("older", "CH1"), ("other", "CH2"), ("newer", "CH1")):
            self.outbox.save(_measurement(identifier, channelId=channel))
            self.outbox.mark_sent(identifier)
        self.assertEqual(["newer", "older"], [row["measurementId"] for row in self.outbox.latest_sent("CH1")])
        self.assertEqual(1, len(self.outbox.latest_sent("CH1", 1)))
        for limit in (0, -1, 1001, True):
            with self.assertRaises(ValueError):
                self.outbox.pending(limit)

    def test_detection_and_sender_threads_share_connection_safely(self):
        def save_and_send(index):
            identifier = "concurrent-%d" % index
            self.outbox.save(_measurement(identifier))
            self.outbox.mark_sent(identifier)
            return self.outbox.get(identifier).status
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
            self.assertEqual(["sent"] * 40, list(pool.map(save_and_send, range(40))))
            list(pool.map(lambda _: self.outbox.save(_measurement("one-id")), range(20)))
        self.assertEqual({"pending": 1, "sent": 40, "blocked": 0, "total": 41}, self.outbox.stats())

    def test_independent_connections_serialize_idempotent_inserts(self):
        with SQLiteOutbox(self.directory) as other:
            with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
                jobs = [pool.submit((self.outbox if i % 2 else other).save, _measurement()) for i in range(20)]
                self.assertEqual(["measurement-1"] * 20, [job.result() for job in jobs])
        self.assertEqual(1, self.outbox.stats()["total"])


class TestHttpClient(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="load-http-")
        self.calls = []
        self.status = 200
        self.result = {"success": True, "code": 200, "data": {"ack": True}}
        self.body = None
        self.content_type = "application/json"
        self.content_length = "auto"
        self.location = None
        case = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.handle_call()

            def do_POST(self):
                self.handle_call()

            def do_PUT(self):
                self.handle_call()

            def log_message(self, *_):
                pass

            def handle_call(self):
                size = int(self.headers.get("Content-Length", "0"))
                case.calls.append((self.command, self.path, dict(self.headers), self.rfile.read(size)))
                body = case.body if case.body is not None else json.dumps(case.result).encode("utf-8")
                self.send_response(case.status)
                self.send_header("Content-Type", case.content_type)
                if case.content_length is not None:
                    self.send_header("Content-Length", str(len(body)) if case.content_length == "auto" else str(case.content_length))
                if case.location:
                    self.send_header("Location", case.location)
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.config = _configuration(self.temporary.name, base_url="http://127.0.0.1:%d" % self.server.server_port)
        self.client = HttpClient(self.config)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.temporary.cleanup()

    def test_measurement_baseline_and_asset_routes_match_platform(self):
        payload = _measurement()
        self.assertEqual({"ack": True}, self.client.post_measurement(payload))
        self.assertEqual(payload, json.loads(self.calls[-1][3]))
        self.assertEqual(("POST", "/api/v1/load-radar-agent/measurements"), self.calls[-1][:2])
        self.client.get_measurement("measurement-1")
        self.assertEqual(("GET", "/api/v1/load-radar-agent/measurements/measurement-1"), self.calls[-1][:2])
        self.result["data"] = [{"versionId": "baseline-1"}]
        self.assertEqual(self.result["data"], self.client.list_baselines())
        self.assertEqual(("GET", "/api/v1/load-radar-agent/baselines"), self.calls[-1][:2])
        asset = b"baseline-binary-content"
        self.body, self.content_type = asset, "application/octet-stream"
        self.assertEqual(asset, self.client.download_baseline("baseline-1"))
        self.assertEqual(("GET", "/api/v1/load-radar-agent/baselines/baseline-1/asset"), self.calls[-1][:2])
        digest = hashlib.sha256(asset).hexdigest()
        self.body, self.content_type = None, "application/json"
        self.result["data"] = {"archived": True, "assetSha256": digest}
        self.assertTrue(self.client.upload_baseline("measurement-1", digest, asset)["archived"])
        self.assertEqual(("PUT", "/api/v1/load-radar-agent/baseline-assets/%s?measurementId=measurement-1" % digest), self.calls[-1][:2])
        self.assertEqual(asset, self.calls[-1][3])
        for _, _, headers, _ in self.calls:
            lower = {key.lower(): value for key, value in headers.items()}
            self.assertEqual("node-1", lower["x-load-radar-node"])
            self.assertEqual(_KEY, lower["x-load-radar-key"])

    def test_proxy_environment_is_not_used(self):
        with mock.patch.dict(os.environ, {"HTTP_PROXY": "http://127.0.0.1:1", "http_proxy": "http://127.0.0.1:1",
                                         "NO_PROXY": "", "no_proxy": ""}):
            self.assertEqual({"ack": True}, HttpClient(self.config).get_measurement("measurement-1"))

    def test_redirect_is_never_followed(self):
        for status in (301, 302, 303, 307, 308):
            self.status = status
            self.location = self.config.base_url + "/credential-collector"
            count = len(self.calls)
            with self.assertRaises(HttpError) as caught:
                self.client.post_measurement(_measurement())
            self.assertEqual(status, caught.exception.status)
            self.assertFalse(caught.exception.retryable)
            self.assertEqual(count + 1, len(self.calls))

    def test_status_retry_classification_and_errors_do_not_leak_response_or_credentials(self):
        self.body = ("private response " + self.config.base_url + "?key=" + _KEY).encode("ascii")
        for status in (400, 401, 403, 404, 409, 408, 429, 500, 503):
            self.status = status
            with self.subTest(status=status), self.assertRaises(HttpError) as caught:
                self.client.get_measurement("measurement-1")
            self.assertEqual(status, caught.exception.status)
            self.assertEqual(status in (408, 429) or status >= 500, caught.exception.retryable)
            self.assertNotIn(_KEY, str(caught.exception))
            self.assertNotIn(self.config.base_url, repr(caught.exception))

    def test_failure_envelope_preserves_safe_status_and_rejects_wrong_shapes(self):
        self.result = {"success": False, "code": 409, "message": _KEY, "data": None}
        with self.assertRaises(HttpError) as caught:
            self.client.get_measurement("measurement-1")
        self.assertEqual(409, caught.exception.status)
        self.assertFalse(caught.exception.retryable)
        for result in ({"success": True}, {"success": "true", "data": {}},
                       {"success": True, "data": []}, {"success": True, "code": 200, "data": None}):
            self.result = result
            with self.assertRaises(HttpError):
                self.client.get_measurement("measurement-1")
        self.result = {"success": True, "data": {}}
        with self.assertRaises(HttpError):
            self.client.list_baselines()

    def test_response_size_and_truncated_length_are_rejected(self):
        self.body = b"x" * 1025
        from app.adapters import webapi_http
        with mock.patch.object(webapi_http, "MAX_BYTES", 1024):
            with self.assertRaises(HttpError):
                self.client.get_measurement("measurement-1")
            self.content_length = None
            with self.assertRaises(HttpError):
                self.client.get_measurement("measurement-1")
        self.body, self.content_length = b"{}", 20
        with self.assertRaises(HttpError) as caught:
            self.client.get_measurement("measurement-1")
        self.assertTrue(caught.exception.retryable)

    def test_invalid_json_duplicate_fields_and_nonfinite_response_are_rejected(self):
        for body in (b"not json", b'{"success":true,"success":false,"data":{}}',
                     b'{"success":true,"data":{"volume":NaN}}', b"\xff"):
            self.body = body
            with self.assertRaises(HttpError) as caught:
                self.client.get_measurement("measurement-1")
            self.assertTrue(caught.exception.retryable)

    def test_network_and_protocol_failures_are_sanitized_retryable_errors(self):
        for failure in (OSError("url=" + self.config.base_url + "/?key=" + _KEY),
                        http.client.RemoteDisconnected("secret " + _KEY)):
            with mock.patch.object(self.client._opener, "open", side_effect=failure):
                with self.assertRaises(HttpError) as caught:
                    self.client.get_measurement("measurement-1")
                self.assertIsNone(caught.exception.status)
                self.assertTrue(caught.exception.retryable)
                self.assertNotIn(_KEY, str(caught.exception))

    def test_invalid_asset_or_route_identifier_never_makes_network_request(self):
        with self.assertRaises(ValueError):
            self.client.upload_baseline("measurement-1", "a" * 64, b"wrong hash")
        for identifier in ("..", "../operator", "with?query", "with#fragment", "with\nheader"):
            with self.assertRaises(ValueError):
                self.client.get_measurement(identifier)
        self.assertEqual([], self.calls)

    def test_asset_archive_requires_explicit_matching_ack(self):
        asset = b"candidate"
        digest = hashlib.sha256(asset).hexdigest()
        for receipt in ({}, {"archived": False, "assetSha256": digest},
                        {"archived": True, "assetSha256": "a" * 64}):
            self.result["data"] = receipt
            with self.assertRaises(HttpError) as caught:
                self.client.upload_baseline("measurement-1", digest, asset)
            self.assertTrue(caught.exception.retryable)


if __name__ == "__main__":
    unittest.main()
