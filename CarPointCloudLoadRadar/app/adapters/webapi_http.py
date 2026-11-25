"""平台 HTTP 适配器：请求、响应解码及既有错误分类。"""
from __future__ import annotations
import http.client
import json
from typing import Any, Dict, List, Optional
from urllib import error, parse, request
from .webapi_config import BridgeConfig
from .bridge_boundary import (MAX_BYTES, _identifier, _json_bytes, _validate_asset,
                              _unique_object, _reject_constant, HttpError as HttpError)

_API_PATH = "/api/v1/load-radar-agent"

class _NoRedirect(request.HTTPRedirectHandler):
    def redirect_request(self, req: Any, fp: Any, code: int, msg: str, headers: Any, newurl: str) -> None:
        return None


class HttpClient:
    def __init__(self, config: BridgeConfig) -> None:
        self.config = config
        # 明确清空代理，机器凭据不得经环境变量代理或 HTTP 重定向转发。
        self._opener = request.build_opener(request.ProxyHandler({}), _NoRedirect())

    def post_measurement(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        data = self._call("POST", "/measurements", _json_bytes(payload), "application/json")
        return self._dictionary(data)

    def prepare_passage(self, payload: Dict[str, Any], *, measured_at: Any = None) -> Dict[str, Any]:
        suffix = "/passages/prepare"
        if measured_at is not None:
            from ..load_radar_contract import _timestamp
            stamp = _timestamp(measured_at).isoformat().replace("+00:00", "Z")
            suffix += "?" + parse.urlencode({"measuredAt": stamp})
        return self._dictionary(self._call("POST", suffix, _json_bytes(payload), "application/json"))

    def report_standstill(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """CH4 位置停稳通知；是否派发双雷达由 WebApi 按配置和车次状态决定。"""
        return self._dictionary(self._call("POST", "/passages/standstill", _json_bytes(payload), "application/json"))

    def report_departure(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self._dictionary(self._call("POST", "/passages/departure", _json_bytes(payload), "application/json"))

    def get_measurement(self, measurement_id: str) -> Dict[str, Any]:
        return self._dictionary(self._call("GET", "/measurements/" + parse.quote(_identifier(measurement_id), safe="")))

    def list_baselines(self) -> List[Dict[str, Any]]:
        data = self._call("GET", "/baselines")
        if not isinstance(data, list) or any(not isinstance(item, dict) for item in data):
            raise HttpError(200, True)
        return data

    def recalculations(self, channel, after_id=None, from_date=None):
        query = {"channelId": channel}
        if after_id is not None:
            query["afterId"] = after_id
        if from_date is not None:
            query["fromDate"] = from_date
        data = self._call("GET", "/recalculations?" + parse.urlencode(query))
        if not isinstance(data, list) or any(not isinstance(item, dict) for item in data):
            raise HttpError(200, True)
        return data

    def post_recalculation(self, payload):
        return self._dictionary(self._call("POST", "/recalculations", _json_bytes(payload), "application/json"))

    def download_baseline(self, version_id: str) -> bytes:
        return self._call("GET", "/baselines/" + parse.quote(_identifier(version_id), safe="") + "/asset", binary=True)

    def upload_baseline(self, measurement_id: str, sha256: str, content: bytes) -> Dict[str, Any]:
        _identifier(measurement_id)
        _validate_asset(sha256, content)
        suffix = "/baseline-assets/" + sha256 + "?" + parse.urlencode({"measurementId": measurement_id})
        receipt = self._dictionary(self._call("PUT", suffix, content, "application/octet-stream"))
        if receipt.get("archived") is not True or receipt.get("assetSha256") != sha256:
            raise HttpError(200, True)
        return receipt

    @staticmethod
    def _dictionary(data: Any) -> Dict[str, Any]:
        if not isinstance(data, dict):
            raise HttpError(200, True)
        return data

    def _call(self, method: str, suffix: str, body: Optional[bytes] = None,
              content_type: Optional[str] = None, binary: bool = False) -> Any:
        headers = {"X-Load-Radar-Node": self.config.node_id, "X-Load-Radar-Key": self.config.agent_key,
                   "Accept": "application/octet-stream" if binary else "application/json"}
        if content_type:
            headers["Content-Type"] = content_type
        try:
            req = request.Request(self.config.base_url + _API_PATH + suffix, body, headers, method=method)
            with self._opener.open(req, timeout=self.config.timeout_sec) as response:
                status = response.getcode()
                if not 200 <= status < 300:
                    raise HttpError(status)
                declared = response.headers.get("Content-Length")
                if declared is not None and (not declared.isdigit() or int(declared) > MAX_BYTES):
                    raise HttpError(status, True)
                data = bytearray()
                while len(data) <= MAX_BYTES:
                    chunk = response.read(min(64 * 1024, MAX_BYTES + 1 - len(data)))
                    if not chunk:
                        break
                    data.extend(chunk)
                if len(data) > MAX_BYTES or declared is not None and len(data) != int(declared):
                    raise HttpError(status, True)
                if binary:
                    if not data or "json" in response.headers.get("Content-Type", "").lower():
                        raise HttpError(status, True)
                    return bytes(data)
            envelope = json.loads(data.decode("utf-8"), object_pairs_hook=_unique_object, parse_constant=_reject_constant)
            if not isinstance(envelope, dict) or envelope.get("success") is not True or "data" not in envelope or envelope.get("code", 200) != 200:
                code = envelope.get("code") if isinstance(envelope, dict) else None
                raise HttpError(code if type(code) is int and 400 <= code <= 599 else status,
                                None if type(code) is int and 400 <= code <= 599 else True)
            return envelope["data"]
        except error.HTTPError as failure:
            status = failure.code
            failure.close()
            raise HttpError(status) from None
        except HttpError:
            raise
        except (OSError, error.URLError, http.client.HTTPException, ValueError, UnicodeError, RecursionError):
            raise HttpError(None, True) from None
