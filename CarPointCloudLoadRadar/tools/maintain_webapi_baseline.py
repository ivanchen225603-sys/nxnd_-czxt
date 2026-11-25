#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""维护时确认空车基线；只调用既有管理员接口，不加入日常看板。"""
import argparse
import contextlib
import ipaddress
import json
import os
from pathlib import Path
import re
import sys
import tempfile
from urllib import request, parse, error
import uuid

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from app.load_radar_contract import BaselineCache
from app.adapters.bridge_boundary import _safe_storage
from app.adapters.webapi_http import _NoRedirect


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,64}", value):
        raise ValueError("维护记录中的标识无效")
    return value


class AdminApi:
    def __init__(self, url, token):
        parts = parse.urlsplit(url)
        loopback = parts.hostname == "localhost"
        try:
            loopback = loopback or ipaddress.ip_address(parts.hostname or "").is_loopback
        except ValueError:
            pass
        if (parts.scheme != "https" and not (parts.scheme == "http" and loopback) or not parts.hostname or
                parts.username is not None or parts.password is not None or parts.query or parts.fragment or
                parts.path not in ("", "/")):
            raise ValueError("请使用无凭据的 HTTPS 后台地址或本机 HTTP 地址")
        if not token or any(c.isspace() for c in token):
            raise ValueError("请通过 LOAD_RADAR_ADMIN_TOKEN 提供管理员令牌")
        self.url, self.token = url.rstrip("/"), token
        self.opener = request.build_opener(request.ProxyHandler({}), _NoRedirect())

    def __call__(self, method, path, body=None):
        data = None if body is None else json.dumps(body, ensure_ascii=False, allow_nan=False).encode("utf-8")
        req = request.Request(self.url + "/api/v1/load-radar" + path, data=data, method=method,
            headers={"Authorization": "Bearer " + self.token, "Content-Type": "application/json"})
        try:
            with self.opener.open(req, timeout=10) as response:
                content = response.read(4 * 1024 * 1024 + 1)
            if len(content) > 4 * 1024 * 1024:
                raise ValueError("维护回执过大")
            envelope = json.loads(content)
            if envelope.get("success") is not True or envelope.get("code") != 200:
                raise ValueError("后台未确认本次维护成功，请保留原操作记录重试查询")
            return envelope["data"]
        except error.HTTPError as exc:
            code = exc.code
            exc.close()
            raise ValueError("维护接口未确认成功（HTTP %s），保留原操作记录，勿更换编号" % code) from None
        except error.URLError:
            raise ValueError("后台暂不可达，保留原操作记录，勿重新发起另一项维护") from None


def validate_empty(row, node, channel):
    measurement, binding = row.get("measurement", {}), row.get("binding")
    if (measurement.get("nodeId") != node or measurement.get("channelId") != channel or
            measurement.get("sourceMode") != "live" or measurement.get("verdict") != "empty" or
            measurement.get("frameDegraded") is not False or not isinstance(binding, dict) or
            not binding.get("licensePlate") or binding.get("vehicleIdentity") != "plate:" + binding["licensePlate"]):
        raise ValueError("必须是本通道已关联真实车牌的完整空车结果")
    return measurement, binding


def matching_versions(versions, measurement, binding):
    return [v for v in versions if v.get("active") is True and v.get("nodeId") == measurement["nodeId"]
        and v.get("channelId") == "CH4" and v.get("licensePlate") == binding["licensePlate"]]


def maintain(api, node, channel, reason, saved, persist):
    """每次推进同一份持久操作；未完成即返回，不自动新建采集编号。"""
    identifier(node)
    identifier(channel)
    if channel != "CH4":
        raise ValueError("空车基线维护只能在CH4执行")
    if not reason or reason != reason.strip() or len(reason) > 500:
        raise ValueError("请填写简短维护理由")
    if saved is None:
        lanes = api("GET", "/state").get("lanes", [])
        matches = [lane for lane in lanes if lane.get("nodeId") == node and lane.get("channelId") == channel]
        if len(matches) != 1 or matches[0].get("online") is not True:
            raise ValueError("当前通道没有新鲜在线观测")
        state = matches[0].get("state") or {}
        age = state.get("frame_age_sec")
        if not isinstance(age, (int, float)) or not 0 <= age <= 5 or state.get("mode") != "live":
            raise ValueError("只允许当前实车在线维护")
        original = identifier((state.get("current_result") or {}).get("measurement_id"))
        row = api("GET", "/measurements/" + original)
        measurement, binding = validate_empty(row, node, channel)
        if measurement.get("measurementId") != original:
            raise ValueError("后台返回了其他测量")
        active = matching_versions(api("GET", "/baselines"), measurement, binding)
        if len(active) > 1:
            raise ValueError("活动基线冲突，停止维护")
        saved = dict(schema=1, node=node, channel=channel, reason=reason, original=original,
            passage=measurement["passageId"], calibration=measurement["calibrationId"],
            configuration=measurement["configurationHash"], binding=binding,
            requestId="maint-" + uuid.uuid4().hex,
            expectedVersionId=active[0]["versionId"] if active else None)
        # 必须在任何 POST 之前保存原命令和激活比较值；失败不能先触发扫描。
        persist(saved)
    if any(saved.get(k) != v for k, v in (("schema", 1), ("node", node), ("channel", channel), ("reason", reason))):
        raise ValueError("维护参数与原操作记录不一致")
    if saved.get("completed"):
        return saved
    original, request_id = identifier(saved["original"]), identifier(saved["requestId"])
    command_path = "/measurements/" + original + "/commands"
    if "candidate" not in saved:
        # 同 ID 重投只取原命令结果，不会创建另一次扫描；源码重启亦按原命令凭据恢复。
        status = api("POST", command_path, dict(requestId=request_id, kind="baseline-capture", confirmedEmpty=True))
        if (status.get("requestId") != request_id or status.get("channelId") != channel or
                status.get("passageId") != saved["passage"] or status.get("kind") != "baseline-capture"):
            raise ValueError("命令回执身份不一致")
        if status.get("status") in ("pending", "running"):
            return saved
        if status.get("status") != "succeeded":
            raise ValueError("本次空车候选未成功，不能登记或激活；保留原记录核查")
        candidate_id = identifier(status.get("measurementId"))
        candidate = api("GET", "/measurements/" + candidate_id)
        measurement, binding = validate_empty(candidate, node, channel)
        if (measurement.get("measurementId") != candidate_id or measurement.get("passageId") != saved["passage"] or
                measurement.get("calibrationId") != saved["calibration"] or
                measurement.get("configurationHash") != saved["configuration"] or
                any(binding.get(k) != saved["binding"].get(k) for k in
                    ("passageKey", "workflowId", "licensePlate", "cameraIdentifier", "vehicleIdentity"))):
            raise ValueError("空车候选与原维护车辆或配置不一致")
        sha = measurement.get("baselineCandidateSha256")
        if not isinstance(sha, str) or not re.fullmatch("[0-9a-f]{64}", sha):
            raise ValueError("候选资产尚未登记")
        saved.update(candidate=candidate_id, sha256=sha)
        persist(saved)
    baseline = api("POST", "/baselines", dict(measurementId=saved["candidate"], assetSha256=saved["sha256"],
        confirmedEmpty=True, reason=reason))
    version = identifier(baseline.get("versionId"))
    if baseline.get("measurementId") != saved["candidate"] or baseline.get("assetSha256") != saved["sha256"]:
        raise ValueError("基线登记回执不是本次候选")
    if baseline.get("active") is not True:
        baseline = api("POST", "/baselines/" + version + "/activate",
            dict(expectedVersionId=saved["expectedVersionId"], reason=reason))
    if baseline.get("versionId") != version or baseline.get("active") is not True:
        raise ValueError("后台未确认本次基线激活")
    saved.update(completed=True, versionId=version)
    persist(saved)
    return saved


@contextlib.contextmanager
def operation_file(path):
    path = Path(path).absolute()
    _safe_storage(path.parent)
    if not path.parent.is_dir():
        raise ValueError("请先准备维护记录目录，再发起维护")
    # 同步已有目录链，避免新准备的目录尚未持久化就发出命令。
    for directory in reversed((path.parent,) + tuple(path.parent.parents)):
        _sync_directory(directory)
    BaselineCache._check_path(path)
    lock_path = path.with_name(path.name + ".lock")
    BaselineCache._check_path(lock_path)
    with open(lock_path, "a+b") as lock:
        if os.name == "nt":
            import msvcrt
            lock.seek(0)
            msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        def persist(value):
            BaselineCache._check_path(path)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as out:
                    temporary = Path(out.name)
                    out.write(json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8"))
                    out.flush()
                    os.fsync(out.fileno())
                _replace_journal(temporary, path)
                _sync_directory(path.parent)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
        saved = None
        if path.exists():
            if path.stat().st_size > 65536:
                raise ValueError("维护记录超过大小限制")
            saved = json.loads(path.read_bytes())
        yield saved, persist


def _sync_directory(directory):
    if os.name == "nt":
        return  # Windows 由下方 WRITE_THROUGH 替换完成落盘保证。
    descriptor = os.open(str(directory), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _replace_journal(source, destination):
    if os.name != "nt":
        os.replace(source, destination)
        return
    import ctypes
    from ctypes import wintypes
    move = ctypes.WinDLL("kernel32", use_last_error=True).MoveFileExW
    move.argtypes = (wintypes.LPCWSTR, wintypes.LPCWSTR, wintypes.DWORD)
    move.restype = wintypes.BOOL
    if not move(str(source), str(destination), 0x1 | 0x8):  # REPLACE_EXISTING | WRITE_THROUGH
        raise ctypes.WinError(ctypes.get_last_error())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True)
    parser.add_argument("--node", required=True)
    parser.add_argument("--channel", required=True)
    parser.add_argument("--reason", required=True)
    parser.add_argument("--operation-file", required=True, help="保存本次维护编号，未完成时复用同一文件")
    parser.add_argument("--confirm-empty", action="store_true", help="人工确认当前车厢确实为空，授权此次维护采集及激活")
    args = parser.parse_args(argv)
    if not args.confirm_empty:
        parser.error("必须由维护人员明确 --confirm-empty；日常车辆不运行此工具")
    try:
        api = AdminApi(args.url, os.environ.get("LOAD_RADAR_ADMIN_TOKEN", ""))
        with operation_file(args.operation_file) as (saved, persist):
            # 防止把一个后台的维护意图恢复到另一个后台。
            if saved is not None and saved.get("server") != api.url:
                raise ValueError("后台地址与原维护记录不同")
            def save(value):
                value["server"] = api.url
                persist(value)
            result = maintain(api, args.node, args.channel, args.reason, saved, save)
        print("基线维护完成" if result.get("completed") else "已受理，稍后使用完全相同的命令查询并继续，勿更换操作文件")
        return 0
    except (ValueError, OSError, KeyError, TypeError):
        print("维护未完成。检查通道、权限及后台日志；保留原操作文件重试，不要重新生成编号。", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
