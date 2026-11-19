"""桥接边界共用的编码与文件存储规则；不负责联网或数据库生命周期。"""
from __future__ import annotations
import hashlib
import json
import re
import stat
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

MAX_BYTES = 32 * 1024 * 1024
_REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
_IDENTIFIER = re.compile(r"[A-Za-z0-9_.:-]{1,64}\Z")
_SHA256 = re.compile(r"[0-9a-f]{64}\Z")


class HttpError(Exception):
    """只暴露状态与是否可重试，不保留 URL、响应正文、凭据或底层异常。"""
    def __init__(self, status: Optional[int] = None, retryable: Optional[bool] = None) -> None:
        self.status = status
        self.retryable = (status is None or status in (408, 429) or 500 <= status <= 599) if retryable is None else bool(retryable)
        message = "平台接口连接失败" if status is None else "平台接口请求失败（HTTP %d）" % status
        super().__init__(message)

def _identifier(value: Any, limit: int = 64) -> str:
    if not isinstance(value, str) or len(value) > limit or not _IDENTIFIER.fullmatch(value) or value in (".", ".."):
        raise ValueError("接口标识必须为非空字母、数字或下划线等安全字符")
    return value


def _text(value: Any, limit: int = 128) -> str:
    if not isinstance(value, str) or not value.strip() or value != value.strip() or len(value) > limit or any(ord(c) < 32 or 127 <= ord(c) <= 159 for c in value):
        raise ValueError("配置文字字段为空、超长或含控制字符")
    return value


def _inside(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def _safe_storage(value: Union[str, Path]) -> Path:
    """禁止写入回归/取证目录，且不借助符号链接或 Windows 目录联接绕过检查。"""
    try:
        path = Path(value).absolute()
        resolved = path.resolve()
        # ★ 基线目录名走 settings 的常量：写死的话改名那天这道保护会
        #   悄悄指向一个不存在的目录，而真基线失去保护。
        from ..settings import BASELINE_SUBDIR
        for protected in (_REPOSITORY_ROOT.joinpath(*BASELINE_SUBDIR),
                          _REPOSITORY_ROOT / "re"):
            if _inside(path, protected) or _inside(resolved, protected.resolve()):
                raise ValueError("发件箱不能位于回归基线或取证目录")
        for ancestor in (path,) + tuple(path.parents):
            _reject_link(ancestor)
        if path.exists() and not path.is_dir():
            raise ValueError("发件箱位置必须为独立目录")
        return resolved
    except (OSError, RuntimeError, TypeError):
        raise ValueError("发件箱目录不可安全访问") from None


def _reject_link(path: Path) -> None:
    try:
        info = path.lstat()
    except FileNotFoundError:
        return
    if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400):
        raise ValueError("发件箱不接受符号链接或目录联接")


def _private_permissions(path: Path, mode: int) -> None:
    try:
        path.chmod(mode)
    except OSError:
        # Windows ACL 应由部署账号限制；chmod 只作为可用平台上的附加保护。
        pass


def _unique_object(pairs: List[Any]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("JSON 不允许重复字段")
        result[key] = value
    return result


def _reject_constant(_: str) -> None:
    raise ValueError("JSON 不允许非有限数值")


def _json_bytes(value: Any) -> bytes:
    try:
        encoded = json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    except (ValueError, TypeError, UnicodeError, RecursionError):
        raise ValueError("测量必须为可序列化的有限 JSON 数据") from None
    if len(encoded) > MAX_BYTES:
        raise ValueError("测量 JSON 超出大小上限")
    return encoded


def _validate_asset(sha256: Any, content: Any) -> None:
    if not isinstance(sha256, str) or not _SHA256.fullmatch(sha256):
        raise ValueError("候选资产必须声明小写 SHA-256")
    if not isinstance(content, bytes) or not 0 < len(content) <= MAX_BYTES or hashlib.sha256(content).hexdigest() != sha256:
        raise ValueError("候选资产大小或 SHA-256 校验失败")


def command_body(body: Dict[str, Any]) -> Dict[str, Any]:
    """规范化人工命令；完整五字段参与不可变关联，不包含运行状态。"""
    required = {"requestId", "channelId", "passageId", "kind", "confirmedEmpty"}
    if not isinstance(body, dict) or set(body) != required:
        raise ValueError("命令字段不完整或包含未知字段")
    for field in ("requestId", "channelId", "passageId"):
        _identifier(body[field], 64)
    if body["kind"] not in ("baseline-capture", "remeasure") or type(body["confirmedEmpty"]) is not bool:
        raise ValueError("命令类型或确认值错误")
    if body["kind"] == "baseline-capture" and not body["confirmedEmpty"]:
        raise ValueError("必须人工确认实际空车")
    if body["kind"] == "baseline-capture" and body["channelId"] != "CH4":
        raise ValueError("空车基线只能在CH4采集")
    return dict(body)
