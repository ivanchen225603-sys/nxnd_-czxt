"""WebApi 桥接配置入口：文件解析、地址和通道配置校验。"""
from __future__ import annotations
import ipaddress
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Mapping, Union
from urllib import parse
from .bridge_boundary import _identifier, _text, _safe_storage, _unique_object, _reject_constant

@dataclass(frozen=True)
class ChannelConfig:
    calibration_id: str
    # JT128 单帧最早有效位置观测到末包接收的实测上界。未知时只触发采集。
    standstill_observation_age_bound_ms: float | None = None

    def __post_init__(self) -> None:
        _identifier(self.calibration_id)
        bound = self.standstill_observation_age_bound_ms
        if bound is not None and (type(bound) not in (int, float) or not math.isfinite(bound) or bound <= 0):
            raise ValueError("静止观测时延上界必须是现场核定的正毫秒数或 null")


@dataclass(frozen=True)
class BridgeConfig:
    node_id: str
    base_url: str = field(repr=False)
    agent_key: str = field(repr=False)
    storage_dir: Path
    algorithm_version: str
    channels: Mapping[str, ChannelConfig]
    enabled: bool = False
    timeout_sec: float = 10.0
    retry_sec: float = 5.0
    allow_insecure_http: bool = False

    def __post_init__(self) -> None:
        _identifier(self.node_id, 64)
        _text(self.algorithm_version, 64)
        if not isinstance(self.agent_key, str) or not 32 <= len(self.agent_key) <= 4096 or any(not 33 <= ord(c) <= 126 for c in self.agent_key):
            raise ValueError("节点凭据必须包含 32 至 4096 个可见 ASCII 字符")
        if type(self.enabled) is not bool or type(self.allow_insecure_http) is not bool:
            raise ValueError("启用开关必须为布尔值")
        for value, lower, upper in ((self.timeout_sec, 1, 30), (self.retry_sec, 1, 300)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError("接口超时或重试间隔超出配置范围")
        object.__setattr__(self, "base_url", self._base_url())
        object.__setattr__(self, "storage_dir", _safe_storage(self.storage_dir))
        if not isinstance(self.channels, dict) or not 1 <= len(self.channels) <= 16:
            raise ValueError("通道映射必须为含 1 至 16 项的对象")
        channels: Dict[str, ChannelConfig] = {}
        for key, value in self.channels.items():
            _identifier(key, 64)
            if isinstance(value, dict):
                if "calibration_id" not in value or set(value) - {"calibration_id", "standstill_observation_age_bound_ms"}:
                    raise ValueError("通道配置缺失字段或包含未知字段")
                value = ChannelConfig(**value)
            if not isinstance(value, ChannelConfig):
                raise ValueError("通道配置格式无效")
            channels[key] = value
        object.__setattr__(self, "channels", channels)

    def _base_url(self) -> str:
        if not isinstance(self.base_url, str) or any(ord(c) <= 32 or ord(c) == 127 for c in self.base_url):
            raise ValueError("平台地址格式无效")
        try:
            parts = parse.urlsplit(self.base_url)
            if parts.scheme not in ("http", "https") or not parts.hostname or parts.username is not None or parts.password is not None:
                raise ValueError()
            if "?" in self.base_url or "#" in self.base_url or "\\" in self.base_url or parts.port == 0:
                raise ValueError()
            loopback = parts.hostname.lower() == "localhost"
            if not loopback:
                try:
                    loopback = ipaddress.ip_address(parts.hostname).is_loopback
                except ValueError:
                    pass
            if parts.scheme == "http" and not loopback and not self.allow_insecure_http:
                raise ValueError()
        except (ValueError, UnicodeError):
            raise ValueError("平台地址必须为无凭据、查询和片段的 HTTPS 地址；非本机 HTTP 需显式允许") from None
        return self.base_url.rstrip("/")

    @classmethod
    def load(cls, path: Union[str, Path]) -> "BridgeConfig":
        """相对 storage_dir 按配置文件目录解析；缺少 enabled 时保持关闭。"""
        try:
            config_path = Path(path)
            with config_path.open("r", encoding="utf-8-sig") as stream:
                raw = stream.read(1024 * 1024 + 1)
            if len(raw) > 1024 * 1024:
                raise ValueError("配置文件超出大小上限")
            data = json.loads(raw, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
        except (OSError, UnicodeError, json.JSONDecodeError, RecursionError):
            raise ValueError("载货雷达接口配置无法读取或 JSON 格式无效") from None
        required = {"node_id", "base_url", "agent_key", "storage_dir", "algorithm_version", "channels"}
        optional = {"enabled", "timeout_sec", "retry_sec", "allow_insecure_http"}
        if not isinstance(data, dict) or not required <= set(data) or set(data) - required - optional:
            raise ValueError("载货雷达接口配置缺失字段或包含未知字段")
        if not isinstance(data["storage_dir"], str) or not data["storage_dir"].strip():
            raise ValueError("必须配置独立的发件箱目录")
        directory = Path(data["storage_dir"])
        if not directory.is_absolute():
            directory = config_path.resolve().parent / directory
        data["storage_dir"] = directory
        return cls(**data)
