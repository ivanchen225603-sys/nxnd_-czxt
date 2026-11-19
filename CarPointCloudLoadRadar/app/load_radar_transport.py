"""兼容既有桥接导入路径；具体实现按配置、HTTP、SQLite 职责分离。"""
from .adapters.webapi_config import BridgeConfig as BridgeConfig, ChannelConfig as ChannelConfig
from .adapters.webapi_http import HttpClient as HttpClient, HttpError as HttpError
from .adapters.sqlite_outbox import SQLiteOutbox as SQLiteOutbox, OutboxItem as OutboxItem
from .adapters.bridge_boundary import (
    MAX_BYTES as MAX_BYTES,
    _REPOSITORY_ROOT as _REPOSITORY_ROOT,
    _IDENTIFIER as _IDENTIFIER,
    _SHA256 as _SHA256,
    _identifier as _identifier,
    _text as _text,
    _inside as _inside,
    _safe_storage as _safe_storage,
    _reject_link as _reject_link,
    _private_permissions as _private_permissions,
    _unique_object as _unique_object,
    _reject_constant as _reject_constant,
    _json_bytes as _json_bytes,
    _validate_asset as _validate_asset,
    command_body as command_body,
)
