"""基础环境配置解析，不绑定任何业务后端。"""

import os


def env_bool(name: str, default: bool) -> bool:
    """解析严格的布尔环境变量。"""
    raw = os.getenv(name)
    if raw is None:
        return default
    normalized = raw.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} 必须是 true/false")
