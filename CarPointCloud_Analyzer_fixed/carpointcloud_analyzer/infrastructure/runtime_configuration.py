"""按文件加载服务配置；不区分本地和生产。"""
from __future__ import annotations

import json
import os
from pathlib import Path


def load_runtime_configuration(path: Path) -> None:
    """用文件内容替换本进程 CARPOINT_* 设置；布尔值写成 true/false。"""
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    for key in [key for key in os.environ if key.startswith("CARPOINT_")]:
        del os.environ[key]
    os.environ.update({
        key: str(value).lower() if isinstance(value, bool) else str(value)
        for key, value in data.items()
    })
