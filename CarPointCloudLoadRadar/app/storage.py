#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
storage.py — 落盘样本的盘点与清理（纯函数，不打印、不解析命令行）

一份"样本"= 同名的三个文件：

    <时间>.xyz              点云（原厂格式）
    <时间>.xyz.reason.txt   判定理由
    <时间>.xyz.meta.json    诊断信息（本项目新增）

以 `.xyz` 为准：附属两个文件跟着它一起算、一起删，不会留下
"有理由文件却没有点云"的半拉记录。

为什么单独放一个模块：命令行（`cli/cleanup.py`）和桌面版
（`载重雷达检测系统.exe --cleanup`）都要用。桌面版是打包成 exe 的，
`cli/` 下的脚本在里面 import 不到，所以真正的逻辑必须待在 `app/` 里。

★ 唯一一处会删文件的代码。三条硬规矩写在这里，调用方不许绕过：
    1. 只删 `.xyz` + 它的两个附属文件，其它文件一律不碰；
    2. `is_baseline()` 命中的目录（`baseline` 及其子目录）由调用方负责拒绝
       —— 那是 cli/verify.py 的回归基线，删了闸门会变成"0 个样本全部通过"；
    3. 只删空目录，绝不删输出目录本身。
"""
from __future__ import annotations

import json
import os
import time
from typing import Dict, List, Optional, Tuple

from . import settings as _settings
from .settings import root

XYZ_SUFFIXES = (".reason.txt", ".meta.json")


def is_archive(path: str) -> bool:
    """成套归档由其日志管理；旧版逐 XYZ 清理绝不能拆包。"""
    paths = (os.path.abspath(path), os.path.realpath(path))
    return any("archives-v1" in os.path.normcase(item).split(os.sep) for item in paths)


def baseline_dir() -> str:
    """回归基线目录（10 份原版样本 + reason.txt）—— 任何清理都必须绕开它。

    ★ 必须**每次调用时**算，不能在 import 时算成常量：
      打包成 exe 后，desktop/app.py 会在启动过程中把 app.settings.PROJECT_ROOT
      换成 exe 所在目录（打包解包目录 _MEIPASS ≠ 程序目录）。import 时定下来的
      常量会停在 _MEIPASS 上，于是"别碰回归基线"这道闸门在打包版里等于没有。
    """
    # ★ 和 PROJECT_ROOT 同理，常量也要**调用时**取：用 from-import 绑进来的话
    #   拿到的是 import 那一刻的值，之后改不动 —— 而这正是本函数上面
    #   那段说明想防的事。
    return os.path.normcase(os.path.abspath(root(*_settings.BASELINE_SUBDIR)))


def is_baseline(path: str) -> bool:
    """这个目录是不是回归基线（或它的子目录）。"""
    base = baseline_dir()
    p = os.path.normcase(os.path.abspath(path))
    return p == base or p.startswith(base + os.sep)


def human(n: float) -> str:
    """字节数转人话。"""
    if n < 1024:
        return f"{int(n)} B"
    v = float(n)
    for unit in ("KB", "MB", "GB", "TB"):
        v /= 1024.0
        if v < 1024 or unit == "TB":
            return f"{v:.1f} {unit}"
    return f"{v:.1f} TB"


def scan(out_dir: str) -> List[Tuple[str, float, int]]:
    """扫出目录下所有样本。返回 [(xyz 绝对路径, mtime, 这一份的总字节数)]。"""
    out: List[Tuple[str, float, int]] = []
    if not os.path.isdir(out_dir) or is_archive(out_dir):
        return out
    for r, _dirs, files in os.walk(out_dir):
        _dirs[:] = [name for name in _dirs if not is_archive(os.path.join(r, name))]
        for f in files:
            if not f.endswith(".xyz"):
                continue
            p = os.path.join(r, f)
            try:
                mt = os.path.getmtime(p)
                size = os.path.getsize(p)
            except OSError:
                continue
            for suf in XYZ_SUFFIXES:
                try:
                    size += os.path.getsize(p + suf)
                except OSError:
                    pass
            out.append((p, mt, size))
    return out


def sample_is_empty(xyz: str) -> bool:
    """这一份落盘样本是不是**确认的空车**。判不出来时返回 False。

    读的是 `<xyz>.meta.json` 的 `empty` —— 它记的是**判定链结论**
    （2026-09-17 修正过：以前写的是六面拟合是否成功，137 份现场样本里
    115 份标反了，照它挑基线会把载货点云存成空车基线）。
    `fit_ok` 是拟合结果，**不能拿来当空车判据**。

    为什么要有这个：现场要求"最新的空车车厢永久保存、载重云图只留最近
    N 天"，而按天数清理是按 mtime 一刀切的，会把空车一起删掉。

    ▲ **判不出来时按"不是空车"处理**，也就是照常参与清理。第一版写反了
      （读不出来就保留），后果是：老样本、没有 meta 的文件、meta 坏掉的
      文件**全部永久豁免**，7 天策略对现存的几百 MB 历史数据等于没有。
      之所以敢这么定，是因为空车真正的底本**不在这里**：
        · `data/in/*.npz` 是基线本体，根本不在 `output_dir` 下，清理扫不到；
        · 成套归档按 manifest 的 `empty_observation` 保护，与本文件无关。
      落盘的这一份只是**投影**，删了不丢底本。
    """
    try:
        with open(xyz + ".meta.json", "rb") as handle:
            meta = json.loads(handle.read().decode("utf-8-sig"))
    except (OSError, ValueError):
        return False
    return meta.get("empty") is True


def remove_sample(xyz: str) -> Tuple[int, int]:
    """删掉一份样本（点云 + 理由 + 元数据）。返回 (删掉几个文件, 释放多少字节)。"""
    if is_archive(xyz):
        return 0, 0
    n = b = 0
    for p in (xyz,) + tuple(xyz + s for s in XYZ_SUFFIXES):
        try:
            b += os.path.getsize(p)
            os.remove(p)
            n += 1
        except OSError:
            pass
    return n, b


def prune_empty_dirs(out_dir: str) -> int:
    """删掉清空后的车牌目录（只删空目录，且不碰 out_dir 自己）。"""
    killed = 0
    base = os.path.abspath(out_dir)
    for r, dirs, files in os.walk(out_dir, topdown=False):
        if is_archive(r):
            continue
        if os.path.abspath(r) == base:
            continue
        if not dirs and not files:
            try:
                os.rmdir(r)
                killed += 1
            except OSError:
                pass
    return killed


def summarize(out_dir: str, days: float = 0.0,
              items: Optional[List[Tuple[str, float, int]]] = None) -> Dict:
    """盘点一个输出目录。**只读**，不删任何东西。

    days > 0 时顺便算出"超过这么多天"的那部分（`old` / `old_bytes` / `old_items`）。
    """
    items = scan(out_dir) if items is None else items
    total = sum(x[2] for x in items)
    cutoff = (time.time() - days * 86400.0) if days > 0 else None
    old = [x for x in items if cutoff is not None and x[1] < cutoff]
    d: Dict = {
        "dir": out_dir,
        "exists": os.path.isdir(out_dir),
        "samples": len(items),
        "bytes": total,
        "oldest": min((x[1] for x in items), default=0.0),
        "newest": max((x[1] for x in items), default=0.0),
        "per_day": 0.0,
        "bytes_per_day": 0.0,
        "days": days,
        "old": len(old),
        "old_bytes": sum(x[2] for x in old),
        "old_items": old,
    }
    span = (d["newest"] - d["oldest"]) / 86400.0
    if len(items) > 1 and span > 0.5:
        d["per_day"] = len(items) / span
        d["bytes_per_day"] = total / span
    return d


def dir_usage(path: str, top: int = 3) -> Dict:
    """一个目录占了多少（不分文件类型，纯统计）。**只读**。

    给"磁盘到底被谁吃了"这个问题用的：现场真正的大头往往不是点云样本，
    而是 logs/desktop-profile（浏览器窗口后端的用户数据目录，实测能涨到 500MB+）
    和 logs/export（每点一次"导出 PLY"就多一份，约 2MB）。
    这些 cli/cleanup.py 不会去删，但必须让人看得见。
    """
    total = files = 0
    subs: Dict[str, int] = {}
    if not os.path.isdir(path):
        return {"dir": path, "exists": False, "bytes": 0, "files": 0, "top": []}
    for r, _dirs, fs in os.walk(path):
        n = 0
        for f in fs:
            try:
                n += os.path.getsize(os.path.join(r, f))
            except OSError:
                continue
            files += 1
        total += n
        # 只归到一级子目录上
        rel = os.path.relpath(r, path).split(os.sep)[0]
        key = rel if rel not in (".", "") else "(根)"
        subs[key] = subs.get(key, 0) + n
    ranked = sorted(subs.items(), key=lambda kv: -kv[1])[:top]
    return {"dir": path, "exists": True, "bytes": total, "files": files,
            "top": ranked}


def purge(out_dir: str, days: float) -> Dict:
    """真删：清掉超过 days 天的样本，再删掉空掉的车牌目录。

    调用方必须先自己挡住 `is_baseline(out_dir)` —— 这里不替它做决定，
    但会在 days <= 0 时直接拒绝（"保留 0 天"= 删光，绝不是谁想要的默认行为）。
    """
    if days <= 0:
        return {"removed": 0, "removed_files": 0, "removed_bytes": 0,
                "removed_dirs": 0, "refused": "未指定保留天数，不做任何删除"}
    info = summarize(out_dir, days)
    files = nbytes = 0
    for p, _mt, _sz in info["old_items"]:
        n, b = remove_sample(p)
        files += n
        nbytes += b
    dirs = prune_empty_dirs(out_dir) if files else 0
    return {"removed": info["old"], "removed_files": files,
            "removed_bytes": nbytes, "removed_dirs": dirs, "refused": ""}
