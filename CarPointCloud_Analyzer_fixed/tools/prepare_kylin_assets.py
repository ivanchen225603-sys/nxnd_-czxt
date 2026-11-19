#!/usr/bin/env python3
"""生成银河麒麟 V10 SP1 x86_64 离线安装包所需的原生运行资产。

本脚本必须在可联网的构建机执行，不依赖构建机操作系统：

* 下载可再分发的 CPython 3.11 Linux x86_64 运行时；
* 为 GLIBC 2.31 的 Linux x86_64 下载全部预编译 wheels；
* 绝不从源码构建 NumPy、SciPy、Open3D，避免把构建机二进制误用于麒麟。

最终生成的 assets 仅由 ``build_kylin_package.py`` 消费，安装时完全离线。
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEPLOY_ROOT = ROOT / "deploy" / "kylin"
ASSETS_ROOT = DEPLOY_ROOT / "assets"
RUNTIME_DIR = ASSETS_ROOT / "runtime" / "python"
WHEELHOUSE_DIR = ASSETS_ROOT / "wheelhouse"
REQUIREMENTS = DEPLOY_ROOT / "requirements-kylin-py311.txt"

# CPython 3.11 是该项目当前受支持的最高版本。GNU build 适用于麒麟 V10 SP1
# x86_64；不选用 x86_64-v3/v4 构建，避免对兆芯 CPU 施加额外指令集要求。
PYTHON_RUNTIME_URL = (
    "https://github.com/astral-sh/python-build-standalone/releases/download/"
    "20260807/cpython-3.11.15%2B20260807-"
    "x86_64-unknown-linux-gnu-install_only.tar.gz"
)
LINUX_PLATFORM = "manylinux_2_31_x86_64"


def _safe_extract_tar(archive: Path, destination: Path) -> None:
    """安全解压受控运行时归档，拒绝任何试图写出目标目录的成员。"""
    destination.mkdir(parents=True, exist_ok=True)
    root = destination.resolve()
    with tarfile.open(archive, "r:gz") as tar:
        for member in tar.getmembers():
            member_path = (destination / member.name).resolve()
            if member_path != root and root not in member_path.parents:
                raise RuntimeError(f"运行时归档包含越界路径：{member.name}")
        # Python 3.11 尚未提供 tarfile 的 ``filter=`` 参数；路径已经逐个校验，
        # 因此按成员解压既兼容构建机 Python 3.11，又避免归档路径穿越。
        for member in tar.getmembers():
            tar.extract(member, destination)


def _download_runtime(refresh: bool) -> None:
    python_bin = RUNTIME_DIR / "bin" / "python3"
    if python_bin.is_file() and not refresh:
        print(f"复用现有 CPython 运行时：{python_bin}")
        return

    if RUNTIME_DIR.parent.exists():
        shutil.rmtree(RUNTIME_DIR.parent)
    with tempfile.TemporaryDirectory(prefix="carpoint-kylin-runtime-") as tmp:
        archive = Path(tmp) / "python-runtime.tar.gz"
        print("下载 CPython 3.11 Linux x86_64 运行时…")
        urllib.request.urlretrieve(PYTHON_RUNTIME_URL, archive)
        # install_only 归档内的根目录为 python/。先解到 runtime，再由构建脚本
        # 直接把 runtime/python 打进最终安装包。
        _safe_extract_tar(archive, RUNTIME_DIR.parent)

    if not python_bin.is_file():
        raise RuntimeError(f"CPython 运行时解压后缺少：{python_bin}")


def _download_wheelhouse(refresh: bool) -> None:
    if refresh and WHEELHOUSE_DIR.exists():
        shutil.rmtree(WHEELHOUSE_DIR)
    WHEELHOUSE_DIR.mkdir(parents=True, exist_ok=True)
    existing = list(WHEELHOUSE_DIR.glob("open3d-0.19.0-*.whl"))

    # NumPy/SciPy/Matplotlib 等通用科学计算轮子仍使用 manylinux2014 标签；
    # Open3D 0.19 则使用 manylinux_2_31。目标麒麟的 GLIBC 2.31 同时兼容它们，
    # 但 pip 的一次跨平台解析只能指定一种平台标签，因此拆成两次下载。
    requirements = [
        line.strip()
        for line in REQUIREMENTS.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
        and not line.strip().startswith("open3d==")
    ]
    common_command = [
        sys.executable,
        "-m",
        "pip",
        "download",
        "--dest",
        str(WHEELHOUSE_DIR),
        "--only-binary=:all:",
        "--platform",
        "manylinux2014_x86_64",
        "--implementation",
        "cp",
        "--python-version",
        "311",
        "--abi",
        "cp311",
    ]
    open3d_command = [
        sys.executable,
        "-m",
        "pip",
        "download",
        "--dest",
        str(WHEELHOUSE_DIR),
        "--only-binary=:all:",
        "--no-deps",
        "--platform",
        LINUX_PLATFORM,
        "--implementation",
        "cp",
        "--python-version",
        "311",
        "--abi",
        "cp311",
        "open3d==0.19.0",
    ]
    print("下载 GLIBC 2.31 / CPython 3.11 Linux x86_64 wheels…")
    # Windows 构建机可能仍以 GBK 作为 locale；依赖锁定文件保留中文说明时，
    # pip 会按 locale 解码而失败。显式开启 UTF-8 模式不会影响目标 Linux wheels。
    environment = os.environ.copy()
    environment.setdefault("PYTHONUTF8", "1")
    subprocess.run([*common_command, *requirements], check=True, env=environment)
    if existing and not refresh:
        print(f"复用现有 Open3D Linux wheel：{existing[0].name}")
    else:
        subprocess.run(open3d_command, check=True, env=environment)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--refresh",
        action="store_true",
        help="重新下载运行时和所有 wheels。",
    )
    parser.add_argument(
        "--skip-runtime",
        action="store_true",
        help="不下载 CPython 运行时。",
    )
    parser.add_argument(
        "--skip-wheelhouse",
        action="store_true",
        help="不下载 wheelhouse。",
    )
    args = parser.parse_args()

    if not REQUIREMENTS.is_file():
        raise RuntimeError(f"缺少依赖锁定文件：{REQUIREMENTS}")
    if not args.skip_runtime:
        _download_runtime(args.refresh)
    if not args.skip_wheelhouse:
        _download_wheelhouse(args.refresh)

    print("\n资产准备完成：")
    print(f"  runtime: {RUNTIME_DIR}")
    print(f"  wheels : {WHEELHOUSE_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
