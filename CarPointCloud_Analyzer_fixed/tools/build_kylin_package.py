#!/usr/bin/env python3
"""构建银河麒麟 V10 SP1 x86_64 的可交付离线安装归档。"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import tarfile
import tempfile
from datetime import UTC, datetime
from pathlib import Path, PurePosixPath


ROOT = Path(__file__).resolve().parents[1]
DEPLOY_ROOT = ROOT / "deploy" / "kylin"
ASSETS_ROOT = DEPLOY_ROOT / "assets"
DIST_ROOT = ROOT / "dist"
PACKAGE_NAME = "carpointcloud-analyzer-kylin-x86_64-1.2.1"


def _copy_app(destination: Path) -> None:
    """复制运行所需源码，明确排除缓存、日志、测试和开发报告。"""
    destination.mkdir(parents=True, exist_ok=True)
    source_package = ROOT / "carpointcloud_analyzer"
    if not source_package.is_dir():
        raise RuntimeError(f"缺少应用源码目录：{source_package}")
    shutil.copytree(
        source_package,
        destination / "carpointcloud_analyzer",
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
    )


def _copy_deploy_files(destination: Path) -> None:
    """复制部署脚本；assets 由单独路径处理，避免误带构建缓存。"""
    for filename in (
        "install.sh",
        "uninstall.sh",
        "preflight.sh",
        "verify-install.sh",
        "README.md",
        "requirements-kylin-py311.txt",
    ):
        target = destination / filename
        shutil.copy2(DEPLOY_ROOT / filename, target)
        if target.suffix == ".sh":
            # Windows 工作区通常不保留 Unix 可执行位；归档必须在麒麟解压后即可执行。
            target.chmod(target.stat().st_mode | 0o111)
    shutil.copytree(DEPLOY_ROOT / "config", destination / "config")
    shutil.copytree(DEPLOY_ROOT / "systemd", destination / "systemd")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_manifest(package_root: Path) -> None:
    """写入安装前校验使用的 SHA-256 清单，不把清单自身递归包含进去。"""
    lines: list[str] = []
    for path in sorted(package_root.rglob("*")):
        if not path.is_file() or path.name == "SHA256SUMS":
            continue
        relative = path.relative_to(package_root).as_posix()
        lines.append(f"{_sha256(path)}  {relative}")
    (package_root / "SHA256SUMS").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _assert_assets() -> None:
    required = (
        ASSETS_ROOT / "runtime" / "python" / "bin" / "python3",
        ASSETS_ROOT / "wheelhouse",
    )
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        joined = "\n  ".join(missing)
        raise RuntimeError(
            "缺少麒麟原生资产，请先执行：\n"
            "  python tools/prepare_kylin_assets.py\n"
            f"缺少：\n  {joined}"
        )
    if not list((ASSETS_ROOT / "wheelhouse").glob("open3d-0.19.0-*.whl")):
        raise RuntimeError("wheelhouse 中缺少 open3d==0.19.0 的 Linux wheel")


def _build(destination: Path) -> None:
    _assert_assets()
    package_root = destination / PACKAGE_NAME
    package_root.mkdir(parents=True)
    _copy_app(package_root / "app")
    _copy_deploy_files(package_root)
    shutil.copytree(ASSETS_ROOT / "runtime", package_root / "runtime")
    shutil.copytree(ASSETS_ROOT / "wheelhouse", package_root / "wheelhouse")
    # Windows 文件系统不具备 Unix 可执行位语义。为 Linux 运行时 bin 下的实际
    # 可执行文件补齐 0755，保证服务器解压后无需依赖宿主系统的权限继承策略。
    for executable in (package_root / "runtime" / "python" / "bin").iterdir():
        if executable.is_file():
            executable.chmod(executable.stat().st_mode | 0o111)

    metadata = {
        "package": PACKAGE_NAME,
        "target": {
            "os": "银河麒麟桌面操作系统 V10 SP1",
            "architecture": "x86_64",
            "minimumGlibc": "2.31",
            "python": "3.11",
        },
        "serverDefault": {
            "host": "127.0.0.1",
            "port": 8010,
            "cpu": "兆芯 KaiXian KX-U6780A / 8 核",
        },
        "builtAtUtc": datetime.now(UTC).isoformat(),
    }
    (package_root / "package-metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    _write_manifest(package_root)


def _archive_filter(member: tarfile.TarInfo) -> tarfile.TarInfo:
    """为跨 Windows 构建的 Linux 归档补齐必须的可执行权限。"""
    path = PurePosixPath(member.name)
    is_shell_script = path.suffix == ".sh"
    is_python_runtime_binary = "runtime/python/bin" in path.as_posix()
    if member.isfile() and (is_shell_script or is_python_runtime_binary):
        member.mode = 0o755
    return member


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DIST_ROOT,
        help="归档输出目录，默认项目根目录下 dist。",
    )
    args = parser.parse_args()

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    archive = output_dir / f"{PACKAGE_NAME}.tar.gz"
    if archive.exists():
        archive.unlink()

    with tempfile.TemporaryDirectory(prefix="carpoint-kylin-package-") as temp:
        stage = Path(temp)
        _build(stage)
        with tarfile.open(archive, "w:gz", compresslevel=6) as tar:
            tar.add(
                stage / PACKAGE_NAME,
                arcname=PACKAGE_NAME,
                filter=_archive_filter,
            )

    print(f"安装包已生成：{archive}")
    print(f"SHA-256：{_sha256(archive)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
