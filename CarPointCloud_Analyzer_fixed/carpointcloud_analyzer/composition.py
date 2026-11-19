"""启动组合根：选择 HTTP、算法及存储实现。"""
import argparse
import faulthandler
import hashlib
import logging
import os
import sys
from pathlib import Path
from carpointcloud_analyzer.application.service import (
    AnalysisService, AnalyzerFactory, MAX_SERVICE_CONCURRENCY, DEFAULT_HOST, DEFAULT_PORT,
)
from carpointcloud_analyzer.application.pipeline import PointCloudAnalyzer
from carpointcloud_analyzer.application.ports import DimensionStore
from carpointcloud_analyzer.infrastructure.persistence import Dm8DimensionStore
from carpointcloud_analyzer.infrastructure.profile_storage import EmptyVehicleProfileStorage, DisabledEmptyVehicleProfileStorage


def create_app(
    max_concurrency: int = MAX_SERVICE_CONCURRENCY,
    analyzer_factory: AnalyzerFactory = PointCloudAnalyzer,
    dimension_store: DimensionStore | None = None,
    empty_profile_storage: EmptyVehicleProfileStorage
    | DisabledEmptyVehicleProfileStorage
    | None = None,
    extrinsic_path: str | None = None,
):
    """创建常驻 FastAPI 应用并绑定分析服务生命周期。

    Args:
        max_concurrency: 进程内最大并发分析数。
        analyzer_factory: 分析器创建工厂。
        dimension_store: 可注入的尺寸存储；为空时按环境创建 DM8 存储。
        empty_profile_storage: 可注入的空车点云存储；为空时按环境创建。
        extrinsic_path: 场地外参；为空时读取 CARPOINT_EXTRINSIC_PATH。

    Returns:
        已注册健康、分析和预览端点的 FastAPI 应用。
    """
    from carpointcloud_analyzer.api.http_transport import build_http_app

    def service_factory() -> AnalysisService:
        """组合根负责选择后端；请求模型仍先于后端配置校验。"""
        return AnalysisService(
            max_concurrency=max_concurrency,
            analyzer_factory=analyzer_factory,
            extrinsic_path=extrinsic_path or os.getenv("CARPOINT_EXTRINSIC_PATH"),
            dimension_store=dimension_store or Dm8DimensionStore.from_env(),
            empty_profile_storage=empty_profile_storage or EmptyVehicleProfileStorage.from_env(),
        )

    return build_http_app(service_factory)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """解析服务监听、并发和场地外参参数。"""
    parser = argparse.ArgumentParser(
        description="Run the resident CarPointCloud analyzer HTTP service."
    )
    parser.add_argument("--host", default=os.getenv("CARPOINT_SERVICE_HOST", DEFAULT_HOST))
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.getenv("CARPOINT_SERVICE_PORT", str(DEFAULT_PORT))),
    )
    parser.add_argument(
        "--max-concurrency",
        type=int,
        default=int(
            os.getenv("CARPOINT_SERVICE_MAX_CONCURRENCY", str(MAX_SERVICE_CONCURRENCY))
        ),
    )
    # 旧部署命令仍可能带这两个参数；接受但不使用，避免启动失败。
    parser.add_argument("--allowed-root", help=argparse.SUPPRESS)
    parser.add_argument("--baseline-dir", help=argparse.SUPPRESS)
    parser.add_argument(
        "--extrinsic-path",
        default=os.getenv("CARPOINT_EXTRINSIC_PATH"),
        help=(
            "Fixed site B_to_A extrinsic JSON. When set it wins over the request "
            "field and is loaded once at startup."
        ),
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """读取当前目录 appsettings.json，创建应用并启动 Uvicorn 常驻服务。"""
    from carpointcloud_analyzer.infrastructure.runtime_configuration import load_runtime_configuration
    from carpointcloud_analyzer.infrastructure.runtime_diagnostics import enable_gc_timing
    import uvicorn

    # 标准启动只使用当前文件。算法工厂/单元测试入口不会加载真实配置。
    load_runtime_configuration(Path.cwd() / "appsettings.json")
    # Uvicorn只配置自身日志；应用INFO必须显式开启，才能保留算法与数据库阶段耗时。
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(threadName)s %(name)s %(message)s")
    logging.getLogger("carpointcloud_analyzer").setLevel(logging.INFO)
    faulthandler.enable(all_threads=True)
    enable_gc_timing()
    args = parse_args(argv)
    logging.getLogger("carpointcloud_analyzer").info(
        "Analyzer启动 pid=%s python=%s concurrency=%s OMP_NUM_THREADS=%s OPENBLAS_NUM_THREADS=%s MKL_NUM_THREADS=%s",
        os.getpid(), sys.executable, args.max_concurrency, os.getenv("OMP_NUM_THREADS"),
        os.getenv("OPENBLAS_NUM_THREADS"), os.getenv("MKL_NUM_THREADS"))
    package = Path(__file__).parent
    source_hash = hashlib.sha256()
    for path in sorted(package.rglob("*.py")):
        source_hash.update(path.relative_to(package).as_posix().encode())
        source_hash.update(path.read_bytes())
    logging.getLogger("carpointcloud_analyzer").info(
        "Analyzer代码标识 pid=%s source_sha256=%s faulthandler=%s",
        os.getpid(), source_hash.hexdigest(), faulthandler.is_enabled())
    app = create_app(max_concurrency=args.max_concurrency, extrinsic_path=args.extrinsic_path)
    uvicorn.run(app, host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
