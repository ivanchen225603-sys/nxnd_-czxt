"""HTTP 传输适配：请求模型、原路由与异常映射；不创建算法或数据库。"""

from contextlib import asynccontextmanager
from datetime import datetime
from typing import Any, Callable, Protocol

from carpointcloud_analyzer.application.service_contracts import AnalyzeRequest, BatchAnalyzeRequest, PairAnalyzeRequest, MAX_BATCH_SIZE

class AnalysisBackend(Protocol):
    """HTTP 所需的应用端口；适配层不依赖具体执行器或数据库实现。"""

    # 端口方法的语义由实现方（AnalysisService）的同名方法定义，此处只声明形状。
    async def analyze(self, request: AnalyzeRequest) -> dict[str, Any]:
        """分析单份点云。"""
        ...

    async def analyze_batch(self, request: BatchAnalyzeRequest) -> dict[str, Any]:
        """顺序分析一批点云。"""
        ...

    async def analyze_pair(self, request: PairAnalyzeRequest) -> dict[str, Any]:
        """合并固定 A/B 双雷达的两份点云后分析。"""
        ...

    async def preview(self, payload: Any) -> dict[str, Any]:
        """只读补点预览，不写库、不建档案。"""
        ...

    async def health(self) -> dict[str, Any]:
        """返回服务能力与配置就绪情况。"""
        ...

    def startup(self) -> None:
        """启动服务后台工作。"""
        ...

    def shutdown(self) -> None:
        """释放执行器等运行期资源。"""
        ...




def build_http_app(service_factory: Callable[[], AnalysisBackend]):
    """注册传输契约，在依赖与模型校验之后装配业务服务。"""
    try:
        from fastapi import FastAPI
        from pydantic import BaseModel, Field
    except ImportError as exc:
        raise RuntimeError(
            "FastAPI service dependencies are missing. "
            "Install them with: pip install -r requirements.txt"
        ) from exc

    class AnalyzePayload(BaseModel):
        """单文件 HTTP 请求模型。"""
        pcdPath: str = Field(..., min_length=1)
        camera: str = ""
        plate: str = ""
        baselineDir: str | None = None  # 旧调用方仍会发送，接受但不使用。
        inputUnit: str = "auto"
        workflowId: str | None = Field(default=None, max_length=128)
        capturedAt: datetime | None = None

        def to_request(self) -> AnalyzeRequest:
            """转换为与 Web 框架解耦的单文件请求。"""
            return AnalyzeRequest(
                pcd_path=self.pcdPath,
                camera=self.camera,
                plate=self.plate,
                input_unit=self.inputUnit,
                workflow_id=self.workflowId,
                captured_at=self.capturedAt,
            )

    class BatchAnalyzePayload(BaseModel):
        """批量 HTTP 请求模型。"""
        pcdPaths: list[str] = Field(
            ..., min_length=1, max_length=MAX_BATCH_SIZE
        )
        camera: str = ""
        plate: str = ""
        baselineDir: str | None = None  # 旧调用方仍会发送，接受但不使用。
        inputUnit: str = "auto"

        def to_request(self) -> BatchAnalyzeRequest:
            """转换为与 Web 框架解耦的批量请求。"""
            return BatchAnalyzeRequest(
                pcd_paths=tuple(self.pcdPaths),
                camera=self.camera,
                plate=self.plate,
                input_unit=self.inputUnit,
            )

    class PairAnalyzePayload(BaseModel):
        """固定 A/B 双雷达 HTTP 请求模型。"""
        pcdPathA: str = Field(..., min_length=1)
        cameraA: str = Field(..., min_length=1)
        pcdPathB: str = Field(..., min_length=1)
        cameraB: str = Field(..., min_length=1)
        pairId: str = Field(..., min_length=1, max_length=64)
        workflowId: str = Field(..., min_length=1, max_length=128)
        # 服务端配置 CARPOINT_EXTRINSIC_PATH 后可省略；老 Worker 仍可发送同一路径。
        extrinsicPath: str | None = Field(default=None, min_length=1)
        plate: str = ""
        baselineDir: str | None = None  # 旧调用方仍会发送，接受但不使用。
        inputUnit: str = "m"
        capturedAt: datetime | None = None

        def to_request(self) -> PairAnalyzeRequest:
            """转换为经过业务校验的双雷达请求。"""
            return PairAnalyzeRequest(
                pcd_path_a=self.pcdPathA,
                camera_a=self.cameraA,
                pcd_path_b=self.pcdPathB,
                camera_b=self.cameraB,
                pair_id=self.pairId,
                workflow_id=self.workflowId,
                extrinsic_path=self.extrinsicPath,
                plate=self.plate,
                input_unit=self.inputUnit,
                captured_at=self.capturedAt,
            )

    class PreviewPayload(BaseModel):
        """Worker 已配对并转换到 A 坐标系的抽样；禁止身份/路径/业务字段。"""
        model_config = {"extra": "forbid"}
        points: str = Field(..., max_length=960000)
        pointsEncoding: str
        pointCountA: int = Field(..., gt=0, le=30000, strict=True)
        pointCountB: int = Field(..., gt=0, le=30000, strict=True)

    service = service_factory()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        """在应用退出时等待线程池任务结束。"""
        service.startup()
        try:
            yield
        finally:
            service.shutdown()

    app = FastAPI(
        title="CarPointCloud Analyzer Service",
        version="1.2.1",
        description=(
            "Resident single-file and fixed A/B pair point-cloud analysis service "
            "capped at 4 concurrent jobs."
        ),
        lifespan=lifespan,
    )
    app.state.analysis_service = service

    @app.get("/health")
    async def health() -> dict[str, Any]:
        """返回并发、路径、数据库和空车存储健康状态。"""
        return await service.health()

    @app.post("/analyze")
    async def analyze(payload: AnalyzePayload) -> dict[str, Any]:
        """接收并执行单份 PCD 分析。"""
        return await _handle_service_call(lambda: service.analyze(payload.to_request()))

    @app.post("/analyze-batch")
    async def analyze_batch(payload: BatchAnalyzePayload) -> dict[str, Any]:
        """接收并执行共享配置的 PCD 批次。"""
        return await _handle_service_call(lambda: service.analyze_batch(payload.to_request()))

    @app.post("/analyze-pair")
    async def analyze_pair(payload: PairAnalyzePayload) -> dict[str, Any]:
        """接收并执行固定 A/B 双雷达融合分析。"""
        return await _handle_service_call(lambda: service.analyze_pair(payload.to_request()))

    @app.post("/preview-completion")
    async def preview_completion(payload: PreviewPayload) -> dict[str, Any]:
        """只读参考补点，不返回正式尺寸，不落盘/入库。"""
        return await _handle_service_call(lambda: service.preview(payload))

    return app


async def _handle_service_call(call):
    """业务输入异常映射为 400；其余异常由框架记录并返回 500。"""
    from fastapi import HTTPException

    try:
        return await call()
    except (FileNotFoundError, ValueError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
