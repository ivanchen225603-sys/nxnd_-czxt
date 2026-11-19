"""常驻分析编排：读取输入、融合双云、调用算法并持久化结果。"""
import asyncio
from collections import deque
import hashlib
from itertools import count
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import numpy as np

from carpointcloud_analyzer.application.dual_cloud_merge import DualCloudMergeError, merge_vehicle_clouds
from carpointcloud_analyzer.application.pipeline import PointCloudAnalyzer
from carpointcloud_analyzer.application.ports import (
    DimensionStore, DisabledDimensionStore, EmptyProfileStorage,
    DisabledEmptyVehicleProfileStorage,
)
from carpointcloud_analyzer.application.service_contracts import (
    AnalyzeRequest as AnalyzeRequest,
    BatchAnalyzeRequest as BatchAnalyzeRequest,
    PairAnalyzeRequest as PairAnalyzeRequest,
)
from carpointcloud_analyzer.domain import stage_timing
from carpointcloud_analyzer.domain.utils.parallel import map_parallel
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.fusion import Extrinsic
from carpointcloud_analyzer.domain.models import AnalysisOutput
from carpointcloud_analyzer.domain.preview import decode_preview, encode_preview
from carpointcloud_analyzer.domain.runtime import logger
from carpointcloud_analyzer.infrastructure.cloud_io import write_merged_pcd
from carpointcloud_analyzer.infrastructure.loader import PointCloudLoader
from carpointcloud_analyzer.infrastructure.runtime_diagnostics import snapshot

MAX_SERVICE_CONCURRENCY = 4
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8010


AnalyzerFactory = Callable[[Config], PointCloudAnalyzer]


def load_trusted_extrinsic(path: str) -> Extrinsic:
    """读取双雷达外参；在线融合只接受带可信质量证明和基准残差的标定。"""
    extrinsic = Extrinsic.load(path)
    if extrinsic.quality is None or not extrinsic.quality.trustworthy:
        raise ValueError(
            "双雷达外参缺少可信质量证明；请用公共地磅/车辆重叠点完成标定，"
            "并确保 quality.trustworthy=true"
        )
    if extrinsic.baseline_residual is None:
        raise ValueError("双雷达外参缺少 baselineResidual，无法在线判断固定外参是否漂移")
    return extrinsic


class AnalysisService:
    """常驻分析编排，计算工作交给固定线程池。"""

    def __init__(
        self,
        max_concurrency: int = MAX_SERVICE_CONCURRENCY,
        analyzer_factory: AnalyzerFactory = PointCloudAnalyzer,
        dimension_store: DimensionStore | None = None,
        empty_profile_storage: EmptyProfileStorage | None = None,
        extrinsic_path: str | None = None,
    ) -> None:
        """组装执行器与存储端口；配置了场地外参时启动即加载一次。

        Raises:
            ValueError: 空车存储缺少数据库支持，或场地外参不可用。
        """
        self.max_concurrency = max_concurrency
        self._analyzer_factory = analyzer_factory
        self._dimension_store = dimension_store or DisabledDimensionStore()
        self._empty_profile_storage = (
            empty_profile_storage or DisabledEmptyVehicleProfileStorage()
        )
        if self._empty_profile_storage.enabled and not self._dimension_store.enabled:
            raise ValueError("启用永久空车云图时必须同时启用 DM8 直写")
        # 外参是固定场地标定，配置在服务端后以它为准，请求里的 extrinsicPath 不再生效。
        self._extrinsic_path = str(Path(extrinsic_path).resolve()) if extrinsic_path else None
        self._site_extrinsic = (
            load_trusted_extrinsic(self._extrinsic_path) if self._extrinsic_path else None
        )
        if self._site_extrinsic is not None:
            logger.info(
                "场地外参已加载：%s → %s，来源 %s，空磅道验收 %s",
                self._site_extrinsic.camera_b, self._site_extrinsic.camera_a, self._extrinsic_path,
                "已记录" if self._site_extrinsic.empty_lane_baseline else "未记录",
            )
        self._executor = ThreadPoolExecutor(
            max_workers=max_concurrency,
            thread_name_prefix="carpointcloud-analysis",
        )
        # 与Worker最近200条任务窗口一致；在途任务不淘汰，成功结果用于HTTP超时后的重取。
        self._pair_jobs: dict[PairAnalyzeRequest, asyncio.Task] = {}
        self._completed_pairs: deque[PairAnalyzeRequest] = deque()
        self._execution_ids = count(1)

    async def analyze(self, request: AnalyzeRequest) -> dict[str, Any]:
        """在线程池中异步执行单文件分析。"""
        return await self._run_bounded(self.analyze_sync, request)

    async def analyze_batch(self, request: BatchAnalyzeRequest) -> dict[str, Any]:
        """在线程池中异步执行一个顺序批次。"""
        return await self._run_bounded(self.analyze_batch_sync, request)

    async def record_dimension_unavailable(
        self, workflow_id: str, camera: str, captured_at: datetime,
    ) -> dict[str, str]:
        """登记 Worker 已结束的配对结果，采集时刻沿用原分析请求。"""
        if not self._dimension_store.enabled:
            return {"status": "disabled"}
        recorded = await asyncio.to_thread(
            self._dimension_store.record_dimension_unavailable,
            workflow_id, camera, captured_at,
        )
        return {"status": "recorded" if recorded else "unknown_workflow"}

    async def analyze_pair(self, request: PairAnalyzeRequest) -> dict[str, Any]:
        """在线程池中异步执行固定 A/B 双雷达分析。"""
        job = self._pair_jobs.get(request)
        if job is None:
            job = asyncio.create_task(self._run_bounded(self.analyze_pair_sync, request))
            self._pair_jobs[request] = job

            def completed(task):
                # 真正失败允许下一次重试；客户端断开不取消被shield保护的任务。
                if task.cancelled() or task.exception() is not None:
                    self._pair_jobs.pop(request, None)
                    return
                self._completed_pairs.append(request)
                if len(self._completed_pairs) > 200:
                    self._pair_jobs.pop(self._completed_pairs.popleft(), None)

            job.add_done_callback(completed)
            reuse = "new"
        else:
            reuse = "completed" if job.done() else "running"
        logger.info("双雷达请求 workflowId=%s pairId=%s reuse=%s",
                    request.workflow_id, request.pair_id, reuse)
        started = time.perf_counter()
        try:
            return await asyncio.shield(job)
        finally:
            logger.info("双雷达请求等待结束 workflowId=%s pairId=%s reuse=%s wait_ms=%.1f job_done=%s",
                        request.workflow_id, request.pair_id, reuse,
                        (time.perf_counter() - started) * 1000, job.done())

    async def preview(self, payload):
        """在线程池执行只读预览。"""
        return await self._run_bounded(self.preview_sync, payload)

    def preview_sync(self, payload):
        """只读补点预览：必须有场地标定与地磅平面，结果不入库也不建档案。

        Raises:
            ValueError: 未配置场地外参，或外参缺少地磅标定平面。
        """
        if self._site_extrinsic is None:
            raise ValueError("preview requires configured site calibration")
        plane = self._site_extrinsic.weighbridge_plane_a
        if plane is None:
            raise ValueError("preview requires calibrated weighbridge plane")
        points = decode_preview(payload)
        config = Config(
            input_unit="m", license_plate="", save_clean_pointcloud=False,
            scene_preprocessing_fail_open=False,
            scene_auto_level_enable=True, scene_auto_level_required=True,
            floor_require_weighbridge_plane=True,
            weighbridge_plane_source=tuple(float(x) for x in plane),
        )
        result = self._analyzer_factory(config).analyze_preview(points)
        return encode_preview(result)

    async def _run_bounded(self, function, request):
        """线程池管理计算并发，HTTP 不另设拒绝或恢复状态。"""
        submitted = time.perf_counter()
        execution_id = next(self._execution_ids)
        logger.info("计算入队 pid=%s executionId=%s method=%s workflowId=%s pairId=%s",
                    os.getpid(), execution_id, function.__name__,
                    getattr(request, "workflow_id", None), getattr(request, "pair_id", None))

        def execute():
            logger.info("分析开始 method=%s workflowId=%s pairId=%s queue_ms=%.1f",
                        function.__name__, getattr(request, "workflow_id", None),
                        getattr(request, "pair_id", None), (time.perf_counter() - submitted) * 1000)
            logger.info("计算执行 pid=%s executionId=%s nativeThreadId=%s",
                        os.getpid(), execution_id, threading.get_native_id())
            try:
                return function(request)
            finally:
                logger.info("计算结束 pid=%s executionId=%s workflowId=%s",
                            os.getpid(), execution_id, getattr(request, "workflow_id", None))

        return await asyncio.get_running_loop().run_in_executor(self._executor, execute)

    def _resolve_request_plate(self, request):
        """按 workflow 取车辆身份，不依赖 Worker 是否开启车牌查询。"""
        if request.workflow_id and self._dimension_store.enabled:
            plate = self._dimension_store.get_workflow_plate(request.workflow_id)
            return replace(request, plate=plate or "")
        return request

    def analyze_sync(self, request: AnalyzeRequest) -> dict[str, Any]:
        """同步完成单文件分析。"""
        request = self._resolve_request_plate(request)
        analyzer = self._analyzer_factory(self._build_config(request))
        return self._run_single(
            analyzer,
            str(Path(request.pcd_path).resolve()),
            workflow_id=request.workflow_id,
            captured_at=request.captured_at,
        )

    def analyze_batch_sync(self, request: BatchAnalyzeRequest) -> dict[str, Any]:
        """复用同一分析器顺序处理批次并汇总耗时。"""
        analyzer = self._analyzer_factory(self._build_config(request))
        started = time.perf_counter()
        results = [self._run_single(analyzer, str(Path(path).resolve())) for path in request.pcd_paths]
        elapsed_ms = round((time.perf_counter() - started) * 1000, 3)
        return {
            "success": True,
            "count": len(results),
            "elapsed_ms": elapsed_ms,
            "results": results,
        }

    def analyze_pair_sync(self, request: PairAnalyzeRequest) -> dict[str, Any]:
        """合并固定 A/B 的两份点云，并把本次分阶段耗时写入日志（不进响应）。"""
        outcome = "failed"
        with stage_timing.collect(workflow_id=request.workflow_id, pair_id=request.pair_id, snapshot=snapshot) as timer:
            try:
                result = self._analyze_pair(request)
                outcome = "ok"
                return result
            finally:
                logger.info(
                    "双雷达分析分阶段耗时 workflowId=%s pairId=%s outcome=%s total_ms=%s %s",
                    request.workflow_id, request.pair_id, outcome, timer.total_ms(), timer.describe())

    def _analyze_pair(self, request: PairAnalyzeRequest) -> dict[str, Any]:
        """合并固定 A/B 的两份点云，质量不合格时自动降级到基准雷达 A。"""
        started = time.perf_counter()
        pcd_path_a = Path(request.pcd_path_a).resolve()
        pcd_path_b = Path(request.pcd_path_b).resolve()
        if self._site_extrinsic is not None:
            extrinsic_path, extrinsic = self._extrinsic_path, self._site_extrinsic
        elif request.extrinsic_path:
            extrinsic_path = str(Path(request.extrinsic_path).resolve())
            extrinsic = load_trusted_extrinsic(extrinsic_path)
        else:
            raise ValueError("未配置 CARPOINT_EXTRINSIC_PATH，请求必须提供 extrinsicPath")
        extrinsic.validate_camera_pair(request.camera_a, request.camera_b)
        with stage_timing.stage("resolve-plate"):
            request = self._resolve_request_plate(request)

        config = self._build_config(request)
        # 双雷达外参和质量阈值都以米为单位，生产配对接口禁止再做启发式单位猜测；
        # 场景提车失败也不能把整幅环境点云继续送入尺寸算法。
        config.input_unit = "m"
        config.scene_preprocessing_fail_open = False
        loader = PointCloudLoader(config)
        with stage_timing.stage("load-pair"):
            # 两份原始云互不相关，同时读取；A 的读取错误仍先于 B 抛出。
            points_a, points_b = map_parallel(loader.load, (str(pcd_path_a), str(pcd_path_b)))
        config.scene_auto_level_enable = True
        config.scene_auto_level_required = True
        config.floor_require_weighbridge_plane = True
        calibrated_plane = extrinsic.weighbridge_plane_a
        config.weighbridge_plane_source = (
            tuple(float(value) for value in calibrated_plane)
            if calibrated_plane is not None else None
        )
        # 标定面朝向按"一次性验收、之后信任"处理，不再逐帧从背景反推。
        #
        # 逐帧判据本质不可靠：它比较标定面上下两侧的点分布，而磅台下方能看到多少
        # 背景取决于当帧视野。实测同一份正确标定面 ——
        #   0903 共同场景 P99/|P1| = 1.08（仅以 0.27m 通过，属侥幸）
        #   0907 共同场景 P99/|P1| = 0.28（被判为反向，整单失败）
        #   两组的 CAM_004 单帧分别是 0.16 / 0.18（永远判反向）
        # 换成有界带点数比后 0903/0907 为 3.97/1.04，新场景余量仍只有 4%。
        # 因此已通过一次性空磅道验收（携带 emptyLaneBaseline）的场地标定，直接
        # 视为朝向已确认；朝向真的写反时由底板高度的物理合理性判据兜底。
        if calibrated_plane is not None and extrinsic.empty_lane_baseline:
            config.weighbridge_plane_orientation_verified = True
        vehicle_merge = None
        try:
            with stage_timing.stage("fusion"):
                vehicle_merge = merge_vehicle_clouds(
                    points_a,
                    points_b,
                    extrinsic,
                    camera_a=request.camera_a,
                    camera_b=request.camera_b,
                    config=config,
                )
        except DualCloudMergeError as exc:
            # 共同场景提车失败时不得退回“原始 A+B 直接堆叠”，记录拒绝原因并降级到固定基准雷达 A。
            reason = str(exc)
            fusion_payload = {
                "mode": "rejected",
                "qualityPassed": False,
                "reason": reason,
                "warnings": [reason],
                "pointsFromA": int(len(points_a)),
                "pointsFromB": int(len(points_b)),
                "pointsCombined": int(len(points_a) + len(points_b)),
                # 车辆提取阶段失败时没有产生融合点云，降级分析的 A 点数不冒充融合点数。
                "pointsMerged": 0,
                "vehicleExtraction": None,
            }
        else:
            # 未携带 emptyLaneBaseline 的标定仍走逐帧校验；融合成功即视为共同场景
            # 已验证过朝向，降级分析 A 时不得用地磅下方背景再次否决它。
            config.weighbridge_plane_orientation_verified = calibrated_plane is not None
            fusion_payload = vehicle_merge.fusion.to_dict()
            vehicle_report = vehicle_merge.to_dict()
            fusion_payload["vehicleExtraction"] = {
                "coordinateFrame": vehicle_report["coordinateFrame"],
                "points": vehicle_report["points"],
                "temporarySiteFrame": vehicle_report["temporarySiteFrame"],
                "environmentFilter": vehicle_report["environmentFilter"],
            }
            reason = vehicle_merge.fusion.reason or fusion_payload.get("reason")
        quality_passed = vehicle_merge is not None and bool(vehicle_merge.fusion.quality_passed)
        if not quality_passed and not reason:
            reason = "；".join(str(item) for item in fusion_payload.get("warnings", []))
        if not quality_passed and not reason:
            reason = "融合质量门禁未通过"

        if quality_passed:
            analysis_path = self._pair_merged_path(
                request, pcd_path_a, pcd_path_b, Path(extrinsic_path)
            )
            with stage_timing.stage("write-merged-pcd"):
                self._write_merged_pcd(analysis_path, vehicle_merge.points)
            analysis_camera = request.camera
            mode = "fused"
            # merge_vehicle_clouds 已在共同场景中严格提取单台完整车辆。分析阶段若
            # 再跑一次 DBSCAN，会把与车厢有物理间隙的车头重新丢掉。
            config.scene_preprocessing_enable = False
            # 只有启用永久空车档案时才生成补点后 PCD。纯算法/无库
            # 测试不留额外文件。
            config.save_clean_pointcloud = self._empty_profile_storage.enabled
        else:
            # 不可信融合不能进入尺寸主链。A 是固定基准雷达 CAM_004，直接分析
            # 原始 A 文件可复用 pipeline 自带的保守场景清理与稀疏点找回；若改用
            # 已做 SOR 的中间点反而可能永久损失远端车头。
            analysis_path = pcd_path_a
            analysis_camera = request.camera_a
            mode = "fallback_a"
            logger.warning("双雷达融合质量不合格，降级使用 A 点云：%s", reason)

        # 业务表的 cameraIdentifier 契约只允许字母、数字和下划线，不能写 A+B。
        # 融合来源仍完整保留在 pairMetadata/analysisCamera；尺寸主记录归属固定基准 A。
        config.camera_identifier = request.camera_a
        analyzer = self._analyzer_factory(config)
        result = self._run_single(
            analyzer,
            str(analysis_path),
            workflow_id=request.workflow_id,
            pair_id=request.pair_id,
            captured_at=request.captured_at,
            empty_profile_source=(str(analysis_path) if quality_passed else None),
            # 局部测量需要未体素化实测点，但只能接收融合时同一掩码选中的目标车。
            # 原始 A+B 全场景可能含第二辆车，不能重新流入尺寸/拉筋链。
            measurement_points=(vehicle_merge.selected_observed_points if quality_passed else None),
        )
        fusion_payload.update(
            {
                "mode": mode,
                "qualityPassed": quality_passed,
                "reason": reason,
            }
        )
        pair_metadata = {
            "pairId": request.pair_id,
            "workflowId": request.workflow_id,
            "sourcePcdPaths": {
                "a": str(pcd_path_a),
                "b": str(pcd_path_b),
            },
            "sourceCameras": {
                "a": request.camera_a,
                "b": request.camera_b,
                "combined": request.camera,
            },
            "analysisCamera": analysis_camera,
            "extrinsicPath": str(extrinsic_path),
            "weighbridgeReference": {
                "available": calibrated_plane is not None,
                "source": (
                    "extrinsic.weighbridgePlaneA"
                    if calibrated_plane is not None else None
                ),
            },
        }
        analysis_elapsed_ms = result.get("elapsed_ms")
        total_elapsed_ms = round((time.perf_counter() - started) * 1000, 3)
        result.update(
            {
                **pair_metadata,
                "fusion": fusion_payload,
                "analysis_elapsed_ms": analysis_elapsed_ms,
                "elapsed_ms": total_elapsed_ms,
            }
        )
        return result

    async def health(self) -> dict[str, Any]:
        """异步获取后端健康与配置情况，不暴露路径或阻塞 HTTP 事件循环。"""
        # DM8 驱动是同步接口，必须移出事件循环，否则一次慢连接会冻结所有 HTTP 请求。
        persistence_health = await asyncio.to_thread(
            self._dimension_store.health
        )
        return {
            "status": "ok",
            "max_concurrency": self.max_concurrency,
            "siteExtrinsicConfigured": self._site_extrinsic is not None,
            "persistence": persistence_health,
            "emptyVehicleProfiles": {
                "enabled": self._empty_profile_storage.enabled,
                "rootConfigured": self._empty_profile_storage.root is not None,
            },
        }

    def shutdown(self) -> None:
        """等待在途分析完成并关闭工作线程池。"""
        self._executor.shutdown(wait=True, cancel_futures=True)

    def _build_config(
        self, request: AnalyzeRequest | BatchAnalyzeRequest | PairAnalyzeRequest
    ) -> Config:
        """把服务请求选项映射为一次分析使用的配置；inputUnit 是唯一进入算法配置的外部取值。"""
        if request.input_unit not in ("auto", "m", "mm"):
            raise ValueError("inputUnit 只能是 auto、m 或 mm")
        config = Config()
        config.camera_identifier = request.camera
        config.license_plate = request.plate
        config.input_unit = request.input_unit
        return config

    @staticmethod
    def _pair_merged_path(
        request: PairAnalyzeRequest,
        pcd_path_a: Path,
        pcd_path_b: Path,
        extrinsic_path: Path,
    ) -> Path:
        """用稳定输入生成确定性文件名，确保重试时幂等键不漂移。"""
        identity = "\n".join(
            (
                request.pair_id,
                request.camera_a,
                str(pcd_path_a),
                request.camera_b,
                str(pcd_path_b),
                str(extrinsic_path),
            )
        )
        digest = hashlib.sha256(identity.encode("utf-8")).hexdigest()[:24]
        return pcd_path_a.parent / f"pair_{digest}_merged.pcd"

    _write_merged_pcd = staticmethod(write_merged_pcd)

    def _run_single(
        self,
        analyzer: PointCloudAnalyzer,
        pcd_path: str,
        workflow_id: str | None = None,
        pair_id: str | None = None,
        captured_at: datetime | None = None,
        empty_profile_source: str | None = None,
        measurement_points: np.ndarray | None = None,
    ) -> dict[str, Any]:
        """执行一次分析，回填流程字段并事务性持久化集成载荷。"""
        started = time.perf_counter()
        with stage_timing.stage("algorithm"):
            analyzer_outputs: AnalysisOutput = (analyzer.analyze(pcd_path, measurement_points=measurement_points)
                                if measurement_points is not None else analyzer.analyze(pcd_path))
        # 集成载荷从算法内存结果直接取得，不再经结果 JSON 中转。
        integration_payloads = analyzer_outputs.get("integrationPayloads")
        outputs = {
            key: value
            for key, value in analyzer_outputs.items()
            if key != "integrationPayloads"
        }
        elapsed_ms = round((time.perf_counter() - started) * 1000, 3)
        result = {
            "success": True,
            "pcd_path": pcd_path,
            "outputs": outputs,
            "elapsed_ms": elapsed_ms,
        }
        if workflow_id:
            # 回显让调用方能确认自己连的分析服务确实认识 workflowId，
            # 而不是把请求里的字段静默丢掉、留下一批无法回连的记录。
            result["workflowId"] = workflow_id
        if integration_payloads is not None:
            dimension_payload = integration_payloads.get("vehicleDimensions")
            if dimension_payload is not None:
                _stamp_workflow_fields(
                    dimension_payload, workflow_id, pair_id, captured_at)
            result["integrationPayloads"] = integration_payloads
            if self._dimension_store.enabled and dimension_payload is not None:
                with stage_timing.stage("persist"):
                    result["persistence"] = self._archive_and_persist(
                        dimension_payload, outputs, workflow_id, pair_id, captured_at,
                        empty_profile_source)
        return result


    def _stage_empty_profile(
        self,
        dimension_payload: dict[str, Any],
        outputs: dict[str, Any],
        workflow_id: str | None,
        pair_id: str | None,
        captured_at: datetime | None,
        empty_profile_source: str | None,
    ):
        """车牌可辨识时把最终净化点云暂存为永久档案候选。

        Returns:
            暂存句柄；不满足归档条件时返回 ``None``。

        Raises:
            ValueError: 该归档已就绪却拿不到最终净化输出——不得退回原始融合中间文件。
        """
        archive_eligible = (self._empty_profile_storage.enabled
                            and empty_profile_source is not None)
        if not archive_eligible or not _has_identified_plate(dimension_payload):
            return None
        # 空车即使部分尺寸缺测，也应保存本次最终补点云；Loaded/Uncertain
        # 不能仅凭候选结构或可空尺寸替代空车判定。
        load = outputs.get('loadStatus') or {}
        if load.get('status') != 'Empty':
            return None
        # 归档必须来自最终净化/补点输出；不得退回原始融合中间文件。
        completed_profile_source = outputs.get("pcd")
        completed_profile_count = outputs.get("pcdPointCount")
        if not completed_profile_source or not completed_profile_count:
            raise ValueError("permanent profile requires a completed clean PCD")
        return self._empty_profile_storage.stage(
            completed_profile_source,
            str(dimension_payload.get("licensePlate") or ""),
            workflow_id or pair_id or str(dimension_payload["idempotencyKey"]),
            int(completed_profile_count),
            captured_at or datetime.now(),
        )

    def _archive_and_persist(
        self, dimension_payload: dict[str, Any], outputs: dict[str, Any],
        workflow_id: str | None, pair_id: str | None, captured_at: datetime | None,
        empty_profile_source: str | None,
    ) -> dict[str, Any]:
        """本次有已知车牌就归档，尺寸与档案链接在同一数据库事务保存。

        该事务只覆盖数据库记录；候选 PCD 不参与事务。调用失败时提交结果可能
        不确定，因此保留候选并原样抛出异常，不凭其他请求的回执批量回收。
        """
        with stage_timing.stage("archive-stage"):
            staged = self._stage_empty_profile(
                dimension_payload, outputs, workflow_id, pair_id, captured_at, empty_profile_source)
        with stage_timing.stage("database"):
            if staged is None:
                persisted = self._dimension_store.persist_vehicle_dimensions(dimension_payload)
            else:
                persisted = self._dimension_store.persist_vehicle_dimensions(
                    dimension_payload, staged.to_mapping())
        result = persisted.to_dict()
        if staged is not None:
            with stage_timing.stage("archive-finalize"):
                self._empty_profile_storage.finalize(staged.point_cloud_path, result)
        return result


def _has_identified_plate(dimension_payload: dict[str, Any]) -> bool:
    """车牌是否可辨识；空串与 UNKNOWN 都不算，不能据此建立永久档案。"""
    plate = str(dimension_payload.get("licensePlate") or "").strip().upper()
    return plate not in ("", "UNKNOWN")


def _stamp_workflow_fields(
    dimension_payload: dict[str, Any],
    workflow_id: str | None,
    pair_id: str | None,
    captured_at: "datetime | None",
) -> None:
    """把流程标识回填进上传载荷；workflowId 同时作为幂等键。"""
    if workflow_id:
        dimension_payload["workflowId"] = workflow_id
        dimension_payload["idempotencyKey"] = workflow_id
    if pair_id:
        dimension_payload["pairId"] = pair_id
    if captured_at is not None:
        dimension_payload["getTime"] = captured_at.isoformat()
