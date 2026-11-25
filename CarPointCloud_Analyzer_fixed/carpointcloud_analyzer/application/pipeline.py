from carpointcloud_analyzer.infrastructure.cloud_io import write_clean_pointcloud
from dataclasses import dataclass, replace
from typing import Any, Dict, Optional, Tuple

import numpy as np
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.carriage import (BoxCompletionResult, CarriageAnalyzer, complete_empty_carriage_walls,
                       spatially_balanced_cloud)
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.dimensions import DimensionCalculator, vehicle_height_above_ground, vehicle_length_extent
from carpointcloud_analyzer.domain.box_geometry import rotate_wl
from carpointcloud_analyzer.domain.top_end_measurement import side_ends_needing_bridge
from carpointcloud_analyzer.domain.inner_length_measurement import (
    measure_final_inner_end_faces, select_rib_side_reference,
)
from carpointcloud_analyzer.domain.top_opening import measure_top_opening_front_edge
from carpointcloud_analyzer.domain.side_wall_measurement import measure_inner_width
from carpointcloud_analyzer.domain.rib_side_measurement import refine_rib_side_positions
from carpointcloud_analyzer.domain.floor_evidence import lower_wall_floor_conflict
from carpointcloud_analyzer.domain.rib_completion import complete_rib_gaps
from carpointcloud_analyzer.domain.errors import DualCloudMergeError
from carpointcloud_analyzer.domain.site_frame import (
    SiteFrame,
    estimate_site_frame,
    site_frame_from_plane,
)
from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.domain.floor import CASE_FULL_SLOPE, FloorProfileAnalyzer
from carpointcloud_analyzer.domain.geometry import CoordinateTransform, LocalCarriageExtractor, LocalRibDetector, PointCloudGeometryNormalizer
from carpointcloud_analyzer.domain.heading import cab_requires_heading_reversal
from carpointcloud_analyzer.infrastructure.loader import PointCloudLoader
from carpointcloud_analyzer.domain.models import AnalysisOutput, BoxCarriageBox, LoadStatusResult, VehicleDimensions
from carpointcloud_analyzer.domain.preprocessing import PreprocessResult, ScenePreprocessor
from carpointcloud_analyzer.domain.runtime import logger
from carpointcloud_analyzer.domain.stage_timing import lap
from carpointcloud_analyzer.domain.load_status import LoadStatusClassifier
from carpointcloud_analyzer.domain.wall_reference import (
    align_observed_rectangular_body, complete_rectangular_wall_reference, observe_rectangular_body,
)
from carpointcloud_analyzer.application.review_measurements import analyze_rectangular_final_cloud
from carpointcloud_analyzer.application.reference_outputs import build_rectangular_result
from carpointcloud_analyzer.domain.rectangular_completion import (
    completed_cloud_dimensions, completed_inner_dimensions,
    complete_rectangular_interior_gaps,
)
from carpointcloud_analyzer.domain.shell_appearance import build_shell_appearance
from carpointcloud_analyzer.domain.symmetric_opening import attach_symmetric_opening


@dataclass(frozen=True)
class _SceneContext:
    """一次分析的场地参考，供两种车型分支只读共用。"""

    preprocessed: PreprocessResult
    site_frame: Optional[SiteFrame]
    height_plane_analysis: Optional[np.ndarray]
    height_plane_source: Optional[np.ndarray]
    height_plane_calibrated: bool
    measurement_points: Optional[np.ndarray]
    vertical_reference: Optional[np.ndarray]
    vertical_source: str


@dataclass(frozen=True)
class _AnalysisMode:
    """本次输出选项，不承载算法状态或车型先验。"""

    pcd_path: str
    preview: bool
    full_reference: bool
    appearance: bool


class PointCloudAnalyzer:
    """
    点云解析主流程编排器。

    功能说明：
        串联 PCD 加载、单位识别、场地调平、场景提车、车辆局部坐标建系、
        车厢分割、尺寸计算、空载判别、筋条检测和结果导出。
    异常说明：
        文件读取失败、点云为空、配置非法、算法计算异常会向上抛出，由 CLI main() 统一记录并返回非 0 退出码。
    """

    def __init__(self, config: Optional[Config] = None):
        """校验配置并组装端到端分析所需的算法组件。

        Args:
            config: 可选分析配置；为空时使用默认配置。

        """
        self.config = config or Config()

        self.loader = PointCloudLoader(self.config)
        self.preprocessor = ScenePreprocessor(self.config)
        self.carriage_analyzer = CarriageAnalyzer(self.config)
        self.floor_analyzer = FloorProfileAnalyzer(self.config)
        self.geometry = PointCloudGeometryNormalizer(self.config)
        self.local_splitter = LocalCarriageExtractor(self.config)
        self.local_rib_detector = LocalRibDetector(self.config)
        self.dim_calc = DimensionCalculator(self.config)
        self.load_classifier = LoadStatusClassifier(self.config)
        self._last_site_frame: Optional[SiteFrame] = None

    def analyze(self, pcd_path: str, *, measurement_points=None) -> AnalysisOutput:
        """
        执行端到端点云解析。

        Args:
            pcd_path: PCD 点云文件路径。
        Returns:
            dict: 包含内存业务载荷及可选的永久空车 PCD 信息。
        """
        # 1. 加载
        points = self.loader.load(pcd_path)
        lap("load-pcd")
        return self._analyze_points(points, pcd_path, measurement_points=measurement_points)

    def analyze_preview(self, points: np.ndarray) -> Dict[str, Any]:
        """只读内存预览；禁止身份、基准和文件输出，绝不进入业务导出分支。"""
        if self.config.license_plate or self.config.save_clean_pointcloud:
            raise ValueError("preview must disable vehicle identity and file output")
        return self._analyze_points(points, "", preview=True)

    def analyze_wall_reference(self, pcd_path: str) -> Dict[str, Any]:
        """显式离线参考重建，返回全分辨率源坐标点；不写库/基准/文件。"""
        if self.config.license_plate or self.config.save_clean_pointcloud:
            raise ValueError("preview must disable vehicle identity and file output")
        return self._analyze_points(self.loader.load(pcd_path), "", preview=True, full_reference=True)

    def analyze_appearance(self, pcd_path: str, *, measurement_points=None) -> Dict[str, Any]:
        """显式离线展示导出入口，自动识别新车型；返回源坐标，不自行落盘。"""
        if self.config.license_plate or self.config.save_clean_pointcloud:
            raise ValueError("preview must disable vehicle identity and file output")
        return self._analyze_points(self.loader.load(pcd_path), "", preview=True,
                                    full_reference=True, appearance=True, measurement_points=measurement_points)

    def _analyze_points(self, points, pcd_path, preview=False, full_reference=False, appearance=False,
                        measurement_points=None):
        """准备实测场景并选择车型分支；补点不会重新参与车型识别。"""
        # 2. 先统一场地竖直轴，再做任何依赖“高度”的环境判断。这里仅做刚体
        # 坐标变换，不删点；最终合并 PCD 仍保持 CAM_004 原始坐标系。
        (
            points,
            height_plane_analysis,
            height_plane_source,
            height_plane_calibrated,
        ) = self._level_scene(points)
        lap("level-scene")
        site_frame = self._last_site_frame
        if measurement_points is not None:
            measurement_points = np.asarray(measurement_points, dtype=float)
            if (measurement_points.ndim != 2 or measurement_points.shape[1:] != (3,)
                    or not np.isfinite(measurement_points).all()):
                raise ValueError('invalid original pair measurement points')
            if site_frame is not None:
                measurement_points = site_frame.transform(measurement_points)

        # 3. 场景前处理：剔除地面/护栏/杂物，只留单台目标车辆。
        # 必须先于坐标标准化 —— 否则 PCA 会按地面主轴建系（实测在 59m 场景上
        # 得到 58.74m 的"车辆"长度），后续车厢端面与拉筋检测全部失效。
        preprocessed = self.preprocessor.process(points)
        points = preprocessed.primary_vehicle_points
        lap("preprocess")

        # 4. 去噪并建立车辆局部 W/L/H 坐标系。建系是后续所有结构测量的前提，
        # 失败时 process() 直接抛错，不再有"按输入轴向继续算"的兼容分支。
        # 高度轴必须由独立于车辆点云的竖直基准约束，否则侧壁比底板密时会被当成
        # 底板，车宽与车厢高整体互换且置信度照样是 1.00。
        vertical_reference, vertical_source = self._vertical_reference(
            height_plane_analysis, height_plane_calibrated, preprocessed
        )
        lap("vertical-reference")
        scene = _SceneContext(
            preprocessed, site_frame, height_plane_analysis, height_plane_source,
            height_plane_calibrated, measurement_points, vertical_reference, vertical_source,
        )
        mode = _AnalysisMode(pcd_path, preview, full_reference, appearance)
        # 矩形外壳必须先识别，不能先让随机底板拟合/车头启发式决定箱体范围。
        # 内部可见、端面不一致或无可信竖直基准时，不接管原车型算法。
        candidate = None
        if self.config.carriage_wall_reference_enable:
            candidate = self.geometry.vertical_candidate(points, vertical_reference, vertical_source)
        if candidate is not None:
            reference_points, _ = candidate
            reference = complete_rectangular_wall_reference(reference_points, self.config)
            lap("rectangular-reference")
            if reference.box is not None:
                return self._analyze_rectangular(
                    candidate, reference, scene, mode,
                )
        return self._analyze_standard(scene, mode)

    def _analyze_rectangular(
        self, candidate: Tuple[np.ndarray, CoordinateTransform],
        reference: BoxCompletionResult, scene: _SceneContext, mode: _AnalysisMode,
    ) -> Dict[str, Any]:
        """仅处理已识别矩形车型；预览和正式输出共享观测，不进入原车型分支。"""
        preprocessed = scene.preprocessed
        points = preprocessed.primary_vehicle_points
        measurement_points = scene.measurement_points
        reference_points, reference_transform = candidate
        pcd_path, preview = mode.pcd_path, mode.preview
        full_reference, appearance = mode.full_reference, mode.appearance
        site_frame = scene.site_frame
        height_plane_analysis = scene.height_plane_analysis
        height_plane_calibrated = scene.height_plane_calibrated
        # 识别和开口约束由实测点建立；正式尺寸在补点完成后从最终云图量取。
        if self.config.carriage_wall_reference_front_lip_enable:
            source = points if measurement_points is None else measurement_points
            reference = attach_symmetric_opening(reference_transform.to_local(source), reference)
            reference.metadata['symmetricOpening']['measurementSource'] = (
                'original_pair_before_voxel_filter' if measurement_points is not None
                else 'input_vehicle_before_geometry_denoising')
        status = LoadStatusResult(
            "Uncertain", 0.0, "interior_not_observable",
            {"interiorBelowRimPoints": float(reference.metadata["interiorBelowRimPoints"])},
        ).to_dict()
        if preview:
            # 展示分支可简化双层皮；正式分支在下方构建统一最终云。
            # 历史全分辨率参考导出仍保留实测＋补点，便于逐点审计。
            if appearance or not full_reference:
                visual = build_shell_appearance(
                    reference_points, reference,
                    spacing=.01 if full_reference else .04,
                    max_points=550000 if full_reference else 29000,
                )
                if visual is not None:
                    return self._preview_points(
                        visual.context, visual.shell, reference_transform, site_frame,
                        {**reference.metadata, 'appearance': visual.metadata}, status,
                        full_reference, visual.triangles,
                    )
            return self._preview_points(
                reference_points, reference.points, reference_transform, site_frame,
                reference.metadata, status, full_reference,
            )
        # 内高取观测跨度；长宽优先取有点带支持的对称开口推算，外形单列。
        # 矩形分支统一最终云，复用共享尺寸校验与逐项缺测输出。
        # 不能继续常规分割：这里会把前方车架误计入车厢。
        final = analyze_rectangular_final_cloud(
            reference_points, reference, reference_transform,
            (height_plane_analysis if height_plane_analysis is not None else
             getattr(preprocessed, 'ground_plane', None)),
            height_plane_calibrated, self.config,
        )
        review = final.review
        exported = ResultExporter.build(
            final.ribs, final.dimensions, final.split, final.load_status,
            pcd_path, self.config, profile=final.profile,
            floor_profile=final.floor, wall_completion=final.completion,
        )
        pcd_out_path = ''
        output_count = 0
        if (self.config.save_clean_pointcloud and final.load_status.calculation_allowed
                and exported['integrationPayloads']):
            output = self._restore_source_coordinates(final.points, reference_transform, site_frame)
            pcd_out_path = self._save_clean_pointcloud(output, pcd_path)
            output_count = len(output)
        return build_rectangular_result(
            review, reference, reference_transform, site_frame, final.load_status.to_dict(), self.config,
            pcd_path, pcd_out_path, output_count, exported_result=exported,
        )

    def _standard_coordinates(
        self, scene: _SceneContext,
    ) -> tuple[np.ndarray, CoordinateTransform, BoxCarriageBox | None]:
        """可见内部的短箱仍用独立竖直建系；未确认短箱时保持长车建系。"""
        points = scene.preprocessed.primary_vehicle_points
        if self.config.carriage_wall_reference_enable:
            candidate = self.geometry.vertical_candidate(
                points, scene.vertical_reference, scene.vertical_source)
            if candidate is not None:
                local, transform = candidate
                aligned = align_observed_rectangular_body(local, self.config)
                if aligned is not None:
                    # 外壳分支因内部可见而让位，不意味着可以换成车头顶面等
                    # 密集平面的法向。补顶、补底及后续量尺共用这一竖直系。
                    local, box, yaw = aligned
                    transform = replace(
                        transform, rotation=transform.rotation @ rotate_wl(np.eye(3), -yaw))
                    return local, transform, box
        local, transform = self.geometry.process(
            points, scene.vertical_reference, scene.vertical_source)
        return local, transform, observe_rectangular_body(local, self.config)

    def _analyze_standard(
        self, scene: _SceneContext, mode: _AnalysisMode,
    ) -> Dict[str, Any]:
        """实测结构锚定 → 壁面补点 → 判空 → 尺寸/底板/拉筋。"""
        preprocessed = scene.preprocessed
        points = preprocessed.primary_vehicle_points
        pcd_path, preview, full_reference = mode.pcd_path, mode.preview, mode.full_reference
        site_frame = scene.site_frame
        height_plane_analysis = scene.height_plane_analysis
        height_plane_source = scene.height_plane_source
        height_plane_calibrated = scene.height_plane_calibrated
        points_f, coordinate_transform, observed_box = self._standard_coordinates(scene)

        # 5. 分割
        split = (self.local_splitter.split_observed_box(points_f, observed_box)
                 if observed_box is not None else self.local_splitter.split(points_f))
        box_cab_on_low_end = bool(
            observed_box is not None
            and observed_box.rear_l - split.tail_y > split.head_y - observed_box.front_l
        )
        if box_cab_on_low_end or cab_requires_heading_reversal(points_f, split):
            # 窄端启发式可能把车尾当车头。必须先同步 W/L 与源坐标变换，
            # 再重取前内壁拟合面；底板、拉筋、补点及导出共用修正后的方向。
            axis_signs = np.array([-1., -1., 1.])
            points_f = points_f * axis_signs
            coordinate_transform = replace(
                coordinate_transform, rotation=coordinate_transform.rotation * axis_signs,
            )
            if observed_box is not None:
                # 已核验箱体只随轴向作刚体变换，不因重新分箱/拟合改变底沿。
                observed_box = replace(
                    observed_box, rear_l=-observed_box.front_l, front_l=-observed_box.rear_l,
                    w_range=(-observed_box.w_range[1], -observed_box.w_range[0]),
                    front_points=observed_box.rear_points, rear_points=observed_box.front_points)
            else:
                observed_box = observe_rectangular_body(points_f, self.config)
            split = (self.local_splitter.split_observed_box(points_f, observed_box)
                     if observed_box is not None else self.local_splitter.split(points_f))

        # 业务定义的“车头到车厢前内壁”必须在地面以上 1.40m 截面量取。
        # head_y 仍保留全高度整车端点，供整车长度使用；不能用新截面覆盖它。
        lap("coordinates-split")
        # 对磅面定义的高度/截面只认标定面；场景估计面可能是旁边更低的路面。
        vehicle_ground_plane = (coordinate_transform.plane_to_local(height_plane_analysis)
                                if height_plane_calibrated else None)
        box_front_reference_l = None

        # 6. 建立本次点云的结构锚点并补齐几何；此时不作 Empty/Loaded 决策。
        # 结构锚点只用于确定补点区域，业务判断统一在最终云形成后执行。
        profile = self.carriage_analyzer.analyze(split.carriage_points)
        lap("carriage-profile")

        wall_completion: Dict[str, Any] = {
            "completed": False, "reason": "boundaries_unverified",
        }
        completed_wall_points = np.empty((0, 3), dtype=float)
        completed_floor_points = np.empty((0, 3), dtype=float)
        box_raw_local = None
        if observed_box is not None:
            box_raw_local = coordinate_transform.to_local(points)
            completion = complete_rectangular_interior_gaps(
                points_f, observed_box, profile.interior_w_range,
                profile.floor_z, profile.wall_top, self.config,
                existing_points=box_raw_local,
            )
            completed_wall_points = completion.points
            completed_floor_points = completion.floor_points
            wall_completion = {
                **completion.metadata, 'stage': 'before_load_classification',
            }
            # 以与输出净化 PCD 相同的完整点云（实测＋补点）重算外包络，
            # 供集装箱的正式业务尺寸及补点后云图核对。
            if completion.metadata.get('completed'):
                wall_completion['completedCloudDimensions'] = (
                    completed_cloud_dimensions(
                        box_raw_local, completed_wall_points, observed_box,
                        percentile_low=self.config.dimension_percentile_low,
                        percentile_high=self.config.dimension_percentile_high,
                    )
                )
                # 补点后车厢内长/内宽/内高（扣水槽净空口径，含主体壁面
                # 与端壁顶梁分层双口径）。仅集装箱（短矩形箱车）路径：
                # 普通长车 observed_box 为 None，不进入本分支、不产出
                # 该字段，仍走原 complete_empty_carriage_walls 补点。
                inner_dimensions = completed_inner_dimensions(
                    box_raw_local, completed_wall_points, observed_box,
                    profile.floor_z, profile.wall_top, self.config,
                )
                wall_completion['completedInnerDimensions'] = inner_dimensions
                if inner_dimensions is not None:
                    box_front_reference_l = inner_dimensions['perFace']['frontWall']['narrowM']
                plane_normal = coordinate_transform.rotation[:, 0]
                plane_point = coordinate_transform.origin + (
                    plane_normal * completion.metadata['symmetryPlane']['offsetW'])
                if site_frame is not None:
                    plane_point = site_frame.axes @ plane_point + site_frame.origin
                    plane_normal = site_frame.axes @ plane_normal
                wall_completion['symmetryPlaneSourceXYZ'] = {
                    'normal': plane_normal.tolist(),
                    'offset': -float(plane_normal @ plane_point),
                }
        elif split.boundaries_are_inner_faces:
            completion = complete_empty_carriage_walls(
                points_f, split.carriage_points, split, profile, self.config,
            )
            completed_wall_points = completion.points
            wall_completion = {
                **completion.metadata,
                "stage": "before_load_classification",
            }
        if observed_box is not None and box_raw_local is not None:
            # 集装箱保存的是去环境后的完整车辆云，车厢判断也从它的箱体
            # 空间子集取得，而不是另用去噪后较稀的分割副本。
            box_mask = (
                (box_raw_local[:, 1] >= observed_box.rear_l - .05)
                & (box_raw_local[:, 1] <= observed_box.front_l + .05)
                & (box_raw_local[:, 0] >= observed_box.w_range[0] - .05)
                & (box_raw_local[:, 0] <= observed_box.w_range[1] + .05)
                & (box_raw_local[:, 2] >= observed_box.h_range[0] - .05)
                & (box_raw_local[:, 2] <= observed_box.h_range[1] + .05)
            )
            carriage_base = box_raw_local[box_mask]
        else:
            carriage_base = split.carriage_points
        completed_carriage_points = (
            np.vstack((carriage_base, completed_wall_points))
            if len(completed_wall_points) else carriage_base
        )
        lap("wall-completion")
        retained_rib_points = np.empty((0, 3), dtype=float)
        if observed_box is None and split.boundaries_are_inner_faces and not preview:
            # 全局统计去噪会删掉远端稀疏横杆。把本次融合车辆云中位于
            # 车厢内部、底板上方的未保留回波并入最终云；不重复已有点。
            raw_local = coordinate_transform.to_local(points)
            rib_band = (
                (raw_local[:, 1] > split.carriage_start_y + .12)
                & (raw_local[:, 1] < split.split_y - .12)
                & (raw_local[:, 0] > profile.interior_w_range[0])
                & (raw_local[:, 0] < profile.interior_w_range[1])
                & (raw_local[:, 2] > profile.floor_z + self.config.rib_floor_clearance)
                & (raw_local[:, 2] < profile.wall_top + .05)
            )
            candidates = raw_local[rib_band]
            if len(candidates):
                # 只与过滤后保留的扫描点去重；补壁点可能恰好落在稀疏
                # 横杆附近，不能因此吞掉原本真实存在的杆件回波。
                distances, _ = cKDTree(carriage_base).query(candidates, k=1, workers=-1)
                retained_rib_points = candidates[distances > 1e-6]
                if len(retained_rib_points):
                    completed_carriage_points = np.vstack(
                        (completed_carriage_points, retained_rib_points))

        dimension_profile = (
            self.carriage_analyzer.analyze(completed_carriage_points)
            if len(completed_wall_points) and observed_box is None and not preview
            else profile
        )

        # 已有可用顶部测线的长车保持原补点结果；端点盲段使顶部不连续时，
        # 先用双端实测壁面与同高连续侧壁支持跨过扫描缺口。业务内长仍只在
        # 最终选定的一份补点云上测量一次，不用旧云的失败值作尺寸来源。
        if (not preview and observed_box is None
                and split.boundaries_are_inner_faces and hasattr(profile, 'floor_z')
                and all(wall_completion.get(name, {}).get('model', {}).get('valid') is True
                        for name in ('rearWall', 'frontWall'))
                and (gap_ends := side_ends_needing_bridge(
                    completed_carriage_points, split.carriage_start_y, split.split_y,
                    dimension_profile.floor_z, dimension_profile.wall_top,
                ))):
            extended = complete_empty_carriage_walls(
                points_f, split.carriage_points, split, profile, self.config,
                bridge_long_side_gaps=gap_ends,
            )
            if any(seam.get('longGapHeightRows', 0) > 0
                   for seam in extended.metadata.get('cornerSeams', [])):
                completed_wall_points = extended.points
                wall_completion = {
                    **extended.metadata,
                    "stage": "before_load_classification",
                }
                completed_carriage_points = np.vstack((split.carriage_points,
                                                       completed_wall_points,
                                                       retained_rib_points))
                dimension_profile = self.carriage_analyzer.analyze(completed_carriage_points)

        # 横杆先在已补四壁的云中定位，再沿杆轴填补遮挡段。正式检测和
        # 后续全部判断均重跑于同一份含横杆的最终车厢云。
        lap("wall-bridge")
        final_carriage_points = completed_carriage_points
        completed_rib_points = np.empty((0, 3), dtype=float)
        rib_completion = {'completed': False, 'pointCount': 0, 'ribs': []}
        ribs = []
        if not preview and observed_box is None and split.boundaries_are_inner_faces:
            seed_ribs = self.local_rib_detector.detect(
                final_carriage_points, split.carriage_start_y, split.split_y,
                dimension_profile.interior_w_range,
            )
            completed_rib_points, rib_completion = complete_rib_gaps(
                final_carriage_points, seed_ribs, dimension_profile.interior_w_range,
                split.carriage_start_y, split.split_y,
                dimension_profile.floor_z, dimension_profile.wall_top,
            )
            if len(completed_rib_points):
                final_carriage_points = np.vstack((final_carriage_points, completed_rib_points))
            ribs = self.local_rib_detector.detect(
                final_carriage_points, split.carriage_start_y, split.split_y,
                dimension_profile.interior_w_range,
            )
            ribs.sort(key=lambda rib: abs(split.split_y - rib.y_position))
            for index, rib in enumerate(ribs, 1):
                rib.rib_id = index
        wall_completion['ribCompletion'] = rib_completion
        lap("ribs")
        final_profile = (self.carriage_analyzer.analyze(final_carriage_points)
                         if not preview else dimension_profile)
        lap("final-profile")
        final_vehicle_points = (
            np.vstack((box_raw_local, completed_wall_points, completed_rib_points))
            if observed_box is not None and box_raw_local is not None
            else np.vstack((points_f, retained_rib_points,
                            completed_wall_points, completed_rib_points))
        )
        # 普通长车的半挂前端取顶部开口前沿，车头距与内长共用同一前端；
        # 量不到时两者仍按前端壁内表面。
        opening_front_l, opening_metadata = None, None
        if (not preview and observed_box is None and split.boundaries_are_inner_faces
                and hasattr(final_profile, 'floor_z')):
            opening_front_l, opening_metadata = measure_top_opening_front_edge(
                split.carriage_points, final_carriage_points,
                split.carriage_start_y, split.split_y,
                final_profile.floor_z, final_profile.wall_top,
            )
        front_reference = (
            {'front_reference_l': box_front_reference_l} if box_front_reference_l is not None
            else {'front_reference_l': opening_front_l,
                  'front_reference_source': 'top_opening_front_edge'}
            if opening_front_l is not None else {})
        if split.boundaries_are_inner_faces or box_front_reference_l is not None:
            self.dim_calc.measure_cab_to_front_wall_at_height(
                final_vehicle_points, split, vehicle_ground_plane, **front_reference,
            )
        else:
            split.cab_measurement_height = self.config.cab_distance_measurement_height
            split.cab_measurement_method = "front_inner_wall_unverified"
            split.cab_measurement_reason = "front_inner_wall_unverified"

        # 7. 判空与测量都取最终云；空间格等权，补点密度不能稀释货面。
        # 补出的底板只填未观测格，不能替这些格投“空”票；判空排除它们。
        lap("cab-distance")
        load_cloud = final_carriage_points
        if len(completed_floor_points):
            distance, _ = cKDTree(completed_floor_points).query(final_carriage_points, k=1, workers=-1)
            load_cloud = final_carriage_points[distance > 0]
        load_status = self._assess_load(
            split, final_profile, final_carriage_points=load_cloud, ribs=ribs,
        )
        lap("load-status")

        # 侧壁主体前后端决定长度，不以顶部篷布或内折水槽决定端点。
        # 长、宽、高统一消费最终车厢云，内部底板和判空计算不变。
        # 端壁内表面不可见但端壁间车体连续时，侧壁长度同样可量。
        if (not preview and hasattr(profile, 'floor_z')
                and (observed_box is not None or split.boundaries_are_inner_faces
                     or split.continuous_body_range)):
            side_points = final_carriage_points
            if observed_box is not None:
                # 矩形补点已定位主体范围，外部回波不属于其侧壁。
                side_points = side_points[(side_points[:, 1] >= observed_box.rear_l)
                                          & (side_points[:, 1] <= observed_box.front_l)]
            split.longitudinal_measurement = measure_final_inner_end_faces(
                side_points,
                split.carriage_start_y, split.split_y, final_profile.floor_z,
                final_profile.wall_top, vehicle_ground_plane,
                # 业务内长：后端壁内表面到顶部开口前沿；只有已实测内表面才作端点。
                inner_face_endpoints=split.boundaries_are_inner_faces,
                front_endpoint=opening_front_l,
                front_endpoint_metadata=opening_metadata,
            )
        lap("end-faces")

        # 8. 两种车型都以本次最终云图量取可补的结构尺寸。
        completed_inner = wall_completion.get('completedInnerDimensions')
        completed_cloud = wall_completion.get('completedCloudDimensions')
        completed_vehicle = ((completed_cloud or {}).get('completedCloud') or {}).get('vehicle')
        completed_outer = ((completed_cloud or {}).get('completedCloud') or {}).get('carriageOuter')
        if (observed_box is not None and load_status.calculation_allowed
                and completed_inner is not None and completed_vehicle is not None
                and completed_outer is not None):
            vehicle_height, vehicle_height_metadata = vehicle_height_above_ground(
                final_vehicle_points, vehicle_ground_plane, self.config.dimension_percentile_high, self.config)
            width = measure_inner_width(
                final_carriage_points, split.carriage_start_y, split.split_y,
                final_profile.floor_z, final_profile.wall_top,
                allow_sparse_rectangular=True, config=self.config)
            length = split.longitudinal_measurement
            dimensions = VehicleDimensions(
                vehicle_length=(completed_vehicle['lengthM'] if vehicle_ground_plane is None
                                else vehicle_length_extent(final_vehicle_points, vehicle_ground_plane)),
                vehicle_width=max(completed_vehicle['widthM'], completed_outer['widthM']),
                vehicle_height=(completed_vehicle['heightM'] if vehicle_height is None
                                else vehicle_height),
                vehicle_height_measurement=vehicle_height_metadata,
                carriage_length=length.length if length is not None and length.valid else float('nan'),
                carriage_width=width.width_m if width.width_m is not None else float('nan'),
                carriage_height=(completed_inner['heightM'] if completed_inner['heightM'] is not None
                                 else float('nan')),
                height_measurement=completed_inner['heightMeasurement'],
                carriage_outer_width=completed_outer['widthM'],
                width_measurement={
                    **width.metadata, 'completionUsed': True,
                    'adoptedSource': 'completed_cloud_inner_side_faces',
                    'outerWidthM': completed_outer['widthM'],
                },
            )
        else:
            dimensions = self.dim_calc.calculate(
                final_vehicle_points,
                final_carriage_points,
                split, final_profile,
                ground_plane_local=vehicle_ground_plane,
                synthetic_point_count=len(completed_wall_points) + len(completed_rib_points),
            )
        if observed_box is not None:
            self.dim_calc.validate_observed_box_inner_dimensions(dimensions, split)
        lap("dimensions")

        floor_profile = self._measure_current_floor(
            split, final_profile, load_status, preprocessed, coordinate_transform,
            height_plane_analysis, height_plane_source, height_plane_calibrated,
            completed_floor_points,
            final_carriage_points=final_carriage_points,
        ) if not preview else None
        lap("floor")
        wall_repair: Dict[str, Any] = {"repaired": False}
        synthetic_count = 0
        if load_status.calculation_allowed and (
            split.boundaries_are_inner_faces or observed_box is not None
        ):
            wall_repair = wall_completion.get(
                "sideWall", wall_completion.get('sideWalls', {'repaired': False})
            )
            if (len(retained_rib_points) or len(completed_wall_points)
                    or len(completed_rib_points)):
                points_f = np.vstack([points_f, retained_rib_points,
                                      completed_wall_points, completed_rib_points])
                synthetic_count = int(len(completed_wall_points) + len(completed_rib_points))
        else:
            # 候选几何不能变成载货/不确定车辆的永久空车档案。
            wall_completion = {**wall_completion, "completed": False,
                               "reason": ("boundaries_unverified"
                                          if load_status.calculation_allowed
                                          else "vehicle_not_empty_or_uncertain"),
                               "candidateSynthesizedPoints": len(completed_wall_points),
                               "synthesizedPoints": 0}

        if preview:
            measured_count = len(points_f) - synthetic_count
            return self._preview_points(
                points_f[:measured_count], points_f[measured_count:],
                coordinate_transform, site_frame, wall_completion,
                load_status.to_dict(), full_reference,
            )

        # 9. 杆形仍取最终车厢云；拉筋允许选用独立的实测侧壁，沿用业务内长端点和投影轴。
        if load_status.calculation_allowed and split.boundaries_are_inner_faces:
            measurement = split.longitudinal_measurement
            if measurement is not None and measurement.valid:
                reference = (select_rib_side_reference(
                    measurement, split.carriage_points, split.carriage_start_y, split.split_y,
                    final_profile.floor_z, final_profile.wall_top, wall_completion,
                ) if ribs else measurement)
                metadata = measurement.metadata
                if reference is not measurement:
                    metadata = {**metadata, 'ribSideReference': reference.metadata}
                split.longitudinal_measurement = replace(
                    measurement, metadata=metadata, rib_side_projections=refine_rib_side_positions(
                        final_carriage_points, ribs, reference,
                        final_profile.floor_z, final_profile.wall_top,
                        final_cloud=True))
        else:
            # 拉筋、底板剖面和侧壁修补都依赖车厢内部真实可见。Loaded 或
            # Uncertain 时宁可输出空列表，也不能把货面起伏当成结构件。
            ribs = []
            logger.info(
                "载荷状态=%s，端壁内表面有效=%s，跳过拉筋业务输出",
                load_status.status,
                split.boundaries_are_inner_faces,
            )

        # 10. 只构造内存业务载荷，不再生成 PNG 或结果 JSON。
        # 纯净车辆点云仅用于永久空车档案，它不是诊断图片。
        lap("rib-side")
        pcd_out_path = ''
        pcd_point_count = 0
        if self.config.save_clean_pointcloud:
            output_local = (
                np.vstack((box_raw_local, completed_wall_points, completed_rib_points))
                if observed_box is not None and box_raw_local is not None
                and load_status.calculation_allowed
                else points_f
            )
            output_points = self._restore_source_coordinates(
                output_local,
                coordinate_transform,
                site_frame,
            )
            pcd_out_path = self._save_clean_pointcloud(output_points, pcd_path)
            pcd_point_count = len(output_points)
        lap("clean-pcd")

        analysis_result = ResultExporter.build(
            ribs, dimensions, split, load_status, pcd_path, self.config,
            profile=final_profile, wall_repair=wall_repair, clean_pcd_path=pcd_out_path,
            floor_profile=floor_profile, wall_completion=wall_completion,
        )
        lap("export")

        return {
            'png': '',
            'json': '',
            'pcd': pcd_out_path,
            'pcdPointCount': pcd_point_count if pcd_out_path else 0,
            'syntheticPointCount': synthetic_count,
            'integrationPayloads': analysis_result['integrationPayloads'],
            # 必须使用导出边界校验后的结果，不能返回候选 dimensions。
            # Loaded/Uncertain 或测量不合格时这里必须保持 null。
            'dimensions': analysis_result['dimensions'],
            'measurementStatus': analysis_result['measurementStatus'],
            'loadStatus': analysis_result['loadStatus'],
            'wallCompletion': analysis_result['wallCompletion'],
            **({'reviewMeasurements': analysis_result['reviewMeasurements']}
               if 'reviewMeasurements' in analysis_result else {}),
        }

    def _preview_points(self, measured, synthetic, transform, site_frame,
                        completion, load_status, full_reference, triangles=None):
        """实测与合成点分别抽样/计数；显式离线导出可保留全分辨率。"""
        if not full_reference:
            measured = measured[::max(1, int(np.ceil(len(measured) / 30000)))]
            synthetic = synthetic[::max(1, int(np.ceil(len(synthetic) / 30000)))]
        output = self._restore_source_coordinates(
            np.vstack([measured, synthetic]), transform, site_frame
        )
        result = {
            "coordinates": output, "measuredPointCount": len(measured),
            "syntheticPointCount": len(synthetic), "wallCompletion": completion,
            "loadStatus": load_status, "referenceOnly": True,
        }
        if triangles is not None:
            surface = self._restore_source_coordinates(triangles, transform, site_frame)
            if not np.isfinite(surface).all() or not 0 < len(surface) <= 60 or len(surface) % 3:
                raise ValueError('invalid display surface')
            result['displayMesh'] = {
                'encoding': 'xyz-triangles-v1', 'displayOnly': True,
                'coordinates': surface.ravel().tolist(),
            }
        return result

    @staticmethod
    def _vertical_reference(
        height_plane_analysis: Optional[np.ndarray],
        height_plane_calibrated: bool,
        preprocessed,
    ) -> Tuple[Optional[np.ndarray], str]:
        """挑选建立高度轴时使用的竖直基准，按可信度从高到低。

        1. 地磅标定面 —— 固定场地标定，最可信；
        2. 场地支撑面 —— 自动调平估计出的大面积支撑平面；
        3. 场景地面基准 —— 前处理估计的地平面，法向已统一朝上。

        三者都取不到时返回 ``(None, "none")``，由 ``geometry.process`` 按
        ``geometry_require_vertical_reference`` 决定拒绝还是降级。
        """
        if height_plane_analysis is not None:
            source = "weighbridge" if height_plane_calibrated else "site_frame"
            return np.asarray(height_plane_analysis, dtype=float)[:3], source
        ground_plane = getattr(preprocessed, 'ground_plane', None)
        if ground_plane is not None:
            return np.asarray(ground_plane, dtype=float)[:3], "scene_ground"
        return None, "none"

    def _level_scene(
        self, points: np.ndarray
    ) -> Tuple[
        np.ndarray,
        Optional[np.ndarray],
        Optional[np.ndarray],
        bool,
    ]:
        """把任意源 XYZ 方向临时变换为 Z-up，且不改变点云物理尺度。

        Returns:
            ``(场地点, 场地坐标高度基准面, 源坐标高度基准面, 是否地磅标定)``。
            地磅面存在时，它既定义竖直轴又定义业务高度零点；否则自动支撑面只
            负责竖直轴。生产双雷达链路会要求自动建系成功，但不会把自动支撑面
            冒充为已标定地磅面。
        """
        cfg = self.config
        self._last_site_frame = None
        if not cfg.scene_auto_level_enable:
            return points, None, None, False

        try:
            if cfg.weighbridge_plane_source is not None:
                source_plane = np.asarray(cfg.weighbridge_plane_source, dtype=float)
                site_frame = site_frame_from_plane(
                    points,
                    source_plane,
                    validate_orientation=(
                        not cfg.weighbridge_plane_orientation_verified
                    ),
                )
                calibrated = True
                logger.info("使用固定地磅标定面统一场地 Z 轴和高度零点")
            else:
                site_frame = estimate_site_frame(points)
                source_plane = site_frame.plane
                calibrated = False
                logger.warning(
                    "未配置地磅标定面：场景支撑面仅用于统一 Z 轴，"
                    "不得解释为到底磅高度"
                )
        except (DualCloudMergeError, ValueError) as exc:
            if cfg.scene_auto_level_required:
                raise ValueError(f"无法可靠确定场地竖直轴：{exc}") from exc
            logger.warning("场地自动调平失败，按输入坐标继续兼容分析：%s", exc)
            return points, None, None, False

        # SiteFrame 的原点位于参考平面，第三轴就是单位法向，因此源平面在临时
        # 场地坐标里严格等价于 z=0。避免再次 RANSAC 引入毫米级随机抖动。
        plane_in_analysis = np.array([0.0, 0.0, 1.0, 0.0], dtype=float)
        self._last_site_frame = site_frame
        return (
            site_frame.transform(points),
            plane_in_analysis,
            np.asarray(source_plane, dtype=float),
            calibrated,
        )

    def _assess_load(
        self,
        split,
        profile,
        *, final_carriage_points: Optional[np.ndarray] = None,
        ribs=None,
    ) -> LoadStatusResult:
        """从最终云的等权空间支持判空；不按原始/合成点建立两套业务判断。"""
        cfg = self.config
        cloud = split.carriage_points if final_carriage_points is None else final_carriage_points
        # 端壁和拉筋表面都高于底板，若参与填充率投票会被误认为货物。
        # 不读取历史底板或车牌基准，杆带剔除仍沿本次几何位置。
        usable = ((cloud[:, 1] > split.carriage_start_y + .12)
                  & (cloud[:, 1] < split.split_y - .12))
        for rib in ribs or []:
            usable &= np.abs(cloud[:, 1] - rib.y_position) > .065
        cloud = cloud[usable]
        cloud = spatially_balanced_cloud(cloud, cfg)
        load_profile = self.carriage_analyzer.analyze(cloud) if len(cloud) else profile
        grid_points = self._interior_grid_points(
            cloud,
            load_profile.interior_w_range,
        )
        if len(grid_points) < cfg.carriage_interior_min_points:
            # 观测不足仍不能判空，但这个门禁不再依赖是否存在历史文件。
            status = LoadStatusResult(
                "Uncertain",
                0.20,
                "carriage_interior_grid_insufficient",
                {
                    "interiorPoints": float(len(grid_points)),
                    "baselineAvailable": 0.0,
                },
            )
        else:
            status = self.load_classifier.classify(
                cloud,
                load_profile,
            )
            if status.status == 'Empty':
                floor_evidence = lower_wall_floor_conflict(cloud, load_profile, cfg)
                if floor_evidence['conflict']:
                    status = LoadStatusResult(
                        'Uncertain', .2, 'observed_floor_contradicts_inner_walls',
                        {**status.metrics, 'floorEvidence': floor_evidence})
        if status.calculation_allowed and not split.boundaries_are_inner_faces:
            # 判空不等于尺寸可测：端壁物理面不可靠时仍保留测量门禁。
            logger.warning(
                "车辆判为空车，但端壁内表面未通过质量门禁：%s",
                split.boundary_warning or split.boundary_method,
            )
        logger.info(
            "载荷状态：%s, confidence=%.2f, reason=%s",
            status.status, status.confidence, status.reason,
        )
        return status

    def _measure_current_floor(
        self, split, profile, status, preprocessed, transform,
        height_plane_analysis, height_plane_source, height_plane_calibrated,
        completed_floor_points=None, *, final_carriage_points=None,
    ):
        """判空与补后测长完成后，从最终云裁切底板剖面。"""
        if not (status.calculation_allowed
                and (split.boundaries_are_inner_faces or split.observed_outer_box is not None)
                and self.config.floor_profile_enable):
            return None
        cloud = (split.carriage_points if final_carriage_points is None
                 else final_carriage_points)
        # 内长缺测只影响纵向裁切，不应把底板数据源退回补点前的副本。
        floor_split = replace(split, carriage_points=cloud)
        measurement = split.longitudinal_measurement
        if split.observed_outer_box is not None:
            real_floor = cloud[np.abs(cloud[:, 2] - profile.floor_z) <= .10]
            # 正式最终云已含底板补点，不能再追加一次而改变剖面采样权重。
            # 未提供最终云的离线旧入口仍允许显式补点参数。
            modeled = (completed_floor_points if final_carriage_points is None
                       and completed_floor_points is not None
                       else np.empty((0, 3), dtype=float))
            floor_split = replace(
                split, carriage_points=np.vstack((real_floor, modeled)))
        if (measurement is not None and measurement.valid
                and (split.observed_outer_box is None or measurement.longitudinal_axis is not None)):
            floor_cloud = floor_split.carriage_points
            longitudinal = measurement.project_longitudinal(floor_cloud)
            selected = floor_cloud[(longitudinal >= measurement.rear_l)
                                   & (longitudinal <= measurement.front_l)]
            floor_split = replace(floor_split, carriage_points=selected,
                                  split_y=measurement.front_l,
                                  carriage_start_y=measurement.rear_l)
        floor_result = self._analyze_floor_profile(
            floor_split, profile, preprocessed, transform,
            height_plane_analysis, height_plane_source, height_plane_calibrated)
        if floor_result is not None and split.observed_outer_box is not None:
            height_range = floor_result.measured_height_range
            # measured_height_range 专用于整体坡面；平底/台阶由各段的
            # 实际高度给出范围，不能把它们的 None 误作箱体高度异常。
            if height_range is None and floor_result.case != CASE_FULL_SLOPE:
                heights = np.asarray([
                    segment.height_to_ground for segment in floor_result.segments
                ], dtype=float)
                if len(heights) and np.isfinite(heights).all():
                    height_range = (float(heights.min()), float(heights.max()))
            if (height_range is None or height_range[1] - height_range[0]
                    > split.observed_outer_box.height * 0.30):
                logger.warning('矩形箱体底板剖面超出箱体高度约束，不发布底板到地面高度')
                return None
            floor_result.confidence = 'low'
            floor_result.warnings.append('底板由本次可见半幅沿车长前后镜像补点')
        return floor_result

    @staticmethod
    def _interior_grid_points(
        carriage_points: np.ndarray,
        interior_w_range: Tuple[float, float],
    ) -> np.ndarray:
        """只保留左右内壁之间的实测点，检查本次内部观测是否充足。"""
        w_lo, w_hi = interior_w_range
        if not np.isfinite(w_lo) or not np.isfinite(w_hi) or w_hi <= w_lo:
            return np.empty((0, 3), dtype=float)
        width = carriage_points[:, 0]
        return carriage_points[(width > w_lo) & (width < w_hi)]

    def _analyze_floor_profile(
        self,
        split,
        profile,
        preprocessed,
        transform,
        height_plane_analysis: Optional[np.ndarray],
        height_plane_source: Optional[np.ndarray],
        height_plane_calibrated: bool,
    ):
        """空车底板剖面；生产高度只允许引用固定地磅标定面。"""
        if (
            self.config.floor_require_weighbridge_plane
            and not height_plane_calibrated
        ):
            logger.warning(
                "缺少固定地磅标定面，跳过底板到底磅高度；"
                "不会用普通地面或单帧拟合结果代替"
            )
            return None

        analysis_plane = height_plane_analysis
        source_plane = height_plane_source
        if analysis_plane is None:
            analysis_plane = getattr(preprocessed, 'ground_plane', None)
            source_plane = analysis_plane
        local_plane = analysis_plane
        if analysis_plane is not None:
            local_plane = transform.plane_to_local(analysis_plane)
        return self.floor_analyzer.analyze(
            split.carriage_points,
            profile.interior_w_range,
            front_l=split.split_y,
            rear_l=split.carriage_start_y,
            ground_plane_local=local_plane,
            confidence=profile.measurement_confidence,
            ground_plane_world=source_plane,
            ground_plane_source=(
                "weighbridge_calibration"
                if height_plane_calibrated else "scene_ransac"
            ),
            longitudinal_axis=(split.longitudinal_measurement.longitudinal_axis
                               if split.longitudinal_measurement is not None else None),
        )

    @staticmethod
    def _restore_source_coordinates(
        points: np.ndarray,
        transform,
        site_frame: Optional[SiteFrame],
    ) -> np.ndarray:
        """把局部 W/L/H 补全点恢复到输入 PCD（双雷达时为 CAM_004）坐标。"""
        restored = transform.to_source(points)
        if site_frame is not None:
            restored = restored @ site_frame.axes.T + site_frame.origin
        return restored

    _save_clean_pointcloud = staticmethod(write_clean_pointcloud)
