import math
from typing import Any, Dict, List, Optional

from carpointcloud_analyzer.domain.carriage import CarriageProfile
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.dimensions import BOX_INNER_OUTER_TOLERANCE_M
from carpointcloud_analyzer.domain.longitudinal_measurement import measurement_metadata
from carpointcloud_analyzer.domain.floor import FloorProfile
from carpointcloud_analyzer.application.partial_measurements import build_partial_measurements
from carpointcloud_analyzer.domain.models import (
    CarriageSplitResult,
    LoadStatusResult,
    RibInfo,
    VehicleDimensions,
)
from carpointcloud_analyzer.domain.runtime import _utc_now_iso
from carpointcloud_analyzer.domain.dimension_payload import DimensionPayload

class ResultExporter:
    """在内存中构造分析结果和 DM8 直写载荷。"""

    @staticmethod
    def build(
        ribs: List[RibInfo],
        dimensions: VehicleDimensions,
        split: CarriageSplitResult,
        load_status: LoadStatusResult,
        input_pcd_path: str,
        cfg: Config,
        profile: Optional["CarriageProfile"] = None,
        wall_repair: Optional[Dict[str, Any]] = None,
        clean_pcd_path: str = '',
        floor_profile: Optional["FloorProfile"] = None,
        wall_completion: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """构造分析结果并在确认空车时发布车辆尺寸载荷。

        Args:
            ribs: 已检测的车厢筋条。
            dimensions: 本次车辆与车厢尺寸。
            split: 车头、车厢及内表面边界结果。
            load_status: 空载、载货或不确定状态。
            input_pcd_path: 原始输入点云路径。
            cfg: 导出字段和业务标识配置。
            profile: 车厢内部剖面。
            wall_repair: 低侧壁修补说明。
            clean_pcd_path: 预处理后点云路径。
            floor_profile: 仅空车有效的底板轮廓。
            wall_completion: 侧壁、前壁和后壁的对称补点审计信息。

        Returns:
            可直接供服务层入库和返回的内存字典。

        Notes:
            明确载货不输出空车结构；其余结果保留独立测量值，缺项保持空值。
        """
        # floorProfile、wallRepair、wallCompletion、ribs 都是确认空车后才有意义的产物。
        # 即使某个历史调用方误传了 Loaded/Uncertain 的候选结果，也在导出边界
        # 不将候选货面作为拉筋或底板档案；独立尺寸仍可通过后面的逐项路径保存。
        is_empty = load_status.calculation_allowed
        # 新流水线必须同时完成固定业务高度的车头测距。method 为 None 只会来自
        # 尚未迁移的直接模型调用，保留兼容；缺车头距时仍输出其余已算字段。
        cab_measurement_valid = (
            split.cab_measurement_valid
            if split.cab_measurement_method is not None
            else True
        )
        # 经验尺寸范围只用于算法日志，不阻止计算结果保存；缺测走逐项输出。
        dimensions_available = all(math.isfinite(value) and value >= 0 for value in (
            dimensions.vehicle_length, dimensions.vehicle_width, dimensions.vehicle_height,
            dimensions.carriage_length, dimensions.carriage_width, dimensions.carriage_height))
        length_measurement = split.longitudinal_measurement
        length_metadata = (measurement_metadata(length_measurement, ribs)
                           if length_measurement is not None else None)
        length_valid = length_measurement is None or length_measurement.valid
        width_metadata = dimensions.width_measurement
        width_valid = width_metadata is None or width_metadata.get('valid') is True
        completed_inner = ((wall_completion or {}).get('completedInnerDimensions')
                           if isinstance(wall_completion, dict) else None)
        completed_container = bool(
            is_empty and split.observed_outer_box is not None
            and isinstance(completed_inner, dict)
            and width_metadata is not None
            and width_metadata.get('adoptedSource') == 'completed_cloud_inner_side_faces'
            and length_measurement is not None and length_measurement.valid
            and all(math.isfinite(value) and value > 0 for value in (
                dimensions.carriage_length, dimensions.carriage_width,
                dimensions.carriage_height))
            and dimensions_available
        )
        measurement_valid = (
            is_empty
            and split.boundaries_are_inner_faces
            and cab_measurement_valid
            and dimensions_available
            and length_valid
            and width_valid
        )
        exported_ribs = ribs if measurement_valid else []
        exported_floor_profile = floor_profile if measurement_valid else None
        exported_wall_repair = (
            wall_repair or {'repaired': False}
            if measurement_valid
            else {'repaired': False}
        )
        exported_wall_completion = (
            wall_completion or {'completed': False}
            if is_empty
            else {'completed': False, 'reason': 'measurement_not_valid'}
        )
        # 空车某一项尺寸缺测，不得把已实际生成的补壁结果重写为“未补点”。
        # Loaded/Uncertain 的候选仍不作为正式补壁云导出。
        wall_diagnostic = {}
        if isinstance(wall_completion, dict):
            for name in ('rearWall', 'frontWall'):
                item = wall_completion.get(name)
                if isinstance(item, dict):
                    model = item.get('model')
                    model = model if isinstance(model, dict) else {}
                    wall_diagnostic[name] = {
                        'reason': item.get('reason'),
                        'modelReason': model.get('reason'),
                        'faceSelection': model.get('faceSelection'),
                        'observedCellCount': model.get('observedCellCount'),
                    }
        # 项目会扫描载货车，但业务尺寸只允许由确认空车的结构面产生。Loaded /
        # Uncertain 不生成正式载荷；逐项只读展示另带缺测及来源，不能冒充已确认内尺寸。
        integration_payloads: Dict[str, Any] = {}
        if measurement_valid:
            integration_payloads["vehicleDimensions"] = (
                ResultExporter._build_vehicle_dimension_payload(
                    cfg,
                    exported_ribs,
                    dimensions,
                    split,
                    input_pcd_path,
                    exported_floor_profile,
                )
            )
        result = {
            'input_file': input_pcd_path,
            'timestampUtc': _utc_now_iso(),
            'dimensions': dimensions.to_dict() if measurement_valid else None,
            'measurementStatus': {
                'valid': measurement_valid,
                'reason': (
                    'ok'
                    if measurement_valid
                    else (
                        ('load_status_uncertain' if load_status.status == 'Uncertain' else 'vehicle_not_empty')
                        if not is_empty
                        else (
                            'longitudinal_inner_edges_unavailable'
                            if not length_valid else 'inner_wall_boundaries_unverified'
                            if not split.boundaries_are_inner_faces
                            else 'inner_width_unavailable'
                            if not width_valid
                            else (
                                'cab_front_at_measurement_height_unavailable'
                                if not cab_measurement_valid
                                else (dimensions.height_measurement or {}).get('reason')
                                or 'dimensions_out_of_physical_range'
                            )
                        )
                    )
                ),
                'boundaryMethod': split.boundary_method,
                **({'continuousBodyRange': True} if split.continuous_body_range else {}),
                **({'observedOuterBox': split.observed_outer_box.to_dict(),
                    'measurementConfidence': 'low' if not measurement_valid else 'medium',
                    'sizeLimits': {
                        'innerLengthMaximumM': round(
                            split.observed_outer_box.length + BOX_INNER_OUTER_TOLERANCE_M, 4),
                        'innerWidthMaximumM': round(
                            split.observed_outer_box.width + BOX_INNER_OUTER_TOLERANCE_M, 4),
                        'innerHeightMaximumM': round(
                            split.observed_outer_box.height + BOX_INNER_OUTER_TOLERANCE_M, 4),
                    }} if split.observed_outer_box is not None else {}),
                **({'wallCompletionDiagnostic': wall_diagnostic}
                   if not measurement_valid and wall_diagnostic else {}),
                **({'lengthDefinition': length_measurement.metadata['definition'],
                    'lengthMeasurement': length_metadata}
                   if length_measurement is not None else
                   {'lengthDefinition': 'main_end_wall_spacing_not_full_height_minimum_clearance'}
                   if split.front_wall_plane is not None else {}),
                'calculationSucceeded': measurement_valid,
                'warning': split.boundary_warning,
                **({'widthMeasurement': dimensions.width_measurement}
                   if dimensions.width_measurement is not None else {}),
                **({'heightMeasurement': dimensions.height_measurement}
                   if dimensions.height_measurement is not None else {}),
                **({'vehicleHeightMeasurement': dimensions.vehicle_height_measurement}
                   if dimensions.vehicle_height_measurement is not None else {}),
                # 车头距两端都在离地固定截面；前壁端点的来源用于人工核对
                # （下部实测面 / 整面投影 / 分割端点）。
                **({'cabMeasurement': {
                    'heightAboveGround': round(split.cab_measurement_height, 3),
                    'headY': round(split.cab_measurement_head_y, 4),
                    'frontY': round(split.cab_measurement_front_y, 4),
                    'frontWallSource': split.cab_measurement_front_source,
                }} if split.cab_measurement_valid
                    and split.cab_measurement_front_y is not None else
                   {'cabMeasurement': {
                       'heightAboveGround': split.cab_measurement_height,
                       'reason': split.cab_measurement_reason,
                   }} if split.cab_measurement_method is not None else {}),
                'weighbridgeHeightAvailable': bool(
                    (exported_floor_profile is not None or (
                        is_empty and split.observed_outer_box is not None
                        and floor_profile is not None))
                    and (exported_floor_profile or floor_profile).ground_plane_source
                    == 'weighbridge_calibration'
                ),
            },
            # 车厢定位边界：便于人工复核分割是否正确（此前只在
            # integrationPayloads 里以 CM 形式给出 cabToTrailerFrontDist）
            'carriageBoundaries': (
                {
                    'tailY': round(split.tail_y, 4),
                    'carriageStartY': round(split.carriage_start_y, 4),
                    'splitY': round(split.split_y, 4),
                    'headY': round(split.head_y, 4),
                    'cabToFrontWall': round(split.cab_to_front_wall, 4),
                    'cabMeasurement': (
                        {
                            'heightAboveGround': round(
                                split.cab_measurement_height, 3
                            ),
                            'heightHalfBand': round(
                                split.cab_measurement_half_band, 3
                            ),
                            'headY': round(split.cab_measurement_head_y, 4),
                            'pointCount': split.cab_measurement_point_count,
                            'method': split.cab_measurement_method,
                        }
                        if split.cab_measurement_valid
                        else None
                    ),
                }
                if measurement_valid else None
            ),
            'loadStatus': load_status.to_dict(),
            # 车厢内部剖面：两侧壁高、填充特征、点云质量与告警，供人工复核。
            'carriageProfile': ResultExporter._build_carriage_profile(
                profile, dimensions
            ) if measurement_valid else None,
            # 底板剖面（车辆档案）：只在空车时产出，规格见 docs/floor_profile_spec.md
            'floorProfile': (
                exported_floor_profile.to_dict()
                if exported_floor_profile else None
            ),
            'wallRepair': exported_wall_repair,
            # wallRepair 保留历史“低侧壁”契约；wallCompletion 是新增的全壁
            # 补点审计，分别记录侧壁、前壁、后壁，便于下游区分实测与合成点。
            'wallCompletion': exported_wall_completion,
            'cleanPointCloudPath': clean_pcd_path or None,
            'baselineFile': None,  # 旧响应字段保留；新分析不读写车辆历史判空基准。
            'ribs_count': len(exported_ribs),
            'ribs': [r.to_dict() for r in exported_ribs],
            'integrationPayloads': integration_payloads,
        }
        if not measurement_valid:
            floor_dimensions, plates = ResultExporter._build_high_low_plate_payload(
                floor_profile if is_empty else None)
            partial = build_partial_measurements(
                dimensions, split, load_status, profile, floor_profile, floor_dimensions,
                plates, ribs, length_metadata, result['measurementStatus']['reason'],
            )
            result['reviewMeasurements'] = partial
            result['measurementStatus']['partial'] = True
            unavailable = [name for name, field in partial['fields'].items()
                           if field['status'] == 'unavailable']
            result['measurementStatus']['degradation'] = {
                'mode': 'partial_current_capture',
                'availableFieldCount': partial['availableFieldCount'],
                'unavailableFields': unavailable,
                'message': ('可校验点不足时保留本次实测的独立尺寸；缺测项不使用外轮廓、'
                            '车架跨度或历史车辆尺寸填充。'),
            }
            # 已独立测出的结构字段直接入库；判空不确定不再使整条结果丢失。
            # 明确载货时仍不能把煤面当作空车底板和拉筋。
            # 结构未完整时不发布永久空车档案，也不把外宽/结构跨度冒充内宽/内长。
            if load_status.status != 'Loaded':
                payload = ResultExporter._build_partial_vehicle_dimension_payload(
                    cfg, partial, input_pcd_path, split=split, ribs=ribs)
                if any(value is not None for value in payload['dimensions'].values()):
                    integration_payloads['vehicleDimensions'] = payload
                    result['dimensions'] = partial['dimensions']
                    result['measurementStatus']['calculationSucceeded'] = True
                    if completed_container:
                        # 集装箱已从最终补全云取得完整长宽高，不再继承长车
                        # 下部内缘的失败状态或把它标成待完成的局部尺寸。
                        partial['reason'] = 'completed_container_cloud'
                        partial['partial'] = any(
                            value is None for value in (
                                *partial['dimensions']['vehicle'].values(),
                                *partial['dimensions']['carriage'].values(),
                                partial['cabToTrailerFrontDist'],
                                partial['trailerBottomToGroundHeight'],
                            )
                        )
                        status = result['measurementStatus']
                        status['valid'] = True
                        status['reason'] = 'completed_container_cloud'
                        status['partial'] = partial['partial']
                        status['lengthDefinition'] = length_measurement.metadata['definition']
                        status['warning'] = None
                        status.pop('measurementConfidence', None)
                        status.pop('wallCompletionDiagnostic', None)
                        if not partial['partial']:
                            status.pop('degradation', None)
                        else:
                            status['degradation']['mode'] = 'partial_completed_container'
                            status['degradation']['message'] = (
                                '车厢长宽高已从最终补全云计算；其他缺测项目保持空值。')
                        result['floorProfile'] = (
                            floor_profile.to_dict() if floor_profile is not None else None)
                        result['carriageProfile'] = (
                            ResultExporter._build_carriage_profile(profile, dimensions))
        return ResultExporter._without_point_origin_labels(result)

    @staticmethod
    def _without_point_origin_labels(value: Any) -> Any:
        """对外只给计算值和几何诊断，不标记点来自扫描还是补洞。"""
        origin_keys = {
            'source', 'modelSource', 'adoptedSource', 'inputSource',
            'observedBaseSource', 'ribPositionSource', 'frontWallSource',
            'loadEvidenceSource', 'sourceSupportPointCount',
            'syntheticUsedForLoadEvidence', 'syntheticUsedAsObservedInnerEdge',
            'completionUsed', 'inferredGeometry', 'observedMeasurement',
            'sourceHalf', 'confidence', 'measurementConfidence',
        }
        if isinstance(value, dict):
            return {key: ResultExporter._without_point_origin_labels(item)
                    for key, item in value.items() if key not in origin_keys}
        if isinstance(value, list):
            return [ResultExporter._without_point_origin_labels(item) for item in value]
        return value


    @staticmethod
    def _build_carriage_profile(
        profile: Optional[CarriageProfile], dimensions: VehicleDimensions
    ) -> Optional[Dict[str, Any]]:
        """车厢剖面导出：两侧壁高、填充特征和数据质量。"""
        if profile is None:
            return None
        interior_search_width = float(
            profile.interior_w_range[1] - profile.interior_w_range[0]
        )
        payload: Dict[str, Any] = {
            'floorZ': round(profile.floor_z, 4),
            # wallDepth 是剖面/载荷诊断的整侧 P98 高度；正式内高改用
            # dimensions.carriage_height，并在 heightMeasurement 给出上下端点。
            # 此处接收最终车厢云，缺失侧可能已有镜像补点，不可将左右数值
            # 都解释为原始实测侧壁顶。
            'wallDepth': round(profile.wall_depth, 3),
            'wallHeightLeftMeasured': (
                round(profile.wall_top_left - profile.floor_z, 3)
                if profile.wall_top_left is not None else None
            ),
            'wallHeightRightMeasured': (
                round(profile.wall_top_right - profile.floor_z, 3)
                if profile.wall_top_right is not None else None
            ),
            # 外沿量独立保留，不能在尺寸接口改用净内宽后让这个字段悄悄变义。
            # 以下内部宽度
            # 仅表示避开侧壁后的算法搜索区，为兼容旧 JSON 保留 interiorWidth，
            # 新调用方应使用语义明确的 interiorSearchWidth。
            'outerWidthMeasured': round(
                dimensions.carriage_outer_width
                if dimensions.carriage_outer_width is not None else dimensions.carriage_width, 3
            ),
            'interiorWidth': round(interior_search_width, 3),
            'interiorSearchWidth': round(interior_search_width, 3),
            'fillMetrics': {
                'medianFillRatio': round(profile.median_fill_ratio, 3),
                'highCellFraction': round(profile.high_cell_fraction, 3),
                'midBandDensity': round(profile.mid_band_density, 3),
                'gridCells': profile.n_grid_cells,
            },
        }
        payload['dataQuality'] = profile.to_quality_dict()
        return payload

    # 旧调用入口保留为同一函数的别名，不增加转换层或重复校验。
    _build_partial_vehicle_dimension_payload = staticmethod(DimensionPayload._build_partial_vehicle_dimension_payload)
    _quantize_rib_distances = staticmethod(DimensionPayload._quantize_rib_distances)
    _build_vehicle_dimension_payload = staticmethod(DimensionPayload._build_vehicle_dimension_payload)
    _build_high_low_plate_payload = staticmethod(DimensionPayload._build_high_low_plate_payload)
