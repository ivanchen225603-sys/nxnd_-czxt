"""将分析模型映射成既有厘米制业务载荷；不执行采集、HTTP 或写库。"""
import math
from pathlib import Path
from typing import Any, Dict, List, Optional
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.floor import CASE_FLAT, FloorProfile
from carpointcloud_analyzer.domain.longitudinal_measurement import rib_projections
from carpointcloud_analyzer.domain.models import CarriageSplitResult, RibInfo, VehicleDimensions
from carpointcloud_analyzer.domain.runtime import logger


class DimensionPayload:
    """尺寸、拉筋与高低板的业务字段转换。"""

    @staticmethod
    def _build_partial_vehicle_dimension_payload(
        cfg: Config, partial: Dict[str, Any], source_file: str,
        *, split: Optional[CarriageSplitResult] = None,
        ribs: Optional[List[RibInfo]] = None,
    ) -> Dict[str, Any]:
        """把当前空车中已确认的各项计算值写入现有可空厘米字段。"""
        def cm(value: Any) -> Optional[float]:
            return round(float(value) * 100, 1) if value is not None else None

        carriage = partial['dimensions']['carriage']
        intervals = partial.get('ribIntervals')
        observed_count = partial.get('ribCount')
        if observed_count is not None:
            rib_count = int(observed_count)
        elif intervals is not None:
            rib_count = max(0, len(intervals) - 1)
        else:
            rib_count = None
        rib_distances = ([{'ribId': index, 'distToPrev': cm(value)}
                          for index, value in enumerate(intervals, 1)]
                         if intervals is not None else [])
        if intervals is None and rib_count:
            # 根数来自已执行的检测；内长端点缺测时 N+1 段距离全部未知。
            rib_distances = [{'ribId': index, 'distToPrev': None}
                             for index in range(1, rib_count + 2)]
        measurement = split.longitudinal_measurement if split is not None else None
        if (intervals is not None and measurement is not None
                and measurement.valid and ribs is not None):
            ordered_ribs = sorted(ribs, key=lambda rib: -rib.y_position)
            projections = rib_projections(ordered_ribs, measurement)
            positions = ([row['positionM'] for row in projections]
                         if projections is not None else [None] * len(ribs))
            _, rib_distances = DimensionPayload._quantize_rib_distances(
                measurement.front_l, measurement.rear_l, positions)
        # 旧式调用保持已知间距；正式路径统一量化端点，不能从缺测间距反推位置。
        plates = partial.get('highLowPlates') or []
        return {
            'idempotencyKey': Path(source_file).stem,
            'cameraIdentifier': cfg.camera_identifier,
            'licensePlate': cfg.license_plate or None,
            'getTime': None,
            'detectStatus': 'success',
            'sourceFile': source_file,
            'resultJsonPath': None,
            'resultImagePath': None,
            'dimensions': {
                'cabToTrailerFrontDist': cm(partial.get('cabToTrailerFrontDist')),
                'trailerLength': cm(carriage.get('length')),
                'trailerWidth': cm(carriage.get('width')),
                'trailerBodyHeight': cm(carriage.get('height')),
                'trailerBottomToGroundHeight': cm(partial.get('trailerBottomToGroundHeight')),
                'highLowPlateHeight': cm(partial.get('highLowPlateHeight')),
            },
            'ribs': rib_distances,
            'actualRibCount': rib_count,
            'maximumWidthCm': cm((partial.get('referenceDimensions') or {})
                                 .get('carriage', {}).get('width')),
            'wires': [],
            'highLowPlates': [
                {**plate, 'highestToGround': cm(plate['highestToGround']),
                 'lowestToGround': cm(plate['lowestToGround'])}
                for plate in plates
            ],
        }


    @staticmethod
    def _quantize_rib_distances(
        front_inner: float, rear_inner: float,
        ordered_positions: List[Optional[float]],
    ) -> tuple[float, List[Dict[str, Any]]]:
        """统一量化已核验的累计端点；未知筋位置仅清空其相邻分段。"""
        trailer_length_cm = round(abs(front_inner - rear_inner) * 100.0, 1)
        cumulative_cm = [
            round(abs(front_inner - position) * 100.0, 1) if position is not None else None
            for position in ordered_positions
        ]
        cumulative_cm.append(trailer_length_cm)

        rib_distances = []
        previous_cm = 0.0
        for index, boundary_cm in enumerate(cumulative_cm, 1):
            rib_distances.append({
                'ribId': index,
                'distToPrev': (round(boundary_cm - previous_cm, 1)
                               if boundary_cm is not None and previous_cm is not None
                               and boundary_cm >= previous_cm else None),
            })
            previous_cm = boundary_cm
        return trailer_length_cm, rib_distances


    @staticmethod
    def _build_vehicle_dimension_payload(
        cfg: Config,
        ribs: List[RibInfo],
        dimensions: VehicleDimensions,
        split: CarriageSplitResult,
        source_file: str,
        floor_profile: Optional[FloorProfile] = None,
    ) -> Dict[str, Any]:
        """
        构造 WebApi 内部写入接口请求体。

        Returns:
            dict: 符合 Worker 可靠投递和 WebApi 内部车辆尺寸接口要求的 JSON 对象。
        业务逻辑关键点：
            split_y 是前内壁，carriage_start_y 是后内壁；拉筋按前壁→后壁排序，
            N 根拉筋输出 N+1 段。所有拉筋位置有效且顺序正确时，各段由同一组
            累计位置做 0.1cm 量化，distToPrev 之和严格等于 trailerLength，不会
            累积逐段舍入误差；任一位置缺测时只清空其相邻分段，不跨缺测位置
            推算间距，此时不要求分段之和闭合。
        """
        front_inner = float(split.split_y)
        rear_inner = float(split.carriage_start_y)
        measurement = split.longitudinal_measurement
        positions = [float(rib.y_position) for rib in ribs]
        if measurement is not None:
            if not measurement.valid:
                raise ValueError('侧壁内缘缺测，不能构造正式尺寸载荷')
            front_inner, rear_inner = measurement.front_l, measurement.rear_l
            if not math.isclose(dimensions.carriage_length, measurement.length, abs_tol=1e-8):
                raise ValueError('正式内长与业务测量端点不一致')
            ordered_ribs = sorted(ribs, key=lambda rib: -rib.y_position)
            projections = rib_projections(ordered_ribs, measurement)
            positions = ([row['positionM'] for row in projections] if projections is not None
                         else [None] * len(ribs))
        low, high = sorted((front_inner, rear_inner))
        front_to_rear_sign = 1.0 if front_inner >= rear_inner else -1.0

        # 几何拟合中心理论上都在两内壁之间；对数值精修造成的亚毫米越界做夹紧，
        # 避免 abs() 把越界量悄悄加进总长度。
        ordered_positions = (positions if measurement is not None else sorted(
            (min(max(position, low), high) for position in positions),
            key=lambda position: (front_inner - position) * front_to_rear_sign,
        ))
        trailer_length_cm, rib_distances = DimensionPayload._quantize_rib_distances(
            front_inner, rear_inner, ordered_positions)

        floor_dimensions, high_low_plates = (
            DimensionPayload._build_high_low_plate_payload(floor_profile)
        )

        return {
            'idempotencyKey': Path(source_file).stem,
            'cameraIdentifier': cfg.camera_identifier,
            'licensePlate': cfg.license_plate or None,
            # 采集时间尚未由 Worker 契约传入。保持 null 让存储端使用首次入库时间，
            # 同一 PCD 重试时不会因“当前时间”变化而破坏幂等载荷指纹。
            'getTime': None,
            'detectStatus': 'success',
            'sourceFile': source_file,
            # Analyzer 生产链路直接写 DM8，不再生成结果 JSON/PNG。
            # 字段保留在库表契约中，以 null 兼容历史记录。
            'resultJsonPath': None,
            'resultImagePath': None,
            'dimensions': {
                'cabToTrailerFrontDist': round(abs(split.cab_to_front_wall) * 100, 1),
                # 与拉筋首尾段使用同一对内壁，禁止从另一份尺寸估计重复换算。
                'trailerLength': trailer_length_cm,
                'trailerWidth': round(dimensions.carriage_width * 100, 1),
                'trailerBodyHeight': round(dimensions.carriage_height * 100, 1),
                **floor_dimensions,
            },
            'ribs': rib_distances,
            'actualRibCount': len(ribs),
            'maximumWidthCm': (round(dimensions.carriage_outer_width * 100, 1)
                               if dimensions.carriage_outer_width is not None
                               and math.isfinite(dimensions.carriage_outer_width)
                               and dimensions.carriage_outer_width > 0 else None),
            # floorProfile 保留完整算法审计信息；highLowPlates 是 4.3 接口使用的
            # 业务结构。两者都只允许来自固定地磅标定面。
            'floorProfile': (
                floor_profile.to_dict() if floor_profile is not None else None
            ),
            'highLowPlates': high_low_plates,
            'wires': [],
        }


    @staticmethod
    def _build_high_low_plate_payload(
        floor_profile: Optional[FloorProfile],
    ) -> tuple[Dict[str, Optional[float]], List[Dict[str, float | int]]]:
        """把米制底板剖面映射为接口 4.3 的厘米制高低板字段。

        底板离磅台高度取车厢前内壁处的底板，不能用最低点或台阶高点P1代替。
        只有固定地磅标定面才具有“到底磅高度”的业务含义。平板车保留零高差和
        空明细；其余类型按原三段 H/L 生成明细，整组高差由底板剖面独立确定。
        """
        empty_dimensions: Dict[str, Optional[float]] = {
            'trailerBottomToGroundHeight': None,
            'highLowPlateHeight': None,
        }
        if (
            floor_profile is None
            or floor_profile.ground_plane_source != 'weighbridge_calibration'
            or not floor_profile.segments
        ):
            return empty_dimensions, []

        heights_m = [
            float(segment.height_to_ground) for segment in floor_profile.segments
        ]
        # 台阶/斜接用两侧平台高度，整体坡面用全部已观测分箱的极值。
        height_range = floor_profile.step_platform_height_range or floor_profile.measured_height_range
        if any(
            not math.isfinite(height) or height < 0.0 or height > 10.0
            for height in (*heights_m, *(height_range or ()))
        ):
            logger.warning("底板剖面包含非法到底磅高度，跳过高低板业务字段")
            return empty_dimensions, []

        flat = floor_profile.case == CASE_FLAT
        # 无剖面范围的旧调用仍可使用已提供的测量点。
        low, high = height_range or (min(heights_m), max(heights_m))
        dimensions = {
            'trailerBottomToGroundHeight': (
                round(floor_profile.front_height_to_ground * 100.0, 1)
                if floor_profile.front_height_to_ground is not None else None
            ),
            'highLowPlateHeight': 0.0 if flat else round((high - low) * 100.0, 1),
        }

        if flat:
            return dimensions, []

        plates = []
        for index, segment in enumerate(floor_profile.segments, 1):
            height_cm = round(float(segment.height_to_ground) * 100.0, 1)
            plates.append({
                'plateId': index,
                # 等效段为同一水平面，兼容内部旧列；对外板长独立使用 LengthCm。
                'highestToGround': height_cm,
                'lowestToGround': height_cm,
                'lengthCm': round(float(segment.length_to_prev) * 100.0, 1),
            })
        return dimensions, plates
