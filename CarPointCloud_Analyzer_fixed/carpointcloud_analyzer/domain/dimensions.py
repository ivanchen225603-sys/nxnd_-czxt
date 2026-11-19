from dataclasses import replace
from typing import Optional

import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix

from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer, CarriageProfile
from carpointcloud_analyzer.domain.utils import occupancy_extent
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.models import CarriageSplitResult, VehicleDimensions
from carpointcloud_analyzer.domain.inner_faces import FACE_CELL_M, FACE_TOLERANCE_M, FACE_MIN_POINTS, FACE_SEARCH_HALF_BAND_M, ObservedFace, supported_faces
from carpointcloud_analyzer.domain.side_wall_measurement import measure_inner_width
from carpointcloud_analyzer.domain.side_wall_body import side_body_mask
from carpointcloud_analyzer.domain.floor import FloorProfileAnalyzer
from carpointcloud_analyzer.domain.runtime import logger
from carpointcloud_analyzer.domain.utils.histogram import robust_span as _robust_span
from carpointcloud_analyzer.domain.utils.cells import unique_cells
from carpointcloud_analyzer.domain.utils.parallel import stacked_svd

# 测距截面处的前壁位置只取离截面最近的这一段下部墙面（从截面或墙面最低观测点
# 起算）。整面拟合会被前壁顶部的篷布/挡板带偏：09-24 车 B 前壁顶部有篷布，整面
# 拟合成倾斜 4.6° 的面，外推到 1.40m 比下部实测墙面靠厢内约 9cm，车头距在
# 440/449.5cm 之间跳；该车实测前壁下部竖直。0.8m 保证面内至少 4 行 0.2m 格。
CAB_FRONT_WALL_LOW_WINDOW_M = 0.8
BOX_INNER_OUTER_TOLERANCE_M = 0.05


def _measure_continuous_side_height(
    points: np.ndarray, profile: CarriageProfile, config: Config,
) -> tuple[float, dict]:
    """用沿车长连续出现的侧壁上沿量内高，避开端壁局部高点。

    每个纵向分箱取侧壁本体上沿点 P98，量到车厢内部底板后取分箱中位数。
    独立杆件不属于侧壁；细顶沿必须与二维侧壁相接。各分箱记入 binTopsLH。
    """
    floor = float(profile.floor_z)
    fallback_top = float(profile.wall_top)
    length_lo, length_hi = np.percentile(points[:, 1], [2., 98.])
    central_lo = length_lo + (length_hi - length_lo) * .1
    central_hi = length_hi - (length_hi - length_lo) * .1
    edges = np.linspace(central_lo, central_hi, 13)
    left_w, right_w = profile.interior_w_range
    interior = points[(points[:, 0] > left_w) & (points[:, 0] < right_w)]
    floor_l, floor_h = np.empty(0), np.empty(0)
    if len(interior) >= config.carriage_interior_min_points:
        start, end = float(points[:, 1].min()), float(points[:, 1].max())
        floor_l, floor_h = FloorProfileAnalyzer(config).extract_profile(
            interior, interior[:, 1] - start, end - start, np.array([0., 0., 1., 0.]))
        floor_l += start
    candidates = {}
    valid_heights = []
    valid_tops = []
    support = {}
    bin_report = {}
    height_report = {}
    for name, side_mask in (
        ('left', points[:, 0] <= left_w),
        ('right', points[:, 0] >= right_w),
    ):
        side_mask = side_body_mask(points, side_mask, edges, config.statistical_outlier_neighbors)
        bin_tops = []
        bin_heights = []
        for index in range(len(edges) - 1):
            mask = (side_mask & (points[:, 1] >= edges[index])
                    & (points[:, 1] < edges[index + 1]))
            if np.count_nonzero(mask) >= 30:
                top = float(np.percentile(points[mask, 2], 98.))
                local_floor = floor_h[(floor_l >= edges[index]) & (floor_l < edges[index + 1])]
                bottom = float(np.median(local_floor)) if len(local_floor) else floor
                # 侧壁缺失时，密集底板边缘也会填满纵向分箱，但不构成壁顶。
                # 只排除既有底板层容差内的分箱，不能用尺寸合法下限筛点：
                # 真实低矮侧壁仍需输出实算高度，由尺寸门禁判断其是否越界。
                if top - bottom > config.floor_layer_tolerance:
                    bin_tops.append(top)
                    bin_heights.append(top - bottom)
                    bin_report.setdefault(name, []).append(
                        [round(float((edges[index] + edges[index + 1]) / 2), 3), round(top, 4)])
                    height_report.setdefault(name, []).append({
                        'l': round(float((edges[index] + edges[index + 1]) / 2), 3),
                        'topH': top, 'floorH': bottom, 'heightM': top - bottom,
                        'floorSource': 'local_floor_profile' if len(local_floor) else 'carriage_profile_fallback',
                    })
        support[name] = len(bin_tops)
        if len(bin_tops) >= 6:
            candidates[name] = float(np.median(bin_tops))
            valid_heights.extend(bin_heights)
            valid_tops.extend(bin_tops)
    selected = 'both' if len(candidates) == 2 else next(iter(candidates), None)
    top = float(np.median(valid_tops)) if valid_tops else None
    # 两侧有效段等权参与；不能先各取中位数，再选择较高的一侧。
    height = float(np.median(valid_heights)) if valid_heights else float('nan')
    metadata = {
        'method': ('central_side_rim_longitudinal_bin_median'
                   if selected else 'side_rim_unavailable'),
        'valid': selected is not None,
        'reason': None if selected else 'insufficient_supported_side_sections',
        'definition': 'inner_floor_to_continuous_side_wall_rim',
        'floorH': floor, 'topH': top,
        'heightM': height if selected else None,
        'selectedSide': selected,
        'aggregation': 'pooled_valid_side_sections_median',
        'sideTopH': candidates,
        'supportBinCount': support,
        'binTopsLH': bin_report,
        'binHeightMeasurements': height_report,
        'centralLengthRangeM': [float(central_lo), float(central_hi)],
        'previousProfileTopH': fallback_top,
    }
    return height, metadata


def vehicle_length_extent(points: np.ndarray, ground_plane_local: Optional[np.ndarray]) -> float:
    """整车纵向占用跨度；有地磅面时使用投影到磅面的单位纵轴。"""
    longitudinal = points[:, 1]
    if ground_plane_local is not None:
        normal = ground_plane_local[:3] / np.linalg.norm(ground_plane_local[:3])
        axis = np.array([0., 1., 0.]) - normal[1] * normal
        axis /= np.linalg.norm(axis)
        longitudinal = points @ axis
    low, high = occupancy_extent(longitudinal)
    return float(high - low)


def heights_above_ground(points: np.ndarray, ground_plane_local: np.ndarray) -> np.ndarray:
    """点到地面平面的有符号法向高度；法向按车辆整体位于地面上方定向。"""
    plane = np.asarray(ground_plane_local, dtype=float)
    heights = (points @ plane[:3] + plane[3]) / np.linalg.norm(plane[:3])
    # 标定面法向应朝上；兼容旧文件中法向整体反号的情况。
    return -heights if float(np.median(heights)) < 0.0 else heights


def vehicle_height_above_ground(
    points: np.ndarray, ground_plane_local: Optional[np.ndarray], percentile: float,
    config: Optional[Config] = None,
) -> tuple[Optional[float], dict]:
    """整车高 = 车顶离地磅标定面高度（法向距离的高分位）；无标定面时返回 None。

    车辆局部 H 是车厢底板主平面法向，不是地面法向，且看不到轮胎接地点；
    局部 H 的可见跨度既不含车底到地面的距离，又会被车厢俯仰拉长。
    """
    if ground_plane_local is None:
        return None, {'definition': 'visible_local_h_span_not_ground_height',
                      'reason': 'weighbridge_plane_unavailable'}
    cfg = config or Config()
    values = np.unique(points, axis=0)
    if len(values) >= 4:
        neighbors = cKDTree(values).query(
            values, k=min(cfg.statistical_outlier_neighbors + 1, len(values)), workers=-1)[1][:, 1:]
        samples = values[neighbors]
        centers = samples.mean(axis=1)
        centered = samples - centers[:, None, :]
        _, _, axes = stacked_svd(centered)
        normals = axes[:, -1, :]
        residual = np.abs(np.einsum('ij,ij->i', values - centers, normals))
        errors = np.abs(np.einsum('ijk,ik->ij', centered, normals))
        supported = ((residual <= FACE_TOLERANCE_M)
                     & (np.percentile(errors, 90, axis=1) <= FACE_TOLERANCE_M))
        rows = np.repeat(np.arange(len(values)), neighbors.shape[1])
        graph = csr_matrix((np.ones(len(rows), dtype=bool), (rows, neighbors.ravel())),
                           shape=(len(values), len(values)))
        supported &= graph.minimum(graph.T).getnnz(axis=1) > 0
        values = values[supported]
    if len(values) == 0:
        return float('nan'), {'definition': 'top_above_ground_plane', 'reason': 'supported_vehicle_surface_unavailable'}
    # 百分位作用于每个水平格的车体顶部，而非全部高低点；底板多扫几次不能压低车顶。
    normal = ground_plane_local[:3] / np.linalg.norm(ground_plane_local[:3])
    longitudinal = np.array([0., 1., 0.]) - normal[1] * normal
    longitudinal /= np.linalg.norm(longitudinal)
    lateral = np.cross(longitudinal, normal)
    grid = np.floor(np.c_[values @ lateral, values @ longitudinal] / FACE_CELL_M).astype(int)
    _, ids = unique_cells(grid, return_inverse=True)
    tops = np.full(ids.max() + 1, -np.inf)
    np.maximum.at(tops, ids, heights_above_ground(values, ground_plane_local))
    height = float(np.percentile(tops, percentile))
    return height, {'definition': 'top_above_ground_plane', 'percentile': percentile,
                    'aggregation': 'supported_horizontal_cell_top', 'cellSizeM': FACE_CELL_M,
                    'supportCellCount': len(tops), 'heightM': height}


def out_of_physical_range(dim: VehicleDimensions, config: Config) -> list:
    """列出超出物理合理范围的尺寸项。

    Args:
        dim: 待检查的整车与车厢尺寸。
        config: 提供 ``sanity_vehicle_*`` 三组区间。

    Returns:
        形如 ``[(名称, 实测值, (下限, 上限)), ...]`` 的越界项；全部合理时为空列表。

    Notes:
        阈值只有一份，告警与上传门禁共用，避免"告警说不合理、却照样入库"。
    """
    checks = [
        ("整车长", dim.vehicle_length, config.sanity_vehicle_length),
        ("整车宽", dim.vehicle_width, config.sanity_vehicle_width),
        ("整车高", dim.vehicle_height, config.sanity_vehicle_height),
        ("车厢长", dim.carriage_length, config.sanity_vehicle_length),
        ("车厢宽", dim.carriage_width, config.sanity_vehicle_width),
        ("车厢高", dim.carriage_height, config.sanity_vehicle_height),
    ]
    return [
        (name, float(value), (float(lo), float(hi)))
        for name, value, (lo, hi) in checks
        if not lo <= value <= hi
    ]


class DimensionCalculator:
    """车辆尺寸计算器"""

    def __init__(self, config: Config):
        """初始化统一尺寸口径计算器。

        Args:
            config: 稳健分位数、占用 bin 与几何约束配置。
        """
        self.config = config

    @staticmethod
    def validate_observed_box_inner_dimensions(
        dimensions: VehicleDimensions, split: CarriageSplitResult,
    ) -> None:
        """箱车内尺寸不得大于本次点云验证的外包络；越界项逐项置缺测。"""
        box = split.observed_outer_box
        if box is None:
            return
        # 整车稳健分位数可裁掉波纹外壁边缘；整车外包络至少覆盖
        # 已经由两端面独立观测到的箱体外宽。
        dimensions.carriage_outer_width = box.width
        dimensions.vehicle_width = max(dimensions.vehicle_width, box.width)
        dimensions.width_measurement = {
            **(dimensions.width_measurement or {}), 'outerWidthM': box.width,
        }
        # 两端面的独立拟合及包络分箱存在厘米级偏差；此容差只处理噪声，
        # 不把 11.5m 裁剪成 5.9m，更不以外尺寸填充未知内尺寸。
        tolerance_m = BOX_INNER_OUTER_TOLERANCE_M
        if (np.isfinite(dimensions.carriage_length)
                and dimensions.carriage_length > box.length + tolerance_m):
            candidate = dimensions.carriage_length
            dimensions.carriage_length = float('nan')
            measurement = split.longitudinal_measurement
            if measurement is not None:
                split.longitudinal_measurement = replace(
                    measurement, rear_l=None, front_l=None,
                    metadata={**measurement.metadata,
                              'reason': 'inner_length_exceeds_observed_outer_box',
                              'rejectedLengthM': candidate,
                              'observedOuterLengthM': box.length},
                )
        if (np.isfinite(dimensions.carriage_width)
                and dimensions.carriage_width > box.width + tolerance_m):
            candidate = dimensions.carriage_width
            dimensions.carriage_width = float('nan')
            dimensions.width_measurement = {
                **(dimensions.width_measurement or {}), 'valid': False,
                'reason': 'inner_width_exceeds_observed_outer_box',
                'rejectedWidthM': candidate, 'observedOuterWidthM': box.width,
            }
        if (np.isfinite(dimensions.carriage_height)
                and dimensions.carriage_height > box.height + tolerance_m):
            dimensions.carriage_height = float('nan')

    def calculate(
        self,
        all_points: np.ndarray,
        carriage_points: np.ndarray,
        split: CarriageSplitResult,
        profile: Optional[CarriageProfile] = None,
        ground_plane_local: Optional[np.ndarray] = None,
        synthetic_point_count: int = 0,
    ) -> VehicleDimensions:
        """计算整车与车厢尺寸。

        量取口径（与 carriage.py 的剖面共用同一套基准，避免多处实现分歧）：
            * 长度用 bin 占用跨度而非分位数 —— 俯视雷达下车头/远端点稀疏，
              分位数会把它们当噪声裁掉（实测整车长偏短约 1.4m）；
            * 整车高 = 车顶到地磅标定面的法向高度；没有标定面时退回车辆局部
              H 轴的稳健可见跨度（场景估计的地面可能是更低的路面，不能代替）；
            * 车厢高 = 左右两侧有效段的壁顶到对应底板高度合并取中位数；
            * 车厢宽度从最终车厢云量同一纵向位置两侧上沿的距离，按窗口
              取中位数；外沿宽仍由原始点独立量取，不冒充上沿宽度。
        """
        if len(all_points) == 0:
            raise ValueError("整车点云为空")
        if len(carriage_points) == 0:
            raise ValueError("车厢点云为空")

        cfg = self.config
        p_low, p_high = cfg.dimension_percentile_low, cfg.dimension_percentile_high
        observed_carriage_points = split.carriage_points
        if profile is None:
            profile = CarriageAnalyzer(cfg).analyze(observed_carriage_points)

        # 整车长用占用跨度抗稀疏（车头/远端点稀疏是雷达沿车长方向的衰减特性）；
        # 宽度方向是横向密集采样，没有稀疏问题，仍用分位数 —— 占用跨度的低点数
        # 门槛会把后视镜/挡泥板/残留噪声算进去（实测整车宽虚高 0.3m）。
        vehicle_length = vehicle_length_extent(all_points, ground_plane_local)
        vehicle_width = _robust_span(all_points[:, 0], p_low, p_high)
        vehicle_height, vehicle_height_metadata = vehicle_height_above_ground(
            all_points, ground_plane_local, p_high, cfg)
        if vehicle_height is None:
            vehicle_height = _robust_span(all_points[:, 2], p_low, p_high)

        # 正式链路的 measurement 是主体侧壁前后端；拉筋 N+1 段共用它，
        # 保证间距之和等于侧壁长度。未执行测长的离线入口保留分割跨度。
        measurement = split.longitudinal_measurement
        carriage_length = (
            float(split.split_y - split.carriage_start_y) if measurement is None
            else measurement.length if measurement.valid else float('nan')
        )
        completed = np.asarray(carriage_points, dtype=float)
        completion_used = synthetic_point_count > 0
        # 业务高度和净宽也使用同一份最终车厢云。原剖面仅供补点前的
        # 载荷判断/几何锚定，不能在尺寸入口里暗中优先采用原始云。
        final_profile = (
            CarriageAnalyzer(cfg).analyze(completed)
            if len(completed) != len(observed_carriage_points)
            and len(getattr(profile, 'interior_mask', ())) != len(completed)
            else profile
        )
        carriage_height, height_metadata = _measure_continuous_side_height(
            completed, final_profile, cfg)
        carriage_outer_width = _robust_span(observed_carriage_points[:, 0], p_low, p_high)
        width_result = measure_inner_width(
            completed, split.carriage_start_y, split.split_y,
            getattr(final_profile, 'floor_z', float('nan')),
            getattr(final_profile, 'wall_top', float('nan')),
            config=cfg,
            allow_sparse_rectangular=bool(
                completion_used or split.observed_outer_box is not None
                or (measurement is not None and measurement.valid
                    and measurement.metadata.get('sparseSideCompletion') is True)),
        )
        carriage_width = (width_result.width_m if width_result.width_m is not None
                          else float('nan'))
        width_metadata = {
            **width_result.metadata,
            'adoptedSource': (
                'completed_cloud_inner_side_faces' if completion_used
                else 'sparse_rectangular_observed_inner_faces'
                if width_result.metadata.get('sparseRectangularInference')
                else 'observed_inner_side_faces' if width_result.width_m is not None
                else 'inner_width_unavailable'),
            'outerWidthM': carriage_outer_width,
            'source': 'current_final_carriage_cloud',
            'completionUsed': completion_used,
            'syntheticPointCount': synthetic_point_count,
        }
        # 车厢是整车点集的子集，但分别计算分位数时会因点密度权重不同出现毫米级
        # “车厢比整车宽”的倒挂。整车外包络必须覆盖已观测到的车厢外沿。
        vehicle_width = max(vehicle_width, carriage_outer_width)

        dimensions = VehicleDimensions(
            vehicle_length=vehicle_length,
            vehicle_width=vehicle_width,
            vehicle_height=vehicle_height,
            carriage_length=carriage_length,
            carriage_width=carriage_width,
            carriage_height=carriage_height,
            carriage_outer_width=carriage_outer_width,
            width_measurement=width_metadata,
            height_measurement=height_metadata,
            vehicle_height_measurement=vehicle_height_metadata,
        )

        logger.info(
            f"整车尺寸：长={vehicle_length:.2f}m, "
            f"宽={vehicle_width:.2f}m, "
            f"高={vehicle_height:.2f}m（{vehicle_height_metadata['definition']}）"
        )
        logger.info(
            f"车厢实测尺寸：内壁长={carriage_length:.2f}m, "
            f"采用宽={carriage_width:.2f}m ({width_metadata['adoptedSource']}), "
            f"外沿宽={carriage_outer_width:.2f}m, 栏板深={carriage_height:.2f}m"
        )

        # [优化 L15] 物理合理性自检
        self._sanity_check(dimensions)

        return dimensions

    def measure_cab_to_front_wall_at_height(
        self,
        all_points: np.ndarray,
        split: CarriageSplitResult,
        ground_plane_local: Optional[np.ndarray],
        front_reference_l: Optional[float] = None,
        front_reference_source: str = "container_front_gutter_clearance",
    ) -> bool:
        """按离地固定高度截面定位车头前沿并写回业务测距元数据。

        ``split.head_y`` 是全高度整车端点，不能用于这项业务测量。这里按地面
        平面的有符号法向距离选点，优先使用 1.40±0.02m；点数不足时只扩到
        ±0.05m。先逐点排除前内壁搜索带内的壁厚/附件，再检查点数并取纵向
        99% 分位，避免密集壁点冒充车头或挤掉稀疏的真实车头前沿。

        前壁端点同样落在该截面：取离截面最近的一段下部前壁实测面外推到截面，
        不用被顶部篷布/挡板带偏的整面拟合；下部证据不足时才退回整面投影，
        来源记在 ``cab_measurement_front_source``。

        集装箱（短矩形箱车）业务口径：``front_reference_l`` 传前端水槽
        净空面（补点后最内结构面）的局部 L，车头距量到该参考面，来源记
        ``container_front_gutter_clearance``。普通长车传顶部开口前沿
        （``top_opening.measure_top_opening_front_edge``），来源记
        ``top_opening_front_edge``，与内长前端同一位置；量不到时不传，仍按前内壁。
        """
        cfg = self.config
        split.cab_measurement_height = cfg.cab_distance_measurement_height
        split.cab_measurement_method = "ground_height_slice_percentile"
        split.cab_measurement_reason = None
        split.cab_measurement_head_y = None
        split.cab_measurement_half_band = None
        split.cab_measurement_point_count = 0
        split.cab_measurement_front_y = None
        split.cab_measurement_front_source = None

        if ground_plane_local is None:
            split.cab_measurement_reason = "weighbridge_plane_unavailable"
            logger.warning("缺少地磅标定面，无法按磅面以上 1.40m 测量车头到车厢前内壁")
            return False

        plane = np.asarray(ground_plane_local, dtype=float)
        if plane.shape != (4,) or not np.all(np.isfinite(plane)):
            split.cab_measurement_reason = "ground_plane_invalid"
            logger.warning("地面平面非法，无法执行固定高度车头测距")
            return False
        normal_norm = float(np.linalg.norm(plane[:3]))
        if normal_norm <= 1e-9:
            split.cab_measurement_reason = "ground_plane_invalid"
            logger.warning("地面平面法向量为零，无法执行固定高度车头测距")
            return False

        heights = heights_above_ground(all_points, plane)

        target = cfg.cab_distance_measurement_height
        half_bands = [cfg.cab_distance_height_half_band]
        if cfg.cab_distance_max_height_half_band > half_bands[0]:
            half_bands.append(cfg.cab_distance_max_height_half_band)

        front_at_point = split.split_y
        if split.front_wall_plane is not None:
            a, b, c = split.front_wall_plane
            front_at_point = a * all_points[:, 0] + b * all_points[:, 2] + c
        ahead_distance = all_points[:, 1] - front_at_point
        separated = ahead_distance > FACE_SEARCH_HALF_BAND_M
        candidates = np.empty(0, dtype=float)
        used_half_band = half_bands[-1]
        for half_band in half_bands:
            height_mask = (
                (heights >= target - half_band)
                & (heights <= target + half_band)
            )
            mask = height_mask & separated
            candidates = all_points[mask, 1]
            used_half_band = half_band
            if len(candidates) >= cfg.cab_distance_min_slice_points:
                break

        split.cab_measurement_half_band = float(used_half_band)
        split.cab_measurement_point_count = int(len(candidates))
        if len(candidates) < cfg.cab_distance_min_slice_points:
            near_wall_count = np.count_nonzero(height_mask & (ahead_distance > 0.))
            split.cab_measurement_reason = (
                "insufficient_separated_cab_slice_points"
                if near_wall_count >= cfg.cab_distance_min_slice_points
                else "insufficient_height_slice_points"
            )
            logger.warning(
                "离地 %.2fm 排除端壁搜索带后车头薄层仅 %d 点（要求至少 %d），测距无效",
                target,
                len(candidates),
                cfg.cab_distance_min_slice_points,
            )
            return False

        measured_head = float(
            np.percentile(candidates, cfg.cab_distance_head_percentile)
        )
        front_y = split.split_y
        split.cab_measurement_front_source = "carriage_split_front_inner"
        if front_reference_l is not None and np.isfinite(front_reference_l):
            # 集装箱：前端水槽净空面；普通长车：顶部开口前沿（与内长前端同位）。
            front_y = float(front_reference_l)
            split.cab_measurement_front_source = front_reference_source
        elif split.front_wall_plane is not None:
            # 两个端点必须同处标定地面以上的业务截面；车厢底板更高时，
            # 将已观测前内壁平面投影到该截面，不改用顶部外梁或整车最前点。
            nose = all_points[mask]
            nose = nose[nose[:, 1] >= np.percentile(nose[:, 1], 95)]
            w = float(np.median(nose[:, 0]))
            lower_face = self._lower_front_wall_face(all_points, heights, split, target)
            if lower_face is not None:
                a, b, c = lower_face.coefficients
                split.cab_measurement_front_source = "lower_wall_face_near_measurement_height"
            else:
                a, b, c = split.front_wall_plane
                split.cab_measurement_front_source = "full_height_front_face_projection"
            up_plane = plane / normal_norm
            if float(np.median((all_points @ up_plane[:3]) + up_plane[3])) < 0:
                up_plane = -up_plane
            denominator = up_plane[2] + up_plane[1] * b
            if abs(denominator) > 1e-6:
                h = (target - up_plane[0] * w - up_plane[1] * (a * w + c)
                     - up_plane[3]) / denominator
                front_y = float(a * w + b * h + c)
        distance = measured_head - front_y
        if not np.isfinite(distance) or distance <= 0.0:
            split.cab_measurement_reason = "measured_head_not_ahead_of_front_wall"
            logger.warning("固定高度车头前沿未落在车厢前内壁之前，测距无效")
            return False

        split.cab_measurement_head_y = measured_head
        split.cab_measurement_front_y = front_y
        logger.info(
            "固定高度车头测距：地面以上 %.2f±%.2fm，点数=%d，"
            "车头前沿=%.3fm，车厢前内壁=%.3fm（%s），距离=%.3fm",
            target,
            used_half_band,
            len(candidates),
            measured_head,
            front_y,
            split.cab_measurement_front_source,
            distance,
        )
        return True

    @staticmethod
    def _lower_front_wall_face(
        all_points: np.ndarray,
        heights: np.ndarray,
        split: CarriageSplitResult,
        target: float,
    ) -> Optional[ObservedFace]:
        """取离测距截面最近的一段下部前壁实测面；证据不足时返回 None。

        在整面前壁拟合面两侧的端壁搜索带内，只保留离地不低于截面、且不超出最低
        观测点以上 ``CAB_FRONT_WALL_LOW_WINDOW_M`` 的点，重新提取有二维覆盖的实测面。
        顶部篷布、挡板因此不参与；多块平行面按分割端壁的同一规则取面向厢内的一面。
        """
        a, b, c = split.front_wall_plane
        offset = all_points[:, 1] - (a * all_points[:, 0] + b * all_points[:, 2] + c)
        band = (np.abs(offset) <= FACE_SEARCH_HALF_BAND_M) & (heights >= target)
        carriage = split.carriage_points
        if len(carriage):
            w_lo, w_hi = np.percentile(carriage[:, 0], [2.0, 98.0])
            band &= (all_points[:, 0] >= w_lo) & (all_points[:, 0] <= w_hi)
        if np.count_nonzero(band) < FACE_MIN_POINTS:
            return None
        start = max(target, float(np.percentile(heights[band], 2.0)))
        window = all_points[band & (heights <= start + CAB_FRONT_WALL_LOW_WINDOW_M)]
        faces = supported_faces(window, 1)
        if not faces:
            return None
        max_cells = max(len(face.cells) for face in faces)
        faces = [face for face in faces if len(face.cells) >= max_cells * 0.5]
        w_ref, h_ref = np.median(window[:, [0, 2]], axis=0)
        # 前壁面向厢内即 L 较小的一面，与 measure_end_faces 的取面规则一致。
        return min(faces, key=lambda face: face.at(w_ref, h_ref))

    def _sanity_check(self, dim: VehicleDimensions) -> None:
        """超出物理范围告警。

        Notes:
            这里只负责说清楚哪一项越界；是否放行由导出层的
            ``measurement_valid`` 决定，两处共用 :func:`out_of_physical_range`。
        """
        cfg = self.config
        for name, value, (lo, hi) in out_of_physical_range(dim, cfg):
            if not np.isfinite(value):
                continue
            logger.warning(
                f"⚠ {name}={value:.2f}m 超出合理范围 [{lo}, {hi}]，该值不入库"
            )

        # 车厢/整车比例
        if dim.vehicle_length > 0:
            ratio = dim.carriage_length / dim.vehicle_length
            if ratio < cfg.sanity_carriage_min_ratio:
                logger.warning(
                    f"⚠ 车厢/整车长度比 {ratio:.2f} 过低，分割可能错误"
                )
            elif ratio > 1.0:
                logger.warning(
                    f"⚠ 车厢长度({dim.carriage_length:.2f}) > 整车长度({dim.vehicle_length:.2f})"
                )
