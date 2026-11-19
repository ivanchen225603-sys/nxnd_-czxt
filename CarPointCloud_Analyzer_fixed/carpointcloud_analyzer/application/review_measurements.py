"""矩形车型逐字段计算结果；各字段独立输出，不把空值解释为零。"""

from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.box_geometry import rotate_wl
from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer, CarriageProfile, spatially_balanced_cloud
from carpointcloud_analyzer.domain.dimensions import DimensionCalculator, _measure_continuous_side_height, vehicle_height_above_ground, vehicle_length_extent
from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.domain.floor import FloorProfile, FloorProfileAnalyzer
from carpointcloud_analyzer.domain.floor_evidence import lower_wall_floor_conflict
from carpointcloud_analyzer.domain.geometry import LocalRibDetector
from carpointcloud_analyzer.domain.load_status import LoadStatusClassifier
from carpointcloud_analyzer.domain.inner_length_measurement import measure_final_inner_end_faces
from carpointcloud_analyzer.domain.longitudinal_measurement import measurement_metadata, rib_projections
from carpointcloud_analyzer.domain.models import (
    CarriageSplitResult, LoadStatusResult, RibInfo, VehicleDimensions,
)
from carpointcloud_analyzer.domain.rectangular_completion import complete_rectangular_floor
from carpointcloud_analyzer.domain.rib_completion import complete_rib_gaps
from carpointcloud_analyzer.domain.rib_side_measurement import refine_rib_side_positions
from carpointcloud_analyzer.domain.utils import occupancy_extent
from carpointcloud_analyzer.domain.utils.histogram import robust_span

# 复核接口统一用米，已有上传辅助函数用厘米。
CM_PER_M = 100

# 车头朝向证据：端壁外侧多出来的点。与 wall_reference 同口径。
ORIENTATION_MARGIN_M = 1
MIN_ORIENTATION_POINTS = 500
ORIENTATION_DOMINANCE = 3

# 取车厢内部点时从四壁内缩，避开壁面自身与转角的回波。
INTERIOR_WIDTH_MARGIN_M = 0.2
END_WALL_MARGIN_M = 0.12
COMPLETED_BOUNDS_TOLERANCE_M = 0.05
INNER_RIM_SEARCH_EPS_M = 0.001
INNER_RIM_COMPLETENESS_TOLERANCE_M = 0.01


@dataclass
class RectangularFinalAnalysis:
    """矩形分支共用的最终点集和测量产物；points 保持输入参考坐标系。"""

    review: dict
    points: np.ndarray
    dimensions: VehicleDimensions
    split: CarriageSplitResult
    load_status: LoadStatusResult
    profile: CarriageProfile
    ribs: list[RibInfo]
    floor: FloorProfile | None
    completion: dict


def _dimension_values(values):
    """将兼容的字典尺寸转换成统一校验模型。"""
    return VehicleDimensions(**{
        f'{group}_{name}': float(value) if value is not None else float('nan')
        for group in ('vehicle', 'carriage') for name, value in values[group].items()
    })


def build_review_dimension_payload(review, config, source_file):
    """将矩形车型计算值写入现有厘米字段；缺测保持 null。"""
    def cm(value):
        """米转厘米；``None`` 保持为空，不能被解释成 0。"""
        if value is None:
            return None
        if not np.isfinite(value) or value < 0:
            raise ValueError("invalid review measurement")
        return round(float(value) * CM_PER_M, 1)

    carriage = review["dimensions"]["carriage"]
    bounds = review.get('innerLengthBoundsM')
    if bounds is not None and 'ribPositionsM' in review:
        _, rib_distances = ResultExporter._quantize_rib_distances(
            bounds[1], bounds[0], review['ribPositionsM'])
    else:
        rib_distances = [{"ribId": index, "distToPrev": cm(distance)}
                         for index, distance in enumerate(review["ribIntervals"], 1)]
    return {
        "idempotencyKey": Path(source_file).stem,
        "cameraIdentifier": config.camera_identifier,
        "licensePlate": config.license_plate or None, "getTime": None,
        "detectStatus": "success", "sourceFile": source_file,
        "resultJsonPath": None, "resultImagePath": None,
        "dimensions": {
            "cabToTrailerFrontDist": cm(review["cabToTrailerFrontDist"]),
            "trailerLength": cm(carriage["length"]), "trailerWidth": cm(carriage["width"]),
            "trailerBodyHeight": cm(carriage["height"]),
            "trailerBottomToGroundHeight": cm(review["trailerBottomToGroundHeight"]),
            "highLowPlateHeight": cm(review["highLowPlateHeight"]),
        },
        "ribs": rib_distances,
        "actualRibCount": review.get('detectedRibCount'),
        "maximumWidthCm": cm(review.get('carriageOuterWidthM')),
        "wires": [],
        "highLowPlates": [{**plate, "highestToGround": cm(plate["highestToGround"]),
                           "lowestToGround": cm(plate["lowestToGround"])}
                          for plate in review["highLowPlates"]],
    }


def _orient_front_to_high_l(local, box, plane):
    """判断车头在哪一端；车头在低 L 侧时把点云与地面整体镜像到高 L 侧。

    就地修改 ``local`` 与 ``plane``（两者都是本函数调用链内的副本），使下游
    只需处理"车头在高 L 侧"一种情形。

    Returns:
        ``(low, high, w0, w1, front_low, front_high)``，均为镜像后的取值。
    """
    low, high = box.rear_l, box.front_l
    w0, w1 = box.w_range
    low_support = int(np.sum(local[:, 1] < low - ORIENTATION_MARGIN_M))
    high_support = int(np.sum(local[:, 1] > high + ORIENTATION_MARGIN_M))
    front_low = low_support > max(MIN_ORIENTATION_POINTS,
                                  high_support * ORIENTATION_DOMINANCE)
    front_high = high_support > max(MIN_ORIENTATION_POINTS,
                                    low_support * ORIENTATION_DOMINANCE)
    if front_low:
        local[:, :2] *= -1
        low, high = -high, -low
        w0, w1 = -w1, -w0
        if plane is not None:
            plane[:2] *= -1
    return low, high, w0, w1, front_low, front_high


def _build_dimensions(completed, synthetic, box, config, opening, opening_applied, front_low):
    """所有尺寸均在最终云按几何范围选点；模型仅提供搜索边界。"""
    if not len(synthetic) or not np.isfinite(synthetic).all():
        raise ValueError('completed rectangular cloud required for dimensions')
    l0, l1 = occupancy_extent(completed[:, 1])
    p_low, p_high = config.dimension_percentile_low, config.dimension_percentile_high
    low, high = ((-box.front_l, -box.rear_l) if front_low
                 else (box.rear_l, box.front_l))
    w0, w1 = ((-box.w_range[1], -box.w_range[0]) if front_low else box.w_range)
    tolerance = COMPLETED_BOUNDS_TOLERANCE_M
    # 补点只覆盖缺失区域，跨度可以小于整箱；只核对是否落在本箱坐标内。
    if (np.any(np.min(synthetic, axis=0) < np.array([w0, low, box.h_range[0]]) - tolerance)
            or np.any(np.max(synthetic, axis=0) > np.array([w1, high, box.h_range[1]]) + tolerance)):
        raise ValueError('completed rectangular wall bounds do not match detected box')
    body = completed[(completed[:, 0] >= w0 - tolerance)
                     & (completed[:, 0] <= w1 + tolerance)
                     & (completed[:, 1] >= low - tolerance)
                     & (completed[:, 1] <= high + tolerance)
                     & (completed[:, 2] >= box.h_range[0] - tolerance)
                     & (completed[:, 2] <= box.h_range[1] + tolerance)]
    outer_width = float(np.ptp(body[:, 0]))
    completed_height = float(np.ptp(body[:, 2]))
    # 已知四壁各自的搜索窗中取主平面，避免把回波厚度算进内尺寸。
    def face_coordinate(axis, anchor):
        values = body[np.abs(body[:, axis] - anchor) <= tolerance, axis]
        if len(values) < 4:
            raise ValueError('completed rectangular face missing')
        return float(np.median(values))

    rear, front = face_coordinate(1, low), face_coordinate(1, high)
    left, right = face_coordinate(0, w0), face_coordinate(0, w1)
    if opening_applied:
        w0, w1 = opening['innerWRange']
        y0, y1 = opening['innerLRange']
        if front_low:
            w0, w1, y0, y1 = -w1, -w0, -y1, -y0
        # 外壁点落在开口范围外，范围内的补点是顶部连接带的内缘。
        # 元数据仅用于确定搜索窗口；长宽取实际生成的点坐标。
        inner = body[
            (body[:, 0] >= w0 - INNER_RIM_SEARCH_EPS_M)
            & (body[:, 0] <= w1 + INNER_RIM_SEARCH_EPS_M)
            & (body[:, 1] >= y0 - INNER_RIM_SEARCH_EPS_M)
            & (body[:, 1] <= y1 + INNER_RIM_SEARCH_EPS_M)
            & (body[:, 2] >= box.h_range[1] - .15)
        ]
        if len(inner) < 4:
            raise ValueError('completed rectangular inner rim missing')
        rear, front = float(np.min(inner[:, 1])), float(np.max(inner[:, 1]))
        left, right = float(np.min(inner[:, 0])), float(np.max(inner[:, 0]))
        if (abs(right - left - (w1 - w0)) > INNER_RIM_COMPLETENESS_TOLERANCE_M
                or abs(front - rear - (y1 - y0)) > INNER_RIM_COMPLETENESS_TOLERANCE_M):
            raise ValueError('completed rectangular inner rim is incomplete')
    dimensions = {
        "vehicle": {
            "length": float(l1 - l0),
            "width": max(float(robust_span(completed[:, 0], p_low, p_high)), outer_width),
            "height": max(float(robust_span(completed[:, 2], p_low, p_high)),
                          completed_height),
        },
        "carriage": {"length": front - rear, "width": right - left,
                     "height": completed_height},
    }
    return dimensions, l0, l1, (left, right, rear, front), outer_width, body


def _weighbridge_heights(local, plane, box, w0, w1, low, high, p_high, config):
    """模型上下沿与整车顶面相对地磅面的高度；无标定平面时全部留空。"""
    if plane is None:
        return None, None, None
    norm = np.linalg.norm(plane[:3])
    center = np.array([(w0 + w1) / 2, (low + high) / 2, box.h_range[0]])
    model_bottom = float((center @ plane[:3] + plane[3]) / norm)
    center[2] = box.h_range[1]
    model_top = float((center @ plane[:3] + plane[3]) / norm)
    vehicle_top = max(vehicle_height_above_ground(local, plane, p_high, config)[0], model_top)
    return model_bottom, model_top, vehicle_top


def _field_provenance(height_source, opening_applied, floor, ribs, vehicle_top_known):
    """逐字段标注实测或几何推算来源。"""
    opening_source = ("completed_cloud_inner_rim" if opening_applied
                      else "completed_cloud_outer_wall_span")
    return {
        "vehicle.height": ("completed_cloud_top_above_weighbridge" if vehicle_top_known
                           else "completed_cloud_vertical_span_not_ground_height"),
        "dimensions.carriage": "completed_rectangular_cloud",
        "dimensions.carriage.height": height_source,
        "dimensions.carriage.length": opening_source,
        "dimensions.carriage.width": opening_source,
        "trailerBottomToGroundHeight": ("observed_floor_profile" if floor
                                        else "floor_not_observed"),
        "modelLowerEdgeToWeighbridgeHeight": "model_lower_edge_not_observed_floor",
        "highLowPlates": "calculated" if floor else "floor_profile_unavailable",
        "ribs": ("detector_returned_candidates" if ribs
                 else "no_candidates_not_proof_of_absence"),
        "wires": "detector_not_implemented",
    }


def calculate_review_measurements(points, reference, transform, ground_plane,
                                  calibrated, config):
    """保留离线字典接口；所有测量委托同一最终云分析入口。"""
    return analyze_rectangular_final_cloud(
        points, reference, transform, ground_plane, calibrated, config).review


def analyze_rectangular_final_cloud(points, reference, transform, ground_plane,
                                    calibrated, config):
    """补完结构后冻结点集，判空、测量与归档共享它的几何子集。"""
    yaw = np.deg2rad(reference.metadata["yawDeg"])
    local = rotate_wl(points, -yaw)
    synthetic = rotate_wl(reference.points, -yaw)
    box = reference.box
    matched = bool(reference.metadata.get("profileMatched"))
    height_source = 'completed_cloud_wall_height'
    opening = reference.metadata.get("symmetricOpening", {})
    opening_applied = matched and opening.get("applied", False)

    plane = None
    if ground_plane is not None:
        plane = transform.plane_to_local(ground_plane)
        plane[:3] = rotate_wl(plane[None, :3], -yaw)[0]
    low, high, w0, w1, front_low, front_high = _orient_front_to_high_l(local, box, plane)
    if front_low:
        synthetic[:, :2] *= -1
    completed_local = np.vstack((local, synthetic))
    direction_known = front_low or front_high
    inner_w = (w0 + INTERIOR_WIDTH_MARGIN_M, w1 - INTERIOR_WIDTH_MARGIN_M)
    aligned_box = replace(box, rear_l=low, front_l=high, w_range=(w0, w1))
    floor_added, floor_completion = complete_rectangular_floor(
        completed_local, aligned_box, inner_w, box.h_range[0], config)
    before_ribs = np.vstack((completed_local, floor_added))
    detector = LocalRibDetector(config)
    rib_search = before_ribs[(before_ribs[:, 1] > low + END_WALL_MARGIN_M)
                             & (before_ribs[:, 1] < high - END_WALL_MARGIN_M)]
    seeds = detector.detect(rib_search, low, high, inner_w)
    rib_added, rib_completion = complete_rib_gaps(
        before_ribs, seeds, inner_w, low, high, *box.h_range)
    completed_local = np.vstack((before_ribs, rib_added))
    dimensions, l0, l1, bounds, outer_width, carriage = _build_dimensions(
        completed_local, np.vstack((synthetic, floor_added, rib_added)), box,
        config, opening, opening_applied, front_low)
    if calibrated:
        dimensions['vehicle']['length'] = vehicle_length_extent(completed_local, plane)
    wall_span = dimensions['carriage']['height']
    _, _, rear, front = bounds
    profile = None
    height_metadata = None
    split = CarriageSplitResult(carriage, front, rear, l1, l0,
                                boundaries_are_inner_faces=True,
                                boundary_method="completed_rectangular_cloud",
                                observed_outer_box=aligned_box)
    side_points = carriage[(carriage[:, 1] >= low) & (carriage[:, 1] <= high)]
    measurement = measure_final_inner_end_faces(
        side_points, low, high, *box.h_range, plane if calibrated else None)
    measurement.metadata['completionUsed'] = True
    dimensions['carriage']['length'] = measurement.length if measurement.valid else float('nan')
    split.longitudinal_measurement = measurement
    split.cab_measurement_method = 'front_direction_unknown'
    split.cab_measurement_reason = 'front_direction_unknown'
    if direction_known:
        DimensionCalculator(config).measure_cab_to_front_wall_at_height(
            completed_local, split, plane if calibrated else None)
    cab_distance = float(split.cab_to_front_wall) if split.cab_measurement_valid else None

    away_from_ends = ((carriage[:, 1] > rear + END_WALL_MARGIN_M)
                      & (carriage[:, 1] < front - END_WALL_MARGIN_M))
    ribs = detector.detect(carriage[away_from_ends], rear, front, inner_w)
    ribs.sort(key=lambda rib: high - rib.y_position)
    for index, rib in enumerate(ribs, 1):
        rib.rib_id = index
    measurement.rib_side_projections = refine_rib_side_positions(
        carriage, ribs, measurement, *box.h_range, final_cloud=True)
    length_metadata = measurement_metadata(measurement, ribs)
    positions = [row['positionM'] for row in rib_projections(ribs, measurement) or []]
    usable = away_from_ends.copy()
    for rib in ribs:
        usable &= np.abs(carriage[:, 1] - rib.y_position) > .065
    if len(floor_added):
        # 补出的底板只填未观测格，不能替这些格投“空”票；判空排除它们。
        usable &= cKDTree(floor_added).query(carriage, k=1)[0] > 0
    load_cloud = carriage[usable]
    interior = load_cloud[(load_cloud[:, 0] > inner_w[0])
                          & (load_cloud[:, 0] < inner_w[1])]
    if len(interior) < config.carriage_interior_min_points:
        load_status = LoadStatusResult('Uncertain', .2, 'carriage_interior_grid_insufficient',
                                       {'interiorPoints': float(len(interior))})
    else:
        load_cloud = spatially_balanced_cloud(load_cloud, config)
        load_profile = CarriageAnalyzer(config).analyze(load_cloud)
        load_status = LoadStatusClassifier(config).classify(
            load_cloud, load_profile)
        # 与普通路径共用矛盾检查，输入仍是同一最终云，不能把高处货面当底板。
        if (load_status.calculation_allowed and
                lower_wall_floor_conflict(load_cloud, load_profile, config)['conflict']):
            load_status = LoadStatusResult('Uncertain', .2, 'observed_floor_contradicts_inner_walls',
                                           load_status.metrics)
        if load_status.calculation_allowed:
            profile = load_profile
            dimensions['carriage']['height'], height_metadata = _measure_continuous_side_height(
                carriage, profile, config)
    if profile is None:
        profile = CarriageAnalyzer(config).analyze(carriage)
    floor = None
    if load_status.calculation_allowed and direction_known and calibrated and plane is not None:
        floor = FloorProfileAnalyzer(config).analyze(
            carriage, inner_w,
            measurement.front_l if measurement.valid else front,
            measurement.rear_l if measurement.valid else rear, ground_plane_local=plane,
            confidence="final_cloud",
            ground_plane_source="weighbridge_calibration",
            longitudinal_axis=measurement.longitudinal_axis,
        )
    floor_dimensions, high_low = ResultExporter._build_high_low_plate_payload(floor)
    floor_height = floor_dimensions["trailerBottomToGroundHeight"]
    high_low_height = floor_dimensions["highLowPlateHeight"]
    model_bottom, model_top, vehicle_top = _weighbridge_heights(
        completed_local, plane if calibrated else None, box, w0, w1, low, high,
        config.dimension_percentile_high, config)
    if vehicle_top is not None:
        # 整车高是车顶离地磅面高度，不是局部 H 的可见跨度。
        dimensions['vehicle']['height'] = vehicle_top

    result = {
        "unit": "m", "dimensions": dimensions,
        "cabToTrailerFrontDist": cab_distance,
        "cabMeasurement": {
            "heightAboveGround": config.cab_distance_measurement_height,
            "heightReference": "weighbridge" if calibrated else "unavailable",
            "heightHalfBand": split.cab_measurement_half_band,
            "pointCount": split.cab_measurement_point_count,
            "frontDirectionKnown": bool(direction_known),
            "reason": ("calculated" if cab_distance is not None else
                       split.cab_measurement_reason or "front_direction_unknown"),
            "frontWallSource": "reconstructed_end_plane",
        },
        "vehicleTopToWeighbridgeHeight": vehicle_top,
        "trailerBottomToGroundHeight": (floor_height / CM_PER_M
                                        if floor_height is not None else None),
        "modelLowerEdgeToWeighbridgeHeight": model_bottom,
        "modelUpperEdgeToWeighbridgeHeight": model_top,
        "highLowPlateHeight": (high_low_height / CM_PER_M
                               if high_low_height is not None else None),
        "highLowPlates": [{**p, "highestToGround": p["highestToGround"] / CM_PER_M,
                           "lowestToGround": p["lowestToGround"] / CM_PER_M}
                          for p in high_low],
        "floorProfile": floor.to_dict() if floor else None,
        "ribs": [rib.to_dict() for rib in ribs],
        "detectedRibCount": len(ribs), "ribCount": len(ribs) if ribs else None,
        "ribIntervals": length_metadata['ribIntervalsM'],
        "ribPositionsM": positions,
        "innerLengthBoundsM": [rear, front],
        "carriageOuterWidthM": outer_width,
        "carriageVisibleWallSpanM": wall_span,
        "loadStatus": load_status.to_dict(),
        "wires": [], "wireCount": None,
        "fields": _field_provenance(height_source, opening_applied, floor, ribs,
                                    vehicle_top is not None),
        "observedInteriorPointCount": len(interior),
        "measuredPointCount": len(points),
        "syntheticPointCount": len(completed_local) - len(local),
        "wallCompletion": {**reference.metadata, 'frontFloor': floor_completion,
                           'ribCompletion': rib_completion},
    }
    if matched:
        result["carriageOpening"] = opening
        result["carriageHeight"] = {
            "innerHeightM": dimensions['carriage']['height'] if height_metadata else None,
            "exteriorModelHeightM": box.height,
            "innerHeightSource": height_source,
            "exteriorHeightSource": reference.metadata["heightSource"],
            "interpretation": "customer_confirmed_for_matched_vehicle_type",
            "note": "内高从最终补点云图的壁面跨度量取；未补出底板时不代表底板已观测。",
        }
    measured = _dimension_values(dimensions)
    if height_metadata is None:
        measured.carriage_height = float('nan')
    measured.carriage_outer_width = outer_width
    measured.height_measurement = height_metadata
    measured.vehicle_height_measurement = (
        {'definition': 'top_above_ground_plane', 'percentile': config.dimension_percentile_high,
         'aggregation': 'maximum_of_supported_horizontal_cell_top_and_rectangular_model_top',
         'heightM': vehicle_top} if vehicle_top is not None
        else {'definition': 'visible_local_h_span_not_ground_height',
              'reason': 'weighbridge_plane_unavailable'})
    # 矩形模型已由 _build_dimensions 定位同一开口的左右上沿；直接沿用，
    # 不把已确认的稀疏模型再送入普通车型的实测壁面密度检查。
    measured.width_measurement = {
        'valid': True, 'definition': 'median_paired_side_rim_spacing',
        'method': 'completed_rectangular_paired_rims',
        'widthM': measured.carriage_width, 'completionUsed': True,
        'adoptedSource': 'completed_cloud_inner_side_faces',
    }
    DimensionCalculator.validate_observed_box_inner_dimensions(measured, split)
    # 外包络仅作为合法性上界；实际外宽仍来自最终点集。
    measured.carriage_outer_width = outer_width
    measured.width_measurement['outerWidthM'] = outer_width
    result['dimensions']['carriage']['length'] = measured.to_dict()['carriage']['length']
    result['dimensions']['carriage']['width'] = measured.to_dict()['carriage']['width']
    result['innerLengthBoundsM'] = [measurement.rear_l, measurement.front_l]
    result['fields']['dimensions.carriage.length'] = 'completed_cloud_body_side_length'
    result['fields']['dimensions.carriage.width'] = 'completed_cloud_paired_side_rims'
    restored = completed_local.copy()
    if front_low:
        restored[:, :2] *= -1
    restored = rotate_wl(restored, yaw)
    return RectangularFinalAnalysis(result, restored, measured, split, load_status,
                                    profile, ribs, floor, result['wallCompletion'])
