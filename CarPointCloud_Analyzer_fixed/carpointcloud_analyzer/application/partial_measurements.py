"""普通车型逐项计算结果；缺一项不隐藏其余项。"""

from __future__ import annotations

import math
from typing import Any, TYPE_CHECKING

from carpointcloud_analyzer.domain.dimensions import BOX_INNER_OUTER_TOLERANCE_M

if TYPE_CHECKING:
    from carpointcloud_analyzer.domain.carriage import CarriageProfile
    from carpointcloud_analyzer.domain.floor import FloorProfile
    from carpointcloud_analyzer.domain.models import CarriageSplitResult, LoadStatusResult, RibInfo, VehicleDimensions


def _number(value: Any) -> float | None:
    """未知、非法及非有限数保持 null，不用零代替缺测。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) and value >= 0 else None


def build_partial_measurements(
    dimensions: VehicleDimensions, split: CarriageSplitResult, load_status: LoadStatusResult,
    profile: CarriageProfile | None, floor_profile: FloorProfile | None,
    floor_dimensions: dict[str, Any], high_low_plates: list[dict[str, Any]],
    ribs: list[RibInfo], length_metadata: dict[str, Any] | None, reason: str,
) -> dict[str, Any]:
    """仅采用已执行的侧壁长度、上沿间距与高度测量；不回填水槽净空。"""
    fields = {}

    def field(key: str, value: Any, source: str | None, unavailable: str,
              unit: str = 'm', confidence: str = 'medium') -> float | None:
        value = _number(value)
        fields[key] = {'value': value, 'unit': unit,
                       'status': 'calculated' if value is not None else 'unavailable',
                       'confidence': confidence if value is not None else None,
                       'source': source if value is not None else None,
                       'reason': None if value is not None else unavailable}
        return value

    empty = load_status.status == 'Empty'
    inner = split.boundaries_are_inner_faces
    length = split.longitudinal_measurement
    width = dimensions.width_measurement or {}
    # 载荷证据不足不应抹掉已由本次补全结构量出的尺寸；它们只作逐项
    # 诊断输出，不会因本标志生成确认空车的正式入库载荷。
    completed_candidate = (
        load_status.status == 'Uncertain' and inner
        and ((length is not None and length.valid
              and length.metadata.get('completionUsed') is True)
             or width.get('completionUsed') is True)
    )
    structural_measurement = empty or completed_candidate
    measured_length = (length.length if structural_measurement and length is not None and length.valid
                       and (inner or split.observed_outer_box is not None
                            or split.continuous_body_range) else None)
    measured_width = (width.get('widthM') if width.get('valid') is True
                      and width.get('adoptedSource') in {
                          'observed_inner_side_faces',
                          'sparse_rectangular_observed_inner_faces',
                          'completed_cloud_inner_side_faces'} else None)
    measured_height = (dimensions.carriage_height
                       if structural_measurement and (split.observed_outer_box is not None
                                     or getattr(profile, 'valid', False)) else None)
    boundary_reason = ('current_capture_not_confirmed_empty' if not empty else
                       'inner_wall_boundaries_unverified' if not inner else
                       'lower_inner_edges_unavailable')
    container = split.observed_outer_box is not None
    vehicle_source = ('completed_cloud_vehicle_envelope' if split.observed_outer_box is not None
                      else 'observed_vehicle_extent')
    vehicle = {name: field(f'dimensions.vehicle.{name}', value,
                           vehicle_source if container else (
                               'observed_vertical_span_not_ground_height' if name == 'height'
                               else 'observed_vehicle_extent'),
                           'vehicle_extent_unavailable')
               for name, value in dimensions.to_dict()['vehicle'].items()}
    carriage = {
        'length': field('dimensions.carriage.length', measured_length,
                        (length_metadata or {}).get('source'), boundary_reason,
                        confidence='low' if (split.observed_outer_box is not None
                                             or completed_candidate) else 'medium'),
        'width': field('dimensions.carriage.width', measured_width,
                       width.get('adoptedSource'), 'inner_width_unavailable',
                       confidence='low' if (width.get('completionUsed')
                                            or width.get('sparseRectangularInference'))
                       else 'medium'),
        'height': field('dimensions.carriage.height', measured_height,
                        'completed_cloud_floor_to_side_top' if split.observed_outer_box is not None
                        else 'current_completed_carriage_wall_depth'
                        if completed_candidate else 'current_empty_carriage_wall_depth',
                        (dimensions.height_measurement or {}).get('reason')
                        or 'empty_floor_or_wall_height_unavailable',
                        confidence='low' if completed_candidate else 'medium'),
    }
    cab = (split.cab_to_front_wall
           if (inner or container) and split.cab_measurement_valid else None)
    # 集装箱车头距量到前端水槽净空面；长车仍是 1.40m 截面到前内壁。
    cab_source = ('container_front_gutter_clearance' if container
                  else 'observed_ground_height_slice')
    can_show_floor = empty and floor_profile is not None
    floor_values = floor_dimensions if can_show_floor else {}
    floor_source = ('current_observed_and_completed_front_floor'
                    if split.observed_outer_box is not None else 'observed_floor_profile')
    structures_measured = empty and inner
    result = {
        'partial': True,
        'unit': 'm', 'reason': reason, 'dimensions': {'vehicle': vehicle, 'carriage': carriage},
        'cabToTrailerFrontDist': field('cabToTrailerFrontDist', cab,
                                      cab_source,
                                      split.cab_measurement_reason or 'front_inner_wall_unavailable'),
        'cabMeasurementHeight': split.cab_measurement_height,
        'trailerBottomToGroundHeight': field(
            'trailerBottomToGroundHeight',
            floor_values['trailerBottomToGroundHeight'] / 100
            if floor_values.get('trailerBottomToGroundHeight') is not None else None,
            floor_source, 'floor_profile_unavailable',
            confidence='low' if split.observed_outer_box is not None else 'medium'),
        'highLowPlateHeight': field(
            'highLowPlateHeight', floor_values['highLowPlateHeight'] / 100
            if floor_values.get('highLowPlateHeight') is not None else None,
            floor_source, 'floor_profile_unavailable',
            confidence='low' if split.observed_outer_box is not None else 'medium'),
        'floorProfile': floor_profile.to_dict() if can_show_floor else None,
        'highLowPlates': [dict(row, highestToGround=row['highestToGround'] / 100,
                              lowestToGround=row['lowestToGround'] / 100)
                          for row in high_low_plates] if can_show_floor else [],
        'ribs': [rib.to_dict() for rib in ribs] if structures_measured else [],
        'ribCount': field('ribCount', len(ribs) if structures_measured else None,
                          'current_observed_rib_detection', 'rib_detection_not_performed', 'count'),
        'ribIntervals': (length_metadata or {}).get('ribIntervalsM') if structures_measured else None,
        'wires': [],
        'wireCount': field('wireCount', None, None, 'wire_detector_not_implemented', 'count'),
        'referenceDimensions': {'carriage': {
            'length': field('referenceDimensions.carriage.length',
                            split.observed_outer_box.length if split.observed_outer_box is not None
                            else split.split_y - split.carriage_start_y,
                            'observed_rectangular_outer_length_not_inner_length'
                            if split.observed_outer_box is not None
                            else 'structural_boundary_span_not_verified_inner_length',
                            'structural_span_unavailable'),
            'width': field('referenceDimensions.carriage.width',
                           split.observed_outer_box.width if split.observed_outer_box is not None
                           else dimensions.carriage_outer_width,
                           'observed_rectangular_outer_width_not_inner_width'
                           if split.observed_outer_box is not None
                           else 'observed_outer_width_not_inner_width', 'outer_width_unavailable'),
        }},
        'fields': fields,
        'note': '逐项当前点云计算值；缺测保持空。参考分割跨度不是已确认内长，整车可见高度不是离地总高。',
    }
    result['availableFieldCount'] = sum(item['value'] is not None for item in fields.values())
    result['calculationSucceeded'] = result['availableFieldCount'] > 0
    if split.observed_outer_box is not None:
        result['referenceDimensions']['carriage']['height'] = field(
            'referenceDimensions.carriage.height', split.observed_outer_box.height,
            'observed_rectangular_outer_height_not_inner_height',
            'outer_height_unavailable')
        result['observedOuterBox'] = split.observed_outer_box.to_dict()
        upper = ((length.metadata.get('upperReference') or {}).get('endpoints', {})
                 if length is not None else {})
        upper_rear, upper_front = upper.get('rear', {}), upper.get('front', {})
        upper_length = (
            upper_front.get('positionM') - upper_rear.get('positionM')
            if upper_rear.get('valid') and upper_front.get('valid') else None
        )
        if (upper_length is not None and 0 < upper_length
                <= split.observed_outer_box.length + BOX_INNER_OUTER_TOLERANCE_M):
            result['estimatedCarriageLength'] = {
                'value': round(upper_length, 4), 'unit': 'm', 'confidence': 'low',
                'source': 'same_side_upper_reference_not_lower_inner_length',
                'businessInnerLength': False,
                'note': '上部同侧邻近点估算，仅供诊断；下部内缘未验证，不写入内长。',
            }
        if carriage['length'] is None:
            result['note'] += ' 矩形箱体外包络已识别，内长缺测时不以外长或车架跨度替代。'
        elif split.observed_outer_box is not None:
            result['note'] += ' 车厢长度取主体侧壁前后端，宽度取对应位置两侧上沿间距。'
        result['availableFieldCount'] = sum(item['value'] is not None for item in fields.values())
    return result
