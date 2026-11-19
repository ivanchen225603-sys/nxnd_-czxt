"""独立测量逐项入库，不把参考跨度或未执行项目变成业务尺寸。"""

from copy import deepcopy
from dataclasses import replace
import json
from types import SimpleNamespace

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.domain.models import (
    BoxCarriageBox, CarriageSplitResult, LoadStatusResult, LongitudinalMeasurement,
    VehicleDimensions,
)
from carpointcloud_analyzer.application.partial_measurements import build_partial_measurements


def _inputs():
    dimensions = VehicleDimensions(15.7, 2.49, 2.42, 11.60, 2.38, 1.80)
    dimensions.carriage_outer_width = 2.49
    dimensions.width_measurement = {
        'valid': True, 'adoptedSource': 'observed_inner_side_faces', 'widthM': 2.38,
    }
    split = CarriageSplitResult(
        np.zeros((5, 3)), 8.2, -3.4, 12.3, -3.4,
        boundaries_are_inner_faces=False, boundary_method='wall_core_fallback',
        cab_measurement_method='fixed_height', cab_measurement_height=1.4,
        cab_measurement_reason='head_slice_unavailable',
    )
    profile = SimpleNamespace(valid=True, wall_depth=1.8)
    return dimensions, split, profile


def _partial(*, status='Empty', inner=False, floor=False):
    dimensions, split, profile = _inputs()
    split.boundaries_are_inner_faces = inner
    floor_profile = SimpleNamespace(to_dict=lambda: {'case': 1}) if floor else None
    return build_partial_measurements(
        dimensions, split, LoadStatusResult(status, .67, 'test', {}), profile, floor_profile,
        {'trailerBottomToGroundHeight': 155., 'highLowPlateHeight': 0.}, [], [], None,
        'inner_wall_boundaries_unverified',
    )


def test_unverified_endpoints_keep_independent_width_height_and_outline():
    result = _partial()
    assert result['dimensions']['vehicle'] == {'length': 15.7, 'width': 2.49, 'height': 2.42}
    assert result['dimensions']['carriage'] == {'length': None, 'width': 2.38, 'height': 1.8}
    assert result['referenceDimensions']['carriage']['length'] == 11.6
    assert result['fields']['referenceDimensions.carriage.length']['source'] == (
        'structural_boundary_span_not_verified_inner_length')
    for key in ('cabToTrailerFrontDist', 'trailerBottomToGroundHeight', 'highLowPlateHeight',
                'ribCount', 'wireCount'):
        assert result[key] is None
        assert result['fields'][key]['status'] == 'unavailable'
        assert result['fields'][key]['reason']
    assert result['partial'] is True
    assert result['calculationSucceeded'] is True
    json.dumps(result, allow_nan=False)


def test_continuous_body_range_publishes_side_length_without_end_wall_fields():
    """端壁内表面未验证但车体连续时只开放侧壁内长，车头距/拉筋仍缺测。"""
    dimensions, split, profile = _inputs()
    split.continuous_body_range = True
    split.longitudinal_measurement = LongitudinalMeasurement(
        -3.4, 8.2, (0., 0., 1.),
        {'valid': True, 'source': 'current_final_carriage_cloud',
         'method': 'body_side_wall_longitudinal_extent'},
    )
    result = build_partial_measurements(
        dimensions, split, LoadStatusResult('Empty', .67, 'test', {}), profile, None,
        {'trailerBottomToGroundHeight': 155., 'highLowPlateHeight': 0.}, [], [], None,
        'inner_wall_boundaries_unverified',
    )
    assert result['dimensions']['carriage'] == {'length': 11.6, 'width': 2.38, 'height': 1.8}
    assert result['fields']['dimensions.carriage.length']['status'] != 'unavailable'
    for key in ('cabToTrailerFrontDist', 'ribCount', 'wireCount'):
        assert result[key] is None
    without = _partial()
    assert without['dimensions']['carriage']['length'] is None


@pytest.mark.parametrize('status', ['Loaded', 'Uncertain'])
def test_partial_result_does_not_override_load_guard(status):
    result = _partial(status=status, inner=True, floor=True)
    assert result['dimensions']['carriage'] == {'length': None, 'width': 2.38, 'height': None}
    assert result['floorProfile'] is None
    assert result['trailerBottomToGroundHeight'] is None
    assert result['ribCount'] is None
    assert result['highLowPlateHeight'] is None
    assert result['fields']['dimensions.carriage.length']['reason'] == 'current_capture_not_confirmed_empty'


def test_uncertain_load_persists_calculated_structure_without_review():
    dimensions, split, profile = _inputs()
    split.boundaries_are_inner_faces = True
    split.longitudinal_measurement = LongitudinalMeasurement(
        -3.4, 8.2, (0., 0., 1.),
        {'valid': True, 'completionUsed': True,
         'definition': 'same_side_lower_band_longitudinal_projection',
         'source': 'current_measured_and_completed_carriage_cloud'},
    )
    dimensions.width_measurement = {
        'valid': True, 'widthM': 2.38, 'completionUsed': True,
        'adoptedSource': 'completed_cloud_inner_side_faces',
    }
    result = ResultExporter.build(
        [], dimensions, split, LoadStatusResult('Uncertain', .2,
                                               'carriage_interior_grid_insufficient', {}),
        'partial.pcd', Config(), profile=profile,
        wall_completion={'completed': True, 'synthesizedPoints': 100},
    )
    diagnostic = result['reviewMeasurements']
    assert diagnostic['dimensions']['carriage'] == {
        'length': pytest.approx(11.6), 'width': pytest.approx(2.38), 'height': 1.8}
    assert 'confidence' not in diagnostic['fields']['dimensions.carriage.length']
    assert 'source' not in diagnostic['fields']['dimensions.carriage.width']
    payload = result['integrationPayloads']['vehicleDimensions']
    assert payload['detectStatus'] == 'success'
    assert payload['dimensions']['trailerLength'] == pytest.approx(1160.)
    assert payload['dimensions']['trailerWidth'] == pytest.approx(238.)
    assert payload['dimensions']['trailerBodyHeight'] == pytest.approx(180.)
    assert result['dimensions'] == diagnostic['dimensions']
    assert result['measurementStatus']['calculationSucceeded'] is True
    assert 'reviewStatus' not in result['measurementStatus']


def test_real_zero_plate_difference_is_distinct_from_unexecuted_measurement():
    result = _partial(inner=True, floor=True)
    assert result['trailerBottomToGroundHeight'] == 1.55
    assert result['highLowPlateHeight'] == 0.
    assert result['ribCount'] == 0
    assert result['wireCount'] is None
    assert result['fields']['highLowPlateHeight']['status'] == 'calculated'


@pytest.mark.parametrize('width', [None, float('nan'), float('inf'), -2.3, True, '2.3'])
def test_invalid_numeric_width_is_not_exported(width):
    dimensions, split, profile = _inputs()
    dimensions.width_measurement['widthM'] = width
    result = build_partial_measurements(
        dimensions, split, LoadStatusResult('Empty', .67, 'test', {}), profile, None,
        {}, [], [], None, 'test',
    )
    assert result['dimensions']['carriage']['width'] is None
    json.dumps(result, allow_nan=False)


def test_width_without_measured_side_face_evidence_remains_unknown():
    dimensions, split, profile = _inputs()
    dimensions.width_measurement['adoptedSource'] = 'outer_wall_fallback'
    result = build_partial_measurements(
        dimensions, split, LoadStatusResult('Empty', .67, 'test', {}), profile, None,
        {}, [], [], None, 'test',
    )
    assert result['dimensions']['carriage']['width'] is None


def _container_inputs():
    dimensions, split, profile = _inputs()
    split.observed_outer_box = BoxCarriageBox(
        5.85, .15, (-1.2, 1.2), (0., 2.5), 5000, 5000)
    return dimensions, split, profile


def test_container_dimensions_publish_measured_side_length_and_rim_width():
    """短箱与普通车型共用独立长宽结果。"""
    dimensions, split, profile = _container_inputs()
    split.longitudinal_measurement = LongitudinalMeasurement(
        .15, 5.85, (0., 0., -1.2), {'definition': 'main_side_wall_front_to_rear_length'})
    result = build_partial_measurements(
        dimensions, split, LoadStatusResult('Empty', .67, 'test', {}), profile, None,
        {'trailerBottomToGroundHeight': 155., 'highLowPlateHeight': 0.}, [], [], None,
        'inner_wall_boundaries_unverified',
    )
    assert result['dimensions']['carriage']['length'] == pytest.approx(5.7)
    assert result['dimensions']['carriage']['width'] == 2.38
    assert result['fields']['dimensions.carriage.width']['source'] == 'observed_inner_side_faces'
    json.dumps(result, allow_nan=False)


def test_container_missing_side_length_stays_unknown():
    """没有侧壁长度时保持缺测；已有上沿宽度独立输出。"""
    dimensions, split, profile = _container_inputs()
    result = build_partial_measurements(
        dimensions, split, LoadStatusResult('Empty', .67, 'test', {}), profile, None,
        {}, [], [], None, 'test',
    )
    assert result['dimensions']['carriage']['length'] is None
    assert result['dimensions']['carriage']['width'] == 2.38


def test_container_cab_distance_uses_gutter_clearance_source():
    """集装箱车头距（量到前端水槽净空面）随逐项结果输出。"""
    dimensions, split, profile = _container_inputs()
    split.cab_measurement_height = 1.4
    split.cab_measurement_method = 'ground_height_slice_percentile'
    split.cab_measurement_head_y = 3.0
    split.cab_measurement_front_y = -0.53
    split.cab_measurement_front_source = 'container_front_gutter_clearance'
    split.cab_measurement_point_count = 120
    result = build_partial_measurements(
        dimensions, split, LoadStatusResult('Empty', .67, 'test', {}), profile, None,
        {}, [], [], None, 'test',
    )
    assert result['cabToTrailerFrontDist'] == pytest.approx(3.53)
    assert result['fields']['cabToTrailerFrontDist']['source'] == (
        'container_front_gutter_clearance')
    json.dumps(result, allow_nan=False)


def test_long_vehicle_preserves_independent_width_when_length_missing():
    """普通长车缺测内长时，已有上沿宽度仍独立输出。"""
    dimensions, split, profile = _inputs()
    result = build_partial_measurements(
        dimensions, split, LoadStatusResult('Empty', .67, 'test', {}), profile, None,
        {}, [], [], None, 'test',
    )
    assert result['dimensions']['carriage']['length'] is None
    assert result['dimensions']['carriage']['width'] == 2.38


def test_exporter_preserves_business_guard_and_does_not_mutate_observations():
    dimensions, split, profile = _inputs()
    original = deepcopy(dimensions.to_dict())
    for status in ('Empty', 'Loaded', 'Uncertain'):
        result = ResultExporter.build(
            [], dimensions, split, LoadStatusResult(status, .67, 'test', {}),
            'partial.pcd', Config(), profile=profile,
        )
        if status != 'Loaded':
            assert result['dimensions']['carriage'] == {
                'length': None, 'width': 2.38, 'height': 1.8 if status == 'Empty' else None}
            payload = result['integrationPayloads']['vehicleDimensions']
            assert payload['detectStatus'] == 'success'
            assert payload['dimensions']['trailerLength'] is None
            assert payload['dimensions']['trailerWidth'] == 238.0
        else:
            assert result['dimensions'] is None
            assert result['integrationPayloads'] == {}
        assert result['measurementStatus']['valid'] is False
        assert result['measurementStatus']['calculationSucceeded'] is (status != 'Loaded')
        assert result['reviewMeasurements']['partial'] is True
        assert result['loadStatus']['status'] == status
        assert dimensions.to_dict() == original


def test_valid_length_survives_independent_cab_failure():
    dimensions, split, profile = _inputs()
    split = replace(split, boundaries_are_inner_faces=True,
                    longitudinal_measurement=LongitudinalMeasurement(
                        -3.4, 8.2, (0., 0., 1.), {'definition': 'test', 'valid': True}))
    result = ResultExporter.build(
        [], dimensions, split, LoadStatusResult('Empty', .67, 'test', {}),
        'partial.pcd', Config(), profile=profile,
    )
    assert result['dimensions']['carriage']['length'] == pytest.approx(11.6)
    assert result['integrationPayloads']['vehicleDimensions']['dimensions']['cabToTrailerFrontDist'] is None
    assert result['reviewMeasurements']['dimensions']['carriage']['length'] == pytest.approx(11.6)
    assert result['reviewMeasurements']['cabToTrailerFrontDist'] is None
