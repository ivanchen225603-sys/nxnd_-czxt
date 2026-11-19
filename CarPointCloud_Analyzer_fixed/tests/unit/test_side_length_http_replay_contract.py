"""离线验收工具核对直接入库的厘米尺寸和可空逐项结果。"""

from copy import deepcopy

import pytest

from tools.verify_pair_http import validate_dimension_output


def body():
    return {'outputs': {
        'dimensions': {'carriage': {'length': 9.877, 'width': 2.4, 'height': 1.8}},
        'measurementStatus': {'valid': True, 'calculationSucceeded': True},
    }, 'integrationPayloads': {'vehicleDimensions': {
        'detectStatus': 'success',
        'dimensions': {'trailerLength': 987.7, 'trailerWidth': 240.0,
                       'trailerBodyHeight': 180.0},
    }}}


def test_complete_dimensions_need_no_review_marker():
    validate_dimension_output(body(), 'synthetic')


def test_partial_empty_dimensions_keep_missing_length_null():
    value = body()
    value['outputs']['measurementStatus'].update(valid=False, partial=True)
    value['outputs']['dimensions']['carriage']['length'] = None
    value['integrationPayloads']['vehicleDimensions']['dimensions']['trailerLength'] = None
    validate_dimension_output(value, 'partial')


@pytest.mark.parametrize('failure', ['payload', 'length', 'status', 'calculation', 'review', 'partial'])
def test_dimension_validation_rejects_lost_contract(failure):
    value = deepcopy(body())
    if failure == 'payload':
        value['integrationPayloads'] = {}
    elif failure == 'length':
        value['integrationPayloads']['vehicleDimensions']['dimensions']['trailerLength'] = 1200.
    elif failure == 'status':
        value['integrationPayloads']['vehicleDimensions']['detectStatus'] = 'low_confidence'
    elif failure == 'calculation':
        value['outputs']['measurementStatus']['calculationSucceeded'] = False
    elif failure == 'review':
        value['outputs']['measurementStatus']['reviewStatus'] = 'pending'
    elif failure == 'partial':
        value['outputs']['measurementStatus']['valid'] = False
    with pytest.raises(AssertionError):
        validate_dimension_output(value, 'synthetic')
