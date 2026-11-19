"""前端底板高度与坡面极值分别导出，不改变三段 H/L 契约。"""

import numpy as np
import pytest

from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.domain.floor import FloorProfile, FloorSegment


@pytest.mark.parametrize('samples,drop_cm', [
    ([2.5, 2.3, 2.2, 1.9], 60.0),
    ([1.9, 2.2, 2.3, 2.5], 60.0),
    ([2.5, 1.8, 2.3, 2.5], 70.0),
    ([2.0004, 2.2, 2.3, 2.5056], 50.5),
])
def test_slope_export_uses_all_observed_bins_without_changing_segments(samples, drop_cm):
    floor = FloorProfile(4, np.array([0, 0, 1, 0]), 'weighbridge_calibration',
                         segments=[FloorSegment(i + 1, 'slope', h, 3.0)
                                   for i, h in enumerate([2.3, 2.2, 2.3])],
                         profile_s=np.array([.1, 3., 6., 8.9]), profile_h=np.array(samples),
                         front_height_to_ground=samples[0])
    before = floor.to_dict()
    values, plates = ResultExporter._build_high_low_plate_payload(floor)
    assert values['trailerBottomToGroundHeight'] == round(samples[0] * 100, 1)
    assert values['highLowPlateHeight'] == drop_cm
    assert [(p['plateId'], p['lengthCm'], p['lowestToGround']) for p in plates] == [
        (1, 300.0, 230.0), (2, 300.0, 220.0), (3, 300.0, 230.0)
    ]
    assert floor.to_dict() == before


def test_slope_observed_extrema_cannot_bypass_physical_height_validation():
    floor = FloorProfile(4, np.array([0, 0, 1, 0]), 'weighbridge_calibration',
                         segments=[FloorSegment(1, 'slope', 2.0, 9.0)],
                         profile_s=np.arange(4.), profile_h=np.array([2., 2., -1., 2.]))
    values, plates = ResultExporter._build_high_low_plate_payload(floor)
    assert values['trailerBottomToGroundHeight'] is None
    assert plates == []
