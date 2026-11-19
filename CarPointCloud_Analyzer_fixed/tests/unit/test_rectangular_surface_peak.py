"""精确平面和缺失主体壁面的矩形尺寸回归。"""

import json

import numpy as np
import pytest

from carpointcloud_analyzer.domain.models import BoxCarriageBox
from carpointcloud_analyzer.domain.rectangular_completion import (
    _face_peak, completed_inner_dimensions,
)


def _exact_box():
    longitudinal, height = np.meshgrid(np.linspace(.6, 5.4, 100),
                                       np.linspace(.1, 2.8, 60))
    sides = [np.c_[np.full(longitudinal.size, width),
                   longitudinal.ravel(), height.ravel()] for width in (-1.2, 1.2)]
    width, height = np.meshgrid(np.linspace(-.9, .9, 50), np.linspace(.1, 2.8, 60))
    ends = [np.c_[width.ravel(), np.full(width.size, position), height.ravel()]
            for position in (0., 6.)]
    width, longitudinal = np.meshgrid(np.linspace(-.9, .9, 50),
                                      np.linspace(.6, 5.4, 100))
    floor = np.c_[width.ravel(), longitudinal.ravel(), np.full(width.size, .1)]
    return np.vstack((*sides, *ends, floor))


@pytest.mark.parametrize('coordinate', [0., .1, -1.2, 1.2])
def test_exact_planar_coordinate_has_a_finite_peak(coordinate):
    assert _face_peak(np.full(100, coordinate)) == coordinate


def test_final_box_dimensions_accept_exact_planar_end_faces():
    observed = _exact_box()
    synthetic = np.array([[-1.2, 3., 2.8]])
    box = BoxCarriageBox(6., 0., (-1.2, 1.2), (0., 2.8), 3000, 3000)
    dimensions = completed_inner_dimensions(observed, synthetic, box, .1, 2.8)
    assert dimensions is not None
    json.dumps(dimensions, allow_nan=False)
    assert dimensions['lengthGutterClearanceM'] == pytest.approx(6.)
    assert dimensions['widthGutterClearanceM'] == pytest.approx(2.4)
    # 与普通车厢统一：原 60 行等密度壁面按 P98 取次高行，再汇总左右有效段。
    # 这是替换短箱专用 P99 的口径断言，仍用原 pytest.approx 精度。
    expected = round(np.linspace(.1, 2.8, 60)[-2] - .1, 4)
    assert dimensions['heightM'] == pytest.approx(expected)


def test_top_rim_alone_cannot_supply_missing_main_end_face():
    observed = _exact_box()
    observed = observed[~((observed[:, 1] == 0.) & (observed[:, 2] < 2.45))]
    box = BoxCarriageBox(6., 0., (-1.2, 1.2), (0., 2.8), 3000, 3000)
    assert completed_inner_dimensions(
        observed, np.array([[-1.2, 3., 2.8]]), box, .1, 2.8,
    ) is None
