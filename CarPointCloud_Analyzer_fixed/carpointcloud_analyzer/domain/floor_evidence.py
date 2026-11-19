"""从本次最终云的内壁面支持查找候选底面矛盾，不读取历史基准。"""

import numpy as np

from carpointcloud_analyzer.domain.inner_faces import FACE_TOLERANCE_M, supported_faces


def lower_wall_floor_conflict(points, profile, config):
    """两侧同位置的二维内壁均延伸到候选面下方时，底板尚未确认。"""
    values = np.asarray(points, dtype=float)
    separation = max(4 * config.floor_layer_tolerance, config.carriage_grid_cell)
    band = max(config.floor_layer_tolerance, FACE_TOLERANCE_M)
    centers = (profile.interior_w_range[0] - config.rib_wall_band,
               profile.interior_w_range[1] + config.rib_wall_band)
    sides = []
    for name, center in zip(('left', 'right'), centers):
        lower = values[(np.abs(values[:, 0] - center) <= band)
                       & (values[:, 2] < profile.floor_z - separation)]
        rows = []
        # 复用内壁的二维面支持门槛，水平踏板、孤立立柱或斜杆不能代替墙面。
        for face in supported_faces(lower, 0):
            middle = np.median(face.cells[:, [1, 2]], axis=0)
            if abs(face.at(*middle) - center) <= band:
                rows.append({'cells': len(face.cells),
                             'longitudinalSpan': float(np.ptp(face.cells[:, 1])),
                             'heightSpan': float(np.ptp(face.cells[:, 2]))})
        sides.append({'side': name, 'wallW': float(center),
                      'lowerPoints': len(lower), 'supportedFaces': rows})
    return {'conflict': all(bool(side['supportedFaces']) for side in sides),
            'candidateFloorH': float(profile.floor_z), 'separationM': float(separation),
            'sides': sides}
