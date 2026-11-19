"""矩形车型结果契约组装；只映射已计算数据，不执行测量或文件/数据库写入。"""

from typing import Any

from carpointcloud_analyzer.domain.carriage import BoxCompletionResult
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.application.review_measurements import build_review_dimension_payload
from carpointcloud_analyzer.domain.site_frame import SiteFrame


def build_rectangular_result(
    review: dict[str, Any], reference: BoxCompletionResult,
    reference_transform: CoordinateTransform, site_frame: SiteFrame | None,
    status: dict[str, Any], config: Config, pcd_path: str,
    pcd_out_path: str, output_count: int,
    *, exported_result: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """输出从最终补点云量取的尺寸及对应厘米载荷。"""
    if exported_result is None:
        # 兼容直接组装调用，但不绕过正式路径的空车/尺寸范围约束。
        payload = None
        reason = 'load_status_uncertain' if status.get('status') == 'Uncertain' else 'vehicle_not_empty'
        if status.get('status') == 'Empty':
            try:
                payload = build_review_dimension_payload(review, config, pcd_path)
                reason = 'ok'
            except ValueError:
                reason = 'dimensions_out_of_physical_range'
        exported_result = {
            'integrationPayloads': {'vehicleDimensions': payload} if payload else {},
            'dimensions': review['dimensions'] if payload else None,
            'measurementStatus': {'valid': payload is not None, 'reason': reason,
                                  'calculationSucceeded': payload is not None},
        }
    result = {
        **exported_result,
        'png': '', 'json': '', 'pcd': pcd_out_path, 'pcdPointCount': output_count,
        'syntheticPointCount': review['syntheticPointCount'],
        'cleanPointCloudPath': pcd_out_path or None,
        # 逐项缺测/越界结果由共享导出器决定，矩形诊断不能覆盖它。
        'reviewMeasurements': {**review, **exported_result.get('reviewMeasurements', {})},
        'measurementStatus': {
            **exported_result['measurementStatus'], 'unit': 'm',
            'dimensionSource': 'completed_rectangular_cloud',
            'lengthSource': review['fields']['dimensions.carriage.length'],
            'widthSource': review['fields']['dimensions.carriage.width'],
            'heightSource': review['fields']['dimensions.carriage.height'],
            'modelHeightSource': reference.metadata['heightSource'],
            'note': '尺寸从融合点云与本次补点组成的最终云图计算；缺测项保持空值。',
        },
        'loadStatus': status,
        'wallCompletion': review['wallCompletion'],
        'carriageObservation': {
            'algorithmBranch': 'rectangular_shell_observation',
            'referenceOnly': True,
            'scope': ('completed_rectangular_cloud'
                      if reference.metadata.get('symmetricOpening', {}).get('applied')
                      else 'observed_exterior'),
            'unit': 'm',
            'lengthM': reference.box.length, 'widthM': reference.box.width,
            'visibleHeightM': reference.metadata['observedHeightM'],
            'modelHeightM': reference.box.height,
            'interiorHeightM': (review.get('carriageHeight') or {}).get('innerHeightM'),
            'inferredOpening': review.get('carriageOpening'),
            'interiorDimensions': (
                review['dimensions']['carriage']
                if reference.metadata.get('symmetricOpening', {}).get('applied')
                else None),
            'floorToWeighbridgeHeightM': None,
            'syntheticPointsUsedForMeasurement': review['syntheticPointCount'],
            'coordinateFrame': 'leveled_scene' if site_frame is not None else 'input_pcd',
            'coordinateTransform': reference_transform.to_dict(),
            'referenceCompletion': reference.metadata,
        },
    }
    return ResultExporter._without_point_origin_labels(result)
