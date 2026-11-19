import base64
import asyncio
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from fastapi.testclient import TestClient

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.application.pipeline import PointCloudAnalyzer
from carpointcloud_analyzer.domain.preview import decode_preview
from carpointcloud_analyzer.composition import create_app
from carpointcloud_analyzer.application.service import AnalysisService


def payload(points=None):
    if points is None:
        points = np.zeros((2, 3), dtype="<f4")
    return dict(points=base64.b64encode(points.tobytes()).decode(),
                pointsEncoding="float32le-xyz-base64", pointCountA=1, pointCountB=1)


@pytest.mark.parametrize("change", [
    {"points": "!invalid!"}, {"pointCountA": 3}, {"pointsEncoding": "float64"},
    {"points": base64.b64encode(np.full((2, 3), np.nan, dtype="<f4").tobytes()).decode()},
])
def test_preview_rejects_invalid_data(change):
    with pytest.raises(ValueError):
        decode_preview(SimpleNamespace(**(payload() | change)))


def test_preview_http_forbids_business_fields_and_requires_calibration(tmp_path):
    with TestClient(create_app()) as client:
        for extra in ["plate", "workflowId", "pcdPath", "baselineDir"]:
            assert client.post("/preview-completion", json=payload() | {extra: "x"}).status_code == 422
        assert client.post("/preview-completion", json=payload() | {"pointCountA": True}).status_code == 422
        assert client.post("/preview-completion", json=payload()).status_code == 400


def test_real_memory_preview_has_no_business_or_file_output(monkeypatch):
    from test_site_leveling import _weighbridge_scene

    config = Config(input_unit="m", license_plate="", save_clean_pointcloud=False,
                    weighbridge_plane_source=(0., 0., 1., 0.), scene_preprocessing_fail_open=False)
    analyzer = PointCloudAnalyzer(config)
    forbidden = Mock(side_effect=AssertionError("preview attempted file/business output"))
    monkeypatch.setattr(analyzer.loader, "load", forbidden)
    monkeypatch.setattr(analyzer, "_save_clean_pointcloud", forbidden)
    monkeypatch.setattr("carpointcloud_analyzer.application.pipeline.ResultExporter.build", forbidden)
    result = analyzer.analyze_preview(_weighbridge_scene(np.random.default_rng(20260905)))
    assert result["loadStatus"]["status"] == "Empty"
    assert result["measuredPointCount"] > 0
    assert result["syntheticPointCount"] > 0
    assert len(result["coordinates"]) == result["measuredPointCount"] + result["syntheticPointCount"]
    assert "dimensions" not in result and "integrationPayloads" not in result
    forbidden.assert_not_called()


def test_preview_refuses_identity_or_file_output():
    for config in [Config(license_plate="TEST"), Config(save_clean_pointcloud=True)]:
        with pytest.raises(ValueError, match="preview must disable"):
            PointCloudAnalyzer(config).analyze_preview(np.zeros((2, 3)))




def test_preview_runs_without_formal_request_cooldown():
    service = AnalysisService(max_concurrency=1)
    service.preview_sync = lambda payload: {'preview': payload}
    try:
        assert asyncio.run(service.preview('current')) == {'preview': 'current'}
    finally:
        service.shutdown()
