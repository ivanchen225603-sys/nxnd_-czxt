"""显式、不可变的体积条件目录；配置声明不等于现场认证，不选择或激活基线。"""
from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Optional, Tuple

from .archive_io import _read_regular
from .bed_reference import _digest, _label, baseline_digest, surface_digest, valid_surface_scope
from .volume_context import VolumeContext, context_error, grid_geometry


MAX_PROVIDER_BYTES = 1024 * 1024
MAX_MODELS = 128
BINDING_FIELDS = frozenset(("node_id", "channel_id", "calibration_id", "configuration_sha256",
                            "baseline_version_id", "baseline_asset_sha256", "baseline_sha256"))
MODEL_FIELDS = frozenset(("model_id", "binding", "reference_id", "coordinate_frame", "valid_from", "valid_until",
                          "domain_xy", "domain_source", "max_height_m", "height_source", "support_cell_m",
                          "support_origin_xy", "support_model", "error_source", "surface_error_m", "bed_error_m",
                          "roi_error_area_m2", "conditions_status", "registration"))


def _canonical(value) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def _keys(value, expected, name):
    if type(value) is not dict or set(value) != set(expected):
        raise ValueError(name + "字段缺失或含未知字段")


def _number(value, *, positive=False):
    try:
        return (type(value) in (int, float) and math.isfinite(value) and
                (value > 0 if positive else value >= 0))
    except OverflowError:
        return False


def _binding(value):
    _keys(value, BINDING_FIELDS, "体积模型绑定")
    if any(not (_digest(item) if key.endswith("sha256") else _label(item)) for key, item in value.items()):
        raise ValueError("体积模型绑定标识或摘要非法")


def _decode(data: bytes):
    if not data or len(data) > MAX_PROVIDER_BYTES or b"\x00" in data or data.startswith(b"\xef\xbb\xbf"):
        raise ValueError("体积参数文件必须为预算内无BOM/NUL的UTF-8")
    text = data.decode("utf-8", "strict")
    depth = nodes = numeric = 0
    quoted = escaped = False
    for byte in data:
        numeric = numeric + 1 if not quoted and byte in b"0123456789+-.eE" else 0
        if numeric > 80:
            raise ValueError("体积参数数字超过预算")
        if quoted:
            if escaped:
                escaped = False
            elif byte == 92:
                escaped = True
            elif byte == 34:
                quoted = False
        elif byte == 34:
            quoted = True
        elif byte in (91, 123):
            depth += 1
            nodes += 1
        elif byte in (93, 125):
            depth -= 1
        elif byte in (44, 58):
            nodes += 1
        if depth > 16 or nodes > 20000:
            raise ValueError("体积参数结构超过预算")
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("体积参数含重复字段")
            result[key] = value
        return result
    def reject_constant(value):
        raise ValueError("体积参数含非有限常量")
    def bounded_float(value):
        number = float(value)
        if not math.isfinite(number) or (number == 0. and Decimal(value) != 0):
            raise ValueError("体积参数数字上溢或非零值下溢")
        return number
    return json.loads(text, object_pairs_hook=unique, parse_constant=reject_constant, parse_float=bounded_float)


def _validate_model(value):
    _keys(value, MODEL_FIELDS, "体积模型")
    _binding(value["binding"])
    if not all(_label(value[key]) for key in ("model_id", "reference_id")):
        raise ValueError("体积模型标识非法")
    enums = {"coordinate_frame": {"baseline-heading-aligned-m-v1"},
             "domain_source": {"synthetic_model", "surveyed_interior"},
             "height_source": {"synthetic_model", "independent_vehicle_envelope"},
             "support_model": {"cell_mean_max_relative_height"},
             "error_source": {"synthetic_model", "independent_error_budget"},
             "conditions_status": {"declared_model"}}
    for key, allowed in enums.items():
        if type(value[key]) is not str or value[key] not in allowed:
            raise ValueError("体积模型来源/坐标系/支持声明非法")
    for key, size in (("domain_xy", 4), ("support_origin_xy", 2)):
        numbers = value[key]
        if type(numbers) is not list or len(numbers) != size or any(
                type(v) not in (int, float) or not _number(abs(v)) for v in numbers):
            raise ValueError("体积模型域或原点非法")
    if any(not _number(value[key], positive=True) for key in ("valid_from", "valid_until", "max_height_m", "support_cell_m")):
        raise ValueError("体积模型尺度/有效期非法")
    if value["valid_until"] < value["valid_from"] or any(not _number(value[key]) for key in
            ("surface_error_m", "bed_error_m", "roi_error_area_m2")):
        raise ValueError("体积模型有效期或误差预算非法")
    x0, x1, y0, y1 = map(float, value["domain_xy"])
    cell = float(value["support_cell_m"])
    if not (x0 < x1 and y0 < y1):
        raise ValueError("体积模型内腔域为空")
    area = (x1 - x0) * (y1 - y0)
    grid = ((x1 - x0) / cell + 2) * ((y1 - y0) / cell + 2)
    derived = (area, grid, area * value["max_height_m"],
               area * (value["surface_error_m"] + value["bed_error_m"]),
               value["roi_error_area_m2"] * value["max_height_m"],
               (area + value["roi_error_area_m2"]) * value["max_height_m"])
    if any(not _number(v) for v in derived) or grid > 1_000_000 or area <= 0:
        raise ValueError("体积模型派生尺度/预算溢出或网格过大")
    grid_geometry(value["domain_xy"], cell, value["support_origin_xy"])
    registration = value["registration"]
    if type(registration) is not dict or type(registration.get("mode")) is not str:
        raise ValueError("体积模型必须显式声明配准策略")
    if registration["mode"] == "estimated":
        _keys(registration, {"mode"}, "估计配准")
    elif registration["mode"] == "fixed":
        _keys(registration, {"mode", "dy_m"}, "固定配准")
        if type(registration["dy_m"]) not in (int, float) or not _number(abs(registration["dy_m"])):
            raise ValueError("固定配准偏移必须为有限实数")
        dy = registration["dy_m"]
        ox, oy = value["support_origin_xy"]
        shifted = (x0, x1, y0 + dy, y1 + dy)
        grid_geometry(shifted, cell, (ox, oy + dy))
        if not math.isclose(shifted[3] - shifted[2], y1 - y0, rel_tol=1e-12, abs_tol=0.):
            raise ValueError("固定配准使物理域宽度失去数值精度")
        shifted_area = (x1 - x0) * (shifted[3] - shifted[2])
        if not all(_number(v) for v in (
                (shifted_area + value["roi_error_area_m2"]) * value["max_height_m"],
                shifted_area * (value["surface_error_m"] + value["bed_error_m"]))):
            raise ValueError("固定配准后的派生预算溢出")
    else:
        raise ValueError("未知配准策略")


@dataclass(frozen=True)
class VolumeModel:
    canonical_json: bytes

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.canonical_json).hexdigest()

    def data(self) -> dict:
        return json.loads(self.canonical_json)

    def supply(self, *, scope, baseline, surfaces, dy: float, registration_status: str) -> VolumeContext:
        model = self.data()
        binding = model["binding"]
        if (not valid_surface_scope(scope, len(surfaces)) or
                any(getattr(scope, key) != binding[key] for key in
                    ("baseline_version_id", "calibration_id", "configuration_sha256", "channel_id")) or
                scope.coordinate_frame != model["coordinate_frame"] or
                baseline_digest(baseline) != binding["baseline_sha256"]):
            raise ValueError("体积条件与实际基线或运行身份不匹配")
        registration = model["registration"]
        if type(dy) not in (int, float) or not _number(abs(dy)) or (registration["mode"] == "estimated" and registration_status != "resolved"):
            raise ValueError("体积条件缺少已解决配准")
        if registration["mode"] == "fixed" and (registration_status != "declared_fixed" or dy != registration["dy_m"]):
            raise ValueError("体积条件与显式固定配准不一致")
        x0, x1, y0, y1 = model["domain_xy"]
        ox, oy = model["support_origin_xy"]
        context = VolumeContext(scope=scope, reference_id=model["reference_id"],
            baseline_sha256=binding["baseline_sha256"], surface_sha256=surface_digest(surfaces),
            domain_xy=(x0, x1, y0 + dy, y1 + dy), domain_source=model["domain_source"],
            max_height_m=model["max_height_m"], height_source=model["height_source"],
            conditions_status=model["conditions_status"], support_cell_m=model["support_cell_m"],
            support_origin_xy=(ox, oy + dy), support_model=model["support_model"],
            error_source=model["error_source"], surface_error_m=model["surface_error_m"],
            bed_error_m=model["bed_error_m"], roi_error_area_m2=model["roi_error_area_m2"],
            registration_dy_m=dy, valid_from=model["valid_from"], valid_until=model["valid_until"])
        error = context_error(context, scope, baseline, surfaces, dy)
        if error is not None:
            raise ValueError(error)
        return context


@dataclass(frozen=True)
class VolumeCatalog:
    models: Tuple[VolumeModel, ...]
    source_sha256: str

    @classmethod
    def from_bytes(cls, data: bytes) -> "VolumeCatalog":
        value = _decode(data)
        _keys(value, {"schema", "models"}, "体积参数目录")
        if value["schema"] != "volume-provider-v1" or type(value["models"]) is not list or len(value["models"]) > MAX_MODELS:
            raise ValueError("体积参数schema或模型数量非法")
        models, seen = [], set()
        for item in value["models"]:
            _validate_model(item)
            if item["model_id"] in seen:
                raise ValueError("体积模型标识重复")
            seen.add(item["model_id"])
            for previous in models:
                other = previous.data()
                if (other["binding"] == item["binding"] and max(other["valid_from"], item["valid_from"]) <=
                        min(other["valid_until"], item["valid_until"])):
                    raise ValueError("相同绑定的体积模型有效期重叠")
            models.append(VolumeModel(_canonical(item)))
        return cls(tuple(models), hashlib.sha256(data).hexdigest())

    @classmethod
    def load(cls, path: str) -> "VolumeCatalog":
        if not isinstance(path, str) or not path.strip():
            raise ValueError("必须显式提供体积参数文件")
        return cls.from_bytes(_read_regular(Path(os.path.abspath(path)), MAX_PROVIDER_BYTES))

    def select(self, binding: dict, measured_at: float) -> Tuple[Optional[VolumeModel], str]:
        _binding(binding)
        if not _number(measured_at, positive=True):
            raise ValueError("体积测量时间非法")
        matches = [model for model in self.models if model.data()["binding"] == binding]
        active = [model for model in matches if model.data()["valid_from"] <= measured_at <= model.data()["valid_until"]]
        if len(active) > 1:
            raise ValueError("体积模型绑定歧义")
        return (active[0], "matched") if active else (None, "expired" if matches else "binding_not_found")
