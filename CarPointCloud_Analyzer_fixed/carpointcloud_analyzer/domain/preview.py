"""只读补点传输契约：A 坐标系、米制、有限输入，不接受身份和输出路径。"""

import base64
import binascii

import numpy as np

MAX_PREVIEW_POINTS = 60000


def decode_preview(payload):
    """把只读预览载荷解成 A 坐标系米制点。

    Raises:
        ValueError: 编码不受支持、点数与字节数不符、超出上限，或坐标非有限/超出场地范围。
    """
    if payload.pointsEncoding != "float32le-xyz-base64":
        raise ValueError("unsupported preview encoding")
    try:
        data = base64.b64decode(payload.points, validate=True)
    except (ValueError, binascii.Error) as exc:
        raise ValueError("invalid preview encoding") from exc
    count = payload.pointCountA + payload.pointCountB
    if len(data) != count * 12 or count > MAX_PREVIEW_POINTS:
        raise ValueError("preview point count mismatch or limit exceeded")
    points = np.frombuffer(data, dtype="<f4").reshape(-1, 3)
    if not np.isfinite(points).all() or np.any(np.abs(points) > 1000):
        raise ValueError("preview requires finite meter coordinates within site bounds")
    return points.astype(np.float64)


def encode_preview(result):
    """把预览结果里的坐标编码回传输格式，并标记为仅供参考。

    Raises:
        ValueError: 坐标非有限或超过点数上限。
    """
    points = np.asarray(result.pop("coordinates"), dtype="<f4")
    if not np.isfinite(points).all() or len(points) > MAX_PREVIEW_POINTS:
        raise ValueError("invalid preview output")
    return {
        **result,
        "points": base64.b64encode(points.tobytes()).decode("ascii"),
        "pointsEncoding": "float32le-xyz-base64",
        "referenceOnly": True,
    }
