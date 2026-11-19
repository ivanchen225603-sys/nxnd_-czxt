"""车厢点云分析：api（HTTP/CLI）→ application（编排）→ domain（算法）← infrastructure（文件、DM8）。"""
from .application.pipeline import PointCloudAnalyzer
from .domain.config import Config
from .domain.models import CarriageSplitResult, LoadStatusResult, RibInfo, VehicleDimensions

__all__ = [
    "Config",
    "PointCloudAnalyzer",
    "RibInfo",
    "VehicleDimensions",
    "CarriageSplitResult",
    "LoadStatusResult",
]
