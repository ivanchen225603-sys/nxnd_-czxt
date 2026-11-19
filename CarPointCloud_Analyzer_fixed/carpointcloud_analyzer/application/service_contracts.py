"""框架无关的分析请求；字段校验只在 HTTP 请求模型入口做一次。"""

from dataclasses import dataclass
from datetime import datetime

MAX_BATCH_SIZE = 16


@dataclass(frozen=True)
class AnalyzeRequest:
    """单份 PCD 分析请求及其车辆、流程和输出选项。"""
    pcd_path: str
    camera: str = ""
    plate: str = ""
    input_unit: str = "auto"
    # 单雷达路径同样可能属于一次来车流程（双雷达其中一台超时后的兜底分析）。
    workflow_id: str | None = None
    captured_at: datetime | None = None


@dataclass(frozen=True)
class BatchAnalyzeRequest:
    """共享同一配置的一批独立 PCD 分析请求。"""
    pcd_paths: tuple[str, ...]
    camera: str = ""
    plate: str = ""
    input_unit: str = "auto"


@dataclass(frozen=True)
class PairAnalyzeRequest:
    """固定 A/B 语义的双雷达分析请求。

    ``pcd_path_a`` 是基准坐标系 A，``pcd_path_b`` 会按外参 B→A 变换。
    调用方不得按到达先后交换 A/B。
    """

    pcd_path_a: str
    camera_a: str
    pcd_path_b: str
    camera_b: str
    pair_id: str
    workflow_id: str
    # 服务端配置了 CARPOINT_EXTRINSIC_PATH 时以服务端为准，此字段不再生效。
    extrinsic_path: str | None = None
    plate: str = ""
    input_unit: str = "m"
    captured_at: datetime | None = None

    @property
    def camera(self) -> str:
        """融合来源的稳定标识，只用于 pairMetadata/analysisCamera 的审计字段。

        写入业务表的 ``cameraIdentifier`` 不能带 ``+``（见 ``_analyze_pair`` 中
        对 ``config.camera_identifier`` 的覆盖），那里固定用基准雷达 A。
        """
        return f"{self.camera_a}+{self.camera_b}"
