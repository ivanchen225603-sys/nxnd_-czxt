"""跨层共享的业务异常，不依赖接口或基础设施。"""


class DualCloudMergeError(RuntimeError):
    """双雷达清理/合并无法安全完成。"""
