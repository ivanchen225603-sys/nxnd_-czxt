from datetime import datetime, timezone
import logging


logger = logging.getLogger(__name__)

def _utc_now_iso() -> str:
    """带时区的 UTC 时间戳，ISO-8601 格式，精确到秒。

    用于替代已废弃的 ``datetime.utcnow()``：返回的字符串带显式 ``+00:00``
    偏移，下游拿到的是真正的 UTC 时间，而不是手工拼上 'Z' 的本地无时区值。
    """
    return datetime.now(timezone.utc).isoformat(timespec='seconds')
