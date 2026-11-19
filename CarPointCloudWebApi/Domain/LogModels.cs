namespace CarPointCloudWebApi.Domain;

/// <summary>
/// 日志查询相关模型。
/// </summary>
/// <remarks>
/// 供日志查看页（logs.html）使用。数据来源为数据库中的 ApiAccessLog 表，
/// 映射为前端期望的 { id, time, level, source, message } 结构。
/// </remarks>
public class LogEntryDto
{
    /// <summary>日志 ID。</summary>
    public long Id { get; set; }

    /// <summary>日志时间（字符串，yyyy-MM-dd HH:mm:ss.fff）。</summary>
    public string Time { get; set; } = string.Empty;

    /// <summary>根据完成时间和耗时推算的调用开始时间。</summary>
    public string StartTime { get; set; } = string.Empty;

    /// <summary>日志级别：debug / info / warn / error。</summary>
    public string Level { get; set; } = "info";

    /// <summary>日志来源。</summary>
    public string Source { get; set; } = string.Empty;

    /// <summary>日志消息内容。</summary>
    public string Message { get; set; } = string.Empty;

    /// <summary>记录类型：Access 或 Lifecycle。</summary>
    public string RecordKind { get; set; } = "Access";

    /// <summary>接口生命周期阶段。</summary>
    public string? LifecycleStage { get; set; }

    /// <summary>串联同一次跨系统交互所有生命周期记录的标识。</summary>
    public string? InteractionId { get; set; }
    /// <summary>集成业务类型，例如 LicensePlate 或 CoalVolume。</summary>
    public string? IntegrationType { get; set; }
    /// <summary>调用方向：Inbound 或 Outbound。</summary>
    public string? Direction { get; set; }
    /// <summary>本次调用的发起系统。</summary>
    public string? CallerSystem { get; set; }
    /// <summary>本次调用的目标系统。</summary>
    public string? TargetSystem { get; set; }
    /// <summary>跨来车、采集和分析链路的业务流程 ID。</summary>
    public string? WorkflowId { get; set; }
    /// <summary>调用方提供的业务事件 ID。</summary>
    public string? EventId { get; set; }
    /// <summary>从请求或响应中提取的车牌号。</summary>
    public string? LicensePlate { get; set; }
    /// <summary>从调用数据中提取的雷达或摄像头点位。</summary>
    public string? CameraIdentifier { get; set; }
    /// <summary>同一交互链路中的外呼尝试序号。</summary>
    public int? AttemptNo { get; set; }
    /// <summary>脱敏后的请求头 JSON。</summary>
    public string? RequestHeadersJson { get; set; }
    /// <summary>按审计容量限制截断后的请求体。</summary>
    public string? RequestBody { get; set; }
    /// <summary>脱敏后的响应头 JSON。</summary>
    public string? ResponseHeadersJson { get; set; }
    /// <summary>按审计容量限制截断后的响应体。</summary>
    public string? ResponseBody { get; set; }
    /// <summary>HTTP 请求方法。</summary>
    public string? HttpMethod { get; set; }
    /// <summary>不包含查询字符串的请求路径。</summary>
    public string? RequestPath { get; set; }
    /// <summary>原始查询字符串。</summary>
    public string? QueryString { get; set; }
    /// <summary>请求协议，例如 http 或 https。</summary>
    public string? Scheme { get; set; }
    /// <summary>请求的 Host 头。</summary>
    public string? Host { get; set; }
    /// <summary>ASP.NET Core 请求跟踪标识。</summary>
    public string? TraceIdentifier { get; set; }
    /// <summary>调用方网络地址。</summary>
    public string? ClientIp { get; set; }
    /// <summary>请求体媒体类型。</summary>
    public string? RequestContentType { get; set; }
    /// <summary>响应体媒体类型。</summary>
    public string? ResponseContentType { get; set; }
    /// <summary>合作方签名使用的 Timestamp 请求头。</summary>
    public string? TimestampHeader { get; set; }
    /// <summary>脱敏后的 Signature 请求头。</summary>
    public string? SignatureHeader { get; set; }
    /// <summary>调用方提供的幂等键请求头。</summary>
    public string? IdempotencyKeyHeader { get; set; }
    /// <summary>请求体长度，未知时为空。</summary>
    public long? RequestContentLength { get; set; }
    /// <summary>响应体长度，未知时为空。</summary>
    public long? ResponseContentLength { get; set; }
    /// <summary>HTTP 响应状态码。</summary>
    public int HttpStatus { get; set; }
    /// <summary>业务响应码；当前通常与 HTTP 状态码一致。</summary>
    public int BusinessCode { get; set; }
    /// <summary>该阶段或完整请求耗时，单位毫秒。</summary>
    public int ElapsedMs { get; set; }
    /// <summary>调用失败或拒绝时记录的错误摘要。</summary>
    public string? ErrorMessage { get; set; }
}

/// <summary>
/// 日志查询响应（分页）。
/// </summary>
public class LogPageResult
{
    /// <summary>当前页日志列表。</summary>
    public List<LogEntryDto> Items { get; set; } = new();

    /// <summary>符合条件的总条数。</summary>
    public long Total { get; set; }

    /// <summary>当前页码（从 1 开始）。</summary>
    public int Page { get; set; }

    /// <summary>每页条数。</summary>
    public int PageSize { get; set; }
}
