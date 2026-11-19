
namespace CarPointCloudWebApi.Infrastructure;

/// <summary>
/// 日志查询数据访问接口。
/// </summary>
/// <remarks>
/// 数据来源：ApiAccessLog 表（由 ApiAccessLogMiddleware 记录所有 API 请求）。
/// </remarks>
public interface ILogRepository
{
    /// <summary>
    /// 分页查询 API 访问日志，映射为前端日志查看器所需结构。
    /// </summary>
    /// <param name="keyword">关键字（匹配路径/来源/错误消息/交互 ID/业务字段，模糊匹配）。</param>
    /// <param name="startTime">起始时间（含）。</param>
    /// <param name="endTime">结束时间（含）。</param>
    /// <param name="level">级别过滤：debug/info/warn/error，空为全部。</param>
    /// <param name="source">来源过滤（按请求路径前缀），空为全部。</param>
    /// <param name="interactionId">跨系统调用链交互编号，空为全部。</param>
    /// <param name="workflowId">来车业务流程编号，空为全部。</param>
    /// <param name="licensePlate">车牌号码过滤条件，空为全部。</param>
    /// <param name="integrationType">第三方交互类型过滤条件，空为全部。</param>
    /// <param name="page">页码，从 1 开始。</param>
    /// <param name="pageSize">每页条数。</param>
    /// <param name="total">符合条件的总条数。</param>
    /// <returns>当前页日志列表。</returns>
    List<LogEntryDto> QueryLogs(
        string? keyword,
        DateTime? startTime,
        DateTime? endTime,
        string? level,
        string? source,
        string? interactionId,
        string? workflowId,
        string? licensePlate,
        string? integrationType,
        int page,
        int pageSize,
        out long total);

    /// <summary>按交互 ID 查询完整链路明细。</summary>
    List<LogEntryDto> QueryInteraction(string interactionId);

    /// <summary>获取日志级别对应的 HTTP 状态码集合，用于级别过滤条件。</summary>
    IReadOnlyList<int> GetStatusCodesForLevel(string? level);
}
