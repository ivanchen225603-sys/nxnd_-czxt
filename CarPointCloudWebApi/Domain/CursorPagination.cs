namespace CarPointCloudWebApi.Domain;

/// <summary>内部使用的稳定键集分页游标。</summary>
public sealed record QueryCursor(
    int Version,
    string Endpoint,
    string Mode,
    int PageSize,
    long LastTimeTicks,
    long LastId,
    long? WindowStartTicks);

/// <summary>
/// 一次分页查询实际执行的参数与结果规模，仅用于审计日志。
/// </summary>
/// <remarks>
/// 第三方只看到 data 数组，看不到服务端把 limit/timeWindow/cursor 解析成了什么。
/// 查出 0 条时如果没有这份记录，就无法区分"时间窗解析成了别的区间"和"确实没数据"。
/// 不进任何 HTTP 响应体，只经 <c>HttpContext.Items</c> 交给访问日志中间件落库。
/// </remarks>
public sealed record CursorQueryDiagnostics(
    string Mode,
    int PageSize,
    DateTime? WindowStart,
    DateTime? BeforeTime,
    long? BeforeId,
    int ReturnedCount,
    bool HasMore,
    bool DatabaseEnabled)
{
    /// <summary>合同查询实际检查的记录数，不改变内部分页的既有字段。</summary>
    public int ScannedCount { get; init; }
    /// <summary>因身份或合同必填字段不完整而排除的数量。</summary>
    public int FilteredCount { get; init; }

    /// <summary>渲染成审计日志里的一行人类可读说明。</summary>
    public string Describe(string endpoint)
    {
        if (!DatabaseEnabled)
        {
            return $"{endpoint} 查询未执行：Database:Enabled=false，直接返回空集";
        }

        var window = WindowStart.HasValue
            ? WindowStart.Value.ToString("yyyy-MM-dd HH:mm:ss.fff")
            : "不限";
        var continued = BeforeTime.HasValue
            ? $"续传自 {BeforeTime.Value:yyyy-MM-dd HH:mm:ss.fff}/id={BeforeId}"
            : "首页";
        return $"{endpoint} 查询完成：模式={Mode}，每页={PageSize}，时间窗起点={window}，" +
               $"{continued}，返回={ReturnedCount} 条，还有更多={(HasMore ? "是" : "否")}" +
               (ScannedCount > 0 ? $"，扫描={ScannedCount} 条，按合同排除={FilteredCount} 条" : "");
    }
}

/// <summary>保持现有 data 数组响应结构时使用的分页结果。</summary>
public sealed record CursorPage<T>(
    IReadOnlyList<T> Items,
    string? NextCursor,
    bool HasMore,
    CursorQueryDiagnostics? Diagnostics = null);
