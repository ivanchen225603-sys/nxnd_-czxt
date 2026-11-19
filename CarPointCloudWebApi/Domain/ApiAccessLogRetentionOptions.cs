namespace CarPointCloudWebApi.Domain;

/// <summary>
/// Controls how long API access logs are retained and how many rows are
/// removed in each database operation.
/// </summary>
public sealed class ApiAccessLogRetentionOptions
{
    /// <summary>配置文件中审计日志保留策略的节名称。</summary>
    public const string SectionName = "AuditLog";

    /// <summary>Number of days to retain API access logs.</summary>
    public int RetentionDays { get; set; } = 90;

    /// <summary>Maximum rows removed by one repository call.</summary>
    public int CleanupBatchSize { get; set; } = 1000;

    /// <summary>Maximum time allowed for persisting one access-log record.</summary>
    public int WriteTimeoutMs { get; set; } = 2000;

    /// <summary>Maximum request or response characters stored in one audit row.</summary>
    public int MaxBodyChars { get; set; } = 1024 * 1024;

    /// <summary>仅为乙方尺寸、煤量 GET 查询记录脱敏正文和文件响应日志，不改变其他采集入口的正文策略。</summary>
    public bool CapturePartnerQueryBodies { get; set; }
}
