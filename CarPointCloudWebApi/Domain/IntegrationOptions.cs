namespace CarPointCloudWebApi.Domain;

/// <summary>
/// 第三方数据交互配置项，对应 appsettings.json 中的 Integration 节点。
/// </summary>
public class IntegrationOptions
{
    /// <summary>合作方完整链路开关，控制车牌外呼及合作方写入口；查询可由 ExternalQueriesEnabled 独立配置。</summary>
    public bool ExternalPartnerEnabled { get; set; } = true;
    /// <summary>乙方查询甲方煤量和尺寸的独立开关；未配置时沿用 ExternalPartnerEnabled，兼容旧配置。</summary>
    public bool? ExternalQueriesEnabled { get; set; }
    /// <summary>乙方车牌系统未上线时，为每个来车 workflow 生成并固化一个模拟车牌。</summary>
    public bool EnableMockLicensePlate { get; set; }

    /// <summary>模拟车牌前缀；默认使用粤A，后续随机生成 5 位字母数字。</summary>
    public string MockLicensePlatePrefix { get; set; } = "粤A";

    /// <summary>双方约定的接口密钥，用于生成和校验小写 MD5 签名。</summary>
    public string AppSecret { get; set; } = string.Empty;

    /// <summary>Timestamp 允许偏移秒数，超过该窗口认为请求过期。</summary>
    public int TimestampToleranceSeconds { get; set; } = 60;

    /// <summary>同一 IP + 路径每秒速率；对外尺寸/煤量最多 3 次，分别超限返回 429/400。</summary>
    public int RateLimitPerSecond { get; set; } = 3;

    /// <summary>乙方车牌识别服务基础地址；为空时不允许调用车牌拉取接口。</summary>
    public string PartnerBaseUrl { get; set; } = string.Empty;

    /// <summary>乙方车牌接口相对路径。</summary>
    public string LicensePlatePath { get; set; } = "/api/v1/licensePlateRecords";

    /// <summary>调用乙方接口单次超时时间，单位毫秒，最多 3000。</summary>
    public int PartnerTimeoutMs { get; set; } = 3000;

    /// <summary>乙方接口最大尝试次数，包含首次请求，最多 3 次。</summary>
    public int PartnerRetryCount { get; set; } = 3;

    /// <summary>首次调用乙方前等待时间，单位毫秒。</summary>
    public int PartnerFirstCallDelayMs { get; set; } = 10_000;

    /// <summary>乙方重试间隔，单位毫秒。</summary>
    public int PartnerRetryDelayMs { get; set; } = 5_000;
}
