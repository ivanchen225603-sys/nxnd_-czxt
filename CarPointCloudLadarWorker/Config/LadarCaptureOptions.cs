namespace CarPointCloudLadarWorker.Config;

/// <summary>appsettings.json 的 <c>LadarCapture</c> 段。</summary>
/// <remarks>
/// 配置是外部输入，只在 <see cref="Validate"/> 中校验一次；校验通过后各模块直接使用。
/// Devices[0] 是基准雷达 A，Devices[1] 是雷达 B，外参方向固定为 B_to_A。
/// </remarks>
public sealed class LadarCaptureOptions
{
    /// <summary>两台雷达；顺序决定 A/B。</summary>
    public List<LadarDeviceOptions> Devices { get; set; } = [];

    /// <summary>B_to_A 外参 JSON 路径，实时预览使用，并原样传给 Analyzer。</summary>
    public string DualRadarExtrinsicPath { get; set; } = string.Empty;

    /// <summary>PCD 落盘根目录。</summary>
    public string CaptureDirectory { get; set; } = @"D:\data\captures";

    /// <summary>写入 PCD 前对坐标乘的系数。</summary>
    public double CoordinateScale { get; set; } = 1.0;

    /// <summary>收到扫描指令后，每台雷达先丢弃的合格完整场数。</summary>
    public int DiscardCompleteCyclesAfterCommand { get; set; } = 1;

    /// <summary>指令后这段时间内回调的场不计作边界（雷达交付延迟）；未配置为 0。</summary>
    public int? MaximumFrameDeliveryDelayMilliseconds { get; set; }

    /// <summary>扫描会话超时。</summary>
    public int ScanOnceTimeoutSeconds { get; set; } = 30;

    /// <summary>仅采集 PCD，不调用 Analyzer。</summary>
    public bool CollectPcdOnly { get; set; }

    /// <summary>是否调用 Analyzer。</summary>
    public bool EnablePythonAnalyzer { get; set; } = true;

    /// <summary>Analyzer 服务地址。</summary>
    public string AnalyzerServiceUrl { get; set; } = "http://127.0.0.1:8010";

    /// <summary>原样传给 Analyzer 的 baselineDir。</summary>
    public string AnalyzerBaselineDirectory { get; set; } = string.Empty;

    /// <summary>单次 Analyzer 调用超时。</summary>
    public int AnalyzerTimeoutSeconds { get; set; } = 120;

    /// <summary>分析任务 JSON 目录；空时使用 CaptureDirectory/analysis-tasks。</summary>
    public string AnalysisTaskDirectory { get; set; } = string.Empty;

    /// <summary>首次失败后的重试次数；默认 2，即正常情况下最多调用 3 次。</summary>
    public int AnalysisRetryCount { get; set; } = 2;

    /// <summary>同时调用 Analyzer 的任务数。默认逐车执行；等待重试或静态确认不占此名额。</summary>
    public int MaxConcurrentAnalyses { get; set; } = 1;

    /// <summary>现场标称完整场周期，仅用于自动采集时间预算告警。</summary>
    public double? ExpectedFrameCycleSeconds { get; set; }

    /// <summary>失败后固定等待的秒数。</summary>
    public int AnalysisRetryDelaySeconds { get; set; } = 10;

    /// <summary>为 false 时不访问 WebApi。</summary>
    public bool EnableApiRequests { get; set; }

    /// <summary>分析前按 workflowId 向 WebApi 查询已确认车牌。</summary>
    public bool FetchLicensePlateOnCapture { get; set; }

    /// <summary>查询车牌超时。</summary>
    public int LicensePlateFetchTimeoutSeconds { get; set; } = 2;

    /// <summary>未查询或查不到时使用的固定车牌，空表示不传。</summary>
    public string LicensePlate { get; set; } = string.Empty;

    /// <summary>WebApi 地址。</summary>
    public string WebApiUrl { get; set; } = "http://localhost:5031";

    /// <summary>WebApi 签名密钥。</summary>
    public string AppSecret { get; set; } = string.Empty;

    /// <summary>每日清理时刻 HH:mm；非空时按日清理，删除该时刻之前写入的 PCD。</summary>
    public string CaptureCleanupDailyAt { get; set; } = string.Empty;

    /// <summary>未配置每日清理时，PCD 保留小时数；0 关闭清理。</summary>
    public int CaptureRetentionHours { get; set; }

    /// <summary>按保留小时清理时的检查间隔。</summary>
    public int CaptureCleanupIntervalMinutes { get; set; } = 60;

    /// <summary>清理时跳过的目录（永久空车档案）。</summary>
    public string[] PermanentPointCloudDirectories { get; set; } = [];

    /// <summary>健康日志及快照文件间隔；0 关闭。</summary>
    public int HealthLogIntervalSeconds { get; set; } = 10;

    /// <summary>每台雷达一份 {camera}.json 健康快照的目录。</summary>
    public string HealthSnapshotDirectory { get; set; } = @"D:\data\worker-health";

    /// <summary>是否启动 HTTP 控制接口。</summary>
    public bool EnableHttpApi { get; set; } = true;

    /// <summary>HTTP 控制接口监听地址。</summary>
    public string HttpApiHost { get; set; } = "127.0.0.1";

    /// <summary>HTTP 控制接口端口。</summary>
    public int HttpApiPort { get; set; } = 8000;

    /// <summary>是否生成磅房实时融合预览。</summary>
    public bool EnableLivePreview { get; set; }

    /// <summary>预览每台雷达抽样点数上限。</summary>
    public int LivePreviewMaxPointsPerCamera { get; set; } = 12000;

    /// <summary>两台预览样本采集时间差上限。</summary>
    public int LivePreviewPairingWindowSeconds { get; set; } = 7;

    /// <summary>预览样本过期时间。</summary>
    public int LivePreviewStaleSeconds { get; set; } = 21;

    /// <summary>用内置合成雷达代替厂家 SDK（联调用）。</summary>
    public bool EnableSyntheticRadar { get; set; }

    /// <summary>合成雷达出场间隔。</summary>
    public double SyntheticFrameIntervalSeconds { get; set; } = 4;

    /// <summary>合成雷达每场点数。</summary>
    public int SyntheticPointCount { get; set; } = 6000;

    /// <summary>合成雷达两台之间的出场间隔。</summary>
    public int SyntheticInterDeviceDelayMilliseconds { get; set; } = 50;

    /// <summary>是否调用 Analyzer。</summary>
    public bool AnalysisEnabled => EnablePythonAnalyzer && !CollectPcdOnly;

    /// <summary>启动时校验一次；不通过直接抛出，进程不启动。</summary>
    public void Validate()
    {
        if (Devices.Count != 2)
            throw new InvalidOperationException("LadarCapture:Devices 必须恰好配置两台雷达（A、B）");
        if (Devices[0].CameraIdentifier == Devices[1].CameraIdentifier)
            throw new InvalidOperationException("LadarCapture:Devices 两台雷达的 CameraIdentifier 不能相同");
        if (string.IsNullOrWhiteSpace(CaptureDirectory))
            throw new InvalidOperationException("LadarCapture:CaptureDirectory 不能为空");
        if (AnalysisRetryCount < 0 || AnalysisRetryDelaySeconds < 0)
            throw new InvalidOperationException("LadarCapture:AnalysisRetryCount 和 AnalysisRetryDelaySeconds 不能小于 0");
        if (MaxConcurrentAnalyses is < 1 or > 4)
            throw new InvalidOperationException("LadarCapture:MaxConcurrentAnalyses 必须在 1 到 4 之间");
        if (CaptureCleanupDailyAt.Length > 0)
            TimeOnly.ParseExact(CaptureCleanupDailyAt, "HH:mm");
    }
}

/// <summary>一台雷达。</summary>
public sealed class LadarDeviceOptions
{
    /// <summary>型号，仅用于日志和状态。</summary>
    public string RadarModel { get; set; } = "Ladar91";

    /// <summary>雷达 IP。</summary>
    public string Address { get; set; } = string.Empty;

    /// <summary>雷达端口。</summary>
    public ushort Port { get; set; } = 1112;

    /// <summary>点位标识，如 CAM_004。</summary>
    public string CameraIdentifier { get; set; } = string.Empty;

    /// <summary>有限点数低于该值的场不作为边界或业务帧。</summary>
    public int MinValidFramePointCount { get; set; } = 5000;

    /// <summary>现场验证的一场扫描最大时长；空表示尚无证据，单位毫秒。</summary>
    public int? VerifiedMaxScanDurationMs { get; set; }

    /// <summary>现场验证的场结束到 SDK 回调取时的最大延迟，单位毫秒。</summary>
    public int? VerifiedMaxCallbackDelayMs { get; set; }

    /// <summary>与停稳观测时基之间已验证的总误差上限，含跨机时钟差，单位毫秒。</summary>
    public int? VerifiedClockUncertaintyMs { get; set; }
}
