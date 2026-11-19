using System.Text.Json;

namespace CarPointCloudWebApi.Domain;

// JT128 独立于双雷达尺寸协议；原始测量载荷不可随业务关联修改。
public sealed record LoadMeasurementInput
{
    public string MeasurementId { get; init; } = "";
    public string NodeId { get; init; } = "";
    public string ChannelId { get; init; } = "";
    public string PassageId { get; init; } = "";
    public string CalibrationId { get; init; } = "";
    public string SourceMode { get; init; } = "";
    public string AlgorithmVersion { get; init; } = "";
    public string ConfigurationHash { get; init; } = "";
    public string? BaselineCandidateSha256 { get; init; }
    [System.Text.Json.Serialization.JsonIgnore(Condition = System.Text.Json.Serialization.JsonIgnoreCondition.WhenWritingDefault)]
    public bool AutomaticBaseline { get; init; }
    public DateTimeOffset MeasuredAt { get; init; }
    /// <summary>构成本次体积结果的实际观测起止时间；历史载荷缺失时不补造。</summary>
    [System.Text.Json.Serialization.JsonIgnore(Condition = System.Text.Json.Serialization.JsonIgnoreCondition.WhenWritingNull)]
    public DateTimeOffset? MeasurementStartedAt { get; init; }
    [System.Text.Json.Serialization.JsonIgnore(Condition = System.Text.Json.Serialization.JsonIgnoreCondition.WhenWritingNull)]
    public DateTimeOffset? MeasurementEndedAt { get; init; }
    public string Verdict { get; init; } = "unknown";
    public bool FrameDegraded { get; init; }
    public string VolumeStatus { get; init; } = "unavailable";
    public LoadRadarNumber? VolumeM3 { get; init; }
    public string? BaselineVersionId { get; init; }
    [System.Text.Json.Serialization.JsonIgnore(Condition = System.Text.Json.Serialization.JsonIgnoreCondition.WhenWritingNull)]
    public LoadBaselineUse? BaselineUse { get; init; }
    public LoadRadarNumber? FilledRatio { get; init; }
    public LoadRadarNumber? BoundSpread { get; init; }
    public LoadRadarNumber? HoleRatio { get; init; }
    public int PointCount { get; init; }
    public string Reason { get; init; } = "";
    public float[][] Points { get; init; } = [];
}

// 来源描述CH4空车；测量本身的ChannelId/CalibrationId仍描述实际载货车道。
public sealed record LoadBaselineUse(string SourceChannelId, string SourceCalibrationId,
    string SourceConfigurationHash, string Policy, string RegistrationStatus, string RegistrationReason,
    double DxM, double DyM, double YawSlope, double YawPivotYM, LoadBaselinePose? Pose);
public sealed record LoadBaselinePose(double[] Left, double[] Right, double XLeft, double XRight, double YMid);

public sealed record LoadMeasurementRecord(string PassageKey, string PayloadHash,
    LoadMeasurementInput Measurement, DateTimeOffset ReceivedAt, LoadPassageBinding? Binding = null,
    LoadCoalPublication? Publication = null,
    [property: System.Text.Json.Serialization.JsonIgnore(Condition = System.Text.Json.Serialization.JsonIgnoreCondition.WhenWritingNull)]
    LoadVolumeRevision? LatestRevision = null);

public sealed record LoadPassageBinding(string PassageKey, string WorkflowId, string LicensePlate,
    string CameraIdentifier, string VehicleIdentity, string Actor, string Reason, DateTimeOffset BoundAt);

public sealed record LoadBindRequest(string WorkflowId, string VehicleIdentity, string Reason);
public sealed record LoadBaselineCreateRequest(string MeasurementId, string AssetSha256,
    bool ConfirmedEmpty, string Reason);
public sealed record LoadBaselineActivateRequest(string? ExpectedVersionId, string Reason);
public sealed record LoadPublishRequest(string Reason);

public sealed record LoadBaselineVersion(string VersionId, string ScopeKey, string NodeId,
    string ChannelId, string CalibrationId, string VehicleIdentity, string LicensePlate,
    string MeasurementId, string AssetSha256, string Actor, string Reason, DateTimeOffset CreatedAt,
    bool Active = false, DateTimeOffset? ActivatedAt = null);

public sealed record LoadCoalPublication(string PassageKey, string WorkflowId,
    string MeasurementId, long CoalVolumeId, string Actor, string Reason, DateTimeOffset PublishedAt);

public sealed class LoadRadarOptions
{
    public bool Enabled { get; set; }
    public string StorageRoot { get; set; } = "data/load-radar";

    /// <summary>
    /// 正式煤量的体积上下界相对宽度上限；<c>null</c>（默认）= 不设这道门槛。
    /// </summary>
    /// <remarks>
    /// 原为三处写死 0.03，而实测该值最低只到 46%（几何决定），等于把煤量永久关死。
    /// 详见 <see cref="Common.LoadRadarPublicationPolicy.MeetsQuality"/> 的说明。
    /// 设了值就必须在 (0, 1] 内，由 <c>LoadRadarRegistration.IsValid</c> 校验。
    /// </remarks>
    public decimal? VolumeBoundSpreadMax { get; set; }

    public List<LoadRadarNodeOptions> Nodes { get; set; } = [];
}

public sealed class LoadRadarNodeOptions
{
    public string NodeId { get; set; } = "";
    public string AgentKey { get; set; } = "";
    public string? BaseUrl { get; set; }
    public List<LoadRadarLaneOptions> Channels { get; set; } = [];
}

public sealed class LoadRadarLaneOptions
{
    public string ChannelId { get; set; } = "";
    public string CameraIdentifier { get; set; } = "";
    public string Name { get; set; } = "";
}

public sealed record LoadRadarLaneState(string NodeId, string ChannelId, string Name,
    string CameraIdentifier, bool Online, JsonElement? State, string? Error);

public sealed class LoadRadarException(int statusCode, string message, bool retryable = false) : Exception(message)
{
    public int StatusCode { get; } = statusCode;
    public bool Retryable { get; } = retryable;
}
