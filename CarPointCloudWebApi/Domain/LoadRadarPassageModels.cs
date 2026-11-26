namespace CarPointCloudWebApi.Domain;

/// <summary>第一次测量之前冻结的真实到车上下文；重试不改变任何字段。</summary>
public sealed record LoadPassageStart(string NodeId, string ChannelId, string PassageId,
    string CalibrationId, string ConfigurationHash, DateTimeOffset ArrivedAt, string SourceMode);

/// <summary>CH4 位置停稳通知；是否派发双雷达由 WebApi 按配置与原车次雷达状态决定。</summary>
public sealed record LoadPassageStandstill(string NodeId, string ChannelId, string PassageId,
    DateTimeOffset? StoppedAt = null, string? SegmentId = null);

public sealed record LoadPassageDeparture(string NodeId, string ChannelId, string PassageId);

/// <summary>停稳和离场通知的答复，包含站点原车次对应的 workflowId。</summary>
public sealed record LoadPassageStandstillReceipt(string PassageId, string Status, string? WorkflowId);

public sealed record LoadPassageStartRecord(string PassageKey, string PayloadHash,
    LoadPassageStart Start, string CameraIdentifier, string WorkflowId, LoadPassageBinding? Binding = null);

public sealed record LoadPassagePreparation(LoadPassageStart Start, string WorkflowId,
    string CameraIdentifier, string Status, LoadPassageBinding? Binding, LoadBaselineVersion? Baseline,
    DateTimeOffset? BaselineAsOf = null);
