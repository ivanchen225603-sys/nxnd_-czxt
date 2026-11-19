using System.Text.Json.Nodes;

namespace CarPointCloudLadarWorker.Domain;

/// <summary>分析进度。</summary>
public enum AnalysisStage
{
    Queued,
    Analyzing,
    Completed,
    Failed
}

/// <summary>一个流程的分析进度与 Analyzer 结果（诊断用，尺寸以 DM8 为准）。</summary>
public sealed record AnalysisRecord(
    string WorkflowId,
    string PairId,
    AnalysisStage Stage,
    DateTime UpdatedAt,
    string CameraA,
    string CameraB,
    DateTime CapturedAt,
    IReadOnlyList<string> SourcePcdPaths)
{
    public JsonObject? Fusion { get; init; }
    public JsonObject? Dimensions { get; init; }
    public JsonObject? MeasurementStatus { get; init; }
    public JsonObject? ReviewMeasurements { get; init; }
    public JsonObject? LoadStatus { get; init; }
    public JsonObject? Persistence { get; init; }
    /// <summary>本次计算的厘米业务载荷；与数据库是否已保存分开保留。</summary>
    public JsonObject? DimensionPayload { get; init; }
    public bool HasDimensionPayload { get; init; }
    public string? Error { get; init; }

    /// <summary>重试等待使用 Queued，重试耗尽才发布 Failed 终局。</summary>
    public bool PermanentFailure => Stage == AnalysisStage.Failed;
}
