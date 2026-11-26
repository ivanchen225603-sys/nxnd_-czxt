using System.Runtime.InteropServices;
using CarPointCloudLadarWorker.Domain;
using CarPointCloudLadarWorker.Preview;

namespace CarPointCloudLadarWorker.Api;

/// <summary>控制接口的响应形状与状态码（字段 camelCase，枚举小写）。</summary>
internal static class ApiResponses
{
    public static object Scan(ScanSnapshot s, double? estimatedCycleSeconds) => new
    {
        state = s.State.ToString().ToLowerInvariant(),
        outcome = s.Outcome.ToString().ToLowerInvariant(),
        requestId = s.RequestId,
        sessionId = s.SessionId,
        startedAt = s.StartedAt,
        completedAt = s.CompletedAt,
        expectedCameras = s.ExpectedCameras,
        capturedCameras = s.CapturedCameras,
        persistedCameras = s.PersistedCameras,
        persistenceState = s.PersistenceState,
        lastError = s.LastError,
        effectiveTimeoutSeconds = s.EffectiveTimeoutSeconds,
        estimatedCycleSeconds
    };

    public static object Command(ScanCommandResult r, double? estimatedCycleSeconds) => new
    {
        success = r.Success,
        code = r.Code,
        started = r.Started,
        idempotent = r.Idempotent,
        message = r.Message,
        scan = Scan(r.Snapshot, estimatedCycleSeconds)
    };

    /// <summary>202 新会话，200 重复请求，400 requestId 非法，409 已有会话。</summary>
    public static int StartStatusCode(ScanCommandResult r) => r.Code switch
    {
        "existing" => 200,
        "scanning" => 202,
        "invalid_request_id" => 400,
        _ => 409
    };

    /// <summary>202 已取消，200 本无会话，409 会话不属于该流程。</summary>
    public static int StopStatusCode(ScanCommandResult r) => r.Code switch
    {
        "cancelled" => 202,
        "already_idle" => 200,
        _ => 409
    };

    public static object Analysis(AnalysisRecord r) => new
    {
        success = true,
        workflowId = r.WorkflowId,
        pairId = r.PairId,
        stage = r.Stage.ToString().ToLowerInvariant(),
        updatedAt = r.UpdatedAt,
        cameraA = r.CameraA,
        cameraB = r.CameraB,
        capturedAt = r.CapturedAt,
        sourcePcdPaths = r.SourcePcdPaths,
        fusion = r.Fusion,
        dimensions = r.Dimensions,
        measurementStatus = r.MeasurementStatus,
        reviewMeasurements = r.ReviewMeasurements,
        loadStatus = r.LoadStatus,
        persistence = r.Persistence,
        dimensionPayload = r.DimensionPayload,
        hasDimensionPayload = r.HasDimensionPayload,
        staticStatus = r.StaticStatus,
        staticReason = r.StaticReason,
        error = r.Error,
        permanentFailure = r.PermanentFailure
    };

    /// <summary>预览点云用 base64(Float32 小端 xyz 交错) 传输，体积约为 JSON 数组的三分之一。</summary>
    public static object FusedFrame(LiveFusedPreviewFrame frame, LivePreviewStatus status) => new
    {
        success = true,
        updated = true,
        sessionId = frame.SessionId,
        version = frame.Version,
        capturedAt = frame.CapturedAt,
        capturedAtA = frame.CapturedAtA,
        capturedAtB = frame.CapturedAtB,
        cameraA = frame.CameraA,
        cameraB = frame.CameraB,
        pairDeltaMilliseconds = frame.PairDeltaMilliseconds,
        sourcePointCountA = frame.SourcePointCountA,
        sourcePointCountB = frame.SourcePointCountB,
        pointCountA = frame.PointCountA,
        pointCountB = frame.PointCountB,
        previewPointCount = frame.PreviewPointCount,
        pointsEncoding = "float32le-xyz-base64",
        points = Convert.ToBase64String(MemoryMarshal.AsBytes(frame.Coordinates.AsSpan())),
        preview = status
    };
}
