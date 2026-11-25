using System.Net.Http.Json;
using System.Text.Json;
using System.Text.Json.Nodes;
using CarPointCloudLadarWorker.Config;
using CarPointCloudLadarWorker.Domain;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Analysis;

/// <summary>调用 Analyzer 的 <c>POST /analyze-pair</c>；Analyzer 负责融合、计算并直写 DM8。</summary>
public sealed class AnalyzerClient(HttpClient http, IOptions<LadarCaptureOptions> options)
{
    private static readonly JsonSerializerOptions Json = new(JsonSerializerDefaults.Web);

    private readonly LadarCaptureOptions _options = options.Value;

    /// <summary>分析一对 PCD，返回 completed 或 failed 的记录。</summary>
    public async Task<AnalysisRecord> AnalyzePairAsync(
        AnalysisRecord record, PairedPcdCapture pair, string? plate, CancellationToken cancellationToken)
    {
        var request = new
        {
            pcdPathA = pair.Primary.PcdPath,
            cameraA = pair.Primary.CameraIdentifier,
            pcdPathB = pair.Secondary.PcdPath,
            cameraB = pair.Secondary.CameraIdentifier,
            pairId = pair.VehicleCaptureId,
            workflowId = pair.WorkflowId,
            plate = plate ?? string.Empty,
            extrinsicPath = string.IsNullOrWhiteSpace(_options.DualRadarExtrinsicPath) ? null : _options.DualRadarExtrinsicPath,
            baselineDir = _options.AnalyzerBaselineDirectory,
            inputUnit = "m",
            // 新任务保存两路较晚的采集时刻；恢复的旧任务保持原时刻，保证重试载荷一致。
            capturedAt = record.CapturedAt
        };
        using var response = await http.PostAsJsonAsync(
            $"{_options.AnalyzerServiceUrl.TrimEnd('/')}/analyze-pair", request, Json, cancellationToken);
        var body = await response.Content.ReadAsStringAsync(cancellationToken);
        if (!response.IsSuccessStatusCode)
            return record with { Stage = AnalysisStage.Failed, Error = $"status=HttpFailure http={(int)response.StatusCode} message={body}" };

        var result = JsonNode.Parse(body)!.AsObject();
        if (result["success"]?.GetValue<bool>() != true)
            return record with { Stage = AnalysisStage.Failed, Error = $"status=ServiceFailure message={body}" };

        var outputs = result["outputs"] as JsonObject;
        return record with
        {
            Stage = AnalysisStage.Completed,
            Fusion = result["fusion"] as JsonObject,
            Dimensions = outputs?["dimensions"] as JsonObject,
            MeasurementStatus = outputs?["measurementStatus"] as JsonObject,
            ReviewMeasurements = outputs?["reviewMeasurements"] as JsonObject,
            LoadStatus = outputs?["loadStatus"] as JsonObject,
            Persistence = result["persistence"] as JsonObject,
            DimensionPayload = result["integrationPayloads"]?["vehicleDimensions"] as JsonObject,
            HasDimensionPayload = result["integrationPayloads"]?["vehicleDimensions"]?["dimensions"] is JsonObject
        };
    }
}
