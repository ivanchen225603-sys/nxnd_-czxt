using System.Text.Json.Nodes;

namespace CarPointCloudWebApi.Application;

/// <summary>
/// 把 DM8 尺寸记录、Worker 扫描快照和 Worker 分析记录合成磅房页面的一个进度阶段。
/// 只有查到 DM8 尺寸记录才算 completed；Worker 用尽分析重试后才报告 failed。
/// </summary>
public static class CaptureProgressBuilder
{
    public static CaptureProgressResponse Build(
        string workflowId,
        VehicleArrivalRecord arrival,
        JsonNode? scan,
        JsonNode? analysis,
        VehicleDimensionDto? dimensions)
    {
        var progress = new CaptureProgressResponse
        {
            WorkflowId = workflowId,
            LicensePlate = arrival.LicensePlate,
            Scan = scan?.DeepClone(),
            Analysis = analysis?.DeepClone(),
            Dimensions = dimensions
        };

        // 按流程查询的快照只用于该流程的采集进度。
        var scanBelongsToWorkflow = Text(scan, "requestId") == workflowId;
        if (scanBelongsToWorkflow)
        {
            progress.CapturedCameras = TextArray(scan?["capturedCameras"]);
            progress.PersistedCameras = TextArray(scan?["persistedCameras"]);
            progress.PersistenceState = Text(scan, "persistenceState");
        }

        if (dimensions is not null)
            return Set(progress, CaptureStages.Completed, true, dimensions.ReviewStatus == "pending"
                ? "计算完成，尺寸已写入数据库，待人工审核（模型/计算值不等于已核验实测值）"
                : "计算完成，尺寸已写入数据库");

        if (Text(analysis, "staticStatus") == "pending")
            return Set(progress, CaptureStages.Queued, false, "两路点云已保存，正在核对完整扫描是否处于同一停稳段");
        if (Text(analysis, "staticStatus") is "rejected" or "insufficient")
        {
            progress.Error = Text(analysis, "staticReason");
            return Set(progress, CaptureStages.CaptureFailed, true, $"静止证据未确认，已保留原始点云。原因：{progress.Error}");
        }
        switch (Text(analysis, "stage"))
        {
            case "failed":
                progress.Error = Text(analysis, "error");
                return Set(progress, CaptureStages.AnalysisFailed, true, "分析失败，需人工处理");
            case "completed" when analysis!["hasDimensionPayload"]?.GetValue<bool>() == false:
                if (IsPartialReview(analysis))
                    return Set(progress, CaptureStages.PendingReview, true,
                        "分析完成，已计算项目和缺测原因已分别显示，待人工审核（尚未入库，未更新永久空车 PCD）");
                if (HasPendingReview(analysis))
                    return Set(progress, CaptureStages.PendingReview, true,
                        "两份原始 PCD 已保存，融合计算完成，完整计算数据已显示，待人工审核（尚未入库，未更新永久空车 PCD）");
                return Set(progress, CaptureStages.CompletedWithoutDimensions, true, DescribeMissingDimensions(analysis));
            case "completed":
                return Set(progress, CaptureStages.CompletedUnpersisted, true,
                    "计算已完成，尚未查到本次数据库记录；已返回的计算值在下方显示，可刷新核对入库状态");
            case "analyzing":
                return Set(progress, CaptureStages.Analyzing, false, "Analyzer 正在融合与计算");
            case "queued":
                return Set(progress, CaptureStages.Queued, false, "双雷达点云已配对，等待分析");
        }

        if (!scanBelongsToWorkflow)
            return Set(progress, CaptureStages.Unknown, false, "暂未查到本次采集进度，请稍后刷新");

        var outcome = Text(scan, "outcome");
        if (outcome is "timedout" or "cancelled")
        {
            progress.Error = Text(scan, "lastError");
            return Set(progress, CaptureStages.CaptureFailed, true,
                outcome == "timedout" ? "采集超时，未收齐两台雷达的完整场" : "采集已取消");
        }
        if (progress.PersistenceState == "saved" && progress.PersistedCameras.Count >= 2)
            return Set(progress, CaptureStages.Saved, false, "两份原始 PCD 已落盘，等待配对与分析");
        if (progress.CapturedCameras.Count > 0)
            return Set(progress, CaptureStages.Captured, false, progress.CapturedCameras.Count >= 2
                ? "两台雷达点云均已捕获，正在写盘"
                : $"已捕获 {string.Join("、", progress.CapturedCameras)}，等待另一台");
        return Set(progress, CaptureStages.Waiting, false, "正在采集双雷达点云");
    }

    private static CaptureProgressResponse Set(CaptureProgressResponse progress, string stage, bool terminal, string message)
    {
        progress.Stage = stage;
        progress.Terminal = terminal;
        progress.Message = message;
        return progress;
    }

    /// <summary>Analyzer 判定待审核且三项车厢尺寸都已算出。</summary>
    private static bool HasPendingReview(JsonNode analysis)
    {
        var status = analysis["measurementStatus"];
        var carriage = (analysis["reviewMeasurements"]?["dimensions"] ?? analysis["dimensions"])?["carriage"];
        return Text(status, "reviewStatus") == "pending" && status?["calculationSucceeded"]?.GetValue<bool>() == true &&
               new[] { "length", "width", "height" }.All(key => carriage?[key] is JsonValue);
    }

    /// <summary>Analyzer 只算出部分项目（至少一项有数值），缺测原因随结果展示。</summary>
    private static bool IsPartialReview(JsonNode analysis)
    {
        var review = analysis["reviewMeasurements"];
        if (review?["partial"]?.GetValue<bool>() != true || Text(review, "reviewStatus") != "pending")
            return false;
        var dimensions = review["dimensions"];
        return new[] { "vehicle", "carriage" }.Any(group => new[] { "length", "width", "height" }
                   .Any(key => dimensions?[group]?[key] is JsonValue)) ||
               new[] { "cabToTrailerFrontDist", "trailerBottomToGroundHeight", "ribCount", "highLowPlateHeight", "wireCount" }
                   .Any(key => review[key] is JsonValue);
    }

    private static string DescribeMissingDimensions(JsonNode analysis)
    {
        var loadStatus = Text(analysis["loadStatus"], "status");
        var cause = loadStatus == "Uncertain" ? "载荷状态不确定，本次不测量尺寸" : Text(analysis["measurementStatus"], "reason") switch
        {
            "vehicle_not_empty" => "车厢非空，本次不测量尺寸",
            "load_status_uncertain" => "载荷状态不确定，本次不测量尺寸",
            "inner_wall_boundaries_unverified" => "端壁内表面未通过校验，尺寸不可信",
            "longitudinal_inner_edges_unavailable" => "完整侧壁下部内缘缺测，无法可靠计算内长",
            "cab_front_at_measurement_height_unavailable" => "测量高度处未找到车头前沿",
            _ => loadStatus == "Loaded" ? "车厢载货，本次不测量尺寸" : "算法判定本次不产出尺寸"
        };
        return $"分析已完成：{cause}。已计算项目与缺测原因分别显示；本次未生成完整尺寸入库记录。";
    }

    private static string? Text(JsonNode? node, string property) =>
        node is JsonObject obj && obj[property] is JsonValue value && value.TryGetValue<string>(out var text) ? text : null;

    private static IReadOnlyList<string> TextArray(JsonNode? node) =>
        node is JsonArray array ? [.. array.Select(item => item?.GetValue<string>()).OfType<string>()] : [];
}
