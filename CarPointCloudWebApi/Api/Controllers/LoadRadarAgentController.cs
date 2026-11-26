using Microsoft.AspNetCore.Mvc;

namespace CarPointCloudWebApi.Api;

/// <summary>仅独立节点凭据可访问；测量成功回执表示已提交数据库，不代表煤量已发布。</summary>
[ApiController]
[Route("api/v1/load-radar-agent")]
public sealed class LoadRadarAgentController(LoadRadarService service, LoadRadarAssetStore assets) : ControllerBase
{
    private string NodeId => HttpContext.Items[LoadRadarAgentAuthMiddleware.NodeIdKey] as string
        ?? throw new LoadRadarException(401, "缺少已验证节点身份");

    [HttpGet("recalculations")]
    public IActionResult Recalculations([FromQuery] string channelId, [FromQuery] string? afterId = null,
        [FromQuery] DateOnly? fromDate = null) =>
        Ok(ApiResponse<object>.Success(service.Recalculations(NodeId, channelId, afterId, fromDate)));

    [HttpPost("recalculations")]
    public IActionResult Recalculate(LoadRecalculationInput input) =>
        Ok(ApiResponse<object>.Success(service.Revise(NodeId, input)));

    [HttpPost("passages/prepare")]
    public IActionResult Prepare(LoadPassageStart input, [FromServices] LoadRadarPassageService passages,
        [FromQuery] DateTimeOffset? measuredAt = null) =>
        Ok(ApiResponse<object>.Success(passages.Prepare(NodeId, input, measuredAt)));

    [HttpPost("passages/standstill")]
    public async Task<IActionResult> Standstill(LoadPassageStandstill input,
        [FromServices] LoadRadarStandstillService standstill) =>
        Ok(ApiResponse<object>.Success(await standstill.ReportAsync(NodeId, input)));

    [HttpPost("passages/departure")]
    public async Task<IActionResult> Departure(LoadPassageDeparture input,
        [FromServices] LoadRadarStandstillService standstill) =>
        Ok(ApiResponse<object>.Success(await standstill.DepartAsync(NodeId, input)));

    [HttpPost("passages/standstill-state")]
    public IActionResult ObserveStandstill(StandstillObservation input,
        [FromServices] LoadRadarStandstillService standstill) =>
        Ok(ApiResponse<StaticDecision>.Success(standstill.Observe(NodeId, input)));

    [HttpPost("measurements")]
    public IActionResult Ingest(LoadMeasurementInput input) =>
        Ok(ApiResponse<object>.Success(service.Ingest(NodeId, input)));

    [HttpGet("measurements/{id}")]
    public IActionResult Measurement(string id)
    {
        var row = service.Get(id);
        if (row.Measurement.NodeId != NodeId) throw new LoadRadarException(403, "不能读取其他节点测量");
        return Ok(ApiResponse<object>.Success(row));
    }

    [HttpPut("baseline-assets/{sha256}")]
    [RequestSizeLimit(LoadRadarAssetStore.MaxBytes)]
    public async Task<IActionResult> Upload(string sha256, [FromQuery] string measurementId,
        CancellationToken cancellationToken)
    {
        var row = service.Get(measurementId);
        if (row.Measurement.NodeId != NodeId || row.Measurement.BaselineCandidateSha256 != sha256)
            throw new LoadRadarException(403, "资产必须属于本节点已登记的测量候选");
        if (row.Measurement.Verdict != "empty" || row.Measurement.FrameDegraded || row.Measurement.SourceMode != "live")
            throw new LoadRadarException(409, "仅接收完整实车空车测量的基线资产");
        await assets.Upload(sha256, Request.Body, cancellationToken);
        if (row.Measurement.AutomaticBaseline) service.ActivateAutomaticBaseline(measurementId);
        return Ok(ApiResponse<object>.Success(new { AssetSha256 = sha256, Archived = true }));
    }

    [HttpGet("baselines")]
    public IActionResult Baselines() => Ok(ApiResponse<object>.Success(
        service.Baselines().Where(b => b.NodeId == NodeId && b.Active)));

    [HttpGet("baselines/{id}/asset")]
    public IActionResult Download(string id)
    {
        var baseline = service.Baseline(id);
        if (baseline.NodeId != NodeId) throw new LoadRadarException(403, "不能读取其他节点基线");
        return File(assets.Open(baseline.AssetSha256), "application/octet-stream", id + ".npz");
    }
}
