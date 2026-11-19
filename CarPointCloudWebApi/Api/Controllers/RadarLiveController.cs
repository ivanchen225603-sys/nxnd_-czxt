using System.Text.Json.Nodes;
using System.Text.RegularExpressions;
using Microsoft.AspNetCore.Mvc;

namespace CarPointCloudWebApi.Api;

/// <summary>
/// 磅房实时点云页面：代理 Worker 预览，人工输入车牌后登记来车并触发一次双雷达采集，按流程查询进度。
/// </summary>
[ApiController]
[Route("api/v1/radar-live")]
public sealed class RadarLiveController(
    RadarLiveClient radarClient,
    IIntegrationRepository repository,
    IEmptyVehicleProfileRepository emptyProfiles,
    IntegrationQueryService queryService,
    IConfiguration configuration,
    ILogger<RadarLiveController> logger) : ControllerBase
{
    private static readonly Regex IdempotencyKeyRegex = new("^[A-Za-z0-9_\\-:.]{1,64}$", RegexOptions.Compiled);

    [HttpGet("status")]
    [RequireRole(Roles = "admin,viewer")]
    public async Task<ActionResult<ApiResponse<JsonNode>>> GetStatus(CancellationToken cancellationToken) =>
        Relay(await radarClient.GetLiveStatusAsync(cancellationToken), "雷达服务状态不可用");

    [HttpGet("fused-frame")]
    [RequireRole(Roles = "admin,viewer")]
    public async Task<ActionResult<ApiResponse<JsonNode>>> GetFusedFrame(
        [FromQuery] long afterVersion, CancellationToken cancellationToken) =>
        afterVersion < 0
            ? BadRequest(ApiResponse<JsonNode>.Fail(400, "afterVersion 不能为负数"))
            : Relay(await radarClient.GetFusedFrameAsync(afterVersion, cancellationToken), "雷达预览不可用");

    [HttpGet("completed-frame")]
    [RequireRole(Roles = "admin,viewer")]
    public async Task<ActionResult<ApiResponse<JsonNode>>> GetCompletedFrame(
        [FromQuery] long afterVersion, [FromServices] RadarCompletionClient completion, CancellationToken cancellationToken) =>
        afterVersion < 0
            ? BadRequest(ApiResponse<JsonNode>.Fail(400, "afterVersion 不能为负数"))
            : Relay(await completion.GetAsync(afterVersion, cancellationToken), "车厢补点不可用");

    [HttpPost("captures")]
    [RequireRole(Roles = "admin")]
    public async Task<ActionResult<ApiResponse<ManualCaptureResponse>>> CreateCapture(
        [FromBody] ManualCaptureRequest? request, CancellationToken cancellationToken)
    {
        if (!ManualLicensePlate.TryNormalize(request?.LicensePlate, out var licensePlate, out var plateError))
            return BadRequest(ApiResponse<ManualCaptureResponse>.Fail(400, plateError));
        var bodyKey = Normalize(request!.IdempotencyKey);
        var headerKey = Normalize(Request.Headers["Idempotency-Key"].ToString());
        if (bodyKey is not null && headerKey is not null && bodyKey != headerKey)
            return BadRequest(ApiResponse<ManualCaptureResponse>.Fail(400, "Idempotency-Key 请求头与请求体 idempotencyKey 不一致"));
        var idempotencyKey = bodyKey ?? headerKey;
        if (idempotencyKey is not null && !IdempotencyKeyRegex.IsMatch(idempotencyKey))
            return BadRequest(ApiResponse<ManualCaptureResponse>.Fail(400,
                "Idempotency-Key 格式非法，要求 1-64 位字母、数字、下划线、连字符、冒号或点"));

        var eventId = $"manual_{idempotencyKey ?? Guid.NewGuid().ToString("N")}";
        if (idempotencyKey is not null && repository.FindVehicleArrivalByEventId(eventId) is { } replay)
            return await ReplayAsync(replay, licensePlate, cancellationToken);

        // 车牌输错会覆盖另一辆车的永久空车档案，已有档案时须人工确认。
        if (!request.ConfirmOverwriteEmptyProfile && emptyProfiles.GetByLicensePlate(licensePlate) is { } existingProfile)
            return Conflict(ApiResponse<ManualCaptureResponse>.Fail(409,
                $"车牌 {licensePlate} 已存在空车档案（扫描时间 {existingProfile.ScanTime:yyyy-MM-dd HH:mm:ss}）。" +
                "若本次仍为该车空车扫描，请确认后重试；车牌输错会覆盖另一辆车的永久档案。"));

        var now = BusinessClock.Now;
        var workflowId = $"sess_{now:yyyyMMddHHmmssfff}_{Guid.NewGuid().ToString("N")[..12]}";
        var inserted = repository.InsertVehicleArrivalRecord(new VehicleArrivalRecord
        {
            EventId = eventId,
            DeviceId = configuration["RadarLive:ManualDeviceId"] ?? "MANUAL_CONSOLE",
            CameraIdentifier = configuration["RadarLive:ArrivalCameraIdentifier"] ?? "CAM004",
            EventType = "ARRIVAL",
            SessionId = workflowId,
            LicensePlate = licensePlate,
            PlateStatus = VehicleArrivalPlateStatus.Succeeded,
            PlateSource = "MANUAL",
            PlateCapturedAt = now,
            ArrivalTime = now,
            RadarStatus = VehicleArrivalRadarStatus.Pending,
            CreatedAt = now,
            UpdatedAt = now
        });
        if (!inserted.IsNew)
            return await ReplayAsync(inserted.Record, licensePlate, cancellationToken);

        logger.LogInformation("手动采集已建立流程: workflowId={WorkflowId}, plate={LicensePlate}, operator={Operator}, note={Note}",
            workflowId, licensePlate, CurrentUsername, request.Note);
        return await DispatchCaptureAsync(inserted.Record, cancellationToken);
    }

    private async Task<ActionResult<ApiResponse<ManualCaptureResponse>>> DispatchCaptureAsync(
        VehicleArrivalRecord arrival, CancellationToken cancellationToken)
    {
        var workflowId = arrival.SessionId;
        var response = await RadarScanDispatcher.DispatchAsync(radarClient, repository, arrival.Id, workflowId, cancellationToken);
        if (!response.IsSuccess)
        {
            var status = response.StatusCode is 400 or 409 ? response.StatusCode : StatusCodes.Status502BadGateway;
            var message = response.Body?["message"]?.GetValue<string>() ?? $"雷达服务返回 HTTP {response.StatusCode}";
            logger.LogWarning("手动采集触发雷达失败: workflowId={WorkflowId}, status={StatusCode}, message={Message}",
                workflowId, response.StatusCode, message);
            return StatusCode(status, ApiResponse<ManualCaptureResponse>.Fail(status, message));
        }

        return Ok(ApiResponse<ManualCaptureResponse>.Success(new ManualCaptureResponse
        {
            WorkflowId = workflowId,
            LicensePlate = arrival.LicensePlate!,
            Duplicate = response.Body?["idempotent"]?.GetValue<bool>() == true,
            Status = response.Body?["scan"]?["state"]?.GetValue<string>() ?? "accepted",
            Message = response.Body?["message"]?.GetValue<string>() ?? "雷达已受理采集指令",
            Scan = response.Body?["scan"]?.DeepClone()
        }));
    }

    [HttpGet("captures/{workflowId}")]
    [RequireRole(Roles = "admin,viewer")]
    public async Task<ActionResult<ApiResponse<CaptureProgressResponse>>> GetCapture(
        [FromRoute] string workflowId, CancellationToken cancellationToken)
    {
        if (workflowId.Length > 128 || workflowId.Any(char.IsControl))
            return BadRequest(ApiResponse<CaptureProgressResponse>.Fail(400, "workflowId 格式无效"));
        var arrival = repository.GetVehicleArrivalBySessionId(workflowId);
        if (arrival is null)
            return NotFound(ApiResponse<CaptureProgressResponse>.Fail(404, "流程不存在"));

        // 数据库结果不依赖 Worker 在线。只按本 workflow 读取，不用同牌历史档案替代。
        var dimensions = queryService.GetVehicleDimensionByWorkflow(workflowId);
        RadarWorkerResponse? scan = null, analysis = null;
        string? unavailable = null;
        if (dimensions is null)
        {
            try
            {
                analysis = await radarClient.GetAnalysisAsync(workflowId, cancellationToken);
                // Analyzer 在返回 completed 前已经完成持久化；重新读库消除查询期间完成的时序差。
                if (analysis.Body?["stage"]?.GetValue<string>() == "completed")
                    dimensions = queryService.GetVehicleDimensionByWorkflow(workflowId);
                if (dimensions is null)
                {
                    scan = await radarClient.GetScanSnapshotAsync(workflowId, cancellationToken);
                    if (analysis.StatusCode == 404 && scan.StatusCode == 404)
                        unavailable = "Worker 中未找到本次任务，数据库也没有本次结果；可刷新查询或重新发起采集";
                    else if (!analysis.IsSuccess && analysis.StatusCode != 404 || !scan.IsSuccess && scan.StatusCode != 404)
                        unavailable = "Worker 进度查询失败；可按本流程刷新结果，不代表采集未执行";
                }
            }
            catch (Exception ex) when (ex is HttpRequestException || ex is OperationCanceledException && !cancellationToken.IsCancellationRequested)
            {
                logger.LogWarning(ex, "本次采集进度暂不可查询: workflowId={WorkflowId}", workflowId);
                dimensions = queryService.GetVehicleDimensionByWorkflow(workflowId);
                unavailable = "Worker 暂不可达，尚未查到本次数据库结果；可按本流程刷新，不代表采集未执行";
            }
        }
        var progress = CaptureProgressBuilder.Build(workflowId, arrival, scan?.IsSuccess == true ? scan.Body?["scan"] : null,
            analysis?.IsSuccess == true ? analysis.Body : null, dimensions);
        if (!progress.Terminal && unavailable is not null)
        {
            progress.Stage = CaptureStages.ProgressUnavailable;
            progress.Terminal = true;
            progress.Message = unavailable;
        }
        else if (progress.Stage == CaptureStages.Saved && scan?.Body?["analysisEnabled"]?.GetValue<bool>() == false)
        {
            progress.Stage = CaptureStages.AnalysisFailed;
            progress.Terminal = true;
            progress.Error = "analysis_disabled";
            progress.Message = "两份 PCD 已保存，但 Worker 未开启尺寸分析；请检查 CollectPcdOnly 和 EnablePythonAnalyzer 配置";
        }
        if (dimensions is not null && !string.IsNullOrWhiteSpace(arrival.LicensePlate) &&
            emptyProfiles.GetByLicensePlate(arrival.LicensePlate)?.WorkflowId == workflowId)
            progress.EmptyVehiclePointCloudUrl =
                $"/api/empty-vehicle-profiles/{Uri.EscapeDataString(arrival.LicensePlate)}/pointcloud?expectedWorkflowId={Uri.EscapeDataString(workflowId)}";
        return Ok(ApiResponse<CaptureProgressResponse>.Success(progress));
    }

    [HttpPost("captures/{workflowId}/cancel")]
    [RequireRole(Roles = "admin")]
    public async Task<ActionResult<ApiResponse<JsonNode>>> CancelCapture(
        [FromRoute] string workflowId, CancellationToken cancellationToken)
    {
        logger.LogInformation("手动取消采集会话: workflowId={WorkflowId}, operator={Operator}", workflowId, CurrentUsername);
        return Relay(await radarClient.StopScanAsync(workflowId, cancellationToken), "停止采集失败");
    }

    private string CurrentUsername => HttpContext.Items[JwtAuthMiddleware.UserInfoKey] is TokenUserInfo user
        ? user.Username
        : RadarLiveRecorderAccess.AuditActor;

    /// <summary>已受理的请求直接返回；尚未受理的请求沿用原流程派发，由 Worker 按 requestId 去重。</summary>
    private async Task<ActionResult<ApiResponse<ManualCaptureResponse>>> ReplayAsync(
        VehicleArrivalRecord existing, string licensePlate, CancellationToken cancellationToken)
    {
        if (existing.LicensePlate != licensePlate)
            return Conflict(ApiResponse<ManualCaptureResponse>.Fail(409,
                $"相同 Idempotency-Key 已用于车牌 {existing.LicensePlate}，与本次请求的 {licensePlate} 不一致"));
        if (existing.RadarStatus != VehicleArrivalRadarStatus.Succeeded)
            return await DispatchCaptureAsync(existing, cancellationToken);
        return Ok(ApiResponse<ManualCaptureResponse>.Success(new ManualCaptureResponse
        {
            WorkflowId = existing.SessionId,
            LicensePlate = licensePlate,
            Duplicate = true,
            Status = "already_accepted",
            Message = "该请求已受理，未重复触发采集；请按流程编号查询进度"
        }));
    }

    private ActionResult<ApiResponse<JsonNode>> Relay(RadarWorkerResponse response, string failureMessage)
    {
        if (response.IsSuccess)
            return Ok(ApiResponse<JsonNode>.Success(response.Body!));
        var status = response.StatusCode is 400 or 404 or 409 or 503 ? response.StatusCode : StatusCodes.Status502BadGateway;
        var message = response.Error ?? response.Body?["message"]?.GetValue<string>() ?? failureMessage;
        return StatusCode(status, ApiResponse<JsonNode>.Fail(status, message));
    }

    private static string? Normalize(string? value) => string.IsNullOrWhiteSpace(value) ? null : value.Trim();
}
