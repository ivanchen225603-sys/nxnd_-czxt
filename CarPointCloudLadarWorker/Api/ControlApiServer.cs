using System.Net;
using System.Globalization;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using CarPointCloudLadarWorker.Analysis;
using CarPointCloudLadarWorker.Config;
using CarPointCloudLadarWorker.Domain;
using CarPointCloudLadarWorker.Preview;
using CarPointCloudLadarWorker.Radar;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Api;

/// <summary>
/// 本机 HTTP 控制接口。停稳段提前留帧仅接受同机 WebApi 的签名派发。
/// </summary>
public sealed class ControlApiServer(
    IOptions<LadarCaptureOptions> options,
    ScanSessions sessions,
    RadarHost radars,
    LivePreview preview,
    AnalysisWorker analysis,
    ILogger<ControlApiServer> logger) : BackgroundService
{
    private static readonly JsonSerializerOptions Json = new() { PropertyNamingPolicy = JsonNamingPolicy.CamelCase };

    private readonly LadarCaptureOptions _options = options.Value;

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        using var listener = new HttpListener();
        listener.Prefixes.Add($"http://{_options.HttpApiHost}:{_options.HttpApiPort}/");
        listener.Start();
        logger.LogInformation("雷达 HTTP 控制服务已启动: http://{Host}:{Port}", _options.HttpApiHost, _options.HttpApiPort);
        while (true)
            await HandleAsync(await listener.GetContextAsync().WaitAsync(stoppingToken));
    }

    private async Task HandleAsync(HttpListenerContext context)
    {
        var request = context.Request;
        var path = request.Url!.AbsolutePath.ToLowerInvariant();
        logger.LogInformation("收到 HTTP 请求: {Method} {Path}", request.HttpMethod, path);
        object body;
        int status;
        try
        {
            (body, status) = Route(request.HttpMethod, path, request.QueryString, request.Headers, request.RemoteEndPoint?.Address);
        }
        catch (Exception ex)
        {
            logger.LogError(ex, "处理 HTTP 请求时发生异常");
            (body, status) = (new { success = false, message = "服务器内部错误，请联系管理员" }, 500);
        }

        var response = context.Response;
        response.StatusCode = status;
        response.ContentType = "application/json; charset=utf-8";
        try
        {
            await JsonSerializer.SerializeAsync(response.OutputStream, body, Json);
            response.Close();
        }
        catch (HttpListenerException ex)
        {
            // 调用方已断开；丢弃本次响应。
            logger.LogDebug(ex, "写入 HTTP 响应失败");
        }
    }

    private (object Body, int Status) Route(
        string method, string path, System.Collections.Specialized.NameValueCollection query,
        System.Collections.Specialized.NameValueCollection headers, IPAddress? caller)
    {
        var cycle = radars.EstimatedCycleSeconds;
        switch (method, path)
        {
            case ("POST", "/api/radars/scan-once"):
                DateTimeOffset? captureUntil = null;
                if (query["captureUntil"] is { } value)
                {
                    if (!DateTimeOffset.TryParseExact(value, "O", CultureInfo.InvariantCulture, DateTimeStyles.None, out var until))
                        return (new { success = false, message = "captureUntil 必须是带偏移的 ISO 时间" }, 400);
                    captureUntil = until;
                }
                var requestId = string.IsNullOrWhiteSpace(query["requestId"]) ? headers["Idempotency-Key"] : query["requestId"];
                var segmentId = query["segmentId"];
                var signedDispatch = IsSignedStandstillDispatch(caller, _options.AppSecret, requestId, segmentId,
                    captureUntil, headers["X-Standstill-Dispatch"]);
                var start = sessions.Start(requestId, captureUntil, segmentId, signedDispatch);
                logger.LogInformation("scan-once 指令已处理 workflowId={WorkflowId} started={Started} idempotent={Idempotent} code={Code}",
                    start.Snapshot.RequestId, start.Started, start.Idempotent, start.Code);
                return (ApiResponses.Command(start, cycle), ApiResponses.StartStatusCode(start));

            case ("POST", "/api/radars/scan-once/stop"):
                var stop = sessions.Stop("人工停止单次扫描", query["requestId"]);
                return (StopBody(stop, cycle), ApiResponses.StopStatusCode(stop));

            case ("POST", "/api/radars/scan-once/passage-ended"):
                var ended = sessions.EndPassage(query["requestId"] ?? "");
                return (ApiResponses.Command(ended, cycle), ended.Success ? 200 : 400);

            case ("POST", "/api/radar/standby"):
                return Standby(query, cycle);

            case ("GET", "/api/radars/scan-once" or "/api/radars/scan-once/status"):
                return (ApiResponses.Scan(sessions.Snapshot(), cycle), 200);

            case ("GET", "/api/radars/scan-once/lookup"):
                if (string.IsNullOrWhiteSpace(query["requestId"]))
                    return (new { success = false, message = "缺少 requestId 参数" }, 400);
                return sessions.Find(query["requestId"]!) is { } found
                    ? (new { success = true, known = true, analysisEnabled = _options.AnalysisEnabled, requestId = query["requestId"], scan = ApiResponses.Scan(found, cycle) }, 200)
                    : (new { success = false, known = false, message = "本进程没有该 requestId 的受理记录；这不代表它没有执行过" }, 404);

            case ("GET", "/api/radars/status"):
                return (radars.Statuses(DateTime.Now), 200);

            case ("GET", "/api/radar/status"):
                var camera = query["camera"];
                var statuses = radars.Statuses(DateTime.Now);
                if (string.IsNullOrWhiteSpace(camera))
                    return (statuses, 200);
                return statuses.FirstOrDefault(x => string.Equals(x.CameraIdentifier, camera, StringComparison.OrdinalIgnoreCase)) is { } one
                    ? (one, 200)
                    : (new { success = false, message = $"雷达未找到: {camera}" }, 404);

            case ("GET", "/api/radars/live/status"):
                var now = DateTime.Now;
                return (new
                {
                    success = true,
                    timestamp = now,
                    scan = ApiResponses.Scan(sessions.Snapshot(), cycle),
                    preview = preview.GetStatus(),
                    radars = radars.Statuses(now)
                }, 200);

            case ("GET", "/api/radars/live/fused-frame"):
                var afterText = query["afterVersion"];
                long afterVersion = 0;
                if (!string.IsNullOrWhiteSpace(afterText) && (!long.TryParse(afterText, out afterVersion) || afterVersion < 0))
                    return (new { success = false, message = "afterVersion 必须是非负整数" }, 400);
                var previewStatus = preview.GetStatus();
                return preview.GetFrameAfter(afterVersion) is { } frame
                    ? (ApiResponses.FusedFrame(frame, previewStatus), 200)
                    : (new { success = true, updated = false, sessionId = previewStatus.SessionId, version = previewStatus.LatestVersion, preview = previewStatus }, 200);

            case ("GET", "/api/radars/analysis"):
                var workflowId = query["workflowId"];
                if (string.IsNullOrWhiteSpace(workflowId))
                    return (new { success = false, message = "缺少 workflowId 参数" }, 400);
                return analysis.Find(workflowId.Trim()) is { } record
                    ? (ApiResponses.Analysis(record), 200)
                    : (new { success = false, message = "该流程尚无分析记录" }, 404);

            case ("GET", "/api/health"):
                return (new { status = "ok", timestamp = DateTime.Now }, 200);

            default:
                return (new { success = false, message = $"未知的端点 {method} {path}" }, 404);
        }
    }

    /// <summary>兼容旧待机接口：持续扫描模式下雷达不待机，mode=1 只用于取消当前会话。</summary>
    private (object Body, int Status) Standby(System.Collections.Specialized.NameValueCollection query, double? cycle)
    {
        if (query["mode"] is not ("0" or "1"))
            return (new { success = false, message = "缺少或无效的 mode 参数，请提供 0 或 1" }, 400);
        if (query["mode"] == "0")
            return (new { success = false, message = "接口单次扫描模式下禁止直接退出待机，请调用 POST /api/radars/scan-once" }, 409);
        if (!string.IsNullOrWhiteSpace(query["camera"]))
            return (new { success = false, message = "接口单次扫描模式下不能单独控制一台雷达，请停止整个会话" }, 409);
        var snapshot = sessions.Snapshot();
        if (snapshot.State != ScanState.Scanning)
            return (new { success = false, message = "持续扫描模式下雷达不进入待机；当前没有可取消的扫描会话", scan = ApiResponses.Scan(snapshot, cycle) }, 409);

        var stop = sessions.Stop("通过 standby mode=1 人工停止单次扫描");
        return (StopBody(stop, cycle), ApiResponses.StopStatusCode(stop));
    }

    internal static bool IsSignedStandstillDispatch(IPAddress? caller, string? secret, string? workflow,
        string? segment, DateTimeOffset? deadline, string? signature)
    {
        if (caller is null || !IPAddress.IsLoopback(caller.IsIPv4MappedToIPv6 ? caller.MapToIPv4() : caller) ||
            string.IsNullOrWhiteSpace(secret) || string.IsNullOrWhiteSpace(workflow) || string.IsNullOrWhiteSpace(segment) ||
            deadline is null || signature?.Length != 64)
            return false;
        var message = FormattableString.Invariant($"standstill-scan\n{workflow}\n{segment}\n{deadline.Value.ToUnixTimeMilliseconds()}");
        var mac = HMACSHA256.HashData(Encoding.UTF8.GetBytes(secret), Encoding.UTF8.GetBytes(message));
        var expected = Convert.ToHexString(mac).ToLowerInvariant();
        return CryptographicOperations.FixedTimeEquals(Encoding.ASCII.GetBytes(expected), Encoding.ASCII.GetBytes(signature));
    }

    private static object StopBody(ScanCommandResult r, double? cycle) =>
        new { success = r.Success, message = r.Message, scan = ApiResponses.Scan(r.Snapshot, cycle) };
}
