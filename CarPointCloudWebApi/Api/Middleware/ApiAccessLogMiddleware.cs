using System.Diagnostics;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;

namespace CarPointCloudWebApi.Api;

/// <summary>
/// 第三方交互审计：对车牌、来车、尺寸、煤量接口写入"请求到达 / 查询执行 / 请求结束"记录到 ApiAccessLog，
/// 供日志页面按 interactionId 串联。其他路径直接放行。
/// </summary>
public class ApiAccessLogMiddleware(
    RequestDelegate next,
    IServiceScopeFactory scopeFactory,
    IConfiguration configuration,
    ILogger<ApiAccessLogMiddleware> logger,
    PartnerQueryExecutor? partnerQueries = null)
{
    private static readonly HashSet<string> SensitiveHeaders = new(StringComparer.OrdinalIgnoreCase)
    {
        "Authorization", "Cookie", "Set-Cookie", "Proxy-Authorization", "Signature", "X-Api-Key", "X-Load-Radar-Key",
        RadarLiveRecorderAccess.HeaderName, "X-Auth-Token"
    };

    private readonly bool _databaseEnabled = configuration.GetValue("Database:Enabled", true);
    private readonly bool _captureBodies = configuration.GetValue("AuditLog:CaptureBodies", false);
    private readonly bool _capturePartnerQueryBodies = configuration.GetValue("AuditLog:CapturePartnerQueryBodies", false);
    private readonly bool _captureHeaders = configuration.GetValue("AuditLog:CaptureHeaders", false);
    private readonly int _maxBodyChars = configuration.GetValue("AuditLog:MaxBodyChars", 1024 * 1024);
    private readonly PartnerQueryExecutor _partnerQueries = partnerQueries ?? new(scopeFactory);

    public async Task InvokeAsync(HttpContext context)
    {
        var request = context.Request;
        var integrationType = IntegrationAudit.ResolveIntegrationType(request.Path);
        if (integrationType is null)
        {
            await next(context);
            return;
        }

        var interactionId = IntegrationAudit.GetOrCreateInteractionId(context);
        var callerSystem = IntegrationAudit.ResolveCallerSystem(context, integrationType);
        var stopwatch = Stopwatch.StartNew();
        var isPartnerQuery = HttpMethods.IsGet(request.Method) && PartnerResponseFactory.IsPartnerQueryPath(request.Path);
        var captureQueryBodies = _capturePartnerQueryBodies && isPartnerQuery && PartnerResponseFactory.IsPartnerQuery(request.Path);
        var captureBodies = _captureBodies || captureQueryBodies;
        var auditQuery = isPartnerQuery ? SanitizePartnerQuery(request) : request.QueryString.Value;
        var requestBodyText = captureBodies && !isPartnerQuery ? await ReadRequestBodyAsync(request) : null;
        var auditBudget = TimeSpan.FromMilliseconds(400);

        async Task SaveForRequestAsync(ApiAccessLogRecord record)
        {
            if (!isPartnerQuery)
            {
                await SaveAsync(record);
                return;
            }
            if (auditBudget <= TimeSpan.Zero)
                return;
            var started = Stopwatch.GetTimestamp();
            await SaveAsync(record, auditBudget, context.RequestAborted);
            auditBudget -= Stopwatch.GetElapsedTime(started);
        }

        logger.LogInformation("{LifecycleStage} {IntegrationType} interactionId={InteractionId} {Method} {Path}{Query} caller={CallerSystem}",
            "RequestReceived", integrationType, interactionId, request.Method, request.Path, auditQuery, callerSystem);
        await SaveForRequestAsync(Lifecycle(context, interactionId, integrationType, callerSystem, "debug", "RequestReceived",
            $"收到{DisplayName(integrationType)}请求，开始进入接口处理链路", auditQuery));

        var originalBody = context.Response.Body;
        await using var responseBody = new MemoryStream();
        context.Response.Body = responseBody;
        string? errorMessage = null;
        try
        {
            await next(context);
        }
        catch (Exception ex)
        {
            errorMessage = ex.ToString();
            throw;
        }
        finally
        {
            responseBody.Position = 0;
            var responseBodyText = captureBodies ? await ReadResponseBodyAsync(responseBody, context.Response.ContentType) : null;
            responseBody.Position = 0;
            await responseBody.CopyToAsync(originalBody);
            context.Response.Body = originalBody;

            if (IntegrationAudit.GetQueryDiagnostics(context) is { Length: > 0 } diagnostics)
            {
                logger.LogInformation("{LifecycleStage} {IntegrationType} interactionId={InteractionId} {Diagnostics}",
                    "QueryExecuted", integrationType, interactionId, diagnostics);
                await SaveForRequestAsync(Lifecycle(context, interactionId, integrationType, callerSystem, "info", "QueryExecuted",
                    diagnostics, auditQuery));
            }

            var status = context.Response.StatusCode;
            var failed = errorMessage is not null || status >= 500;
            var record = new ApiAccessLogRecord
            {
                RecordKind = "Access",
                LogLevel = failed ? "error" : status >= 400 ? "warn" : "info",
                LifecycleStage = failed ? "RequestFailed" : status >= 400 ? "RequestRejected" : "RequestCompleted",
                LogMessage = $"{DisplayName(integrationType)}请求{(failed ? "异常结束" : status >= 400 ? "被拒绝" : "处理完成")}，" +
                             $"HTTP {status}，耗时 {stopwatch.ElapsedMilliseconds} ms",
                InteractionId = interactionId,
                IntegrationType = integrationType,
                Direction = "Inbound",
                CallerSystem = callerSystem,
                TargetSystem = "WebApi",
                WorkflowId = IntegrationAudit.FirstNonEmpty(Header(context, "X-Workflow-Id"),
                    Convert.ToString(request.RouteValues["workflowId"]), request.Query["workflowId"].FirstOrDefault(),
                    IntegrationAudit.JsonString(requestBodyText, "workflowId")),
                EventId = IntegrationAudit.FirstNonEmpty(Header(context, "X-Event-Id"), request.Query["eventId"].FirstOrDefault(),
                    IntegrationAudit.JsonString(requestBodyText, "eventId")),
                LicensePlate = IntegrationAudit.FirstNonEmpty(request.Query["licensePlate"].FirstOrDefault(),
                                   IntegrationAudit.JsonString(requestBodyText, "licensePlate")) ??
                               (integrationType == "LicensePlate" ? IntegrationAudit.JsonFirstString(responseBodyText, "licensePlate") : null),
                CameraIdentifier = IntegrationAudit.FirstNonEmpty(request.Query["cameraIdentifier"].FirstOrDefault(),
                    IntegrationAudit.JsonString(requestBodyText, "cameraIdentifier", "cameraId")),
                AttemptNo = 1,
                HttpMethod = request.Method,
                RequestPath = request.Path,
                QueryString = auditQuery,
                Scheme = request.Scheme,
                Host = request.Host.HasValue ? request.Host.Value : null,
                TraceIdentifier = context.TraceIdentifier,
                ClientIp = context.Connection.RemoteIpAddress?.ToString(),
                UserAgent = Header(context, "User-Agent"),
                RequestContentType = request.ContentType,
                TimestampHeader = Header(context, "Timestamp"),
                SignatureHeader = HashForAudit(Header(context, "Signature")),
                IdempotencyKeyHeader = Header(context, "Idempotency-Key") ?? IntegrationAudit.JsonString(requestBodyText, "idempotencyKey"),
                RequestHeadersJson = _captureHeaders ? SerializeHeaders(request.Headers) : null,
                RequestBody = requestBodyText,
                ResponseContentType = context.Response.ContentType,
                ResponseHeadersJson = _captureHeaders ? SerializeHeaders(context.Response.Headers) : null,
                ResponseBody = responseBodyText,
                RequestContentLength = request.ContentLength,
                ResponseContentLength = responseBody.Length,
                HttpStatus = status,
                BusinessCode = IntegrationAudit.JsonInt(responseBodyText, "code") ?? status,
                ElapsedMs = (int)stopwatch.ElapsedMilliseconds,
                ErrorMessage = Truncate(errorMessage, 1024),
                CreatedAt = BusinessClock.Now
            };
            if (captureQueryBodies)
            {
                var dataCount = ResponseDataCount(responseBodyText);
                record.LogMessage += $"，实际出参 data 条数={dataCount?.ToString() ?? "无数组数据或正文已截断"}";
                logger.LogInformation(
                    "{LifecycleStage} {IntegrationType} interactionId={InteractionId} {Method} {Path} query={Query} status={StatusCode} dataCount={DataCount} responseBody={ResponseBody}",
                    "QueryResponse", integrationType, interactionId, record.HttpMethod, record.RequestPath,
                    record.QueryString, record.HttpStatus, dataCount, responseBodyText);
            }
            await SaveForRequestAsync(record);
            logger.Log(failed ? LogLevel.Error : status >= 400 ? LogLevel.Warning : LogLevel.Information,
                "API access {Method} {Path}{Query} {StatusCode} {ElapsedMs}ms interactionId={InteractionId} stage={LifecycleStage} trace={TraceIdentifier}",
                record.HttpMethod, record.RequestPath, record.QueryString, record.HttpStatus, stopwatch.ElapsedMilliseconds,
                record.InteractionId, record.LifecycleStage, record.TraceIdentifier);
        }
    }

    private async Task SaveAsync(ApiAccessLogRecord record, TimeSpan? budget = null,
        CancellationToken requestAborted = default)
    {
        if (!_databaseEnabled)
            return;
        try
        {
            if (budget is { } bounded)
            {
                await _partnerQueries.WriteAuditAsync(record, bounded, requestAborted);
                return;
            }
            using var scope = scopeFactory.CreateScope();
            await scope.ServiceProvider.GetRequiredService<IIntegrationRepository>().InsertApiAccessLogAsync(record);
        }
        catch (Exception ex)
        {
            // 审计是旁路记录，写库失败不能改变业务响应。
            logger.LogError(ex, "第三方交互审计日志写入失败：stage={LifecycleStage} interactionId={InteractionId}",
                record.LifecycleStage, record.InteractionId);
        }
    }

    private static ApiAccessLogRecord Lifecycle(HttpContext context, string interactionId, string integrationType,
        string? callerSystem, string level, string stage, string message, string? auditQuery) => new()
        {
            RecordKind = "Lifecycle",
            LogLevel = level,
            LifecycleStage = stage,
            LogMessage = message,
            InteractionId = interactionId,
            IntegrationType = integrationType,
            Direction = "Inbound",
            CallerSystem = callerSystem,
            TargetSystem = "WebApi",
            WorkflowId = Header(context, "X-Workflow-Id") ?? context.Request.Query["workflowId"].FirstOrDefault(),
            EventId = Header(context, "X-Event-Id") ?? context.Request.Query["eventId"].FirstOrDefault(),
            LicensePlate = context.Request.Query["licensePlate"].FirstOrDefault(),
            CameraIdentifier = context.Request.Query["cameraIdentifier"].FirstOrDefault(),
            AttemptNo = 1,
            HttpMethod = context.Request.Method,
            RequestPath = context.Request.Path,
            QueryString = auditQuery,
            Scheme = context.Request.Scheme,
            Host = context.Request.Host.HasValue ? context.Request.Host.Value : null,
            TraceIdentifier = context.TraceIdentifier,
            ClientIp = context.Connection.RemoteIpAddress?.ToString(),
            UserAgent = Header(context, "User-Agent"),
            RequestContentType = context.Request.ContentType,
            TimestampHeader = Header(context, "Timestamp"),
            SignatureHeader = HashForAudit(Header(context, "Signature")),
            IdempotencyKeyHeader = Header(context, "Idempotency-Key"),
            RequestContentLength = context.Request.ContentLength,
            CreatedAt = BusinessClock.Now
        };

    private static string DisplayName(string integrationType) => integrationType switch
    {
        "LicensePlate" => "获取车牌号",
        "VehicleDimensions" => "获取车辆尺寸",
        "CoalVolume" => "获取煤炭体积",
        "VehicleArrival" => "来车通知",
        _ => integrationType
    };

    private static int? ResponseDataCount(string? body)
    {
        if (string.IsNullOrWhiteSpace(body))
            return null;
        try
        {
            using var document = JsonDocument.Parse(body);
            return document.RootElement.TryGetProperty("data", out var data) && data.ValueKind == JsonValueKind.Array
                ? data.GetArrayLength()
                : null;
        }
        catch (JsonException)
        {
            // 记录正文超过 MaxBodyChars 会被截断，此时无法计数。
            return null;
        }
    }

    private static string? SanitizePartnerQuery(HttpRequest request)
    {
        var query = new Microsoft.AspNetCore.Http.Extensions.QueryBuilder();
        foreach (var item in request.Query)
            foreach (var value in item.Value)
                query.Add(item.Key, AuditBodySanitizer.IsSensitiveName(item.Key) ? "***REDACTED***" : value ?? "");
        return Truncate(query.ToQueryString().Value, 1024);
    }

    private async Task<string?> ReadRequestBodyAsync(HttpRequest request)
    {
        if (request.ContentLength is null or 0)
            return null;
        if (!IsTextContent(request.ContentType))
            return $"[body skipped, contentType={request.ContentType}, length={request.ContentLength}]";
        request.EnableBuffering();
        using var reader = new StreamReader(request.Body, Encoding.UTF8, false, leaveOpen: true);
        var text = await reader.ReadToEndAsync();
        request.Body.Position = 0;
        return Truncate(AuditBodySanitizer.Sanitize(text), _maxBodyChars);
    }

    private async Task<string?> ReadResponseBodyAsync(Stream responseBody, string? contentType)
    {
        if (responseBody.Length == 0)
            return null;
        if (!IsTextContent(contentType))
            return $"[body skipped, contentType={contentType}, length={responseBody.Length}]";
        using var reader = new StreamReader(responseBody, Encoding.UTF8, false, leaveOpen: true);
        return Truncate(AuditBodySanitizer.Sanitize(await reader.ReadToEndAsync()), _maxBodyChars);
    }

    private static string? SerializeHeaders(IHeaderDictionary headers) => Truncate(JsonSerializer.Serialize(
        headers.ToDictionary(x => x.Key, x => SensitiveHeaders.Contains(x.Key) ? "***REDACTED***" : x.Value.ToString())), 16 * 1024);

    private static string? Header(HttpContext context, string name) =>
        context.Request.Headers.TryGetValue(name, out var value) ? value.ToString() : null;

    private static string? Truncate(string? value, int maxChars) =>
        value is null || value.Length <= maxChars ? value : value[..maxChars] + $"...[truncated, originalLength={value.Length}]";

    private static bool IsTextContent(string? contentType) =>
        string.IsNullOrWhiteSpace(contentType) ||
        new[] { "json", "text", "xml", "urlencoded" }.Any(kind => contentType.Contains(kind, StringComparison.OrdinalIgnoreCase));

    private static string? HashForAudit(string? raw) =>
        string.IsNullOrWhiteSpace(raw) ? null : Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(raw)))[..16].ToLowerInvariant();
}
