using System.Diagnostics;
using System.Globalization;
using System.Net.Http.Headers;
using System.Text.Json;
using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>
/// 调用乙方车牌接口 <c>GET {PartnerBaseUrl}{LicensePlatePath}?cameraIdentifier=</c>（MD5 签名），每次调用写一条出站审计日志。
/// </summary>
public class PartnerLicensePlateClient(
    HttpClient httpClient,
    IOptions<IntegrationOptions> options,
    IIntegrationRepository repository,
    IConfiguration configuration,
    ILogger<PartnerLicensePlateClient> logger,
    IHttpContextAccessor httpContextAccessor)
{
    internal int MaxAttempts => Math.Clamp(options.Value.PartnerRetryCount, 1, 3);
    private static readonly JsonSerializerOptions Json = new() { PropertyNameCaseInsensitive = true };

    /// <summary>
    /// 对外"获取车牌"接口：首调前等待 PartnerFirstCallDelayMs，空结果或失败按 PartnerRetryDelayMs 重试，共 3 次；
    /// 结果写入 LicensePlateRecord。
    /// </summary>
    public async Task<List<PartnerLicensePlateData>> GetLicensePlateAsync(string cameraIdentifier, CancellationToken cancellationToken = default)
    {
        if (!PartnerContract.IsCamera(cameraIdentifier))
            throw new ArgumentException("cameraIdentifier 格式非法，要求 1-32 位字母、数字或下划线");
        var settings = options.Value;
        if (!settings.ExternalPartnerEnabled)
            throw new InvalidOperationException("合作方接入已关闭");

        var interactionId = IntegrationAudit.GetInteractionId(httpContextAccessor.HttpContext) ?? Guid.NewGuid().ToString("N");
        var requestTime = BusinessClock.Now;
        Exception lastError = new InvalidOperationException("乙方车牌接口返回空数组");
        for (var attempt = 1; attempt <= MaxAttempts; attempt++)
        {
            await Task.Delay(attempt == 1 ? settings.PartnerFirstCallDelayMs : settings.PartnerRetryDelayMs, cancellationToken);
            try
            {
                var data = await GetLicensePlateSnapshotForWorkflowAsync(cameraIdentifier!, null, interactionId, attempt, cancellationToken);
                if (data.Count > 0)
                {
                    var latest = data.OrderByDescending(item => PartnerContract.TryParseTime(item.CaptureTime, out var at) ? at : DateTime.MinValue).First();
                    SaveLicensePlateRecord(cameraIdentifier!, interactionId, requestTime, latest, null);
                    return data;
                }
                lastError = new InvalidOperationException("乙方车牌接口返回空数组");
            }
            catch (Exception ex) when (!cancellationToken.IsCancellationRequested)
            {
                lastError = ex;
            }
        }

        SaveLicensePlateRecord(cameraIdentifier!, interactionId, requestTime, null, lastError.Message);
        logger.LogError(lastError, "乙方车牌接口 {AttemptCount} 次尝试均未取得车牌: interactionId={InteractionId}, camera={Camera}",
            MaxAttempts, interactionId, cameraIdentifier);
        throw lastError;
    }

    /// <summary>调用一次乙方接口；兼容合同响应码和现场布尔成功标志。</summary>
    public async Task<List<PartnerLicensePlateData>> GetLicensePlateSnapshotForWorkflowAsync(
        string cameraIdentifier, string? workflowId, string interactionId, int attempt, CancellationToken cancellationToken)
    {
        var settings = options.Value;
        if (!PartnerContract.IsCamera(cameraIdentifier))
            throw new ArgumentException("cameraIdentifier 格式非法，要求 1-32 位字母、数字或下划线");
        if (!settings.ExternalPartnerEnabled)
            throw new InvalidOperationException("合作方接入已关闭");
        var uri = new Uri(new Uri(settings.PartnerBaseUrl.TrimEnd('/') + "/"),
            $"{settings.LicensePlatePath.TrimStart('/')}?cameraIdentifier={Uri.EscapeDataString(cameraIdentifier)}");
        using var request = new HttpRequestMessage(HttpMethod.Get, uri) { Content = new ByteArrayContent([]) };
        request.Content.Headers.ContentType = new MediaTypeHeaderValue("application/x-www-form-urlencoded");
        var timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds().ToString(CultureInfo.InvariantCulture);
        request.Headers.Add("Timestamp", timestamp);
        request.Headers.Add("Signature", ApiSignatureHelper.GenerateSignature(timestamp, settings.AppSecret));

        using var timeout = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        timeout.CancelAfter(Math.Clamp(settings.PartnerTimeoutMs, 1, 3000));
        var requestedAt = BusinessClock.Now;
        var stopwatch = Stopwatch.StartNew();
        var httpStatus = 599;
        string? body = null;
        PartnerLicensePlateResponse? parsed = null;
        List<PartnerLicensePlateData>? candidates = null;
        Exception? error = null;
        try
        {
            using var response = await httpClient.SendAsync(request, timeout.Token);
            httpStatus = (int)response.StatusCode;
            body = await response.Content.ReadAsStringAsync(timeout.Token);
            response.EnsureSuccessStatusCode();
            parsed = JsonSerializer.Deserialize<PartnerLicensePlateResponse>(body, Json);
            if (parsed is not { IsSuccessful: true, Data: not null })
                throw new InvalidOperationException($"乙方车牌接口返回失败: code={parsed?.BusinessCode}, success={parsed?.Success.ValueKind}");
            candidates = parsed.Data.Where(item => item is not null && item.CameraId == cameraIdentifier &&
                PartnerContract.IsLicensePlate(item.LicensePlate) &&
                PartnerContract.TryParseTime(item.CaptureTime, out _) &&
                !string.IsNullOrWhiteSpace(item.EnterId)).ToList();
            var observedAt = BusinessClock.Now;
            if (candidates.Count != 0 && configuration.GetValue("Database:Enabled", true))
                repository.ObservePartnerCaptures(cameraIdentifier, candidates, observedAt);
            else
                foreach (var item in candidates) item.FirstSeenAt = observedAt;
            return candidates;
        }
        catch (Exception ex)
        {
            error = ex;
            throw;
        }
        finally
        {
            var receivedAt = BusinessClock.Now;
            var elapsedMs = (int)stopwatch.ElapsedMilliseconds;
            var responseBody = AuditBodySanitizer.Sanitize(body);
            var maxBodyChars = Math.Clamp(configuration.GetValue("AuditLog:MaxBodyChars", 1024 * 1024), 1024, 1024 * 1024);
            var truncated = responseBody?.Length > maxBodyChars;
            if (truncated)
                responseBody = responseBody![..maxBodyChars];
            logger.LogInformation(
                "乙方车牌响应 camera={CameraIdentifier} workflowId={WorkflowId} interactionId={InteractionId} attempt={Attempt} " +
                "requestedAt={RequestedAt:yyyy-MM-dd HH:mm:ss.fff} receivedAt={ReceivedAt:yyyy-MM-dd HH:mm:ss.fff} " +
                "elapsedMs={ElapsedMs} httpStatus={HttpStatus} businessCode={BusinessCode} partnerSuccess={PartnerSuccess} candidateCount={CandidateCount} " +
                "errorType={ErrorType} truncated={Truncated} response={Response}",
                cameraIdentifier, workflowId, interactionId, attempt, requestedAt, receivedAt, elapsedMs, httpStatus,
                parsed?.BusinessCode, parsed?.Success.ValueKind, candidates?.Count, error?.GetType().Name, truncated, responseBody);
            await WriteAuditAsync(new ApiAccessLogRecord
            {
                RecordKind = "Access",
                LogLevel = error is null ? "info" : "warn",
                LifecycleStage = error is null ? "PartnerResponseReceived" : "PartnerAttemptFailed",
                LogMessage = $"第 {attempt} 次乙方车牌调用，HTTP {httpStatus}，有效候选 {candidates?.Count ?? 0}，耗时 {elapsedMs} ms"
                    + (truncated ? "，响应已截断" : ""),
                InteractionId = interactionId,
                IntegrationType = "LicensePlate",
                Direction = "Outbound",
                CallerSystem = "WebApi",
                TargetSystem = "Partner",
                WorkflowId = workflowId ?? httpContextAccessor.HttpContext?.Request.Headers["X-Workflow-Id"].FirstOrDefault(),
                LicensePlate = candidates is [var candidate] ? candidate.LicensePlate : null,
                CameraIdentifier = cameraIdentifier,
                AttemptNo = attempt,
                HttpMethod = "GET",
                RequestPath = uri.AbsolutePath,
                QueryString = uri.Query,
                Scheme = uri.Scheme,
                Host = uri.IsDefaultPort ? uri.Host : uri.Authority,
                TimestampHeader = timestamp,
                ResponseBody = responseBody,
                HttpStatus = httpStatus,
                BusinessCode = parsed?.BusinessCode ?? (parsed?.IsSuccessful == true ? 200 : 0),
                ElapsedMs = elapsedMs,
                ErrorMessage = error?.Message,
                CreatedAt = receivedAt
            });
        }
    }

    private async Task WriteAuditAsync(ApiAccessLogRecord record)
    {
        if (!configuration.GetValue("Database:Enabled", true))
            return;
        try
        {
            await repository.InsertApiAccessLogAsync(record);
        }
        catch (Exception ex)
        {
            // 审计是旁路记录，写库失败不能改变车牌调用结果。
            logger.LogError(ex, "保存乙方车牌交互审计日志失败: interactionId={InteractionId}", record.InteractionId);
        }
    }

    private void SaveLicensePlateRecord(
        string cameraIdentifier, string interactionId, DateTime requestTime, PartnerLicensePlateData? data, string? error)
    {
        if (!configuration.GetValue("Database:Enabled", true))
            return;
        repository.InsertLicensePlateRecord(new LicensePlateRecord
        {
            InteractionId = interactionId,
            CameraIdentifier = cameraIdentifier,
            CameraId = data?.CameraId ?? string.Empty,
            LicensePlate = data?.LicensePlate ?? string.Empty,
            CaptureTime = PartnerContract.TryParseTime(data?.CaptureTime, out var captured) ? captured : null,
            EnterId = data?.EnterId,
            RequestTime = requestTime,
            ResponseCode = data is null ? 500 : 200,
            ResponseMessage = data is null ? $"调用乙方车牌接口失败（{MaxAttempts}次尝试后仍无数据）" : "success",
            IsSuccess = data is not null,
            ErrorMessage = error,
            CreatedAt = BusinessClock.Now
        });
    }
}
