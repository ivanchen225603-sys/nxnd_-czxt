using System.Diagnostics;
using System.Text.Json;
using System.Text.RegularExpressions;
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
    private const int MaxAttempts = 3;
    private static readonly Regex CameraIdentifierRegex = new("^[A-Za-z0-9_]{1,32}$", RegexOptions.Compiled);
    private static readonly JsonSerializerOptions Json = new() { PropertyNameCaseInsensitive = true };

    /// <summary>
    /// 对外"获取车牌"接口：首调前等待 PartnerFirstCallDelayMs，空结果或失败按 PartnerRetryDelayMs 重试，共 3 次；
    /// 结果写入 LicensePlateRecord。
    /// </summary>
    public async Task<List<PartnerLicensePlateData>> GetLicensePlateAsync(string cameraIdentifier, CancellationToken cancellationToken = default)
    {
        if (!CameraIdentifierRegex.IsMatch(cameraIdentifier ?? string.Empty))
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
            catch (Exception ex) when (attempt < MaxAttempts && !cancellationToken.IsCancellationRequested)
            {
                lastError = ex;
            }
        }

        SaveLicensePlateRecord(cameraIdentifier!, interactionId, requestTime, null, lastError.Message);
        logger.LogError(lastError, "乙方车牌接口 {AttemptCount} 次尝试均未取得车牌: interactionId={InteractionId}, camera={Camera}",
            MaxAttempts, interactionId, cameraIdentifier);
        throw lastError;
    }

    /// <summary>调用一次乙方接口，超时 PartnerTimeoutMs；非 2xx 或 code≠200 抛出异常。</summary>
    public async Task<List<PartnerLicensePlateData>> GetLicensePlateSnapshotForWorkflowAsync(
        string cameraIdentifier, string? workflowId, string interactionId, int attempt, CancellationToken cancellationToken)
    {
        var settings = options.Value;
        var uri = new Uri(new Uri(settings.PartnerBaseUrl.TrimEnd('/') + "/"),
            $"{settings.LicensePlatePath.TrimStart('/')}?cameraIdentifier={Uri.EscapeDataString(cameraIdentifier)}");
        using var request = new HttpRequestMessage(HttpMethod.Get, uri);
        var timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds().ToString();
        request.Headers.Add("Timestamp", timestamp);
        request.Headers.Add("Signature", ApiSignatureHelper.GenerateSignature(timestamp, settings.AppSecret));

        using var timeout = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        timeout.CancelAfter(settings.PartnerTimeoutMs);
        var stopwatch = Stopwatch.StartNew();
        var httpStatus = 599;
        string? body = null;
        PartnerLicensePlateResponse? parsed = null;
        Exception? error = null;
        try
        {
            using var response = await httpClient.SendAsync(request, timeout.Token);
            httpStatus = (int)response.StatusCode;
            body = await response.Content.ReadAsStringAsync(timeout.Token);
            response.EnsureSuccessStatusCode();
            parsed = JsonSerializer.Deserialize<PartnerLicensePlateResponse>(body, Json)!;
            if (parsed.Code != 200 || parsed.Data is null)
                throw new InvalidOperationException($"乙方车牌接口返回失败: code={parsed.Code}");
            return parsed.Data;
        }
        catch (Exception ex)
        {
            error = ex;
            throw;
        }
        finally
        {
            await WriteAuditAsync(new ApiAccessLogRecord
            {
                RecordKind = "Access",
                LogLevel = error is null ? "info" : "warn",
                LifecycleStage = error is null ? "PartnerResponseReceived" : "PartnerAttemptFailed",
                LogMessage = $"第 {attempt} 次乙方车牌接口调用{(error is null ? "完成" : "失败")}，HTTP {httpStatus}，耗时 {stopwatch.ElapsedMilliseconds} ms",
                InteractionId = interactionId,
                IntegrationType = "LicensePlate",
                Direction = "Outbound",
                CallerSystem = "WebApi",
                TargetSystem = "Partner",
                WorkflowId = workflowId ?? httpContextAccessor.HttpContext?.Request.Headers["X-Workflow-Id"].FirstOrDefault(),
                LicensePlate = parsed?.Data?.FirstOrDefault()?.LicensePlate,
                CameraIdentifier = cameraIdentifier,
                AttemptNo = attempt,
                HttpMethod = "GET",
                RequestPath = uri.AbsolutePath,
                QueryString = uri.Query,
                Scheme = uri.Scheme,
                Host = uri.IsDefaultPort ? uri.Host : uri.Authority,
                TimestampHeader = timestamp,
                ResponseBody = configuration.GetValue("AuditLog:CaptureBodies", false) ? AuditBodySanitizer.Sanitize(body) : null,
                HttpStatus = httpStatus,
                BusinessCode = parsed?.Code ?? 0,
                ElapsedMs = (int)stopwatch.ElapsedMilliseconds,
                ErrorMessage = error?.Message,
                CreatedAt = BusinessClock.Now
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
