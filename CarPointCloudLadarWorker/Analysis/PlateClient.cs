using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using CarPointCloudLadarWorker.Config;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Analysis;

/// <summary>分析前按 workflowId 向 WebApi 取已确认车牌；未启用时使用配置的固定车牌。</summary>
public sealed class PlateClient(HttpClient http, IOptions<LadarCaptureOptions> options, ILogger<PlateClient> logger)
{
    private readonly LadarCaptureOptions _options = options.Value;

    public async Task<string?> FetchAsync(string workflowId, CancellationToken cancellationToken)
    {
        if (!_options.EnableApiRequests || !_options.FetchLicensePlateOnCapture)
            return string.IsNullOrWhiteSpace(_options.LicensePlate) ? null : _options.LicensePlate;

        var timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds().ToString();
        using var request = new HttpRequestMessage(HttpMethod.Get,
            $"{_options.WebApiUrl.TrimEnd('/')}/api/v1/vehicleWorkflows/{Uri.EscapeDataString(workflowId)}/licensePlate");
        request.Headers.Add("Timestamp", timestamp);
        request.Headers.Add("Signature", Convert.ToHexString(
            MD5.HashData(Encoding.UTF8.GetBytes(timestamp + _options.AppSecret))).ToLowerInvariant());
        request.Headers.Add("X-Interaction-Id", Guid.NewGuid().ToString("N"));
        request.Headers.Add("X-Caller-System", "Worker");
        request.Headers.Add("X-Workflow-Id", workflowId);

        try
        {
            using var response = await http.SendAsync(request, cancellationToken);
            var body = await response.Content.ReadAsStringAsync(cancellationToken);
            var result = response.IsSuccessStatusCode ? JsonNode.Parse(body) : null;
            var plate = result?["code"]?.GetValue<int>() is 0 or 200 ? result["data"]?["licensePlate"]?.GetValue<string>() : null;
            logger.LogInformation("按 workflow 获取车牌: workflowId={WorkflowId}, httpStatus={Status}, plate={Plate}",
                workflowId, (int)response.StatusCode, plate);
            return string.IsNullOrWhiteSpace(plate) ? null : plate;
        }
        catch (Exception error) when (!cancellationToken.IsCancellationRequested &&
            error is HttpRequestException or OperationCanceledException or JsonException or InvalidOperationException or FormatException)
        {
            // 尺寸计算和待办归档可以先开始，WebApi 自己的识牌任务仍独立执行。
            logger.LogWarning("本次预取车牌失败，继续分析: workflowId={WorkflowId}, errorType={ErrorType}",
                workflowId, error.GetType().Name);
            return null;
        }
    }
}
