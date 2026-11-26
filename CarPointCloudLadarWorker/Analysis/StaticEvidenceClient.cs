using System.Net.Http.Json;
using System.Text.Json;
using System.Text.Json.Nodes;
using CarPointCloudLadarWorker.Config;
using CarPointCloudLadarWorker.Domain;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Analysis;

internal sealed record StaticDecision(string Status, string? Reason = null);

/// <summary>向同机 WebApi 登记采集证据，重复提交同一对时读取最新确认结果。</summary>
public sealed class StaticEvidenceClient(HttpClient http, IOptions<LadarCaptureOptions> options)
{
    internal async Task<StaticDecision> CheckAsync(PairedPcdCapture pair, CancellationToken token)
    {
        var a = pair.Primary.StaticEvidence;
        var b = pair.Secondary.StaticEvidence;
        if (a is null || b is null || a.Window != b.Window || string.IsNullOrWhiteSpace(a.Window.SegmentId))
            return new("insufficient", "缺少同一停稳段的自动派发信息");
        if (!options.Value.EnableApiRequests || string.IsNullOrWhiteSpace(options.Value.AppSecret))
            return new("insufficient", "Worker 未配置 WebApi 证据回传或签名密钥");

        try
        {
            using var request = WorkflowRequest.Create(options.Value, HttpMethod.Post, pair.WorkflowId, "staticEvidence");
            request.Content = JsonContent.Create(new
            {
                pairId = pair.VehicleCaptureId, segmentId = a.Window.SegmentId,
                primary = new { pair.Primary.CameraIdentifier, a.CallbackUtc, a.ScanStartUtc, a.ScanEndUtc },
                secondary = new { pair.Secondary.CameraIdentifier, b.CallbackUtc, b.ScanStartUtc, b.ScanEndUtc }
            });
            using var response = await http.SendAsync(request, token);
            if (!response.IsSuccessStatusCode)
                return new("pending", $"静态确认接口 HTTP {(int)response.StatusCode}");
            var json = JsonNode.Parse(await response.Content.ReadAsStringAsync(token));
            if (json?["code"]?.GetValue<int>() is not (0 or 200))
                return new("pending", "静态确认接口返回异常");
            var data = json["data"];
            var status = data?["status"]?.GetValue<string>();
            if (status == "confirmed" &&
                (a.ScanStartUtc is null || a.ScanEndUtc is null || b.ScanStartUtc is null || b.ScanEndUtc is null))
                return new("insufficient", "缺少现场验证的完整扫描时间界限");
            return status is "confirmed" or "rejected" or "insufficient" or "pending"
                ? new(status, data?["reason"]?.GetValue<string>()) : new("pending", "无法识别静态确认回执");
        }
        catch (Exception ex) when (!token.IsCancellationRequested &&
            ex is HttpRequestException or OperationCanceledException or JsonException or InvalidOperationException or FormatException)
        {
            return new("pending", $"静态确认回执未取得：{ex.GetType().Name}");
        }
    }
}
