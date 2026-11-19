using System.Text.Json.Nodes;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>Worker 端点的一次调用结果。</summary>
public sealed record RadarWorkerResponse(int StatusCode, JsonNode? Body, string? Error)
{
    public bool IsSuccess => StatusCode is >= 200 and < 300;
}

/// <summary>访问双雷达 Worker（Radar:BaseUrl）的实时预览、扫描与分析端点。</summary>
public sealed class RadarLiveClient(IHttpClientFactory httpClientFactory)
{
    public const string HttpClientName = "RadarLive";

    public Task<RadarWorkerResponse> GetLiveStatusAsync(CancellationToken cancellationToken) =>
        SendAsync(HttpMethod.Get, "/api/radars/live/status", cancellationToken);

    public Task<RadarWorkerResponse> GetScanSnapshotAsync(string workflowId, CancellationToken cancellationToken) =>
        SendAsync(HttpMethod.Get, $"/api/radars/scan-once/lookup?requestId={Uri.EscapeDataString(workflowId)}", cancellationToken);

    public Task<RadarWorkerResponse> GetAnalysisAsync(string workflowId, CancellationToken cancellationToken) =>
        SendAsync(HttpMethod.Get, $"/api/radars/analysis?workflowId={Uri.EscapeDataString(workflowId)}", cancellationToken);

    public Task<RadarWorkerResponse> StartScanAsync(string workflowId, CancellationToken cancellationToken) =>
        SendAsync(HttpMethod.Post, $"/api/radars/scan-once?requestId={Uri.EscapeDataString(workflowId)}", cancellationToken);

    public Task<RadarWorkerResponse> StopScanAsync(string workflowId, CancellationToken cancellationToken) =>
        SendAsync(HttpMethod.Post, $"/api/radars/scan-once/stop?requestId={Uri.EscapeDataString(workflowId)}", cancellationToken);

    public Task<RadarWorkerResponse> GetFusedFrameAsync(long afterVersion, CancellationToken cancellationToken) =>
        SendAsync(HttpMethod.Get, $"/api/radars/live/fused-frame?afterVersion={afterVersion}", cancellationToken);

    private async Task<RadarWorkerResponse> SendAsync(HttpMethod method, string requestUri, CancellationToken cancellationToken)
    {
        using var request = new HttpRequestMessage(method, requestUri);
        if (method == HttpMethod.Post)
            request.Content = new ByteArrayContent([]);
        using var response = await httpClientFactory.CreateClient(HttpClientName).SendAsync(request, cancellationToken);
        var text = await response.Content.ReadAsStringAsync(cancellationToken);
        return new RadarWorkerResponse((int)response.StatusCode, string.IsNullOrWhiteSpace(text) ? null : JsonNode.Parse(text), null);
    }
}
