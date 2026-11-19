using System.Globalization;
using System.Net.Http.Json;
using System.Text.Json.Nodes;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>
/// 磅房页面的车厢补点参考画面：取 Worker 最新融合帧交给 Analyzer <c>/preview-completion</c>，结果仅供显示。
/// </summary>
/// <remarks>补点计算较重：同一时刻只算一帧，最多每 5 秒一次；画面超过 10 秒或 Worker 会话变化即不再显示。</remarks>
public sealed class RadarCompletionClient(RadarLiveClient radar, IHttpClientFactory clients)
{
    public const string HttpClientName = "RadarCompletion";

    private readonly SemaphoreSlim _gate = new(1, 1);
    private JsonNode? _cached;
    private DateTimeOffset _nextAttempt;

    public async Task<RadarWorkerResponse> GetAsync(long afterVersion, CancellationToken cancellationToken)
    {
        if (!await _gate.WaitAsync(0, cancellationToken))
            return Unavailable("补点正在计算，请稍后刷新");
        try
        {
            var status = await radar.GetLiveStatusAsync(cancellationToken);
            if (status.Body?["preview"]?["available"]?.GetValue<bool>() != true)
                return Unavailable("双雷达实时画面不可用，已清空补点");
            if (status.Body["scan"]?["state"]?.GetValue<string>() != "idle")
                return Unavailable("正在正式采集，暂缓参考补点");
            var session = status.Body["preview"]!["sessionId"]!.GetValue<string>();

            if (_cached is not null && Fresh(_cached, session))
            {
                if (_cached["version"]!.GetValue<long>() > afterVersion)
                    return new(200, _cached.DeepClone(), null);
                if (DateTimeOffset.UtcNow < _nextAttempt)
                    return new(200, new JsonObject { ["updated"] = false, ["sessionId"] = session }, null);
            }
            if (DateTimeOffset.UtcNow < _nextAttempt)
                return Unavailable("补点刷新受限，等待下一帧");
            _nextAttempt = DateTimeOffset.UtcNow.AddSeconds(5);

            var source = await radar.GetFusedFrameAsync(0, cancellationToken);
            if (source.Body?["updated"]?.GetValue<bool>() != true || !Fresh(source.Body, session))
                return Unavailable("没有新鲜的双雷达配对画面");
            var frame = source.Body.DeepClone();
            if (_cached?["version"]?.GetValue<long>() == frame["version"]!.GetValue<long>())
                return new(200, new JsonObject { ["updated"] = false, ["sessionId"] = session }, null);

            using var response = await clients.CreateClient(HttpClientName).PostAsJsonAsync("/preview-completion", new
            {
                points = frame["points"]!.GetValue<string>(),
                pointsEncoding = frame["pointsEncoding"]!.GetValue<string>(),
                pointCountA = frame["pointCountA"]!.GetValue<int>(),
                pointCountB = frame["pointCountB"]!.GetValue<int>()
            }, cancellationToken);
            if (!response.IsSuccessStatusCode)
                return Unavailable(response.StatusCode == System.Net.HttpStatusCode.BadRequest
                    ? "当前点云不足以可靠提取车厢，暂不显示补点"
                    : "补点服务忙或不可用，正式计算结果不受此提示替代");

            var result = (await response.Content.ReadFromJsonAsync<JsonNode>(cancellationToken))!;
            frame["points"] = result["points"]!.DeepClone();
            frame["pointCountA"] = result["measuredPointCount"]!.DeepClone();
            frame["pointCountB"] = 0;
            frame["syntheticPointCount"] = result["syntheticPointCount"]!.DeepClone();
            frame["previewPointCount"] = frame["pointCountA"]!.GetValue<int>() + frame["syntheticPointCount"]!.GetValue<int>();
            frame["wallCompletion"] = result["wallCompletion"]?.DeepClone();
            if (result["displayMesh"] is { } mesh)
                frame["displayMesh"] = mesh.DeepClone();
            frame["loadStatus"] = result["loadStatus"]?.DeepClone();
            frame["referenceOnly"] = true;
            frame["expiresAt"] = Oldest(frame).AddSeconds(10).ToString("O");
            _cached = frame;
            return new(200, frame.DeepClone(), null);
        }
        finally
        {
            _gate.Release();
        }
    }

    private static DateTimeOffset Oldest(JsonNode frame)
    {
        var a = DateTimeOffset.Parse(frame["capturedAtA"]!.GetValue<string>(), CultureInfo.InvariantCulture);
        var b = DateTimeOffset.Parse(frame["capturedAtB"]!.GetValue<string>(), CultureInfo.InvariantCulture);
        return a < b ? a : b;
    }

    private static bool Fresh(JsonNode frame, string session) =>
        frame["sessionId"]?.GetValue<string>() == session && DateTimeOffset.UtcNow - Oldest(frame) < TimeSpan.FromSeconds(10);

    private static RadarWorkerResponse Unavailable(string reason) => new(503, null, reason);
}
