using System.Text.Json;
using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>读取各载货雷达节点的 <c>/api/state</c>，按配置的通道拆成页面上的车道状态。</summary>
public sealed class LoadRadarLiveClient(IOptions<LoadRadarOptions> options, IHttpClientFactory clientFactory)
{
    public const string HttpClientName = "LoadRadarPreview";

    /// <summary>节点帧超过该时长未更新即视为离线，不再显示点云。</summary>
    private const double FreshFrameSeconds = 5;

    public async Task<LoadRadarLaneState[]> State() => options.Value.Enabled
        ? (await Task.WhenAll(options.Value.Nodes.Select(FetchNode))).SelectMany(x => x).ToArray()
        : [];

    private async Task<LoadRadarLaneState[]> FetchNode(LoadRadarNodeOptions node)
    {
        JsonElement? root = null;
        string? error = null;
        try
        {
            var json = await clientFactory.CreateClient(HttpClientName).GetStringAsync(new Uri(new Uri(node.BaseUrl!), "/api/state?history=0"));
            root = JsonDocument.Parse(json).RootElement;
        }
        catch (Exception ex) when (ex is HttpRequestException or TaskCanceledException)
        {
            // 状态页的职责就是显示节点在线与否，单个节点离线不影响其他车道。
            error = "采集服务离线或返回无效快照";
        }

        return node.Channels.Select(lane =>
        {
            JsonElement? state = null;
            if (root?.TryGetProperty("channels", out var channels) == true)
                state = channels.EnumerateArray().Cast<JsonElement?>()
                    .FirstOrDefault(c => c!.Value.GetProperty("id").GetString() == lane.ChannelId);
            var fresh = state?.TryGetProperty("frame_age_sec", out var age) == true && age.GetDouble() <= FreshFrameSeconds;
            var online = fresh && state!.Value.GetProperty("online").GetBoolean();
            if (state is { } snapshot && !online)
            {
                var cleared = snapshot.EnumerateObject()
                    .Where(p => p.Name is not ("preparation" or "volume_preparation" or "pending_compute"))
                    .ToDictionary(p => p.Name, p => (object?)p.Value);
                cleared["cloud"] = Array.Empty<float[]>();
                cleared["current_result"] = null;
                state = JsonSerializer.SerializeToElement(cleared);
            }
            return new LoadRadarLaneState(node.NodeId, lane.ChannelId, lane.Name, lane.CameraIdentifier, online, state,
                error ?? (state is null ? "快照未包含该通道" : !fresh ? "尚无新鲜帧或预览协议需升级" : null));
        }).ToArray();
    }
}
