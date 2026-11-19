using System.Net.Http.Json;
using System.Text.Json;
using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>管理员人工命令：采空车基线候选或重测。</summary>
public sealed record LoadRadarCommandRequest(string RequestId, string Kind, bool ConfirmedEmpty = false);

/// <summary>采集端返回的命令状态。</summary>
public sealed record LoadRadarCommandStatus(string RequestId, string ChannelId, string PassageId,
    string Kind, string Status, string? MeasurementId, string? Error);

/// <summary>向测量所属节点的 <c>/api/integration/commands</c> 下发人工命令或查询命令状态（节点密钥认证）。</summary>
public sealed class LoadRadarCommandClient(IOptions<LoadRadarOptions> options, IHttpClientFactory clients)
{
    public const string HttpClientName = "LoadRadarCommands";

    private static readonly JsonSerializerOptions Json = new(JsonSerializerDefaults.Web);

    public Task<LoadRadarCommandStatus> Submit(LoadMeasurementRecord row, LoadRadarCommandRequest command,
        CancellationToken cancellationToken)
    {
        LoadRadarService.Identifier(command.RequestId, "requestId");
        if (command.Kind is not ("baseline-capture" or "remeasure")) throw new LoadRadarException(400, "未知人工命令");
        if (command.Kind == "baseline-capture" && !command.ConfirmedEmpty)
            throw new LoadRadarException(409, "采空车候选前必须人工确认当前车辆为空车");
        if (command.Kind == "baseline-capture" && row.Measurement.ChannelId != "CH4")
            throw new LoadRadarException(409, "空车基线只能在CH4采集");
        if (row.Measurement.SourceMode != "live") throw new LoadRadarException(409, "只允许对实车测量所属过车发起命令");
        if (command.Kind == "remeasure" && row.Binding is null)
            throw new LoadRadarException(409, "重测前请先关联流程及车辆、挂车身份");
        return Send(row, command.RequestId, command, cancellationToken);
    }

    public Task<LoadRadarCommandStatus> Status(LoadMeasurementRecord row, string requestId, CancellationToken cancellationToken)
    {
        LoadRadarService.Identifier(requestId, "requestId");
        return Send(row, requestId, null, cancellationToken);
    }

    private async Task<LoadRadarCommandStatus> Send(LoadMeasurementRecord row, string requestId,
        LoadRadarCommandRequest? command, CancellationToken cancellationToken)
    {
        var measurement = row.Measurement;
        var node = options.Value.Nodes.Single(n => n.NodeId == measurement.NodeId);
        var baseUri = new Uri(node.BaseUrl!);
        using var request = command is null
            ? new HttpRequestMessage(HttpMethod.Get, new Uri(baseUri, "/api/integration/commands/" + Uri.EscapeDataString(requestId)))
            : new HttpRequestMessage(HttpMethod.Post, new Uri(baseUri, "/api/integration/commands"))
            {
                Content = JsonContent.Create(new
                {
                    requestId,
                    measurement.ChannelId,
                    measurement.PassageId,
                    command.Kind,
                    command.ConfirmedEmpty
                }, options: Json)
            };
        request.Headers.Add(LoadRadarAgentAuthMiddleware.NodeHeader, node.NodeId);
        request.Headers.Add(LoadRadarAgentAuthMiddleware.KeyHeader, node.AgentKey);

        using var response = await clients.CreateClient(HttpClientName).SendAsync(request, cancellationToken);
        var code = (int)response.StatusCode;
        if (!response.IsSuccessStatusCode)
            throw new LoadRadarException(code is 400 or 404 or 409 or 429 ? code : 502,
                code == 409 ? "采集端拒绝命令：过车可能已结束、状态不满足或编号载荷冲突，请核对现场" :
                code == 404 ? "采集端尚无此命令回执；请保留原编号核查，勿新建命令重复执行" :
                "采集端命令请求未获确认；请保留原编号查询状态");

        var value = await response.Content.ReadFromJsonAsync<JsonElement>(cancellationToken);
        var status = value.GetProperty("status").GetString()!;
        var resultId = value.TryGetProperty("measurementId", out var id) && id.ValueKind == JsonValueKind.String ? id.GetString() : null;
        return new(requestId, measurement.ChannelId, measurement.PassageId, value.GetProperty("kind").GetString()!, status, resultId,
            status == "failed" ? "命令未确认完成，可能已有文件或测量；请先核对历史、当前车辆及节点日志，不会自动重新采集" : null);
    }
}
