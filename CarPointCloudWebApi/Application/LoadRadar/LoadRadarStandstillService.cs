namespace CarPointCloudWebApi.Application;

/// <summary>
/// CH4 车辆停稳通知：本车次到车已登记时，用到车的 workflowId 触发一次双雷达尺寸采集，每车次只触发一次。
/// 只有 LoadRadar:DimensionScanChannelId 指定的通道会触发。
/// </summary>
public sealed class LoadRadarStandstillService(ILoadRadarRepository repository, IIntegrationRepository integration,
    LoadRadarService service, RadarLiveClient radarClient, IConfiguration configuration,
    ILogger<LoadRadarStandstillService> logger)
{
    public async Task<LoadPassageStandstillReceipt> ReportAsync(string nodeId, LoadPassageStandstill input)
    {
        if (!IsDimensionPassage(nodeId, input.NodeId, input.ChannelId, input.PassageId))
            return new(input.PassageId, "not_required", null);

        var stored = FindPassage(nodeId, input.ChannelId, input.PassageId);
        var now = DateTimeOffset.UtcNow;
        if (input.StoppedAt is not { } stoppedAt || stoppedAt < stored.Start.ArrivedAt || stoppedAt > now.AddSeconds(2))
            throw new LoadRadarException(400, "停稳时间缺失或超出到车至当前时间范围");
        var captureUntil = stoppedAt.AddSeconds(10);
        if (captureUntil <= now)
            return new(input.PassageId, "expired", stored.WorkflowId);

        var workflow = integration.FindVehicleArrivalByEventId(stored.WorkflowId)!;
        // NOT_REQUIRED → PENDING 只会成功一次，保证每车次最多一次扫描。
        if (!integration.TryClaimVehicleArrivalAutoScan(workflow.Id, BusinessClock.Now))
        {
            logger.LogInformation("CH4 停稳通知：本车次已派发过，不再扫描 workflowId={WorkflowId}", stored.WorkflowId);
            return new(input.PassageId, "already", stored.WorkflowId);
        }

        var response = await RadarScanDispatcher.DispatchAsync(radarClient, integration, workflow.Id, stored.WorkflowId,
            CancellationToken.None, captureUntil);
        var status = response.IsSuccess ? "dispatched" : "rejected";
        logger.LogInformation("CH4 停稳触发双雷达: workflowId={WorkflowId}, passageId={PassageId}, result={Status}",
            stored.WorkflowId, input.PassageId, status);
        return new(input.PassageId, status, stored.WorkflowId);
    }

    public async Task<LoadPassageStandstillReceipt> DepartAsync(string nodeId, LoadPassageDeparture input)
    {
        if (!IsDimensionPassage(nodeId, input.NodeId, input.ChannelId, input.PassageId))
            return new(input.PassageId, "not_required", null);
        var stored = FindPassage(nodeId, input.ChannelId, input.PassageId);
        var response = await radarClient.EndPassageAsync(stored.WorkflowId, CancellationToken.None);
        if (!response.IsSuccess)
            throw new LoadRadarException(503, "Worker 未确认关闭本车次采集");
        logger.LogInformation("已确认离场采集状态 workflowId={WorkflowId} passageId={PassageId} status={Code}",
            stored.WorkflowId, input.PassageId, response.Body?["code"]);
        return new(input.PassageId, "closed", stored.WorkflowId);
    }

    private bool IsDimensionPassage(string nodeId, string requestedNodeId, string channelId, string passageId)
    {
        if (requestedNodeId != nodeId) throw new LoadRadarException(403, "车次通知节点身份不符");
        service.Lane(nodeId, channelId);
        LoadRadarService.Identifier(passageId, "passageId");
        return configuration["LoadRadar:DimensionScanChannelId"]?.Trim() == channelId;
    }

    private LoadPassageStartRecord FindPassage(string nodeId, string channelId, string passageId) =>
        ((ILoadRadarPassageRepository)repository).GetPassageStart(LoadRadarService.Key(nodeId, channelId, passageId))
        ?? throw new LoadRadarException(409, "本车次尚未完成到车登记");
}
