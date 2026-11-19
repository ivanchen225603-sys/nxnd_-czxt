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
        if (input.NodeId != nodeId) throw new LoadRadarException(403, "停稳通知节点身份不符");
        service.Lane(nodeId, input.ChannelId);
        LoadRadarService.Identifier(input.PassageId, "passageId");
        if (configuration["LoadRadar:DimensionScanChannelId"]?.Trim() != input.ChannelId)
            return new(input.PassageId, "not_required", null);

        var stored = ((ILoadRadarPassageRepository)repository)
            .GetPassageStart(LoadRadarService.Key(nodeId, input.ChannelId, input.PassageId))
            ?? throw new LoadRadarException(409, "本车次尚未完成到车登记");
        var workflow = integration.FindVehicleArrivalByEventId(stored.WorkflowId)!;
        // NOT_REQUIRED → PENDING 只会成功一次，保证每车次最多一次扫描。
        if (!integration.TryClaimVehicleArrivalAutoScan(workflow.Id, BusinessClock.Now))
        {
            logger.LogInformation("CH4 停稳通知：本车次已派发过，不再扫描 workflowId={WorkflowId}", stored.WorkflowId);
            return new(input.PassageId, "already", stored.WorkflowId);
        }

        var response = await RadarScanDispatcher.DispatchAsync(radarClient, integration, workflow.Id, stored.WorkflowId,
            CancellationToken.None);
        var status = response.IsSuccess ? "dispatched" : "rejected";
        logger.LogInformation("CH4 停稳触发双雷达: workflowId={WorkflowId}, passageId={PassageId}, result={Status}",
            stored.WorkflowId, input.PassageId, status);
        return new(input.PassageId, status, stored.WorkflowId);
    }
}
