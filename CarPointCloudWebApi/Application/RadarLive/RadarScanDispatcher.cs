namespace CarPointCloudWebApi.Application;

/// <summary>向 Worker 发起一次双雷达扫描，并把结果写回 VehicleArrival.RadarStatus。</summary>
public static class RadarScanDispatcher
{
    /// <summary>Worker 返回 2xx 记为 SUCCEEDED，否则记为 FAILED 并保存 Worker 的说明。</summary>
    public static async Task<RadarWorkerResponse> DispatchAsync(
        RadarLiveClient radarClient,
        IIntegrationRepository repository,
        long arrivalId,
        string workflowId,
        CancellationToken cancellationToken)
    {
        var response = await radarClient.StartScanAsync(workflowId, cancellationToken);
        repository.UpdateVehicleArrivalRadarResult(
            arrivalId,
            response.IsSuccess ? VehicleArrivalRadarStatus.Succeeded : VehicleArrivalRadarStatus.Failed,
            response.IsSuccess ? null : $"HTTP {response.StatusCode}: {response.Body?["message"]}",
            BusinessClock.Now);
        return response;
    }
}
