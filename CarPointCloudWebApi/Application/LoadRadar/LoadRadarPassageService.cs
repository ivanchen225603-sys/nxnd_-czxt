using System.Text.RegularExpressions;

namespace CarPointCloudWebApi.Application;

/// <summary>
/// 载货雷达到车准备：首次到车时登记车次并建立 VehicleArrival（等待识牌），
/// 车牌确定后绑定车次，并按测量时刻选出该车牌的 CH4 空车基线。
/// </summary>
public sealed class LoadRadarPassageService(ILoadRadarRepository repository, IIntegrationRepository integration,
    LoadRadarService service, LoadRadarAssetStore assets, IRadarLicensePlateSource plates, IConfiguration configuration)
{
    public LoadPassagePreparation Prepare(string nodeId, LoadPassageStart input, DateTimeOffset? measuredAt = null)
    {
        if (input.NodeId != nodeId) throw new LoadRadarException(403, "到车节点身份不符");
        var lane = service.Lane(nodeId, input.ChannelId);
        LoadRadarService.Identifier(input.PassageId, "passageId");
        LoadRadarService.Identifier(input.CalibrationId, "calibrationId");
        if (input.SourceMode != "live" || !Regex.IsMatch(input.ConfigurationHash ?? "", "\\A[a-f0-9]{64}\\z"))
            throw new LoadRadarException(400, "到车必须为真实完整观测，包含有效时间和配置摘要");
        input = input with { ArrivedAt = input.ArrivedAt.ToUniversalTime() };

        var passages = (ILoadRadarPassageRepository)repository;
        var key = LoadRadarService.Key(nodeId, input.ChannelId, input.PassageId);
        var proposed = new LoadPassageStartRecord(key, LoadRadarService.Key(input), input,
            lane.CameraIdentifier, LoadRadarService.Key("load-radar-arrival", key));
        var arrived = input.ArrivedAt.ToOffset(TimeSpan.FromHours(8)).DateTime;
        var plate = passages.GetPassageStart(key) is null && plates.Enabled ? plates.Resolve(proposed.CameraIdentifier) : null;
        var stored = passages.RegisterPassageStart(proposed, new VehicleArrivalRecord
        {
            EventId = proposed.WorkflowId,
            SessionId = proposed.WorkflowId,
            DeviceId = "LOAD_RADAR",
            EventType = "ARRIVAL",
            CameraIdentifier = proposed.CameraIdentifier,
            LicensePlate = plate?.LicensePlate,
            PlateSource = plate?.Source,
            PlateCapturedAt = plate?.CaptureTime,
            PlateStatus = plate is null ? VehicleArrivalPlateStatus.Pending : VehicleArrivalPlateStatus.Succeeded,
            RadarStatus = VehicleArrivalRadarStatus.NotRequired,
            ArrivalTime = arrived,
            CreatedAt = arrived,
            UpdatedAt = arrived
        }, configuration.GetValue("PlateAcquisition:ArrivalRegistrationMaxSeconds", 30));

        LoadPassagePreparation Result(string status, LoadPassageBinding? binding = null, LoadBaselineVersion? baseline = null) =>
            new(stored.Start, stored.WorkflowId, stored.CameraIdentifier, status, binding, baseline, measuredAt?.ToUniversalTime());

        var workflow = integration.FindVehicleArrivalByEventId(stored.WorkflowId)!;
        if (workflow.PlateStatus != VehicleArrivalPlateStatus.Succeeded)
            return Result(workflow.PlateStatus == VehicleArrivalPlateStatus.Pending ? "awaiting_plate" : "plate_unavailable");

        ManualLicensePlate.TryNormalize(integration.GetSucceededWorkflowLicensePlate(stored.WorkflowId), out var bound, out _);
        var binding = passages.BindStartedPassage(stored, new(key, stored.WorkflowId, bound,
            stored.CameraIdentifier, "plate:" + bound, "system:load-radar", "固定车牌车厢关系自动关联", DateTimeOffset.UtcNow));
        var selected = repository.GetPlateBaselineAt(nodeId, binding.LicensePlate, measuredAt ?? DateTimeOffset.UtcNow);
        if (selected is null)
            return Result("baseline_required", binding);
        assets.Verify(selected.AssetSha256);
        return Result("ready", binding, selected);
    }
}
