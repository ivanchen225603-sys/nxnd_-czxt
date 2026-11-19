
namespace CarPointCloudWebApi.Infrastructure;

/// <summary>JT128 测量、流程绑定、版本基线及正式煤量的持久化边界。</summary>
public interface ILoadRadarRepository
{
    IReadOnlyList<LoadRecalculationTarget> GetRecalculations(string nodeId, string channelId, string? afterId, DateOnly fromDate) => [];
    LoadRecalculationReceipt ApplyRecalculation(string nodeId, LoadRecalculationInput input, string resultHash) =>
        throw new NotSupportedException();
    IReadOnlyList<LoadVolumeRevision> GetVolumeRevisions(string measurementId) => [];
    LoadMeasurementRecord SaveMeasurement(LoadMeasurementRecord record);
    LoadMeasurementRecord? GetMeasurement(string id);
    IReadOnlyList<LoadMeasurementRecord> GetHistory(string? nodeId, string? channelId,
        string? licensePlate, string? beforeId, int take, DateTimeOffset? from = null, DateTimeOffset? to = null);
    LoadPassageBinding BindPassage(LoadMeasurementRecord measurement, LoadPassageBinding binding);
    LoadBaselineVersion CreateBaseline(LoadBaselineVersion version);
    LoadBaselineVersion? GetBaseline(string id);
    IReadOnlyList<LoadBaselineVersion> GetBaselines();
    /// <summary>同节点同车牌共用CH4原观测时刻的正式基线，包含旧作用域的激活历史。</summary>
    LoadBaselineVersion? GetPlateBaselineAt(string nodeId, string licensePlate, DateTimeOffset measuredAt) => null;
    LoadBaselineVersion ActivateBaseline(string id, string? expectedVersionId, string actor, string reason);
    LoadBaselineVersion ActivateAutomaticBaseline(string id, string actor, string reason) =>
        throw new NotSupportedException();
    LoadCoalPublication Publish(LoadMeasurementRecord measurement, CoalVolumeRecord coal,
        string actor, string reason);
}
