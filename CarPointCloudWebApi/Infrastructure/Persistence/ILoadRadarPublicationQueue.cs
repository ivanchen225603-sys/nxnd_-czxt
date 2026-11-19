namespace CarPointCloudWebApi.Infrastructure;

public sealed record LoadRadarPublicationJob(string MeasurementId, int AttemptCount);

/// <summary>内部持久化任务；只消费新测量事务建立的意图，不扫描历史补发。</summary>
public interface ILoadRadarPublicationQueue
{
    IReadOnlyList<LoadRadarPublicationJob> GetDuePublications(DateTimeOffset now, int take);
    void SetPublicationOutcome(LoadRadarPublicationJob job, string status, string reasonCode, DateTimeOffset nextAttemptAt);
}
