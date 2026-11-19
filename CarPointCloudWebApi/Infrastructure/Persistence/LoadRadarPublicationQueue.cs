using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public sealed partial class LoadRadarRepository
{
    private void EnqueuePublication(DmConnection connection, DmTransaction transaction, string id, DateTimeOffset now)
    {
        using var command = Command(connection, transaction,
            "INSERT INTO LoadRadarPublishJob (MeasurementId, Status, AttemptCount, NextAttemptAt, ReasonCode) VALUES (?, 'pending', 0, ?, 'awaiting_processing')");
        V(command, "id", id, 128);
        V(command, "next", Stamp(now), 40);
        command.ExecuteNonQuery();
    }

    public IReadOnlyList<LoadRadarPublicationJob> GetDuePublications(DateTimeOffset now, int take)
    {
        if (take is < 1 or > 16) throw new ArgumentOutOfRangeException(nameof(take));
        using var connection = Open();
        using var command = Command(connection, null,
            $"SELECT TOP {take} MeasurementId, AttemptCount FROM LoadRadarPublishJob WHERE Status = 'pending' AND NextAttemptAt <= ? ORDER BY NextAttemptAt, MeasurementId");
        V(command, "now", Stamp(now), 40);
        using var reader = command.ExecuteReader();
        var jobs = new List<LoadRadarPublicationJob>();
        while (reader.Read()) jobs.Add(new(reader.GetString(0), reader.GetInt32(1)));
        return jobs;
    }

    public void SetPublicationOutcome(LoadRadarPublicationJob job, string status, string reasonCode, DateTimeOffset nextAttemptAt)
    {
        if (status is not ("pending" or "published" or "rejected") || job.AttemptCount < 0)
            throw new LoadRadarException(400, "自动入库任务状态无效");
        using var connection = Open();
        // 多实例允许同一原测量幂等重放；迟到任务不能覆盖已完成状态或更新的退避时间。
        using var command = Command(connection, null,
            "UPDATE LoadRadarPublishJob SET Status = ?, ReasonCode = ?, NextAttemptAt = ?, AttemptCount = AttemptCount + 1 WHERE MeasurementId = ? AND Status = 'pending' AND AttemptCount = ?");
        V(command, "status", status, 16);
        V(command, "reason", reasonCode, 64);
        V(command, "next", Stamp(nextAttemptAt), 40);
        V(command, "id", job.MeasurementId, 128);
        command.Parameters.Add("attempt", DmDbType.Int32).Value = job.AttemptCount;
        command.ExecuteNonQuery();
    }
}
