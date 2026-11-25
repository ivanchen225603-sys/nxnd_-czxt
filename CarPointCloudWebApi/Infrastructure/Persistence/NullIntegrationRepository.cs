
namespace CarPointCloudWebApi.Infrastructure;

/// <summary>
/// 数据库关闭时使用的空仓储，保持接口可用但不执行任何持久化操作。
/// </summary>
public class NullIntegrationRepository : IIntegrationRepository
{
    /// <inheritdoc />
    public virtual Task<VolumeQueryPage> QueryVolumesAsync(VolumeQuery query, CancellationToken cancellationToken) =>
        Task.FromResult(new VolumeQueryPage([], 0, query.Page, query.PageSize));

    /// <inheritdoc />
    public List<VehicleDimensionRecord> GetVehicleDimensionsPage(
        int take,
        DateTime? windowStart,
        DateTime? beforeTime,
        long? beforeId) => new();

    /// <inheritdoc />
    public virtual List<CoalVolumeRecord> GetCoalVolumesPage(
        int take,
        DateTime? windowStart,
        DateTime? beforeTime,
        long? beforeId) => new();


    /// <inheritdoc />
    public virtual void InsertLicensePlateRecord(LicensePlateRecord record)
    {
    }

    /// <inheritdoc />
    public virtual string? GetSucceededWorkflowLicensePlate(string workflowId) => null;

    /// <inheritdoc />
    public virtual PlateAcquisitionJob? TryClaimDuePlateAcquisitionJob(DateTime now, DateTime staleBefore) => null;

    public virtual bool TryBeginPlateAcquisitionAttempt(PlateAcquisitionJob job, int maxAttempts, DateTime now)
    {
        if (job.AttemptCount >= maxAttempts) return false;
        job.AttemptCount++;
        return true;
    }

    public virtual bool SavePlateAcquisitionProgress(PlateAcquisitionJob job, string? candidateJson,
        DateTime nextAttemptAt, string? error, DateTime now)
    {
        job.CandidateJson = candidateJson;
        job.NextAttemptAt = nextAttemptAt;
        job.LastError = error;
        return true;
    }

    /// <inheritdoc />
    public virtual bool TryCompletePlateAcquisitionJob(PlateAcquisitionJob job,
        IReadOnlyList<LicensePlateRecord> candidates) => false;

    /// <inheritdoc />
    public virtual void FailPlateAcquisitionJob(
        PlateAcquisitionJob job,
        string error,
        DateTime nextAttemptAt,
        int maxAttempts,
        DateTime updatedAt)
    {
    }

    /// <inheritdoc />
    public virtual Task InsertApiAccessLogAsync(ApiAccessLogRecord record, CancellationToken cancellationToken = default) => Task.CompletedTask;

    /// <inheritdoc />
    public virtual Task<int> DeleteApiAccessLogsOlderThanAsync(
        DateTime cutoff,
        int batchSize,
        CancellationToken cancellationToken = default) => Task.FromResult(0);

    /// <inheritdoc />
    public virtual Task<long> InsertVehicleDimensionRecordAsync(
        VehicleDimensionRecord record,
        CancellationToken cancellationToken = default) => Task.FromResult(0L);

    /// <inheritdoc />
    public virtual Task<long> InsertCoalVolumeRecordAsync(
        CoalVolumeRecord record,
        CancellationToken cancellationToken = default) => Task.FromResult(0L);

    /// <inheritdoc />
    public virtual string? FindWorkflowIdForMeasurement(
        string licensePlate,
        string cameraIdentifier,
        DateTime measureTime) => null;

    /// <inheritdoc />
    public virtual VehicleArrivalInsertResult InsertVehicleArrivalRecord(VehicleArrivalRecord record)
    {
        throw new InvalidOperationException("Vehicle-arrival persistence requires Database:Enabled=true.");
    }

    /// <inheritdoc />
    public virtual VehicleArrivalRecord? FindVehicleArrivalByEventId(string eventId)
    {
        return null;
    }

    /// <inheritdoc />
    public virtual VehicleArrivalRecord? GetVehicleArrivalBySessionId(string sessionId)
    {
        throw new InvalidOperationException("Vehicle-arrival persistence requires Database:Enabled=true.");
    }

    /// <inheritdoc />
    public virtual void UpdateVehicleArrivalRadarResult(long recordId, string radarStatus, string? radarError, DateTime updatedAt)
    {
        throw new InvalidOperationException("Vehicle-arrival persistence requires Database:Enabled=true.");
    }

    /// <inheritdoc />
    public virtual bool TryClaimVehicleArrivalAutoScan(long recordId, DateTime updatedAt)
    {
        throw new InvalidOperationException("Vehicle-arrival persistence requires Database:Enabled=true.");
    }
}
