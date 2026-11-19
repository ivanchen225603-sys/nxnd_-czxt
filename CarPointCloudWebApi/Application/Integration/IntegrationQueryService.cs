using static CarPointCloudWebApi.Application.IntegrationResponseMapper;

namespace CarPointCloudWebApi.Application;

/// <summary>合作方查询（尺寸、煤量）、流程车牌查询，以及内部测量写入的入口。</summary>
public class IntegrationQueryService(IIntegrationRepository repository, IConfiguration configuration,
    PartnerQueryExecutor? partnerQueries = null)
{
    private const int MaxLimit = 100;
    private const int MaxTimeWindowMinutes = 120;
    private const int BatchSize = 101;

    private readonly MeasurementIngestionService _ingestion = new(repository, configuration);

    private bool DatabaseEnabled => configuration.GetValue("Database:Enabled", true);

    public VehicleWorkflowPlateResponse GetVehicleWorkflowPlate(string workflowId)
    {
        var record = repository.GetVehicleArrivalBySessionId(workflowId.Trim())
            ?? throw new KeyNotFoundException("车辆流程不存在");
        if (record.PlateStatus != VehicleArrivalPlateStatus.Succeeded || string.IsNullOrWhiteSpace(record.LicensePlate))
            throw new InvalidOperationException("车辆流程的车牌尚未就绪");
        return new VehicleWorkflowPlateResponse
        {
            WorkflowId = record.SessionId,
            LicensePlate = record.LicensePlate,
            Source = record.PlateSource ?? "UNKNOWN",
            CapturedAt = record.PlateCapturedAt
        };
    }

    /// <summary>合同尺寸查询：只返回车牌、点位符合合同格式的记录。</summary>
    public Task<CursorPage<VehicleDimensionDto>> GetPartnerVehiclesAsync(int? limit, int? timeWindow, CancellationToken token) =>
        QueryAsync((source, queryToken) => ReadPartnerPageAsync(limit, timeWindow,
            source.GetPartnerVehiclePageAsync, row => row.GetTime, row => row.Id,
            row => PartnerContract.IsLicensePlate(row.LicensePlate) && PartnerContract.IsCamera(row.CameraIdentifier),
            row => ToVehicleDimensionDto(row), queryToken), token);

    /// <summary>合同煤量查询：缺少合同必填身份或真实测量区间的记录只留在内部。</summary>
    public Task<CursorPage<CoalVolumeDto>> GetPartnerCoalsAsync(int? limit, int? timeWindow, CancellationToken token) =>
        QueryAsync((source, queryToken) => ReadPartnerPageAsync(limit, timeWindow, source.GetPartnerCoalPageAsync,
            row => row.MeasurementStartedAt ?? row.MeasureTime, row => row.Id,
            row => PartnerContract.IsLicensePlate(row.LicensePlate) && PartnerContract.IsCamera(row.CameraIdentifier) &&
                   Round1(row.Volume) > 0 && row.Unit is "m³" or "m3" &&
                   string.Equals(row.Status, "success", StringComparison.OrdinalIgnoreCase) &&
                   !string.IsNullOrWhiteSpace(row.EnterId) && row.CaptureTime.HasValue &&
                   row.MeasurementStartedAt.HasValue && row.MeasurementEndedAt.HasValue &&
                   row.MeasurementStartedAt <= row.MeasurementEndedAt,
            ToCoalVolumeDto, queryToken), token);

    private Task<T> QueryAsync<T>(Func<IIntegrationRepository, CancellationToken, Task<T>> operation,
        CancellationToken token) =>
        partnerQueries is null ? operation(repository, token) : partnerQueries.QueryAsync(operation, token);

    public Task<long> IngestPointCloudDimensionsAsync(PointCloudDimensionIngestRequest request,
        CancellationToken cancellationToken = default) =>
        _ingestion.IngestPointCloudDimensionsAsync(request, cancellationToken);

    public Task<long> IngestCoalVolumeAsync(PointCloudCoalVolumeIngestRequest request,
        CancellationToken cancellationToken = default) =>
        _ingestion.IngestCoalVolumeAsync(request, cancellationToken);

    public VehicleDimensionDto? GetVehicleDimensionByWorkflow(string workflowId) =>
        DatabaseEnabled && repository.GetVehicleDimensionByWorkflowId(workflowId.Trim()) is { } record
            ? ToVehicleDimensionDto(record, includeReview: true)
            : null;

    /// <summary>
    /// limit 与 timeWindow 二选一：limit 取最近 N 条（超范围按 100），timeWindow 取最近 N 分钟（超范围按 120）内最多 100 条。
    /// 按时间倒序分批读取并筛掉不合格记录，直到凑满一页。
    /// </summary>
    private async Task<CursorPage<TDto>> ReadPartnerPageAsync<TRecord, TDto>(int? limit, int? timeWindow,
        Func<int, DateTime?, DateTime, DateTime?, long?, CancellationToken, Task<List<TRecord>>> read,
        Func<TRecord, DateTime> time, Func<TRecord, long> id, Func<TRecord, bool> allowed,
        Func<TRecord, TDto> map, CancellationToken token)
    {
        if (limit.HasValue == timeWindow.HasValue)
            throw new ArgumentException("limit 与 timeWindow 不可同时传入，也不可都不传");
        var mode = limit.HasValue ? "limit" : "timeWindow";
        var pageSize = limit is { } l ? (l is < 1 or > MaxLimit ? MaxLimit : l) : MaxLimit;
        DateTime? windowStart = timeWindow is { } w
            ? BusinessClock.Now.AddMinutes(-(w is < 1 or > MaxTimeWindowMinutes ? MaxTimeWindowMinutes : w))
            : null;
        if (!DatabaseEnabled)
            return new CursorPage<TDto>([], null, false,
                new CursorQueryDiagnostics(mode, pageSize, windowStart, null, null, 0, false, false));

        var asOf = BusinessClock.Now;
        var items = new List<TDto>();
        var scanned = 0;
        var filtered = 0;
        DateTime? before = null;
        long? beforeId = null;
        while (items.Count <= pageSize)
        {
            token.ThrowIfCancellationRequested();
            var batch = await read(BatchSize, windowStart, asOf, before, beforeId, token);
            foreach (var row in batch)
            {
                scanned++;
                if (time(row) <= asOf && allowed(row)) items.Add(map(row));
                else filtered++;
                if (items.Count > pageSize) break;
            }
            if (batch.Count < BatchSize) break;
            before = time(batch[^1]);
            beforeId = id(batch[^1]);
        }
        var more = items.Count > pageSize;
        var result = items.Take(pageSize).ToList();
        return new CursorPage<TDto>(result, null, more,
            new CursorQueryDiagnostics(mode, pageSize, windowStart, null, null, result.Count, more, true)
            { ScannedCount = scanned, FilteredCount = filtered });
    }
}
