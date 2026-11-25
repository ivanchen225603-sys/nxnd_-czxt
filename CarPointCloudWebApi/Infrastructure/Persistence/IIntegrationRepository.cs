

namespace CarPointCloudWebApi.Infrastructure;
/// <summary>
/// 数据访问层仓储接口。
/// </summary>
/// <remarks>
/// 定义所有与 CarPointCloud 数据库交互的方法契约。
/// 实现类需负责 SQL 连接管理和事务处理。
/// </remarks>
public interface IIntegrationRepository
{
    /// <summary>按业务日期、完整车牌和磅号分页读取已成功入库的立方米体积。</summary>
    Task<VolumeQueryPage> QueryVolumesAsync(VolumeQuery query, CancellationToken cancellationToken);

    /// <summary>使用时间和主键复合游标稳定读取车辆尺寸页。</summary>
    List<VehicleDimensionRecord> GetVehicleDimensionsPage(
        int take,
        DateTime? windowStart,
        DateTime? beforeTime,
        long? beforeId);

    /// <summary>使用时间和主键复合游标稳定读取煤炭体积页。</summary>
    List<CoalVolumeRecord> GetCoalVolumesPage(
        int take,
        DateTime? windowStart,
        DateTime? beforeTime,
        long? beforeId);

    /// <summary>合同只读查询专用，可取消；不改变内部查询及采集写入。</summary>
    Task<List<VehicleDimensionRecord>> GetPartnerVehiclePageAsync(int take, DateTime? windowStart,
        DateTime asOf, DateTime? beforeTime, long? beforeId, CancellationToken token) =>
        Task.FromResult(GetVehicleDimensionsPage(take, windowStart, beforeTime, beforeId));

    /// <summary>合同煤量只读查询专用，可取消；只允许原识牌关联及实际测量区间完整的数据。</summary>
    Task<List<CoalVolumeRecord>> GetPartnerCoalPageAsync(int take, DateTime? windowStart,
        DateTime asOf, DateTime? beforeTime, long? beforeId, CancellationToken token) =>
        Task.FromResult(GetCoalVolumesPage(take, windowStart, beforeTime, beforeId));

    /// <summary>
    /// 插入车牌识别记录。
    /// </summary>
    /// <param name="record">车牌识别记录。</param>
    /// <summary>按来车流程编号读取该次过车的车辆尺寸记录。</summary>
    /// <param name="workflowId">来车流程编号，即 VehicleArrival.SessionId。</param>
    /// <returns>存在时返回含明细的尺寸记录，否则返回 <see langword="null"/>。</returns>
    /// <remarks>
    /// 磅房页面据此判断"这次采集是否真的写进了 DM8"。Analyzer 用 workflowId 作幂等键，
    /// 同一次过车至多一条主记录；出现多条只可能是幂等键被复用，此时取最新一条并由
    /// 审计日志暴露异常，而不是在这里静默合并。
    /// </remarks>
    VehicleDimensionRecord? GetVehicleDimensionByWorkflowId(string workflowId) => null;

    void InsertLicensePlateRecord(LicensePlateRecord record);

    /// <summary>读取 workflow 已成功绑定的权威车牌；未完成或不存在时返回 null。</summary>
    string? GetSucceededWorkflowLicensePlate(string workflowId) => null;

    /// <summary>原子占用一个到期的车牌获取任务。</summary>
    PlateAcquisitionJob? TryClaimDuePlateAcquisitionJob(DateTime now, DateTime staleBefore) => null;

    /// <summary>外呼前持久占用一次尝试；重启不能恢复或扩大三次预算。</summary>
    bool TryBeginPlateAcquisitionAttempt(PlateAcquisitionJob job, int maxAttempts, DateTime now) => false;

    /// <summary>释放消费者并持久保存原候选及到期时间，不占用线程等待网络或归属复核。</summary>
    bool SavePlateAcquisitionProgress(PlateAcquisitionJob job, string? candidateJson,
        DateTime nextAttemptAt, string? error, DateTime now) => false;

    /// <summary>在同一事务中筛选并保存唯一归属的抓拍。</summary>
    bool TryCompletePlateAcquisitionJob(PlateAcquisitionJob job, IReadOnlyList<LicensePlateRecord> candidates) =>
        throw new NotSupportedException();

    /// <summary>持久化任务失败状态；达到上限后同时把流程车牌状态置为失败。</summary>
    void FailPlateAcquisitionJob(
        PlateAcquisitionJob job,
        string error,
        DateTime nextAttemptAt,
        int maxAttempts,
        DateTime updatedAt) => throw new NotSupportedException();

    /// <summary>仅用现有业务字段为煤量记录匹配内部车辆流程。</summary>
    string? FindWorkflowIdForMeasurement(
        string licensePlate,
        string cameraIdentifier,
        DateTime measureTime) => null;

    /// <summary>
    /// 异步插入API访问日志记录。
    /// </summary>
    /// <param name="record">API访问日志记录。</param>
    /// <param name="cancellationToken">取消令牌。</param>
    /// <returns>异步任务。</returns>
    Task InsertApiAccessLogAsync(ApiAccessLogRecord record, CancellationToken cancellationToken = default);

    /// <summary>
    /// Deletes at most <paramref name="batchSize"/> API access logs older than
    /// <paramref name="cutoff"/>.
    /// </summary>
    /// <returns>The number of rows deleted in this batch.</returns>
    Task<int> DeleteApiAccessLogsOlderThanAsync(
        DateTime cutoff,
        int batchSize,
        CancellationToken cancellationToken = default);

    /// <summary>
    /// 插入车辆尺寸记录。
    /// </summary>
    /// <remarks>
    /// 支持幂等键，防止重复插入。
    /// 在事务中插入主记录和关联明细。
    /// </remarks>
    /// <param name="record">车辆尺寸记录（含明细）。</param>
    /// <param name="cancellationToken">在并发幂等等待期间取消请求的令牌。</param>
    /// <returns>插入记录或幂等命中记录的数据库ID。</returns>
    Task<long> InsertVehicleDimensionRecordAsync(
        VehicleDimensionRecord record,
        CancellationToken cancellationToken = default);

    /// <summary>
    /// 插入煤炭体积记录。
    /// </summary>
    /// <remarks>
    /// 支持幂等键，防止重复插入。
    /// </remarks>
    /// <param name="record">煤炭体积记录。</param>
    /// <param name="cancellationToken">在并发幂等等待期间取消请求的令牌。</param>
    /// <returns>插入记录或幂等命中记录的数据库ID。</returns>
    Task<long> InsertCoalVolumeRecordAsync(
        CoalVolumeRecord record,
        CancellationToken cancellationToken = default);

    /// <summary>
    /// 插入来车事件记录。
    /// </summary>
    /// <param name="record">来车事件记录。</param>
    VehicleArrivalInsertResult InsertVehicleArrivalRecord(VehicleArrivalRecord record);

    /// <summary>按事件编号读取来车记录。</summary>
    /// <param name="eventId">来车事件编号；手动采集用它承载客户端幂等键。</param>
    /// <returns>存在时返回记录，否则返回 <see langword="null"/>。</returns>
    /// <remarks>
    /// 供调用方在<b>写入之前</b>识别幂等重放。没有这一步，重放会先经过"是否覆盖空车档案"
    /// 之类只应对新采集生效的前置检查，把一次正常重试变成 409。
    /// </remarks>
    VehicleArrivalRecord? FindVehicleArrivalByEventId(string eventId) => null;

    /// <summary>按跨系统 workflow/session ID 查询来车流程。</summary>
    VehicleArrivalRecord? GetVehicleArrivalBySessionId(string sessionId);

    /// <summary>Persists the outcome of the CAM004 radar operation.</summary>
    void UpdateVehicleArrivalRadarResult(long recordId, string radarStatus, string? radarError, DateTime updatedAt);

    /// <summary>CH4 停稳自动触发：仅当雷达状态仍为 NOT_REQUIRED 时置为 PENDING，同一车次只能成功一次。</summary>
    /// <remarks>不复用 <see cref="TryClaimVehicleArrivalRadarProcessing"/>：它允许 FAILED/超时 PENDING 重新占用，会变成自动补扫。</remarks>
    bool TryClaimVehicleArrivalAutoScan(long recordId, DateTime updatedAt) => false;
}
