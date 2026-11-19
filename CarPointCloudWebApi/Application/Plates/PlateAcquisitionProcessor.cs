namespace CarPointCloudWebApi.Application;

/// <summary>
/// 处理一条到期的识牌任务：来车后等待首调延迟，向乙方查询本点位抓拍，
/// 取来车窗口内最新的一条作为本车车牌；未取到则按间隔重试，共最多 3 次。
/// </summary>
public sealed class PlateAcquisitionProcessor(IIntegrationRepository repository, PartnerLicensePlateClient client,
    IConfiguration configuration, ILogger logger)
{
    private const int MaxAttempts = 3;

    public async Task RunAsync(PlateAcquisitionJob job, CancellationToken stoppingToken)
    {
        var firstCallAt = job.CreatedAt.AddMilliseconds(configuration.GetValue("Integration:PartnerFirstCallDelayMs", 10000));
        if (BusinessClock.Now < firstCallAt)
        {
            repository.SavePlateAcquisitionProgress(job, null, firstCallAt, null, BusinessClock.Now);
            return;
        }
        if (!repository.TryBeginPlateAcquisitionAttempt(job, MaxAttempts, BusinessClock.Now))
            return;

        string error;
        try
        {
            var requestTime = BusinessClock.Now;
            var data = await client.GetLicensePlateSnapshotForWorkflowAsync(
                job.CameraIdentifier, job.WorkflowId, job.InteractionId, job.AttemptCount, stoppingToken);
            var windowStart = job.CreatedAt.AddSeconds(-configuration.GetValue("PlateAcquisition:CaptureLeadToleranceSeconds", 5));
            var windowEnd = job.CreatedAt.AddSeconds(configuration.GetValue("PlateAcquisition:DeadlineSeconds", 30));
            var match = data
                .Select(item => (Item: item, Ok: PartnerContract.TryParseTime(item.CaptureTime, out var at), At: at))
                .Where(x => x.Ok && x.At >= windowStart && x.At <= windowEnd && x.Item.CameraId == job.CameraIdentifier &&
                            PartnerContract.IsLicensePlate(x.Item.LicensePlate))
                .OrderByDescending(x => x.At)
                .FirstOrDefault();
            if (match.Item is { } item)
            {
                repository.CompletePlateAcquisitionJob(job, new LicensePlateRecord
                {
                    WorkflowId = job.WorkflowId,
                    InteractionId = job.InteractionId,
                    CameraIdentifier = job.CameraIdentifier,
                    CameraId = item.CameraId,
                    LicensePlate = item.LicensePlate,
                    CaptureTime = match.At,
                    EnterId = item.EnterId,
                    RequestTime = requestTime,
                    ResponseCode = 200,
                    ResponseMessage = "success",
                    IsSuccess = true,
                    CreatedAt = BusinessClock.Now
                });
                return;
            }
            error = PlateFailureCode.NoCapture + " 乙方数组没有当前车辆的新抓拍";
        }
        catch (Exception ex) when (!stoppingToken.IsCancellationRequested)
        {
            // 乙方不可用与"暂无抓拍"同样计入本车 3 次识牌预算。
            error = PlateFailureCode.PartnerError + " " + ex.Message;
            logger.LogWarning("车牌外呼未完成：workflowId={WorkflowId}, attempt={Attempt}", job.WorkflowId, job.AttemptCount);
        }

        if (job.AttemptCount >= MaxAttempts)
            repository.FailPlateAcquisitionJob(job, error, BusinessClock.Now, maxAttempts: 0, BusinessClock.Now);
        else
            repository.SavePlateAcquisitionProgress(job, null,
                BusinessClock.Now.AddMilliseconds(configuration.GetValue("Integration:PartnerRetryDelayMs", 5000)), error, BusinessClock.Now);
    }
}
