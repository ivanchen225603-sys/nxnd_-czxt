namespace CarPointCloudWebApi.Application;

/// <summary>
/// 处理一条到期的识牌任务：来车后等待首调延迟，向乙方查询本点位抓拍，
/// 按本车实际到磅时间筛选并原子绑定唯一抓拍，未确认时按间隔重试，共最多 3 次。
/// </summary>
public sealed class PlateAcquisitionProcessor(IIntegrationRepository repository, PartnerLicensePlateClient client,
    IConfiguration configuration, ILogger logger)
{
    public async Task RunAsync(PlateAcquisitionJob job, CancellationToken stoppingToken)
    {
        var maxAttempts = client.MaxAttempts;
        if (job.AttemptCount >= maxAttempts)
        {
            var now = BusinessClock.Now;
            repository.FailPlateAcquisitionJob(job, PlateFailureCode.NoCapture + " 本车识牌次数已耗尽",
                now, maxAttempts, now);
            return;
        }
        var firstCallAt = job.CreatedAt.AddMilliseconds(configuration.GetValue("Integration:PartnerFirstCallDelayMs", 10000));
        if (BusinessClock.Now < firstCallAt)
        {
            repository.SavePlateAcquisitionProgress(job, null, firstCallAt, null, BusinessClock.Now);
            return;
        }
        if (!repository.TryBeginPlateAcquisitionAttempt(job, maxAttempts, BusinessClock.Now))
            return;

        string error;
        try
        {
            var requestTime = BusinessClock.Now;
            var data = await client.GetLicensePlateSnapshotForWorkflowAsync(
                job.CameraIdentifier, job.WorkflowId, job.InteractionId, job.AttemptCount, stoppingToken);
            var completedAt = BusinessClock.Now;
            var candidates = data.Select(item => new LicensePlateRecord
            {
                WorkflowId = job.WorkflowId,
                InteractionId = job.InteractionId,
                CameraIdentifier = job.CameraIdentifier,
                CameraId = item.CameraId,
                LicensePlate = item.LicensePlate,
                CaptureTime = PartnerContract.TryParseTime(item.CaptureTime, out var captured) ? captured : null,
                EnterId = item.EnterId,
                FirstSeenAt = item.FirstSeenAt,
                ConflictingIdentity = item.ConflictingIdentity,
                RequestTime = requestTime,
                ResponseCode = 200,
                ResponseMessage = "success",
                IsSuccess = true,
                CreatedAt = completedAt
            }).ToArray();
            if (candidates.Length > 0 && repository.TryCompletePlateAcquisitionJob(job, candidates))
                return;
            error = PlateFailureCode.NoCapture + " 未找到可唯一归属本车的抓拍";
        }
        catch (Exception ex) when (!stoppingToken.IsCancellationRequested)
        {
            // 乙方不可用与"暂无抓拍"同样计入本车 3 次识牌预算。
            error = PlateFailureCode.PartnerError + " " + ex.Message;
            logger.LogWarning("车牌外呼未完成：workflowId={WorkflowId}, attempt={Attempt}", job.WorkflowId, job.AttemptCount);
        }

        logger.LogInformation("车牌匹配未确认 camera={CameraIdentifier} workflowId={WorkflowId} " +
            "interactionId={InteractionId} attempt={Attempt} decisionAt={DecisionAt:yyyy-MM-dd HH:mm:ss.fff} " +
            "terminal={Terminal} reason={Reason}",
            job.CameraIdentifier, job.WorkflowId, job.InteractionId, job.AttemptCount, BusinessClock.Now,
            job.AttemptCount >= maxAttempts, error.Split(' ', 2)[0]);
        if (job.AttemptCount >= maxAttempts)
            repository.FailPlateAcquisitionJob(job, error, BusinessClock.Now, maxAttempts: 0, BusinessClock.Now);
        else
            repository.SavePlateAcquisitionProgress(job, null,
                BusinessClock.Now.AddMilliseconds(configuration.GetValue("Integration:PartnerRetryDelayMs", 5000)), error, BusinessClock.Now);
    }
}
