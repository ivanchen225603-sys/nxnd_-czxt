using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Application;

/// <summary>自动煤量入库，不派发扫描、不生成基线、不执行最近车牌匹配。</summary>
public sealed class LoadRadarPublicationProcessor(ILoadRadarRepository repository, LoadRadarService service)
{
    public int RunBatch(DateTimeOffset now, CancellationToken cancellationToken)
    {
        service.EnsureEnabled();
        var queue = (ILoadRadarPublicationQueue)repository;
        var jobs = queue.GetDuePublications(now, 16);
        foreach (var job in jobs)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var status = "pending";
            var reason = "retry_storage";
            try
            {
                var row = service.Get(job.MeasurementId);
                var lane = service.Lane(row.Measurement.NodeId, row.Measurement.ChannelId);
                if (row.Binding is null)
                    reason = "awaiting_identity";
                else
                {
                    if (row.Binding.CameraIdentifier != lane.CameraIdentifier)
                        throw new LoadRadarException(409, "当前点位配置与原绑定不一致");
                    service.Publish(job.MeasurementId, new("合格载货测量自动入库"), "system:load-radar");
                    status = "published";
                    reason = "published";
                }
            }
            catch (LoadRadarException ex) when (!ex.Retryable && ex.StatusCode is 400 or 404 or 409)
            {
                status = "rejected";
                reason = "validation_rejected";
            }
            catch (Exception)
            {
                // 发布可能已提交但回执丢失；只重试原测量，不能换 ID 或重新采集。
                // 不把异常文本（可能含数据库配置）写进任务或用户页面。
                reason = "retry_storage";
            }
            var delaySeconds = Math.Min(300, 5 * (1 << Math.Clamp(job.AttemptCount, 0, 6)));
            // 状态保存故障向上抛出；原任务留在持久层，下一轮重新读取并幂等核实。
            queue.SetPublicationOutcome(job, status, reason, now.AddSeconds(delaySeconds));
        }
        return jobs.Count;
    }
}

/// <summary>每 5 秒处理一批到期的自动入库任务。</summary>
public sealed class LoadRadarPublicationWorker(IServiceScopeFactory scopes, IOptions<LoadRadarOptions> options) : BackgroundService
{
    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        if (!options.Value.Enabled) return;
        using var timer = new PeriodicTimer(TimeSpan.FromSeconds(5));
        while (await timer.WaitForNextTickAsync(stoppingToken))
        {
            using var scope = scopes.CreateScope();
            scope.ServiceProvider.GetRequiredService<LoadRadarPublicationProcessor>().RunBatch(DateTimeOffset.UtcNow, stoppingToken);
        }
    }
}
