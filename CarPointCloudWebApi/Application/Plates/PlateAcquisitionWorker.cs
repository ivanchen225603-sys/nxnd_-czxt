namespace CarPointCloudWebApi.Application;

/// <summary>
/// 识牌任务消费者：按 PlateAcquisition:MaxConcurrentJobs 并行轮询 DM8 中到期的任务，交给
/// <see cref="PlateAcquisitionProcessor"/> 处理。任务在来车登记时与 VehicleArrival 同事务创建。
/// </summary>
public sealed class PlateAcquisitionWorker(
    IServiceScopeFactory scopeFactory,
    IConfiguration configuration,
    ILogger<PlateAcquisitionWorker> logger) : BackgroundService
{
    protected override Task ExecuteAsync(CancellationToken stoppingToken) =>
        Task.WhenAll(Enumerable.Range(0, configuration.GetValue("PlateAcquisition:MaxConcurrentJobs", 4))
            .Select(_ => ConsumeAsync(stoppingToken)));

    private async Task ConsumeAsync(CancellationToken stoppingToken)
    {
        var pollInterval = TimeSpan.FromMilliseconds(configuration.GetValue("PlateAcquisition:PollIntervalMs", 500));
        var staleAge = TimeSpan.FromSeconds(configuration.GetValue("PlateAcquisition:ProcessingStaleSeconds", 35));
        while (!stoppingToken.IsCancellationRequested)
        {
            using var scope = scopeFactory.CreateScope();
            var repository = scope.ServiceProvider.GetRequiredService<IIntegrationRepository>();
            var now = BusinessClock.Now;
            var job = repository.TryClaimDuePlateAcquisitionJob(now, now.Subtract(staleAge));
            if (job is null)
            {
                await Task.Delay(pollInterval, stoppingToken);
                continue;
            }
            var client = scope.ServiceProvider.GetRequiredService<PartnerLicensePlateClient>();
            await new PlateAcquisitionProcessor(repository, client, configuration, logger).RunAsync(job, stoppingToken);
        }
    }
}

/// <summary>识牌失败原因前缀，写入任务 LastError。</summary>
public static class PlateFailureCode
{
    public const string NoCapture = "[NO_CAPTURE]";
    public const string PartnerError = "[PARTNER_ERROR]";
}
