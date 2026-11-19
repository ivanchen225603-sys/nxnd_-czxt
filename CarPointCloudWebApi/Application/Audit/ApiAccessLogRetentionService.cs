using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Application;

/// <summary>启动时及此后每天删除超过 AuditLog:RetentionDays 天的审计日志，每批 CleanupBatchSize 条。</summary>
public sealed class ApiAccessLogRetentionService(
    IServiceScopeFactory scopeFactory,
    IOptions<ApiAccessLogRetentionOptions> options,
    IConfiguration configuration,
    ILogger<ApiAccessLogRetentionService> logger) : BackgroundService
{
    public async Task<long> RunCleanupOnceAsync(CancellationToken cancellationToken = default)
    {
        var cutoff = BusinessClock.Now.AddDays(-options.Value.RetentionDays);
        using var scope = scopeFactory.CreateScope();
        var repository = scope.ServiceProvider.GetRequiredService<IIntegrationRepository>();
        long total = 0;
        int deleted;
        do
        {
            deleted = await repository.DeleteApiAccessLogsOlderThanAsync(cutoff, options.Value.CleanupBatchSize, cancellationToken);
            total += deleted;
        }
        while (deleted == options.Value.CleanupBatchSize);
        if (total > 0)
            logger.LogInformation("Deleted {DeletedCount} API access logs older than {Cutoff}", total, cutoff);
        return total;
    }

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        if (!configuration.GetValue("Database:Enabled", true))
            return;
        using var timer = new PeriodicTimer(TimeSpan.FromDays(1));
        do
        {
            await RunCleanupOnceAsync(stoppingToken);
        }
        while (await timer.WaitForNextTickAsync(stoppingToken));
    }
}
