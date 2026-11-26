namespace CarPointCloudWebApi.Application;

/// <summary>Worker 掉电或只落盘单路 PCD 后，仍会结束数据库中已到期的待证车次。</summary>
public sealed class StaticCaptureExpiryWorker(IServiceScopeFactory scopes, ILogger<StaticCaptureExpiryWorker> logger)
    : BackgroundService
{
    protected override async Task ExecuteAsync(CancellationToken token)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromSeconds(5));
        while (await timer.WaitForNextTickAsync(token))
        {
            try
            {
                using var scope = scopes.CreateScope();
                scope.ServiceProvider.GetRequiredService<IIntegrationRepository>().ExpireStaticScans(DateTimeOffset.UtcNow);
            }
            catch (Exception ex) when (!token.IsCancellationRequested)
            {
                logger.LogError(ex, "到期静态证据暂未收口，将在下一轮重试");
            }
        }
    }
}
