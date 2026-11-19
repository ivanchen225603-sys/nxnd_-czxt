using System.Diagnostics;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>为合作方查询和审计分别限制等待时间及未完成的数据库操作。</summary>
public sealed class PartnerQueryExecutor(IServiceScopeFactory scopes)
{
    private readonly SemaphoreSlim _querySlots = new(2, 2);
    private readonly SemaphoreSlim _auditSlots = new(2, 2);

    public Task<T> QueryAsync<T>(Func<IIntegrationRepository, CancellationToken, Task<T>> query,
        CancellationToken requestAborted) =>
        RunAsync(_querySlots, TimeSpan.FromMilliseconds(2200), query, requestAborted);

    public Task WriteAuditAsync(ApiAccessLogRecord record, TimeSpan budget, CancellationToken requestAborted) =>
        RunAsync(_auditSlots, budget, async (repository, token) =>
        {
            await repository.InsertApiAccessLogAsync(record, token);
            return true;
        }, requestAborted);

    private async Task<T> RunAsync<T>(SemaphoreSlim slots, TimeSpan budget,
        Func<IIntegrationRepository, CancellationToken, Task<T>> operation, CancellationToken requestAborted)
    {
        var started = Stopwatch.GetTimestamp();
        if (!await slots.WaitAsync(budget, requestAborted))
            throw new TimeoutException();

        // 驱动可能在返回 Task 前同步阻塞，也可能忽略取消。作用域和槽位随真实操作结束释放。
        var work = Task.Run(async () =>
        {
            try
            {
                var remaining = budget - Stopwatch.GetElapsedTime(started);
                if (remaining <= TimeSpan.Zero)
                    throw new TimeoutException();
                using var cancellation = CancellationTokenSource.CreateLinkedTokenSource(requestAborted);
                cancellation.CancelAfter(remaining);
                cancellation.Token.ThrowIfCancellationRequested();
                await using var scope = scopes.CreateAsyncScope();
                return await operation(scope.ServiceProvider.GetRequiredService<IIntegrationRepository>(), cancellation.Token);
            }
            finally { slots.Release(); }
        });

        try
        {
            var remaining = budget - Stopwatch.GetElapsedTime(started);
            // 等待计时独立于驱动取消回调，后者可能阻塞。
            return await work.WaitAsync(remaining > TimeSpan.Zero ? remaining : TimeSpan.Zero, requestAborted);
        }
        catch
        {
            _ = work.ContinueWith(task => { _ = task.Exception; }, CancellationToken.None,
                TaskContinuationOptions.OnlyOnFaulted | TaskContinuationOptions.ExecuteSynchronously, TaskScheduler.Default);
            throw;
        }
    }
}
