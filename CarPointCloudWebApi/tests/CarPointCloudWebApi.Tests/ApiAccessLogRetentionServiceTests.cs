using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class ApiAccessLogRetentionServiceTests
{
    [Fact]
    public async Task RunCleanupOnceAsync_UsesDefaultsAndDrainsAllBatches()
    {
        var repository = new RecordingRepository(1000, 37);
        using var provider = BuildProvider(repository);
        var options = new ApiAccessLogRetentionOptions();
        var service = CreateService(provider, options, databaseEnabled: true);
        var earliestExpectedCutoff = BusinessClock.Now.AddDays(-90);

        var deleted = await service.RunCleanupOnceAsync();

        var latestExpectedCutoff = BusinessClock.Now.AddDays(-90);
        Assert.Equal(1037, deleted);
        Assert.Equal(2, repository.Calls.Count);
        Assert.All(repository.Calls, call => Assert.Equal(1000, call.BatchSize));
        Assert.All(
            repository.Calls,
            call => Assert.InRange(call.Cutoff, earliestExpectedCutoff, latestExpectedCutoff));
        Assert.Equal(repository.Calls[0].Cutoff, repository.Calls[1].Cutoff);
    }

    [Fact]
    public async Task RunCleanupOnceAsync_UsesConfiguredBatchSize()
    {
        var repository = new RecordingRepository(250, 250, 1);
        using var provider = BuildProvider(repository);
        var options = new ApiAccessLogRetentionOptions
        {
            RetentionDays = 30,
            CleanupBatchSize = 250
        };
        var service = CreateService(provider, options, databaseEnabled: true);

        var deleted = await service.RunCleanupOnceAsync();

        Assert.Equal(501, deleted);
        Assert.Equal(3, repository.Calls.Count);
        Assert.All(repository.Calls, call => Assert.Equal(250, call.BatchSize));
    }

    [Fact]
    public async Task NullRepository_CleanupIsSafeNoOp()
    {
        var repository = new NullIntegrationRepository();

        var deleted = await repository.DeleteApiAccessLogsOlderThanAsync(
            BusinessClock.Now.AddDays(-90),
            1000);

        Assert.Equal(0, deleted);
    }

    private static ServiceProvider BuildProvider(IIntegrationRepository repository)
    {
        var services = new ServiceCollection();
        services.AddScoped<IIntegrationRepository>(_ => repository);
        return services.BuildServiceProvider();
    }

    private static ApiAccessLogRetentionService CreateService(
        IServiceProvider provider,
        ApiAccessLogRetentionOptions options,
        bool databaseEnabled)
    {
        var configuration = new ConfigurationBuilder()
            .AddInMemoryCollection(new Dictionary<string, string?>
            {
                ["Database:Enabled"] = databaseEnabled.ToString()
            })
            .Build();

        return new ApiAccessLogRetentionService(
            provider.GetRequiredService<IServiceScopeFactory>(),
            Options.Create(options),
            configuration,
            NullLogger<ApiAccessLogRetentionService>.Instance);
    }

    private sealed class RecordingRepository : NullIntegrationRepository
    {
        private readonly Queue<int> _deleteResults;

        public RecordingRepository(params int[] deleteResults)
        {
            _deleteResults = new Queue<int>(deleteResults);
        }

        public List<CleanupCall> Calls { get; } = new();

        public override Task<int> DeleteApiAccessLogsOlderThanAsync(
            DateTime cutoff,
            int batchSize,
            CancellationToken cancellationToken = default)
        {
            Calls.Add(new CleanupCall(cutoff, batchSize));
            return Task.FromResult(_deleteResults.Count == 0 ? 0 : _deleteResults.Dequeue());
        }
    }

    private sealed record CleanupCall(DateTime Cutoff, int BatchSize);
}
