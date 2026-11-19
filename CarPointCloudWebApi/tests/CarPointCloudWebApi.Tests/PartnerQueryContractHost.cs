using System.Collections.Concurrent;
using System.Net;
using System.Text.Json;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Mvc.Testing;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.DependencyInjection.Extensions;
using Microsoft.Extensions.Logging;

namespace CarPointCloudWebApi.Tests;

/// <summary>运行完整生产 HTTP 管线；业务数据和审计全部使用内存仓储，不使用任何数据库或设备。</summary>
internal sealed class PartnerQueryContractHost : WebApplicationFactory<Program>
{
    internal const string Secret = "contract-query-fixture-secret-not-for-production";
    private static int _address;
    private readonly string _root = Path.Combine(Path.GetTempPath(), "partner-contract-tests-" + Guid.NewGuid().ToString("N"));
    internal QueryFixtureRepository Repository { get; } = new();
    internal QueryLogProvider Logs { get; } = new();
    internal RejectQueryOutboundHandler Outbound { get; } = new();

    internal PartnerQueryContractHost()
    {
        Directory.CreateDirectory(_root);
        File.WriteAllText(Path.Combine(_root, "appsettings.json"), JsonSerializer.Serialize(new
        {
            // 占位连接串只能指向本机无服务端口。启动前再替换全部业务/审计仓储，绝不回落真实配置。
            ConnectionStrings = new { CarPointCloudDb = "Server=127.0.0.1;Port=1;User=query_fixture;PWD=fixture_only;Schema=QUERY_FIXTURE" },
            Database = new { Enabled = true, InitializeOnStartup = false },
            Auth = new { JwtSecret = "contract-query-jwt-fixture-only-0123456789" },
            Integration = new
            {
                AppSecret = Secret,
                ExternalQueriesEnabled = true,
                ExternalPartnerEnabled = false,
                PartnerBaseUrl = "",
                EnableMockLicensePlate = false,
                RateLimitPerSecond = 3
            },
            AuditLog = new
            {
                CaptureBodies = false,
                CapturePartnerQueryBodies = true,
                CaptureHeaders = true,
                MaxBodyChars = 1048576
            },
            Radar = new { BaseUrl = "http://127.0.0.1:1" },
            LoadRadar = new { Enabled = false }
        }));
        var now = BusinessClock.Now.AddSeconds(-1);
        Repository.Vehicles.Add(Vehicle(1, now));
        Repository.Coals.Add(Coal(1, now));
    }

    internal static VehicleDimensionRecord Vehicle(long id, DateTime time) => new()
    {
        Id = id,
        LicensePlate = "宁A48394Q",
        CameraIdentifier = "CAM_001",
        GetTime = time,
        WorkflowId = "synthetic-workflow",
        PairId = "synthetic-pair",
        SourceFile = "internal-only.pcd",
        DetectStatus = "success",
        CabToTrailerFrontDist = 440.16m,
        TrailerLength = 1050.24m,
        TrailerWidth = 229.24m,
        MaximumWidthCm = 255.36m,
        TrailerBodyHeight = null,
        TrailerBottomToGroundHeight = 150.14m,
        HighLowPlateHeight = 20.16m,
        ActualRibCount = 2,
        Ribs = [new() { RibId = 3, DistToPrev = 300 }, new() { RibId = 1, DistToPrev = 350.16m },
            new() { RibId = 2, DistToPrev = null }],
        Wires = [new() { WireId = 2, DistToPrev = 999 }, new() { WireId = 1, DistToPrev = 400.16m }],
        HighLowPlates = [new() { PlateId = 1, LengthCm = 180.16m, HighestToGround = 999, LowestToGround = 130.14m }]
    };

    internal static CoalVolumeRecord Coal(long id, DateTime time) => new()
    {
        Id = id,
        LicensePlate = "宁A48394Q",
        CameraIdentifier = "CAM_001",
        MeasureTime = time,
        WorkflowId = "synthetic-workflow",
        Volume = 52.36m,
        Unit = "m3",
        Status = "success",
        EnterId = "SYNTHETIC-ENTER-001",
        CaptureTime = time.AddSeconds(-12),
        MeasurementStartedAt = time.AddSeconds(-3),
        MeasurementEndedAt = time,
        SourceFile = "internal-only.pcd",
        Confidence = 0.99m
    };

    protected override void ConfigureWebHost(IWebHostBuilder builder)
    {
        builder.UseEnvironment("Production").UseContentRoot(_root);
        builder.ConfigureServices((context, services) =>
        {
            if (context.Configuration["Integration:AppSecret"] != Secret ||
                context.Configuration.GetValue("Database:InitializeOnStartup", true) ||
                context.Configuration.GetConnectionString("CarPointCloudDb")?.Contains("Port=1;") != true)
                throw new InvalidOperationException("Contract tests refused non-isolated configuration.");
            services.RemoveAll<IIntegrationRepository>();
            services.AddSingleton<IIntegrationRepository>(Repository);
            services.AddSingleton<ILoggerFactory>(_ => LoggerFactory.Create(log => log.SetMinimumLevel(LogLevel.Debug)
                .AddFilter("Microsoft.AspNetCore.Hosting.Diagnostics", LogLevel.Warning).AddProvider(Logs)));
            services.AddSingleton<IStartupFilter>(new AddressFilter(IPAddress.Parse("2001:db8:1::" +
                Interlocked.Increment(ref _address).ToString("x"))));
            services.AddHttpClient<PartnerLicensePlateClient>().ConfigurePrimaryHttpMessageHandler(() => Outbound);
        });
    }

    internal void SaveEvidence(string name, string uri, string interactionId, int status, string response, double? elapsedMs = null)
    {
        var directory = Environment.GetEnvironmentVariable("CARPOINTCLOUD_QUERY_EVIDENCE_DIRECTORY");
        if (string.IsNullOrWhiteSpace(directory)) return;
        Directory.CreateDirectory(directory);
        var data = new
        {
            syntheticFixture = true,
            testCase = name,
            request = new { method = "GET", uri, interactionId },
            response = new { status, elapsedMs, body = JsonSerializer.Deserialize<JsonElement>(response) },
            queries = Repository.Queries.ToArray(),
            auditRecords = Repository.Records.ToArray(),
            processLogs = Logs.Entries.ToArray(),
            outboundCalls = Outbound.Calls
        };
        File.WriteAllText(Path.Combine(directory, name + "-" + Guid.NewGuid().ToString("N") + ".json"),
            JsonSerializer.Serialize(data, new JsonSerializerOptions { WriteIndented = true }));
    }

    protected override void Dispose(bool disposing)
    {
        base.Dispose(disposing);
        if (!disposing || !Directory.Exists(_root)) return;
        if (!Path.GetFullPath(_root).StartsWith(Path.Combine(Path.GetTempPath(), "partner-contract-tests-"), StringComparison.OrdinalIgnoreCase))
            throw new InvalidOperationException("Contract test cleanup prefix is invalid.");
        Directory.Delete(_root, true);
    }

    private sealed class AddressFilter(IPAddress address) : IStartupFilter
    {
        public Action<IApplicationBuilder> Configure(Action<IApplicationBuilder> next) => app =>
        {
            app.Use((context, continuation) =>
            {
                context.Connection.RemoteIpAddress = context.Request.Headers.ContainsKey("X-Query-Test-Source")
                    ? IPAddress.Parse("2001:db8:1::" + Interlocked.Increment(ref _address).ToString("x")) : address;
                return continuation(context);
            });
            next(app);
        };
    }
}

internal sealed record FixtureQuery(string Endpoint, int Take, DateTime? WindowStart, DateTime? BeforeTime, long? BeforeId);

internal sealed class QueryFixtureRepository : NullIntegrationRepository, IIntegrationRepository
{
    internal List<VehicleDimensionRecord> Vehicles { get; } = [];
    internal List<CoalVolumeRecord> Coals { get; } = [];
    internal ConcurrentQueue<FixtureQuery> Queries { get; } = new();
    internal ConcurrentQueue<ApiAccessLogRecord> Records { get; } = new();
    internal Exception? QueryError { get; set; }
    internal bool AuditFails { get; set; }
    internal TaskCompletionSource? QueryBlock { get; set; }
    internal TaskCompletionSource? AuditBlock { get; set; }
    internal TaskCompletionSource AuditSettled { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);

    public async Task<List<VehicleDimensionRecord>> GetPartnerVehiclePageAsync(int take, DateTime? windowStart,
        DateTime asOf, DateTime? beforeTime, long? beforeId, CancellationToken token)
    {
        if (QueryBlock is not null) await QueryBlock.Task; // 特意模拟忽略取消的驱动。
        token.ThrowIfCancellationRequested();
        return GetVehicleDimensionsPage(take, windowStart, beforeTime, beforeId);
    }

    public async Task<List<CoalVolumeRecord>> GetPartnerCoalPageAsync(int take, DateTime? windowStart,
        DateTime asOf, DateTime? beforeTime, long? beforeId, CancellationToken token)
    {
        if (QueryBlock is not null) await QueryBlock.Task;
        token.ThrowIfCancellationRequested();
        Queries.Enqueue(new("coalVolumes", take, windowStart, beforeTime, beforeId));
        if (QueryError is not null) throw QueryError;
        DateTime Clock(CoalVolumeRecord row) => row.MeasurementStartedAt ?? row.MeasureTime;
        return Coals.Where(row => (!windowStart.HasValue || Clock(row) >= windowStart.Value) &&
            (!beforeTime.HasValue || Clock(row) < beforeTime.Value || Clock(row) == beforeTime.Value && row.Id < beforeId))
            .OrderByDescending(Clock).ThenByDescending(row => row.Id).Take(take).ToList();
    }

    public new List<VehicleDimensionRecord> GetVehicleDimensionsPage(int take, DateTime? windowStart, DateTime? beforeTime, long? beforeId)
    {
        Queries.Enqueue(new("vehicleDimensions", take, windowStart, beforeTime, beforeId));
        if (QueryError is not null) throw QueryError;
        return Vehicles.Where(row => (!windowStart.HasValue || row.GetTime >= windowStart.Value) &&
            (!beforeTime.HasValue || row.GetTime < beforeTime.Value || row.GetTime == beforeTime.Value && row.Id < beforeId))
            .OrderByDescending(row => row.GetTime).ThenByDescending(row => row.Id).Take(take).ToList();
    }

    public override List<CoalVolumeRecord> GetCoalVolumesPage(int take, DateTime? windowStart, DateTime? beforeTime, long? beforeId)
    {
        Queries.Enqueue(new("coalVolumes", take, windowStart, beforeTime, beforeId));
        if (QueryError is not null) throw QueryError;
        return Coals.Where(row => (!windowStart.HasValue || row.MeasureTime >= windowStart.Value) &&
            (!beforeTime.HasValue || row.MeasureTime < beforeTime.Value || row.MeasureTime == beforeTime.Value && row.Id < beforeId))
            .OrderByDescending(row => row.MeasureTime).ThenByDescending(row => row.Id).Take(take).ToList();
    }

    public override async Task InsertApiAccessLogAsync(ApiAccessLogRecord record, CancellationToken cancellationToken = default)
    {
        if (AuditFails) throw new IOException("isolated-audit-failure");
        if (AuditBlock is not null) await AuditBlock.Task;
        Records.Enqueue(record);
        AuditSettled.TrySetResult();
    }
}

internal sealed record QueryLogEntry(string Level, string Category, string? Stage, string? InteractionId, string Message, string? Exception);

internal sealed class QueryLogProvider : ILoggerProvider
{
    internal ConcurrentQueue<QueryLogEntry> Entries { get; } = new();
    public ILogger CreateLogger(string categoryName) => new QueryLogger(this, categoryName);
    public void Dispose() { }

    private sealed class QueryLogger(QueryLogProvider provider, string category) : ILogger
    {
        public IDisposable? BeginScope<TState>(TState state) where TState : notnull => null;
        public bool IsEnabled(LogLevel level) => true;
        public void Log<TState>(LogLevel level, EventId eventId, TState state, Exception? exception, Func<TState, Exception?, string> formatter)
        {
            var values = state as IEnumerable<KeyValuePair<string, object?>>;
            string? Value(string name) => Convert.ToString(values?.FirstOrDefault(item => item.Key == name).Value);
            provider.Entries.Enqueue(new(level.ToString(), category, Value("LifecycleStage"), Value("InteractionId"),
                formatter(state, exception), exception?.ToString()));
        }
    }
}

internal sealed class RejectQueryOutboundHandler : HttpMessageHandler
{
    internal int Calls;
    protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
    {
        Interlocked.Increment(ref Calls);
        throw new InvalidOperationException("Contract query test attempted an external HTTP call.");
    }
}
