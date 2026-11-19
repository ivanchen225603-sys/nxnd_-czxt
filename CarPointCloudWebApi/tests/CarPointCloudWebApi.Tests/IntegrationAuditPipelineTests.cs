using System.Net;
using System.Text;
using System.Text.Json;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class IntegrationAuditPipelineTests
{
    [Fact]
    public async Task Internal_only_mode_never_calls_partner_even_from_background_entry_points()
    {
        var handler = new RetryPartnerHandler();
        using var httpClient = new HttpClient(handler);
        var repository = new CaptureRepository();
        var client = new PartnerLicensePlateClient(httpClient,
            new StaticOptionsMonitor<IntegrationOptions>(new IntegrationOptions { ExternalPartnerEnabled = false }),
            repository, CreateAuditConfiguration(), NullLogger<PartnerLicensePlateClient>.Instance,
            new HttpContextAccessor());
        await Assert.ThrowsAsync<InvalidOperationException>(() => client.GetLicensePlateAsync("CAM_004"));
        Assert.Equal(0, handler.Attempts);
        Assert.Empty(repository.Records);
    }

    [Theory]
    [InlineData("/api/v1/vehicleDimensions", "VehicleDimensions")]
    [InlineData("/api/v1/coalVolumes", "CoalVolume")]
    public async Task PartnerQuery_CapturesSanitizedRequestAndCompleteResponse(
        string path,
        string integrationType)
    {
        var repository = new CaptureRepository();
        var configuration = CreateAuditConfiguration();
        using var provider = CreateProvider(repository);
        var middleware = new ApiAccessLogMiddleware(
            async context =>
            {
                context.Response.StatusCode = StatusCodes.Status200OK;
                context.Response.ContentType = "application/json";
                context.Response.Headers["X-Next-Cursor"] = "cursor-001";
                await context.Response.WriteAsync("{\"code\":731,\"message\":\"business-result\",\"data\":[]}");
            },
            provider.GetRequiredService<IServiceScopeFactory>(),
            configuration,
            NullLogger<ApiAccessLogMiddleware>.Instance);

        var context = new DefaultHttpContext();
        context.Request.Method = HttpMethods.Get;
        context.Request.Path = path;
        context.Request.QueryString = new QueryString("?limit=1");
        context.Request.Headers["X-Interaction-Id"] = "query-chain-001";
        context.Request.Headers["X-Caller-System"] = "Partner";
        context.Request.Headers["Signature"] = "must-not-leak";
        context.Response.Body = new MemoryStream();

        await middleware.InvokeAsync(context);

        Assert.Equal(2, repository.Records.Count);
        var received = Assert.Single(repository.Records, item => item.RecordKind == "Lifecycle");
        Assert.Equal("debug", received.LogLevel);
        Assert.Equal("RequestReceived", received.LifecycleStage);
        var record = Assert.Single(repository.Records, item => item.RecordKind == "Access");
        Assert.Equal("info", record.LogLevel);
        Assert.Equal("RequestCompleted", record.LifecycleStage);
        Assert.Equal("query-chain-001", record.InteractionId);
        Assert.Equal(integrationType, record.IntegrationType);
        Assert.Equal("Inbound", record.Direction);
        Assert.Equal("Partner", record.CallerSystem);
        Assert.Equal("WebApi", record.TargetSystem);
        Assert.Equal("?limit=1", record.QueryString);
        Assert.Equal(200, record.HttpStatus);
        Assert.Equal(731, record.BusinessCode);
        Assert.Contains("business-result", record.ResponseBody);
        Assert.Contains("X-Next-Cursor", record.ResponseHeadersJson);
        Assert.Contains("***REDACTED***", record.RequestHeadersJson);
        Assert.DoesNotContain("must-not-leak", record.RequestHeadersJson);
    }

    [Fact]
    public async Task LicensePlateFetch_CapturesWorkerToWebApiAndWebApiToPartnerWithOneInteractionId()
    {
        const string interactionId = "plate-chain-001";
        const string workflowId = "workflow-001";
        const string partnerBody = "{\"code\":200,\"message\":\"ok\",\"data\":[{\"cameraId\":\"CAM_004\",\"licensePlate\":\"粤A64727\",\"captureTime\":\"2026-08-14 10:00:00\",\"enterID\":\"business-1\"}]}";
        var repository = new CaptureRepository();
        var configuration = CreateAuditConfiguration();
        var accessor = new HttpContextAccessor();
        var partnerHandler = new PartnerHandler(partnerBody);
        using var httpClient = new HttpClient(partnerHandler);
        var options = new StaticOptionsMonitor<IntegrationOptions>(new IntegrationOptions
        {
            AppSecret = "test-secret",
            PartnerBaseUrl = "http://partner.local",
            LicensePlatePath = "/api/v1/licensePlateRecords",
            PartnerTimeoutMs = 3000,
            PartnerRetryCount = 1,
            PartnerFirstCallDelayMs = 0,
            PartnerRetryDelayMs = 0
        });
        var partnerClient = new PartnerLicensePlateClient(
            httpClient,
            options,
            repository,
            configuration,
            NullLogger<PartnerLicensePlateClient>.Instance,
            accessor);

        using var provider = CreateProvider(repository);
        var middleware = new ApiAccessLogMiddleware(
            async context =>
            {
                var data = await partnerClient.GetLicensePlateAsync("CAM_004", context.RequestAborted);
                context.Response.StatusCode = StatusCodes.Status200OK;
                context.Response.ContentType = "application/json";
                await context.Response.WriteAsync(JsonSerializer.Serialize(new
                {
                    code = 200,
                    message = "success",
                    data
                }));
            },
            provider.GetRequiredService<IServiceScopeFactory>(),
            configuration,
            NullLogger<ApiAccessLogMiddleware>.Instance);

        var context = new DefaultHttpContext();
        accessor.HttpContext = context;
        context.Request.Method = HttpMethods.Post;
        context.Request.Path = "/api/v1/licensePlateRecords/fetch";
        context.Request.QueryString = new QueryString("?cameraIdentifier=CAM_004");
        context.Request.Headers["X-Interaction-Id"] = interactionId;
        context.Request.Headers["X-Caller-System"] = "Worker";
        context.Request.Headers["X-Workflow-Id"] = workflowId;
        context.Response.Body = new MemoryStream();

        await middleware.InvokeAsync(context);

        Assert.Equal(3, repository.Records.Count);
        var inbound = Assert.Single(repository.Records, item => item.Direction == "Inbound" && item.RecordKind == "Access");
        var outbound = Assert.Single(repository.Records, item => item.Direction == "Outbound" && item.RecordKind == "Access");

        Assert.Equal(interactionId, inbound.InteractionId);
        Assert.Equal(interactionId, outbound.InteractionId);
        Assert.Equal(workflowId, inbound.WorkflowId);
        Assert.Equal(workflowId, outbound.WorkflowId);
        Assert.Equal("粤A64727", inbound.LicensePlate);
        Assert.Equal("粤A64727", outbound.LicensePlate);
        Assert.Equal(1, outbound.AttemptNo);
        Assert.Contains("粤A64727", outbound.ResponseBody);
        Assert.Null(partnerHandler.InteractionId);
        var standalonePlate = Assert.Single(repository.BusinessRecords);
        Assert.Null(standalonePlate.WorkflowId);
        Assert.Equal(1, repository.Records.Count(item => item.RecordKind == "Lifecycle"));
        Assert.All(repository.Records.Where(item => item.RecordKind == "Lifecycle"), item => Assert.Equal("debug", item.LogLevel));
    }

    [Theory]
    [InlineData("/api/v1/vehicleDimensions", "VehicleDimensions", "vehicleDimensions")]
    [InlineData("/api/v1/coalVolumes", "CoalVolume", "coalVolumes")]
    public async Task PartnerQuery_RecordsHowTheQueryWasActuallyExecuted(
        string path,
        string integrationType,
        string endpoint)
    {
        // 第三方只拿到 data 数组。查出 0 条时必须能区分"时间窗被解析成了别的区间"
        // 和"确实没数据"，否则对账时只能猜。
        var diagnostics = new CursorQueryDiagnostics(
            Mode: "timeWindow",
            PageSize: 100,
            WindowStart: new DateTime(2026, 8, 14, 9, 0, 0),
            BeforeTime: null,
            BeforeId: null,
            ReturnedCount: 0,
            HasMore: false,
            DatabaseEnabled: true);
        var repository = new CaptureRepository();
        var configuration = CreateAuditConfiguration();
        using var provider = CreateProvider(repository);
        var middleware = new ApiAccessLogMiddleware(
            async context =>
            {
                IntegrationAudit.SetQueryDiagnostics(context, diagnostics.Describe(endpoint));
                context.Response.StatusCode = StatusCodes.Status200OK;
                context.Response.ContentType = "application/json";
                await context.Response.WriteAsync("{\"code\":200,\"message\":\"success\",\"data\":[]}");
            },
            provider.GetRequiredService<IServiceScopeFactory>(),
            configuration,
            NullLogger<ApiAccessLogMiddleware>.Instance);

        var context = new DefaultHttpContext();
        context.Request.Method = HttpMethods.Get;
        context.Request.Path = path;
        context.Request.QueryString = new QueryString("?timeWindow=120");
        context.Request.Headers["X-Interaction-Id"] = "query-chain-diag";
        context.Response.Body = new MemoryStream();

        await middleware.InvokeAsync(context);

        var executed = Assert.Single(
            repository.Records,
            item => item.LifecycleStage == "QueryExecuted");
        Assert.Equal("Lifecycle", executed.RecordKind);
        Assert.Equal("info", executed.LogLevel);
        Assert.Equal(integrationType, executed.IntegrationType);
        Assert.Equal("query-chain-diag", executed.InteractionId);
        Assert.Contains("模式=timeWindow", executed.LogMessage);
        Assert.Contains("2026-08-14 09:00:00", executed.LogMessage);
        Assert.Contains("返回=0 条", executed.LogMessage);
        // 顺序必须是 收到 → 查询执行 → 完成，否则链路页读起来是乱的。
        Assert.Equal(
            new[] { "RequestReceived", "QueryExecuted", "RequestCompleted" },
            repository.Records.Select(item => item.LifecycleStage).ToArray());
    }

    [Theory]
    [InlineData("/api/v1/vehicleDimensions")]
    [InlineData("/api/v1/coalVolumes")]
    public async Task PartnerQuery_OmitsQueryExecutedWhenHandlerFailedBeforeQuerying(
        string path)
    {
        // 控制器抛异常时不能留下一条"查询完成"的孤立记录。
        var repository = new CaptureRepository();
        var configuration = CreateAuditConfiguration();
        using var provider = CreateProvider(repository);
        var middleware = new ApiAccessLogMiddleware(
            _ => throw new InvalidOperationException("boom"),
            provider.GetRequiredService<IServiceScopeFactory>(),
            configuration,
            NullLogger<ApiAccessLogMiddleware>.Instance);

        var context = new DefaultHttpContext();
        context.Request.Method = HttpMethods.Get;
        context.Request.Path = path;
        context.Response.Body = new MemoryStream();

        await Assert.ThrowsAsync<InvalidOperationException>(() => middleware.InvokeAsync(context));

        Assert.DoesNotContain(repository.Records, item => item.LifecycleStage == "QueryExecuted");
        Assert.Contains(repository.Records, item => item.LifecycleStage == "RequestFailed");
    }

    [Fact]
    public void QueryDiagnostics_ExplainsWhyNothingWasQueriedWhenDatabaseDisabled()
    {
        var description = new CursorQueryDiagnostics(
            "limit", 100, null, null, null, 0, false, DatabaseEnabled: false)
            .Describe("coalVolumes");

        Assert.Contains("Database:Enabled=false", description);
    }

    private static IConfiguration CreateAuditConfiguration()
    {
        return new ConfigurationBuilder()
            .AddInMemoryCollection(new Dictionary<string, string?>
            {
                ["Database:Enabled"] = "true",
                ["AuditLog:CaptureBodies"] = "true",
                ["AuditLog:CaptureHeaders"] = "true",
                ["AuditLog:WriteTimeoutMs"] = "2000",
                ["AuditLog:MaxBodyChars"] = "1048576"
            })
            .Build();
    }

    private static ServiceProvider CreateProvider(CaptureRepository repository)
    {
        return new ServiceCollection()
            .AddSingleton<IIntegrationRepository>(repository)
            .BuildServiceProvider();
    }

    private sealed class CaptureRepository : NullIntegrationRepository
    {
        public List<ApiAccessLogRecord> Records { get; } = new();

        public List<LicensePlateRecord> BusinessRecords { get; } = new();

        public override void InsertLicensePlateRecord(LicensePlateRecord record) =>
            BusinessRecords.Add(record);

        public override Task InsertApiAccessLogAsync(
            ApiAccessLogRecord record,
            CancellationToken cancellationToken = default)
        {
            Records.Add(record);
            return Task.CompletedTask;
        }
    }

    private sealed class PartnerHandler(string body) : HttpMessageHandler
    {
        public string? InteractionId { get; private set; }

        protected override Task<HttpResponseMessage> SendAsync(
            HttpRequestMessage request,
            CancellationToken cancellationToken)
        {
            InteractionId = request.Headers.TryGetValues("X-Interaction-Id", out var values)
                ? values.Single()
                : null;
            var response = new HttpResponseMessage(HttpStatusCode.OK)
            {
                Content = new StringContent(body, Encoding.UTF8, "application/json")
            };
            response.Headers.Add("X-Partner-Trace", "partner-trace-001");
            return Task.FromResult(response);
        }
    }

    private sealed class RetryPartnerHandler : HttpMessageHandler
    {
        private int _attempt;
        public int Attempts => _attempt;

        protected override Task<HttpResponseMessage> SendAsync(
            HttpRequestMessage request,
            CancellationToken cancellationToken)
        {
            _attempt++;
            if (_attempt == 1)
            {
                return Task.FromResult(new HttpResponseMessage(HttpStatusCode.InternalServerError)
                {
                    Content = new StringContent(
                        "{\"code\":500,\"message\":\"temporary\",\"data\":[]}",
                        Encoding.UTF8,
                        "application/json")
                });
            }

            return Task.FromResult(new HttpResponseMessage(HttpStatusCode.OK)
            {
                Content = new StringContent(
                    "{\"code\":200,\"message\":\"ok\",\"data\":[{\"cameraId\":\"CAM_004\",\"licensePlate\":\"粤A22581\",\"captureTime\":\"2026-08-14 10:00:00\",\"enterID\":\"business-2\"}]}",
                    Encoding.UTF8,
                    "application/json")
            });
        }
    }

    private sealed class AlwaysFailPartnerHandler : HttpMessageHandler
    {
        protected override Task<HttpResponseMessage> SendAsync(
            HttpRequestMessage request,
            CancellationToken cancellationToken)
        {
            return Task.FromResult(new HttpResponseMessage(HttpStatusCode.InternalServerError)
            {
                Content = new StringContent(
                    "{\"code\":500,\"message\":\"still unavailable\",\"data\":[]}",
                    Encoding.UTF8,
                    "application/json")
            });
        }
    }

    private sealed class StaticOptionsMonitor<T>(T currentValue) : IOptionsMonitor<T>, IOptions<T> where T : class
    {
        public T CurrentValue { get; } = currentValue;
        public T Value => CurrentValue;
        public T Get(string? name) => CurrentValue;
        public IDisposable? OnChange(Action<T, string?> listener) => null;
    }
}
