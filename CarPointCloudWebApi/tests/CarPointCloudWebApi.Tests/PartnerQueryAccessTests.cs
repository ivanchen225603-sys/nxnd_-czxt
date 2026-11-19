using System.Net;
using System.Text.Json;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc.Testing;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Options;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class PartnerQueryAccessTests
{
    [Theory]
    [InlineData("GET", "/api/v1/vehicleDimensions", true)]
    [InlineData("GET", "/api/v1/coalVolumes", true)]
    [InlineData("GET", "/API/V1/CoalVolumes/", true)]
    [InlineData("POST", "/api/v1/coalVolumes", false)]
    [InlineData("POST", "/api/v1/licensePlateRecords/fetch", false)]
    [InlineData("POST", "/api/v1/radar/vehicle-arrival", false)]
    [InlineData("POST", "/api/v1/internal/pointcloud/coalVolumes", false)]
    [InlineData("POST", "/api/v1/internal/pointcloud/vehicleDimensions", true)]
    [InlineData("POST", "/api/v1/radar-live/captures", true)]
    [InlineData("GET", "/api/v1/radar-live/status", true)]
    [InlineData("GET", "/api/v1/radar/licensePlate", true)]
    [InlineData("GET", "/api/v1/vehicleWorkflows/workflow/licensePlate", true)]
    [InlineData("POST", "/api/v1/load-radar-agent/passages/prepare", true)]
    public async Task Query_only_mode_preserves_capture_and_machine_routes(string method, string path, bool expected)
    {
        var reached = false;
        var context = new DefaultHttpContext();
        context.Request.Method = method;
        context.Request.Path = path;
        context.Response.Body = new MemoryStream();
        await new ExternalPartnerGateMiddleware(_ => { reached = true; return Task.CompletedTask; },
            Options.Create(new IntegrationOptions { ExternalPartnerEnabled = false, ExternalQueriesEnabled = true }))
            .InvokeAsync(context);
        Assert.Equal(expected, reached);
        Assert.Equal(expected ? 200 : 503, context.Response.StatusCode);
    }

    [Theory]
    [InlineData(false, null, false)]
    [InlineData(true, null, true)]
    [InlineData(true, false, false)]
    [InlineData(false, true, true)]
    public async Task Query_switch_defaults_to_legacy_setting(bool legacy, bool? queries, bool expected)
    {
        var reached = false;
        var context = new DefaultHttpContext();
        context.Request.Method = HttpMethods.Get;
        context.Request.Path = "/api/v1/vehicleDimensions";
        context.Response.Body = new MemoryStream();
        await new ExternalPartnerGateMiddleware(_ => { reached = true; return Task.CompletedTask; },
            Options.Create(new IntegrationOptions { ExternalPartnerEnabled = legacy, ExternalQueriesEnabled = queries }))
            .InvokeAsync(context);
        Assert.Equal(expected, reached);
    }

    [Theory]
    [InlineData("/api/v1/vehicleDimensions")]
    [InlineData("/api/v1/coalVolumes")]
    public async Task Signed_queries_work_without_partner_url_but_still_require_authentication(string path)
    {
        using var factory = new QueryOnlyFactory();
        using var client = factory.CreateClient();
        using var missing = await client.GetAsync(path + "?limit=1");
        Assert.Equal(HttpStatusCode.Unauthorized, missing.StatusCode);
        using var wrong = await SendSignedAsync(client, path, "wrong-secret");
        Assert.Equal(HttpStatusCode.Unauthorized, wrong.StatusCode);
        using var expired = await SendSignedAsync(client, path, QueryOnlyFactory.Secret, DateTimeOffset.UtcNow.AddMinutes(-2));
        Assert.Equal(HttpStatusCode.Unauthorized, expired.StatusCode);
        using var valid = await SendSignedAsync(client, path);
        Assert.Equal(HttpStatusCode.OK, valid.StatusCode);
        using var json = JsonDocument.Parse(await valid.Content.ReadAsStringAsync());
        Assert.Equal(new[] { "code", "data", "message" }, json.RootElement.EnumerateObject().Select(p => p.Name).OrderBy(n => n));
        Assert.Equal(200, json.RootElement.GetProperty("code").GetInt32());
        Assert.Equal(JsonValueKind.Array, json.RootElement.GetProperty("data").ValueKind);
        Assert.Equal(0, factory.Outbound.Calls);
    }

    [Theory]
    [InlineData("/api/v1/vehicleDimensions", 429)]
    [InlineData("/api/v1/coalVolumes", 400)]
    public async Task Query_only_mode_keeps_contract_rate_limits(string path, int status)
    {
        using var factory = new QueryOnlyFactory();
        using var client = factory.CreateClient();
        // 预热主机，避免首次启动时间跨越限流窗口。
        using var health = await client.GetAsync("/healthz");
        for (var i = 0; i < 3; i++)
        {
            using var allowed = await SendSignedAsync(client, path);
            Assert.Equal(HttpStatusCode.OK, allowed.StatusCode);
        }
        using var limited = await SendSignedAsync(client, path);
        Assert.Equal(status, (int)limited.StatusCode);
    }

    [Fact]
    public async Task Query_only_host_does_not_start_plate_worker_or_call_partner()
    {
        using var factory = new QueryOnlyFactory();
        using var http = factory.CreateClient();
        var settings = factory.Services.GetRequiredService<IOptions<IntegrationOptions>>().Value;
        Assert.True(settings.ExternalQueriesEnabled);
        Assert.False(settings.ExternalPartnerEnabled);
        Assert.False(settings.EnableMockLicensePlate);
        Assert.Empty(settings.PartnerBaseUrl);
        Assert.DoesNotContain(factory.Services.GetServices<IHostedService>(), service => service is PlateAcquisitionWorker);
        Assert.IsType<NullIntegrationRepository>(factory.Services.GetRequiredService<IIntegrationRepository>());
        var partner = factory.Services.GetRequiredService<PartnerLicensePlateClient>();
        await Assert.ThrowsAsync<InvalidOperationException>(() => partner.GetLicensePlateAsync("CAM_001"));
        Assert.Equal(0, factory.Outbound.Calls);
    }

    private static async Task<HttpResponseMessage> SendSignedAsync(HttpClient client, string path,
        string secret = QueryOnlyFactory.Secret, DateTimeOffset? at = null)
    {
        var timestamp = (at ?? DateTimeOffset.UtcNow).ToUnixTimeMilliseconds().ToString(System.Globalization.CultureInfo.InvariantCulture);
        using var request = new HttpRequestMessage(HttpMethod.Get, path + "?limit=1");
        request.Headers.Add("Timestamp", timestamp);
        request.Headers.Add("Signature", ApiSignatureHelper.GenerateSignature(timestamp, secret));
        return await client.SendAsync(request);
    }

    private sealed class QueryOnlyFactory : WebApplicationFactory<Program>
    {
        internal const string Secret = "query-tests-only-not-a-real-partner-secret";
        private static int _address;
        private readonly string _root = Path.Combine(Path.GetTempPath(), "partner-query-tests-" + Guid.NewGuid().ToString("N"));
        internal readonly RejectOutboundHandler Outbound = new();

        internal QueryOnlyFactory()
        {
            Directory.CreateDirectory(_root);
            File.WriteAllText(Path.Combine(_root, "appsettings.json"), JsonSerializer.Serialize(new
            {
                Database = new { Enabled = false, InitializeOnStartup = false },
                Auth = new { JwtSecret = "query-tests-jwt-only-0123456789abcdef" },
                Integration = new
                {
                    ExternalPartnerEnabled = false,
                    ExternalQueriesEnabled = true,
                    EnableMockLicensePlate = false,
                    AppSecret = Secret,
                    PartnerBaseUrl = ""
                },
                Radar = new { BaseUrl = "http://127.0.0.1:65530" },
                LoadRadar = new { Enabled = false }
            }));
        }

        protected override void ConfigureWebHost(IWebHostBuilder builder)
        {
            builder.UseEnvironment("Production").UseContentRoot(_root);
            builder.ConfigureServices((context, services) =>
            {
                // 必须在宿主启动前确认读到隔离文件，禁止退回工作目录中的真实库配置。
                if (context.Configuration.GetValue("Database:Enabled", true) ||
                    context.Configuration.GetValue("Integration:ExternalPartnerEnabled", true) ||
                    context.Configuration["Integration:AppSecret"] != Secret)
                    throw new InvalidOperationException("Isolated query test configuration was not loaded.");
                services.AddSingleton<IStartupFilter>(new AddressFilter(IPAddress.Parse("2001:db8::" +
                    Interlocked.Increment(ref _address).ToString("x"))));
                services.AddHttpClient<PartnerLicensePlateClient>().ConfigurePrimaryHttpMessageHandler(() => Outbound);
            });
        }

        protected override void Dispose(bool disposing)
        {
            base.Dispose(disposing);
            if (disposing && Directory.Exists(_root))
            {
                var prefix = Path.Combine(Path.GetTempPath(), "partner-query-tests-");
                if (!Path.GetFullPath(_root).StartsWith(prefix, StringComparison.OrdinalIgnoreCase))
                    throw new InvalidOperationException("Query test cleanup path is outside its temporary prefix.");
                Directory.Delete(_root, true);
            }
        }
    }

    private sealed class AddressFilter(IPAddress address) : IStartupFilter
    {
        public Action<IApplicationBuilder> Configure(Action<IApplicationBuilder> next) => app =>
        {
            app.Use((context, continuation) => { context.Connection.RemoteIpAddress = address; return continuation(context); });
            next(app);
        };
    }

    private sealed class RejectOutboundHandler : HttpMessageHandler
    {
        internal int Calls;
        protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
        {
            Interlocked.Increment(ref Calls);
            throw new InvalidOperationException("Query-only tests must never call a partner or radar over HTTP.");
        }
    }
}
