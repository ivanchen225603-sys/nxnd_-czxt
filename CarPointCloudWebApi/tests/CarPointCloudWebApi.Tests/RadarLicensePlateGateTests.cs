using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.FileProviders;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using Xunit;

namespace CarPointCloudWebApi.Tests;

/// <summary>
/// JT128 载货雷达按点位拉车牌（GET /api/v1/radar/licensePlate）的三道门。
///
/// 背景：这条链此前在麒麟生产环境**一辆车都取不到牌**，而且三道门给出三种
/// 互不相关的错误码，排查时很容易只看到最外面那一层：
///   1. RadarDeviceAuthorizationMiddleware -> 403（来源白名单，另有用例覆盖）
///   2. ExternalPartnerGateMiddleware      -> 503「合作方接入已关闭」（本文件）
///   3. 控制器自身仅 Development 可见       -> 404「接口不存在」（本文件）
/// 三道门任意一道没开，现象都是"取不到车牌"，但提示词会把人引向不同方向。
/// </summary>
public sealed class RadarLicensePlateGateTests
{
    // ───────────────────────────── 第 2 道门：合作方开关

    [Theory]
    [InlineData("/api/v1/radar/licensePlate", true)]
    [InlineData("/api/v1/radar/licensePlate?cameraIdentifier=CAM_002", true)]
    [InlineData("/api/v1/radar/vehicle-arrival", false)]
    [InlineData("/api/v1/licensePlateRecords/fetch", false)]
    [InlineData("/api/v1/coalVolumes", false)]
    public async Task Partner_gate_only_lets_the_self_owned_pull_endpoint_through(string path, bool passes)
    {
        var reached = false;
        var middleware = new ExternalPartnerGateMiddleware(
            _ => { reached = true; return Task.CompletedTask; },
            Options.Create(new IntegrationOptions { ExternalPartnerEnabled = false }));

        var context = new DefaultHttpContext();
        context.Request.Path = new PathString(path.Split('?')[0]);
        context.Response.Body = new MemoryStream();
        await middleware.InvokeAsync(context);

        Assert.Equal(passes, reached);
        Assert.Equal(passes ? 200 : StatusCodes.Status503ServiceUnavailable, context.Response.StatusCode);
    }

    [Fact]
    public async Task Partner_gate_is_transparent_once_the_partner_chain_is_enabled()
    {
        var reached = false;
        var middleware = new ExternalPartnerGateMiddleware(
            _ => { reached = true; return Task.CompletedTask; },
            Options.Create(new IntegrationOptions { ExternalPartnerEnabled = true }));

        var context = new DefaultHttpContext();
        context.Request.Path = new PathString("/api/v1/radar/vehicle-arrival");
        await middleware.InvokeAsync(context);

        Assert.True(reached);
    }

    // ───────────────────────────── 第 3 道门：可见性开关

    [Theory]
    [InlineData("Production", false, 404)]   // 改动前的生产行为：一律 404，这也是现场的症状
    [InlineData("Production", true, 200)]    // 新增：显式打开才可见
    [InlineData("Development", false, 200)]  // 原有行为不变，不打断既有用例
    [InlineData("Development", true, 200)]
    public async Task Endpoint_visibility_follows_the_explicit_switch(string environment, bool mockEnabled, int expected)
    {
        var controller = CreateController(environment, mockEnabled);
        var result = await controller.GetLicensePlateForRadar("CAM_002", CancellationToken.None);
        Assert.Equal(expected, Assert.IsAssignableFrom<ObjectResult>(result.Result).StatusCode);
    }

    [Fact]
    public async Task Mock_response_declares_its_source_so_downstream_can_refuse_it()
    {
        // ★ 车牌是空车/载货体积配对的唯一键，错配不会报错但结果全错
        //   （实测 12 种错配里 7 种密度都落在合理区间）。所以模拟值必须自报家门。
        var controller = CreateController("Production", mockEnabled: true);
        var result = await controller.GetLicensePlateForRadar("CAM_002", CancellationToken.None);
        var payload = Assert.IsType<ApiResponse<object>>(Assert.IsType<OkObjectResult>(result.Result).Value);

        var json = System.Text.Json.JsonSerializer.Serialize(payload.Data);
        using var document = System.Text.Json.JsonDocument.Parse(json);
        Assert.Equal("MOCK", document.RootElement.GetProperty("source").GetString());
        Assert.Equal("CAM_002", document.RootElement.GetProperty("cameraId").GetString());
        Assert.False(string.IsNullOrWhiteSpace(document.RootElement.GetProperty("licensePlate").GetString()));
    }

    [Theory]
    [InlineData("")]
    [InlineData("CAM 002")]
    [InlineData("测试通道")]
    public async Task Illegal_camera_identifier_is_still_rejected(string cameraIdentifier)
    {
        var controller = CreateController("Production", mockEnabled: true);
        var result = await controller.GetLicensePlateForRadar(cameraIdentifier, CancellationToken.None);
        Assert.Equal(400, Assert.IsAssignableFrom<ObjectResult>(result.Result).StatusCode);
    }

    // ───────────────────────────── 构造

    private static IntegrationController CreateController(string environment, bool mockEnabled)
    {
        var configuration = new ConfigurationBuilder().AddInMemoryCollection(new Dictionary<string, string?>
        {
            ["Database:Enabled"] = "false",
            ["Radar:LicensePlateMockEnabled"] = mockEnabled ? "true" : "false"
        }).Build();

        var repository = new NullIntegrationRepository();
        var partnerClient = new PartnerLicensePlateClient(
            new HttpClient(),
            new StaticOptionsMonitor<IntegrationOptions>(new IntegrationOptions { ExternalPartnerEnabled = false }),
            repository, configuration, NullLogger<PartnerLicensePlateClient>.Instance,
            new HttpContextAccessor());

        return new IntegrationController(
            new IntegrationQueryService(repository, configuration),
            partnerClient,
            new MockRadarLicensePlateSource(configuration, new StubEnvironment(environment)),
            configuration);
    }

    private sealed class StubEnvironment(string environmentName) : IWebHostEnvironment
    {
        public string EnvironmentName { get; set; } = environmentName;
        public string ApplicationName { get; set; } = "CarPointCloudWebApi";
        public string WebRootPath { get; set; } = Path.GetTempPath();
        public IFileProvider WebRootFileProvider { get; set; } = new NullFileProvider();
        public string ContentRootPath { get; set; } = Path.GetTempPath();
        public IFileProvider ContentRootFileProvider { get; set; } = new NullFileProvider();
    }

    private sealed class StaticOptionsMonitor<T>(T currentValue) : IOptionsMonitor<T>, IOptions<T> where T : class
    {
        public T CurrentValue { get; } = currentValue;
        public T Value => CurrentValue;
        public T Get(string? name) => CurrentValue;
        public IDisposable? OnChange(Action<T, string?> listener) => null;
    }
}
