using System.Net;
using System.Text.Json;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.TestHost;
using Microsoft.Extensions.DependencyInjection;
using Xunit;

namespace CarPointCloudWebApi.Tests;

/// <summary>独立MVC宿主与内存仓储，不启动采集、后台任务或数据库连接。</summary>
public sealed class VolumeQueryTests : IDisposable
{
    private readonly FixtureRepository _repository = new();
    private readonly TestServer _server;
    private readonly HttpClient _client;

    public VolumeQueryTests()
    {
        _server = new TestServer(new WebHostBuilder().ConfigureServices(services =>
        {
            services.AddSingleton<IIntegrationRepository>(_repository);
            services.AddControllers().AddApplicationPart(typeof(VolumeQueryController).Assembly);
        }).Configure(app =>
        {
            app.UseRouting();
            app.Use(async (context, next) =>
            {
                // 只模拟已认证角色上下文；MVC继续执行真实RequireRole授权和模型绑定。
                if (context.Request.Headers.TryGetValue("X-Test-Role", out var role))
                    context.Items[JwtAuthMiddleware.UserInfoKey] = new TokenUserInfo
                    { UserId = 1, Username = "query-test", RoleCode = role.ToString() };
                await next();
            });
            app.UseEndpoints(endpoints => endpoints.MapControllers());
        }));
        _client = _server.CreateClient();
        _client.DefaultRequestHeaders.Add("X-Test-Role", "viewer");
    }

    [Fact]
    public async Task Missing_filters_default_to_today_all_lanes_and_first_page()
    {
        var before = BusinessClock.Now.Date;
        using var response = await _client.GetAsync("/api/volume-query");
        response.EnsureSuccessStatusCode();
        var query = Assert.IsType<VolumeQuery>(_repository.LastQuery);
        Assert.InRange(query.From, before, BusinessClock.Now.Date);
        Assert.Equal(query.From.AddDays(1), query.Until);
        Assert.Null(query.CameraIdentifier);
        Assert.Null(query.LicensePlate);
        Assert.Equal(1, query.Page);
        Assert.Equal(10, query.PageSize);
    }

    [Theory]
    [InlineData("CH1", "CAM_001")]
    [InlineData("CH2", "CAM_002")]
    [InlineData("CH3", "CAM_003")]
    [InlineData("ch4", "CAM_004")]
    public async Task Filters_cross_the_HTTP_boundary_with_inclusive_end_date(string channel, string camera)
    {
        using var response = await _client.GetAsync("/api/volume-query?startDate=2026-10-31&endDate=2026-11-02" +
            "&licensePlate=" + Uri.EscapeDataString(" 宁a48394q ") + $"&channelId={channel}&page=2&pageSize=20");
        response.EnsureSuccessStatusCode();
        Assert.Equal(new VolumeQuery(new(2026, 10, 31), new(2026, 11, 3), "宁A48394Q", camera, 2, 20), _repository.LastQuery);
    }

    [Theory]
    [InlineData("startDate=2026-11-03&endDate=2026-11-02")]
    [InlineData("startDate=not-a-date")]
    [InlineData("endDate=9999-12-31")]
    [InlineData("channelId=CH5")]
    [InlineData("channelId=CH4%27%20OR%201=1")]
    [InlineData("page=0")]
    [InlineData("page=100001")]
    [InlineData("pageSize=0")]
    [InlineData("pageSize=101")]
    [InlineData("pageSize=abc")]
    public async Task Invalid_filters_are_rejected_before_repository_access(string query)
    {
        using var response = await _client.GetAsync("/api/volume-query?" + query);
        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
        Assert.Null(_repository.LastQuery);
    }

    [Fact]
    public async Task Overlong_plate_is_rejected_once_at_boundary()
    {
        using var response = await _client.GetAsync("/api/volume-query?licensePlate=" + new string('A', 65));
        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
        Assert.Null(_repository.LastQuery);
    }

    [Fact]
    public async Task Page_contract_preserves_zero_volume_and_excludes_internal_details()
    {
        using var response = await _client.GetAsync("/api/volume-query?startDate=2026-11-02&endDate=2026-11-02&page=2&pageSize=1");
        response.EnsureSuccessStatusCode();
        using var json = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
        var data = json.RootElement.GetProperty("data");
        Assert.Equal(3, data.GetProperty("total").GetInt64());
        Assert.Equal(2, data.GetProperty("page").GetInt32());
        Assert.Equal(1, data.GetProperty("pageSize").GetInt32());
        var item = Assert.Single(data.GetProperty("items").EnumerateArray());
        Assert.Equal("2026-11-02T08:00:00", item.GetProperty("collectedAt").GetString());
        Assert.Equal("宁A48394Q", item.GetProperty("licensePlate").GetString());
        Assert.Equal("CH4", item.GetProperty("channelId").GetString());
        Assert.Equal("4号磅", item.GetProperty("scaleName").GetString());
        Assert.Equal(0m, item.GetProperty("volumeM3").GetDecimal());
        Assert.Equal(5, item.EnumerateObject().Count());
    }

    [Fact]
    public async Task Empty_and_last_inclusive_day_are_paged_without_using_created_time()
    {
        using var response = await _client.GetAsync("/api/volume-query?startDate=2026-11-03&endDate=2026-11-03");
        response.EnsureSuccessStatusCode();
        using var json = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
        var data = json.RootElement.GetProperty("data");
        Assert.Equal(0, data.GetProperty("total").GetInt64());
        Assert.Empty(data.GetProperty("items").EnumerateArray());
    }

    [Theory]
    [InlineData("admin", 200)]
    [InlineData("viewer", 200)]
    [InlineData("operator", 403)]
    [InlineData(null, 401)]
    public async Task Only_logged_in_admin_and_viewer_can_read(string? role, int expected)
    {
        _client.DefaultRequestHeaders.Remove("X-Test-Role");
        if (role is not null) _client.DefaultRequestHeaders.Add("X-Test-Role", role);
        using var response = await _client.GetAsync("/api/volume-query");
        Assert.Equal(expected, (int)response.StatusCode);
        if (expected != 200) Assert.Null(_repository.LastQuery);
    }

    [Fact]
    public async Task Endpoint_accepts_no_write_method()
    {
        using var response = await _client.PostAsync("/api/volume-query", null);
        Assert.Equal(HttpStatusCode.MethodNotAllowed, response.StatusCode);
        Assert.Null(_repository.LastQuery);
    }

    private sealed class FixtureRepository : NullIntegrationRepository
    {
        public VolumeQuery? LastQuery { get; private set; }
        public override Task<VolumeQueryPage> QueryVolumesAsync(VolumeQuery query, CancellationToken cancellationToken)
        {
            LastQuery = query;
            VolumeQueryRow[] rows = [new(new(2026, 11, 2, 23, 59, 59), "宁A48394Q", "CH1", "1号磅", 52.3m),
                new(new(2026, 11, 2, 8, 0, 0), "宁A48394Q", "CH4", "4号磅", 0m),
                new(new(2026, 11, 2, 0, 0, 0), "宁A19080C", "CH2", "2号磅", 49.8m)];
            var matching = rows.Where(row => row.CollectedAt >= query.From && row.CollectedAt < query.Until &&
                (query.LicensePlate is null || row.LicensePlate == query.LicensePlate) &&
                (query.CameraIdentifier is null || "CAM_00" + row.ChannelId![2] == query.CameraIdentifier)).ToArray();
            return Task.FromResult(new VolumeQueryPage(matching.Skip((query.Page - 1) * query.PageSize)
                .Take(query.PageSize).ToArray(), matching.Length, query.Page, query.PageSize));
        }
    }

    public void Dispose() { _client.Dispose(); _server.Dispose(); }
}
