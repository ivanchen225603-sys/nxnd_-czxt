using System.Net;
using System.Reflection;
using System.Text.Json;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.TestHost;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Options;
using Xunit;

namespace CarPointCloudWebApi.Tests;

/// <summary>隔离 MVC 查询合同；固定测试角色不替代 JWT 验收，不读取配置或连接数据库。</summary>
public sealed class LoadRadarHistoryContractTests
{
    [Theory]
    [InlineData("")]
    [InlineData("&from=2026-01-01T00%3A00%3A00Z")]
    [InlineData("&to=2026-01-01T08%3A00%3A00%2B08%3A00")]
    [InlineData("&from=2026-01-01T08%3A00%3A00%2B08%3A00&to=2026-01-01T00%3A00%3A00Z")]
    public async Task QueryParametersReachRepositoryAndListExcludesCloud(string range)
    {
        using var fixture = new Fixture();
        var response = await fixture.Client.GetAsync("/api/v1/load-radar/history?nodeId=n1&channelId=CH1&licensePlate=" +
            Uri.EscapeDataString("宁A25771") + "&beforeId=m-before&take=7" + range);
        Assert.Equal(HttpStatusCode.OK, response.StatusCode);
        var arguments = Assert.IsType<object[]>(fixture.Repository.LastArguments);
        Assert.Equal(new object[] { "n1", "CH1", "宁A25771", "m-before", 7 }, arguments.Take(5));
        var instant = DateTimeOffset.Parse("2026-01-01T00:00:00Z");
        if (range.Contains("from=")) Assert.Equal(instant, Assert.IsType<DateTimeOffset>(arguments[5]));
        else Assert.Null(arguments[5]);
        if (range.Contains("to=")) Assert.Equal(instant, Assert.IsType<DateTimeOffset>(arguments[6]));
        else Assert.Null(arguments[6]);
        using var json = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
        Assert.Empty(json.RootElement.GetProperty("data")[0].GetProperty("measurement").GetProperty("points").EnumerateArray());
        Assert.Single(fixture.Repository.Row.Measurement.Points);
    }

    [Theory]
    [InlineData("from=invalid")]
    [InlineData("to=2026-13-01")]
    [InlineData("from=2026-01-02T00%3A00%3A00Z&to=2026-01-01T00%3A00%3A00Z")]
    public async Task InvalidTimeDoesNotQueryRepository(string query)
    {
        using var fixture = new Fixture();
        Assert.Equal(HttpStatusCode.BadRequest, (await fixture.Client.GetAsync("/api/v1/load-radar/history?" + query)).StatusCode);
        Assert.Null(fixture.Repository.LastArguments);
    }

    private sealed class Fixture : IDisposable
    {
        private readonly TestServer _server;
        public HttpClient Client { get; }
        public HistoryRepository Repository { get; }
        public Fixture()
        {
            var repository = DispatchProxy.Create<ILoadRadarRepository, HistoryRepository>();
            Repository = (HistoryRepository)repository;
            _server = new TestServer(new WebHostBuilder().ConfigureServices(services =>
            {
                services.AddSingleton(Options.Create(new LoadRadarOptions { Enabled = true }));
                services.AddSingleton(repository);
                services.AddSingleton<IIntegrationRepository>(new NullIntegrationRepository());
                services.AddSingleton<LoadRadarAssetStore>();
                services.AddSingleton<LoadRadarService>();
                services.AddSingleton<LoadRadarLiveClient>();
                services.AddHttpClient("LoadRadarPreview");
                services.AddControllers().AddApplicationPart(typeof(LoadRadarController).Assembly);
            }).Configure(app =>
            {
                app.UseMiddleware<ApiExceptionMiddleware>();
                app.Use(async (context, next) =>
                {
                    context.Items[JwtAuthMiddleware.UserInfoKey] = new TokenUserInfo { Username = "history-test", RoleCode = "viewer" };
                    await next();
                });
                app.UseRouting();
                app.UseEndpoints(endpoints => endpoints.MapControllers());
            }));
            Client = _server.CreateClient();
        }
        public void Dispose() { Client.Dispose(); _server.Dispose(); }
    }

    public class HistoryRepository : DispatchProxy
    {
        public object?[]? LastArguments;
        public readonly LoadMeasurementRecord Row = new("p", "hash", new LoadMeasurementInput
        {
            MeasurementId = "m",
            Points = [[1, 2, 3]],
            MeasuredAt = DateTimeOffset.Parse("2026-01-01T00:00:00Z")
        }, DateTimeOffset.UtcNow);
        protected override object? Invoke(MethodInfo? method, object?[]? args)
        {
            if (method?.Name != nameof(ILoadRadarRepository.GetHistory)) throw new NotSupportedException();
            LastArguments = args;
            return new[] { Row };
        }
    }
}
