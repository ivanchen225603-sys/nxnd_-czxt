using System.Net;
using System.Reflection;
using System.Text.Json;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.TestHost;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
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

    [Fact]
    public async Task Display_queries_resolve_profile_DI_and_batch_distinct_plates_but_raw_read_is_unchanged()
    {
        using var fixture = new Fixture(enableCargo: true);
        using var history = JsonDocument.Parse(await fixture.Client.GetStringAsync("/api/v1/load-radar/history?take=3"));
        var rows = history.RootElement.GetProperty("data");
        Assert.Equal(3, rows.GetArrayLength());
        Assert.Equal("half", rows[0].GetProperty("cargoEstimate").GetProperty("loadLevel").GetString());
        Assert.Equal(0.55m, rows[1].GetProperty("cargoEstimate").GetProperty("loadRatio").GetDecimal());
        Assert.Equal("loaded_unknown_capacity", rows[2].GetProperty("cargoEstimate").GetProperty("loadLevel").GetString());
        Assert.Equal(new[] { "宁A20181", "宁A20182" }, Assert.Single(fixture.Profiles.Batches));
        Assert.Empty(rows[0].GetProperty("measurement").GetProperty("points").EnumerateArray());

        using var detail = JsonDocument.Parse(await fixture.Client.GetStringAsync("/api/v1/load-radar/measurements/m"));
        var data = detail.RootElement.GetProperty("data");
        var cargo = data.GetProperty("cargoEstimate");
        Assert.Equal(0.55m, cargo.GetProperty("loadRatio").GetDecimal());
        Assert.Equal(4, cargo.EnumerateObject().Count());
        Assert.Equal(27.5m, data.GetProperty("measurement").GetProperty("volumeM3").GetDecimal());
        Assert.Single(data.GetProperty("measurement").GetProperty("points").EnumerateArray());
        Assert.Equal(2, fixture.Profiles.Batches.Count);
        Assert.Null(fixture.Service.Get("m").CargoEstimate);
        Assert.Equal(2, fixture.Profiles.Batches.Count);
    }

    private sealed class Fixture : IDisposable
    {
        private readonly TestServer _server;
        public HttpClient Client { get; }
        public HistoryRepository Repository { get; }
        public CargoProfiles Profiles { get; } = new();
        public LoadRadarService Service => _server.Services.GetRequiredService<LoadRadarService>();
        public Fixture(bool enableCargo = false)
        {
            var repository = DispatchProxy.Create<ILoadRadarRepository, HistoryRepository>();
            Repository = (HistoryRepository)repository;
            if (enableCargo)
            {
                var row = Repository.Row;
                var binding = new LoadPassageBinding("p", "workflow", "宁A20181", "CAM_001", "vehicle", "test", "", DateTimeOffset.UtcNow);
                Repository.Rows = [
                    row with { Binding = binding },
                    row with { Binding = binding, Measurement = row.Measurement with { MeasurementId = "m-2" } },
                    row with { Binding = binding with { LicensePlate = "宁A20182" }, Measurement = row.Measurement with { MeasurementId = "m-3" } }
                ];
            }
            _server = new TestServer(new WebHostBuilder().ConfigureServices(services =>
            {
                var configuration = new ConfigurationBuilder().AddInMemoryCollection(new Dictionary<string, string?>
                {
                    ["LoadRadar:Enabled"] = "true"
                }).Build();
                services.Configure<LoadRadarOptions>(configuration.GetSection("LoadRadar"));
                if (enableCargo) services.AddSingleton<IEmptyVehicleProfileRepository>(Profiles);
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

    private sealed class CargoProfiles : IEmptyVehicleProfileRepository
    {
        public List<string[]> Batches { get; } = [];
        public EmptyVehicleProfileRecord? GetByLicensePlate(string licensePlate) => throw new NotSupportedException();
        public IReadOnlyDictionary<string, EmptyVehicleProfileRecord> GetDimensionsByLicensePlates(IReadOnlyList<string> licensePlates)
        {
            Batches.Add(licensePlates.ToArray());
            return new Dictionary<string, EmptyVehicleProfileRecord>
            {
                ["宁A20181"] = new()
                {
                    LicensePlate = "宁A20181", TrailerLength = 1000m, TrailerWidth = 250m, TrailerBodyHeight = 200m
                }
            };
        }
    }

    public class HistoryRepository : DispatchProxy
    {
        public object?[]? LastArguments;
        public readonly LoadMeasurementRecord Row = new("p", "hash", new LoadMeasurementInput
        {
            MeasurementId = "m",
            ChannelId = "CH1", Verdict = "loaded", VolumeStatus = "success", VolumeM3 = 27.5m,
            Points = [[1, 2, 3]],
            MeasuredAt = DateTimeOffset.Parse("2026-01-01T00:00:00Z")
        }, DateTimeOffset.UtcNow);
        public IReadOnlyList<LoadMeasurementRecord>? Rows;
        protected override object? Invoke(MethodInfo? method, object?[]? args)
        {
            if (method?.Name == nameof(ILoadRadarRepository.GetMeasurement))
                return Rows?.SingleOrDefault(row => row.Measurement.MeasurementId == (string?)args![0]);
            if (method?.Name != nameof(ILoadRadarRepository.GetHistory)) throw new NotSupportedException();
            LastArguments = args;
            return Rows ?? new[] { Row };
        }
    }
}
