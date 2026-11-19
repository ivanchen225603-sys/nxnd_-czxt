using System.Net;
using System.Text.Json;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public class PartnerContractV4Tests
{
    private static readonly JsonSerializerOptions Json = new(JsonSerializerDefaults.Web);
    private static IntegrationQueryService Service(bool database = false, IIntegrationRepository? repository = null) =>
        new(repository ?? new NullIntegrationRepository(), new ConfigurationBuilder().AddInMemoryCollection(
            new Dictionary<string, string?>
            {
                ["Database:Enabled"] = database.ToString(),
                ["Auth:JwtSecret"] = new string('x', 40)
            }).Build());

    [Fact]
    public async Task Controllers_RejectMissingQueryAndReturnOnlyDocumentFields()
    {
        var controller = new IntegrationController(Service(), null!, null!, new ConfigurationBuilder().Build())
        { ControllerContext = new ControllerContext { HttpContext = new DefaultHttpContext() } };
        Assert.IsType<BadRequestObjectResult>((await controller.GetVehicleDimensions(null, null)).Result);
        Assert.IsType<BadRequestObjectResult>((await controller.GetCoalVolumes(null, null)).Result);
        controller.Request.QueryString = new("?limit=1");
        var response = Assert.IsType<OkObjectResult>((await controller.GetCoalVolumes(1, null)).Result);
        Assert.IsType<PartnerApiResponse<List<PartnerCoalVolumeDto>>>(response.Value);
        using var json = JsonDocument.Parse(JsonSerializer.Serialize(response.Value, Json));
        Assert.Equal(new[] { "code", "data", "message" },
            json.RootElement.EnumerateObject().Select(x => x.Name).OrderBy(x => x).ToArray());
    }

    [Theory]
    [InlineData("/api/v1/coalVolumes", true)]
    [InlineData("/API/V1/VEHICLEDIMENSIONS/", true)]
    [InlineData("/api/v1/internal/pointcloud/vehicleDimensions", false)]
    [InlineData("/api/auth/login", false)]
    public void ErrorEnvelope_UsesContractOnlyForPartnerQueries(string path, bool partner)
    {
        using var json = JsonDocument.Parse(JsonSerializer.Serialize(PartnerResponseFactory.Fail(path, 400, "error"), Json));
        Assert.Equal(partner ? 3 : 4, json.RootElement.EnumerateObject().Count());
        Assert.Equal(!partner, json.RootElement.TryGetProperty("success", out _));
        Assert.Equal(JsonValueKind.Null, json.RootElement.GetProperty("data").ValueKind);
    }

    [Fact]
    public void VehicleProjection_CapsArraysAndExcludesTailAndInternalMetadata()
    {
        var source = new VehicleDimensionsDto
        {
            Ribs = Enumerable.Range(1, 9).Select(i => new RibDto { RibId = i, DistToPrev = i * 10 }).Reverse().ToList(),
            Wires = [new WireDto { WireId = 2, DistToPrev = 50 }, new WireDto { WireId = 1, DistToPrev = 30 }],
            HighLowPlates = Enumerable.Range(1, 4).Select(i => new HighLowPlateDto
            { PlateId = i, HighestToGround = 200, LowestToGround = 100, LengthCm = i * 150 }).ToList()
        };
        var row = new VehicleDimensionDto
        {
            RecordId = 42,
            WorkflowId = "internal",
            PairId = "pair",
            LicensePlate = "宁A25771",
            CameraIdentifier = "CAM_001",
            ActualRibCount = 8,
            MaximumWidthCm = 255.5m,
            Dimensions = source
        };
        using var json = JsonDocument.Parse(JsonSerializer.Serialize(PartnerQueryMapper.Vehicle(row), Json));
        Assert.Equal(new[] { "cameraIdentifier", "dimensions", "getTime", "licensePlate" },
            json.RootElement.EnumerateObject().Select(x => x.Name).OrderBy(x => x).ToArray());
        var dimensions = json.RootElement.GetProperty("dimensions");
        Assert.Equal(255.5m, dimensions.GetProperty("trailerWidth").GetDecimal());
        Assert.Equal(6, dimensions.GetProperty("ribs").GetArrayLength());
        Assert.Empty(dimensions.GetProperty("wires").EnumerateArray());
        Assert.Equal(3, dimensions.GetProperty("highLowPlates").GetArrayLength());
        Assert.Equal(150, dimensions.GetProperty("highLowPlates")[0].GetProperty("highestToGround").GetDecimal());
        Assert.Equal(9, source.Ribs.Count);
        row.ActualRibCount = 2;
        var mapped = Assert.IsType<PartnerVehicleDimensionsDto>(PartnerQueryMapper.Vehicle(row).Dimensions);
        Assert.Equal(2, mapped.Ribs.Count);
    }

    [Fact]
    public void OldPlateHeightAndMissingCoalTimes_AreNeverReinterpreted()
    {
        var row = new VehicleDimensionDto
        {
            Dimensions = new VehicleDimensionsDto
            { TrailerWidth = 240, HighLowPlates = [new HighLowPlateDto { PlateId = 1, HighestToGround = 150, LowestToGround = 120 }] }
        };
        var dimensions = Assert.IsType<PartnerVehicleDimensionsDto>(PartnerQueryMapper.Vehicle(row).Dimensions);
        Assert.Null(Assert.Single(dimensions.HighLowPlates).HighestToGround);
        Assert.Null(dimensions.TrailerWidth);
        Assert.Throws<ArgumentException>(() => PartnerQueryMapper.Coal(
            new CoalVolumeDto { MeasureTime = DateTime.Now, Volume = 12, Unit = "m3" }));
    }

    [Fact]
    public void CoalProjection_PreservesSelectedPartnerIdentityAndActualInterval()
    {
        var start = new DateTime(2026, 10, 12, 9, 0, 1);
        var coal = PartnerQueryMapper.Coal(new CoalVolumeDto
        {
            EnterId = "001-ID",
            CaptureTime = start,
            MeasurementStartedAt = start,
            MeasurementEndedAt = start.AddSeconds(3)
        });
        Assert.Equal("001-ID", coal.EnterId);
        Assert.Equal("2026-10-12 09:00:01", coal.CaptureTime);
        Assert.Equal("2026-10-12 09:00:04", coal.MeasureEndTime);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task CoalIngest_OnlyPairedTimesEstablishAnActualInterval(bool paired)
    {
        var repository = new CaptureCoalRepository();
        var start = new DateTime(2026, 10, 12, 9, 0, 0);
        var request = new PointCloudCoalVolumeIngestRequest
        {
            IdempotencyKey = "v4-coal-test",
            LicensePlate = "宁A25771",
            CameraIdentifier = "CAM_001",
            Volume = 12,
            Unit = "m³",
            Status = "success",
            MeasureTime = start,
            MeasureEndTime = paired ? start.AddSeconds(3) : null
        };
        await Service(true, repository).IngestCoalVolumeAsync(request);
        var record = Assert.IsType<CoalVolumeRecord>(repository.Record);
        Assert.Equal(start, record.MeasureTime);
        Assert.Equal(paired ? start : (DateTime?)null, record.MeasurementStartedAt);
        Assert.Equal(request.MeasureEndTime, record.MeasurementEndedAt);
        if (paired)
        {
            var hash = record.PayloadHash;
            request.MeasureEndTime = start.AddSeconds(4);
            await Service(true, repository).IngestCoalVolumeAsync(request);
            Assert.NotEqual(hash, repository.Record!.PayloadHash);
        }
    }

    private sealed class CaptureCoalRepository : NullIntegrationRepository
    {
        public CoalVolumeRecord? Record { get; private set; }
        public override Task<long> InsertCoalVolumeRecordAsync(CoalVolumeRecord record, CancellationToken cancellationToken = default)
        {
            Record = record;
            return Task.FromResult(1L);
        }
    }

    [Fact]
    public async Task PartnerClient_StopsAtThreeAttemptsWhenConfiguredHigher()
    {
        using var handler = new PartnerHandler("{\"code\":200,\"message\":\"success\",\"data\":[]}");
        using var http = new HttpClient(handler);
        var client = Client(http, 99);
        await Assert.ThrowsAsync<InvalidOperationException>(() => client.GetLicensePlateAsync("CAM_001"));
        Assert.Equal(3, handler.Calls);
    }

    private static PartnerLicensePlateClient Client(HttpClient http, int attempts) => new(http,
        new Monitor(new IntegrationOptions
        {
            AppSecret = "test-only",
            PartnerBaseUrl = "http://partner.local",
            PartnerRetryCount = attempts,
            PartnerFirstCallDelayMs = 0,
            PartnerRetryDelayMs = 0
        }),
        new NullIntegrationRepository(), new ConfigurationBuilder().Build(),
        NullLogger<PartnerLicensePlateClient>.Instance, new HttpContextAccessor());

    [Fact]
    public async Task PartnerClient_DoesNotTrimInvalidCameraQuery()
    {
        using var handler = new PartnerHandler("{}");
        using var http = new HttpClient(handler);
        await Assert.ThrowsAsync<ArgumentException>(() => Client(http, 1).GetLicensePlateAsync(" CAM_001"));
        Assert.Equal(0, handler.Calls);
    }

    private sealed class PartnerHandler(string body) : HttpMessageHandler
    {
        public int Calls { get; private set; }
        protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
        {
            Calls++;
            return Task.FromResult(new HttpResponseMessage(HttpStatusCode.OK)
            {
                Content = new StringContent(body, System.Text.Encoding.UTF8, "application/json")
            });
        }
    }

    [Theory]
    [InlineData("/api/v1/coalVolumes", 400)]
    [InlineData("/API/V1/COALVOLUMES/", 400)]
    [InlineData("/api/v1/vehicleDimensions", 429)]
    public async Task RateLimit_EnforcesThreeRequestsEvenWhenConfigurationIsHigher(string path, int expected)
    {
        var middleware = new PartnerRateLimitMiddleware(_ => Task.CompletedTask,
            new Monitor(new IntegrationOptions { RateLimitPerSecond = 100 }));
        var address = IPAddress.Parse($"10.123.{Random.Shared.Next(1, 254)}.{Random.Shared.Next(1, 254)}");
        for (var i = 0; i < 4; i++)
        {
            var context = new DefaultHttpContext();
            context.Request.Path = path;
            context.Connection.RemoteIpAddress = address;
            context.Response.Body = new MemoryStream();
            await middleware.InvokeAsync(context);
            Assert.Equal(i < 3 ? 200 : expected, context.Response.StatusCode);
        }
    }

    private sealed class Monitor(IntegrationOptions value) : IOptionsMonitor<IntegrationOptions>, IOptions<IntegrationOptions>
    {
        public IntegrationOptions CurrentValue => value;
        public IntegrationOptions Value => value;
        public IntegrationOptions Get(string? name) => value;
        public IDisposable? OnChange(Action<IntegrationOptions, string?> listener) => null;
    }
}
