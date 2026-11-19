using System.Net;
using System.Text;
using System.Text.Json;
using Xunit;

namespace CarPointCloudWebApi.Tests;

[Collection("Integration")]
public class VehicleDimensionsIngestTests
{
    private readonly HttpClient _client;
    private readonly TestDbHelper _db;

    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase
    };

    public VehicleDimensionsIngestTests(TestFixture fixture)
    {
        _client = fixture.Client;
        _db = fixture.DbHelper;
    }

    private async Task<HttpResponseMessage> PostWithAuthAsync(string url, string jsonBody)
    {
        var (ts, sig) = AuthHelper.GenerateAuthHeaders();
        var req = new HttpRequestMessage(HttpMethod.Post, url);
        req.Headers.Add("Timestamp", ts);
        req.Headers.Add("Signature", sig);
        req.Content = new StringContent(jsonBody, Encoding.UTF8, "application/json");
        return await _client.SendAsync(req);
    }

    private static string BuildDimensionsJson(
        string idempotencyKey,
        string cameraIdentifier = "CAM001",
        decimal? trailerLength = 800.0m,
        object? ribs = null,
        object? plates = null,
        string licensePlate = "TESTINGEST001",
        string? getTime = "2026-01-01T12:00:00")
    {
        var body = new
        {
            idempotencyKey,
            cameraIdentifier,
            licensePlate,
            getTime,
            detectStatus = "success",
            dimensions = new
            {
                cabToTrailerFrontDist = 100.5m,
                trailerLength,
                trailerWidth = 250.0m,
                trailerBodyHeight = 150.0m,
                trailerBottomToGroundHeight = 90.0m,
                highLowPlateHeight = 20.0m
            },
            highLowPlates = plates ?? new object[]
            {
                new { plateId = 1, highestToGround = 120.0m, lowestToGround = 100.0m },
                new { plateId = 2, highestToGround = 110.0m, lowestToGround = 90.0m }
            },
            ribs = ribs ?? new object[]
            {
                new { ribId = 1, distToPrev = 50.0m },
                new { ribId = 2, distToPrev = 100.0m }
            },
            wires = new object[]
            {
                new { wireId = 1, distToPrev = 200.0m }
            },
            sourceFile = "/data/test.pcd",
            resultJsonPath = "/results/test.json",
            resultImagePath = "/results/test.png"
        };
        return JsonSerializer.Serialize(body, JsonOptions);
    }

    private static async Task<long> ExtractIdAsync(HttpResponseMessage response)
    {
        var content = await response.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(content);
        return doc.RootElement.GetProperty("data").GetProperty("id").GetInt64();
    }

    [DatabaseFact]
    public async Task IngestVehicleDimensions_Success_ReturnsId()
    {
        var idempotencyKey = $"test-ingest-{Guid.NewGuid():N}";
        var json = BuildDimensionsJson(idempotencyKey);
        var response = await PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", json);

        Assert.Equal(HttpStatusCode.OK, response.StatusCode);
        var id = await ExtractIdAsync(response);
        Assert.True(id > 0);
    }

    [DatabaseFact]
    public async Task IdempotentKeyDuplicate_ReturnsSameId()
    {
        var json = BuildDimensionsJson("test-ingest-dup-001");

        var response1 = await PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", json);
        Assert.Equal(HttpStatusCode.OK, response1.StatusCode);
        var id1 = await ExtractIdAsync(response1);

        var response2 = await PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", json);
        Assert.Equal(HttpStatusCode.OK, response2.StatusCode);
        var id2 = await ExtractIdAsync(response2);

        Assert.Equal(id1, id2);
    }

    [DatabaseFact]
    public async Task HeaderAndBodyIdempotencyKeysDiffer_Returns400()
    {
        var bodyKey = $"test-ingest-body-{Guid.NewGuid():N}";
        var json = BuildDimensionsJson(bodyKey);
        var (timestamp, signature) = AuthHelper.GenerateAuthHeaders();
        using var request = new HttpRequestMessage(HttpMethod.Post, "/api/v1/internal/pointcloud/vehicleDimensions");
        request.Headers.Add("Timestamp", timestamp);
        request.Headers.Add("Signature", signature);
        request.Headers.Add("Idempotency-Key", $"test-ingest-header-{Guid.NewGuid():N}");
        request.Content = new StringContent(json, Encoding.UTF8, "application/json");

        using var response = await _client.SendAsync(request);

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
        var body = await response.Content.ReadAsStringAsync();
        Assert.Contains("不一致", body, StringComparison.Ordinal);
    }

    [DatabaseFact]
    public async Task SameIdempotencyKeyDifferentPayload_Returns409()
    {
        var key = $"test-ingest-conflict-{Guid.NewGuid():N}";
        using var first = await PostWithAuthAsync(
            "/api/v1/internal/pointcloud/vehicleDimensions",
            BuildDimensionsJson(key, trailerLength: 800m));
        using var second = await PostWithAuthAsync(
            "/api/v1/internal/pointcloud/vehicleDimensions",
            BuildDimensionsJson(key, trailerLength: 801m));

        Assert.Equal(HttpStatusCode.OK, first.StatusCode);
        Assert.Equal(HttpStatusCode.Conflict, second.StatusCode);
        Assert.Equal(1, _db.CountVehicleDimensionsByIdempotencyKey(key));
    }

    [DatabaseFact]
    public async Task MissingBusinessTime_RetryReturnsSameId()
    {
        var key = $"test-ingest-no-time-{Guid.NewGuid():N}";
        var json = BuildDimensionsJson(key, getTime: null);

        using var first = await PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", json);
        using var second = await PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", json);

        Assert.Equal(HttpStatusCode.OK, first.StatusCode);
        Assert.Equal(HttpStatusCode.OK, second.StatusCode);
        Assert.Equal(await ExtractIdAsync(first), await ExtractIdAsync(second));
        Assert.Equal(1, _db.CountVehicleDimensionsByIdempotencyKey(key));
    }

    [DatabaseFact]
    public async Task ConcurrentSamePayload_CreatesOneRecord()
    {
        var key = $"test-ingest-concurrent-{Guid.NewGuid():N}";
        var json = BuildDimensionsJson(key);
        var responses = await Task.WhenAll(Enumerable.Range(0, 100)
            .Select(_ => PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", json)));
        try
        {
            Assert.All(responses, response => Assert.Equal(HttpStatusCode.OK, response.StatusCode));
            var ids = await Task.WhenAll(responses.Select(ExtractIdAsync));
            Assert.Single(ids.Distinct());
            Assert.Equal(1, _db.CountVehicleDimensionsByIdempotencyKey(key));
        }
        finally
        {
            foreach (var response in responses)
            {
                response.Dispose();
            }
        }
    }

    [DatabaseFact]
    public async Task EmptyCameraIdentifier_Returns400()
    {
        var json = BuildDimensionsJson($"test-ingest-{Guid.NewGuid():N}", cameraIdentifier: "");
        var response = await PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", json);

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
    }

    [DatabaseFact]
    public async Task TrailerLengthOutOfRange_Returns400()
    {
        var jsonNegative = BuildDimensionsJson($"test-ingest-{Guid.NewGuid():N}", trailerLength: -1m);
        var responseNegative = await PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", jsonNegative);
        Assert.Equal(HttpStatusCode.BadRequest, responseNegative.StatusCode);

        var jsonOverMax = BuildDimensionsJson($"test-ingest-{Guid.NewGuid():N}", trailerLength: 3001m);
        var responseOverMax = await PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", jsonOverMax);
        Assert.Equal(HttpStatusCode.BadRequest, responseOverMax.StatusCode);
    }

    [DatabaseFact]
    public async Task DuplicateRibIds_Returns400()
    {
        var duplicateRibs = new object[]
        {
            new { ribId = 1, distToPrev = 50.0m },
            new { ribId = 1, distToPrev = 100.0m }
        };
        var json = BuildDimensionsJson($"test-ingest-{Guid.NewGuid():N}", ribs: duplicateRibs);
        var response = await PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", json);

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
    }

    [DatabaseFact]
    public async Task DuplicatePlateIds_Returns400()
    {
        var duplicatePlates = new object[]
        {
            new { plateId = 1, highestToGround = 120.0m, lowestToGround = 100.0m },
            new { plateId = 1, highestToGround = 110.0m, lowestToGround = 90.0m }
        };
        var json = BuildDimensionsJson($"test-ingest-{Guid.NewGuid():N}", plates: duplicatePlates);
        var response = await PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", json);

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
    }
}
