using System.Net;
using System.Text;
using System.Text.Json;
using Xunit;

namespace CarPointCloudWebApi.Tests;

[Collection("Integration")]
public class CoalVolumeIngestTests
{
    private readonly HttpClient _client;
    private readonly TestDbHelper _db;

    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase
    };

    public CoalVolumeIngestTests(TestFixture fixture)
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

    private static string BuildCoalVolumeJson(
        string idempotencyKey,
        string cameraIdentifier = "CAM001",
        decimal volume = 45.5m,
        decimal? confidence = 0.95m,
        string licensePlate = "TESTCOAL001",
        string? measureTime = "2026-01-01T12:00:00")
    {
        var body = new
        {
            idempotencyKey,
            cameraIdentifier,
            licensePlate,
            measureTime,
            volume,
            unit = "m³",
            status = "Success",
            confidence,
            // 每个测试业务测量使用独立源文件；同一请求重放仍保持完全相同。
            sourceFile = $"/data/{idempotencyKey}.pcd",
            baselineFile = "/data/baseline.pcd"
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
    public async Task IngestCoalVolume_Success_ReturnsId()
    {
        var idempotencyKey = $"test-coal-{Guid.NewGuid():N}";
        var json = BuildCoalVolumeJson(idempotencyKey);
        var response = await PostWithAuthAsync("/api/v1/internal/pointcloud/coalVolumes", json);

        Assert.Equal(HttpStatusCode.OK, response.StatusCode);
        var id = await ExtractIdAsync(response);
        Assert.True(id > 0);
    }

    [DatabaseFact]
    public async Task VolumeOutOfRange_Returns400()
    {
        var jsonNegative = BuildCoalVolumeJson($"test-coal-{Guid.NewGuid():N}", volume: -1m);
        var responseNegative = await PostWithAuthAsync("/api/v1/internal/pointcloud/coalVolumes", jsonNegative);
        Assert.Equal(HttpStatusCode.BadRequest, responseNegative.StatusCode);

        var jsonOverMax = BuildCoalVolumeJson($"test-coal-{Guid.NewGuid():N}", volume: 1001m);
        var responseOverMax = await PostWithAuthAsync("/api/v1/internal/pointcloud/coalVolumes", jsonOverMax);
        Assert.Equal(HttpStatusCode.BadRequest, responseOverMax.StatusCode);
    }

    [DatabaseFact]
    public async Task ConfidenceOutOfRange_Returns400()
    {
        var json = BuildCoalVolumeJson($"test-coal-{Guid.NewGuid():N}", confidence: 1.5m);
        var response = await PostWithAuthAsync("/api/v1/internal/pointcloud/coalVolumes", json);

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
    }

    [DatabaseFact]
    public async Task SameIdempotencyKeySamePayload_ReturnsSameId()
    {
        var key = $"test-coal-duplicate-{Guid.NewGuid():N}";
        var json = BuildCoalVolumeJson(key);
        using var first = await PostWithAuthAsync("/api/v1/internal/pointcloud/coalVolumes", json);
        using var second = await PostWithAuthAsync("/api/v1/internal/pointcloud/coalVolumes", json);

        Assert.Equal(HttpStatusCode.OK, first.StatusCode);
        Assert.Equal(HttpStatusCode.OK, second.StatusCode);
        Assert.Equal(await ExtractIdAsync(first), await ExtractIdAsync(second));
        Assert.Equal(1, _db.CountCoalVolumesByIdempotencyKey(key));
    }

    [DatabaseFact]
    public async Task SameIdempotencyKeyDifferentPayload_Returns409()
    {
        var key = $"test-coal-conflict-{Guid.NewGuid():N}";
        using var first = await PostWithAuthAsync(
            "/api/v1/internal/pointcloud/coalVolumes",
            BuildCoalVolumeJson(key, volume: 45.5m));
        using var second = await PostWithAuthAsync(
            "/api/v1/internal/pointcloud/coalVolumes",
            BuildCoalVolumeJson(key, volume: 46.5m));

        Assert.Equal(HttpStatusCode.OK, first.StatusCode);
        Assert.Equal(HttpStatusCode.Conflict, second.StatusCode);
        Assert.Equal(1, _db.CountCoalVolumesByIdempotencyKey(key));
    }

    [DatabaseFact]
    public async Task MissingBusinessTime_RetryRemainsIdempotent()
    {
        var key = $"test-coal-no-time-{Guid.NewGuid():N}";
        var json = BuildCoalVolumeJson(key, measureTime: null);
        using var first = await PostWithAuthAsync("/api/v1/internal/pointcloud/coalVolumes", json);
        using var second = await PostWithAuthAsync("/api/v1/internal/pointcloud/coalVolumes", json);

        Assert.Equal(HttpStatusCode.OK, first.StatusCode);
        Assert.Equal(HttpStatusCode.OK, second.StatusCode);
        Assert.Equal(await ExtractIdAsync(first), await ExtractIdAsync(second));
    }
}
