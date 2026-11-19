using System.Text;
using System.Text.Json;
using Xunit;

namespace CarPointCloudWebApi.Tests;

[Collection("Integration")]
public class EndToEndTests
{
    private readonly HttpClient _client;
    private readonly TestDbHelper _db;

    public EndToEndTests(TestFixture fixture)
    {
        _client = fixture.Client;
        _db = fixture.DbHelper;
    }

    [DatabaseFact]
    public async Task IngestVehicleDimensions_ThenQuery_VerifyConsistency()
    {
        var idempotencyKey = $"test-dim-e2e-{Guid.NewGuid():N}";
        var json = "{\"idempotencyKey\":\"" + idempotencyKey + "\"," +
            "\"cameraIdentifier\":\"CAM001\"," +
            "\"licensePlate\":\"宁A69497\"," +
            "\"detectStatus\":\"success\"," +
            "\"actualRibCount\":1,\"maximumWidthCm\":255.5," +
            "\"dimensions\":{\"cabToTrailerFrontDist\":100.5,\"trailerLength\":800.0,\"trailerWidth\":250.0,\"trailerBodyHeight\":200.0,\"trailerBottomToGroundHeight\":80.0,\"highLowPlateHeight\":30.0}," +
            "\"highLowPlates\":[{\"plateId\":2,\"highestToGround\":110.0,\"lowestToGround\":90.0},{\"plateId\":1,\"highestToGround\":120.0,\"lengthCm\":350.0,\"lowestToGround\":100.0}]," +
            "\"ribs\":[{\"ribId\":2,\"distToPrev\":100.0},{\"ribId\":1,\"distToPrev\":50.0}]," +
            "\"wires\":[{\"wireId\":1,\"distToPrev\":200.0}]}";

        var postResponse = await PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", json);
        Assert.Equal(200, (int)postResponse.StatusCode);

        var postContent = await postResponse.Content.ReadAsStringAsync();
        using var postDoc = JsonDocument.Parse(postContent);
        var postData = postDoc.RootElement.GetProperty("data");
        var id = postData.GetProperty("id").GetInt64();
        Assert.True(id > 0);

        await Task.Delay(1100);

        var getRequest = CreateAuthGetRequest("/api/v1/vehicleDimensions?limit=5");
        var getResponse = await _client.SendAsync(getRequest);
        Assert.Equal(200, (int)getResponse.StatusCode);

        var content = await getResponse.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(content);
        var data = doc.RootElement.GetProperty("data");

        var record = data.EnumerateArray()
            .FirstOrDefault(r => r.TryGetProperty("licensePlate", out var lp) && lp.GetString() == "宁A69497");
        Assert.NotEqual(JsonValueKind.Undefined, record.ValueKind);

        var dimensions = record.GetProperty("dimensions");
        Assert.Equal(255.5m, dimensions.GetProperty("trailerWidth").GetDecimal());

        var plates = dimensions.GetProperty("highLowPlates").EnumerateArray().ToList();
        Assert.Equal(2, plates.Count);
        Assert.Equal(1, plates[0].GetProperty("plateId").GetInt32());
        Assert.Equal(2, plates[1].GetProperty("plateId").GetInt32());
        Assert.Equal(350m, plates[0].GetProperty("highestToGround").GetDecimal());
        Assert.Equal(JsonValueKind.Null, plates[1].GetProperty("highestToGround").ValueKind);

        var ribs = dimensions.GetProperty("ribs").EnumerateArray().ToList();
        Assert.Single(ribs);
        Assert.Equal(1, ribs[0].GetProperty("ribId").GetInt32());
        Assert.Equal(50.0m, ribs[0].GetProperty("distToPrev").GetDecimal());

        var wires = dimensions.GetProperty("wires").EnumerateArray().ToList();
        Assert.Empty(wires);
    }

    [DatabaseFact]
    public async Task IncompleteCoalIngest_RemainsInternalAndIsNotPublished()
    {
        var idempotencyKey = $"test-coal-e2e-{Guid.NewGuid():N}";
        var json = "{\"idempotencyKey\":\"" + idempotencyKey + "\"," +
            "\"cameraIdentifier\":\"CAM001\"," +
            "\"licensePlate\":\"宁A77895\"," +
            "\"volume\":45.5," +
            "\"unit\":\"m³\"," +
            "\"status\":\"Success\"," +
            "\"confidence\":0.95}";

        var postResponse = await PostWithAuthAsync("/api/v1/internal/pointcloud/coalVolumes", json);
        Assert.Equal(200, (int)postResponse.StatusCode);

        var postContent = await postResponse.Content.ReadAsStringAsync();
        using var postDoc = JsonDocument.Parse(postContent);
        var postData = postDoc.RootElement.GetProperty("data");
        var id = postData.GetProperty("id").GetInt64();
        Assert.True(id > 0);

        await Task.Delay(1100);

        var getRequest = CreateAuthGetRequest("/api/v1/coalVolumes?limit=10");
        var getResponse = await _client.SendAsync(getRequest);
        Assert.Equal(200, (int)getResponse.StatusCode);

        var content = await getResponse.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(content);
        var data = doc.RootElement.GetProperty("data");

        var record = data.EnumerateArray()
            .FirstOrDefault(r => r.TryGetProperty("licensePlate", out var lp) && lp.GetString() == "宁A77895");
        Assert.Equal(JsonValueKind.Undefined, record.ValueKind);
    }

    [DatabaseFact]
    public async Task IngestFailedRecord_ThenQuery_DimensionsIsEmpty()
    {
        var idempotencyKey = $"test-dim-e2e-fail-{Guid.NewGuid():N}";
        var json = "{\"idempotencyKey\":\"" + idempotencyKey + "\"," +
            "\"cameraIdentifier\":\"CAM001\"," +
            "\"licensePlate\":\"宁A72140\"," +
            "\"detectStatus\":\"failed\"," +
            "\"dimensions\":{\"cabToTrailerFrontDist\":100.5,\"trailerLength\":800.0}}";

        var postResponse = await PostWithAuthAsync("/api/v1/internal/pointcloud/vehicleDimensions", json);
        Assert.Equal(200, (int)postResponse.StatusCode);

        await Task.Delay(1100);

        var getRequest = CreateAuthGetRequest("/api/v1/vehicleDimensions?limit=50");
        var getResponse = await _client.SendAsync(getRequest);
        Assert.Equal(200, (int)getResponse.StatusCode);

        var content = await getResponse.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(content);
        var data = doc.RootElement.GetProperty("data");

        var record = data.EnumerateArray()
            .FirstOrDefault(r => r.TryGetProperty("licensePlate", out var lp) && lp.GetString() == "宁A72140");
        Assert.NotEqual(JsonValueKind.Undefined, record.ValueKind);

        var dimensions = record.GetProperty("dimensions");
        Assert.Empty(dimensions.EnumerateObject());
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

    private HttpRequestMessage CreateAuthGetRequest(string url)
    {
        var (ts, sig) = AuthHelper.GenerateAuthHeaders();
        var req = new HttpRequestMessage(HttpMethod.Get, url);
        req.Headers.Add("Timestamp", ts);
        req.Headers.Add("Signature", sig);
        return req;
    }
}
