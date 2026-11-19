using System.Net;
using System.Text.Json;
using Xunit;

namespace CarPointCloudWebApi.Tests;

[Collection("Integration")]
public class CoalVolumesQueryTests
{
    private readonly HttpClient _client;
    private readonly TestDbHelper _db;

    public CoalVolumesQueryTests(TestFixture fixture)
    {
        _client = fixture.Client;
        _db = fixture.DbHelper;
    }

    [DatabaseFact]
    public async Task QueryByLimit_Returns200()
    {
        await Task.Delay(1100);

        _db.InsertTestCoalVolumeRecord("宁A84332", "CAM001", 50.5m, "m³", DateTime.Now, "Success", 0.95m);
        _db.InsertTestCoalVolumeRecord("宁A12790", "CAM001", 60.0m, "m³", DateTime.Now, "Success", 0.90m);

        using var req = CreateAuthRequest("/api/v1/coalVolumes?limit=10");
        using var resp = await _client.SendAsync(req);

        Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        var content = await resp.Content.ReadAsStringAsync();
        var root = JsonDocument.Parse(content).RootElement;
        Assert.True(root.TryGetProperty("data", out var dataEl), "响应应包含 data 字段");
        Assert.Equal(JsonValueKind.Array, dataEl.ValueKind);

        var data = dataEl.EnumerateArray();
        var testRecord = data.FirstOrDefault(r => r.GetProperty("licensePlate").GetString() == "宁A84332");
        Assert.True(testRecord.ValueKind == JsonValueKind.Object, "应能找到测试记录 宁A84332");
        Assert.True(testRecord.TryGetProperty("volume", out _), "应包含 volume");
        Assert.True(testRecord.TryGetProperty("unit", out _), "应包含 unit");
        Assert.False(testRecord.TryGetProperty("status", out _));
        Assert.False(testRecord.TryGetProperty("isAbnormal", out _));
        Assert.True(testRecord.TryGetProperty("enterID", out _));
        Assert.True(testRecord.TryGetProperty("captureTime", out _));
        Assert.True(testRecord.TryGetProperty("measureEndTime", out _));
        Assert.True(testRecord.TryGetProperty("measureTime", out _), "应包含 measureTime");
    }

    [DatabaseFact]
    public async Task ResponseFieldsAreCamelCase()
    {
        await Task.Delay(1100);

        _db.InsertTestCoalVolumeRecord("宁A74534", "CAM001", 55.5m, "m³", DateTime.Now, "Success", 0.95m);

        using var req = CreateAuthRequest("/api/v1/coalVolumes?limit=1");
        using var resp = await _client.SendAsync(req);

        Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        var content = await resp.Content.ReadAsStringAsync();
        Assert.Contains("\"licensePlate\"", content);
        Assert.Contains("\"cameraIdentifier\"", content);
        Assert.Contains("\"volume\"", content);
        Assert.Contains("\"unit\"", content);
        Assert.DoesNotContain("\"status\"", content);
        Assert.DoesNotContain("\"isAbnormal\"", content);
        Assert.Contains("\"enterID\"", content);
        Assert.Contains("\"measureTime\"", content);
    }

    [DatabaseFact]
    public async Task BothParams_Returns400()
    {
        await Task.Delay(1100);

        using var req = CreateAuthRequest("/api/v1/coalVolumes?limit=5&timeWindow=60");
        using var resp = await _client.SendAsync(req);

        Assert.Equal(HttpStatusCode.BadRequest, resp.StatusCode);
    }

    [DatabaseFact]
    public async Task CursorParameter_Returns400()
    {
        await Task.Delay(1100);
        using var request = CreateAuthRequest("/api/v1/coalVolumes?cursor=unsupported");
        using var response = await _client.SendAsync(request);
        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
        Assert.False(response.Headers.Contains("X-Next-Cursor"));
    }

    private HttpRequestMessage CreateAuthRequest(string url)
    {
        var (ts, sig) = AuthHelper.GenerateAuthHeaders();
        var req = new HttpRequestMessage(HttpMethod.Get, url);
        req.Headers.Add("Timestamp", ts);
        req.Headers.Add("Signature", sig);
        return req;
    }
}
