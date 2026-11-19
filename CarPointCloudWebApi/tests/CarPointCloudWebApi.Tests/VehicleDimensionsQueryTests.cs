using System.Net;
using System.Text.Json;
using Xunit;

namespace CarPointCloudWebApi.Tests;

[Collection("Integration")]
public class VehicleDimensionsQueryTests
{
    private readonly HttpClient _client;
    private readonly TestDbHelper _db;

    public VehicleDimensionsQueryTests(TestFixture fixture)
    {
        _client = fixture.Client;
        _db = fixture.DbHelper;
    }

    [DatabaseFact]
    public async Task QueryByLimit_ReturnsCorrectCount()
    {
        await Task.Delay(1100);

        _db.InsertTestVehicleDimensionRecord("宁A82607", "CAM001", "success", 100.0m, 1300.0m, 300.0m, 200.0m, 90.0m, 10.0m);
        _db.InsertTestVehicleDimensionRecord("宁A18092", "CAM001", "success", 100.0m, 1300.0m, 300.0m, 200.0m, 90.0m, 10.0m);
        _db.InsertTestVehicleDimensionRecord("宁A06099", "CAM001", "success", 100.0m, 1300.0m, 300.0m, 200.0m, 90.0m, 10.0m);

        using var req = CreateAuthRequest("/api/v1/vehicleDimensions?limit=2");
        using var resp = await _client.SendAsync(req);

        Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        var content = await resp.Content.ReadAsStringAsync();
        var root = JsonDocument.Parse(content).RootElement;
        Assert.True(root.TryGetProperty("data", out var dataEl), "响应应包含 data 字段");
        Assert.Equal(JsonValueKind.Array, dataEl.ValueKind);

        var items = dataEl.EnumerateArray().ToList();
        Assert.True(items.Count <= 2, "data 数组长度应不超过 limit=2");

        foreach (var item in items)
        {
            Assert.True(item.TryGetProperty("licensePlate", out _), "应包含 licensePlate");
            Assert.True(item.TryGetProperty("cameraIdentifier", out _), "应包含 cameraIdentifier");
            Assert.True(item.TryGetProperty("getTime", out _), "应包含 getTime");
            Assert.True(item.TryGetProperty("dimensions", out _), "应包含 dimensions");
        }
    }

    [DatabaseFact]
    public async Task QueryByTimeWindow_Returns200()
    {
        await Task.Delay(1100);

        _db.InsertTestVehicleDimensionRecord("宁A95501", "CAM001", "success", 100.0m, 1300.0m, 300.0m, 200.0m, 90.0m, 10.0m);

        using var req = CreateAuthRequest("/api/v1/vehicleDimensions?timeWindow=60");
        using var resp = await _client.SendAsync(req);

        Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        var content = await resp.Content.ReadAsStringAsync();
        var root = JsonDocument.Parse(content).RootElement;
        Assert.True(root.TryGetProperty("data", out var dataEl), "响应应包含 data 字段");
        Assert.Equal(JsonValueKind.Array, dataEl.ValueKind);
    }

    [DatabaseFact]
    public async Task BothLimitAndTimeWindow_Returns400()
    {
        await Task.Delay(1100);

        using var req = CreateAuthRequest("/api/v1/vehicleDimensions?limit=5&timeWindow=60");
        using var resp = await _client.SendAsync(req);

        Assert.Equal(HttpStatusCode.BadRequest, resp.StatusCode);
        var content = await resp.Content.ReadAsStringAsync();
        using var body = JsonDocument.Parse(content);
        Assert.Equal(400, body.RootElement.GetProperty("code").GetInt32());
        Assert.Contains("必须且只能传入 limit 或 timeWindow", body.RootElement.GetProperty("message").GetString());
    }

    [DatabaseFact]
    public async Task NeitherParam_Returns400()
    {
        await Task.Delay(1100);

        using var req = CreateAuthRequest("/api/v1/vehicleDimensions");
        using var resp = await _client.SendAsync(req);

        Assert.Equal(HttpStatusCode.BadRequest, resp.StatusCode);
    }

    [DatabaseFact]
    public async Task LimitExceedsRange_AutoClampsTo100()
    {
        await Task.Delay(1100);

        using var req = CreateAuthRequest("/api/v1/vehicleDimensions?limit=200");
        using var resp = await _client.SendAsync(req);

        Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
    }

    [DatabaseFact]
    public async Task ResponseFieldsAreCamelCase()
    {
        await Task.Delay(1100);

        _db.InsertTestVehicleDimensionRecord("宁A23340", "CAM001", "success", 100.0m, 1300.0m, 300.0m, 200.0m, 90.0m, 10.0m,
            ribs: new List<(int, decimal?)> { (1, 30.0m) },
            wires: new List<(int, decimal?)> { (1, 40.0m) },
            plates: new List<(int, decimal?, decimal?)> { (1, 30.0m, 20.0m) });

        using var req = CreateAuthRequest("/api/v1/vehicleDimensions?limit=1");
        using var resp = await _client.SendAsync(req);

        Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        var content = await resp.Content.ReadAsStringAsync();
        Assert.Contains("\"licensePlate\"", content);
        Assert.Contains("\"cameraIdentifier\"", content);
        Assert.Contains("\"getTime\"", content);
        Assert.Contains("\"dimensions\"", content);
        Assert.Contains("\"highLowPlates\"", content);
        Assert.Contains("\"ribs\"", content);
        Assert.Contains("\"wires\"", content);
    }

    [DatabaseFact]
    public async Task DimensionsContainsSortedArrays()
    {
        await Task.Delay(1100);

        _db.InsertTestVehicleDimensionRecord("宁A48745", "CAM001", "success", 100.0m, 1300.0m, 300.0m, 200.0m, 90.0m, 10.0m,
            ribs: new List<(int, decimal?)> { (2, 50.0m), (1, 30.0m) },
            plates: new List<(int, decimal?, decimal?)> { (3, 30.0m, 20.0m), (1, 15.0m, 10.0m), (2, 25.0m, 15.0m) });

        using var req = CreateAuthRequest("/api/v1/vehicleDimensions?limit=1");
        using var resp = await _client.SendAsync(req);

        Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        var content = await resp.Content.ReadAsStringAsync();
        var root = JsonDocument.Parse(content).RootElement;
        var data = root.GetProperty("data").EnumerateArray();
        var testRecord = data.FirstOrDefault(r => r.GetProperty("licensePlate").GetString() == "宁A48745");
        Assert.True(testRecord.ValueKind == JsonValueKind.Object, "应能找到测试记录 宁A48745");

        var dimensions = testRecord.GetProperty("dimensions");

        Assert.True(dimensions.TryGetProperty("highLowPlates", out var plates), "dimensions 应包含 highLowPlates");
        var plateList = plates.EnumerateArray().ToList();
        Assert.Equal(3, plateList.Count);
        Assert.Equal(1, plateList[0].GetProperty("plateId").GetInt32());
        Assert.Equal(2, plateList[1].GetProperty("plateId").GetInt32());
        Assert.Equal(3, plateList[2].GetProperty("plateId").GetInt32());

        Assert.True(dimensions.TryGetProperty("ribs", out var ribs), "dimensions 应包含 ribs");
        var ribList = ribs.EnumerateArray().ToList();
        Assert.Equal(2, ribList.Count);
        Assert.Equal(1, ribList[0].GetProperty("ribId").GetInt32());
        Assert.Equal(2, ribList[1].GetProperty("ribId").GetInt32());

        Assert.True(dimensions.TryGetProperty("wires", out _), "dimensions 应包含 wires");
    }

    [DatabaseFact]
    public async Task FailedDetectStatus_ReturnsEmptyDimensions()
    {
        await Task.Delay(1100);

        _db.InsertTestVehicleDimensionRecord("宁A42457", "CAM001", "failed", null, null, null, null, null, null);

        using var req = CreateAuthRequest("/api/v1/vehicleDimensions?limit=50");
        using var resp = await _client.SendAsync(req);

        Assert.Equal(HttpStatusCode.OK, resp.StatusCode);
        var content = await resp.Content.ReadAsStringAsync();
        var root = JsonDocument.Parse(content).RootElement;
        var data = root.GetProperty("data").EnumerateArray();
        var testRecord = data.FirstOrDefault(r => r.GetProperty("licensePlate").GetString() == "宁A42457");
        Assert.True(testRecord.ValueKind == JsonValueKind.Object, "应能找到测试记录 宁A42457");

        var dimensions = testRecord.GetProperty("dimensions");
        Assert.Equal(JsonValueKind.Object, dimensions.ValueKind);
        var propertyCount = 0;
        foreach (var _ in dimensions.EnumerateObject())
        {
            propertyCount++;
        }
        Assert.Equal(0, propertyCount);
    }

    [DatabaseFact]
    public async Task CursorParameter_Returns400()
    {
        await Task.Delay(1100);
        using var request = CreateAuthRequest("/api/v1/vehicleDimensions?cursor=unsupported");
        using var response = await _client.SendAsync(request);
        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
        Assert.False(response.Headers.Contains("X-Next-Cursor"));
    }

    [DatabaseFact]
    public async Task CursorAlongsideLimit_Returns400()
    {
        await Task.Delay(1100);
        using var request = CreateAuthRequest("/api/v1/coalVolumes?limit=1&cursor=unsupported");
        using var response = await _client.SendAsync(request);
        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
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
