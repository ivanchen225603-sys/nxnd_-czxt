using System.Net;
using System.Text.Json;
using Xunit;

namespace CarPointCloudWebApi.Tests;

[Collection("Integration")]
public class QueryContractRegressionTests
{
    private readonly HttpClient _client;
    private readonly TestDbHelper _db;

    public QueryContractRegressionTests(TestFixture fixture)
    {
        _client = fixture.Client;
        _db = fixture.DbHelper;
    }

    [DatabaseFact]
    public async Task VehicleDimensions_TimeWindow_ExcludesOlderRecords()
    {
        await WaitForRateLimitWindowAsync();
        _db.CleanupTestData();
        var suffix = Guid.NewGuid().ToString("N")[..8].ToUpperInvariant();
        var recentPlate = $"宁A{suffix}";
        var oldPlate = $"宁B{suffix}";
        var now = BusinessClock.Now;

        InsertVehicleDimension(recentPlate, now.AddMinutes(-5));
        InsertVehicleDimension(oldPlate, now.AddMinutes(-90));

        using var request = CreateAuthRequest("/api/v1/vehicleDimensions?timeWindow=60");
        using var response = await _client.SendAsync(request);

        Assert.Equal(HttpStatusCode.OK, response.StatusCode);
        var plates = await ReadStringPropertyAsync(response, "licensePlate");
        Assert.Contains(recentPlate, plates);
        Assert.DoesNotContain(oldPlate, plates);
    }

    [DatabaseFact]
    public async Task CoalVolumes_TimeWindow_ExcludesOlderRecords()
    {
        await WaitForRateLimitWindowAsync();
        _db.CleanupTestData();
        var suffix = Guid.NewGuid().ToString("N")[..8].ToUpperInvariant();
        var recentPlate = $"宁C{suffix}";
        var oldPlate = $"宁D{suffix}";
        var now = BusinessClock.Now;

        _db.InsertTestCoalVolumeRecord(recentPlate, "CAM001", 50m, "m3", now.AddMinutes(-5), "success", 0.9m);
        _db.InsertTestCoalVolumeRecord(oldPlate, "CAM001", 50m, "m3", now.AddMinutes(-90), "success", 0.9m);

        using var request = CreateAuthRequest("/api/v1/coalVolumes?timeWindow=60");
        using var response = await _client.SendAsync(request);

        Assert.Equal(HttpStatusCode.OK, response.StatusCode);
        var plates = await ReadStringPropertyAsync(response, "licensePlate");
        Assert.Contains(recentPlate, plates);
        Assert.DoesNotContain(oldPlate, plates);
    }

    [DatabaseFact]
    public async Task VehicleDimensions_DefaultAndOversizedLimit_ReturnAtMost100Records()
    {
        await WaitForRateLimitWindowAsync();
        var prefix = "宁A" + Random.Shared.Next(10000, 99999);
        var fixedTime = BusinessClock.Now;
        for (var index = 0; index < 101; index++)
        {
            InsertVehicleDimension($"{prefix}{index:D3}", fixedTime);
        }

        using var defaultRequest = CreateAuthRequest("/api/v1/vehicleDimensions");
        using var defaultResponse = await _client.SendAsync(defaultRequest);
        Assert.Equal(HttpStatusCode.BadRequest, defaultResponse.StatusCode);

        using var oversizedRequest = CreateAuthRequest("/api/v1/vehicleDimensions?limit=200");
        using var oversizedResponse = await _client.SendAsync(oversizedRequest);
        Assert.Equal(HttpStatusCode.OK, oversizedResponse.StatusCode);
        Assert.Equal(100, await ReadDataCountAsync(oversizedResponse));
    }

    [DatabaseFact]
    public async Task VehicleDimensions_ReturnsEmptyWiresEvenForHistoricalDetails()
    {
        await WaitForRateLimitWindowAsync();
        var plate = "宁A" + Guid.NewGuid().ToString("N")[..8].ToUpperInvariant();
        InsertVehicleDimension(
            plate,
            BusinessClock.Now,
            wires: new List<(int wireId, decimal? distToPrev)>
            {
                (3, 30m),
                (1, 10m),
                (2, 20m)
            });

        using var request = CreateAuthRequest("/api/v1/vehicleDimensions?limit=100");
        using var response = await _client.SendAsync(request);

        Assert.Equal(HttpStatusCode.OK, response.StatusCode);
        using var document = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
        var record = document.RootElement.GetProperty("data")
            .EnumerateArray()
            .Single(item => item.GetProperty("licensePlate").GetString() == plate);
        Assert.Empty(record.GetProperty("dimensions").GetProperty("wires").EnumerateArray());
    }

    private void InsertVehicleDimension(
        string licensePlate,
        DateTime getTime,
        List<(int wireId, decimal? distToPrev)>? wires = null)
    {
        _db.InsertTestVehicleDimensionRecord(
            licensePlate,
            "CAM001",
            "success",
            100m,
            800m,
            250m,
            150m,
            90m,
            20m,
            wires: wires,
            getTime: getTime);
    }

    private static async Task<int> ReadDataCountAsync(HttpResponseMessage response)
    {
        using var document = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
        return document.RootElement.GetProperty("data").GetArrayLength();
    }

    private static async Task<List<string>> ReadStringPropertyAsync(
        HttpResponseMessage response,
        string propertyName)
    {
        using var document = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
        return document.RootElement.GetProperty("data")
            .EnumerateArray()
            .Select(item => item.GetProperty(propertyName).GetString() ?? string.Empty)
            .ToList();
    }

    private static HttpRequestMessage CreateAuthRequest(string path)
    {
        var (timestamp, signature) = AuthHelper.GenerateAuthHeaders();
        var request = new HttpRequestMessage(HttpMethod.Get, path);
        request.Headers.Add("Timestamp", timestamp);
        request.Headers.Add("Signature", signature);
        return request;
    }

    private static Task WaitForRateLimitWindowAsync() => Task.Delay(1100);
}
