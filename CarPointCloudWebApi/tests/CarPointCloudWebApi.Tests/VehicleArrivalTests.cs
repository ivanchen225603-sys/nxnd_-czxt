using System.Net;
using System.Text;
using System.Text.Json;
using Dm;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using Xunit;

namespace CarPointCloudWebApi.Tests;

[Collection("Integration")]
public class VehicleArrivalTests
{
    private static int _testSourceSequence;
    private readonly HttpClient _client;
    private readonly string _connectionString;

    public VehicleArrivalTests(TestFixture fixture)
    {
        _client = fixture.Client;
        var configuration = fixture.Factory.Services.GetRequiredService<IConfiguration>();
        _connectionString = configuration.GetConnectionString("CarPointCloudDb")
            ?? throw new InvalidOperationException("Missing integration-test database connection string.");
    }

    [DatabaseFact]
    public async Task ValidArrival_Returns200_WithCollisionSafeSessionId()
    {
        var json = "{\"device_id\":\"TESTDEV001\",\"cameraIdentifier\":\"CAM001\",\"timestamp\":\"2026-01-01 12:00:00\",\"event_type\":\"ARRIVAL\"}";
        var response = await PostJsonAsync("/api/v1/radar/vehicle-arrival", json);

        Assert.Equal(HttpStatusCode.OK, response.StatusCode);

        using var doc = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
        Assert.True(doc.RootElement.TryGetProperty("data", out var data), doc.RootElement.ToString());
        var sessionId = data.GetProperty("session_id").GetString();
        Assert.NotNull(sessionId);
        Assert.Matches(@"^sess_\d{17}_[0-9a-f]{12}$", sessionId);
        Assert.Equal(sessionId, data.GetProperty("workflow_id").GetString());
        Assert.Matches("^粤A[A-HJ-NP-Z0-9]{5}$", data.GetProperty("license_plate").GetString());
        Assert.Equal("accepted", data.GetProperty("status").GetString());
        Assert.False(data.GetProperty("duplicate").GetBoolean());
    }

    [DatabaseFact]
    public async Task SameEventId_ConcurrentAuthorizedSources_CreateOneRowAndReturnOneSession()
    {
        var eventId = $"TEST-EVT-{Guid.NewGuid():N}";
        var deviceId = $"TESTDEV-{Guid.NewGuid():N}"[..24];
        var json = JsonSerializer.Serialize(new
        {
            device_id = deviceId,
            cameraIdentifier = "CAM001",
            timestamp = "2026-01-01 12:00:00",
            event_type = "ARRIVAL"
        });

        var responses = await Task.WhenAll(Enumerable.Range(0, 10)
            .Select(_ => PostJsonAsync("/api/v1/radar/vehicle-arrival", json, eventId)));

        Assert.All(responses, response => Assert.Equal(HttpStatusCode.OK, response.StatusCode));

        var sessionIds = new List<string>();
        foreach (var response in responses)
        {
            using var doc = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
            Assert.True(doc.RootElement.TryGetProperty("data", out var data), doc.RootElement.ToString());
            sessionIds.Add(data.GetProperty("session_id").GetString()!);
        }

        Assert.Single(sessionIds.Distinct(StringComparer.Ordinal));
        Assert.Equal(1, CountArrivalRows(eventId));
    }

    [DatabaseFact]
    public async Task SameEventId_WithDifferentPayload_Returns409()
    {
        var eventId = $"TEST-CONFLICT-{Guid.NewGuid():N}";
        var first = "{\"device_id\":\"TESTDEV-A\",\"cameraIdentifier\":\"CAM001\",\"timestamp\":\"2026-01-01 12:00:00\",\"event_type\":\"ARRIVAL\"}";
        var second = "{\"device_id\":\"TESTDEV-B\",\"cameraIdentifier\":\"CAM001\",\"timestamp\":\"2026-01-01 12:00:00\",\"event_type\":\"ARRIVAL\"}";

        var firstResponse = await PostJsonAsync("/api/v1/radar/vehicle-arrival", first, eventId);
        var secondResponse = await PostJsonAsync("/api/v1/radar/vehicle-arrival", second, eventId);

        Assert.Equal(HttpStatusCode.OK, firstResponse.StatusCode);
        Assert.Equal(HttpStatusCode.Conflict, secondResponse.StatusCode);
        Assert.Equal(1, CountArrivalRows(eventId));
    }

    [DatabaseFact]
    public async Task EventIdAndHeaderConflict_Returns400()
    {
        var json = "{\"device_id\":\"TESTDEV001\",\"cameraIdentifier\":\"CAM001\",\"timestamp\":\"2026-01-01 12:00:00\",\"event_type\":\"ARRIVAL\",\"event_id\":\"body-id\"}";
        var response = await PostJsonAsync("/api/v1/radar/vehicle-arrival", json, "header-id");

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
    }

    [DatabaseFact]
    public async Task OversizedDeviceId_Returns400()
    {
        var json = JsonSerializer.Serialize(new
        {
            device_id = new string('D', 65),
            cameraIdentifier = "CAM001",
            timestamp = "2026-01-01 12:00:00",
            event_type = "ARRIVAL"
        });

        var response = await PostJsonAsync("/api/v1/radar/vehicle-arrival", json);

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
    }

    [DatabaseFact]
    public async Task EmptyBody_Returns400()
    {
        var response = await PostJsonAsync("/api/v1/radar/vehicle-arrival", "");

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
    }

    [DatabaseFact]
    public async Task InvalidEventType_Returns400()
    {
        var json = "{\"device_id\":\"TESTDEV001\",\"cameraIdentifier\":\"CAM001\",\"timestamp\":\"2026-01-01 12:00:00\",\"event_type\":\"DEPARTURE\"}";
        var response = await PostJsonAsync("/api/v1/radar/vehicle-arrival", json);

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
    }

    [DatabaseFact]
    public async Task InvalidTimestampFormat_Returns400()
    {
        var json = "{\"device_id\":\"TESTDEV001\",\"cameraIdentifier\":\"CAM001\",\"timestamp\":\"2026/01/01 12:00:00\",\"event_type\":\"ARRIVAL\"}";
        var response = await PostJsonAsync("/api/v1/radar/vehicle-arrival", json);

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
    }

    [DatabaseFact]
    public async Task MissingDeviceId_Returns400()
    {
        var json = "{\"device_id\":\"\",\"cameraIdentifier\":\"CAM001\",\"timestamp\":\"2026-01-01 12:00:00\",\"event_type\":\"ARRIVAL\"}";
        var response = await PostJsonAsync("/api/v1/radar/vehicle-arrival", json);

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
    }

    private async Task<HttpResponseMessage> PostJsonAsync(string url, string jsonBody, string? eventId = null)
    {
        using var request = new HttpRequestMessage(HttpMethod.Post, url)
        {
            Content = new StringContent(jsonBody, Encoding.UTF8, "application/json")
        };
        // 非限流用例隔离来源桶；并发唯一性验证使用独立来源，不能要求同源突发绕过 429。
        // 来源由仅存在于 TestApiFactory 的本机代理传递；产品限流配置保持不变。
        var source = Interlocked.Increment(ref _testSourceSequence);
        request.Headers.TryAddWithoutValidation("X-Forwarded-For", $"10.244.{source / 250 % 250}.{source % 250 + 1}");
        if (eventId != null)
        {
            request.Headers.TryAddWithoutValidation("Idempotency-Key", eventId);
        }

        return await _client.SendAsync(request);
    }

    private int CountArrivalRows(string eventId)
    {
        using var connection = new DmConnection(_connectionString);
        connection.Open();
        using var command = new DmCommand(
            "SELECT COUNT(*) FROM VehicleArrival WHERE EventId = ?",
            connection);
        command.Parameters.Add("@EventId", DmDbType.VarChar, 128).Value = eventId;
        return Convert.ToInt32(command.ExecuteScalar());
    }

    private static VehicleArrivalRequest CreateRequest(string cameraIdentifier)
    {
        return new VehicleArrivalRequest
        {
            DeviceId = "TEST-UNIT",
            CameraIdentifier = cameraIdentifier,
            Timestamp = "2026-01-01 12:00:00",
            EventType = "ARRIVAL",
            EventId = $"TEST-UNIT-{Guid.NewGuid():N}"
        };
    }


    private static int GetStatusCode(ActionResult<ApiResponse<VehicleArrivalResponse>> result)
    {
        return Assert.IsAssignableFrom<ObjectResult>(result.Result).StatusCode
            ?? throw new InvalidOperationException("Controller result did not specify an HTTP status.");
    }



}
