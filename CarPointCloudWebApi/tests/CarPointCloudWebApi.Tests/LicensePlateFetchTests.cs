using System.Globalization;
using System.Net;
using System.Text.Json;
using Dm;
using Xunit;

namespace CarPointCloudWebApi.Tests;

[Collection("Integration")]
public class LicensePlateFetchTests
{
    private readonly HttpClient _client;
    private readonly MockPartnerServer _mockServer;

    public LicensePlateFetchTests(TestFixture fixture)
    {
        _client = fixture.Client;
        _mockServer = fixture.MockServer;
    }

    [DatabaseFact]
    public async Task FetchLicensePlate_Success_ReturnsArray()
    {
        await Task.Delay(1100);
        _mockServer.ResetCapture();
        _mockServer.Mode = "NonEmptyArray";

        var request = CreateAuthPostRequest("/api/v1/licensePlateRecords/fetch?cameraIdentifier=TEST_CAM001");
        using var response = await _client.SendAsync(request);

        Assert.Equal(HttpStatusCode.OK, response.StatusCode);

        var body = await response.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(body);
        var data = doc.RootElement.GetProperty("data");

        Assert.Equal(JsonValueKind.Array, data.ValueKind);
        Assert.True(data.GetArrayLength() >= 1);

        foreach (var element in data.EnumerateArray())
        {
            Assert.True(element.TryGetProperty("cameraId", out var cameraIdProp));
            Assert.True(element.TryGetProperty("licensePlate", out _));
            Assert.True(element.TryGetProperty("captureTime", out var captureTimeProp));

            Assert.Equal("TEST_CAM001", cameraIdProp.GetString());
            Assert.Equal(JsonValueKind.String, captureTimeProp.ValueKind);
        }

        Assert.Equal(1, _mockServer.RequestCount);
        Assert.Equal("GET", _mockServer.LastRequest?.Method);
        Assert.Equal("/api/v1/licensePlateRecords", _mockServer.LastRequest?.Path);
        Assert.Contains("cameraIdentifier=TEST_CAM001", _mockServer.LastRequest?.Query);
        Assert.False(string.IsNullOrWhiteSpace(_mockServer.LastRequest?.Timestamp));
        Assert.False(string.IsNullOrWhiteSpace(_mockServer.LastRequest?.Signature));
    }

    [DatabaseFact]
    public async Task EmptyCameraIdentifier_Returns400()
    {
        await Task.Delay(1100);

        var request = CreateAuthPostRequest("/api/v1/licensePlateRecords/fetch?cameraIdentifier=");
        using var response = await _client.SendAsync(request);

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
    }

    [DatabaseFact]
    public async Task InvalidCameraIdentifier_Returns400()
    {
        await Task.Delay(1100);

        var cameraIdentifier = Uri.EscapeDataString("测试通道");
        var request = CreateAuthPostRequest($"/api/v1/licensePlateRecords/fetch?cameraIdentifier={cameraIdentifier}");
        using var response = await _client.SendAsync(request);

        Assert.Equal(HttpStatusCode.BadRequest, response.StatusCode);
    }

    [DatabaseFact]
    [Trait("Category", "Slow")]
    public async Task PartnerReturnsEmptyArray_RetryThen502()
    {
        await Task.Delay(1100);
        _mockServer.ResetCapture();
        _mockServer.Mode = "EmptyArray";
        try
        {
            var interactionId = "test-empty-plate-" + Guid.NewGuid().ToString("N");
            var request = CreateAuthPostRequest("/api/v1/licensePlateRecords/fetch?cameraIdentifier=TEST_CAM001");
            request.Headers.Add("X-Interaction-Id", interactionId);
            using var response = await _client.SendAsync(request);

            Assert.Equal(HttpStatusCode.BadGateway, response.StatusCode);
            Assert.Equal(3, _mockServer.RequestCount);

            using var connection = new DmConnection(TestConfiguration.GetDatabaseConnectionString());
            connection.Open();
            using var command = new DmCommand(
                "SELECT CameraId, IsSuccess FROM LicensePlateRecord WHERE InteractionId = ?", connection);
            command.Parameters.Add("@InteractionId", DmDbType.VarChar, 64).Value = interactionId;
            using var reader = command.ExecuteReader();
            Assert.True(reader.Read());
            Assert.Equal(string.Empty, reader.GetString(0));
            Assert.False(Convert.ToBoolean(reader.GetValue(1)));
            Assert.False(reader.Read());
        }
        finally
        {
            _mockServer.Mode = "NonEmptyArray";
        }
    }

    [DatabaseFact]
    [Trait("Category", "Slow")]
    public async Task PartnerHttpError_ReturnsGeneric502WithoutLeakingBody()
    {
        await Task.Delay(1100);
        _mockServer.ResetCapture();
        _mockServer.Mode = "Http500";
        try
        {
            var request = CreateAuthPostRequest("/api/v1/licensePlateRecords/fetch?cameraIdentifier=TEST_CAM001");
            using var response = await _client.SendAsync(request);

            Assert.Equal(HttpStatusCode.BadGateway, response.StatusCode);
            Assert.Equal(3, _mockServer.RequestCount);
            var body = await response.Content.ReadAsStringAsync();
            Assert.DoesNotContain("partner-sensitive-debug-data", body, StringComparison.Ordinal);
            Assert.Contains("乙方车牌服务暂时不可用", body, StringComparison.Ordinal);
        }
        finally
        {
            _mockServer.Mode = "NonEmptyArray";
        }
    }

    [DatabaseFact]
    public async Task ResponseDataElementsHaveCorrectFields()
    {
        await Task.Delay(1100);
        _mockServer.Mode = "NonEmptyArray";

        var request = CreateAuthPostRequest("/api/v1/licensePlateRecords/fetch?cameraIdentifier=TEST_CAM002");
        using var response = await _client.SendAsync(request);

        Assert.Equal(HttpStatusCode.OK, response.StatusCode);

        var body = await response.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(body);
        var data = doc.RootElement.GetProperty("data");

        Assert.Equal(JsonValueKind.Array, data.ValueKind);
        Assert.True(data.GetArrayLength() >= 1);

        foreach (var element in data.EnumerateArray())
        {
            Assert.True(element.TryGetProperty("cameraId", out var cameraIdProp));
            Assert.True(element.TryGetProperty("licensePlate", out var licensePlateProp));
            Assert.True(element.TryGetProperty("captureTime", out var captureTimeProp));

            Assert.Equal("TEST_CAM002", cameraIdProp.GetString());

            Assert.Equal(JsonValueKind.String, licensePlateProp.ValueKind);
            var licensePlate = licensePlateProp.GetString();
            Assert.False(string.IsNullOrEmpty(licensePlate));

            Assert.Equal(JsonValueKind.String, captureTimeProp.ValueKind);
            var captureTime = captureTimeProp.GetString();
            Assert.True(
                DateTime.TryParseExact(captureTime, "yyyy-MM-dd HH:mm:ss",
                    CultureInfo.InvariantCulture, DateTimeStyles.None, out _),
                $"captureTime '{captureTime}' 不符合 yyyy-MM-dd HH:mm:ss 格式");
        }
    }

    private HttpRequestMessage CreateAuthPostRequest(string url)
    {
        var (ts, sig) = AuthHelper.GenerateAuthHeaders();
        var req = new HttpRequestMessage(HttpMethod.Post, url);
        req.Headers.Add("Timestamp", ts);
        req.Headers.Add("Signature", sig);
        return req;
    }
}
