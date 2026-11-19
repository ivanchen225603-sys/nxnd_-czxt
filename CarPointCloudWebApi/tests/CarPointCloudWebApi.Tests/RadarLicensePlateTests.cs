using System.Text;
using System.Text.Json;
using Xunit;

namespace CarPointCloudWebApi.Tests;

[Collection("Integration")]
public class RadarLicensePlateTests
{
    private readonly HttpClient _client;

    public RadarLicensePlateTests(TestFixture fixture)
    {
        _client = fixture.Client;
    }

    [DatabaseFact]
    public async Task ValidCameraIdentifier_Returns200()
    {
        var response = await _client.GetAsync("/api/v1/radar/licensePlate?cameraIdentifier=CAM001");

        Assert.Equal(200, (int)response.StatusCode);

        var content = await response.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(content);
        var root = doc.RootElement;

        Assert.True(root.TryGetProperty("data", out var data));
        Assert.True(data.TryGetProperty("cameraId", out var cameraId));
        Assert.True(data.TryGetProperty("licensePlate", out _));
        Assert.True(data.TryGetProperty("captureTime", out _));
        Assert.Equal("CAM001", cameraId.GetString());
    }

    [DatabaseFact]
    public async Task InvalidCameraIdentifier_Returns400()
    {
        var cameraIdentifier = Uri.EscapeDataString("测试通道");
        var response = await _client.GetAsync($"/api/v1/radar/licensePlate?cameraIdentifier={cameraIdentifier}");

        Assert.Equal(400, (int)response.StatusCode);
    }

    [DatabaseFact]
    public async Task CameraIdentifierTooLong_Returns400()
    {
        var cameraIdentifier = new string('A', 33);
        var response = await _client.GetAsync($"/api/v1/radar/licensePlate?cameraIdentifier={cameraIdentifier}");

        Assert.Equal(400, (int)response.StatusCode);
    }
}
