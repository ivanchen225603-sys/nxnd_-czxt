using System.Net;
using System.Text;
using System.Text.Json;
using Xunit;

namespace CarPointCloudWebApi.Tests;

[Collection("Integration")]
public class ProtectedRouteAuthMatrixTests
{
    private readonly HttpClient _client;

    public ProtectedRouteAuthMatrixTests(TestFixture fixture)
    {
        _client = fixture.Client;
    }

    [DatabaseTheory]
    [InlineData("GET", "/api/v1/vehicleDimensions")]
    [InlineData("GET", "/api/v1/vehicleWorkflows/sess_test/licensePlate")]
    [InlineData("GET", "/api/v1/coalVolumes")]
    [InlineData("POST", "/api/v1/licensePlateRecords/fetch?cameraIdentifier=CAM001")]
    [InlineData("POST", "/api/v1/internal/pointcloud/vehicleDimensions")]
    [InlineData("POST", "/api/v1/internal/pointcloud/coalVolumes")]
    public async Task MissingAuthenticationHeaders_OnEveryProtectedRoute_Returns401(
        string method,
        string path)
    {
        using var request = new HttpRequestMessage(new HttpMethod(method), path);
        if (string.Equals(method, "POST", StringComparison.Ordinal))
        {
            request.Content = new StringContent("{}", Encoding.UTF8, "application/json");
        }

        using var response = await _client.SendAsync(request);

        Assert.Equal(HttpStatusCode.Unauthorized, response.StatusCode);
        using var document = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
        Assert.Equal(401, document.RootElement.GetProperty("code").GetInt32());
    }
}
