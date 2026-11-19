using Xunit;

namespace CarPointCloudWebApi.Tests;

[Collection("Integration")]
public class RateLimitTests
{
    private readonly HttpClient _client;

    public RateLimitTests(TestFixture fixture)
    {
        _client = fixture.Client;
    }

    [DatabaseFact]
    public async Task CoalVolumeExceedRateLimit_Returns400()
    {
        // 等待限流窗口完全重置，避免与其他测试的计数重叠
        await Task.Delay(1100);

        var statusCodes = new List<int>();
        for (var i = 0; i < 4; i++)
        {
            var (timestamp, signature) = AuthHelper.GenerateAuthHeaders();
            var request = new HttpRequestMessage(HttpMethod.Get, "/api/v1/coalVolumes?limit=1");
            request.Headers.Add("Timestamp", timestamp);
            request.Headers.Add("Signature", signature);
            var response = await _client.SendAsync(request);
            statusCodes.Add((int)response.StatusCode);
        }

        Assert.Equal(new[] { 200, 200, 200, 400 }, statusCodes);
    }

    [DatabaseFact]
    public async Task PathCaseAndTrailingSlash_ShareTheSameRateLimitBucket()
    {
        await Task.Delay(1100);

        var paths = new[]
        {
            "/api/v1/coalVolumes?limit=1",
            "/API/V1/COALVOLUMES?limit=1",
            "/api/v1/coalVolumes/?limit=1",
            "/api/v1/coalVolumes?limit=1"
        };
        var statusCodes = new List<int>();
        foreach (var path in paths)
        {
            var (timestamp, signature) = AuthHelper.GenerateAuthHeaders();
            using var request = new HttpRequestMessage(HttpMethod.Get, path);
            request.Headers.Add("Timestamp", timestamp);
            request.Headers.Add("Signature", signature);
            using var response = await _client.SendAsync(request);
            statusCodes.Add((int)response.StatusCode);
        }

        Assert.Equal(new[] { 200, 200, 200, 400 }, statusCodes);
    }
}
