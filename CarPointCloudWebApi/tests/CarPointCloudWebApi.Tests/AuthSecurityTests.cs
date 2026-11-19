using System.Text.Json;
using Xunit;

namespace CarPointCloudWebApi.Tests;

[Collection("Integration")]
public class AuthSecurityTests
{
    private const string TargetUrl = "/api/v1/vehicleDimensions?limit=1";
    private readonly HttpClient _client;
    private readonly TestDbHelper _db;

    public AuthSecurityTests(TestFixture fixture)
    {
        _client = fixture.Client;
        _db = fixture.DbHelper;
        _db.InsertTestVehicleDimensionRecord(
            "TESTAUTH001", "CAM001", "success",
            100.5m, 800.0m, 250.0m, 150.0m, 90.0m, 20.0m);
    }

    [DatabaseTheory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    public async Task MissingTimestampOrSignature_Returns401(bool includeTimestamp, bool includeSignature)
    {
        await Task.Delay(1100);

        var request = new HttpRequestMessage(HttpMethod.Get, TargetUrl);
        var (timestamp, signature) = AuthHelper.GenerateAuthHeaders();
        if (includeTimestamp)
            request.Headers.Add("Timestamp", timestamp);
        if (includeSignature)
            request.Headers.Add("Signature", signature);

        var response = await _client.SendAsync(request);
        var content = await response.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(content);

        Assert.Equal(401, (int)response.StatusCode);
        Assert.Equal(401, doc.RootElement.GetProperty("code").GetInt32());
        Assert.Contains("缺少 Timestamp 或 Signature", doc.RootElement.GetProperty("message").GetString() ?? string.Empty);
    }

    [DatabaseFact]
    public async Task InvalidTimestampFormat_Returns401()
    {
        await Task.Delay(1100);

        var request = new HttpRequestMessage(HttpMethod.Get, TargetUrl);
        request.Headers.Add("Timestamp", "not_a_number");
        request.Headers.Add("Signature", "some_signature");

        var response = await _client.SendAsync(request);
        var content = await response.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(content);

        Assert.Equal(401, (int)response.StatusCode);
        Assert.Equal(401, doc.RootElement.GetProperty("code").GetInt32());
        Assert.Contains("Timestamp 格式非法", doc.RootElement.GetProperty("message").GetString() ?? string.Empty);
    }

    [DatabaseFact]
    public async Task ExpiredTimestamp_Returns401()
    {
        await Task.Delay(1100);

        var expiredTimestamp = DateTimeOffset.UtcNow.AddSeconds(-120).ToUnixTimeMilliseconds().ToString();
        var signature = AuthHelper.GenerateSignature(expiredTimestamp, TestConfiguration.AppSecret);

        var request = new HttpRequestMessage(HttpMethod.Get, TargetUrl);
        request.Headers.Add("Timestamp", expiredTimestamp);
        request.Headers.Add("Signature", signature);

        var response = await _client.SendAsync(request);
        var content = await response.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(content);

        Assert.Equal(401, (int)response.StatusCode);
        Assert.Equal(401, doc.RootElement.GetProperty("code").GetInt32());
        Assert.Contains("Timestamp 已过期", doc.RootElement.GetProperty("message").GetString() ?? string.Empty);
    }

    [DatabaseFact]
    public async Task WrongSignature_Returns401()
    {
        await Task.Delay(1100);

        var (timestamp, _) = AuthHelper.GenerateAuthHeaders();

        var request = new HttpRequestMessage(HttpMethod.Get, TargetUrl);
        request.Headers.Add("Timestamp", timestamp);
        request.Headers.Add("Signature", "wrong_sig");

        var response = await _client.SendAsync(request);
        var content = await response.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(content);

        Assert.Equal(401, (int)response.StatusCode);
        Assert.Equal(401, doc.RootElement.GetProperty("code").GetInt32());
        Assert.Contains("Signature 不匹配", doc.RootElement.GetProperty("message").GetString() ?? string.Empty);
    }

    [DatabaseFact]
    public async Task ValidAuth_Returns200()
    {
        await Task.Delay(1100);

        var (timestamp, signature) = AuthHelper.GenerateAuthHeaders();

        var request = new HttpRequestMessage(HttpMethod.Get, TargetUrl);
        request.Headers.Add("Timestamp", timestamp);
        request.Headers.Add("Signature", signature);

        var response = await _client.SendAsync(request);
        var content = await response.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(content);

        Assert.Equal(200, (int)response.StatusCode);
        Assert.Equal(200, doc.RootElement.GetProperty("code").GetInt32());
    }
}
