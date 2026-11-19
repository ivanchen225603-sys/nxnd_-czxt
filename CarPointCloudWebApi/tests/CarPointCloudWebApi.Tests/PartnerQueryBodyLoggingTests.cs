using System.Text;
using System.Text.Json;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class PartnerQueryBodyLoggingTests
{
    [Theory]
    [InlineData("GET", "/api/v1/vehicleDimensions", true, true)]
    [InlineData("GET", "/API/V1/coalVolumes/", true, true)]
    [InlineData("GET", "/api/v1/coalVolumes", false, false)]
    [InlineData("POST", "/api/v1/coalVolumes", true, false)]
    [InlineData("GET", "/api/v1/vehicleDimensions/other", true, false)]
    [InlineData("POST", "/api/v1/internal/pointcloud/vehicleDimensions", true, false)]
    [InlineData("GET", "/api/v1/vehicleWorkflows/workflow/licensePlate", true, false)]
    public async Task Query_body_logging_never_expands_to_acquisition_or_internal_uploads(
        string method, string path, bool enabled, bool expectedCapture)
    {
        var repository = new QueryFixtureRepository();
        var logs = new QueryLogProvider();
        using var provider = new ServiceCollection().AddSingleton<IIntegrationRepository>(repository).BuildServiceProvider();
        using var logging = LoggerFactory.Create(builder => builder.AddProvider(logs));
        var config = Configuration(enabled);
        var context = new DefaultHttpContext();
        context.Request.Method = method; context.Request.Path = path;
        context.Request.QueryString = new("?limit=1");
        context.Response.Body = new MemoryStream();
        var middleware = new ApiAccessLogMiddleware(async current =>
        {
            current.Response.ContentType = "application/json";
            await current.Response.WriteAsync("{\"code\":200,\"message\":\"success\",\"data\":[]}");
        }, provider.GetRequiredService<IServiceScopeFactory>(), config, logging.CreateLogger<ApiAccessLogMiddleware>());
        await middleware.InvokeAsync(context);
        var access = Assert.Single(repository.Records.Where(record => record.RecordKind == "Access"));
        Assert.Equal(expectedCapture, access.ResponseBody is not null);
        Assert.Equal(expectedCapture, logs.Entries.Any(entry => entry.Stage == "QueryResponse"));
        Assert.Equal("?limit=1", access.QueryString);
    }

    [Fact]
    public async Task Logging_redacts_and_truncates_without_consuming_input_or_changing_output()
    {
        const string secret = "sensitive-body-sentinel";
        var requestBody = "{\"password\":\"" + secret + "\",\"parameter\":1}";
        var responseBody = JsonSerializer.Serialize(new
        {
            code = 200,
            message = "success",
            data = new[] { new { token = secret, licensePlate = "宁A48394Q", text = new string('x', 6000) } }
        });
        var repository = new QueryFixtureRepository();
        var logs = new QueryLogProvider();
        using var provider = new ServiceCollection().AddSingleton<IIntegrationRepository>(repository).BuildServiceProvider();
        using var logging = LoggerFactory.Create(builder => builder.AddProvider(logs));
        var context = new DefaultHttpContext();
        context.Request.Method = "GET"; context.Request.Path = "/api/v1/coalVolumes";
        context.Request.ContentType = "application/json"; context.Request.ContentLength = Encoding.UTF8.GetByteCount(requestBody);
        context.Request.Body = new MemoryStream(Encoding.UTF8.GetBytes(requestBody));
        context.Request.Headers["Signature"] = secret; context.Request.Headers["Authorization"] = "Bearer " + secret;
        context.Response.Body = new MemoryStream();
        var middleware = new ApiAccessLogMiddleware(async current =>
        {
            using var reader = new StreamReader(current.Request.Body, leaveOpen: true);
            Assert.Equal(requestBody, await reader.ReadToEndAsync());
            current.Response.ContentType = "application/json";
            await current.Response.WriteAsync(responseBody);
        }, provider.GetRequiredService<IServiceScopeFactory>(), Configuration(true, 4096), logging.CreateLogger<ApiAccessLogMiddleware>());
        await middleware.InvokeAsync(context);
        context.Response.Body.Position = 0;
        Assert.Equal(responseBody, await new StreamReader(context.Response.Body).ReadToEndAsync());
        var access = Assert.Single(repository.Records.Where(record => record.RecordKind == "Access"));
        Assert.Null(access.RequestBody); // 合同 GET 不读取或记录额外正文。
        Assert.Contains("truncated", access.ResponseBody);
        Assert.InRange(access.ResponseBody!.Length, 4096, 4200);
        Assert.DoesNotContain(secret, JsonSerializer.Serialize(new { records = repository.Records, logs = logs.Entries }));
    }

    private static IConfiguration Configuration(bool enabled, int maxBodyChars = 1048576) =>
        new ConfigurationBuilder().AddInMemoryCollection(new Dictionary<string, string?>
        {
            ["Database:Enabled"] = "true",
            ["AuditLog:CaptureBodies"] = "false",
            ["AuditLog:CapturePartnerQueryBodies"] = enabled.ToString(),
            ["AuditLog:CaptureHeaders"] = "true",
            ["AuditLog:MaxBodyChars"] = maxBodyChars.ToString(System.Globalization.CultureInfo.InvariantCulture)
        }).Build();
}
