using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Mvc.Testing;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.AspNetCore.Builder;
using System.Net;

namespace CarPointCloudWebApi.Tests;

public class TestApiFactory : WebApplicationFactory<Program>
{
    private readonly string _mockPartnerBaseUrl;
    private readonly string _databaseConnectionString;

    public TestApiFactory(string mockPartnerBaseUrl, string databaseConnectionString)
    {
        _mockPartnerBaseUrl = mockPartnerBaseUrl;
        _databaseConnectionString = databaseConnectionString;
    }

    protected override void ConfigureWebHost(IWebHostBuilder builder)
    {
        builder.UseEnvironment("Development");
        builder.ConfigureServices(services => services.AddSingleton<IStartupFilter, TestRemoteAddressFilter>());

        builder.ConfigureAppConfiguration((context, config) =>
        {
            config.AddInMemoryCollection(new Dictionary<string, string?>
            {
                ["Integration:PartnerBaseUrl"] = _mockPartnerBaseUrl,
                ["Integration:AppSecret"] = TestConfiguration.AppSecret,
                ["Integration:ExternalPartnerEnabled"] = "true",
                ["Integration:EnableMockLicensePlate"] = "true",
                ["Integration:MockLicensePlatePrefix"] = "粤A",
                ["Integration:LicensePlatePath"] = "/api/v1/licensePlateRecords",
                ["Integration:PartnerRetryCount"] = "3",
                ["Integration:PartnerFirstCallDelayMs"] = "0",
                ["Integration:PartnerRetryDelayMs"] = "0",
                ["Database:Enabled"] = "true",
                ["Database:InitializeOnStartup"] = "false",
                ["LoadRadar:Enabled"] = "false",
                ["ConnectionStrings:CarPointCloudDb"] = _databaseConnectionString,
                ["Auth:JwtSecret"] = "integration-test-jwt-secret-0123456789abcdef",
                ["Radar:AllowedDeviceAddresses:0"] = "127.0.0.1"
            });
        });
        builder.ConfigureServices((context, _) =>
        {
            if (!string.Equals(context.Configuration.GetConnectionString("CarPointCloudDb"),
                    _databaseConnectionString, StringComparison.Ordinal))
                throw new InvalidOperationException("Integration test database configuration was not applied; startup refused.");
        });
    }

    private sealed class TestRemoteAddressFilter : IStartupFilter
    {
        public Action<IApplicationBuilder> Configure(Action<IApplicationBuilder> next) => app =>
        {
            app.Use((context, continuation) =>
            {
                // 仅 TestServer：模拟受授权的本机代理，随后由生产 ForwardedHeaders 解析客户 IP。
                context.Connection.RemoteIpAddress = IPAddress.Loopback;
                return continuation(context);
            });
            next(app);
        };
    }
}
