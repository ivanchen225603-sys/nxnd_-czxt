using System.Text.Json;
using Microsoft.AspNetCore.HttpOverrides;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.StaticFiles;
using Serilog;
using Serilog.Events;

// 组合根：唯一配置来源是工作目录的 appsettings.json（不按环境叠加）。
// 分层：Api（控制器、中间件）→ Application（用例）→ Domain（模型与规则）← Infrastructure（DM8、外部 HTTP）。
var builder = WebApplication.CreateBuilder(args);
builder.Configuration.Sources.Clear();
builder.Configuration.AddJsonFile(
    Path.Combine(builder.Environment.ContentRootPath, "appsettings.json"), optional: false, reloadOnChange: false);

Log.Logger = new LoggerConfiguration()
    .ReadFrom.Configuration(builder.Configuration)
    .MinimumLevel.Override("CarPointCloudWebApi.Api.ApiAccessLogMiddleware", LogEventLevel.Debug)
    .MinimumLevel.Override("CarPointCloudWebApi.Infrastructure.PartnerLicensePlateClient", LogEventLevel.Debug)
    .MinimumLevel.Override("Microsoft.AspNetCore.Hosting.Diagnostics", LogEventLevel.Warning)
    .Enrich.FromLogContext()
    .Enrich.WithProperty("Application", "CarPointCloudWebApi")
    .WriteTo.Console(outputTemplate:
        "[{Timestamp:HH:mm:ss.fff}] [{Level:u3}] {Message:lj} {Properties:j}{NewLine}{Exception}")
    .WriteTo.File(
        path: "logs/app-.log",
        rollingInterval: RollingInterval.Day,
        retainedFileCountLimit: 15,
        outputTemplate:
        "{Timestamp:yyyy-MM-dd HH:mm:ss.fff zzz} [{Level:u3}] {Message:lj} {Properties:j}{NewLine}{Exception}",
        encoding: System.Text.Encoding.UTF8)
    .CreateLogger();
builder.Host.UseSerilog();

var configuration = builder.Configuration;
var services = builder.Services;
var databaseEnabled = configuration.GetValue("Database:Enabled", true);

services.AddControllers()
    .ConfigureApiBehaviorOptions(options => options.InvalidModelStateResponseFactory = context =>
    {
        var errors = context.ModelState
            .Where(x => x.Value?.Errors.Count > 0)
            .Select(x => $"{x.Key}: {x.Value!.Errors[0].ErrorMessage}");
        return new BadRequestObjectResult(PartnerResponseFactory.Fail(
            context.HttpContext.Request.Path, 400, $"参数校验失败: {string.Join("; ", errors)}"));
    })
    .AddJsonOptions(options => options.JsonSerializerOptions.PropertyNamingPolicy = JsonNamingPolicy.CamelCase);
services.AddHttpContextAccessor();
services.AddEndpointsApiExplorer();
services.AddSwaggerGen();
services.AddHealthChecks();
services.AddSingleton(TimeProvider.System);

services.Configure<IntegrationOptions>(configuration.GetSection("Integration"));
services.Configure<ApiAccessLogRetentionOptions>(configuration.GetSection(ApiAccessLogRetentionOptions.SectionName));
services.Configure<JwtOptions>(configuration.GetSection(JwtOptions.SectionName));

if (databaseEnabled)
{
    services.AddScoped<IIntegrationRepository, IntegrationRepository>();
    services.AddScoped<IAuthRepository, AuthRepository>();
    services.AddScoped<ILogRepository, LogRepository>();
    services.AddScoped<IEmptyVehicleProfileRepository, EmptyVehicleProfileRepository>();
}
else
{
    services.AddScoped<IIntegrationRepository, NullIntegrationRepository>();
    services.AddScoped<IEmptyVehicleProfileRepository, NullEmptyVehicleProfileRepository>();
}

services.AddSingleton<PartnerQueryExecutor>();
services.AddScoped<IntegrationQueryService>();
services.AddSingleton<JwtTokenService>();
services.AddSingleton<IRadarLicensePlateSource, MockRadarLicensePlateSource>();
services.AddSingleton<RadarLiveClient>();
services.AddSingleton<RadarCompletionClient>();
services.AddLoadRadar(configuration);
services.AddHostedService<ApiAccessLogRetentionService>();
if (databaseEnabled && configuration.GetValue("Integration:ExternalPartnerEnabled", true))
    services.AddHostedService<PlateAcquisitionWorker>();
if (configuration.GetValue("Database:InitializeOnStartup", true))
    services.AddHostedService<DatabaseInitService>();

services.AddHttpClient<PartnerLicensePlateClient>(client => client.Timeout = TimeSpan.FromSeconds(10))
    .ConfigurePrimaryHttpMessageHandler(() => new HttpClientHandler
    {
        UseProxy = !(Uri.TryCreate(configuration["Integration:PartnerBaseUrl"], UriKind.Absolute, out var partner) &&
                     partner.IsLoopback)
    });
services.AddHttpClient(RadarLiveClient.HttpClientName, client =>
{
    client.BaseAddress = new Uri(configuration["Radar:BaseUrl"]!);
    client.Timeout = TimeSpan.FromSeconds(15);
}).ConfigurePrimaryHttpMessageHandler(() => new HttpClientHandler { UseProxy = false });
services.AddHttpClient(RadarCompletionClient.HttpClientName, client =>
{
    client.BaseAddress = new Uri("http://127.0.0.1:8010");
    client.Timeout = TimeSpan.FromSeconds(8);
}).ConfigurePrimaryHttpMessageHandler(() => new HttpClientHandler { UseProxy = false });

try
{
    var app = builder.Build();
    if (app.Environment.IsDevelopment())
    {
        app.UseSwagger();
        app.UseSwaggerUI();
    }

    app.UseForwardedHeaders(new ForwardedHeadersOptions
    {
        ForwardedHeaders = ForwardedHeaders.XForwardedFor | ForwardedHeaders.XForwardedProto
    });
    app.UseMiddleware<ApiAccessLogMiddleware>();
    app.UseMiddleware<ApiExceptionMiddleware>();
    app.UseMiddleware<PartnerAuthMiddleware>();
    app.UseMiddleware<ExternalPartnerGateMiddleware>();
    app.UseMiddleware<PartnerRateLimitMiddleware>();
    app.UseStatusCodePages(async context =>
    {
        var response = context.HttpContext.Response;
        if (response.HasStarted || response.ContentType?.Contains("json", StringComparison.OrdinalIgnoreCase) == true)
            return;
        response.ContentType = "application/json; charset=utf-8";
        await response.WriteAsJsonAsync(PartnerResponseFactory.Fail(
            context.HttpContext.Request.Path, response.StatusCode, $"请求处理失败 (HTTP {response.StatusCode})"));
    });
    var contentTypes = new FileExtensionContentTypeProvider();
    contentTypes.Mappings[".pcd"] = "application/octet-stream";
    app.UseStaticFiles(new StaticFileOptions { ContentTypeProvider = contentTypes, ServeUnknownFileTypes = true });
    app.UseMiddleware<LoadRadarAgentAuthMiddleware>();
    app.UseMiddleware<JwtAuthMiddleware>();
    app.MapControllers();
    app.MapHealthChecks("/healthz");
    app.Run();
}
catch (Exception ex)
{
    Log.Fatal(ex, "应用程序启动失败");
    throw;
}
finally
{
    Log.CloseAndFlush();
}

public partial class Program;
