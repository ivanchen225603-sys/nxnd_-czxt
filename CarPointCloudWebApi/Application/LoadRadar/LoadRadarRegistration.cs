using Swashbuckle.AspNetCore.SwaggerGen;

namespace CarPointCloudWebApi.Application;

/// <summary>载货雷达（JT128）模块的服务注册。</summary>
public static class LoadRadarRegistration
{
    public static IServiceCollection AddLoadRadar(this IServiceCollection services, IConfiguration configuration)
    {
        services.Configure<SwaggerGenOptions>(options => options.MapType<LoadRadarNumber>(() =>
            new Microsoft.OpenApi.Models.OpenApiSchema { Type = "number", Format = "double" }));
        services.Configure<LoadRadarOptions>(configuration.GetSection("LoadRadar"));
        services.AddScoped<ILoadRadarRepository>(sp =>
            configuration.GetValue<bool>("LoadRadar:Enabled") && configuration.GetValue("Database:Enabled", true)
                ? ActivatorUtilities.CreateInstance<LoadRadarRepository>(sp)
                : new UnavailableLoadRadarRepository());
        services.AddSingleton<LoadRadarAssetStore>();
        services.AddHttpClient(LoadRadarLiveClient.HttpClientName, client => client.Timeout = TimeSpan.FromSeconds(3))
            .ConfigurePrimaryHttpMessageHandler(() => new SocketsHttpHandler { UseProxy = false });
        services.AddSingleton<LoadRadarLiveClient>();
        services.AddHttpClient(LoadRadarCommandClient.HttpClientName, client => client.Timeout = TimeSpan.FromSeconds(5))
            .RedactLoggedHeaders([LoadRadarAgentAuthMiddleware.KeyHeader])
            .ConfigurePrimaryHttpMessageHandler(() => new SocketsHttpHandler { UseProxy = false });
        services.AddSingleton<LoadRadarCommandClient>();
        services.AddScoped<LoadRadarService>();
        services.AddScoped<LoadRadarPassageService>();
        services.AddScoped<LoadRadarStandstillService>();
        services.AddScoped<LoadRadarPublicationProcessor>();
        services.AddHostedService<LoadRadarPublicationWorker>();
        return services;
    }
}

// 不用内存假入库，数据库关闭时绝不能给采集端持久化成功回执。
internal sealed class UnavailableLoadRadarRepository : ILoadRadarRepository
{
    private static LoadRadarException Unavailable() => new(503, "载货雷达持久化未启用");
    public LoadMeasurementRecord SaveMeasurement(LoadMeasurementRecord record) => throw Unavailable();
    public LoadMeasurementRecord? GetMeasurement(string id) => throw Unavailable();
    public IReadOnlyList<LoadMeasurementRecord> GetHistory(string? nodeId, string? channelId, string? licensePlate,
        string? beforeId, int take, DateTimeOffset? from = null, DateTimeOffset? to = null) => throw Unavailable();
    public LoadPassageBinding BindPassage(LoadMeasurementRecord measurement, LoadPassageBinding binding) => throw Unavailable();
    public LoadBaselineVersion CreateBaseline(LoadBaselineVersion version) => throw Unavailable();
    public LoadBaselineVersion? GetBaseline(string id) => throw Unavailable();
    public IReadOnlyList<LoadBaselineVersion> GetBaselines() => throw Unavailable();
    public LoadBaselineVersion ActivateBaseline(string id, string? expectedVersionId, string actor, string reason) => throw Unavailable();
    public LoadCoalPublication Publish(LoadMeasurementRecord measurement, CoalVolumeRecord coal, string actor, string reason) => throw Unavailable();
}
