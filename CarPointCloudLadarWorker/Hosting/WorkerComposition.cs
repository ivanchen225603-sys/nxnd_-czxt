using System.Threading.Channels;
using CarPointCloudLadarWorker.Analysis;
using CarPointCloudLadarWorker.Api;
using CarPointCloudLadarWorker.Capture;
using CarPointCloudLadarWorker.Config;
using CarPointCloudLadarWorker.Domain;
using CarPointCloudLadarWorker.Preview;
using CarPointCloudLadarWorker.Radar;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Hosting;

/// <summary>注册 Worker 的全部服务；Program 与端到端测试共用。</summary>
public static class WorkerComposition
{
    /// <remarks>
    /// 数据流：RadarHost（SDK 回调 → ScanSessions 判定）→ 帧通道 → CaptureWorker（PCD、配对）
    /// → AnalysisWorker（Analyzer /analyze-pair，Analyzer 直写 DM8）；ControlApiServer 对外提供控制与查询。
    /// </remarks>
    public static IServiceCollection AddWorker(this IServiceCollection services, LadarCaptureOptions options)
    {
        services.AddSingleton(Options.Create(options));
        services.AddSingleton(TimeProvider.System);
        services.AddSingleton(Channel.CreateUnbounded<PointCloudFrame>());
        services.AddSingleton<ScanSessions>();
        services.AddSingleton<LivePreview>();
        services.AddSingleton<AnalysisTaskStore>();
        services.AddHttpClient<AnalyzerClient>(c => c.Timeout = TimeSpan.FromSeconds(options.AnalyzerTimeoutSeconds))
            .ConfigurePrimaryHttpMessageHandler(() => new HttpClientHandler { UseProxy = false });
        services.AddHttpClient<PlateClient>(c => c.Timeout = TimeSpan.FromSeconds(options.LicensePlateFetchTimeoutSeconds));
        services.AddHttpClient<StaticEvidenceClient>(c => c.Timeout = TimeSpan.FromSeconds(3))
            .ConfigurePrimaryHttpMessageHandler(() => new HttpClientHandler { UseProxy = false });

        // 先恢复分析任务，再启动采集；停止时反序，先停雷达和采集，最后停分析。
        AddHostedSingleton<AnalysisWorker>(services);
        AddHostedSingleton<CaptureWorker>(services);
        AddHostedSingleton<RadarHost>(services);
        if (options.EnableHttpApi)
            services.AddHostedService<ControlApiServer>();
        services.AddHostedService<HealthLogWorker>();
        services.AddHostedService<CaptureCleanupWorker>();
        return services;
    }

    private static void AddHostedSingleton<T>(IServiceCollection services) where T : class, IHostedService
    {
        services.AddSingleton<T>();
        services.AddHostedService(sp => sp.GetRequiredService<T>());
    }
}
