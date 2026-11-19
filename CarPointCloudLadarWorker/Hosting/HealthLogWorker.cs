using System.Text.Json;
using CarPointCloudLadarWorker.Config;
using CarPointCloudLadarWorker.Radar;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Hosting;

/// <summary>定期记录每台雷达的连接与帧计数，并写 {camera}.json 快照供运维脚本读取。</summary>
public sealed class HealthLogWorker(
    IOptions<LadarCaptureOptions> options,
    RadarHost radars,
    TimeProvider time,
    ILogger<HealthLogWorker> logger) : BackgroundService
{
    private static readonly JsonSerializerOptions Json = new() { PropertyNamingPolicy = JsonNamingPolicy.CamelCase };

    private readonly LadarCaptureOptions _options = options.Value;

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        if (_options.HealthLogIntervalSeconds <= 0)
            return;

        Directory.CreateDirectory(_options.HealthSnapshotDirectory);
        using var timer = new PeriodicTimer(TimeSpan.FromSeconds(_options.HealthLogIntervalSeconds), time);
        while (await timer.WaitForNextTickAsync(stoppingToken))
        {
            foreach (var s in radars.Statuses(DateTime.Now))
            {
                logger.LogInformation(
                    "雷达健康状态: camera={Camera}, endpoint={Address}:{Port}, connected={Connected}, " +
                    "rawFrames={RawFrames}, decoded={Decoded}, accepted={Accepted}, 最近有限点数={FinitePoints}, " +
                    "周期中位数={Cycle}s, lastRawFrameAgo={LastRawFrameAgo}s",
                    s.CameraIdentifier, s.Address, s.Port, s.Connected, s.RawFrameCount, s.DecodedFrameCount,
                    s.AcceptedFrameCount, s.LastFinitePointCount, s.ObservedCycleSeconds?.ToString("0.000"),
                    s.LastRawFrameAgoSeconds?.ToString("0.0"));
                await File.WriteAllTextAsync(
                    Path.Combine(_options.HealthSnapshotDirectory, $"{s.CameraIdentifier}.json"),
                    JsonSerializer.Serialize(s, Json), stoppingToken);
            }
        }
    }
}
