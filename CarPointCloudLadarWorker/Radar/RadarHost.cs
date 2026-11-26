using System.Runtime.InteropServices;
using System.Threading.Channels;
using CarPointCloudLadarWorker.Config;
using CarPointCloudLadarWorker.Domain;
using CarPointCloudLadarWorker.Preview;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Radar;

/// <summary>
/// 雷达 SDK 生命周期与点云回调：让两台雷达保持持续扫描，把每场点云交给预览与扫描会话，
/// 会话接纳的场写入帧通道等待落盘。
/// </summary>
public sealed class RadarHost(
    IOptions<LadarCaptureOptions> options,
    ScanSessions sessions,
    LivePreview preview,
    Channel<PointCloudFrame> frames,
    TimeProvider time,
    ILogger<RadarHost> logger) : BackgroundService
{
    private const uint ReadyStatus = 2;

    private readonly LadarCaptureOptions _options = options.Value;
    private readonly List<RadarDevice> _devices = [];
    private ILadarSdkBackend? _backend;
    private Thread? _pollerThread;

    private string Source => _options.EnableSyntheticRadar ? "SyntheticLadar91" : "Ladar118";

    /// <summary>全部雷达当前状态，顺序同 Devices 配置。</summary>
    public IReadOnlyList<RadarStatus> Statuses(DateTime now) => [.. _devices.Select(x => x.Status(now))];

    /// <summary>两台雷达解析周期中较大的一个（秒），尚无样本时为 null。</summary>
    public double? EstimatedCycleSeconds => _devices.Max(x => x.CycleSeconds);

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        // 设备在第一次 await 之前加载完，后注册的 HTTP 服务启动时即可读到完整清单。
        _backend = _options.EnableSyntheticRadar ? new SyntheticLadarSdkBackend(_options) : CreatePlatformBackend();
        foreach (var device in _options.Devices)
        {
            var sdk = _backend.AddLadar(device.Address, device.Port, OnFrame, OnStatus);
            _devices.Add(new RadarDevice(device, sdk));
            logger.LogInformation("{Model} 已配置: camera={Camera}, endpoint={Address}:{Port}",
                device.RadarModel, device.CameraIdentifier, device.Address, device.Port);
        }
        _pollerThread = new Thread(_backend.Run) { IsBackground = true, Name = "LadarPoller" };
        _pollerThread.Start();

        // 持续扫描：每台雷达连上后下发一次 Standby(0)；断线重连后再下发一次。
        using var timer = new PeriodicTimer(TimeSpan.FromSeconds(1), time);
        do
        {
            foreach (var device in _devices.Where(x => !x.Resumed && x.Sdk.IsConnected()))
            {
                var result = device.Sdk.Standby(0);
                device.Resumed = result > 0;
                if (device.Resumed)
                {
                    device.LastResumedAt = DateTime.Now;
                    logger.LogInformation("雷达已进入持续扫描: camera={Camera}", device.Camera);
                }
                else
                {
                    logger.LogWarning("雷达进入持续扫描失败: camera={Camera}, SDK 返回={Result}", device.Camera, result);
                }
            }
        }
        while (await timer.WaitForNextTickAsync(stoppingToken).ConfigureAwait(false));
    }

    public override async Task StopAsync(CancellationToken cancellationToken)
    {
        await base.StopAsync(cancellationToken);
        _backend?.Stop();
        _pollerThread?.Join();
        _backend?.Dispose();
        frames.Writer.Complete();
    }

    private static ILadarSdkBackend CreatePlatformBackend()
    {
#if WINDOWS_LADAR_SDK
        return new WindowsLadarSdkBackend();
#else
        return new LinuxLadarSdkBackend();
#endif
    }

    private RadarDevice Find(SdkDevice sdk) => _devices.First(x => ReferenceEquals(x.Sdk, sdk));

    private void OnStatus(SdkDevice sdk, uint oldStatus, uint status)
    {
        var device = Find(sdk);
        if (oldStatus == ReadyStatus && status != ReadyStatus)
            device.Resumed = false;
        logger.LogInformation("{Model} 状态变更: camera={Camera}, {OldStatus} → {Status}",
            device.Options.RadarModel, device.Camera, oldStatus, status);
    }

    private void OnFrame(SdkDevice sdk, IntPtr frame, int frameLength)
    {
        // 时间戳取在回调入口：扫描会话据此判断这场是否早于指令。
        var timestamp = time.GetTimestamp();
        var callbackUtc = time.GetUtcNow();
        var device = Find(sdk);
        switch ((ushort)Marshal.ReadInt16(frame, 2))
        {
            case 118:
                OnPointCloud(device, sdk, frame, frameLength, timestamp, callbackUtc);
                break;
            case 7105: logger.LogWarning("检测到雷达脏污或遮挡: {Camera}", device.Camera); break;
            case 7106: logger.LogInformation("雷达脏污或遮挡已恢复: {Camera}", device.Camera); break;
            case 7109: logger.LogError("雷达报告故障: {Camera}", device.Camera); break;
            case 7110: logger.LogInformation("雷达故障已恢复: {Camera}", device.Camera); break;
        }
    }

    private void OnPointCloud(RadarDevice device, SdkDevice sdk, IntPtr frame, int frameLength, long timestamp,
        DateTimeOffset callbackUtc)
    {
        var now = callbackUtc.LocalDateTime;
        device.RecordRaw(now);
        if (sdk.Parse118(frame, frameLength, out var points) <= 0 || points.Length == 0)
            return;

        var finite = points.Count(PointFilter.IsFinite);
        device.RecordDecoded(now, timestamp, finite, time);
        preview.Observe(device.Camera, points, finite, now);

        var workflowId = sessions.Offer(device.Camera, finite, timestamp, out var window);
        if (workflowId is null)
            return;

        device.RecordAccepted();
        frames.Writer.TryWrite(new PointCloudFrame(
            [.. points.Where(PointFilter.IsFinite)], device.Camera, Source, now, workflowId,
            window is null ? null : StaticFrameEvidence.FromCallback(window, callbackUtc, device.Options)));
        logger.LogInformation("扫描已接纳指令后的完整场: workflowId={WorkflowId}, camera={Camera}, 点数={Count}",
            workflowId, device.Camera, finite);
    }
}
