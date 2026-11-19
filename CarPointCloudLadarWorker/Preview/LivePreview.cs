using CarPointCloudLadarWorker.Config;
using CarPointCloudLadarWorker.Domain;
using Ladarclr;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Preview;

/// <summary>预览中一台雷达的样本状态。</summary>
public sealed record LivePreviewCameraStatus(
    string CameraIdentifier,
    bool HasSample,
    bool Fresh,
    DateTime? CapturedAt,
    double? AgeSeconds,
    int? SourcePointCount,
    int? PreviewPointCount);

/// <summary>预览链路状态。</summary>
public sealed record LivePreviewStatus(
    bool Enabled,
    bool Available,
    string? UnavailableReason,
    string SessionId,
    long LatestVersion,
    IReadOnlyList<LivePreviewCameraStatus> Cameras);

/// <summary>一份 A+B 融合预览画面；坐标为 A 坐标系下交错的 xyz。</summary>
public sealed record LiveFusedPreviewFrame(
    string SessionId,
    long Version,
    DateTime CapturedAt,
    DateTime CapturedAtA,
    DateTime CapturedAtB,
    string CameraA,
    string CameraB,
    double PairDeltaMilliseconds,
    int SourcePointCountA,
    int SourcePointCountB,
    int PointCountA,
    int PointCountB,
    float[] Coordinates)
{
    public int PreviewPointCount => PointCountA + PointCountB;

    public DateTime OldestCapturedAt => CapturedAtA < CapturedAtB ? CapturedAtA : CapturedAtB;
}

/// <summary>
/// 磅房实时预览：每台雷达每场抽样，用 B_to_A 外参叠加成一份画面。只读、仅内存，与采集准入无关。
/// </summary>
/// <remarks>两台样本都新鲜且采集时间差在窗口内才产出新画面；过期即无画面，而不是旧画面。</remarks>
public sealed class LivePreview
{
    private readonly object _gate = new();
    private readonly LadarCaptureOptions _options;
    private readonly TimeProvider _time;
    private readonly DualRadarExtrinsic? _extrinsic;
    private readonly string _cameraA;
    private readonly string _cameraB;
    private readonly Dictionary<string, Sample> _samples = [];
    private LiveFusedPreviewFrame? _latestFrame;
    private long _version;

    public LivePreview(IOptions<LadarCaptureOptions> options, TimeProvider time)
    {
        _options = options.Value;
        _time = time;
        _cameraA = _options.Devices[0].CameraIdentifier;
        _cameraB = _options.Devices[1].CameraIdentifier;
        if (!_options.EnableLivePreview)
            return;

        _extrinsic = DualRadarExtrinsic.Load(_options.DualRadarExtrinsicPath);
        if (_extrinsic.CameraA != _cameraA || _extrinsic.CameraB != _cameraB)
            throw new InvalidOperationException(
                $"外参点位 {_extrinsic.CameraA}→{_extrinsic.CameraB} 与 Devices {_cameraA}/{_cameraB} 不一致");
    }

    /// <summary>每个进程一个编号，页面据此识别 Worker 重启。</summary>
    public string SessionId { get; } = Guid.NewGuid().ToString("N");

    /// <summary>记录一台雷达的最新一场，两台齐备时生成新画面。</summary>
    public void Observe(string camera, float3[] points, int finiteCount, DateTime capturedAt)
    {
        if (_extrinsic is null || finiteCount == 0)
            return;

        var coordinates = Downsample(points, finiteCount, _options.LivePreviewMaxPointsPerCamera);
        lock (_gate)
        {
            _samples[camera] = new Sample(coordinates, finiteCount, capturedAt);
            TryPublishLocked();
        }
    }

    /// <summary>比 afterVersion 新且未过期的画面，否则 null。</summary>
    public LiveFusedPreviewFrame? GetFrameAfter(long afterVersion)
    {
        lock (_gate)
        {
            return _latestFrame is { } frame && frame.Version > afterVersion && !IsStale(frame.OldestCapturedAt)
                ? frame
                : null;
        }
    }

    public LivePreviewStatus GetStatus()
    {
        var now = _time.GetLocalNow().DateTime;
        lock (_gate)
        {
            var cameras = new[] { _cameraA, _cameraB }.Select(camera =>
            {
                if (!_samples.TryGetValue(camera, out var sample))
                    return new LivePreviewCameraStatus(camera, false, false, null, null, null, null);
                var age = (now - sample.CapturedAt).TotalSeconds;
                return new LivePreviewCameraStatus(camera, true, age <= _options.LivePreviewStaleSeconds,
                    sample.CapturedAt, Math.Round(age, 3), sample.SourcePointCount, sample.Coordinates.Length / 3);
            }).ToArray();

            var reason = UnavailableReasonLocked(cameras, now);
            return new LivePreviewStatus(_options.EnableLivePreview, reason is null, reason, SessionId,
                _latestFrame?.Version ?? 0, cameras);
        }
    }

    private string? UnavailableReasonLocked(LivePreviewCameraStatus[] cameras, DateTime now)
    {
        if (_extrinsic is null)
            return "实时预览未启用（LadarCapture:EnableLivePreview=false）";
        var waiting = cameras.Where(x => !x.HasSample).Select(x => x.CameraIdentifier).ToArray();
        if (waiting.Length > 0)
            return $"等待 {string.Join("、", waiting)} 的首份点云";
        var stale = cameras.Where(x => !x.Fresh).Select(x => x.CameraIdentifier).ToArray();
        if (stale.Length > 0)
            return $"{string.Join("、", stale)} 数据已过期（超过 {_options.LivePreviewStaleSeconds} 秒未更新）";
        if (_latestFrame is null)
            return $"两台雷达采集时间差超过 {_options.LivePreviewPairingWindowSeconds} 秒，尚未形成同场画面";
        var frameAge = (now - _latestFrame.OldestCapturedAt).TotalSeconds;
        return frameAge > _options.LivePreviewStaleSeconds ? $"最近一份融合画面已过期（{frameAge:0} 秒前）" : null;
    }

    private bool IsStale(DateTime capturedAt) =>
        (_time.GetLocalNow().DateTime - capturedAt).TotalSeconds > _options.LivePreviewStaleSeconds;

    private void TryPublishLocked()
    {
        if (!_samples.TryGetValue(_cameraA, out var a) || !_samples.TryGetValue(_cameraB, out var b) ||
            a.Consumed || b.Consumed || IsStale(a.CapturedAt) || IsStale(b.CapturedAt))
            return;
        var delta = Math.Abs((a.CapturedAt - b.CapturedAt).TotalMilliseconds);
        if (delta > _options.LivePreviewPairingWindowSeconds * 1000)
            return;

        var coordinates = new float[a.Coordinates.Length + b.Coordinates.Length];
        Array.Copy(a.Coordinates, coordinates, a.Coordinates.Length);
        for (var i = 0; i < b.Coordinates.Length; i += 3)
        {
            var p = _extrinsic!.Transform(new float3 { x = b.Coordinates[i], y = b.Coordinates[i + 1], z = b.Coordinates[i + 2] });
            coordinates[a.Coordinates.Length + i] = p.x;
            coordinates[a.Coordinates.Length + i + 1] = p.y;
            coordinates[a.Coordinates.Length + i + 2] = p.z;
        }

        _version++;
        _samples[_cameraA] = a with { Consumed = true };
        _samples[_cameraB] = b with { Consumed = true };
        _latestFrame = new LiveFusedPreviewFrame(
            SessionId, _version, a.CapturedAt > b.CapturedAt ? a.CapturedAt : b.CapturedAt,
            a.CapturedAt, b.CapturedAt, _cameraA, _cameraB, Math.Round(delta, 1),
            a.SourcePointCount, b.SourcePointCount, a.Coordinates.Length / 3, b.Coordinates.Length / 3,
            coordinates);
    }

    /// <summary>按固定步长从有限点中抽样，输出交错的 xyz。</summary>
    private static float[] Downsample(float3[] points, int finiteCount, int maxPoints)
    {
        var stride = (finiteCount + maxPoints - 1) / maxPoints;
        var coordinates = new List<float>(Math.Min(finiteCount, maxPoints) * 3);
        var index = 0;
        foreach (var p in points)
        {
            if (!PointFilter.IsFinite(p) || index++ % stride != 0)
                continue;
            coordinates.Add(p.x);
            coordinates.Add(p.y);
            coordinates.Add(p.z);
        }
        return [.. coordinates];
    }

    private sealed record Sample(float[] Coordinates, int SourcePointCount, DateTime CapturedAt)
    {
        public bool Consumed { get; init; }
    }
}
