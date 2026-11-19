using CarPointCloudLadarWorker.Config;

namespace CarPointCloudLadarWorker.Radar;

/// <summary>对外输出的一台雷达状态与帧计数。</summary>
public sealed record RadarStatus(
    string CameraIdentifier,
    string RadarModel,
    string Address,
    ushort Port,
    DateTime Timestamp,
    bool Connected,
    DateTime? LastResumeRequestedAt,
    long RawFrameCount,
    long DecodedFrameCount,
    long AcceptedFrameCount,
    double? LastRawFrameAgoSeconds,
    double? LastDecodedFrameAgoSeconds,
    int? LastFinitePointCount,
    double? ObservedCycleSeconds);

/// <summary>一台已加载的雷达：SDK 句柄、是否已下发持续扫描，以及帧计数。</summary>
/// <remarks>计数由雷达回调线程写、HTTP/健康线程读，统一在本对象锁内。</remarks>
internal sealed class RadarDevice(LadarDeviceOptions options, SdkDevice sdk)
{
    private const int CycleSampleWindow = 12;

    private readonly Queue<double> _intervals = new();
    private long _rawFrames;
    private long _decodedFrames;
    private long _acceptedFrames;
    private DateTime? _lastRawFrameAt;
    private DateTime? _lastDecodedAt;
    private long? _lastDecodedTimestamp;
    private int? _lastFinitePointCount;

    public LadarDeviceOptions Options { get; } = options;

    public SdkDevice Sdk { get; } = sdk;

    public string Camera => Options.CameraIdentifier;

    /// <summary>本次连接是否已成功下发 Standby(0)；连接离开 Ready 后复位。</summary>
    public volatile bool Resumed;

    public DateTime? LastResumedAt { get; set; }

    public void RecordRaw(DateTime now)
    {
        lock (this)
        {
            _rawFrames++;
            _lastRawFrameAt = now;
        }
    }

    public void RecordDecoded(DateTime now, long timestamp, int finitePointCount, TimeProvider time)
    {
        lock (this)
        {
            _decodedFrames++;
            _lastDecodedAt = now;
            _lastFinitePointCount = finitePointCount;
            if (_lastDecodedTimestamp is { } previous)
            {
                _intervals.Enqueue(time.GetElapsedTime(previous, timestamp).TotalSeconds);
                if (_intervals.Count > CycleSampleWindow)
                    _intervals.Dequeue();
            }
            _lastDecodedTimestamp = timestamp;
        }
    }

    public void RecordAccepted()
    {
        lock (this) _acceptedFrames++;
    }

    /// <summary>最近若干次解析点云间隔的中位数（秒）。</summary>
    public double? CycleSeconds
    {
        get
        {
            lock (this)
            {
                if (_intervals.Count == 0)
                    return null;
                var sorted = _intervals.Order().ToArray();
                return sorted[sorted.Length / 2];
            }
        }
    }

    public RadarStatus Status(DateTime now)
    {
        var connected = Sdk.IsConnected();
        var cycle = CycleSeconds;
        lock (this)
        {
            return new RadarStatus(
                Camera, Options.RadarModel, Options.Address, Options.Port, now, connected, LastResumedAt,
                _rawFrames, _decodedFrames, _acceptedFrames,
                (now - _lastRawFrameAt)?.TotalSeconds,
                (now - _lastDecodedAt)?.TotalSeconds,
                _lastFinitePointCount,
                cycle);
        }
    }
}
