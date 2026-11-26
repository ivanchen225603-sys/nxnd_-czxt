using CarPointCloudLadarWorker.Config;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Domain;

/// <summary>扫描会话状态（对外小写输出）。</summary>
public enum ScanState
{
    Idle,
    Scanning
}

/// <summary>扫描会话结局（对外小写输出）。</summary>
public enum ScanOutcome
{
    None,
    InProgress,
    Completed,
    TimedOut,
    Cancelled
}

/// <summary>一个扫描会话的只读快照。</summary>
public sealed record ScanSnapshot(
    ScanState State,
    ScanOutcome Outcome,
    string? RequestId,
    string? SessionId,
    DateTime? StartedAt,
    DateTime? CompletedAt,
    IReadOnlyList<string> ExpectedCameras,
    IReadOnlyList<string> CapturedCameras,
    IReadOnlyList<string> PersistedCameras,
    string? LastError,
    double EffectiveTimeoutSeconds)
{
    /// <summary>not_started / pending / saved。</summary>
    public string PersistenceState =>
        CapturedCameras.Count == 0 ? "not_started" :
        CapturedCameras.All(PersistedCameras.Contains) ? "saved" : "pending";
}

/// <summary>开始或停止扫描的结果。</summary>
public sealed record ScanCommandResult(
    bool Success,
    bool Started,
    bool Idempotent,
    string Code,
    string Message,
    ScanSnapshot Snapshot);

/// <summary>
/// 持续扫描 + 软件门禁：雷达一直出场，只有收到扫描指令后才接纳点云。
/// </summary>
/// <remarks>
/// 人工及旧指令先忽略交付延迟窗内的回调。已验证的自动停稳派发可在此窗内留存原始点云，
/// 最终尺寸须取得两路完整场的静止证据。每路仍先丢弃
/// <see cref="LadarCaptureOptions.DiscardCompleteCyclesAfterCommand"/> 份跨越指令边界的合格场，
/// 然后接纳下一份合格场。两台各接纳一份即完成；超过超时仍未收齐即 timedout。
/// 同一时刻只有一个会话；requestId 即 workflowId，重复提交返回原会话。
/// 由 HTTP 线程与雷达回调线程并发调用，全部状态在一把锁内。
/// </remarks>
public sealed class ScanSessions
{
    private const int MaxRememberedRequests = 256;
    private const int MaxRequestIdLength = 128;

    private readonly object _gate = new();
    private readonly TimeProvider _time;
    private readonly ILogger<ScanSessions> _logger;
    private readonly string[] _cameras;
    private readonly Dictionary<string, int> _minPoints;
    private readonly int _discards;
    private readonly double _guardSeconds;
    private readonly double _timeoutSeconds;
    private readonly double? _expectedCycleSeconds;
    private readonly Dictionary<string, Session> _byRequest = new(StringComparer.Ordinal);
    private readonly Queue<string> _requestOrder = new();
    private Session? _current;

    public ScanSessions(IOptions<LadarCaptureOptions> options, TimeProvider time, ILogger<ScanSessions> logger)
    {
        var o = options.Value;
        _time = time;
        _logger = logger;
        _cameras = [.. o.Devices.Select(x => x.CameraIdentifier)];
        _minPoints = o.Devices.ToDictionary(x => x.CameraIdentifier, x => x.MinValidFramePointCount);
        _discards = o.DiscardCompleteCyclesAfterCommand;
        _guardSeconds = (o.MaximumFrameDeliveryDelayMilliseconds ?? 0) / 1000.0;
        _timeoutSeconds = o.ScanOnceTimeoutSeconds;
        _expectedCycleSeconds = o.ExpectedFrameCycleSeconds;
    }

    /// <summary>开始一次扫描；requestId 为空时自动生成。</summary>
    public ScanCommandResult Start(string? requestId, DateTimeOffset? captureUntil = null, string? segmentId = null,
        bool verifiedStandstillDispatch = false)
    {
        var id = string.IsNullOrWhiteSpace(requestId) ? Guid.NewGuid().ToString("N") : requestId.Trim();
        lock (_gate)
        {
            ExpireLocked();
            if (!ValidRequestId(id))
                return ResultLocked(false, false, false, "invalid_request_id",
                    $"requestId 必须是 1-{MaxRequestIdLength} 个非控制字符（与 Analyzer 的 workflowId 契约一致）", _current);
            if (_byRequest.TryGetValue(id, out var existing))
            {
                if (!existing.ClosedByDeparture && captureUntil is { } requestedDeadline &&
                    existing.StaticWindow != new StaticCaptureWindow(segmentId, requestedDeadline))
                    return ResultLocked(false, false, false, "capture_context_mismatch", "同一流程的停稳段或截止时间不一致", existing);
                return existing.ClosedByDeparture
                    ? ResultLocked(false, false, true, "vehicle_departed", "本车次已经离场", existing)
                    : ResultLocked(true, false, true, "existing",
                        "相同 requestId 已存在，返回原会话而不重复开启扫描", existing);
            }
            var remaining = captureUntil - _time.GetUtcNow();
            if ((segmentId is not null && captureUntil is null) ||
                remaining <= TimeSpan.Zero || remaining > TimeSpan.FromSeconds(30))
                return ResultLocked(false, false, false, "capture_window_closed", "本车次自动采集窗口无效或已结束", null);
            if (_current?.Outcome == ScanOutcome.InProgress)
                return ResultLocked(false, false, false, "busy", "已有单次扫描会话正在执行", _current);

            _current = NewSession(id, Math.Min(_timeoutSeconds, remaining?.TotalSeconds ?? _timeoutSeconds));
            if (captureUntil is { } deadline)
            {
                _current.StaticWindow = new(segmentId, deadline);
                _current.EarlyStaticCapture = verifiedStandstillDispatch && !string.IsNullOrWhiteSpace(segmentId);
                var guard = _current.EarlyStaticCapture ? 0 : _guardSeconds;
                if (_expectedCycleSeconds is > 0 && guard + (_discards + 1) * _expectedCycleSeconds >= _current.TimeoutSeconds)
                    _logger.LogWarning("自动采集余量紧张 workflowId={WorkflowId} 可用秒={Window} 交付保护秒={Guard} 标称周期秒={Cycle} 丢场数={Discards}",
                        id, _current.TimeoutSeconds, guard, _expectedCycleSeconds, _discards);
            }
            RememberLocked(_current);
            return ResultLocked(true, true, false, "scanning",
                $"扫描会话已创建，每台雷达将先丢弃 {_discards} 份完整场再采集", _current);
        }
    }

    /// <summary>关闭已离场的车次；离场先于派发时也要拒绝该车次的迟到指令。</summary>
    public ScanCommandResult EndPassage(string requestId)
    {
        requestId = requestId.Trim();
        lock (_gate)
        {
            ExpireLocked();
            if (!ValidRequestId(requestId))
                return ResultLocked(false, false, false, "invalid_request_id", "无效的车次标识", null);
            var session = _current?.RequestId == requestId ? _current : _byRequest.GetValueOrDefault(requestId);
            if (session is null)
            {
                session = NewSession(requestId, _timeoutSeconds);
                RememberLocked(session);
            }
            if (session.Outcome != ScanOutcome.InProgress)
                return ResultLocked(true, false, true, "already_closed", "本车次没有活动扫描", session);
            session.ClosedByDeparture = true;
            const string reason = "车辆离场，未收齐的点云不会配对";
            session.Finish(ScanOutcome.Cancelled, reason, _time.GetLocalNow().DateTime);
            _logger.LogInformation("因离场关闭扫描会话: workflowId={WorkflowId}", requestId);
            return ResultLocked(true, false, false, "cancelled", reason, session);
        }
    }

    private static bool ValidRequestId(string id) =>
        id.Length is > 0 and <= MaxRequestIdLength && !id.Any(char.IsControl);

    private Session NewSession(string id, double timeoutSeconds) =>
        new(id, _time.GetLocalNow().DateTime, _time.GetTimestamp(), _cameras, _discards, timeoutSeconds);

    private void RememberLocked(Session session)
    {
        _byRequest.Add(session.RequestId, session);
        _requestOrder.Enqueue(session.RequestId);
        if (_requestOrder.Count > MaxRememberedRequests)
            _byRequest.Remove(_requestOrder.Dequeue());
    }

    /// <summary>取消当前会话；带 expectedRequestId 时只在归属一致时取消。</summary>
    public ScanCommandResult Stop(string reason, string? expectedRequestId = null)
    {
        lock (_gate)
        {
            ExpireLocked();
            if (!string.IsNullOrEmpty(expectedRequestId) && _current?.RequestId != expectedRequestId.Trim())
                return ResultLocked(false, false, false, "request_mismatch",
                    $"当前扫描会话不属于请求的流程（当前 requestId={_current?.RequestId ?? "无"}），未执行停止", _current);
            if (_current?.Outcome != ScanOutcome.InProgress)
                return ResultLocked(true, false, true, "already_idle", "当前没有活动扫描会话，雷达正在持续扫描", _current);

            _current.Finish(ScanOutcome.Cancelled, reason, _time.GetLocalNow().DateTime);
            return ResultLocked(true, false, false, "cancelled", reason, _current);
        }
    }

    /// <summary>当前（或最近一次）会话快照。</summary>
    public ScanSnapshot Snapshot()
    {
        lock (_gate)
        {
            ExpireLocked();
            return SnapshotLocked(_current);
        }
    }

    /// <summary>本进程受理过的会话；未知返回 null。</summary>
    public ScanSnapshot? Find(string requestId)
    {
        lock (_gate)
        {
            ExpireLocked();
            return _byRequest.TryGetValue(requestId.Trim(), out var session) ? SnapshotLocked(session) : null;
        }
    }

    /// <summary>
    /// 判定雷达回调的一场点云；接纳时返回会话的 workflowId，否则返回 null。
    /// </summary>
    /// <param name="callbackTimestamp">回调入口取得的单调时间戳，早于指令的场一律不计。</param>
    public string? Offer(string camera, int finitePointCount, long callbackTimestamp)
        => Offer(camera, finitePointCount, callbackTimestamp, out _);

    public string? Offer(string camera, int finitePointCount, long callbackTimestamp, out StaticCaptureWindow? window)
    {
        lock (_gate)
        {
            window = null;
            ExpireLocked();
            var session = _current;
            if (session?.Outcome != ScanOutcome.InProgress || session.Captured.Contains(camera))
                return null;
            var elapsed = _time.GetElapsedTime(session.StartedTimestamp, callbackTimestamp).TotalSeconds;
            if (elapsed < 0 || (!session.EarlyStaticCapture && elapsed < _guardSeconds))
                return null;
            if (finitePointCount < _minPoints[camera])
                return null;
            if (session.RemainingDiscards[camera] > 0)
            {
                session.RemainingDiscards[camera]--;
                _logger.LogInformation("已丢弃跨越指令边界的完整场: camera={Camera}, 点数={Count}", camera, finitePointCount);
                return null;
            }

            session.Captured.Add(camera);
            window = session.StaticWindow;
            if (session.EarlyStaticCapture && elapsed < _guardSeconds)
                _logger.LogInformation("自动停稳采集保留交付保护窗内候选帧，须核对完整静止区间 workflowId={WorkflowId} segmentId={SegmentId} camera={Camera} elapsedSeconds={Elapsed} guardSeconds={Guard}",
                    session.RequestId, window!.SegmentId, camera, elapsed, _guardSeconds);
            if (session.Captured.Count == _cameras.Length)
            {
                session.Finish(ScanOutcome.Completed, null, _time.GetLocalNow().DateTime);
                _logger.LogInformation("扫描会话完成: workflowId={WorkflowId}", session.RequestId);
            }
            return session.RequestId;
        }
    }

    /// <summary>记录某台雷达的 PCD 已落盘。</summary>
    public void MarkPersisted(string workflowId, string camera)
    {
        lock (_gate)
        {
            if (_byRequest.TryGetValue(workflowId, out var session))
                session.Persisted.Add(camera);
        }
    }

    private void ExpireLocked()
    {
        var session = _current;
        if (session?.Outcome != ScanOutcome.InProgress ||
            _time.GetElapsedTime(session.StartedTimestamp).TotalSeconds < session.TimeoutSeconds)
            return;

        var missing = string.Join(", ", _cameras.Where(x => !session.Captured.Contains(x)));
        var error = $"单次扫描已达到 {session.TimeoutSeconds:0.###} 秒采集截止，未完成雷达: {missing}";
        session.Finish(ScanOutcome.TimedOut, error, _time.GetLocalNow().DateTime);
        _logger.LogError("扫描会话超时: workflowId={WorkflowId}, {Error}", session.RequestId, error);
    }

    private ScanCommandResult ResultLocked(
        bool success, bool started, bool idempotent, string code, string message, Session? session) =>
        new(success, started, idempotent, code, message, SnapshotLocked(session));

    private ScanSnapshot SnapshotLocked(Session? s) => new(
        s?.Outcome == ScanOutcome.InProgress ? ScanState.Scanning : ScanState.Idle,
        s?.Outcome ?? ScanOutcome.None,
        s?.RequestId,
        s?.SessionId,
        s?.StartedAt,
        s?.CompletedAt,
        _cameras,
        s is null ? [] : [.. _cameras.Where(s.Captured.Contains)],
        s is null ? [] : [.. _cameras.Where(s.Persisted.Contains)],
        s?.LastError,
        s?.TimeoutSeconds ?? _timeoutSeconds);

    private sealed class Session(
        string requestId, DateTime startedAt, long startedTimestamp, string[] cameras, int discards, double timeoutSeconds)
    {
        public string RequestId { get; } = requestId;
        public string SessionId { get; } = Guid.NewGuid().ToString("N");
        public DateTime StartedAt { get; } = startedAt;
        public long StartedTimestamp { get; } = startedTimestamp;
        public double TimeoutSeconds { get; } = timeoutSeconds;
        public bool ClosedByDeparture { get; set; }
        public bool EarlyStaticCapture { get; set; }
        public StaticCaptureWindow? StaticWindow { get; set; }
        public Dictionary<string, int> RemainingDiscards { get; } = cameras.ToDictionary(x => x, _ => discards);
        public HashSet<string> Captured { get; } = [];
        public HashSet<string> Persisted { get; } = [];
        public ScanOutcome Outcome { get; private set; } = ScanOutcome.InProgress;
        public DateTime? CompletedAt { get; private set; }
        public string? LastError { get; private set; }

        public void Finish(ScanOutcome outcome, string? error, DateTime at)
        {
            Outcome = outcome;
            LastError = error;
            CompletedAt = at;
        }
    }
}
