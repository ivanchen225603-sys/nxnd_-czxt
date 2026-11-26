namespace CarPointCloudWebApi.Domain;

// 内部时间保留 UTC 的亚秒精度，乙方业务时间格式由查询映射负责。
public sealed record StandstillObservation(string NodeId, string ChannelId, string PassageId,
    string SegmentId, long Sequence, DateTimeOffset StoppedAt, DateTimeOffset LastStillAt,
    DateTimeOffset? BrokenAt = null, string? Reason = null);

public sealed record ScanInterval(string CameraIdentifier, DateTimeOffset CallbackUtc,
    DateTimeOffset? ScanStartUtc, DateTimeOffset? ScanEndUtc);
public sealed record StaticPairEvidence(string PairId, string? SegmentId, ScanInterval Primary, ScanInterval Secondary);
public sealed record StaticDecision(string Status, string? Reason);

/// <summary>一次受理过的自动双路采集。证据按停稳段和原始配对冻结，接收顺序不决定归属。</summary>
public sealed class StaticCapture
{
    public required string PrimaryCamera { get; init; }
    public string? SegmentId { get; init; }
    public required DateTimeOffset StoppedAt { get; init; }
    public required DateTimeOffset CaptureUntil { get; init; }
    [System.Text.Json.Serialization.JsonIgnore]
    public DateTimeOffset EvidenceDeadline => CaptureUntil.AddSeconds(30);
    public StandstillObservation? Observation { get; private set; }
    public StaticPairEvidence? Pair { get; private set; }
    public StaticDecision Decision { get; private set; } = new("pending", "awaiting_evidence");

    // DM8 内的证据及判定在同一行锁下读改写。
    [System.Text.Json.Serialization.JsonConstructor]
    public StaticCapture(StandstillObservation? observation = null, StaticPairEvidence? pair = null,
        StaticDecision? decision = null)
    {
        Observation = observation;
        Pair = pair;
        if (decision is not null) Decision = decision;
    }

    public void Observe(StandstillObservation observation, DateTimeOffset now)
    {
        if (observation.SegmentId != SegmentId || observation.StoppedAt != StoppedAt)
            throw new LoadRadarException(409, "停稳段与原派发不符");
        if (observation.Sequence < 1 || observation.LastStillAt < StoppedAt ||
            observation.LastStillAt > now.AddSeconds(2) || observation.BrokenAt < observation.LastStillAt ||
            observation.BrokenAt > now.AddSeconds(2) || observation.Reason?.Length > 128)
            throw new LoadRadarException(400, "静止时序无效");
        var sequence = Observation?.Sequence ?? 0;
        if (observation.Sequence < sequence || observation == Observation) return;
        if (observation.Sequence != sequence + 1 || Observation?.BrokenAt is not null ||
            observation.LastStillAt < Observation?.LastStillAt)
            throw new LoadRadarException(409, "静止证据序号或连续区间冲突");
        Observation = observation;
        Evaluate(now);
    }

    public void Offer(StaticPairEvidence pair, DateTimeOffset now)
    {
        if (pair.SegmentId != SegmentId || Pair is not null && Pair != pair)
            throw new LoadRadarException(409, "扫描证据与原车次、停稳段或配对不一致");
        if (string.IsNullOrWhiteSpace(pair.PairId) || pair.PairId.Length > 64 || pair.Primary is null ||
            pair.Secondary is null || pair.Primary.CameraIdentifier != PrimaryCamera ||
            !PartnerContract.IsCamera(pair.Secondary.CameraIdentifier) || pair.Secondary.CameraIdentifier == PrimaryCamera)
            throw new LoadRadarException(400, "双雷达配对或点位无效");
        Pair = pair;
        Evaluate(now);
    }

    public void Expire(DateTimeOffset now) => Evaluate(now);

    private void Evaluate(DateTimeOffset now)
    {
        if (Decision.Status != "pending") return;
        if (now >= EvidenceDeadline)
            Decision = new("insufficient", "evidence_timeout");
        else if (Pair is { } pair)
        {
            var frames = new[] { pair.Primary, pair.Secondary };
            if (SegmentId is null || frames.Any(f => f.ScanStartUtc is null || f.ScanEndUtc is null))
                Decision = new("insufficient", "timing_unknown");
            else if (frames.Any(f => f.ScanStartUtc >= f.ScanEndUtc || f.ScanStartUtc > f.CallbackUtc ||
                f.CallbackUtc > now.AddSeconds(2)))
                Decision = new("insufficient", "invalid_scan_bounds");
            else if (frames.Any(f => f.ScanStartUtc < StoppedAt || f.ScanEndUtc > CaptureUntil || f.CallbackUtc >= CaptureUntil))
                Decision = new("rejected", "outside_capture_window");
            else if (Observation is { } observed && frames.All(f => f.ScanEndUtc <= observed.LastStillAt))
                Decision = new("confirmed", null);
            else if (Observation?.BrokenAt is not null)
                Decision = new(Observation.Reason is "movement" or "departure" ? "rejected" : "insufficient",
                    Observation.Reason ?? "stillness_interrupted");
        }
    }
}
