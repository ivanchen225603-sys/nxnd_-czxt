namespace CarPointCloudWebApi.Domain;

public readonly record struct PlateArrivalNeighbor(DateTime Arrival, bool Identified, DateTime? ConfirmedCapture = null);

/// <summary>同点位的乙方抓拍时刻相对我方到磅时刻的偏移，包含时钟差和触发延迟。</summary>
public readonly record struct PlateCaptureWindow(TimeSpan FromArrival, TimeSpan ToArrival)
{
    public bool IsCalibrated { get; private init; }
    public bool HasVerifiedBounds { get; private init; }
    public bool HasTimeReference => IsCalibrated || HasVerifiedBounds;
    private TimeSpan UnresolvedToArrival { get; init; }
    private TimeSpan AllowedLocalLead { get; init; }
    // 乙方返回最近一小时的抓拍。未知偏移时用本地来车间隔划定可能冲突的车次。
    public static readonly TimeSpan SnapshotHorizon = TimeSpan.FromHours(1);

    // 现场确认的范围已经包含时钟偏差、触发迟延和秒级误差，直接用于各车的归属判定。
    public static PlateCaptureWindow FromVerifiedBounds(TimeSpan minimum, TimeSpan maximum, int leadSeconds) =>
        new(minimum, maximum)
        {
            HasVerifiedBounds = true, AllowedLocalLead = TimeSpan.FromSeconds(leadSeconds),
            UnresolvedToArrival = maximum
        };

    public TimeSpan NeighborLookback =>
        HasTimeReference ? UnresolvedToArrival - FromArrival + TimeSpan.FromSeconds(1) : SnapshotHorizon;

    public static PlateCaptureWindow FromHistory(IReadOnlyList<TimeSpan> offsets,
        int leadSeconds, int windowSeconds, int toleranceSeconds)
    {
        var lead = TimeSpan.FromSeconds(leadSeconds);
        var cold = new PlateCaptureWindow(-lead, TimeSpan.FromSeconds(Math.Min(windowSeconds, 15)))
            { AllowedLocalLead = lead };
        var ordered = offsets.Take(5).Order().ToArray();
        // 找唯一最大的相容样本组。少于三个不同车次或两组并列时，单次误绑不能改变通道窗口。
        var tolerance = TimeSpan.FromSeconds(toleranceSeconds);
        var bestStart = 0;
        var bestCount = 0;
        var tied = false;
        for (var start = 0; start < ordered.Length; start++)
        {
            var end = start;
            while (end < ordered.Length && ordered[end] - ordered[start] <= tolerance)
                end++;
            var count = end - start;
            if (count > bestCount)
                (bestStart, bestCount, tied) = (start, count, false);
            else if (count == bestCount)
                tied = true;
        }
        if (bestCount < 3 || tied) return cold;

        var group = ordered.AsSpan(bestStart, bestCount);
        var center = group[(bestCount - 1) / 2] + (group[bestCount / 2] - group[(bestCount - 1) / 2]) / 2;
        var from = center - TimeSpan.FromSeconds(Math.Max(1, (center - group[0]).TotalSeconds));
        var to = center + TimeSpan.FromSeconds(Math.Max(1, (group[^1] - center).TotalSeconds));
        return new(from, to)
        {
            IsCalibrated = true, AllowedLocalLead = lead,
            UnresolvedToArrival = to + TimeSpan.FromSeconds(windowSeconds)
        };
    }

    private static TimeSpan Offset(DateTime arrival, DateTime capture) =>
        capture - arrival.AddTicks(-(arrival.Ticks % TimeSpan.TicksPerSecond));

    public bool Includes(DateTime arrival, DateTime capture)
    {
        var offset = Offset(arrival, capture);
        return offset >= FromArrival && offset <= ToArrival;
    }

    private bool CouldBeUnresolvedPrior(PlateArrivalNeighbor neighbor, DateTime arrival, DateTime capture)
    {
        if (neighbor.Identified || neighbor.Arrival >= arrival) return false;
        if (!HasTimeReference) return arrival - neighbor.Arrival <= SnapshotHorizon;
        // 校准后用同一偏移系判定，给早先未识别车保留配置的迟抓拍余量。
        var offset = Offset(neighbor.Arrival, capture);
        return offset >= FromArrival && offset <= UnresolvedToArrival;
    }

    public LicensePlateRecord? MatchUnique(IReadOnlyList<LicensePlateRecord> candidates,
        DateTime arrivalTime, IReadOnlyList<PlateArrivalNeighbor> neighbors, DateTime? latestCaptureTime,
        Func<LicensePlateRecord, bool> isAlreadyClaimed, Action<LicensePlateRecord, string>? observe = null)
    {
        LicensePlateRecord? match = null;
        var competingPassage = false;
        var window = this;
        var order = new PlateCaptureOrder(neighbors.Where(n => n.Identified && n.ConfirmedCapture.HasValue)
            .Select(n => (n.Arrival, n.ConfirmedCapture!.Value)));
        foreach (var candidate in candidates.OrderBy(item => item.CaptureTime))
        {
            var exclusion = candidate.CaptureTime is not { } captured || string.IsNullOrWhiteSpace(candidate.EnterId) ? "missing_identity"
                : candidate.FirstSeenAt < arrivalTime - AllowedLocalLead ? "seen_before_arrival"
                : captured > latestCaptureTime ? "later_than_latest_capture"
                : !Includes(arrivalTime, captured) ? "outside_window"
                : isAlreadyClaimed(candidate) ? "already_claimed"
                : candidate.ConflictingIdentity ? "conflicting_enter_id"
                : order.Compare(arrivalTime, captured) switch
                {
                    PlateCaptureRelation.Impossible => "outside_passage_order",
                    PlateCaptureRelation.Uncertain => "ambiguous_passage_order",
                    _ => NeighborExclusion(candidate, captured)
                };
            if (exclusion is not null)
            {
                // 在本车窗口内但也可能属于邻车的抓拍，仍是本车的第二个可能身份。
                // 即使响应里另有一个仅适配本车的 enterID，也不能直接认领它。
                if (exclusion is "prior_passage_capture" or "neighbor_window" or "ambiguous_passage_order" or "conflicting_enter_id")
                    competingPassage = true;
                observe?.Invoke(candidate, exclusion);
                continue;
            }

            if (match is null)
            {
                match = candidate;
                observe?.Invoke(candidate, "eligible");
            }
            else if (match.EnterId != candidate.EnterId || match.LicensePlate != candidate.LicensePlate)
            {
                observe?.Invoke(candidate, "ambiguous");
                return null;
            }
            else
                observe?.Invoke(candidate, "duplicate");
        }
        return competingPassage ? null : match;

        string? NeighborExclusion(LicensePlateRecord candidate, DateTime captured)
        {
            foreach (var neighbor in neighbors)
            {
                // 每辆实车只有一个 enterID。已确认车次自有其抓拍，其余车次允许被雷达漏登。
                if (neighbor.Identified || candidate.FirstSeenAt < neighbor.Arrival - window.AllowedLocalLead ||
                    order.Compare(neighbor.Arrival, captured) == PlateCaptureRelation.Impossible) continue;
                if (window.CouldBeUnresolvedPrior(neighbor, arrivalTime, captured)) return "prior_passage_capture";
                if (window.Includes(neighbor.Arrival, captured)) return "neighbor_window";
            }
            return null;
        }
    }
}
