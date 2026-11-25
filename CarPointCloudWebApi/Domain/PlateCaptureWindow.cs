namespace CarPointCloudWebApi.Domain;

/// <summary>同一摄像头的抓拍相对实际到磅时间的允许范围。</summary>
public readonly record struct PlateCaptureWindow(TimeSpan FromArrival, TimeSpan ToArrival)
{
    public static PlateCaptureWindow FromHistory(IReadOnlyList<TimeSpan> offsets,
        int leadSeconds, int windowSeconds, int toleranceSeconds)
    {
        if (offsets.Count == 0)
            // 首查约 10 秒，留 5 秒余量。后续重查仍可匹配迟发布的抓拍，
            // 未校准的绝对时间窗口不能随重试伸进下一辆车。
            return new(TimeSpan.FromSeconds(-leadSeconds), TimeSpan.FromSeconds(Math.Min(windowSeconds, 15)));

        var ordered = offsets.Order().ToArray();
        var middle = ordered.Length / 2;
        var center = ordered.Length % 2 == 0
            ? ordered[middle - 1] + (ordered[middle] - ordered[middle - 1]) / 2
            : ordered[middle];
        var tolerance = TimeSpan.FromSeconds(toleranceSeconds);
        if (ordered.Length <= 2)
            return new(center - tolerance, center + tolerance);

        // 最近五条样本中保留中间三条的范围，单个极端样本不会把窗口撑开。
        var trim = ordered.Length >= 5 ? 1 : 0;
        var from = center - TimeSpan.FromSeconds(Math.Clamp(
            (center - ordered[trim]).TotalSeconds, 1, toleranceSeconds));
        var to = center + TimeSpan.FromSeconds(Math.Clamp(
            (ordered[^(trim + 1)] - center).TotalSeconds, 1, toleranceSeconds));
        return new(from, to);
    }

    public bool Includes(DateTime arrivalTime, DateTime captureTime)
    {
        // 乙方时间精确到秒，同一个自然秒内的毫秒差不产生归属差异。
        var offset = captureTime - arrivalTime.AddTicks(-(arrivalTime.Ticks % TimeSpan.TicksPerSecond));
        return offset >= FromArrival && offset <= ToArrival;
    }

    public LicensePlateRecord? MatchUnique(IReadOnlyList<LicensePlateRecord> candidates,
        DateTime arrivalTime, IReadOnlyList<DateTime> neighboringArrivals, DateTime? latestCaptureTime,
        Func<LicensePlateRecord, bool> isAlreadyClaimed, Action<LicensePlateRecord, string>? observe = null)
    {
        LicensePlateRecord? match = null;
        var window = this;
        foreach (var candidate in candidates.OrderBy(item => item.CaptureTime))
        {
            var exclusion = candidate.CaptureTime is not { } captured || string.IsNullOrWhiteSpace(candidate.EnterId) ? "missing_identity"
                : captured > latestCaptureTime ? "later_than_latest_capture"
                : !Includes(arrivalTime, captured) ? "outside_window"
                : neighboringArrivals.Any(neighbor => window.Includes(neighbor, captured)) ? "neighbor_window"
                : isAlreadyClaimed(candidate) ? "already_claimed"
                : null;
            if (exclusion is not null)
            {
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
        return match;
    }
}
