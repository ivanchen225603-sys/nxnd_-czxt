namespace CarPointCloudWebApi.Infrastructure;

/// <summary>现场逐车独立核实的历史配对，仅供本通道的识牌时差校准。</summary>
public sealed class VerifiedPlateCalibration
{
    private sealed record Sample(string WorkflowId, string EnterId, DateTime Arrival,
        DateTime Capture, DateTime VerifiedAt);

    private readonly Dictionary<string, List<Sample>> _byCamera = new(StringComparer.Ordinal);

    public static VerifiedPlateCalibration Read(IConfiguration configuration, ILogger? logger = null)
    {
        var result = new VerifiedPlateCalibration();
        foreach (var row in configuration.GetSection("PlateAcquisition:VerifiedCalibrationSamples").GetChildren())
        {
            var camera = row["CameraIdentifier"];
            var workflow = row["WorkflowId"]?.Trim();
            var enterId = row["EnterId"]?.Trim();
            if (!PartnerContract.IsCamera(camera) || string.IsNullOrWhiteSpace(workflow) ||
                string.IsNullOrWhiteSpace(enterId) || !PartnerContract.TryParseTime(row["ArrivalTime"], out var arrival) ||
                !PartnerContract.TryParseTime(row["CaptureTime"], out var capture) ||
                !PartnerContract.TryParseTime(row["VerifiedAt"], out var verifiedAt) || verifiedAt < arrival)
            {
                logger?.LogWarning("人工识牌校准样本 {Key} 的身份或时间字段无效，已忽略", row.Path);
                continue;
            }
            if (!result._byCamera.TryGetValue(camera!, out var samples))
                result._byCamera[camera!] = samples = [];
            samples.Add(new(workflow, enterId, arrival, capture, verifiedAt));
        }
        return result;
    }

    /// <summary>取本车到磅前已核验的最近五个独立车次，交给 PlateCaptureWindow.FromHistory 检查一致性。</summary>
    public IReadOnlyList<TimeSpan> OffsetsBefore(string camera, DateTime arrival, string currentWorkflowId)
    {
        if (!_byCamera.TryGetValue(camera, out var samples)) return [];
        return IndependentBefore(samples, arrival).Where(s => s.WorkflowId != currentWorkflowId)
            .OrderByDescending(s => s.Arrival).ThenBy(s => s.WorkflowId, StringComparer.Ordinal)
            .Take(5).Select(s => s.Capture - s.Arrival).ToArray();
    }

    /// <summary>现场核验为其他车的历史抓拍，即使业务表尚无成功绑定记录，也不能被本车占用。</summary>
    public bool IsOtherVerifiedCapture(string camera, string currentWorkflowId, string? enterId, DateTime asOf) =>
        enterId is not null && _byCamera.TryGetValue(camera, out var samples) &&
        samples.Any(s => s.VerifiedAt < asOf && s.EnterId == enterId && s.WorkflowId != currentWorkflowId);

    /// <summary>仅用唯一且到磅时刻相符的人工配对确认前车；证据冲突时仍视为归属未明。</summary>
    public bool IsVerifiedPriorWorkflow(string camera, string workflowId, DateTime actualArrival, DateTime asOf)
        => PriorCapture(camera, workflowId, actualArrival, asOf).HasValue;

    public DateTime? PriorCapture(string camera, string workflowId, DateTime actualArrival, DateTime asOf)
    {
        if (actualArrival >= asOf || !_byCamera.TryGetValue(camera, out var samples)) return null;
        var arrivalSecond = actualArrival.AddTicks(-(actualArrival.Ticks % TimeSpan.TicksPerSecond));
        return IndependentBefore(samples, asOf).FirstOrDefault(s => s.WorkflowId == workflowId && s.Arrival == arrivalSecond)?.Capture;
    }

    private static IEnumerable<Sample> IndependentBefore(List<Sample> samples, DateTime asOf)
    {
        var rows = samples.Where(s => s.VerifiedAt < asOf)
            .DistinctBy(s => (s.WorkflowId, s.EnterId, s.Arrival, s.Capture)).ToArray();
        var duplicateTrips = rows.GroupBy(s => s.WorkflowId).Where(g => g.Count() > 1)
            .Select(g => g.Key).ToHashSet(StringComparer.Ordinal);
        var duplicateCaptures = rows.GroupBy(s => s.EnterId).Where(g => g.Count() > 1)
            .Select(g => g.Key).ToHashSet(StringComparer.Ordinal);
        return rows.Where(s => !duplicateTrips.Contains(s.WorkflowId) && !duplicateCaptures.Contains(s.EnterId));
    }
}
