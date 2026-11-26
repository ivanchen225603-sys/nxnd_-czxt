using System.Text.Json;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class StaticCaptureTests
{
    private static readonly DateTimeOffset Stop = DateTimeOffset.Parse("2026-11-26T08:00:00Z");
    private static StaticCapture NewCapture() => new() { StoppedAt = Stop, CaptureUntil = Stop.AddSeconds(10),
        PrimaryCamera = "CAM_004", SegmentId = "seg1" };

    private static StandstillObservation Observation(int seq, double last, double? broken = null) =>
        new("node", "CH4", "passage", "seg1", seq, Stop, Stop.AddSeconds(last),
            broken.HasValue ? Stop.AddSeconds(broken.Value) : null, broken.HasValue ? "movement" : null);

    private static StaticPairEvidence Pair(double firstStart = 3) => new("pair", "seg1",
        new("CAM_004", Stop.AddSeconds(7.8), Stop.AddSeconds(firstStart), Stop.AddSeconds(7.9)),
        new("CAM_005", Stop.AddSeconds(8), Stop.AddSeconds(4), Stop.AddSeconds(8.1)));

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void Only_a_continuous_prefix_covering_both_whole_frames_can_confirm(bool movedAtSeven)
    {
        var capture = NewCapture();
        capture.Observe(Observation(1, 6), Stop.AddSeconds(6.1));
        capture.Offer(Pair(), Stop.AddSeconds(8.2));
        Assert.Equal("pending", capture.Decision.Status);
        // 进程重启与回调先到都不能提前放行。
        capture = JsonSerializer.Deserialize<StaticCapture>(JsonSerializer.Serialize(capture))!;
        var final = movedAtSeven ? Observation(2, 6.8, 7) : Observation(2, 9, 10);
        capture.Observe(final, Stop.AddSeconds(10.5));
        capture.Observe(final, Stop.AddSeconds(11));
        capture.Observe(Observation(1, 6), Stop.AddSeconds(11));
        Assert.Equal(movedAtSeven ? "rejected" : "confirmed", capture.Decision.Status);
    }

    [Fact]
    public void Missing_sequences_restarts_and_earlier_scans_cannot_bridge_a_gap()
    {
        var capture = NewCapture();
        Assert.Throws<LoadRadarException>(() => capture.Observe(Observation(2, 9), Stop.AddSeconds(9)));
        Assert.Throws<LoadRadarException>(() => capture.Observe(Observation(1, 9) with { SegmentId = "seg2" }, Stop.AddSeconds(9)));
        capture.Observe(Observation(1, 9), Stop.AddSeconds(9));
        capture.Offer(Pair(-0.1), Stop.AddSeconds(9));
        Assert.Equal("rejected", capture.Decision.Status);
        Assert.Throws<LoadRadarException>(() => capture.Offer(Pair() with { PairId = "other" }, Stop.AddSeconds(9)));
    }

    [Fact]
    public void Expiry_seals_an_absent_pair_and_a_late_observation_cannot_confirm_it()
    {
        var capture = NewCapture();
        capture.Expire(capture.EvidenceDeadline);
        Assert.Equal(new StaticDecision("insufficient", "evidence_timeout"), capture.Decision);
        capture.Observe(Observation(1, 9), capture.EvidenceDeadline.AddSeconds(1));
        capture.Offer(Pair(), capture.EvidenceDeadline.AddSeconds(2));
        Assert.Equal("insufficient", capture.Decision.Status);
    }

    [Fact]
    public void Unknown_timing_or_missing_observations_never_confirm()
    {
        var capture = NewCapture();
        var pair = Pair();
        capture.Offer(pair with { Primary = pair.Primary with { ScanStartUtc = null } }, Stop.AddSeconds(8.2));
        Assert.Equal("insufficient", capture.Decision.Status);
        capture = NewCapture();
        capture.Offer(pair, Stop.AddSeconds(8.2));
        capture.Offer(pair, Stop.AddSeconds(40));
        Assert.Equal("insufficient", capture.Decision.Status);
    }
}
