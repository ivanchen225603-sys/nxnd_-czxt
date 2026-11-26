using Xunit;

namespace CarPointCloudWebApi.Tests;

public class PlateCaptureOwnershipTests
{
    private static readonly DateTime Start = new(2026, 11, 26, 10, 0, 0);

    private static PlateCaptureWindow Bounds(int minimum, int maximum) =>
        PlateCaptureWindow.FromVerifiedBounds(TimeSpan.FromSeconds(minimum), TimeSpan.FromSeconds(maximum), leadSeconds: 5);

    private static LicensePlateRecord Capture(string enterId, DateTime captured, DateTime firstSeen) => new()
    {
        CameraIdentifier = "CAM_002", CameraId = "CAM_002", LicensePlate = "宁A12345",
        EnterId = enterId, CaptureTime = captured, FirstSeenAt = firstSeen
    };

    [Fact]
    public void Field_CH2_arrivals_are_distinguishable_under_an_explicit_test_bound()
    {
        var previous = Start.AddMinutes(5).AddMilliseconds(53912);
        var arrival = Start.AddMinutes(38).AddMilliseconds(45765);
        var capture = Capture("982317", Start.AddMinutes(38).AddSeconds(42), arrival.AddSeconds(10));

        // 到磅和乙方抓拍采用现场时刻；范围、首次收到时刻和车牌为测试输入。
        var match = Bounds(-5, -2).MatchUnique([capture], arrival, [new(previous, false)], null, _ => false);

        Assert.Same(capture, match);
    }

    [Theory]
    [InlineData(3, 16)]
    [InlineData(3, 21)]
    [InlineData(20, 16)]
    [InlineData(20, 21)]
    public void Pending_prior_trip_retains_its_possible_late_capture(int headway, int priorCaptureOffset)
    {
        var arrival = Start.AddSeconds(headway);
        var priorCapture = Capture("prior", Start.AddSeconds(priorCaptureOffset), arrival.AddSeconds(25));
        var decisions = new List<string>();

        var match = Bounds(-5, 21).MatchUnique([priorCapture], arrival, [new(Start, false)], null,
            _ => false, (_, reason) => decisions.Add(reason));

        Assert.Null(match);
        Assert.Contains("prior_passage_capture", decisions);
    }

    [Fact]
    public void Equidistant_capture_cannot_be_claimed_ahead_of_the_next_trip()
    {
        var capture = Capture("between", Start.AddSeconds(10), Start.AddSeconds(25));
        var match = Bounds(-10, 10).MatchUnique([capture], Start, [new(Start.AddSeconds(20), false)], null, _ => false);

        Assert.Null(match);
    }

    [Theory]
    [InlineData(-5, 4, 8)]
    [InlineData(5, 8, 2)]
    public void Shared_candidate_prevents_claiming_another_candidate_that_only_fits_this_trip(
        int neighborOffset, int sharedOffset, int exclusiveOffset)
    {
        var seen = Start.AddSeconds(12);
        var shared = Capture("shared", Start.AddSeconds(sharedOffset), seen);
        var exclusive = Capture("exclusive", Start.AddSeconds(exclusiveOffset), seen);
        var neighbor = new PlateArrivalNeighbor(Start.AddSeconds(neighborOffset), false);

        Assert.Same(exclusive, Bounds(0, 10).MatchUnique([exclusive], Start, [neighbor], null, _ => false));
        Assert.Null(Bounds(0, 10).MatchUnique([exclusive, shared], Start, [neighbor], null, _ => false));
    }

    [Theory]
    [InlineData(-37, 3)]
    [InlineData(42, 3)]
    [InlineData(-37, 20)]
    [InlineData(42, 20)]
    public void Verified_clock_offset_allows_a_unique_trip_even_with_a_nearby_unidentified_vehicle(int offset, int headway)
    {
        var capture = Capture("current", Start.AddSeconds(offset), Start.AddSeconds(10));
        var window = Bounds(offset - 1, offset + 1);
        var match = window.MatchUnique([capture], Start, [new(Start.AddSeconds(-headway), false)], null, _ => false);

        Assert.Same(capture, match);
    }

    [Fact]
    public void Conflicting_identity_in_this_window_keeps_the_other_grab_unconfirmed()
    {
        var seen = Start.AddSeconds(10);
        var conflict = Capture("disputed", Start.AddSeconds(1), seen);
        conflict.ConflictingIdentity = true;
        var other = Capture("other", Start.AddSeconds(2), seen);
        var window = Bounds(0, 6);
        Assert.Null(window.MatchUnique([conflict, other], Start, [], null, _ => false));
        conflict.CaptureTime = Start.AddMinutes(-30);
        Assert.Same(other, window.MatchUnique([conflict, other], Start, [], null, _ => false));
    }

    [Fact]
    public void Returning_plate_uses_its_new_enter_id_even_if_the_snapshot_also_contains_the_claimed_id()
    {
        var seen = Start.AddSeconds(10);
        var old = Capture("old", Start.AddSeconds(3), seen);
        var current = Capture("new", Start.AddSeconds(5), seen);
        var previous = new PlateArrivalNeighbor(Start.AddSeconds(-3), true);
        var window = Bounds(0, 6);
        bool Claimed(LicensePlateRecord row) => row.EnterId == old.EnterId;

        Assert.Null(window.MatchUnique([old], Start, [previous], null, Claimed));
        Assert.Same(current, window.MatchUnique([current], Start, [previous], null, Claimed));
        Assert.Same(current, window.MatchUnique([old, current], Start, [previous], null, Claimed));
    }
}
