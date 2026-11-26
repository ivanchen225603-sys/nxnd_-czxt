using System.Text.Json;
using CarPointCloudWebApi.Domain;
using CarPointCloudWebApi.Infrastructure;
using Microsoft.Extensions.Configuration;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class VerifiedPlateCalibrationTests
{
    private static readonly DateTime Start = new(2026, 11, 26, 8, 0, 0);
    private sealed record Row(string CameraIdentifier, string WorkflowId, string EnterId,
        string ArrivalTime, string CaptureTime, string VerifiedAt);

    private static Row Sample(int index, int offset, string camera = "CAM_A")
    {
        var arrival = Start.AddMinutes(index);
        return new(camera, $"workflow-{index}", $"{index:D4}", PartnerContract.FormatTime(arrival)!,
            PartnerContract.FormatTime(arrival.AddSeconds(offset))!, PartnerContract.FormatTime(arrival.AddSeconds(15))!);
    }

    private static VerifiedPlateCalibration Read(params Row[] samples)
    {
        using var stream = new MemoryStream(JsonSerializer.SerializeToUtf8Bytes(
            new { PlateAcquisition = new { VerifiedCalibrationSamples = samples } }));
        return VerifiedPlateCalibration.Read(new ConfigurationBuilder().AddJsonStream(stream).Build());
    }

    [Theory]
    [InlineData(-37)]
    [InlineData(42)]
    public void Consistent_verified_trips_calibrate_only_their_own_camera(int offset)
    {
        var samples = Read(Sample(1, offset), Sample(2, offset), Sample(3, offset),
            Sample(4, offset + 90), Sample(5, 8, "CAM_B"), Sample(6, 8, "CAM_B"));
        var arrival = Start.AddMinutes(4).AddSeconds(16);
        var history = samples.OffsetsBefore("CAM_A", arrival, "new-trip");
        var window = PlateCaptureWindow.FromHistory(history, 5, 30, 5);
        Assert.True(window.IsCalibrated);
        Assert.True(window.Includes(arrival, arrival.AddSeconds(offset)));
        Assert.False(PlateCaptureWindow.FromHistory(
            samples.OffsetsBefore("CAM_B", Start.AddHours(1), "new-trip"), 5, 30, 5).IsCalibrated);
    }

    [Fact]
    public void Identical_trip_counts_once_and_conflicts_in_either_identity_exclude_both_rows()
    {
        var one = Sample(1, 8);
        var two = Sample(2, 8);
        var three = Sample(3, 8);
        var samples = Read(one, one with { EnterId = " " + one.EnterId },
            two, Sample(4, 8) with { WorkflowId = two.WorkflowId },
            three, Sample(5, 8) with { EnterId = three.EnterId });
        Assert.Equal([TimeSpan.FromSeconds(8)], samples.OffsetsBefore("CAM_A", Start.AddHours(1), "new-trip"));
    }

    [Fact]
    public void Only_prior_verified_local_trips_count_even_if_the_partner_clock_is_ahead()
    {
        var cutoff = Start.AddMinutes(5);
        var samples = Read(Sample(1, 7) with { WorkflowId = "current" },
            Sample(2, 7) with { VerifiedAt = PartnerContract.FormatTime(cutoff)! },
            Sample(3, 7) with { VerifiedAt = PartnerContract.FormatTime(cutoff.AddSeconds(1))! },
            Sample(4, 120), Sample(5, 7), Sample(6, 7));
        Assert.Equal([TimeSpan.FromSeconds(120)], samples.OffsetsBefore("CAM_A", cutoff, "current"));
    }

    [Fact]
    public void Invalid_times_are_ignored_and_newest_five_valid_trips_are_selected()
    {
        var rows = Enumerable.Range(1, 7).Select(i => Sample(i, i)).ToList();
        rows.Add(Sample(8, 8) with { CaptureTime = "2026-11-26T08:08:08Z" });
        rows.Add(Sample(9, 9) with { VerifiedAt = PartnerContract.FormatTime(Start)! });
        rows.Add(Sample(10, 10) with { WorkflowId = "" });
        var samples = Read([.. rows.AsEnumerable().Reverse()]);
        Assert.Equal(new[] { 7, 6, 5, 4, 3 },
            samples.OffsetsBefore("CAM_A", Start.AddHours(1), "new-trip").Select(s => (int)s.TotalSeconds));
        Assert.Empty(samples.OffsetsBefore("CAM_C", Start.AddHours(1), "new-trip"));
    }

    [Fact]
    public void Verified_capture_is_reserved_for_its_trip_and_known_prior_arrival_must_match()
    {
        var row = Sample(1, 8);
        var arrival = Start.AddMinutes(1);
        var asOf = Start.AddMinutes(2);
        var samples = Read(row, row);
        Assert.True(samples.IsOtherVerifiedCapture("CAM_A", "new-trip", row.EnterId, asOf));
        Assert.False(samples.IsOtherVerifiedCapture("CAM_A", row.WorkflowId, row.EnterId, asOf));
        Assert.False(samples.IsOtherVerifiedCapture("CAM_B", "new-trip", row.EnterId, asOf));
        Assert.False(samples.IsOtherVerifiedCapture("CAM_A", "new-trip", row.EnterId, arrival.AddSeconds(15)));
        Assert.True(samples.IsVerifiedPriorWorkflow("CAM_A", row.WorkflowId, arrival.AddMilliseconds(800), asOf));
        Assert.False(samples.IsVerifiedPriorWorkflow("CAM_A", row.WorkflowId, arrival.AddSeconds(1), asOf));
        Assert.False(samples.IsVerifiedPriorWorkflow("CAM_A", row.WorkflowId, arrival, arrival.AddSeconds(15)));
        Assert.False(samples.IsVerifiedPriorWorkflow("CAM_B", row.WorkflowId, arrival, asOf));
    }

    [Fact]
    public void Contradictory_observations_reserve_grabs_without_releasing_either_prior_passage()
    {
        var a = Sample(1, -37);
        var b = Sample(2, -37) with { EnterId = a.EnterId };
        var c = Sample(3, -37);
        var d = Sample(4, -37) with { WorkflowId = c.WorkflowId };
        var samples = Read(a, b, c, d);
        var asOf = Start.AddHours(1);
        Assert.True(samples.IsOtherVerifiedCapture("CAM_A", a.WorkflowId, a.EnterId, asOf));
        Assert.True(samples.IsOtherVerifiedCapture("CAM_A", "new-trip", d.EnterId, asOf));
        Assert.False(samples.IsVerifiedPriorWorkflow("CAM_A", a.WorkflowId, Start.AddMinutes(1), asOf));
        Assert.False(samples.IsVerifiedPriorWorkflow("CAM_A", b.WorkflowId, Start.AddMinutes(2), asOf));
        Assert.False(samples.IsVerifiedPriorWorkflow("CAM_A", c.WorkflowId, Start.AddMinutes(3), asOf));
    }
}
