using CarPointCloudWebApi.Domain;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class PlateCaptureOrderTests
{
    private static DateTime At(double seconds) => new DateTime(2026, 11, 26).AddSeconds(seconds);
    private static PlateCaptureOrder Order(params (double Arrival, double Capture)[] anchors) =>
        new(anchors.Select(a => (At(a.Arrival), At(a.Capture))));

    [Fact]
    public void No_confirmed_trips_add_no_order_restriction()
    {
        Assert.Equal(PlateCaptureRelation.Compatible, Order().Compare(At(300), At(120)));
    }

    [Fact]
    public void A_confirmed_middle_trip_separates_older_unknowns_but_leaves_later_unknowns_in_competition()
    {
        var order = Order((75, 38));
        var candidate = At(49);
        Assert.Equal(PlateCaptureRelation.Impossible, order.Compare(At(20), candidate));
        Assert.Equal(PlateCaptureRelation.Compatible, order.Compare(At(90), candidate));
        Assert.Equal(PlateCaptureRelation.Compatible, order.Compare(At(100), candidate));
    }

    [Theory]
    [InlineData(60, 40, PlateCaptureRelation.Compatible)]
    [InlineData(60, 61, PlateCaptureRelation.Impossible)]
    [InlineData(81, 59, PlateCaptureRelation.Impossible)]
    [InlineData(81, 130, PlateCaptureRelation.Compatible)]
    [InlineData(81, 151, PlateCaptureRelation.Impossible)]
    [InlineData(200, 149, PlateCaptureRelation.Impossible)]
    [InlineData(200, 3700, PlateCaptureRelation.Compatible)]
    public void Checks_both_ends_without_assuming_all_partner_trips_have_radar_arrivals(
        double arrival, double capture, PlateCaptureRelation expected)
    {
        var order = Order((190, 150), (80, 60));
        Assert.Equal(expected, order.Compare(At(arrival), At(capture)));
    }

    [Theory]
    [InlineData(80.8, 115)]
    [InlineData(79, 110.9)]
    [InlineData(82, 110)]
    public void Same_second_cannot_establish_passage_order(double arrival, double capture)
    {
        Assert.Equal(PlateCaptureRelation.Uncertain, Order((80.1, 110.4)).Compare(At(arrival), At(capture)));
    }

    [Theory]
    [InlineData(99, 110)]
    [InlineData(101, 120.4)]
    [InlineData(80.9, 127)]
    public void Conflicting_or_tied_anchors_cannot_clear_unresolved_trips(double nextArrival, double nextCapture)
    {
        var order = Order((80, 120), (nextArrival, nextCapture));
        Assert.Equal(PlateCaptureRelation.Uncertain, order.Compare(At(70), At(118)));
        Assert.Equal(PlateCaptureRelation.Uncertain, order.Compare(At(140), At(200)));
    }

    [Theory]
    [InlineData("00001", "99999")]
    [InlineData("99999", "00001")]
    public void Reversing_enter_id_order_does_not_change_which_capture_is_possible(string oldId, string laterId)
    {
        var order = Order((20, 100));
        var candidates = new[] { (EnterId: laterId, Capture: At(110)), (EnterId: oldId, Capture: At(90)) };
        Assert.Equal(laterId, Assert.Single(candidates.Where(c =>
            order.Compare(At(35), c.Capture) == PlateCaptureRelation.Compatible)).EnterId);
    }
}
