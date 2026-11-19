using Xunit;

namespace CarPointCloudWebApi.Tests;

public class BusinessClockTests
{
    [Fact]
    public void Now_UsesChinaStandardTime_IndependentlyOfHostLocalTimeZone()
    {
        var earliestExpected = DateTime.UtcNow.AddHours(8);

        var actual = BusinessClock.Now;

        var latestExpected = DateTime.UtcNow.AddHours(8);
        Assert.InRange(actual, earliestExpected, latestExpected);
        Assert.Equal(DateTimeKind.Unspecified, actual.Kind);
    }
}
