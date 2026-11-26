using Microsoft.Extensions.Configuration;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public class VerifiedPlateOffsetBoundsTests
{
    [Fact]
    public void Each_camera_uses_only_its_own_exact_key()
    {
        var values = new Dictionary<string, string?>
        {
            ["PlateAcquisition:VerifiedCaptureOffsetBounds:CAM_002:MinimumSeconds"] = "-38.5",
            ["PlateAcquisition:VerifiedCaptureOffsetBounds:CAM_002:MaximumSeconds"] = "-34",
            ["PlateAcquisition:VerifiedCaptureOffsetBounds:CAM_003:MinimumSeconds"] = "41",
            ["PlateAcquisition:VerifiedCaptureOffsetBounds:CAM_003:MaximumSeconds"] = "44"
        };
        var bounds = VerifiedPlateOffsetBounds.Read(new ConfigurationBuilder().AddInMemoryCollection(values).Build());

        Assert.Equal((TimeSpan.FromSeconds(-38.5), TimeSpan.FromSeconds(-34)), bounds.ForCamera("CAM_002"));
        Assert.Equal((TimeSpan.FromSeconds(41), TimeSpan.FromSeconds(44)), bounds.ForCamera("CAM_003"));
        Assert.Null(bounds.ForCamera("cam_002"));
        Assert.Null(bounds.ForCamera("CAM_004"));
    }

    [Theory]
    [InlineData("20", "10")]
    [InlineData(null, "10")]
    [InlineData("bad", "10")]
    [InlineData("0", "9223372036854775808")]
    [InlineData("-922337203685.4775808", "922337203685.4775807")]
    [InlineData("-2147483648.1", "0")]
    [InlineData("0", "2147483647.1")]
    public void Unusable_range_is_ignored(string? minimum, string maximum)
    {
        var values = new Dictionary<string, string?>
        {
            ["PlateAcquisition:VerifiedCaptureOffsetBounds:CAM_002:MinimumSeconds"] = minimum,
            ["PlateAcquisition:VerifiedCaptureOffsetBounds:CAM_002:MaximumSeconds"] = maximum
        };
        var bounds = VerifiedPlateOffsetBounds.Read(new ConfigurationBuilder().AddInMemoryCollection(values).Build());

        Assert.Null(bounds.ForCamera("CAM_002"));
    }
}
