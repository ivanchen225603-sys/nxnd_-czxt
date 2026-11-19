using Microsoft.Extensions.Configuration;

namespace CarPointCloudLadarWorker.Tests;

public sealed class ShippedConfigurationTests
{
    [Theory]
    [InlineData("appsettings.json")]
    [InlineData("appsettings.Production.example.json")]
    public void ShippedConfiguration_BindsAndValidates(string fileName)
    {
        var path = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", fileName));

        var options = new ConfigurationBuilder().AddJsonFile(path).Build()
            .GetSection("LadarCapture").Get<LadarCaptureOptions>()!;
        options.Validate();

        Assert.Equal(["CAM_004", "CAM_005"], options.Devices.Select(x => x.CameraIdentifier));
        Assert.Equal(1, options.DiscardCompleteCyclesAfterCommand);
        Assert.True(options.AnalysisEnabled);
    }
}
