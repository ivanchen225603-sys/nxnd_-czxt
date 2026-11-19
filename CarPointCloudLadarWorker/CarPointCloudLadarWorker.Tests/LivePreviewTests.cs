using CarPointCloudLadarWorker.Preview;
using Ladarclr;

namespace CarPointCloudLadarWorker.Tests;

public sealed class LivePreviewTests
{
    private readonly TestTimeProvider _time = new(new DateTimeOffset(2026, 11, 3, 2, 0, 0, TimeSpan.Zero));

    private LivePreview Create(bool enabled = true) => new(
        Microsoft.Extensions.Options.Options.Create(TestSupport.Options(o =>
        {
            o.EnableLivePreview = enabled;
            o.DualRadarExtrinsicPath = TestSupport.WriteExtrinsic(TestSupport.TempDirectory());
            o.LivePreviewMaxPointsPerCamera = 2;
        })),
        _time);

    private DateTime Now => _time.GetLocalNow().DateTime;

    [Fact]
    public void Observe_BothCameras_PublishesFusedFrameWithBTransformedIntoA()
    {
        var preview = Create();
        float3[] a = [new() { x = 1, y = 2, z = 3 }, new() { x = float.NaN }, new() { x = 4, y = 5, z = 6 }];
        float3[] b = [new() { x = 0, y = 0, z = 0 }, new() { x = 1, y = 1, z = 1 }, new() { x = 2, y = 2, z = 2 }];

        preview.Observe("CAM_004", a, 2, Now);
        Assert.Null(preview.GetFrameAfter(0));
        preview.Observe("CAM_005", b, 3, Now.AddSeconds(1));

        var frame = preview.GetFrameAfter(0)!;
        Assert.Equal(1, frame.Version);
        Assert.Equal((2, 3, 2, 2), (frame.SourcePointCountA, frame.SourcePointCountB, frame.PointCountA, frame.PointCountB));
        Assert.Equal([1f, 2, 3, 4, 5, 6, 1.5f, 0, 0, 3.5f, 2, 2], frame.Coordinates);
        Assert.Equal(1000, frame.PairDeltaMilliseconds);
        Assert.Null(preview.GetFrameAfter(1));
        Assert.True(preview.GetStatus().Available);
    }

    [Fact]
    public void Status_ExplainsWhyNoFrameIsAvailable()
    {
        var preview = Create();
        Assert.Equal("等待 CAM_004、CAM_005 的首份点云", preview.GetStatus().UnavailableReason);

        preview.Observe("CAM_004", [new() { x = 1 }], 1, Now);
        preview.Observe("CAM_005", [new() { x = 1 }], 1, Now);
        _time.AdvanceSeconds(22);

        Assert.Null(preview.GetFrameAfter(0));
        Assert.Contains("已过期", preview.GetStatus().UnavailableReason);
        Assert.Equal("实时预览未启用（LadarCapture:EnableLivePreview=false）", Create(enabled: false).GetStatus().UnavailableReason);
    }
}
