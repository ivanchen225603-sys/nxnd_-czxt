using CarPointCloudLadarWorker.Capture;
using Ladarclr;

namespace CarPointCloudLadarWorker.Tests;

public sealed class CaptureFilesTests
{
    [Theory]
    [InlineData(2, 59, true)]
    [InlineData(3, 0, false)]
    [InlineData(4, 0, false)]
    public async Task DailyCleanup_UsesThreeAmCutoffEvenWhenHourlyRetentionIsDisabled(int hour, int minute, bool retained)
    {
        var root = TestSupport.TempDirectory();
        var path = Path.Combine(root, "capture.pcd");
        File.WriteAllText(path, "fixture");
        File.SetLastWriteTime(path, new DateTime(2026, 11, 5, 2, 0, 0));
        using var worker = new CaptureCleanupWorker(
            Options.Create(TestSupport.Options(o =>
            {
                o.CaptureDirectory = root;
                o.CaptureCleanupDailyAt = "03:00";
                o.CaptureRetentionHours = 0;
            })),
            new TestTimeProvider(new DateTimeOffset(2026, 11, 5, hour, minute, 0, TimeSpan.Zero)),
            NullLogger<CaptureCleanupWorker>.Instance);
        await worker.StartAsync(CancellationToken.None);
        await worker.StopAsync(CancellationToken.None);
        Assert.Equal(retained, File.Exists(path));
    }

    [Fact]
    public void WritePcd_WritesAsciiXyzWithScaledCoordinates()
    {
        var directory = TestSupport.TempDirectory();
        float3[] points = [new() { x = 1.5f, y = -2f, z = 0.25f }, new() { x = 0.1f, y = 0, z = 3 }];

        var path = CaptureWorker.WritePcd(directory, "CAM_004", points, 2.0);

        Assert.Matches(@"CAM_004_\d{8}_\d{6}_\d{3}_[0-9a-f]{32}\.pcd$", Path.GetFileName(path));
        var lines = File.ReadAllLines(path);
        Assert.Equal(
            ["# .PCD v0.7 - Point Cloud Data file format", "VERSION 0.7", "FIELDS x y z", "SIZE 4 4 4", "TYPE F F F",
             "COUNT 1 1 1", "WIDTH 2", "HEIGHT 1", "VIEWPOINT 0 0 0 1 0 0 0", "POINTS 2", "DATA ascii",
             "3 -4 0.5", "0.200000003 0 6"],
            lines);
    }

    [Fact]
    public void DeleteOlderThan_RemovesExpiredPcdAndArtifactsButKeepsProfiles()
    {
        var root = TestSupport.TempDirectory();
        var profiles = Path.Combine(root, "profiles");
        Directory.CreateDirectory(profiles);
        var old = DateTime.Now.AddDays(-2);
        string File(string name, DateTime writtenAt, string? directory = null)
        {
            var path = Path.Combine(directory ?? root, name);
            System.IO.File.WriteAllText(path, "x");
            System.IO.File.SetLastWriteTime(path, writtenAt);
            return path;
        }
        var expired = File("CAM_004_a.pcd", old);
        var ribs = File("CAM_004_a_rib_coords_1.json", old);
        var fresh = File("CAM_004_b.pcd", DateTime.Now);
        var fusedEmpty = File("x_fused_empty.pcd", old);
        var profile = File("p.pcd", old, profiles);
        var worker = new CaptureCleanupWorker(
            Microsoft.Extensions.Options.Options.Create(TestSupport.Options(o =>
            {
                o.CaptureDirectory = root;
                o.PermanentPointCloudDirectories = [profiles];
            })),
            TimeProvider.System,
            NullLogger<CaptureCleanupWorker>.Instance);

        worker.DeleteOlderThan(DateTime.Now.AddDays(-1));

        Assert.False(System.IO.File.Exists(expired));
        Assert.False(System.IO.File.Exists(ribs));
        Assert.True(System.IO.File.Exists(fresh));
        Assert.True(System.IO.File.Exists(fusedEmpty));
        Assert.True(System.IO.File.Exists(profile));
    }
}
