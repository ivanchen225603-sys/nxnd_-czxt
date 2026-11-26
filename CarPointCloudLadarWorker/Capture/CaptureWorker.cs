using System.Globalization;
using System.Text;
using System.Threading.Channels;
using CarPointCloudLadarWorker.Analysis;
using CarPointCloudLadarWorker.Config;
using CarPointCloudLadarWorker.Domain;
using Ladarclr;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Capture;

/// <summary>
/// 把扫描会话接纳的点云写成 PCD，同一会话两台都落盘后组成一对交给分析。
/// </summary>
/// <remarks>会话串行进行，所以只需记住最近一份尚未配对的 PCD。</remarks>
public sealed class CaptureWorker(
    IOptions<LadarCaptureOptions> options,
    Channel<PointCloudFrame> frames,
    ScanSessions sessions,
    AnalysisWorker analysis,
    ILogger<CaptureWorker> logger) : BackgroundService
{
    private readonly LadarCaptureOptions _options = options.Value;
    private PcdCapture? _waiting;

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        await foreach (var frame in frames.Reader.ReadAllAsync(stoppingToken))
        {
            var path = WritePcd(_options.CaptureDirectory, frame.CameraIdentifier, frame.Points, _options.CoordinateScale);
            sessions.MarkPersisted(frame.WorkflowId, frame.CameraIdentifier);
            logger.LogInformation(
                "{Camera}/{Source} 已采集 {Count} 点 -> {PcdPath} workflowId={WorkflowId} capturedAt={CapturedAt:HH:mm:ss.fff} 采集到落盘={PersistMs}ms",
                frame.CameraIdentifier, frame.Source, frame.Points.Length, path, frame.WorkflowId, frame.CapturedAt,
                (long)(DateTime.Now - frame.CapturedAt).TotalMilliseconds);
            Pair(new PcdCapture(path, frame.CameraIdentifier, frame.CapturedAt, frame.WorkflowId, frame.StaticEvidence));
        }
    }

    private void Pair(PcdCapture capture)
    {
        var other = _waiting;
        if (other is null || other.WorkflowId != capture.WorkflowId || other.CameraIdentifier == capture.CameraIdentifier)
        {
            if (other is not null)
                logger.LogWarning("PCD 未配对: workflowId={WorkflowId}, camera={Camera}, path={Path}",
                    other.WorkflowId, other.CameraIdentifier, other.PcdPath);
            _waiting = capture;
            return;
        }

        _waiting = null;
        var isPrimary = capture.CameraIdentifier == _options.Devices[0].CameraIdentifier;
        var pair = PairedPcdCapture.Create(isPrimary ? capture : other, isPrimary ? other : capture, DateTime.Now);
        logger.LogInformation(
            "双雷达 PCD 已判定为同一辆车: pairId={PairId}, workflowId={WorkflowId}, A={CameraA}, B={CameraB}, 时间差={Gap:0.000}s",
            pair.VehicleCaptureId, pair.WorkflowId, pair.Primary.CameraIdentifier, pair.Secondary.CameraIdentifier,
            pair.TimeDifferenceSeconds);
        if (_options.AnalysisEnabled)
            analysis.Enqueue(pair);
    }

    /// <summary>写 ASCII PCD（x y z，float G9），文件名 {camera}_{yyyyMMdd_HHmmss_fff}_{guid}.pcd。</summary>
    internal static string WritePcd(string directory, string camera, float3[] points, double scale)
    {
        Directory.CreateDirectory(directory);
        var path = Path.Combine(Path.GetFullPath(directory), $"{camera}_{DateTime.Now:yyyyMMdd_HHmmss_fff}_{Guid.NewGuid():N}.pcd");
        using var writer = new StreamWriter(path, false, Encoding.ASCII);
        string[] header =
        [
            "# .PCD v0.7 - Point Cloud Data file format", "VERSION 0.7", "FIELDS x y z", "SIZE 4 4 4",
            "TYPE F F F", "COUNT 1 1 1", $"WIDTH {points.Length}", "HEIGHT 1", "VIEWPOINT 0 0 0 1 0 0 0",
            $"POINTS {points.Length}", "DATA ascii"
        ];
        foreach (var line in header)
            writer.WriteLine(line);
        foreach (var p in points)
        {
            writer.Write((p.x * scale).ToString("G9", CultureInfo.InvariantCulture));
            writer.Write(' ');
            writer.Write((p.y * scale).ToString("G9", CultureInfo.InvariantCulture));
            writer.Write(' ');
            writer.WriteLine((p.z * scale).ToString("G9", CultureInfo.InvariantCulture));
        }
        return path;
    }
}
