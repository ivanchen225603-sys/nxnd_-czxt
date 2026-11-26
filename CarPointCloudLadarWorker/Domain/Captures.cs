using System.Security.Cryptography;
using System.Text;
using Ladarclr;
using CarPointCloudLadarWorker.Config;

namespace CarPointCloudLadarWorker.Domain;

/// <summary>点坐标工具。</summary>
public static class PointFilter
{
    /// <summary>雷达会输出 NaN/Inf 占位点，落盘、计数和预览都只用有限点。</summary>
    public static bool IsFinite(float3 p) => float.IsFinite(p.x) && float.IsFinite(p.y) && float.IsFinite(p.z);
}

/// <summary>扫描会话接纳的一场点云，只含有限点。</summary>
/// <param name="WorkflowId">会话的 requestId，即 WebApi 的 workflowId。</param>
public sealed record PointCloudFrame(
    float3[] Points,
    string CameraIdentifier,
    string Source,
    DateTime CapturedAt,
    string WorkflowId,
    StaticFrameEvidence? StaticEvidence = null);

/// <summary>已落盘的一份 PCD。</summary>
public sealed record PcdCapture(
    string PcdPath,
    string CameraIdentifier,
    DateTime CapturedAt,
    string WorkflowId,
    StaticFrameEvidence? StaticEvidence = null);

/// <summary>WebApi 自动派发时绑定的停稳段；旧版自动指令缺少段编号，只可保留诊断点云。</summary>
public sealed record StaticCaptureWindow(string? SegmentId, DateTimeOffset CaptureUntil);

/// <summary>以设备实测上界标记完整扫描可能覆盖的最大时间区间。</summary>
public sealed record StaticFrameEvidence(StaticCaptureWindow Window, DateTimeOffset CallbackUtc,
    DateTimeOffset? ScanStartUtc, DateTimeOffset? ScanEndUtc)
{
    public static StaticFrameEvidence FromCallback(StaticCaptureWindow window, DateTimeOffset callbackUtc,
        LadarDeviceOptions device)
    {
        if (device.VerifiedMaxScanDurationMs is not > 0 || device.VerifiedMaxCallbackDelayMs is not >= 0 ||
            device.VerifiedClockUncertaintyMs is not >= 0)
            return new(window, callbackUtc, null, null);

        var uncertainty = device.VerifiedClockUncertaintyMs.Value;
        var before = (long)device.VerifiedMaxScanDurationMs.Value + device.VerifiedMaxCallbackDelayMs.Value + uncertainty;
        return new(window, callbackUtc, callbackUtc.AddMilliseconds(-before), callbackUtc.AddMilliseconds(uncertainty));
    }
}

/// <summary>同一扫描会话中两台雷达的 PCD；Primary 恒为 Devices[0]（A）。</summary>
public sealed record PairedPcdCapture(
    string VehicleCaptureId,
    PcdCapture Primary,
    PcdCapture Secondary,
    DateTime PairedAt)
{
    /// <summary>来车流程编号。</summary>
    public string WorkflowId => Primary.WorkflowId;

    /// <summary>两台采集时刻差（秒）。</summary>
    public double TimeDifferenceSeconds =>
        Math.Abs((Primary.CapturedAt - Secondary.CapturedAt).TotalSeconds);

    /// <summary>由两份 PCD 的点位与完整路径生成稳定 pairId（Analyzer 幂等键的一部分）。</summary>
    public static PairedPcdCapture Create(PcdCapture primary, PcdCapture secondary, DateTime pairedAt)
    {
        var source = $"{primary.CameraIdentifier}|{Path.GetFullPath(primary.PcdPath)}|" +
                     $"{secondary.CameraIdentifier}|{Path.GetFullPath(secondary.PcdPath)}";
        var pairId = Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(source)))
            .ToLowerInvariant()[..32];
        return new PairedPcdCapture(pairId, primary, secondary, pairedAt);
    }
}
