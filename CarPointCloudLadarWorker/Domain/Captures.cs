using System.Security.Cryptography;
using System.Text;
using Ladarclr;

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
    string WorkflowId);

/// <summary>已落盘的一份 PCD。</summary>
public sealed record PcdCapture(
    string PcdPath,
    string CameraIdentifier,
    DateTime CapturedAt,
    string WorkflowId);

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
