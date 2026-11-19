using System.Text.Json;
using Ladarclr;

namespace CarPointCloudLadarWorker.Config;

/// <summary>把雷达 B 坐标变换到雷达 A 坐标系的 4x4 刚体外参（frame=B_to_A）。</summary>
public sealed class DualRadarExtrinsic
{
    private readonly float[] _m;

    private DualRadarExtrinsic(string cameraA, string cameraB, float[] matrix)
    {
        CameraA = cameraA;
        CameraB = cameraB;
        _m = matrix;
    }

    /// <summary>基准雷达 A。</summary>
    public string CameraA { get; }

    /// <summary>被变换的雷达 B。</summary>
    public string CameraB { get; }

    /// <summary>读取外参文件；格式不符直接抛出。</summary>
    public static DualRadarExtrinsic Load(string path)
    {
        using var document = JsonDocument.Parse(File.ReadAllText(path));
        var root = document.RootElement;
        if (root.GetProperty("frame").GetString() != "B_to_A")
            throw new InvalidOperationException($"外参 frame 必须为 B_to_A: {path}");

        var matrix = root.GetProperty("matrix").EnumerateArray()
            .Take(3)
            .SelectMany(row => row.EnumerateArray().Select(cell => (float)cell.GetDouble()))
            .ToArray();
        return new DualRadarExtrinsic(
            root.GetProperty("cameraA").GetString()!.Trim(),
            root.GetProperty("cameraB").GetString()!.Trim(),
            matrix);
    }

    /// <summary>B 坐标系下的点变换到 A 坐标系。</summary>
    public float3 Transform(float3 p) => new()
    {
        x = (_m[0] * p.x) + (_m[1] * p.y) + (_m[2] * p.z) + _m[3],
        y = (_m[4] * p.x) + (_m[5] * p.y) + (_m[6] * p.z) + _m[7],
        z = (_m[8] * p.x) + (_m[9] * p.y) + (_m[10] * p.z) + _m[11]
    };
}
