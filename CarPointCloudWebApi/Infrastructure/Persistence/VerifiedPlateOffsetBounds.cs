using System.Globalization;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>逐点位核实的乙方抓拍时间减我方实际到磅时间范围，包含时钟差、触发延迟、迟拍和时间精度。</summary>
public sealed class VerifiedPlateOffsetBounds
{
    private readonly Dictionary<string, (TimeSpan Minimum, TimeSpan Maximum)> _byCamera = new(StringComparer.Ordinal);

    public static VerifiedPlateOffsetBounds Read(IConfiguration configuration, ILogger? logger = null)
    {
        var result = new VerifiedPlateOffsetBounds();
        foreach (var camera in configuration.GetSection("PlateAcquisition:VerifiedCaptureOffsetBounds").GetChildren())
        {
            if (!PartnerContract.IsCamera(camera.Key) || !TryReadSeconds(camera["MinimumSeconds"], out var minimum) ||
                !TryReadSeconds(camera["MaximumSeconds"], out var maximum) || minimum > maximum)
            {
                logger?.LogWarning("点位 {Camera} 的已核实抓拍时间范围无效，已忽略", camera.Key);
                continue;
            }
            result._byCamera[camera.Key] = (minimum, maximum);
        }
        return result;
    }

    public (TimeSpan Minimum, TimeSpan Maximum)? ForCamera(string camera) =>
        _byCamera.TryGetValue(camera, out var bounds) ? bounds : null;

    private static bool TryReadSeconds(string? value, out TimeSpan interval)
    {
        interval = default;
        // 秒数采用配置中带符号 32 位秒的范围；任意两端的差仍可安全用于车次时间窗计算。
        if (!decimal.TryParse(value, NumberStyles.AllowLeadingSign | NumberStyles.AllowDecimalPoint,
                CultureInfo.InvariantCulture, out var seconds) ||
            seconds < int.MinValue || seconds > int.MaxValue)
            return false;

        var ticks = seconds * TimeSpan.TicksPerSecond;
        if (ticks != decimal.Truncate(ticks)) return false;
        interval = TimeSpan.FromTicks((long)ticks);
        return true;
    }
}
