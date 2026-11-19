using System.Globalization;
using System.Text.RegularExpressions;

namespace CarPointCloudWebApi.Domain;

/// <summary>2026-09-29 合作方合同中的车牌、点位与本地时间格式。</summary>
public static class PartnerContract
{
    public const string TimeFormat = "yyyy-MM-dd HH:mm:ss";
    private static readonly Regex PlatePattern = new(
        "\\A[京津沪渝冀豫云辽黑湘皖鲁新苏浙赣鄂桂甘晋蒙陕吉闽贵粤青藏川宁琼][A-Z][A-Z0-9]{5,8}\\z",
        RegexOptions.Compiled);
    private static readonly Regex CameraPattern = new("\\A[A-Za-z0-9_]{1,32}\\z", RegexOptions.Compiled);

    public static bool IsLicensePlate(string? value) => value is not null && PlatePattern.IsMatch(value);
    public static bool IsCamera(string? value) => value is not null && CameraPattern.IsMatch(value);
    public static bool TryParseTime(string? value, out DateTime time) => DateTime.TryParseExact(
        value, TimeFormat, CultureInfo.InvariantCulture, DateTimeStyles.None, out time);
    public static string? FormatTime(DateTime? value) => value?.ToString(TimeFormat, CultureInfo.InvariantCulture);
}
