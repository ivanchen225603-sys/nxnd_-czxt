namespace CarPointCloudWebApi.Domain;

/// <summary>
/// 提供系统约定的北京时间，避免部署主机时区改变无偏移业务时间的含义。
/// </summary>
public static class BusinessClock
{
    private static readonly TimeZoneInfo ChinaTimeZone = ResolveChinaTimeZone();

    /// <summary>当前北京时间，Kind 为 Unspecified，适配 DM8 DATETIME 与既有 JSON 契约。</summary>
    public static DateTime Now => TimeZoneInfo.ConvertTime(DateTimeOffset.UtcNow, ChinaTimeZone).DateTime;

    private static TimeZoneInfo ResolveChinaTimeZone()
    {
        foreach (var id in new[] { "China Standard Time", "Asia/Shanghai" })
        {
            try
            {
                return TimeZoneInfo.FindSystemTimeZoneById(id);
            }
            catch (TimeZoneNotFoundException)
            {
            }
            catch (InvalidTimeZoneException)
            {
            }
        }

        throw new InvalidOperationException("无法加载北京时间时区 China Standard Time/Asia/Shanghai");
    }
}
