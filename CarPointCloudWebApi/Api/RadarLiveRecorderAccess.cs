namespace CarPointCloudWebApi.Api;

/// <summary>
/// 现场录制工具的免登录访问：请求头 X-Radar-Live-Token 等于 RadarLive:RecorderToken 时，
/// 允许读取实时状态、发起与查询/取消手动采集。
/// </summary>
public static class RadarLiveRecorderAccess
{
    public const string HeaderName = "X-Radar-Live-Token";
    public const string AuditActor = "radar-recorder";

    public static bool IsAllowed(HttpContext context)
    {
        var path = context.Request.Path.Value?.TrimEnd('/') ?? string.Empty;
        var method = context.Request.Method;
        const string captures = "/api/v1/radar-live/captures";
        var allowedPath =
            (HttpMethods.IsGet(method) && path.Equals("/api/v1/radar-live/status", StringComparison.OrdinalIgnoreCase)) ||
            (HttpMethods.IsPost(method) && path.Equals(captures, StringComparison.OrdinalIgnoreCase)) ||
            (path.StartsWith(captures + "/", StringComparison.OrdinalIgnoreCase) &&
             (HttpMethods.IsGet(method) || path.EndsWith("/cancel", StringComparison.OrdinalIgnoreCase)));
        var supplied = context.Request.Headers[HeaderName].ToString();
        return allowedPath && supplied.Length > 0 &&
               supplied == context.RequestServices.GetRequiredService<IConfiguration>()["RadarLive:RecorderToken"];
    }
}
