using System.Globalization;
using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Api;

/// <summary>
/// 合作方接口签名：Timestamp 为 Unix 毫秒，Signature 为小写 MD5(Timestamp + AppSecret)。
/// 煤量和车辆尺寸 GET 查询不检查客户端与服务器的时钟偏差。
/// </summary>
public class PartnerAuthMiddleware(RequestDelegate next, IOptions<IntegrationOptions> options, ILogger<PartnerAuthMiddleware> logger)
{
    private static readonly string[] SignedPrefixes =
    [
        "/api/v1/vehicleDimensions", "/api/v1/vehicleWorkflows", "/api/v1/coalVolumes", "/api/v1/licensePlateRecords/fetch",
        "/api/v1/internal/pointcloud/vehicleDimensions", "/api/v1/internal/pointcloud/coalVolumes"
    ];

    public async Task InvokeAsync(HttpContext context)
    {
        if (!SignedPrefixes.Any(prefix => context.Request.Path.StartsWithSegments(prefix, StringComparison.OrdinalIgnoreCase)))
        {
            await next(context);
            return;
        }

        var hasTimestamp = context.Request.Headers.TryGetValue("Timestamp", out var timestampValues);
        var hasSignature = context.Request.Headers.TryGetValue("Signature", out var signatureValues);
        var timestamp = timestampValues.ToString();
        var signature = signatureValues.ToString();
        var now = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        var tolerance = Math.Min(options.Value.TimestampToleranceSeconds, 60) * 1000L;
        var canonical = PartnerResponseFactory.IsPartnerQueryPath(context.Request.Path);
        var isQuery = HttpMethods.IsGet(context.Request.Method) &&
            PartnerResponseFactory.IsPartnerQuery(context.Request.Path);
        var error = !hasTimestamp || !hasSignature ? "缺少 Timestamp 或 Signature"
            : !long.TryParse(timestamp, out var timestampMs) ||
              (canonical && (timestampValues.Count != 1 || timestamp != timestampMs.ToString(CultureInfo.InvariantCulture))) ||
              (isQuery && (timestamp.Length != 13 || timestampMs <= 0))
                ? "Timestamp 格式非法"
            : !isQuery && (timestampMs < now - tolerance || timestampMs > now + tolerance) ? "Timestamp 已过期"
            : signature != ApiSignatureHelper.GenerateSignature(timestamp, options.Value.AppSecret) ? "Signature 不匹配"
            : null;
        if (error is not null)
        {
            logger.LogWarning("Auth failed: {Method} {Path} reason={Reason} clientIp={ClientIp}",
                context.Request.Method, context.Request.Path, error, context.Connection.RemoteIpAddress);
            context.Response.StatusCode = StatusCodes.Status401Unauthorized;
            await context.Response.WriteAsJsonAsync(PartnerResponseFactory.Fail(context.Request.Path, 401, "鉴权失败：" + error));
            return;
        }
        await next(context);
    }
}
