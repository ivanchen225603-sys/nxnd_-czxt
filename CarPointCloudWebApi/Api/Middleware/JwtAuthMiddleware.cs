using System.Security.Claims;

namespace CarPointCloudWebApi.Api;

/// <summary>
/// 管理端登录态：除匿名路径、载货雷达机器认证和录制令牌外，请求必须带有效的 Bearer 令牌。
/// 角色以令牌内容为准，授权由 <see cref="RequireRoleAttribute"/> 完成。
/// </summary>
public class JwtAuthMiddleware(RequestDelegate next, JwtTokenService tokenService)
{
    public const string UserInfoKey = "AuthUser";

    private static readonly string[] AnonymousPrefixes =
    [
        "/api/auth/login", "/swagger", "/api/v1/vehicleDimensions", "/api/v1/vehicleWorkflows", "/api/v1/coalVolumes",
        "/api/v1/licensePlateRecords", "/api/v1/internal", "/api/v1/radar"
    ];

    public async Task InvokeAsync(HttpContext context)
    {
        var path = context.Request.Path;
        if (LoadRadarAgentAuthMiddleware.IsAuthenticatedAgent(context) || RadarLiveRecorderAccess.IsAllowed(context) ||
            path.Equals("/healthz", StringComparison.OrdinalIgnoreCase) ||
            AnonymousPrefixes.Any(prefix => path.StartsWithSegments(prefix, StringComparison.OrdinalIgnoreCase)))
        {
            await next(context);
            return;
        }

        var header = context.Request.Headers.Authorization.ToString();
        var user = header.StartsWith("Bearer ", StringComparison.OrdinalIgnoreCase)
            ? tokenService.ValidateToken(header["Bearer ".Length..].Trim(), DateTime.UtcNow)
            : null;
        if (user is null)
        {
            context.Response.StatusCode = StatusCodes.Status401Unauthorized;
            await context.Response.WriteAsJsonAsync(ApiResponse<object>.Fail(401, "认证失败：未登录或令牌无效"));
            return;
        }

        context.Items[UserInfoKey] = user;
        context.User = new ClaimsPrincipal(new ClaimsIdentity(
        [
            new Claim(ClaimTypes.NameIdentifier, user.UserId.ToString()),
            new Claim(ClaimTypes.Name, user.Username),
            new Claim(ClaimTypes.Role, user.RoleCode)
        ], "Jwt"));
        await next(context);
    }
}
