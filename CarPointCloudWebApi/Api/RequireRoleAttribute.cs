using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.Mvc.Filters;

namespace CarPointCloudWebApi.Api;

/// <summary>
/// 角色授权特性（配合 JwtAuthMiddleware 使用）。
/// </summary>
/// <remarks>
/// 用法：在控制器或 Action 上标注 <c>[RequireRole(Roles = "admin")]</c>。
/// 前置条件：请求必须已通过 <see cref="JwtAuthMiddleware"/> 认证（用户信息已注入 HttpContext.Items）。
/// 校验失败：无用户信息返回 401，角色不匹配返回 403，统一使用 ApiResponse 格式。
/// </remarks>
[AttributeUsage(AttributeTargets.Class | AttributeTargets.Method, AllowMultiple = false, Inherited = true)]
public class RequireRoleAttribute : Attribute, IAsyncActionFilter
{
    /// <summary>允许的角色编码（逗号分隔），例如 "admin" 或 "admin,viewer"。</summary>
    public string Roles { get; set; } = string.Empty;

    /// <inheritdoc />
    public async Task OnActionExecutionAsync(ActionExecutingContext context, ActionExecutionDelegate next)
    {
        if (RadarLiveRecorderAccess.IsAllowed(context.HttpContext))
        {
            await next();
            return;
        }

        var userInfo = context.HttpContext.Items.TryGetValue(JwtAuthMiddleware.UserInfoKey, out var value)
            ? value as TokenUserInfo
            : null;

        if (userInfo is null)
        {
            context.Result = new ObjectResult(ApiResponse<object>.Fail(401, "认证失败：未登录或令牌无效"))
            {
                StatusCode = StatusCodes.Status401Unauthorized
            };
            return;
        }

        var allowed = (Roles ?? string.Empty)
            .Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);

        if (allowed.Length > 0 && !allowed.Contains(userInfo.RoleCode, StringComparer.OrdinalIgnoreCase))
        {
            context.Result = new ObjectResult(ApiResponse<object>.Fail(403, "权限不足：需要角色 " + string.Join("/", allowed)))
            {
                StatusCode = StatusCodes.Status403Forbidden
            };
            return;
        }

        await next();
    }
}
