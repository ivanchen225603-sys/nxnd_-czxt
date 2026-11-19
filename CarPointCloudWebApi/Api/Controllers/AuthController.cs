using Microsoft.AspNetCore.Mvc;

namespace CarPointCloudWebApi.Api;

/// <summary>登录、退出、当前用户与改密。</summary>
[ApiController]
[Route("api/auth")]
public class AuthController(IAuthRepository authRepository, JwtTokenService tokenService, ILogger<AuthController> logger)
    : ControllerBase
{
    private const int UsernameMaxLength = 64;
    private const int PasswordMinLength = 12;
    private const int PasswordMaxLength = 128;

    [HttpPost("login")]
    public ActionResult<ApiResponse<LoginResponse>> Login([FromBody] LoginRequest? request)
    {
        var username = request?.Username?.Trim() ?? string.Empty;
        var password = request?.Password ?? string.Empty;
        if (username.Length is 0 or > UsernameMaxLength)
            return BadRequest(ApiResponse<LoginResponse>.Fail(400, $"用户名长度必须在 1-{UsernameMaxLength} 之间"));
        if (password.Length is 0 or > PasswordMaxLength)
            return BadRequest(ApiResponse<LoginResponse>.Fail(400, $"密码长度必须在 1-{PasswordMaxLength} 之间"));

        var user = authRepository.GetUserByUsername(username);
        var credentials = user is null ? null : authRepository.GetPasswordCredentials(user.Id);
        if (user is null || credentials is not { } stored || !PasswordHasher.VerifyPassword(password, stored.Hash, stored.Salt))
        {
            WriteLoginLog(username, "failed", "用户名或密码错误");
            return Unauthorized(ApiResponse<LoginResponse>.Fail(401, "用户名或密码错误"));
        }
        if (user.Status != AuthModels.UserStatus.Enabled)
        {
            WriteLoginLog(username, "failed", "账号已禁用");
            return Unauthorized(ApiResponse<LoginResponse>.Fail(401, "账号已被禁用，请联系管理员"));
        }

        var nowUtc = DateTime.UtcNow;
        var token = tokenService.IssueToken(
            new TokenUserInfo { UserId = user.Id, Username = user.Username, RoleCode = user.RoleCode }, nowUtc);
        authRepository.UpdateLastLogin(user.Id, DateTime.Now);
        WriteLoginLog(username, "success", null);
        return Ok(ApiResponse<LoginResponse>.Success(new LoginResponse
        {
            Token = token,
            ExpiresAt = nowUtc.AddMinutes(tokenService.TokenExpireMinutes),
            User = user
        }));
    }

    [HttpPost("logout")]
    [RequireRole(Roles = "admin,viewer")]
    public ActionResult<ApiResponse<object>> Logout()
    {
        logger.LogInformation("用户登出: {Username}", CurrentUser.Username);
        return Ok(ApiResponse<object>.Success(new { message = "已退出登录" }));
    }

    [HttpGet("me")]
    [RequireRole(Roles = "admin,viewer")]
    public ActionResult<ApiResponse<UserDto>> Me() =>
        authRepository.GetUserById(CurrentUser.UserId) is { } user
            ? Ok(ApiResponse<UserDto>.Success(user))
            : Unauthorized(ApiResponse<UserDto>.Fail(401, "用户不存在或已被删除"));

    [HttpPost("change-password")]
    [RequireRole(Roles = "admin,viewer")]
    public ActionResult<ApiResponse<object>> ChangePassword([FromBody] ChangePasswordRequest? request)
    {
        var newPassword = request?.NewPassword ?? string.Empty;
        if (newPassword.Length is < PasswordMinLength or > PasswordMaxLength)
            return BadRequest(ApiResponse<object>.Fail(400, $"新密码长度必须在 {PasswordMinLength}-{PasswordMaxLength} 之间"));

        var user = CurrentUser;
        var credentials = authRepository.GetPasswordCredentials(user.UserId);
        if (credentials is not { } stored ||
            !PasswordHasher.VerifyPassword(request!.OldPassword ?? string.Empty, stored.Hash, stored.Salt))
            return BadRequest(ApiResponse<object>.Fail(400, "原密码错误"));

        var (hash, salt) = PasswordHasher.HashPassword(newPassword);
        authRepository.UpdateUser(user.UserId, displayName: null, roleCode: null, status: null, hash, salt);
        logger.LogInformation("用户修改密码成功: {Username}", user.Username);
        return Ok(ApiResponse<object>.Success(new { message = "密码修改成功" }));
    }

    private TokenUserInfo CurrentUser => (TokenUserInfo)HttpContext.Items[JwtAuthMiddleware.UserInfoKey]!;

    private void WriteLoginLog(string username, string result, string? failReason) =>
        authRepository.InsertLoginLog(new LoginLogRecord
        {
            Username = username,
            ClientIp = HttpContext.Connection.RemoteIpAddress?.ToString(),
            UserAgent = Request.Headers.UserAgent.ToString(),
            Result = result,
            FailReason = failReason,
            CreatedAt = DateTime.Now
        });
}
