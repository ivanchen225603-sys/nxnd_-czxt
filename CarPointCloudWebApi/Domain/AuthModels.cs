namespace CarPointCloudWebApi.Domain;

/// <summary>
/// 认证相关请求/响应模型。
/// </summary>
/// <remarks>
/// 认证方案：JWT Bearer Token（HS256，自研签发/校验，无第三方依赖）。
/// 权限模型：RBAC（用户-角色多对多），预置角色 admin/viewer。
/// 密码存储：PBKDF2-SHA256 加盐哈希，见 <see cref="PasswordHasher"/>。
/// </remarks>
public static class AuthModels
{
    /// <summary>角色编码常量。</summary>
    public static class Roles
    {
        /// <summary>系统管理员：用户管理 + 全部功能。</summary>
        public const string Admin = "admin";
        /// <summary>查看者：仅查看日志与点云数据。</summary>
        public const string Viewer = "viewer";
    }

    /// <summary>用户状态常量。</summary>
    public static class UserStatus
    {
        /// <summary>禁用。</summary>
        public const int Disabled = 0;
        /// <summary>启用。</summary>
        public const int Enabled = 1;
    }
}

/// <summary>
/// 登录请求。
/// </summary>
public class LoginRequest
{
    /// <summary>用户名，1-64 字符。</summary>
    public string Username { get; set; } = string.Empty;

    /// <summary>密码，1-128 字符。</summary>
    public string Password { get; set; } = string.Empty;
}

/// <summary>
/// 登录响应。
/// </summary>
public class LoginResponse
{
    /// <summary>JWT 访问令牌。</summary>
    public string Token { get; set; } = string.Empty;

    /// <summary>令牌过期时间（UTC，ISO8601）。</summary>
    public DateTime ExpiresAt { get; set; }

    /// <summary>当前用户信息。</summary>
    public UserDto User { get; set; } = new();
}

/// <summary>
/// 修改密码请求。
/// </summary>
public class ChangePasswordRequest
{
    /// <summary>原密码。</summary>
    public string OldPassword { get; set; } = string.Empty;

    /// <summary>新密码，6-128 字符。</summary>
    public string NewPassword { get; set; } = string.Empty;
}

/// <summary>
/// 创建用户请求（管理员）。
/// </summary>
public class CreateUserRequest
{
    /// <summary>用户名，1-64 字符，唯一。</summary>
    public string Username { get; set; } = string.Empty;

    /// <summary>初始密码，6-128 字符。</summary>
    public string Password { get; set; } = string.Empty;

    /// <summary>显示名称。</summary>
    public string DisplayName { get; set; } = string.Empty;

    /// <summary>角色编码，见 <see cref="AuthModels.Roles"/>。</summary>
    public string RoleCode { get; set; } = AuthModels.Roles.Viewer;

    /// <summary>状态：1=启用 0=禁用。</summary>
    public int Status { get; set; } = AuthModels.UserStatus.Enabled;
}

/// <summary>
/// 更新用户请求（管理员）。
/// </summary>
public class UpdateUserRequest
{
    /// <summary>显示名称。</summary>
    public string? DisplayName { get; set; }

    /// <summary>角色编码，见 <see cref="AuthModels.Roles"/>；不传则不修改。</summary>
    public string? RoleCode { get; set; }

    /// <summary>状态：1=启用 0=禁用；不传则不修改。</summary>
    public int? Status { get; set; }

    /// <summary>重置密码；不传则不修改。</summary>
    public string? NewPassword { get; set; }
}

/// <summary>
/// 用户信息 DTO（对外返回，不含密码哈希/盐）。
/// </summary>
public class UserDto
{
    /// <summary>用户 ID。</summary>
    public long Id { get; set; }

    /// <summary>用户名。</summary>
    public string Username { get; set; } = string.Empty;

    /// <summary>显示名称。</summary>
    public string DisplayName { get; set; } = string.Empty;

    /// <summary>状态：1=启用 0=禁用。</summary>
    public int Status { get; set; } = AuthModels.UserStatus.Enabled;

    /// <summary>角色编码。</summary>
    public string RoleCode { get; set; } = AuthModels.Roles.Viewer;

    /// <summary>角色名称。</summary>
    public string RoleName { get; set; } = string.Empty;

    /// <summary>最近登录时间。</summary>
    public DateTime? LastLoginAt { get; set; }

    /// <summary>创建时间。</summary>
    public DateTime CreatedAt { get; set; }
}

/// <summary>
/// 角色 DTO。
/// </summary>
public class RoleDto
{
    /// <summary>角色 ID。</summary>
    public long Id { get; set; }

    /// <summary>角色编码。</summary>
    public string RoleCode { get; set; } = string.Empty;

    /// <summary>角色名称。</summary>
    public string RoleName { get; set; } = string.Empty;

    /// <summary>角色描述。</summary>
    public string Description { get; set; } = string.Empty;
}

/// <summary>
/// 登录日志记录（数据库实体）。
/// </summary>
public class LoginLogRecord
{
    /// <summary>日志 ID。</summary>
    public long Id { get; set; }

    /// <summary>用户名。</summary>
    public string Username { get; set; } = string.Empty;

    /// <summary>客户端 IP。</summary>
    public string? ClientIp { get; set; }

    /// <summary>User-Agent。</summary>
    public string? UserAgent { get; set; }

    /// <summary>结果：success / failed。</summary>
    public string Result { get; set; } = "success";

    /// <summary>失败原因。</summary>
    public string? FailReason { get; set; }

    /// <summary>创建时间。</summary>
    public DateTime CreatedAt { get; set; } = DateTime.Now;
}
