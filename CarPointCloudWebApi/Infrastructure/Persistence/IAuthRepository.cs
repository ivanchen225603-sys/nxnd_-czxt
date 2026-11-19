
namespace CarPointCloudWebApi.Infrastructure;

/// <summary>
/// 认证与用户管理数据访问接口。
/// </summary>
/// <remarks>
/// 职责：封装 SysUser / SysRole / SysUserRole / SysLoginLog 表的全部交互操作。
/// </remarks>
public interface IAuthRepository
{
    /// <summary>按用户名查询用户（含角色编码/名称），未找到返回 null。</summary>
    UserDto? GetUserByUsername(string username);

    /// <summary>按 ID 查询用户（含角色编码/名称），未找到返回 null。</summary>
    UserDto? GetUserById(long id);

    /// <summary>查询用户的密码哈希与盐（用于登录校验）。</summary>
    (string Hash, string Salt)? GetPasswordCredentials(long userId);

    /// <summary>更新用户最近登录时间。</summary>
    void UpdateLastLogin(long userId, DateTime loginTime);

    /// <summary>记录登录日志。</summary>
    void InsertLoginLog(LoginLogRecord record);

    /// <summary>分页查询用户列表，可按用户名/显示名称模糊搜索。</summary>
    List<UserDto> GetUsersPage(int page, int pageSize, out long total, string? keyword = null);

    /// <summary>创建用户，返回新用户 ID；用户名已存在返回 null。</summary>
    long? CreateUser(CreateUserRequest request, string passwordHash, string passwordSalt);

    /// <summary>更新用户资料/角色/状态/密码，返回受影响行数（0 表示用户不存在或未变更）。</summary>
    int UpdateUser(long userId, string? displayName, string? roleCode, int? status, string? newPasswordHash, string? newPasswordSalt);

    /// <summary>删除用户（同时清理角色关联），返回是否删除成功。</summary>
    bool DeleteUser(long userId);

    /// <summary>查询全部角色列表。</summary>
    List<RoleDto> GetRoles();
}
