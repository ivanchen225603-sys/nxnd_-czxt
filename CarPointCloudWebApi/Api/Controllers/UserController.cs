using Microsoft.AspNetCore.Mvc;

namespace CarPointCloudWebApi.Api;

/// <summary>
/// 用户管理控制器（仅系统管理员）。
/// </summary>
/// <remarks>
/// 权限：所有接口标注 [RequireRole(Roles = "admin")]。
/// 安全要点：创建用户时密码经 PBKDF2 哈希存储；返回的用户信息不包含密码哈希/盐。
/// </remarks>
[ApiController]
[Route("api/users")]
[RequireRole(Roles = AuthModels.Roles.Admin)]
public class UserController : ControllerBase
{
    /// <summary>分页页码上限，防止偏移量计算整数溢出。</summary>
    private const int MaxPage = 100_000;

    private const int UsernameMaxLength = 64;
    private const int PasswordMinLength = 12;
    private const int PasswordMaxLength = 128;
    private const int DisplayNameMaxLength = 128;

    private readonly IAuthRepository _authRepository;
    private readonly ILogger<UserController> _logger;

    /// <summary>创建仅供管理员使用的用户管理控制器。</summary>
    /// <param name="authRepository">用户和角色数据仓储。</param>
    /// <param name="logger">用户管理操作日志记录器。</param>
    public UserController(
        IAuthRepository authRepository,
        ILogger<UserController> logger)
    {
        _authRepository = authRepository;
        _logger = logger;
    }

    /// <summary>
    /// 分页查询用户列表。
    /// </summary>
    /// <param name="page">页码，从 1 开始，默认 1。</param>
    /// <param name="pageSize">每页条数，默认 20，最大 100。</param>
    /// <param name="keyword">可选关键字，按用户名/显示名称模糊搜索。</param>
    /// <returns>用户列表与总数。</returns>
    [HttpGet]
    public ActionResult<ApiResponse<object>> GetUsers(
        [FromQuery] int page = 1,
        [FromQuery] int pageSize = 20,
        [FromQuery] string? keyword = null)
    {
        // 同时钳上限：仅 Math.Max(1, page) 时，page=int.MaxValue 会让 (page-1)*pageSize
        // 整数溢出为负，进而拼出 TOP -N 让数据库报错并返回 500。
        page = Math.Clamp(page, 1, MaxPage);
        pageSize = Math.Clamp(pageSize, 1, 100);

        var users = _authRepository.GetUsersPage(page, pageSize, out var total, keyword);
        return Ok(ApiResponse<object>.Success(new
        {
            items = users,
            total,
            page,
            pageSize
        }));
    }

    /// <summary>
    /// 获取全部角色列表。
    /// </summary>
    /// <returns>角色列表。</returns>
    [HttpGet("~/api/roles")]
    [RequireRole(Roles = AuthModels.Roles.Admin)]
    public ActionResult<ApiResponse<List<RoleDto>>> GetRoles()
    {
        var roles = _authRepository.GetRoles();
        return Ok(ApiResponse<List<RoleDto>>.Success(roles));
    }

    /// <summary>
    /// 创建用户。
    /// </summary>
    /// <param name="request">用户信息。</param>
    /// <returns>创建成功返回新用户信息。</returns>
    [HttpPost]
    public ActionResult<ApiResponse<UserDto>> CreateUser([FromBody] CreateUserRequest? request)
    {
        if (request is null)
        {
            return BadRequest(ApiResponse<UserDto>.Fail(400, "请求体不能为空"));
        }

        var username = request.Username?.Trim() ?? string.Empty;
        var displayName = request.DisplayName?.Trim() ?? string.Empty;
        var password = request.Password ?? string.Empty;

        if (username.Length == 0 || username.Length > UsernameMaxLength)
        {
            return BadRequest(ApiResponse<UserDto>.Fail(400, $"用户名长度必须在 1-{UsernameMaxLength} 之间"));
        }

        if (password.Length < PasswordMinLength || password.Length > PasswordMaxLength)
        {
            return BadRequest(ApiResponse<UserDto>.Fail(400, $"密码长度必须在 {PasswordMinLength}-{PasswordMaxLength} 之间"));
        }

        if (displayName.Length > DisplayNameMaxLength)
        {
            return BadRequest(ApiResponse<UserDto>.Fail(400, $"显示名称长度不能超过 {DisplayNameMaxLength}"));
        }

        if (!IsValidRole(request.RoleCode))
        {
            return BadRequest(ApiResponse<UserDto>.Fail(400, "角色编码不合法，可选值：admin、viewer"));
        }

        if (request.Status is not (AuthModels.UserStatus.Enabled or AuthModels.UserStatus.Disabled))
        {
            return BadRequest(ApiResponse<UserDto>.Fail(400, "状态不合法，可选值：1（启用）、0（禁用）"));
        }

        var (hash, salt) = PasswordHasher.HashPassword(password);
        var userId = _authRepository.CreateUser(new CreateUserRequest
        {
            Username = username,
            Password = password,
            DisplayName = displayName,
            RoleCode = request.RoleCode,
            Status = request.Status
        }, hash, salt);

        if (userId is null)
        {
            return Conflict(ApiResponse<UserDto>.Fail(409, $"用户名 {username} 已存在"));
        }

        var created = _authRepository.GetUserById(userId.Value);
        _logger.LogInformation("管理员创建用户: {Username}", username);
        return Ok(ApiResponse<UserDto>.Success(created!));
    }

    /// <summary>
    /// 更新用户（显示名称/角色/状态/重置密码）。
    /// </summary>
    /// <param name="id">用户 ID。</param>
    /// <param name="request">需要更新的字段，未传字段保持不变。</param>
    /// <returns>更新后的用户信息。</returns>
    [HttpPut("{id:long}")]
    public ActionResult<ApiResponse<UserDto>> UpdateUser(long id, [FromBody] UpdateUserRequest? request)
    {
        if (request is null)
        {
            return BadRequest(ApiResponse<UserDto>.Fail(400, "请求体不能为空"));
        }

        var existing = _authRepository.GetUserById(id);
        if (existing is null)
        {
            return NotFound(ApiResponse<UserDto>.Fail(404, $"用户 {id} 不存在"));
        }

        if (request.DisplayName is { Length: > DisplayNameMaxLength })
        {
            return BadRequest(ApiResponse<UserDto>.Fail(400, $"显示名称长度不能超过 {DisplayNameMaxLength}"));
        }

        if (request.RoleCode is not null && !IsValidRole(request.RoleCode))
        {
            return BadRequest(ApiResponse<UserDto>.Fail(400, "角色编码不合法，可选值：admin、viewer"));
        }

        if (request.Status is not null &&
            request.Status is not (AuthModels.UserStatus.Enabled or AuthModels.UserStatus.Disabled))
        {
            return BadRequest(ApiResponse<UserDto>.Fail(400, "状态不合法，可选值：1（启用）、0（禁用）"));
        }

        string? newHash = null;
        string? newSalt = null;
        if (request.NewPassword is not null)
        {
            if (request.NewPassword.Length < PasswordMinLength || request.NewPassword.Length > PasswordMaxLength)
            {
                return BadRequest(ApiResponse<UserDto>.Fail(400, $"密码长度必须在 {PasswordMinLength}-{PasswordMaxLength} 之间"));
            }

            (newHash, newSalt) = PasswordHasher.HashPassword(request.NewPassword);
        }

        // 禁用或改角色必须立刻生效，不等回查缓存自然过期。
        _authRepository.UpdateUser(
            id,
            request.DisplayName?.Trim(),
            request.RoleCode?.Trim(),
            request.Status,
            newHash,
            newSalt);

        var updated = _authRepository.GetUserById(id);
        _logger.LogInformation("管理员更新用户: {UserId}, {Username}", id, existing.Username);
        return Ok(ApiResponse<UserDto>.Success(updated!));
    }

    /// <summary>
    /// 删除用户。
    /// </summary>
    /// <param name="id">用户 ID。</param>
    /// <returns>删除成功返回 200。</returns>
    [HttpDelete("{id:long}")]
    public ActionResult<ApiResponse<object>> DeleteUser(long id)
    {
        var existing = _authRepository.GetUserById(id);
        if (existing is null)
        {
            return NotFound(ApiResponse<object>.Fail(404, $"用户 {id} 不存在"));
        }

        // 防止管理员误删自身
        var current = GetCurrentUser();
        if (current?.UserId == id)
        {
            return BadRequest(ApiResponse<object>.Fail(400, "不能删除当前登录账号"));
        }

        _authRepository.DeleteUser(id);
        _logger.LogInformation("管理员删除用户: {UserId}, {Username}", id, existing.Username);
        return Ok(ApiResponse<object>.Success(new { message = "用户已删除" }));
    }

    private TokenUserInfo? GetCurrentUser()
    {
        return HttpContext.Items.TryGetValue(JwtAuthMiddleware.UserInfoKey, out var value)
            ? value as TokenUserInfo
            : null;
    }

    private static bool IsValidRole(string? roleCode)
    {
        return roleCode is AuthModels.Roles.Admin or AuthModels.Roles.Viewer;
    }
}
