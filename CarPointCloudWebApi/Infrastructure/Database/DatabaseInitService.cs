using System.Data;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>
/// 启动初始化服务（自动建表 + 种子数据）。
/// </summary>
/// <remarks>
/// 职责：应用启动时自动完成数据库初始化，实现开箱即用：
/// 1. 幂等创建认证相关表（SysUser、SysRole、SysUserRole、SysLoginLog）；
/// 2. 预置角色 admin/viewer；
/// 3. 若不存在任何用户，使用 Auth:InitialAdminUsername 和 Auth:InitialAdminPassword 创建初始管理员。
/// 说明：本服务仅在 Database:Enabled=true 且数据库可连接时执行；所有操作幂等，可重复启动。
/// 连接串通过 ConnectionStrings:CarPointCloudDb 配置（Schema=CarPointCloud）。
/// </remarks>
public class DatabaseInitService : IHostedService
{
    private const string DefaultAdminUsername = "admin";

    private readonly IServiceScopeFactory _scopeFactory;
    private readonly IConfiguration _configuration;
    private readonly ILogger<DatabaseInitService> _logger;

    /// <summary>创建应用启动阶段的 DM8 认证表初始化服务。</summary>
    /// <param name="scopeFactory">保留用于解析应用级作用域服务的工厂。</param>
    /// <param name="configuration">数据库开关与连接串配置。</param>
    /// <param name="logger">初始化结果和异常日志记录器。</param>
    public DatabaseInitService(
        IServiceScopeFactory scopeFactory,
        IConfiguration configuration,
        ILogger<DatabaseInitService> logger)
    {
        _scopeFactory = scopeFactory;
        _configuration = configuration;
        _logger = logger;
    }

    /// <inheritdoc />
    public async Task StartAsync(CancellationToken cancellationToken)
    {
        if (!_configuration.GetValue<bool>("Database:Enabled", true))
        {
            _logger.LogInformation("数据库未启用（Database:Enabled=false），跳过启动初始化");
            return;
        }

        var connectionString = _configuration.GetConnectionString("CarPointCloudDb");
        if (string.IsNullOrWhiteSpace(connectionString))
        {
            throw new InvalidOperationException(
                "Database:Enabled=true 时必须配置 ConnectionStrings:CarPointCloudDb");
        }

        try
        {
            await EnsureSchemaAsync(connectionString, cancellationToken);
            await SeedRolesAndAdminAsync(connectionString, cancellationToken);
            _logger.LogInformation("数据库启动初始化完成");
        }
        catch (Exception ex)
        {
            _logger.LogCritical(ex, "数据库启动初始化失败，应用停止启动；请检查达梦服务与连接串配置");
            throw;
        }
    }

    /// <inheritdoc />
    public Task StopAsync(CancellationToken cancellationToken) => Task.CompletedTask;

    /// <summary>
    /// 幂等创建数据库表结构（与 Database/CreateAuthTables.sql 保持一致）。
    /// </summary>
    private async Task EnsureSchemaAsync(string connectionString, CancellationToken cancellationToken)
    {
        await using var conn = new DmConnection(connectionString);
        await conn.OpenAsync(cancellationToken);

        await ExecuteAsync(conn, AuthTableScripts.SysUserTable, cancellationToken);
        await ExecuteAsync(conn, AuthTableScripts.SysRoleTable, cancellationToken);
        await ExecuteAsync(conn, AuthTableScripts.SysUserRoleTable, cancellationToken);
        await ExecuteAsync(conn, AuthTableScripts.SysLoginLogTable, cancellationToken);
        await ExecuteAsync(conn, AuthTableScripts.Indexes, cancellationToken);
    }

    /// <summary>
    /// 预置角色并创建初始管理员（幂等）。
    /// </summary>
    private async Task SeedRolesAndAdminAsync(string connectionString, CancellationToken cancellationToken)
    {
        await using var conn = new DmConnection(connectionString);
        await conn.OpenAsync(cancellationToken);

        // 1. 预置角色
        await ExecuteAsync(conn, AuthTableScripts.SeedRoles, cancellationToken);

        // 2. 检查是否已有用户
        long userCount;
        await using (var countCmd = new DmCommand("SELECT COUNT(*) FROM SysUser", conn))
        {
            countCmd.CommandTimeout = 30;
            userCount = Convert.ToInt64(await countCmd.ExecuteScalarAsync(cancellationToken));
        }

        if (userCount > 0)
        {
            return;
        }

        // 3. 空库首次启动必须由部署人员显式提供密码，避免产生众所周知的默认口令。
        var username = (_configuration["Auth:InitialAdminUsername"] ?? DefaultAdminUsername).Trim();
        var password = _configuration["Auth:InitialAdminPassword"];
        if (string.IsNullOrWhiteSpace(password))
        {
            throw new InvalidOperationException("空数据库首次启动必须配置 Auth:InitialAdminPassword 初始管理员密码");
        }
        var (hash, salt) = PasswordHasher.HashPassword(password!);
        await using var tx = (DmTransaction)await conn.BeginTransactionAsync(cancellationToken);

        long userId;
        using (var userCmd = new DmCommand(
            @"INSERT INTO SysUser (Username, PasswordHash, PasswordSalt, DisplayName, Status, CreatedAt, UpdatedAt)
              VALUES (?, ?, ?, '系统管理员', 1, SYSDATE, SYSDATE);
              SELECT SCOPE_IDENTITY()", conn, tx))
        {
            userCmd.CommandTimeout = 30;
            userCmd.Parameters.Add("@Username", DmDbType.VarChar, 64).Value = username;
            userCmd.Parameters.Add("@PasswordHash", DmDbType.VarChar, 128).Value = hash;
            userCmd.Parameters.Add("@PasswordSalt", DmDbType.VarChar, 64).Value = salt;
            userId = Convert.ToInt64(await userCmd.ExecuteScalarAsync(cancellationToken));
        }

        // 绑定 admin 角色
        using (var roleCmd = new DmCommand(
            @"INSERT INTO SysUserRole (UserId, RoleId)
              SELECT ?, Id FROM SysRole WHERE RoleCode = 'admin'", conn, tx))
        {
            roleCmd.CommandTimeout = 30;
            roleCmd.Parameters.Add("@UserId", DmDbType.Int64).Value = userId;
            await roleCmd.ExecuteNonQueryAsync(cancellationToken);
        }

        await tx.CommitAsync(cancellationToken);

        _logger.LogWarning("已创建初始管理员账号 {Username}，请登录后立即修改密码", username);
    }

    private static async Task ExecuteAsync(DmConnection conn, string sql, CancellationToken cancellationToken)
    {
        // 按 "/" 分隔多条语句（达梦 disql 风格），逐条执行
        foreach (var statement in SplitScript(sql))
        {
            if (string.IsNullOrWhiteSpace(statement))
            {
                continue;
            }

            using var cmd = new DmCommand(statement, conn);
            cmd.CommandTimeout = 60;
            await cmd.ExecuteNonQueryAsync(cancellationToken);
        }
    }

    private static IEnumerable<string> SplitScript(string sql)
    {
        var lines = sql.Split('\n');
        var buffer = new System.Text.StringBuilder();
        var inString = false;

        foreach (var line in lines)
        {
            var trimmed = line.Trim();
            if (trimmed.StartsWith("--"))
            {
                continue;
            }

            // 语句结束符：行首为 / 且不在字符串中 → 输出当前缓冲（不含 / 行）
            if (!inString && trimmed == "/")
            {
                yield return buffer.ToString();
                buffer.Clear();
                continue;
            }

            foreach (var ch in line)
            {
                if (ch == '\'')
                {
                    inString = !inString;
                }

                buffer.Append(ch);
            }

            buffer.Append('\n');
        }

        if (buffer.ToString().Trim().Length > 0)
        {
            yield return buffer.ToString();
        }
    }
}

/// <summary>
/// 认证与点云文件表结构脚本（达梦 DM8，幂等）。
/// </summary>
/// <remarks>
/// 与 Database/CreateAuthTables.sql 保持语义一致；使用 IF NOT EXISTS 判断实现幂等。
/// 注意：EXECUTE IMMEDIATE 动态 SQL 内避免字符串字面量（列设为可空），规避引号转义问题。
/// </remarks>
public static class AuthTableScripts
{
    /// <summary>幂等创建系统用户表的 DM8 脚本。</summary>
    public const string SysUserTable = @"
IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'SYSUSER') THEN
  EXECUTE IMMEDIATE 'CREATE TABLE SysUser (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_SysUser PRIMARY KEY,
    Username VARCHAR(64) NOT NULL,
    PasswordHash VARCHAR(128) NOT NULL,
    PasswordSalt VARCHAR(64) NOT NULL,
    DisplayName VARCHAR(128) NULL,
    Status INT NOT NULL DEFAULT 1,
    LastLoginAt DATETIME(3) NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    UpdatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    CONSTRAINT UQ_SysUser_Username UNIQUE (Username)
  )';
END IF;
/";

    /// <summary>幂等创建系统角色表的 DM8 脚本。</summary>
    public const string SysRoleTable = @"
IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'SYSROLE') THEN
  EXECUTE IMMEDIATE 'CREATE TABLE SysRole (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_SysRole PRIMARY KEY,
    RoleCode VARCHAR(64) NOT NULL,
    RoleName VARCHAR(128) NOT NULL,
    Description VARCHAR(512) NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    CONSTRAINT UQ_SysRole_RoleCode UNIQUE (RoleCode)
  )';
END IF;
/";

    /// <summary>幂等创建用户与角色关联表的 DM8 脚本。</summary>
    public const string SysUserRoleTable = @"
IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'SYSUSERROLE') THEN
  EXECUTE IMMEDIATE 'CREATE TABLE SysUserRole (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_SysUserRole PRIMARY KEY,
    UserId BIGINT NOT NULL,
    RoleId BIGINT NOT NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    CONSTRAINT UQ_SysUserRole_User_Role UNIQUE (UserId, RoleId),
    CONSTRAINT FK_SysUserRole_User FOREIGN KEY (UserId) REFERENCES SysUser(Id),
    CONSTRAINT FK_SysUserRole_Role FOREIGN KEY (RoleId) REFERENCES SysRole(Id)
  )';
END IF;
/";

    /// <summary>幂等创建登录审计表的 DM8 脚本。</summary>
    public const string SysLoginLogTable = @"
IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'SYSLOGINLOG') THEN
  EXECUTE IMMEDIATE 'CREATE TABLE SysLoginLog (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_SysLoginLog PRIMARY KEY,
    Username VARCHAR(64) NOT NULL,
    ClientIp VARCHAR(64) NULL,
    UserAgent VARCHAR(512) NULL,
    Result VARCHAR(16) NOT NULL,
    FailReason VARCHAR(256) NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE
  )';
END IF;
/";

    /// <summary>幂等创建认证查询所需索引的 DM8 脚本集合。</summary>
    public const string Indexes = @"
IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_SYSUSER_STATUS') THEN
  EXECUTE IMMEDIATE 'CREATE INDEX IX_SysUser_Status ON SysUser(Status)';
END IF;
/
IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_SYSUSERROLE_ROLEID') THEN
  EXECUTE IMMEDIATE 'CREATE INDEX IX_SysUserRole_RoleId ON SysUserRole(RoleId)';
END IF;
/
IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_SYSLOGINLOG_CREATEDAT') THEN
  EXECUTE IMMEDIATE 'CREATE INDEX IX_SysLoginLog_CreatedAt ON SysLoginLog(CreatedAt DESC)';
END IF;
/";

    /// <summary>幂等写入管理员和查看者内置角色的 DM8 脚本。</summary>
    public const string SeedRoles = @"
IF NOT EXISTS (SELECT 1 FROM SysRole WHERE RoleCode = 'admin') THEN
  INSERT INTO SysRole (RoleCode, RoleName, Description, CreatedAt)
  VALUES ('admin', '系统管理员', '拥有用户管理和全部功能权限', SYSDATE);
END IF;
/
IF NOT EXISTS (SELECT 1 FROM SysRole WHERE RoleCode = 'viewer') THEN
  INSERT INTO SysRole (RoleCode, RoleName, Description, CreatedAt)
  VALUES ('viewer', '查看者', '仅拥有查看日志与点云数据的权限', SYSDATE);
END IF;
/";
}
