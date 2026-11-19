using System.Data;
using System.Data.Common;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>
/// 认证与用户管理数据访问实现。
/// </summary>
/// <remarks>
/// 数据库：达梦 DM8，使用 DmCommand 原生 SQL + 参数化查询（防 SQL 注入）。
/// 表结构见 Database/CreateAuthTables.sql。
/// 说明：达梦 VARCHAR 原生支持 Unicode，无需区分 VarChar/NVarChar。
/// </remarks>
public class AuthRepository : IAuthRepository
{
    private readonly string _connectionString;
    private readonly int _commandTimeoutSeconds;

    /// <summary>创建使用达梦连接串的认证与用户管理仓储。</summary>
    /// <param name="configuration">数据库连接串和命令超时配置。</param>
    /// <exception cref="InvalidOperationException">连接串缺失或为空。</exception>
    public AuthRepository(IConfiguration configuration)
    {
        _connectionString = configuration.GetConnectionString("CarPointCloudDb")
            ?? throw new InvalidOperationException("缺少数据库连接字符串 ConnectionStrings:CarPointCloudDb");
        _commandTimeoutSeconds = configuration.GetValue<int>("Database:CommandTimeoutSeconds", 30);

        if (string.IsNullOrWhiteSpace(_connectionString))
        {
            throw new InvalidOperationException("数据库连接字符串 ConnectionStrings:CarPointCloudDb 不能为空");
        }
    }

    /// <inheritdoc />
    public UserDto? GetUserByUsername(string username)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand(
            @"SELECT TOP 1 u.Id, u.Username, u.DisplayName, u.Status, u.LastLoginAt, u.CreatedAt,
                     r.RoleCode, r.RoleName
              FROM SysUser u
              LEFT JOIN SysUserRole ur ON ur.UserId = u.Id
              LEFT JOIN SysRole r ON r.Id = ur.RoleId
              WHERE u.Username = ?", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        cmd.Parameters.Add("@Username", DmDbType.VarChar, 64).Value = username;

        using var reader = cmd.ExecuteReader();
        return reader.Read() ? MapUserDto(reader) : null;
    }

    /// <inheritdoc />
    public UserDto? GetUserById(long id)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand(
            @"SELECT TOP 1 u.Id, u.Username, u.DisplayName, u.Status, u.LastLoginAt, u.CreatedAt,
                     r.RoleCode, r.RoleName
              FROM SysUser u
              LEFT JOIN SysUserRole ur ON ur.UserId = u.Id
              LEFT JOIN SysRole r ON r.Id = ur.RoleId
              WHERE u.Id = ?", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        cmd.Parameters.Add("@Id", DmDbType.Int64).Value = id;

        using var reader = cmd.ExecuteReader();
        return reader.Read() ? MapUserDto(reader) : null;
    }

    /// <inheritdoc />
    public (string Hash, string Salt)? GetPasswordCredentials(long userId)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand(
            "SELECT PasswordHash, PasswordSalt FROM SysUser WHERE Id = ?", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        cmd.Parameters.Add("@Id", DmDbType.Int64).Value = userId;

        using var reader = cmd.ExecuteReader();
        if (!reader.Read())
        {
            return null;
        }

        return (reader.GetString(0), reader.GetString(1));
    }

    /// <inheritdoc />
    public void UpdateLastLogin(long userId, DateTime loginTime)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand(
            "UPDATE SysUser SET LastLoginAt = ?, UpdatedAt = ? WHERE Id = ?", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        cmd.Parameters.Add("@LastLoginAt", DmDbType.DateTime).Value = loginTime;
        cmd.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = loginTime;
        cmd.Parameters.Add("@Id", DmDbType.Int64).Value = userId;
        cmd.ExecuteNonQuery();
    }

    /// <inheritdoc />
    public void InsertLoginLog(LoginLogRecord record)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand(
            @"INSERT INTO SysLoginLog (Username, ClientIp, UserAgent, Result, FailReason, CreatedAt)
              VALUES (?, ?, ?, ?, ?, ?)", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        cmd.Parameters.Add("@Username", DmDbType.VarChar, 64).Value = record.Username;
        cmd.Parameters.Add("@ClientIp", DmDbType.VarChar, 64).Value = (object?)record.ClientIp ?? DBNull.Value;
        cmd.Parameters.Add("@UserAgent", DmDbType.VarChar, 512).Value = (object?)record.UserAgent ?? DBNull.Value;
        cmd.Parameters.Add("@Result", DmDbType.VarChar, 16).Value = record.Result;
        cmd.Parameters.Add("@FailReason", DmDbType.VarChar, 256).Value = (object?)record.FailReason ?? DBNull.Value;
        cmd.Parameters.Add("@CreatedAt", DmDbType.DateTime).Value = record.CreatedAt;
        cmd.ExecuteNonQuery();
    }

    /// <inheritdoc />
    public List<UserDto> GetUsersPage(int page, int pageSize, out long total, string? keyword = null)
    {
        using var conn = CreateConnection();
        conn.Open();

        var searchFilter = string.Empty;
        if (!string.IsNullOrWhiteSpace(keyword))
        {
            searchFilter = "WHERE (u.Username LIKE ? OR u.DisplayName LIKE ?)";
        }

        using (var countCmd = new DmCommand($"SELECT COUNT(*) FROM SysUser u {searchFilter}", conn))
        {
            countCmd.CommandTimeout = _commandTimeoutSeconds;
            if (!string.IsNullOrWhiteSpace(keyword))
            {
                countCmd.Parameters.Add("@Kw1", DmDbType.VarChar, 128).Value = $"%{keyword.Trim()}%";
                countCmd.Parameters.Add("@Kw2", DmDbType.VarChar, 128).Value = $"%{keyword.Trim()}%";
            }

            total = Convert.ToInt64(countCmd.ExecuteScalar());
        }

        var offset = (page - 1) * pageSize;
        var users = new List<UserDto>();
        var pageFilter = offset > 0
            ? $"WHERE u.Id NOT IN (SELECT TOP {offset} Id FROM SysUser ORDER BY Id)"
            : string.Empty;
        var whereAnd = searchFilter.Length > 0 && pageFilter.Length > 0
            ? searchFilter + " AND " + pageFilter.Substring("WHERE ".Length)
            : searchFilter + pageFilter;
        using var cmd = new DmCommand(
            $@"SELECT TOP {pageSize} u.Id, u.Username, u.DisplayName, u.Status, u.LastLoginAt, u.CreatedAt,
                      r.RoleCode, r.RoleName
               FROM SysUser u
               LEFT JOIN SysUserRole ur ON ur.UserId = u.Id
               LEFT JOIN SysRole r ON r.Id = ur.RoleId
               {whereAnd}
               ORDER BY u.Id", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        if (!string.IsNullOrWhiteSpace(keyword))
        {
            cmd.Parameters.Add("@Kw1", DmDbType.VarChar, 128).Value = $"%{keyword.Trim()}%";
            cmd.Parameters.Add("@Kw2", DmDbType.VarChar, 128).Value = $"%{keyword.Trim()}%";
        }

        using var reader = cmd.ExecuteReader();
        while (reader.Read())
        {
            users.Add(MapUserDto(reader));
        }

        return users;
    }

    /// <inheritdoc />
    public long? CreateUser(CreateUserRequest request, string passwordHash, string passwordSalt)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var tx = (DmTransaction)conn.BeginTransaction();

        try
        {
            long userId;
            using (var cmd = new DmCommand(
                @"INSERT INTO SysUser (Username, PasswordHash, PasswordSalt, DisplayName, Status, CreatedAt, UpdatedAt)
                  VALUES (?, ?, ?, ?, ?, SYSDATE, SYSDATE);
                  SELECT SCOPE_IDENTITY()", conn, (DmTransaction)tx))
            {
                cmd.CommandTimeout = _commandTimeoutSeconds;
                cmd.Parameters.Add("@Username", DmDbType.VarChar, 64).Value = request.Username;
                cmd.Parameters.Add("@PasswordHash", DmDbType.VarChar, 128).Value = passwordHash;
                cmd.Parameters.Add("@PasswordSalt", DmDbType.VarChar, 64).Value = passwordSalt;
                cmd.Parameters.Add("@DisplayName", DmDbType.VarChar, 128).Value = request.DisplayName;
                cmd.Parameters.Add("@Status", DmDbType.Int32).Value = request.Status;
                userId = Convert.ToInt64(cmd.ExecuteScalar());
            }

            // 绑定角色
            var roleId = ResolveRoleId(conn, tx, request.RoleCode);
            if (roleId.HasValue)
            {
                using var roleCmd = new DmCommand(
                    "INSERT INTO SysUserRole (UserId, RoleId, CreatedAt) VALUES (?, ?, SYSDATE)",
                    conn, (DmTransaction)tx);
                roleCmd.CommandTimeout = _commandTimeoutSeconds;
                roleCmd.Parameters.Add("@UserId", DmDbType.Int64).Value = userId;
                roleCmd.Parameters.Add("@RoleId", DmDbType.Int64).Value = roleId.Value;
                roleCmd.ExecuteNonQuery();
            }

            tx.Commit();
            return userId;
        }
        catch (DmException ex) when (ex.Number == -6602)
        {
            // 达梦唯一约束冲突错误码：用户名重复
            TryRollback(tx);
            return null;
        }
        catch
        {
            TryRollback(tx);
            throw;
        }
    }

    /// <inheritdoc />
    public int UpdateUser(long userId, string? displayName, string? roleCode, int? status, string? newPasswordHash, string? newPasswordSalt)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var tx = (DmTransaction)conn.BeginTransaction();

        try
        {
            var sets = new List<string>();
            var parameters = new List<(string Name, object Value)>();

            if (displayName is not null)
            {
                sets.Add("DisplayName = ?");
                parameters.Add(("@DisplayName", displayName));
            }

            if (status.HasValue)
            {
                sets.Add("Status = ?");
                parameters.Add(("@Status", status.Value));
            }

            if (newPasswordHash is not null && newPasswordSalt is not null)
            {
                sets.Add("PasswordHash = ?");
                parameters.Add(("@PasswordHash", newPasswordHash));
                sets.Add("PasswordSalt = ?");
                parameters.Add(("@PasswordSalt", newPasswordSalt));
            }

            // 角色更新：先删除旧关联，再插入新关联
            if (roleCode is not null)
            {
                var roleId = ResolveRoleId(conn, tx, roleCode);
                if (roleId.HasValue)
                {
                    using var deleteCmd = new DmCommand(
                        "DELETE FROM SysUserRole WHERE UserId = ?", conn, (DmTransaction)tx);
                    deleteCmd.CommandTimeout = _commandTimeoutSeconds;
                    deleteCmd.Parameters.Add("@UserId", DmDbType.Int64).Value = userId;
                    deleteCmd.ExecuteNonQuery();

                    using var insertCmd = new DmCommand(
                        "INSERT INTO SysUserRole (UserId, RoleId, CreatedAt) VALUES (?, ?, SYSDATE)",
                        conn, (DmTransaction)tx);
                    insertCmd.CommandTimeout = _commandTimeoutSeconds;
                    insertCmd.Parameters.Add("@UserId", DmDbType.Int64).Value = userId;
                    insertCmd.Parameters.Add("@RoleId", DmDbType.Int64).Value = roleId.Value;
                    insertCmd.ExecuteNonQuery();
                }
            }

            if (sets.Count > 0)
            {
                sets.Add("UpdatedAt = SYSDATE");
                var sql = $"UPDATE SysUser SET {string.Join(", ", sets)} WHERE Id = ?";
                using var cmd = new DmCommand(sql, conn, (DmTransaction)tx);
                cmd.CommandTimeout = _commandTimeoutSeconds;
                foreach (var (name, value) in parameters)
                {
                    cmd.Parameters.Add(name, DmDbType.VarChar, 128).Value = value;
                }

                cmd.Parameters.Add("@UserId", DmDbType.Int64).Value = userId;
                var affected = cmd.ExecuteNonQuery();
                tx.Commit();
                return affected;
            }

            tx.Commit();
            return roleCode is null ? 0 : 1;
        }
        catch
        {
            TryRollback(tx);
            throw;
        }
    }

    /// <inheritdoc />
    public bool DeleteUser(long userId)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var tx = (DmTransaction)conn.BeginTransaction();

        try
        {
            using (var roleCmd = new DmCommand(
                "DELETE FROM SysUserRole WHERE UserId = ?", conn, (DmTransaction)tx))
            {
                roleCmd.CommandTimeout = _commandTimeoutSeconds;
                roleCmd.Parameters.Add("@UserId", DmDbType.Int64).Value = userId;
                roleCmd.ExecuteNonQuery();
            }

            using var cmd = new DmCommand(
                "DELETE FROM SysUser WHERE Id = ?", conn, (DmTransaction)tx);
            cmd.CommandTimeout = _commandTimeoutSeconds;
            cmd.Parameters.Add("@UserId", DmDbType.Int64).Value = userId;
            var affected = cmd.ExecuteNonQuery();

            tx.Commit();
            return affected == 1;
        }
        catch
        {
            TryRollback(tx);
            throw;
        }
    }

    /// <inheritdoc />
    public List<RoleDto> GetRoles()
    {
        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand(
            "SELECT Id, RoleCode, RoleName, Description FROM SysRole ORDER BY Id", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;

        var roles = new List<RoleDto>();
        using var reader = cmd.ExecuteReader();
        while (reader.Read())
        {
            roles.Add(new RoleDto
            {
                Id = reader.GetInt64(0),
                RoleCode = reader.GetString(1),
                RoleName = reader.GetString(2),
                Description = reader.IsDBNull(3) ? string.Empty : reader.GetString(3)
            });
        }

        return roles;
    }

    private long? ResolveRoleId(DmConnection conn, DmTransaction tx, string roleCode)
    {
        using var cmd = new DmCommand(
            "SELECT TOP 1 Id FROM SysRole WHERE RoleCode = ?", conn, tx);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        cmd.Parameters.Add("@RoleCode", DmDbType.VarChar, 64).Value = roleCode;
        var result = cmd.ExecuteScalar();
        return result is null || result == DBNull.Value ? null : Convert.ToInt64(result);
    }

    private DmConnection CreateConnection()
    {
        return new DmConnection(_connectionString);
    }

    private static UserDto MapUserDto(DbDataReader reader)
    {
        var roleCode = reader.IsDBNull(6) ? AuthModels.Roles.Viewer : reader.GetString(6);
        var roleName = reader.IsDBNull(7) ? "查看者" : reader.GetString(7);
        return new UserDto
        {
            Id = reader.GetInt64(0),
            Username = reader.GetString(1),
            DisplayName = reader.IsDBNull(2) ? string.Empty : reader.GetString(2),
            Status = reader.GetInt32(3),
            LastLoginAt = reader.IsDBNull(4) ? null : reader.GetDateTime(4),
            CreatedAt = reader.GetDateTime(5),
            RoleCode = roleCode,
            RoleName = roleName
        };
    }

    private static void TryRollback(DmTransaction tx)
    {
        try
        {
            tx.Rollback();
        }
        catch
        {
            // 保留原始异常
        }
    }
}
