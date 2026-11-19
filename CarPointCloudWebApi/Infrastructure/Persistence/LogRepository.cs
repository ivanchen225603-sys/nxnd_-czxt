using System.Data;
using System.Data.Common;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>
/// 日志查询数据访问实现。
/// </summary>
/// <remarks>
/// 数据来源：ApiAccessLog 表。
/// 新日志直接读取 LogLevel（debug / info / warn / error）；历史日志在 LogLevel 为空时
/// 继续按 HTTP 状态码映射，确保数据库升级后旧记录仍可查询。
/// 来源（source）：HttpMethod + RequestPath，如 "GET /api/auth/me"。
/// 消息（message）：请求路径 + 状态码 + 错误信息（如有）。
/// </remarks>
public class LogRepository : ILogRepository
{
    private readonly string _connectionString;
    private readonly int _commandTimeoutSeconds;

    /// <summary>创建从达梦审计表读取第三方交互链路的日志仓储。</summary>
    /// <param name="configuration">数据库连接串和命令超时配置。</param>
    public LogRepository(IConfiguration configuration)
    {
        _connectionString = configuration.GetConnectionString("CarPointCloudDb")
            ?? throw new InvalidOperationException("缺少数据库连接字符串 ConnectionStrings:CarPointCloudDb");
        _commandTimeoutSeconds = configuration.GetValue<int>("Database:CommandTimeoutSeconds", 30);
    }

    /// <inheritdoc />
    public List<LogEntryDto> QueryLogs(
        string? keyword,
        DateTime? startTime,
        DateTime? endTime,
        string? level,
        string? source,
        string? interactionId,
        string? workflowId,
        string? licensePlate,
        string? integrationType,
        int page,
        int pageSize,
        out long total)
    {
        page = Math.Max(1, page);
        pageSize = Math.Clamp(pageSize, 1, 100);

        var where = new List<string>();
        var parameters = new List<(string Name, object Value)>();

        if (startTime.HasValue)
        {
            where.Add("CreatedAt >= ?");
            parameters.Add(("@StartTime", startTime.Value));
        }

        if (endTime.HasValue)
        {
            where.Add("CreatedAt <= ?");
            parameters.Add(("@EndTime", endTime.Value));
        }

        var normalizedLevel = level?.Trim().ToLowerInvariant();
        if (normalizedLevel is "debug" or "info" or "warn" or "error")
        {
            var legacyCondition = normalizedLevel switch
            {
                "info" => "HttpStatus < 400",
                "warn" => "HttpStatus >= 400 AND HttpStatus < 500",
                "error" => "HttpStatus >= 500",
                _ => "1 = 0"
            };
            where.Add($"(LOWER(LogLevel) = ? OR (LogLevel IS NULL AND {legacyCondition}))");
            parameters.Add(("@Level", normalizedLevel));
        }

        if (!string.IsNullOrWhiteSpace(source))
        {
            where.Add("(HttpMethod + ' ' + RequestPath) LIKE ?");
            parameters.Add(("@Source", $"%{source.Trim()}%"));
        }

        AddExactFilter(where, parameters, "InteractionId", "@InteractionId", interactionId);
        AddExactFilter(where, parameters, "WorkflowId", "@WorkflowId", workflowId);
        AddExactFilter(where, parameters, "LicensePlate", "@LicensePlate", licensePlate);
        AddExactFilter(where, parameters, "IntegrationType", "@IntegrationType", integrationType);

        if (!string.IsNullOrWhiteSpace(keyword))
        {
            var kw = keyword.Trim();
            where.Add("(RequestPath LIKE ? OR ErrorMessage LIKE ? OR HttpMethod LIKE ? OR InteractionId LIKE ? OR WorkflowId LIKE ? OR LicensePlate LIKE ? OR IntegrationType LIKE ? OR LifecycleStage LIKE ? OR LogMessage LIKE ?)");
            parameters.Add(("@Kw1", $"%{kw}%"));
            parameters.Add(("@Kw2", $"%{kw}%"));
            parameters.Add(("@Kw3", $"%{kw}%"));
            parameters.Add(("@Kw4", $"%{kw}%"));
            parameters.Add(("@Kw5", $"%{kw}%"));
            parameters.Add(("@Kw6", $"%{kw}%"));
            parameters.Add(("@Kw7", $"%{kw}%"));
            parameters.Add(("@Kw8", $"%{kw}%"));
            parameters.Add(("@Kw9", $"%{kw}%"));
        }

        var whereSql = where.Count == 0 ? string.Empty : $"WHERE {string.Join(" AND ", where)}";

        using var conn = CreateConnection();
        conn.Open();

        using (var countCmd = new DmCommand($"SELECT COUNT(*) FROM ApiAccessLog {whereSql}", conn))
        {
            countCmd.CommandTimeout = _commandTimeoutSeconds;
            foreach (var (name, value) in parameters)
            {
                AddParameter(countCmd, name, value);
            }

            total = Convert.ToInt64(countCmd.ExecuteScalar());
        }

        var offset = (page - 1) * pageSize;
        var pagination = offset > 0
            ? $" AND Id NOT IN (SELECT TOP {offset} Id FROM ApiAccessLog {whereSql} ORDER BY CreatedAt DESC, Id DESC)"
            : string.Empty;
        var items = new List<LogEntryDto>();
        using var cmd = new DmCommand(
            $@"SELECT TOP {pageSize} Id, RecordKind, LogLevel, LifecycleStage, LogMessage,
                      InteractionId, IntegrationType, Direction, CallerSystem, TargetSystem,
                      WorkflowId, EventId, LicensePlate, CameraIdentifier, AttemptNo,
                      HttpMethod, RequestPath, QueryString, Scheme, Host, TraceIdentifier, ClientIp,
                      RequestContentType, ResponseContentType, TimestampHeader, SignatureHeader,
                      IdempotencyKeyHeader, RequestContentLength, ResponseContentLength,
                      HttpStatus, BusinessCode, ElapsedMs,
                      ErrorMessage, RequestHeadersJson, RequestBody, ResponseHeadersJson, ResponseBody, CreatedAt
               FROM ApiAccessLog
               {whereSql}{pagination}
               ORDER BY CreatedAt DESC, Id DESC", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        foreach (var (name, value) in parameters)
        {
            AddParameter(cmd, name, value);
        }
        if (offset > 0)
        {
            // DM8 使用位置参数；分页子查询再次复用了 whereSql，因此必须再次绑定一份参数。
            foreach (var (name, value) in parameters)
            {
                AddParameter(cmd, $"{name}Page", value);
            }
        }

        using var reader = cmd.ExecuteReader();
        while (reader.Read())
        {
            items.Add(MapEntry(reader));
        }

        return items;
    }

    /// <inheritdoc />
    public List<LogEntryDto> QueryInteraction(string interactionId)
    {
        if (string.IsNullOrWhiteSpace(interactionId) || interactionId.Length > 64)
        {
            return new List<LogEntryDto>();
        }

        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand(
            @"SELECT TOP 100 Id, RecordKind, LogLevel, LifecycleStage, LogMessage,
                      InteractionId, IntegrationType, Direction, CallerSystem, TargetSystem,
                      WorkflowId, EventId, LicensePlate, CameraIdentifier, AttemptNo,
                      HttpMethod, RequestPath, QueryString, Scheme, Host, TraceIdentifier, ClientIp,
                      RequestContentType, ResponseContentType, TimestampHeader, SignatureHeader,
                      IdempotencyKeyHeader, RequestContentLength, ResponseContentLength,
                      HttpStatus, BusinessCode, ElapsedMs,
                      ErrorMessage, RequestHeadersJson, RequestBody, ResponseHeadersJson, ResponseBody, CreatedAt
               FROM ApiAccessLog
               WHERE InteractionId = ?
               ORDER BY CreatedAt ASC, Id ASC", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        AddVarcharParameter(cmd, "@InteractionId", 64, interactionId);

        var items = new List<LogEntryDto>();
        using var reader = cmd.ExecuteReader();
        while (reader.Read())
        {
            items.Add(MapEntry(reader));
        }

        // SQL 已按真实落库时间排序。不能再按推算的 StartTime 重排，否则耗时较长的
        // 完成快照会被移到 RequestReceived 之前，破坏生命周期时间线。
        return items;
    }

    /// <inheritdoc />
    public IReadOnlyList<int> GetStatusCodesForLevel(string? level)
    {
        return level?.ToLowerInvariant() switch
        {
            "error" => new List<int> { 500, 502, 503, 504 },
            "warn" => new List<int> { 400, 401, 403, 404, 405, 409, 412, 413, 415, 422, 429 },
            "info" => new List<int> { 200, 201, 202, 204 },
            "debug" => new List<int>(), // debug 使用显式 LogLevel，不再由状态码推导
            _ => new List<int>()       // 全部
        };
    }

    private DmConnection CreateConnection()
    {
        return new DmConnection(_connectionString);
    }

    private static void AddParameter(DmCommand cmd, string name, object value)
    {
        if (value is DateTime dt)
        {
            cmd.Parameters.Add(name, DmDbType.DateTime).Value = dt;
        }
        else if (value is int intVal)
        {
            cmd.Parameters.Add(name, DmDbType.Int32).Value = intVal;
        }
        else
        {
            cmd.Parameters.Add(name, DmDbType.VarChar, 256).Value = value;
        }
    }

    private static LogEntryDto MapEntry(DbDataReader reader)
    {
        var id = reader.GetInt64(0);
        var recordKind = ReadString(reader, 1) ?? "Access";
        var explicitLevel = ReadString(reader, 2);
        var lifecycleStage = ReadString(reader, 3);
        var logMessage = ReadString(reader, 4);
        var interactionId = ReadString(reader, 5);
        var integrationType = ReadString(reader, 6);
        var direction = ReadString(reader, 7);
        var callerSystem = ReadString(reader, 8);
        var targetSystem = ReadString(reader, 9);
        var workflowId = ReadString(reader, 10);
        var eventId = ReadString(reader, 11);
        var licensePlate = ReadString(reader, 12);
        var cameraIdentifier = ReadString(reader, 13);
        var attemptNo = reader.IsDBNull(14) ? (int?)null : reader.GetInt32(14);
        var method = reader.GetString(15);
        var path = reader.GetString(16);
        var queryString = ReadString(reader, 17);
        var scheme = ReadString(reader, 18);
        var host = ReadString(reader, 19);
        var traceIdentifier = ReadString(reader, 20);
        var clientIp = ReadString(reader, 21);
        var requestContentType = ReadString(reader, 22);
        var responseContentType = ReadString(reader, 23);
        var timestampHeader = ReadString(reader, 24);
        var signatureHeader = ReadString(reader, 25);
        var idempotencyKeyHeader = ReadString(reader, 26);
        var requestContentLength = reader.IsDBNull(27) ? (long?)null : Convert.ToInt64(reader.GetValue(27));
        var responseContentLength = reader.IsDBNull(28) ? (long?)null : Convert.ToInt64(reader.GetValue(28));
        var status = reader.GetInt32(29);
        var businessCode = reader.GetInt32(30);
        var elapsedMs = reader.GetInt32(31);
        var errorMessage = ReadString(reader, 32);
        var requestHeadersJson = ReadString(reader, 33);
        var requestBody = ReadString(reader, 34);
        var responseHeadersJson = ReadString(reader, 35);
        var responseBody = ReadString(reader, 36);
        var createdAt = reader.GetDateTime(37);

        var level = explicitLevel?.Trim().ToLowerInvariant() is "debug" or "info" or "warn" or "error"
            ? explicitLevel.Trim().ToLowerInvariant()
            : status >= 500 ? "error" : status >= 400 ? "warn" : "info";
        var message = !string.IsNullOrWhiteSpace(logMessage)
            ? logMessage
            : errorMessage is { Length: > 0 }
            ? $"{method} {path} → HTTP {status}: {errorMessage}"
            : $"{method} {path} → HTTP {status}";

        return new LogEntryDto
        {
            Id = id,
            Time = createdAt.ToString("yyyy-MM-dd HH:mm:ss.fff"),
            StartTime = createdAt.AddMilliseconds(-elapsedMs).ToString("yyyy-MM-dd HH:mm:ss.fff"),
            Level = level,
            Source = $"{method} {path}",
            Message = message,
            RecordKind = recordKind,
            LifecycleStage = lifecycleStage,
            InteractionId = interactionId,
            IntegrationType = integrationType,
            Direction = direction,
            CallerSystem = callerSystem,
            TargetSystem = targetSystem,
            WorkflowId = workflowId,
            EventId = eventId,
            LicensePlate = licensePlate,
            CameraIdentifier = cameraIdentifier,
            AttemptNo = attemptNo,
            RequestHeadersJson = requestHeadersJson,
            RequestBody = requestBody,
            ResponseHeadersJson = responseHeadersJson,
            ResponseBody = responseBody,
            HttpMethod = method,
            RequestPath = path,
            QueryString = queryString,
            Scheme = scheme,
            Host = host,
            TraceIdentifier = traceIdentifier,
            ClientIp = clientIp,
            RequestContentType = requestContentType,
            ResponseContentType = responseContentType,
            TimestampHeader = timestampHeader,
            SignatureHeader = signatureHeader,
            IdempotencyKeyHeader = idempotencyKeyHeader,
            RequestContentLength = requestContentLength,
            ResponseContentLength = responseContentLength,
            HttpStatus = status,
            BusinessCode = businessCode,
            ElapsedMs = elapsedMs,
            ErrorMessage = errorMessage
        };
    }

    private static void AddExactFilter(
        List<string> where,
        List<(string Name, object Value)> parameters,
        string column,
        string parameterName,
        string? value)
    {
        if (!string.IsNullOrWhiteSpace(value))
        {
            where.Add($"{column} = ?");
            parameters.Add((parameterName, value.Trim()));
        }
    }

    private static void AddVarcharParameter(DmCommand command, string name, int size, string value)
    {
        command.Parameters.Add(name, DmDbType.VarChar, size).Value = value;
    }

    private static string? ReadString(DbDataReader reader, int ordinal)
    {
        return reader.IsDBNull(ordinal) ? null : Convert.ToString(reader.GetValue(ordinal));
    }
}
