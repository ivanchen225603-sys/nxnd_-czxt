using System.Data;
using System.Data.Common;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public partial class IntegrationRepository
{

    /// <inheritdoc />
    public async Task InsertApiAccessLogAsync(ApiAccessLogRecord record, CancellationToken cancellationToken = default)
    {
        await using var conn = CreateConnection();
        await conn.OpenAsync(cancellationToken).ConfigureAwait(false);

        using var cmd = new DmCommand(
            @"INSERT INTO ApiAccessLog
              (RecordKind, LogLevel, LifecycleStage, LogMessage,
               InteractionId, IntegrationType, Direction, CallerSystem, TargetSystem, WorkflowId, EventId,
               LicensePlate, CameraIdentifier, AttemptNo,
               HttpMethod, RequestPath, QueryString, Scheme, Host, TraceIdentifier, ClientIp, UserAgent,
               RequestContentType, TimestampHeader, SignatureHeader, IdempotencyKeyHeader, RequestHeadersJson,
               RequestBody, ResponseContentType, ResponseHeadersJson, ResponseBody, RequestContentLength, ResponseContentLength,
               HttpStatus, BusinessCode, ElapsedMs, ErrorMessage, CreatedAt)
              VALUES
              (?, ?, ?, ?, ?, ?, ?, ?, ?, ?,
               ?, ?, ?, ?, ?, ?, ?, ?, ?, ?,
               ?, ?, ?, ?, ?, ?, ?, ?, ?, ?,
               ?, ?, ?, ?, ?, ?, ?, ?)", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;

        AddVarcharParam(cmd, "@RecordKind", 16, record.RecordKind, required: true);
        AddVarcharParam(cmd, "@LogLevel", 16, record.LogLevel, required: true);
        AddVarcharParam(cmd, "@LifecycleStage", 64, record.LifecycleStage);
        AddVarcharParam(cmd, "@LogMessage", 1024, record.LogMessage);
        AddVarcharParam(cmd, "@InteractionId", 64, record.InteractionId);
        AddVarcharParam(cmd, "@IntegrationType", 32, record.IntegrationType);
        AddVarcharParam(cmd, "@Direction", 16, record.Direction);
        AddVarcharParam(cmd, "@CallerSystem", 64, record.CallerSystem);
        AddVarcharParam(cmd, "@TargetSystem", 64, record.TargetSystem);
        AddVarcharParam(cmd, "@WorkflowId", 128, record.WorkflowId);
        AddVarcharParam(cmd, "@EventId", 128, record.EventId);
        AddVarcharParam(cmd, "@LicensePlate", 32, record.LicensePlate);
        AddVarcharParam(cmd, "@CameraIdentifier", 64, record.CameraIdentifier);
        cmd.Parameters.Add("@AttemptNo", DmDbType.Int32).Value = (object?)record.AttemptNo ?? DBNull.Value;
        AddVarcharParam(cmd, "@HttpMethod", 16, record.HttpMethod, required: true);
        AddVarcharParam(cmd, "@RequestPath", 256, record.RequestPath, required: true);
        AddVarcharParam(cmd, "@QueryString", 1024, record.QueryString);
        AddVarcharParam(cmd, "@Scheme", 16, record.Scheme);
        AddVarcharParam(cmd, "@Host", 256, record.Host);
        AddVarcharParam(cmd, "@TraceIdentifier", 128, record.TraceIdentifier);
        AddVarcharParam(cmd, "@ClientIp", 64, record.ClientIp);
        AddVarcharParam(cmd, "@UserAgent", 512, record.UserAgent);
        AddVarcharParam(cmd, "@RequestContentType", 256, record.RequestContentType);
        AddVarcharParam(cmd, "@TimestampHeader", 32, record.TimestampHeader);
        AddVarcharParam(cmd, "@SignatureHeader", 64, record.SignatureHeader);
        AddVarcharParam(cmd, "@IdempotencyKeyHeader", 128, record.IdempotencyKeyHeader);
        AddTextParam(cmd, "@RequestHeadersJson", record.RequestHeadersJson);
        AddTextParam(cmd, "@RequestBody", record.RequestBody);
        AddVarcharParam(cmd, "@ResponseContentType", 256, record.ResponseContentType);
        AddTextParam(cmd, "@ResponseHeadersJson", record.ResponseHeadersJson);
        AddTextParam(cmd, "@ResponseBody", record.ResponseBody);
        cmd.Parameters.Add("@RequestContentLength", DmDbType.Int64).Value = (object?)record.RequestContentLength ?? DBNull.Value;
        cmd.Parameters.Add("@ResponseContentLength", DmDbType.Int64).Value = (object?)record.ResponseContentLength ?? DBNull.Value;
        cmd.Parameters.Add("@HttpStatus", DmDbType.Int32).Value = record.HttpStatus;
        cmd.Parameters.Add("@BusinessCode", DmDbType.Int32).Value = record.BusinessCode;
        cmd.Parameters.Add("@ElapsedMs", DmDbType.Int32).Value = record.ElapsedMs;
        AddVarcharParam(cmd, "@ErrorMessage", 1024, record.ErrorMessage);
        cmd.Parameters.Add("@CreatedAt", DmDbType.DateTime).Value = record.CreatedAt;
        await cmd.ExecuteNonQueryAsync(cancellationToken).ConfigureAwait(false);
    }

    /// <inheritdoc />
    public async Task<int> DeleteApiAccessLogsOlderThanAsync(
        DateTime cutoff,
        int batchSize,
        CancellationToken cancellationToken = default)
    {
        if (batchSize is < 1 or > 1000)
        {
            throw new ArgumentOutOfRangeException(
                nameof(batchSize),
                batchSize,
                "API access log cleanup batch size must be between 1 and 1000.");
        }

        await using var conn = CreateConnection();
        await conn.OpenAsync(cancellationToken).ConfigureAwait(false);
        using var tx = conn.BeginTransaction();

        try
        {
            var ids = new List<long>(batchSize);
            using (var selectCmd = new DmCommand(
                $@"SELECT TOP {batchSize} Id
                   FROM ApiAccessLog
                   WHERE CreatedAt < ?
                   ORDER BY CreatedAt ASC, Id ASC", conn, (DmTransaction)tx))
            {
                selectCmd.CommandTimeout = _commandTimeoutSeconds;
                selectCmd.Parameters.Add("@Cutoff", DmDbType.DateTime).Value = cutoff;

                await using var reader = await selectCmd.ExecuteReaderAsync(cancellationToken).ConfigureAwait(false);
                while (await reader.ReadAsync(cancellationToken).ConfigureAwait(false))
                {
                    ids.Add(Convert.ToInt64(reader.GetValue(0)));
                }
            }

            if (ids.Count == 0)
            {
                tx.Commit();
                return 0;
            }

            var placeholders = string.Join(", ", Enumerable.Repeat("?", ids.Count));
            using var deleteCmd = new DmCommand(
                $"DELETE FROM ApiAccessLog WHERE Id IN ({placeholders})",
                conn,
                (DmTransaction)tx);
            deleteCmd.CommandTimeout = _commandTimeoutSeconds;
            for (var i = 0; i < ids.Count; i++)
            {
                deleteCmd.Parameters.Add($"@Id{i}", DmDbType.Int64).Value = ids[i];
            }

            var deleted = await deleteCmd.ExecuteNonQueryAsync(cancellationToken).ConfigureAwait(false);
            tx.Commit();
            return deleted;
        }
        catch
        {
            TryRollback(tx);
            throw;
        }
    }
}
