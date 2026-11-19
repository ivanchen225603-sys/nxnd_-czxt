using System.Data;
using System.Data.Common;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public partial class IntegrationRepository
{

    /// <inheritdoc />
    public void InsertLicensePlateRecord(LicensePlateRecord record)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var tx = conn.BeginTransaction();
        try
        {
            SaveLicensePlateRecord(conn, (DmTransaction)tx, record, bindWorkflow: true);
            tx.Commit();
        }
        catch
        {
            TryRollback(tx);
            throw;
        }
    }

    /// <inheritdoc />
    public string? GetSucceededWorkflowLicensePlate(string workflowId)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand(
            @"SELECT LicensePlate FROM VehicleArrival
              WHERE SessionId = ? AND PlateStatus = 'SUCCEEDED' AND LicensePlate IS NOT NULL",
            conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        AddVarcharParam(cmd, "@WorkflowId", 128, workflowId, required: true);
        return cmd.ExecuteScalar() as string;
    }

    /// <inheritdoc />
    public string? FindWorkflowIdForMeasurement(
        string licensePlate,
        string cameraIdentifier,
        DateTime measureTime)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand(
            @"SELECT TOP 1 SessionId
              FROM VehicleArrival
              WHERE LicensePlate = ?
                AND REPLACE(UPPER(CameraIdentifier), '_', '') = REPLACE(UPPER(?), '_', '')
                AND PlateStatus = 'SUCCEEDED'
                AND ArrivalTime <= ?
                AND ArrivalTime >= ?
              ORDER BY ArrivalTime DESC, Id DESC",
            conn);
        AddVarcharParam(cmd, "@LicensePlate", 32, licensePlate, required: true);
        AddVarcharParam(cmd, "@CameraIdentifier", 64, cameraIdentifier, required: true);
        cmd.Parameters.Add("@MeasureTime", DmDbType.DateTime).Value = measureTime;
        cmd.Parameters.Add("@WindowStart", DmDbType.DateTime).Value = measureTime.AddHours(-12);
        return cmd.ExecuteScalar() as string;
    }

    private void SaveLicensePlateRecord(
        DmConnection conn,
        DmTransaction tx,
        LicensePlateRecord record,
        bool bindWorkflow)
    {
        if (bindWorkflow &&
            record.IsSuccess &&
            !string.IsNullOrWhiteSpace(record.WorkflowId) &&
            record.CaptureTime.HasValue)
        {
            LockCameraCaptureClaimsAndValidateWorkflow(conn, tx, record);
        }

        long? existingId = null;
        if (!string.IsNullOrWhiteSpace(record.WorkflowId))
        {
            using var existing = new DmCommand(
                "SELECT TOP 1 Id FROM LicensePlateRecord WHERE WorkflowId = ?",
                conn,
                tx);
            AddVarcharParam(existing, "@WorkflowId", 128, record.WorkflowId, required: true);
            var value = existing.ExecuteScalar();
            if (value is not null && value != DBNull.Value)
            {
                existingId = Convert.ToInt64(value);
            }
        }

        var sql = existingId.HasValue
            ? @"UPDATE LicensePlateRecord
                SET InteractionId = ?, CameraIdentifier = ?, CameraId = ?, LicensePlate = ?,
                    CaptureTime = ?, RequestTime = ?, ResponseCode = ?, ResponseMessage = ?,
                    IsSuccess = ?, ErrorMessage = ?, CreatedAt = ?, EnterId = ?
                WHERE Id = ?"
            : @"INSERT INTO LicensePlateRecord
                (WorkflowId, InteractionId, CameraIdentifier, CameraId, LicensePlate, CaptureTime,
                 RequestTime, ResponseCode, ResponseMessage, IsSuccess, ErrorMessage, CreatedAt, EnterId)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)";
        using var cmd = new DmCommand(sql, conn, tx);
        if (!existingId.HasValue)
        {
            AddVarcharParam(cmd, "@WorkflowId", 128, record.WorkflowId);
        }
        AddVarcharParam(cmd, "@InteractionId", 64, record.InteractionId);
        AddVarcharParam(cmd, "@CameraIdentifier", 64, record.CameraIdentifier, required: true);
        AddVarcharParam(cmd, "@CameraId", 64, record.CameraId, required: true);
        AddVarcharParam(cmd, "@LicensePlate", 32, record.LicensePlate, required: true);
        cmd.Parameters.Add("@CaptureTime", DmDbType.DateTime).Value = (object?)record.CaptureTime ?? DBNull.Value;
        cmd.Parameters.Add("@RequestTime", DmDbType.DateTime).Value = record.RequestTime;
        cmd.Parameters.Add("@ResponseCode", DmDbType.Int32).Value = record.ResponseCode;
        AddVarcharParam(cmd, "@ResponseMessage", 512, record.ResponseMessage);
        cmd.Parameters.Add("@IsSuccess", DmDbType.Bit).Value = record.IsSuccess;
        AddVarcharParam(cmd, "@ErrorMessage", 1024, record.ErrorMessage);
        cmd.Parameters.Add("@CreatedAt", DmDbType.DateTime).Value = record.CreatedAt;
        cmd.Parameters.Add("@EnterId", DmDbType.Clob).Value = (object?)record.EnterId ?? DBNull.Value;
        if (existingId.HasValue)
        {
            cmd.Parameters.Add("@Id", DmDbType.Int64).Value = existingId.Value;
        }
        cmd.ExecuteNonQuery();

        if (bindWorkflow && !string.IsNullOrWhiteSpace(record.WorkflowId))
        {
            using var arrival = new DmCommand(
                @"UPDATE VehicleArrival
                  SET LicensePlate = ?, PlateStatus = ?, PlateSource = 'PARTNER',
                      PlateCapturedAt = ?, UpdatedAt = ?
                  WHERE SessionId = ?",
                conn,
                tx);
            AddVarcharParam(arrival, "@LicensePlate", 32, record.IsSuccess ? record.LicensePlate : null);
            AddVarcharParam(
                arrival,
                "@PlateStatus",
                32,
                record.IsSuccess ? VehicleArrivalPlateStatus.Succeeded : VehicleArrivalPlateStatus.Failed,
                required: true);
            arrival.Parameters.Add("@PlateCapturedAt", DmDbType.DateTime).Value =
                record.IsSuccess ? (object?)record.CaptureTime ?? record.CreatedAt : DBNull.Value;
            arrival.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = record.CreatedAt;
            AddVarcharParam(arrival, "@WorkflowId", 128, record.WorkflowId, required: true);
            if (arrival.ExecuteNonQuery() != 1)
            {
                throw new DBConcurrencyException($"Vehicle workflow {record.WorkflowId} no longer exists.");
            }
        }
    }

    private void LockCameraCaptureClaimsAndValidateWorkflow(
        DmConnection conn,
        DmTransaction tx,
        LicensePlateRecord record)
    {
        LockCameraAnchor(conn, tx, record.CameraIdentifier, required: true);

        using (var validateWorkflow = new DmCommand(
                   "SELECT CameraIdentifier, CreatedAt, PlateStatus, LicensePlate FROM VehicleArrival WHERE SessionId = ?",
                   conn,
                   tx))
        {
            AddVarcharParam(validateWorkflow, "@WorkflowId", 128, record.WorkflowId, required: true);
            using var workflow = validateWorkflow.ExecuteReader();
            if (!workflow.Read()) throw new InvalidOperationException("原来车流程不存在");
            var workflowCamera = workflow.GetString(0);
            if (!string.Equals(workflowCamera, record.CameraIdentifier, StringComparison.OrdinalIgnoreCase))
                throw new InvalidOperationException($"Workflow {record.WorkflowId} does not belong to camera {record.CameraIdentifier}.");
            if (workflow.GetString(2) == VehicleArrivalPlateStatus.Succeeded && !workflow.IsDBNull(3) &&
                !string.Equals(workflow.GetString(3), record.LicensePlate, StringComparison.Ordinal))
                throw new InvalidOperationException("原来车已有不同的权威车牌，不能覆盖");
        }

        ValidateCaptureClaim(conn, tx, record);
    }

    internal static void LockCameraAnchor(DmConnection conn, DmTransaction tx, string camera, bool required)
    {
        long anchorId;
        using (var findAnchor = new DmCommand(
                   "SELECT MIN(Id) FROM VehicleArrival WHERE CameraIdentifier = ?",
                   conn,
                   tx))
        {
            AddVarcharParam(findAnchor, "@CameraIdentifier", 64, camera, required: true);
            var value = findAnchor.ExecuteScalar();
            if (value is null or DBNull)
            {
                if (!required) return;
                throw new DBConcurrencyException($"Camera {camera} has no vehicle workflow anchor.");
            }
            anchorId = Convert.ToInt64(value);
        }

        // 同一 camera 的所有车牌完成事务都更新同一个稳定锚点行，从而串行检查抓拍归属。
        // 这避免两个并发 workflow 同时看见“尚未占用”并绑定同一张外部抓拍。
        using (var lockAnchor = new DmCommand(
                   "UPDATE VehicleArrival SET UpdatedAt = UpdatedAt WHERE Id = ?",
                   conn,
                   tx))
        {
            lockAnchor.Parameters.Add("@Id", DmDbType.Int64).Value = anchorId;
            if (lockAnchor.ExecuteNonQuery() != 1)
            {
                throw new DBConcurrencyException(
                    $"Camera {camera} workflow anchor disappeared.");
            }
        }

    }

    private void ValidateCaptureClaim(DmConnection conn, DmTransaction tx, LicensePlateRecord record)
    {
        // 旧库的 CaptureTime 可能是 DATETIME（秒精度），而新库是 DATETIME(3)。
        // 以自然秒为抓拍唯一桶，避免驱动写入时截断毫秒后导致“同一抓拍”漏判。
        var captureSecond = new DateTime(
            record.CaptureTime!.Value.Ticks -
            record.CaptureTime.Value.Ticks % TimeSpan.TicksPerSecond,
            record.CaptureTime.Value.Kind);
        using var existingClaim = new DmCommand(
            @"SELECT TOP 1 WorkflowId FROM LicensePlateRecord
              WHERE CameraIdentifier = ?
                AND ((CaptureTime >= ? AND CaptureTime < ?) OR EnterId = ?)
                AND IsSuccess = 1
                AND WorkflowId IS NOT NULL
                AND WorkflowId <> ?",
            conn,
            tx);
        AddVarcharParam(existingClaim, "@CameraIdentifier", 64, record.CameraIdentifier, required: true);
        existingClaim.Parameters.Add("@CaptureSecond", DmDbType.DateTime).Value = captureSecond;
        existingClaim.Parameters.Add("@NextCaptureSecond", DmDbType.DateTime).Value = captureSecond.AddSeconds(1);
        existingClaim.Parameters.Add("@EnterId", DmDbType.Clob).Value = (object?)record.EnterId ?? DBNull.Value;
        AddVarcharParam(existingClaim, "@WorkflowId", 128, record.WorkflowId, required: true);
        var claimedBy = existingClaim.ExecuteScalar() as string;
        if (!string.IsNullOrWhiteSpace(claimedBy))
        {
            throw new InvalidOperationException(
                $"Plate capture {record.CameraIdentifier}/{record.CaptureTime:O} is already bound to workflow {claimedBy}.");
        }
        ValidateCaptureWindow(conn, tx, record);
    }

    private void ValidateCaptureWindow(DmConnection connection, DmTransaction transaction, LicensePlateRecord record)
    {
        DateTime createdAt;
        using (var current = new DmCommand("SELECT CreatedAt FROM VehicleArrival WHERE SessionId=?", connection, transaction))
        {
            AddVarcharParam(current, "@Workflow", 128, record.WorkflowId, required: true);
            createdAt = (DateTime)current.ExecuteScalar()!;
        }
        var startSecond = createdAt.AddTicks(-(createdAt.Ticks % TimeSpan.TicksPerSecond));
        var lower = startSecond.AddSeconds(-_captureLeadSeconds);
        var upper = createdAt.AddSeconds(_captureWindowSeconds);
        using var neighbors = new DmCommand(
            @"SELECT CreatedAt FROM VehicleArrival WHERE CameraIdentifier=? AND SessionId<>?
              AND CreatedAt>=? AND CreatedAt<=?", connection, transaction);
        neighbors.CommandTimeout = _commandTimeoutSeconds;
        AddVarcharParam(neighbors, "@Camera", 64, record.CameraIdentifier, required: true);
        AddVarcharParam(neighbors, "@Workflow", 128, record.WorkflowId, required: true);
        neighbors.Parameters.Add("@Before", DmDbType.DateTime).Value = createdAt.AddSeconds(-_captureWindowSeconds - _captureLeadSeconds);
        neighbors.Parameters.Add("@After", DmDbType.DateTime).Value = upper.AddSeconds(_captureLeadSeconds);
        using var reader = neighbors.ExecuteReader();
        while (reader.Read())
        {
            var neighbor = reader.GetDateTime(0);
            if (neighbor <= createdAt)
            {
                var exclusiveStart = neighbor.AddSeconds(_captureWindowSeconds).AddTicks(1);
                if (exclusiveStart > lower) lower = exclusiveStart;
            }
            if (neighbor >= createdAt)
            {
                var boundary = neighbor.AddSeconds(-_captureLeadSeconds);
                boundary = boundary.AddTicks(-(boundary.Ticks % TimeSpan.TicksPerSecond)).AddTicks(-1);
                if (boundary < upper) upper = boundary;
            }
        }
        if (record.CaptureTime < lower || record.CaptureTime > upper)
            throw new InvalidOperationException("抓拍不在原来车的独占时间窗口内，保留待确认，禁止串车关联");
    }

    private void BackfillWorkflowLicensePlate(
        DmConnection conn,
        DmTransaction tx,
        string workflowId,
        string licensePlate,
        DateTime updatedAt)
    {
        using (var dimensions = new DmCommand(
                   @"UPDATE VehicleDimensionRecord
                     SET LicensePlate = ?, UpdatedAt = ?
                     WHERE WorkflowId = ?
                       AND (LicensePlate IS NULL OR TRIM(LicensePlate) = '' OR UPPER(LicensePlate) = 'UNKNOWN')",
                   conn,
                   tx))
        {
            AddVarcharParam(dimensions, "@LicensePlate", 32, licensePlate, required: true);
            dimensions.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = updatedAt;
            AddVarcharParam(dimensions, "@WorkflowId", 128, workflowId, required: true);
            dimensions.ExecuteNonQuery();
        }

        using var coal = new DmCommand(
            @"UPDATE CoalVolumeRecord
              SET LicensePlate = ?
              WHERE WorkflowId = ?
                AND (LicensePlate IS NULL OR TRIM(LicensePlate) = '' OR UPPER(LicensePlate) = 'UNKNOWN')",
            conn,
            tx);
        AddVarcharParam(coal, "@LicensePlate", 32, licensePlate, required: true);
        AddVarcharParam(coal, "@WorkflowId", 128, workflowId, required: true);
        coal.ExecuteNonQuery();
    }

    private static string? LockWorkflowAndGetSucceededPlate(
        DmConnection conn,
        DmTransaction tx,
        string? workflowId)
    {
        if (string.IsNullOrWhiteSpace(workflowId))
        {
            return null;
        }

        // 达梦在同一行 UPDATE 上提供写锁。无操作更新用于和车牌完成事务建立明确顺序，
        // 避免依赖不同隔离级别下 UPDATE 对尚未提交 INSERT 的可见性细节。
        using (var lockWorkflow = new DmCommand(
                   "UPDATE VehicleArrival SET UpdatedAt = UpdatedAt WHERE SessionId = ?",
                   conn,
                   tx))
        {
            AddVarcharParam(lockWorkflow, "@WorkflowId", 128, workflowId, required: true);
            lockWorkflow.ExecuteNonQuery();
        }

        using var selectPlate = new DmCommand(
            @"SELECT LicensePlate, PlateStatus
              FROM VehicleArrival
              WHERE SessionId = ?",
            conn,
            tx);
        AddVarcharParam(selectPlate, "@WorkflowId", 128, workflowId, required: true);
        using var reader = selectPlate.ExecuteReader();
        if (!reader.Read() ||
            reader.IsDBNull(0) ||
            !string.Equals(reader.GetString(1), VehicleArrivalPlateStatus.Succeeded, StringComparison.OrdinalIgnoreCase))
        {
            return null;
        }

        var plate = reader.GetString(0).Trim();
        return string.IsNullOrWhiteSpace(plate) ||
               string.Equals(plate, "UNKNOWN", StringComparison.OrdinalIgnoreCase)
            ? null
            : plate;
    }
}
