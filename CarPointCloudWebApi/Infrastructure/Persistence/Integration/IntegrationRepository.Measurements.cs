using System.Data;
using System.Data.Common;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public partial class IntegrationRepository
{

    /// <inheritdoc />
    public List<VehicleDimensionRecord> GetVehicleDimensionsPage(
        int take,
        DateTime? windowStart,
        DateTime? beforeTime,
        long? beforeId)
    {
        ValidatePageArguments(ref take, beforeTime, beforeId);
        using var conn = CreateConnection();
        conn.Open();

        var filters = new List<string>();
        if (windowStart.HasValue)
        {
            filters.Add("GetTime >= ?");
        }

        if (beforeTime.HasValue)
        {
            filters.Add("(GetTime < ? OR (GetTime = ? AND Id < ?))");
        }

        var where = filters.Count == 0 ? string.Empty : $"WHERE {string.Join(" AND ", filters)}";
        var records = new List<VehicleDimensionRecord>();
        using var cmd = new DmCommand(
            $@"SELECT TOP {take} {VehicleDimensionColumns}
               FROM VehicleDimensionRecord
               {where}
               ORDER BY GetTime DESC, Id DESC", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        if (windowStart.HasValue)
        {
            cmd.Parameters.Add("@WindowStart", DmDbType.DateTime).Value = windowStart.Value;
        }

        if (beforeTime.HasValue)
        {
            cmd.Parameters.Add("@BeforeTime1", DmDbType.DateTime).Value = beforeTime.Value;
            cmd.Parameters.Add("@BeforeTime2", DmDbType.DateTime).Value = beforeTime.Value;
            cmd.Parameters.Add("@BeforeId", DmDbType.Int64).Value = beforeId!.Value;
        }

        using (var reader = cmd.ExecuteReader())
        {
            while (reader.Read())
            {
                records.Add(MapVehicleDimension(reader));
            }
        }

        LoadDimensionDetails(conn, records);
        return records;
    }

    /// <inheritdoc />
    public VehicleDimensionRecord? GetVehicleDimensionByWorkflowId(string workflowId)
    {
        if (string.IsNullOrWhiteSpace(workflowId))
        {
            return null;
        }

        using var conn = CreateConnection();
        conn.Open();

        var records = new List<VehicleDimensionRecord>();
        using (var cmd = new DmCommand(
                   $@"SELECT TOP 1 {VehicleDimensionColumns}
                      FROM VehicleDimensionRecord
                      WHERE WorkflowId = ?
                      ORDER BY Id DESC", conn))
        {
            cmd.CommandTimeout = _commandTimeoutSeconds;
            AddVarcharParam(cmd, "@WorkflowId", 128, workflowId.Trim(), required: true);
            using var reader = cmd.ExecuteReader();
            while (reader.Read())
            {
                records.Add(MapVehicleDimension(reader));
            }
        }

        if (records.Count == 0)
        {
            return null;
        }

        LoadDimensionDetails(conn, records);
        return records[0];
    }

    /// <inheritdoc />
    public List<CoalVolumeRecord> GetCoalVolumesPage(
        int take,
        DateTime? windowStart,
        DateTime? beforeTime,
        long? beforeId)
    {
        ValidatePageArguments(ref take, beforeTime, beforeId);
        using var conn = CreateConnection();
        conn.Open();

        var filters = new List<string>();
        if (windowStart.HasValue)
        {
            filters.Add("MeasureTime >= ?");
        }

        if (beforeTime.HasValue)
        {
            filters.Add("(MeasureTime < ? OR (MeasureTime = ? AND Id < ?))");
        }

        var where = filters.Count == 0 ? string.Empty : $"WHERE {string.Join(" AND ", filters)}";
        var records = new List<CoalVolumeRecord>();
        using var cmd = new DmCommand(
            $@"SELECT TOP {take} {CoalVolumeColumns}
               FROM CoalVolumeRecord
               {where}
               ORDER BY MeasureTime DESC, Id DESC", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        if (windowStart.HasValue)
        {
            cmd.Parameters.Add("@WindowStart", DmDbType.DateTime).Value = windowStart.Value;
        }

        if (beforeTime.HasValue)
        {
            cmd.Parameters.Add("@BeforeTime1", DmDbType.DateTime).Value = beforeTime.Value;
            cmd.Parameters.Add("@BeforeTime2", DmDbType.DateTime).Value = beforeTime.Value;
            cmd.Parameters.Add("@BeforeId", DmDbType.Int64).Value = beforeId!.Value;
        }

        using var reader = cmd.ExecuteReader();
        while (reader.Read())
        {
            records.Add(MapCoalVolume(reader));
        }

        return records;
    }

    /// <inheritdoc />
    public async Task<long> InsertVehicleDimensionRecordAsync(
        VehicleDimensionRecord record,
        CancellationToken cancellationToken = default)
    {
        var existing = GetExistingVehicleDimensionIdentity(record.IdempotencyKey);
        if (existing is not null)
        {
            return ResolveIdempotency(existing, record.PayloadHash, "车辆尺寸");
        }

        var existingBusiness = GetExistingVehicleDimensionBusinessIdentity(record.WorkflowId, record.PairId);
        if (existingBusiness is not null)
        {
            return ResolveBusinessIdentity(existingBusiness, record.PayloadHash, "车辆尺寸");
        }

        using var conn = CreateConnection();
        conn.Open();
        using var tx = conn.BeginTransaction();

        try
        {
            // 与车牌后台任务在同一工作流行上串行化。否则可能出现车牌回填先扫描完、
            // 尺寸记录随后才以 UNKNOWN 提交的交叉时序，导致两张表永久不一致。
            var authoritativePlate = LockWorkflowAndGetSucceededPlate(
                conn,
                (DmTransaction)tx,
                record.WorkflowId);
            if (!string.IsNullOrWhiteSpace(authoritativePlate))
            {
                record.LicensePlate = authoritativePlate;
            }

            using var cmd = new DmCommand(
                @"INSERT INTO VehicleDimensionRecord
                   (IdempotencyKey, WorkflowId, PairId, PayloadHash, LicensePlate, CameraIdentifier, GetTime, DetectStatus, CabToTrailerFrontDist, TrailerLength,
                    TrailerWidth, TrailerBodyHeight, TrailerBottomToGroundHeight, HighLowPlateHeight,
                    CreatedAt, UpdatedAt, SourceFile, ResultJsonPath, ResultImagePath, ActualRibCount, MaximumWidthCm)
                   VALUES
                   (?, ?, ?, ?, ?, ?, ?, ?, ?, ?,
                    ?, ?, ?, ?,
                    ?, ?, ?, ?, ?, ?, ?);
                   SELECT SCOPE_IDENTITY()", conn, (DmTransaction)tx);

            AddVarcharParam(cmd, "@IdempotencyKey", 64, record.IdempotencyKey, required: true);
            AddVarcharParam(cmd, "@WorkflowId", 128, record.WorkflowId);
            AddVarcharParam(cmd, "@PairId", 64, record.PairId);
            AddVarcharParam(cmd, "@PayloadHash", 64, record.PayloadHash, required: true);
            AddVarcharParam(cmd, "@LicensePlate", 32, record.LicensePlate, required: true);
            AddVarcharParam(cmd, "@CameraIdentifier", 64, record.CameraIdentifier, required: true);
            cmd.Parameters.Add("@GetTime", DmDbType.DateTime).Value = record.GetTime;
            AddVarcharParam(cmd, "@DetectStatus", 32, record.DetectStatus, required: true);
            AddNullableDecimal(cmd, "@CabToTrailerFrontDist", record.CabToTrailerFrontDist);
            AddNullableDecimal(cmd, "@TrailerLength", record.TrailerLength);
            AddNullableDecimal(cmd, "@TrailerWidth", record.TrailerWidth);
            AddNullableDecimal(cmd, "@TrailerBodyHeight", record.TrailerBodyHeight);
            AddNullableDecimal(cmd, "@TrailerBottomToGroundHeight", record.TrailerBottomToGroundHeight);
            AddNullableDecimal(cmd, "@HighLowPlateHeight", record.HighLowPlateHeight);
            cmd.Parameters.Add("@CreatedAt", DmDbType.DateTime).Value = record.CreatedAt;
            cmd.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = record.UpdatedAt;
            AddVarcharParam(cmd, "@SourceFile", 512, record.SourceFile);
            AddVarcharParam(cmd, "@ResultJsonPath", 512, record.ResultJsonPath);
            AddVarcharParam(cmd, "@ResultImagePath", 512, record.ResultImagePath);
            cmd.Parameters.Add("@ActualRibCount", DmDbType.Int32).Value = (object?)record.ActualRibCount ?? DBNull.Value;
            AddNullableDecimal(cmd, "@MaximumWidthCm", record.MaximumWidthCm);

            var id = Convert.ToInt64(cmd.ExecuteScalar());

            foreach (var rib in record.Ribs.OrderBy(x => x.RibId))
            {
                using var ribCmd = new DmCommand(
                    @"INSERT INTO VehicleDimensionRib (DimensionRecordId, RibId, DistToPrev)
                      VALUES (?, ?, ?)", conn, (DmTransaction)tx);
                ribCmd.Parameters.Add("@DimensionRecordId", DmDbType.Int64).Value = id;
                ribCmd.Parameters.Add("@RibId", DmDbType.Int32).Value = rib.RibId;
                AddNullableDecimal(ribCmd, "@DistToPrev", rib.DistToPrev);
                ribCmd.ExecuteNonQuery();
            }

            foreach (var wire in record.Wires.OrderBy(x => x.WireId))
            {
                using var wireCmd = new DmCommand(
                    @"INSERT INTO VehicleDimensionWire (DimensionRecordId, WireId, DistToPrev)
                      VALUES (?, ?, ?)", conn, (DmTransaction)tx);
                wireCmd.Parameters.Add("@DimensionRecordId", DmDbType.Int64).Value = id;
                wireCmd.Parameters.Add("@WireId", DmDbType.Int32).Value = wire.WireId;
                AddNullableDecimal(wireCmd, "@DistToPrev", wire.DistToPrev);
                wireCmd.ExecuteNonQuery();
            }

            foreach (var plate in record.HighLowPlates.OrderBy(x => x.PlateId))
            {
                using var plateCmd = new DmCommand(
                    @"INSERT INTO VehicleDimensionHighLowPlate (DimensionRecordId, PlateId, HighestToGround, LowestToGround, LengthCm)
                      VALUES (?, ?, ?, ?, ?)", conn, (DmTransaction)tx);
                plateCmd.Parameters.Add("@DimensionRecordId", DmDbType.Int64).Value = id;
                plateCmd.Parameters.Add("@PlateId", DmDbType.Int32).Value = plate.PlateId;
                AddNullableDecimal(plateCmd, "@HighestToGround", plate.HighestToGround);
                AddNullableDecimal(plateCmd, "@LowestToGround", plate.LowestToGround);
                AddNullableDecimal(plateCmd, "@LengthCm", plate.LengthCm);
                plateCmd.ExecuteNonQuery();
            }

            tx.Commit();
            return id;
        }
        catch (DmException ex) when (IsUniqueConstraintViolation(ex))
        {
            TryRollback(tx);
            var raced = await WaitForExistingRecordAsync(() =>
                GetExistingVehicleDimensionIdentity(record.IdempotencyKey) ??
                GetExistingVehicleDimensionBusinessIdentity(record.WorkflowId, record.PairId),
                cancellationToken).ConfigureAwait(false)
                ?? throw new InvalidOperationException("Idempotency key already exists but the existing vehicle dimension record was not found.", ex);
            return ResolveBusinessIdentity(raced, record.PayloadHash, "车辆尺寸");
        }
    }

    /// <inheritdoc />
    public async Task<long> InsertCoalVolumeRecordAsync(
        CoalVolumeRecord record,
        CancellationToken cancellationToken = default)
    {
        var existing = GetExistingCoalVolumeIdentity(record.IdempotencyKey);
        if (existing is not null)
        {
            return ResolveIdempotency(existing, record.PayloadHash, "煤炭体积");
        }

        var existingBusiness = GetExistingCoalVolumeBusinessIdentity(record.BusinessKey);
        if (existingBusiness is not null)
        {
            return ResolveBusinessIdentity(existingBusiness, record.PayloadHash, "煤炭体积");
        }

        using var conn = CreateConnection();
        conn.Open();

        using var cmd = new DmCommand(
            @"INSERT INTO CoalVolumeRecord
              (IdempotencyKey, BusinessKey, WorkflowId, PayloadHash, LicensePlate, CameraIdentifier, Volume, Unit, MeasureTime, LicensePlateRecordId,
               Status, Confidence, SourceFile, BaselineFile, CreatedAt, MeasurementStartedAt, MeasurementEndedAt)
              VALUES
              (?, ?, ?, ?, ?, ?, ?, ?, ?, ?,
               ?, ?, ?, ?, ?, ?, ?);
              SELECT SCOPE_IDENTITY()", conn);

        AddVarcharParam(cmd, "@IdempotencyKey", 64, record.IdempotencyKey, required: true);
        AddVarcharParam(cmd, "@BusinessKey", 64, record.BusinessKey, required: true);
        AddVarcharParam(cmd, "@WorkflowId", 128, record.WorkflowId);
        AddVarcharParam(cmd, "@PayloadHash", 64, record.PayloadHash, required: true);
        AddVarcharParam(cmd, "@LicensePlate", 32, record.LicensePlate, required: true);
        AddVarcharParam(cmd, "@CameraIdentifier", 64, record.CameraIdentifier, required: true);
        cmd.Parameters.Add("@Volume", DmDbType.Decimal).Value = record.Volume;
        cmd.Parameters["@Volume"].Precision = 10;
        cmd.Parameters["@Volume"].Scale = 1;
        AddVarcharParam(cmd, "@Unit", 16, record.Unit, required: true);
        cmd.Parameters.Add("@MeasureTime", DmDbType.DateTime).Value = record.MeasureTime;
        cmd.Parameters.Add("@LicensePlateRecordId", DmDbType.Int64).Value = (object?)record.LicensePlateRecordId ?? DBNull.Value;
        AddVarcharParam(cmd, "@Status", 32, record.Status, required: true);
        AddNullableDecimal(cmd, "@Confidence", record.Confidence, precision: 5, scale: 2);
        AddVarcharParam(cmd, "@SourceFile", 512, record.SourceFile);
        AddVarcharParam(cmd, "@BaselineFile", 512, record.BaselineFile);
        cmd.Parameters.Add("@CreatedAt", DmDbType.DateTime).Value = record.CreatedAt;
        cmd.Parameters.Add("@MeasurementStartedAt", DmDbType.DateTime).Value = (object?)record.MeasurementStartedAt ?? DBNull.Value;
        cmd.Parameters.Add("@MeasurementEndedAt", DmDbType.DateTime).Value = (object?)record.MeasurementEndedAt ?? DBNull.Value;

        try
        {
            return Convert.ToInt64(cmd.ExecuteScalar());
        }
        catch (DmException ex) when (IsUniqueConstraintViolation(ex))
        {
            var raced = await WaitForExistingRecordAsync(() =>
                GetExistingCoalVolumeIdentity(record.IdempotencyKey) ??
                GetExistingCoalVolumeBusinessIdentity(record.BusinessKey),
                cancellationToken).ConfigureAwait(false)
                ?? throw new InvalidOperationException("Idempotency key already exists but the existing coal volume record was not found.", ex);
            return ResolveBusinessIdentity(raced, record.PayloadHash, "煤炭体积");
        }
    }

    private static async Task<IdempotencyIdentity?> WaitForExistingRecordAsync(
        Func<IdempotencyIdentity?> lookup,
        CancellationToken cancellationToken)
    {
        for (var attempt = 0; attempt < 5; attempt++)
        {
            var identity = lookup();
            if (identity is not null)
            {
                return identity;
            }

            // 唯一键并发竞争时给另一事务最多 300ms 提交时间，但不占用 ASP.NET 请求线程。
            await Task.Delay(20 * (attempt + 1), cancellationToken).ConfigureAwait(false);
        }

        return null;
    }

    private IdempotencyIdentity? GetExistingVehicleDimensionIdentity(string idempotencyKey)
    {
        if (string.IsNullOrWhiteSpace(idempotencyKey))
        {
            return null;
        }

        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand("SELECT TOP 1 Id, PayloadHash FROM VehicleDimensionRecord WHERE IdempotencyKey = ?", conn);
        AddVarcharParam(cmd, "@IdempotencyKey", 64, idempotencyKey, required: true);
        using var reader = cmd.ExecuteReader();
        return reader.Read()
            ? new IdempotencyIdentity(reader.GetInt64(0), reader.IsDBNull(1) ? null : reader.GetString(1))
            : null;
    }

    private IdempotencyIdentity? GetExistingCoalVolumeIdentity(string idempotencyKey)
    {
        if (string.IsNullOrWhiteSpace(idempotencyKey))
        {
            return null;
        }

        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand("SELECT TOP 1 Id, PayloadHash FROM CoalVolumeRecord WHERE IdempotencyKey = ?", conn);
        AddVarcharParam(cmd, "@IdempotencyKey", 64, idempotencyKey, required: true);
        using var reader = cmd.ExecuteReader();
        return reader.Read()
            ? new IdempotencyIdentity(reader.GetInt64(0), reader.IsDBNull(1) ? null : reader.GetString(1))
            : null;
    }

    private IdempotencyIdentity? GetExistingVehicleDimensionBusinessIdentity(
        string? workflowId,
        string? pairId)
    {
        if (string.IsNullOrWhiteSpace(workflowId) && string.IsNullOrWhiteSpace(pairId))
        {
            return null;
        }

        using var conn = CreateConnection();
        conn.Open();
        if (!string.IsNullOrWhiteSpace(workflowId))
        {
            using var workflow = new DmCommand(
                "SELECT TOP 1 Id, PayloadHash FROM VehicleDimensionRecord WHERE WorkflowId = ?",
                conn);
            AddVarcharParam(workflow, "@WorkflowId", 128, workflowId, required: true);
            using var reader = workflow.ExecuteReader();
            if (reader.Read())
            {
                return new IdempotencyIdentity(
                    reader.GetInt64(0),
                    reader.IsDBNull(1) ? null : reader.GetString(1));
            }
        }

        if (!string.IsNullOrWhiteSpace(pairId))
        {
            using var pair = new DmCommand(
                "SELECT TOP 1 Id, PayloadHash FROM VehicleDimensionRecord WHERE PairId = ?",
                conn);
            AddVarcharParam(pair, "@PairId", 64, pairId, required: true);
            using var reader = pair.ExecuteReader();
            if (reader.Read())
            {
                return new IdempotencyIdentity(
                    reader.GetInt64(0),
                    reader.IsDBNull(1) ? null : reader.GetString(1));
            }
        }

        return null;
    }

    private IdempotencyIdentity? GetExistingCoalVolumeBusinessIdentity(string businessKey)
    {
        if (string.IsNullOrWhiteSpace(businessKey))
        {
            return null;
        }

        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand(
            "SELECT TOP 1 Id, PayloadHash FROM CoalVolumeRecord WHERE BusinessKey = ?",
            conn);
        AddVarcharParam(cmd, "@BusinessKey", 64, businessKey, required: true);
        using var reader = cmd.ExecuteReader();
        return reader.Read()
            ? new IdempotencyIdentity(reader.GetInt64(0), reader.IsDBNull(1) ? null : reader.GetString(1))
            : null;
    }

    private static long ResolveIdempotency(
        IdempotencyIdentity existing,
        string? incomingPayloadHash,
        string entityName)
    {
        if (string.IsNullOrWhiteSpace(incomingPayloadHash))
        {
            throw new InvalidOperationException($"{entityName}载荷指纹未生成");
        }

        if (string.IsNullOrEmpty(existing.PayloadHash) ||
            string.Equals(existing.PayloadHash, incomingPayloadHash, StringComparison.Ordinal))
        {
            return existing.Id;
        }

        throw new IdempotencyConflictException($"相同 idempotencyKey 已用于不同的{entityName}载荷");
    }

    private static long ResolveBusinessIdentity(
        IdempotencyIdentity existing,
        string? incomingPayloadHash,
        string entityName)
    {
        if (string.IsNullOrWhiteSpace(incomingPayloadHash))
        {
            throw new InvalidOperationException($"{entityName}载荷指纹未生成");
        }

        if (string.IsNullOrEmpty(existing.PayloadHash) ||
            string.Equals(existing.PayloadHash, incomingPayloadHash, StringComparison.Ordinal))
        {
            return existing.Id;
        }

        throw new IdempotencyConflictException($"相同业务测量已用于不同的{entityName}载荷");
    }
}
