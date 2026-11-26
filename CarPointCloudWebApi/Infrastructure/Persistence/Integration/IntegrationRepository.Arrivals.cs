using System.Data;
using System.Data.Common;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public partial class IntegrationRepository
{

    /// <inheritdoc />
    public VehicleArrivalInsertResult InsertVehicleArrivalRecord(VehicleArrivalRecord record)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var tx = conn.BeginTransaction();

        try
        {
            LockCameraAnchor(conn, (DmTransaction)tx, record.CameraIdentifier, required: false);
            InsertVehicleArrival(conn, (DmTransaction)tx, record, _commandTimeoutSeconds);
            tx.Commit();
            return new VehicleArrivalInsertResult { Record = record, IsNew = true };
        }
        catch (DmException ex) when (IsUniqueConstraintViolation(ex))
        {
            TryRollback(tx);
            var existing = GetVehicleArrivalByEventId(conn, record.EventId);
            if (existing == null) throw;
            return new VehicleArrivalInsertResult { Record = existing, IsNew = false };
        }
        catch
        {
            TryRollback(tx);
            throw;
        }
    }

    internal static void InsertVehicleArrival(DmConnection conn, DmTransaction tx, VehicleArrivalRecord record, int timeout)
    {
        using var cmd = new DmCommand(
            @"INSERT INTO VehicleArrival
              (EventId, DeviceId, CameraIdentifier, EventType, SessionId, LicensePlate,
               PlateStatus, PlateSource, PlateCapturedAt, ArrivalTime,
               RadarStatus, RadarError, CreatedAt, UpdatedAt)
              VALUES
              (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);
              SELECT SCOPE_IDENTITY()", conn, tx);
        cmd.CommandTimeout = timeout;

        AddVarcharParam(cmd, "@EventId", 128, record.EventId, required: true);
        AddVarcharParam(cmd, "@DeviceId", 64, record.DeviceId, required: true);
        AddVarcharParam(cmd, "@CameraIdentifier", 64, record.CameraIdentifier, required: true);
        AddVarcharParam(cmd, "@EventType", 32, record.EventType, required: true);
        AddVarcharParam(cmd, "@SessionId", 128, record.SessionId, required: true);
        AddVarcharParam(cmd, "@LicensePlate", 32, record.LicensePlate);
        AddVarcharParam(cmd, "@PlateStatus", 32, record.PlateStatus, required: true);
        AddVarcharParam(cmd, "@PlateSource", 32, record.PlateSource);
        cmd.Parameters.Add("@PlateCapturedAt", DmDbType.DateTime).Value =
            (object?)record.PlateCapturedAt ?? DBNull.Value;
        cmd.Parameters.Add("@ArrivalTime", DmDbType.DateTime).Value = record.ArrivalTime;
        AddVarcharParam(cmd, "@RadarStatus", 32, record.RadarStatus, required: true);
        AddVarcharParam(cmd, "@RadarError", 1024, record.RadarError);
        cmd.Parameters.Add("@CreatedAt", DmDbType.DateTime).Value = record.CreatedAt;
        cmd.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = record.UpdatedAt;

        record.Id = Convert.ToInt64(cmd.ExecuteScalar());
        if (string.Equals(record.PlateStatus, VehicleArrivalPlateStatus.Pending, StringComparison.Ordinal))
        {
            using var jobCmd = new DmCommand(
                @"INSERT INTO PlateAcquisitionJob
                  (WorkflowId, InteractionId, CameraIdentifier, Status, AttemptCount,
                   NextAttemptAt, LastError, CreatedAt, UpdatedAt)
                  VALUES (?, ?, ?, 'PENDING', 0, ?, NULL, ?, ?)",
                conn,
                tx)
            { CommandTimeout = timeout };
            AddVarcharParam(jobCmd, "@WorkflowId", 128, record.SessionId, required: true);
            AddVarcharParam(jobCmd, "@InteractionId", 64, Guid.NewGuid().ToString("N"), required: true);
            AddVarcharParam(jobCmd, "@CameraIdentifier", 64, record.CameraIdentifier, required: true);
            jobCmd.Parameters.Add("@NextAttemptAt", DmDbType.DateTime).Value = record.CreatedAt;
            jobCmd.Parameters.Add("@CreatedAt", DmDbType.DateTime).Value = record.CreatedAt;
            jobCmd.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = record.UpdatedAt;
            jobCmd.ExecuteNonQuery();
        }
    }

    /// <inheritdoc />
    public void UpdateVehicleArrivalRadarResult(
        long recordId,
        string radarStatus,
        string? radarError,
        DateTime updatedAt)
    {
        using var conn = CreateConnection();
        conn.Open();

        using var cmd = new DmCommand(
            @"UPDATE VehicleArrival
              SET RadarStatus = ?, RadarError = ?, UpdatedAt = ?
              WHERE Id = ?", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        AddVarcharParam(cmd, "@RadarStatus", 32, radarStatus, required: true);
        AddVarcharParam(cmd, "@RadarError", 1024, radarError);
        cmd.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = updatedAt;
        cmd.Parameters.Add("@Id", DmDbType.Int64).Value = recordId;

        if (cmd.ExecuteNonQuery() != 1)
        {
            throw new DBConcurrencyException($"VehicleArrival record {recordId} no longer exists.");
        }
    }

    /// <inheritdoc />
    /// <inheritdoc />
    public VehicleArrivalRecord? FindVehicleArrivalByEventId(string eventId)
    {
        if (string.IsNullOrWhiteSpace(eventId))
        {
            return null;
        }

        using var conn = CreateConnection();
        conn.Open();
        return GetVehicleArrivalByEventId(conn, eventId.Trim());
    }

    /// <inheritdoc />
    public bool TryClaimVehicleArrivalAutoScan(long recordId, DateTime updatedAt)
    {
        using var conn = CreateConnection();
        conn.Open();

        using var cmd = new DmCommand(
            @"UPDATE VehicleArrival
              SET RadarStatus = ?, RadarError = NULL, UpdatedAt = ?
              WHERE Id = ? AND RadarStatus = ?", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        AddVarcharParam(cmd, "@NewStatus", 32, VehicleArrivalRadarStatus.Pending, required: true);
        cmd.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = updatedAt;
        cmd.Parameters.Add("@Id", DmDbType.Int64).Value = recordId;
        AddVarcharParam(cmd, "@NotRequired", 32, VehicleArrivalRadarStatus.NotRequired, required: true);
        return cmd.ExecuteNonQuery() == 1;
    }

    /// <inheritdoc />
    public bool DeleteUndispatchedManualArrival(long recordId)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand(
            "DELETE FROM VehicleArrival WHERE Id=? AND PlateSource='MANUAL' AND RadarStatus='PENDING'", conn)
        { CommandTimeout = _commandTimeoutSeconds };
        cmd.Parameters.Add("@Id", DmDbType.Int64).Value = recordId;
        return cmd.ExecuteNonQuery() == 1;
    }

    private VehicleArrivalRecord? GetVehicleArrivalByEventId(DmConnection conn, string eventId) =>
        ReadVehicleArrival(conn, null, eventId, _commandTimeoutSeconds);

    internal static VehicleArrivalRecord? ReadVehicleArrival(DmConnection conn, DmTransaction? transaction, string eventId, int timeout)
    {
        using var cmd = new DmCommand(
            @"SELECT Id, EventId, DeviceId, CameraIdentifier, EventType, SessionId,
                     LicensePlate, PlateStatus, PlateSource, PlateCapturedAt, ArrivalTime,
                     RadarStatus, RadarError, CreatedAt, UpdatedAt
              FROM VehicleArrival
              WHERE EventId = ?", conn, transaction);
        cmd.CommandTimeout = timeout;
        AddVarcharParam(cmd, "@EventId", 128, eventId, required: true);

        using var reader = cmd.ExecuteReader();
        if (!reader.Read())
        {
            return null;
        }

        return MapVehicleArrival(reader);
    }

    /// <inheritdoc />
    public VehicleArrivalRecord? GetVehicleArrivalBySessionId(string sessionId)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var cmd = new DmCommand(
            @"SELECT Id, EventId, DeviceId, CameraIdentifier, EventType, SessionId,
                     LicensePlate, PlateStatus, PlateSource, PlateCapturedAt, ArrivalTime,
                     RadarStatus, RadarError, CreatedAt, UpdatedAt
              FROM VehicleArrival WHERE SessionId = ?", conn);
        cmd.CommandTimeout = _commandTimeoutSeconds;
        AddVarcharParam(cmd, "@SessionId", 128, sessionId, required: true);
        using var reader = cmd.ExecuteReader();
        return reader.Read() ? MapVehicleArrival(reader) : null;
    }
}
