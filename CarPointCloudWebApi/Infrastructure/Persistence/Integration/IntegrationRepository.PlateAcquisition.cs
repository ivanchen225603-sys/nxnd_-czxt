using System.Data;
using System.Data.Common;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public partial class IntegrationRepository
{

    /// <inheritdoc />
    public PlateAcquisitionJob? TryClaimDuePlateAcquisitionJob(DateTime now, DateTime staleBefore)
    {
        using var conn = CreateConnection();
        conn.Open();

        PlateAcquisitionJob? candidate;
        using (var select = new DmCommand(
            @"SELECT TOP 1 Id, WorkflowId, InteractionId, CameraIdentifier, Status,
                     AttemptCount, NextAttemptAt, LastError, CreatedAt, UpdatedAt, CandidateJson
              FROM PlateAcquisitionJob
              WHERE ((Status IN ('PENDING', 'RETRY') AND NextAttemptAt <= ?)
                     OR (Status = 'PROCESSING' AND UpdatedAt < ?))
              ORDER BY NextAttemptAt, Id",
            conn))
        {
            select.Parameters.Add("@Now", DmDbType.DateTime).Value = now;
            select.Parameters.Add("@StaleBefore", DmDbType.DateTime).Value = staleBefore;
            using var reader = select.ExecuteReader();
            candidate = reader.Read() ? MapPlateAcquisitionJob(reader) : null;
        }

        if (candidate is null)
        {
            return null;
        }

        using var claimTransaction = conn.BeginTransaction();
        using var claim = new DmCommand(
            @"UPDATE PlateAcquisitionJob
              SET Status = 'PROCESSING', UpdatedAt = ?
              WHERE Id = ?
                AND ((Status IN ('PENDING', 'RETRY') AND NextAttemptAt <= ?)
                     OR (Status = 'PROCESSING' AND UpdatedAt < ?))",
            conn, (DmTransaction)claimTransaction);
        claim.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = now;
        claim.Parameters.Add("@Id", DmDbType.Int64).Value = candidate.Id;
        claim.Parameters.Add("@Now", DmDbType.DateTime).Value = now;
        claim.Parameters.Add("@StaleBefore", DmDbType.DateTime).Value = staleBefore;
        if (claim.ExecuteNonQuery() != 1)
        {
            return null;
        }

        using (var snapshot = new DmCommand(
            @"SELECT Id, WorkflowId, InteractionId, CameraIdentifier, Status, AttemptCount,
                     NextAttemptAt, LastError, CreatedAt, UpdatedAt, CandidateJson
              FROM PlateAcquisitionJob WHERE Id=?", conn, (DmTransaction)claimTransaction))
        {
            snapshot.Parameters.Add("@Id", DmDbType.Int64).Value = candidate.Id;
            using var reader = snapshot.ExecuteReader();
            if (!reader.Read()) throw new DBConcurrencyException("已领取的识牌任务不存在");
            candidate = MapPlateAcquisitionJob(reader);
        }
        claimTransaction.Commit();
        return candidate;
    }

    public bool TryBeginPlateAcquisitionAttempt(PlateAcquisitionJob job, int maxAttempts, DateTime now)
    {
        if (job.AttemptCount >= maxAttempts) return false;
        using var conn = CreateConnection();
        conn.Open();
        using var transaction = conn.BeginTransaction();
        using var command = new DmCommand(
            @"UPDATE PlateAcquisitionJob SET AttemptCount=AttemptCount+1, UpdatedAt=?
              WHERE Id=? AND Status='PROCESSING' AND AttemptCount=? AND AttemptCount<? AND UpdatedAt=?",
            conn, (DmTransaction)transaction);
        command.CommandTimeout = _commandTimeoutSeconds;
        command.Parameters.Add("@Now", DmDbType.DateTime).Value = now;
        command.Parameters.Add("@Id", DmDbType.Int64).Value = job.Id;
        command.Parameters.Add("@Expected", DmDbType.Int32).Value = job.AttemptCount;
        command.Parameters.Add("@Maximum", DmDbType.Int32).Value = maxAttempts;
        command.Parameters.Add("@Lease", DmDbType.DateTime).Value = job.UpdatedAt;
        if (command.ExecuteNonQuery() != 1) return false;
        var updatedAt = ReadPlateJobUpdatedAt(conn, (DmTransaction)transaction, job.Id);
        transaction.Commit();
        job.AttemptCount++;
        job.UpdatedAt = updatedAt;
        return true;
    }

    private static DateTime ReadPlateJobUpdatedAt(DmConnection connection, DmTransaction transaction, long id)
    {
        using var read = new DmCommand("SELECT UpdatedAt FROM PlateAcquisitionJob WHERE Id=?", connection, transaction);
        read.Parameters.Add("@Id", DmDbType.Int64).Value = id;
        return (DateTime)read.ExecuteScalar()!;
    }

    public bool SavePlateAcquisitionProgress(PlateAcquisitionJob job, string? candidateJson,
        DateTime nextAttemptAt, string? error, DateTime now)
    {
        using var connection = CreateConnection();
        connection.Open();
        using var command = new DmCommand(
            @"UPDATE PlateAcquisitionJob SET Status='RETRY', CandidateJson=?, NextAttemptAt=?, LastError=?, UpdatedAt=?
              WHERE Id=? AND Status='PROCESSING' AND AttemptCount=? AND UpdatedAt=?", connection);
        command.CommandTimeout = _commandTimeoutSeconds;
        command.Parameters.Add("@Candidate", DmDbType.Clob).Value = (object?)candidateJson ?? DBNull.Value;
        command.Parameters.Add("@Next", DmDbType.DateTime).Value = nextAttemptAt;
        AddVarcharParam(command, "@Error", 1024, error);
        command.Parameters.Add("@Now", DmDbType.DateTime).Value = now;
        command.Parameters.Add("@Id", DmDbType.Int64).Value = job.Id;
        AddPlateJobFence(command, job);
        return command.ExecuteNonQuery() == 1;
    }

    private static void AddPlateJobFence(DmCommand command, PlateAcquisitionJob job)
    {
        command.Parameters.Add("@ExpectedAttempts", DmDbType.Int32).Value = job.AttemptCount;
        command.Parameters.Add("@ClaimedAt", DmDbType.DateTime).Value = job.UpdatedAt;
    }

    /// <inheritdoc />
    public void CompletePlateAcquisitionJob(PlateAcquisitionJob job, LicensePlateRecord record)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var tx = conn.BeginTransaction();
        try
        {
            if (!LockPlateJobLease(conn, (DmTransaction)tx, job))
                throw new DBConcurrencyException($"Plate acquisition job {job.Id} is no longer claimed.");
            record.WorkflowId = job.WorkflowId;
            record.InteractionId = job.InteractionId;
            SaveLicensePlateRecord(conn, (DmTransaction)tx, record, bindWorkflow: true);

            // 分析可能早于车牌识别完成并以 UNKNOWN 入库。车牌任务完成时在同一事务
            // 回填所有已存在的 workflow 数据，避免接口查询看到互相矛盾的车辆身份。
            BackfillWorkflowLicensePlate(
                conn,
                (DmTransaction)tx,
                job.WorkflowId,
                record.LicensePlate,
                record.CreatedAt);

            using var complete = new DmCommand(
                @"UPDATE PlateAcquisitionJob
                  SET Status = 'SUCCEEDED', CandidateJson=NULL,
                      NextAttemptAt = ?, LastError = NULL, UpdatedAt = ?
                  WHERE Id = ? AND Status = 'PROCESSING' AND AttemptCount=? AND UpdatedAt=?",
                conn,
                (DmTransaction)tx);
            complete.Parameters.Add("@NextAttemptAt", DmDbType.DateTime).Value = record.CreatedAt;
            complete.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = record.CreatedAt;
            complete.Parameters.Add("@Id", DmDbType.Int64).Value = job.Id;
            AddPlateJobFence(complete, job);
            if (complete.ExecuteNonQuery() != 1)
            {
                throw new DBConcurrencyException($"Plate acquisition job {job.Id} is no longer claimed.");
            }

            tx.Commit();
        }
        catch
        {
            TryRollback(tx);
            throw;
        }
    }

    private static bool LockPlateJobLease(DmConnection connection, DmTransaction transaction, PlateAcquisitionJob job)
    {
        using var command = new DmCommand(
            "UPDATE PlateAcquisitionJob SET UpdatedAt=UpdatedAt WHERE Id=? AND Status='PROCESSING' AND AttemptCount=? AND UpdatedAt=?",
            connection, transaction);
        command.Parameters.Add("@Id", DmDbType.Int64).Value = job.Id;
        AddPlateJobFence(command, job);
        return command.ExecuteNonQuery() == 1;
    }

    /// <inheritdoc />
    public void FailPlateAcquisitionJob(
        PlateAcquisitionJob job,
        string error,
        DateTime nextAttemptAt,
        int maxAttempts,
        DateTime updatedAt)
    {
        var nextAttemptCount = job.AttemptCount;
        var terminal = nextAttemptCount >= maxAttempts;
        using var conn = CreateConnection();
        conn.Open();
        using var tx = conn.BeginTransaction();
        try
        {
            using (var fail = new DmCommand(
                @"UPDATE PlateAcquisitionJob
                  SET Status = ?, AttemptCount = ?, NextAttemptAt = ?, LastError = ?, UpdatedAt = ?
                  WHERE Id = ? AND Status = 'PROCESSING' AND AttemptCount=? AND UpdatedAt=?",
                conn,
                (DmTransaction)tx))
            {
                AddVarcharParam(fail, "@Status", 16, terminal ? "FAILED" : "RETRY", required: true);
                fail.Parameters.Add("@AttemptCount", DmDbType.Int32).Value = nextAttemptCount;
                fail.Parameters.Add("@NextAttemptAt", DmDbType.DateTime).Value = nextAttemptAt;
                AddVarcharParam(fail, "@LastError", 1024, error);
                fail.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = updatedAt;
                fail.Parameters.Add("@Id", DmDbType.Int64).Value = job.Id;
                AddPlateJobFence(fail, job);
                if (fail.ExecuteNonQuery() != 1)
                {
                    throw new DBConcurrencyException($"Plate acquisition job {job.Id} is no longer claimed.");
                }
            }

            if (terminal)
            {
                using var arrival = new DmCommand(
                    @"UPDATE VehicleArrival
                      SET PlateStatus = 'FAILED', PlateSource = 'PARTNER', UpdatedAt = ?
                      WHERE SessionId = ? AND PlateStatus <> 'SUCCEEDED'",
                    conn,
                    (DmTransaction)tx);
                arrival.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = updatedAt;
                AddVarcharParam(arrival, "@WorkflowId", 128, job.WorkflowId, required: true);
                arrival.ExecuteNonQuery();
            }

            tx.Commit();
        }
        catch
        {
            TryRollback(tx);
            throw;
        }
    }
}
