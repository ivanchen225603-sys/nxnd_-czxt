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
    public bool TryCompletePlateAcquisitionJob(PlateAcquisitionJob job, IReadOnlyList<LicensePlateRecord> candidates)
    {
        using var conn = CreateConnection();
        conn.Open();
        using var tx = (DmTransaction)conn.BeginTransaction();
        try
        {
            if (!LockPlateJobLease(conn, tx, job))
                throw new DBConcurrencyException($"Plate acquisition job {job.Id} is no longer claimed.");
            LockCameraAnchor(conn, tx, job.CameraIdentifier, required: true);
            RefreshCandidateObservations(conn, tx, job.CameraIdentifier, candidates);
            var (arrival, boundPlate) = ReadPlateArrival(conn, tx, job);
            var offsets = ReadPlateCaptureOffsets(conn, tx, job.CameraIdentifier, arrival);
            var reference = "partner_history";
            var window = PlateCaptureWindow.FromHistory(
                offsets, _captureLeadSeconds, _captureWindowSeconds, _captureSkewSeconds);
            if (_verifiedPlateOffsetBounds.ForCamera(job.CameraIdentifier) is { } bounds)
            {
                window = PlateCaptureWindow.FromVerifiedBounds(bounds.Minimum, bounds.Maximum, _captureLeadSeconds);
                reference = "verified_bounds";
            }
            else if (!window.IsCalibrated)
            {
                var verifiedOffsets = _verifiedPlateCalibration.OffsetsBefore(job.CameraIdentifier, arrival, job.WorkflowId);
                var verified = PlateCaptureWindow.FromHistory(
                    verifiedOffsets, _captureLeadSeconds, _captureWindowSeconds, _captureSkewSeconds);
                if (verified.IsCalibrated)
                {
                    window = verified;
                    offsets = verifiedOffsets.ToList();
                }
                reference = verified.IsCalibrated ? "verified_passages" : "cold";
            }
            var neighbors = ReadNeighboringArrivals(conn, tx, job, arrival, window);
            var latestCapture = window.HasTimeReference ? (DateTime?)null : BusinessClock.Now.AddSeconds(_captureSkewSeconds);
            var decisions = new List<(LicensePlateRecord Candidate, string Reason)>();
            var record = window.MatchUnique(candidates, arrival, neighbors, latestCapture,
                candidate => _verifiedPlateCalibration.IsOtherVerifiedCapture(job.CameraIdentifier, job.WorkflowId, candidate.EnterId, arrival)
                    || CaptureAlreadyClaimed(conn, tx, candidate),
                (candidate, reason) => decisions.Add((candidate, reason)));
            if (record is null)
            {
                tx.Commit();
                LogPlateMatch(job, candidates.Count, arrival, offsets, window, reference, neighbors.Count, decisions, null);
                return false;
            }
            if (boundPlate is not null && !string.Equals(boundPlate, record.LicensePlate, StringComparison.Ordinal))
                throw new InvalidOperationException("原来车已有不同的权威车牌，不能覆盖");

            SaveLicensePlateRecord(conn, tx, record, bindWorkflow: true, captureVerified: true);

            // 分析可能早于车牌识别完成并以 UNKNOWN 入库。车牌任务完成时在同一事务
            // 回填所有已存在的 workflow 数据，避免接口查询看到互相矛盾的车辆身份。
            BackfillWorkflowLicensePlate(
                conn,
                tx,
                job.WorkflowId,
                record.LicensePlate,
                record.CreatedAt);

            using var complete = new DmCommand(
                @"UPDATE PlateAcquisitionJob
                  SET Status = 'SUCCEEDED', CandidateJson=NULL,
                      NextAttemptAt = ?, LastError = NULL, UpdatedAt = ?
                  WHERE Id = ? AND Status = 'PROCESSING' AND AttemptCount=? AND UpdatedAt=?",
                conn,
                tx);
            complete.Parameters.Add("@NextAttemptAt", DmDbType.DateTime).Value = record.CreatedAt;
            complete.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = record.CreatedAt;
            complete.Parameters.Add("@Id", DmDbType.Int64).Value = job.Id;
            AddPlateJobFence(complete, job);
            if (complete.ExecuteNonQuery() != 1)
            {
                throw new DBConcurrencyException($"Plate acquisition job {job.Id} is no longer claimed.");
            }

            tx.Commit();
            LogPlateMatch(job, candidates.Count, arrival, offsets, window, reference, neighbors.Count, decisions, record);
            return true;
        }
        catch
        {
            TryRollback(tx);
            throw;
        }
    }

    private void LogPlateMatch(PlateAcquisitionJob job, int candidateCount, DateTime arrival, IReadOnlyList<TimeSpan> offsets,
        PlateCaptureWindow window, string reference, int neighborCount, List<(LicensePlateRecord Candidate, string Reason)> decisions,
        LicensePlateRecord? record)
    {
        var decisionAt = BusinessClock.Now;
        _logger?.LogInformation(
            "车牌匹配 camera={CameraIdentifier} workflowId={WorkflowId} interactionId={InteractionId} attempt={Attempt} " +
            "outcome={Outcome} arrivalAt={ArrivalAt:yyyy-MM-dd HH:mm:ss.fff} decisionAt={DecisionAt:yyyy-MM-dd HH:mm:ss.fff} " +
            "arrivalToDecisionMs={ArrivalToDecisionMs} candidateCount={CandidateCount} neighborCount={NeighborCount} " +
            "calibrated={Calibrated} timeReference={TimeReference} windowFromSec={WindowFromSec} windowToSec={WindowToSec} historyOffsetsSec=[{HistoryOffsetsSec}] " +
            "plate={LicensePlate} captureAt={CaptureAt:yyyy-MM-dd HH:mm:ss.fff} enterId={EnterId}",
            job.CameraIdentifier, job.WorkflowId, job.InteractionId, job.AttemptCount,
            record is null ? "no_unique_match" : "committed", arrival, decisionAt,
            (long)(decisionAt - arrival).TotalMilliseconds, candidateCount, neighborCount,
            window.IsCalibrated, reference, window.FromArrival.TotalSeconds, window.ToArrival.TotalSeconds,
            string.Join(",", offsets.Select(offset => offset.TotalSeconds)), record?.LicensePlate, record?.CaptureTime, record?.EnterId);
        foreach (var (candidate, reason) in decisions)
            _logger?.LogInformation(
                "车牌候选 camera={CameraIdentifier} workflowId={WorkflowId} interactionId={InteractionId} attempt={Attempt} " +
                "plate={LicensePlate} captureAt={CaptureAt:yyyy-MM-dd HH:mm:ss.fff} firstSeenAt={FirstSeenAt:yyyy-MM-dd HH:mm:ss.fff} enterId={EnterId} reason={Reason}",
                job.CameraIdentifier, job.WorkflowId, job.InteractionId, job.AttemptCount,
                candidate.LicensePlate, candidate.CaptureTime, candidate.FirstSeenAt, candidate.EnterId, reason);
    }

    private (DateTime Arrival, string? BoundPlate) ReadPlateArrival(
        DmConnection connection, DmTransaction tx, PlateAcquisitionJob job)
    {
        using var command = new DmCommand(
            "SELECT ArrivalTime, PlateStatus, LicensePlate FROM VehicleArrival WHERE SessionId=? AND CameraIdentifier=?",
            connection, tx) { CommandTimeout = _commandTimeoutSeconds };
        AddVarcharParam(command, "@WorkflowId", 128, job.WorkflowId, required: true);
        AddVarcharParam(command, "@CameraIdentifier", 64, job.CameraIdentifier, required: true);
        using var reader = command.ExecuteReader();
        if (!reader.Read()) throw new InvalidOperationException("来车流程不存在或点位不符");
        var boundPlate = reader.GetString(1) == VehicleArrivalPlateStatus.Succeeded && !reader.IsDBNull(2)
            ? reader.GetString(2) : null;
        return (reader.GetDateTime(0), boundPlate);
    }

    private List<TimeSpan> ReadPlateCaptureOffsets(
        DmConnection connection, DmTransaction tx, string camera, DateTime arrival)
    {
        using var command = new DmCommand(
            @"SELECT TOP 5 a.ArrivalTime, p.CaptureTime, p.EnterId
              FROM VehicleArrival a JOIN LicensePlateRecord p ON p.WorkflowId=a.SessionId
              WHERE a.CameraIdentifier=? AND a.ArrivalTime<? AND a.PlateStatus='SUCCEEDED'
                    AND a.PlateSource='PARTNER' AND p.IsSuccess=1 AND p.CaptureTime IS NOT NULL
                    AND p.CameraIdentifier=a.CameraIdentifier AND p.LicensePlate=a.LicensePlate AND p.EnterId IS NOT NULL
              ORDER BY a.ArrivalTime DESC, a.Id DESC",
            connection, tx) { CommandTimeout = _commandTimeoutSeconds };
        AddVarcharParam(command, "@CameraIdentifier", 64, camera, required: true);
        command.Parameters.Add("@ArrivalTime", DmDbType.DateTime).Value = arrival;
        // 每个 workflow 在库内有唯一成功业务行；旧数据若跨车次复用 enterID，不把重复记录当独立样本。
        var offsets = new Dictionary<string, TimeSpan?>(StringComparer.Ordinal);
        using (var reader = command.ExecuteReader())
        {
            while (reader.Read())
            {
                var enterId = reader.GetString(2);
                if (string.IsNullOrWhiteSpace(enterId)) continue;
                var priorArrival = reader.GetDateTime(0);
                var offset = reader.GetDateTime(1) - priorArrival.AddTicks(-(priorArrival.Ticks % TimeSpan.TicksPerSecond));
                var key = ObservationKey(camera, enterId);
                if (!offsets.TryAdd(key, offset)) offsets[key] = null;
            }
        }
        var observations = ReadPlateObservations(connection, tx, offsets.Keys);
        foreach (var row in observations.Where(row => row.Value.Conflict)) offsets[row.Key] = null;
        return offsets.Values.OfType<TimeSpan>().ToList();
    }

    private List<PlateArrivalNeighbor> ReadNeighboringArrivals(
        DmConnection connection, DmTransaction tx, PlateAcquisitionJob job, DateTime arrival, PlateCaptureWindow window)
    {
        using var command = new DmCommand(
            @"SELECT a.ArrivalTime,a.PlateStatus,p.EnterId,a.SessionId,p.CaptureTime,p.LicensePlate,a.PlateSource
              FROM VehicleArrival a LEFT JOIN LicensePlateRecord p ON p.WorkflowId=a.SessionId
                    AND p.IsSuccess=1 AND p.CameraIdentifier=a.CameraIdentifier AND p.LicensePlate=a.LicensePlate
              WHERE a.CameraIdentifier=? AND a.SessionId<>? AND a.ArrivalTime BETWEEN ? AND ?
                    AND (a.PlateSource IS NULL OR a.PlateSource<>'MANUAL' OR a.RadarStatus<>'FAILED')",
            connection, tx) { CommandTimeout = _commandTimeoutSeconds };
        AddVarcharParam(command, "@CameraIdentifier", 64, job.CameraIdentifier, required: true);
        AddVarcharParam(command, "@WorkflowId", 128, job.WorkflowId, required: true);
        var span = window.ToArrival - window.FromArrival + TimeSpan.FromSeconds(1);
        // 一小时是乙方快照范围，在此范围内保留有用的前后顺序证据；归属另按每条候选判断。
        var lookback = window.NeighborLookback > PlateCaptureWindow.SnapshotHorizon
            ? window.NeighborLookback : PlateCaptureWindow.SnapshotHorizon;
        var lookahead = span > PlateCaptureWindow.SnapshotHorizon ? span : PlateCaptureWindow.SnapshotHorizon;
        command.Parameters.Add("@Before", DmDbType.DateTime).Value = arrival - lookback;
        command.Parameters.Add("@After", DmDbType.DateTime).Value = arrival + lookahead;
        var neighbors = new List<PlateArrivalNeighbor>();
        var pending = new List<(int Index, string Key, DateTime Capture, string Plate)>();
        using (var reader = command.ExecuteReader())
        {
            while (reader.Read())
            {
                var at = reader.GetDateTime(0);
                var verified = _verifiedPlateCalibration.PriorCapture(job.CameraIdentifier, reader.GetString(3), at, arrival);
                var enterId = reader.IsDBNull(2) ? null : reader.GetString(2);
                if (reader.GetString(1) == VehicleArrivalPlateStatus.Succeeded && !string.IsNullOrWhiteSpace(enterId)
                    && !reader.IsDBNull(4) && !reader.IsDBNull(5) && !reader.IsDBNull(6) && reader.GetString(6) == "PARTNER")
                    pending.Add((neighbors.Count, ObservationKey(job.CameraIdentifier, enterId),
                        reader.GetDateTime(4), reader.GetString(5)));
                neighbors.Add(new(at, verified.HasValue, verified));
            }
        }
        var observations = ReadPlateObservations(connection, tx, pending.Select(p => p.Key));
        foreach (var p in pending)
            if (observations.TryGetValue(p.Key, out var obs) && !obs.Conflict && obs.Plate == p.Plate &&
                PartnerContract.TryParseTime(obs.CaptureTime, out var capture) && capture == p.Capture)
                neighbors[p.Index] = neighbors[p.Index] with { Identified = true, ConfirmedCapture = capture };
        return neighbors;
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
