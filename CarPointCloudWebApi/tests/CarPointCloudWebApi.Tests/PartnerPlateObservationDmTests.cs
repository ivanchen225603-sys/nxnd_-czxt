using Dm;
using Microsoft.Extensions.Configuration;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class PartnerPlateObservationDmTests
{
    private sealed class IsolatedDmFactAttribute : FactAttribute
    {
        public IsolatedDmFactAttribute()
        {
            if (string.IsNullOrWhiteSpace(Environment.GetEnvironmentVariable("STATIC_GATE_TEST_DSN")))
                Skip = "需要已安装本次迁移的隔离 DM8 测试库";
        }
    }

    [IsolatedDmFact]
    public void Unbound_delayed_grab_is_remembered_and_never_claimed_by_next_arrival()
    {
        var dsn = Environment.GetEnvironmentVariable("STATIC_GATE_TEST_DSN")!;
        var config = new ConfigurationBuilder().AddInMemoryCollection(new Dictionary<string, string?>
            { ["ConnectionStrings:CarPointCloudDb"] = dsn }).Build();
        var repo = new IntegrationRepository(config);
        var camera = "T" + Guid.NewGuid().ToString("N")[..24];
        var now = BusinessClock.Now;
        var start = now.AddTicks(-(now.Ticks % TimeSpan.TicksPerSecond)).AddHours(-2);
        VehicleArrivalRecord Arrival(DateTime at, string status) => new()
        {
            EventId = Guid.NewGuid().ToString("N"), SessionId = Guid.NewGuid().ToString("N"),
            CameraIdentifier = camera, DeviceId = "test", EventType = "ARRIVAL", ArrivalTime = at,
            CreatedAt = at, UpdatedAt = at, PlateStatus = status
        };
        PartnerLicensePlateData Response(string plate) => new()
        {
            CameraId = camera, CaptureTime = start.AddSeconds(21).ToString("yyyy-MM-dd HH:mm:ss"),
            EnterId = "000000001", LicensePlate = plate
        };

        repo.InsertVehicleArrivalRecord(Arrival(start, VehicleArrivalPlateStatus.Failed));
        var next = repo.InsertVehicleArrivalRecord(Arrival(start.AddSeconds(20), VehicleArrivalPlateStatus.Pending)).Record;
        var first = Response("宁A12345");
        repo.ObservePartnerCaptures(camera, [first], start.AddSeconds(29));
        var second = Response("宁A12345");
        new IntegrationRepository(config).ObservePartnerCaptures(camera, [second], start.AddSeconds(31));
        Assert.Equal(start.AddSeconds(29), second.FirstSeenAt);
        Assert.False(second.ConflictingIdentity);

        var job = ClaimOwnJob(dsn, next);
        Assert.True(repo.TryBeginPlateAcquisitionAttempt(job, 3, now));
        var record = new LicensePlateRecord { CameraIdentifier = camera, CameraId = camera, WorkflowId = next.SessionId,
            EnterId = second.EnterId, CaptureTime = start.AddSeconds(21), FirstSeenAt = second.FirstSeenAt,
            LicensePlate = second.LicensePlate, IsSuccess = true, ResponseCode = 200, ResponseMessage = "success",
            RequestTime = now, CreatedAt = now };
        Assert.False(repo.TryCompletePlateAcquisitionJob(job, [record]));
        Assert.Null(repo.GetSucceededWorkflowLicensePlate(next.SessionId));

        var conflict = Response("宁A54321");
        repo.ObservePartnerCaptures(camera, [conflict], start.AddSeconds(34));
        Assert.True(conflict.ConflictingIdentity);
        Assert.Equal(first.FirstSeenAt, conflict.FirstSeenAt);

        var freshArrival = repo.InsertVehicleArrivalRecord(Arrival(start.AddHours(2).AddSeconds(-20),
            VehicleArrivalPlateStatus.Pending)).Record;
        var freshCapture = freshArrival.ArrivalTime.AddSeconds(4);
        var fresh = Response("宁A12345");
        fresh.CaptureTime = freshCapture.ToString("yyyy-MM-dd HH:mm:ss");
        fresh.EnterId = "000000002";
        repo.ObservePartnerCaptures(camera, [fresh], now);
        var freshJob = ClaimOwnJob(dsn, freshArrival);
        Assert.True(repo.TryBeginPlateAcquisitionAttempt(freshJob, 3, now));
        record.EnterId = fresh.EnterId;
        record.CaptureTime = freshCapture;
        record.WorkflowId = freshArrival.SessionId;
        record.FirstSeenAt = fresh.FirstSeenAt;
        Assert.True(repo.TryCompletePlateAcquisitionJob(freshJob, [record]));
        Assert.Equal(fresh.LicensePlate, repo.GetSucceededWorkflowLicensePlate(freshArrival.SessionId));
    }

    [IsolatedDmFact]
    public void Persisted_anchor_observation_separates_an_older_failed_trip_and_binds_the_new_capture()
    {
        var dsn = Environment.GetEnvironmentVariable("STATIC_GATE_TEST_DSN")!;
        var config = new ConfigurationBuilder().AddInMemoryCollection(
            new Dictionary<string, string?> { ["ConnectionStrings:CarPointCloudDb"] = dsn }).Build();
        var repo = new IntegrationRepository(config);
        var camera = "T" + Guid.NewGuid().ToString("N")[..24];
        var now = BusinessClock.Now;
        var start = now.AddTicks(-(now.Ticks % TimeSpan.TicksPerSecond)).AddMinutes(-5);
        VehicleArrivalRecord AddArrival(int seconds, string status)
        {
            var at = start.AddSeconds(seconds);
            return repo.InsertVehicleArrivalRecord(new VehicleArrivalRecord
            {
                EventId = Guid.NewGuid().ToString("N"), SessionId = Guid.NewGuid().ToString("N"),
                CameraIdentifier = camera, DeviceId = "test", EventType = "ARRIVAL",
                ArrivalTime = at, CreatedAt = at, UpdatedAt = at, PlateStatus = status
            }).Record;
        }
        LicensePlateRecord Grab(VehicleArrivalRecord trip, string plate, string id) => new()
        {
            CameraIdentifier = camera, CameraId = camera, WorkflowId = trip.SessionId,
            CaptureTime = trip.ArrivalTime.AddSeconds(8), EnterId = id, LicensePlate = plate,
            RequestTime = now, CreatedAt = now, IsSuccess = true, ResponseCode = 200, ResponseMessage = "success"
        };
        void Observe(LicensePlateRecord grab) => repo.ObservePartnerCaptures(camera, [new PartnerLicensePlateData
        {
            CameraId = camera, EnterId = grab.EnterId!, LicensePlate = grab.LicensePlate,
            CaptureTime = PartnerContract.FormatTime(grab.CaptureTime)!
        }], now);

        var old = AddArrival(0, VehicleArrivalPlateStatus.Failed);
        var anchor = AddArrival(70, VehicleArrivalPlateStatus.Failed);
        var anchored = Grab(anchor, "宁A32101", "00999");
        repo.InsertLicensePlateRecord(anchored);
        var current = AddArrival(140, VehicleArrivalPlateStatus.Pending);
        var candidate = Grab(current, "宁A32102", "00001");
        Observe(candidate);
        var job = ClaimOwnJob(dsn, current);
        Assert.True(repo.TryBeginPlateAcquisitionAttempt(job, 3, now));

        // 历史成功行缺少与之相符的观察记录时，不能据它解除旧车的抓拍保护。
        Assert.False(repo.TryCompletePlateAcquisitionJob(job, [candidate]));
        Assert.Null(repo.GetSucceededWorkflowLicensePlate(current.SessionId));
        Observe(anchored);
        var reopened = new IntegrationRepository(config);
        Assert.True(reopened.TryCompletePlateAcquisitionJob(job, [candidate]));
        Assert.Null(reopened.GetSucceededWorkflowLicensePlate(old.SessionId));
        Assert.Equal(anchored.LicensePlate, reopened.GetSucceededWorkflowLicensePlate(anchor.SessionId));
        Assert.Equal(candidate.LicensePlate, reopened.GetSucceededWorkflowLicensePlate(current.SessionId));

        using var db = new DmConnection(dsn);
        db.Open();
        using var read = new DmCommand("SELECT EnterId FROM LicensePlateRecord WHERE WorkflowId=? AND IsSuccess=1", db);
        read.Parameters.Add("@Workflow", DmDbType.VarChar, 128).Value = current.SessionId;
        using var rows = read.ExecuteReader();
        Assert.True(rows.Read());
        Assert.Equal(candidate.EnterId, rows.GetString(0));
        Assert.False(rows.Read());
    }

    [IsolatedDmFact]
    public void FailedManualArrival_DoesNotCompeteWithAutomaticPlateAcquisition()
    {
        var dsn = Environment.GetEnvironmentVariable("STATIC_GATE_TEST_DSN")!;
        var config = new ConfigurationBuilder().AddInMemoryCollection(
            new Dictionary<string, string?> { ["ConnectionStrings:CarPointCloudDb"] = dsn }).Build();
        var repo = new IntegrationRepository(config);
        var now = BusinessClock.Now;
        var arrival = now.AddTicks(-(now.Ticks % TimeSpan.TicksPerSecond)).AddSeconds(-11);
        foreach (var manual in new[] { true, false })
        {
            var camera = "T" + Guid.NewGuid().ToString("N")[..24];
            VehicleArrivalRecord Insert(DateTime at, string plateStatus, string? source, string radarStatus) =>
                repo.InsertVehicleArrivalRecord(new VehicleArrivalRecord
                {
                    EventId = Guid.NewGuid().ToString("N"), SessionId = Guid.NewGuid().ToString("N"),
                    CameraIdentifier = camera, DeviceId = "test", EventType = "ARRIVAL",
                    ArrivalTime = at, CreatedAt = at, UpdatedAt = at, PlateStatus = plateStatus,
                    PlateSource = source, RadarStatus = radarStatus,
                    LicensePlate = source == "MANUAL" ? "宁A12089" : null
                }).Record;
            var prior = Insert(arrival.AddSeconds(-20), manual ? VehicleArrivalPlateStatus.Succeeded : VehicleArrivalPlateStatus.Failed,
                manual ? "MANUAL" : null, VehicleArrivalRadarStatus.Failed);
            var current = Insert(arrival, VehicleArrivalPlateStatus.Pending, null, VehicleArrivalRadarStatus.NotRequired);
            var candidate = new LicensePlateRecord
            {
                CameraIdentifier = camera, CameraId = camera, WorkflowId = current.SessionId,
                CaptureTime = arrival.AddSeconds(1), EnterId = "new-capture", LicensePlate = "宁A12090",
                RequestTime = now, CreatedAt = now, IsSuccess = true, ResponseCode = 200, ResponseMessage = "success"
            };
            repo.ObservePartnerCaptures(camera, [new PartnerLicensePlateData
            {
                CameraId = camera, EnterId = candidate.EnterId!, LicensePlate = candidate.LicensePlate,
                CaptureTime = PartnerContract.FormatTime(candidate.CaptureTime)!
            }], now);
            var job = ClaimOwnJob(dsn, current);
            Assert.True(repo.TryBeginPlateAcquisitionAttempt(job, 3, now));
            Assert.Equal(manual, repo.TryCompletePlateAcquisitionJob(job, [candidate]));
            Assert.Equal(manual ? candidate.LicensePlate : null, repo.GetSucceededWorkflowLicensePlate(current.SessionId));
            Assert.False(repo.DeleteUndispatchedManualArrival(prior.Id));
            Assert.NotNull(repo.GetVehicleArrivalBySessionId(prior.SessionId));

            var unaccepted = Insert(now, VehicleArrivalPlateStatus.Succeeded, "MANUAL", VehicleArrivalRadarStatus.Pending);
            Assert.True(repo.DeleteUndispatchedManualArrival(unaccepted.Id));
            Assert.Null(repo.GetVehicleArrivalBySessionId(unaccepted.SessionId));
        }
    }

    private static PlateAcquisitionJob ClaimOwnJob(string dsn, VehicleArrivalRecord arrival)
    {
        // 仅领取本测试的 workflow，避免消费其他同时执行的集成用例。
        using var connection = new DmConnection(dsn);
        connection.Open();
        using var claim = new DmCommand("UPDATE PlateAcquisitionJob SET Status='PROCESSING' WHERE WorkflowId=?", connection);
        claim.Parameters.Add("@Workflow", DmDbType.VarChar, 128).Value = arrival.SessionId;
        Assert.Equal(1, claim.ExecuteNonQuery());
        using var read = new DmCommand(
            "SELECT Id,InteractionId,CreatedAt,UpdatedAt FROM PlateAcquisitionJob WHERE WorkflowId=?", connection);
        read.Parameters.Add("@Workflow", DmDbType.VarChar, 128).Value = arrival.SessionId;
        using var reader = read.ExecuteReader();
        Assert.True(reader.Read());
        return new PlateAcquisitionJob { Id = reader.GetInt64(0), InteractionId = reader.GetString(1),
            CameraIdentifier = arrival.CameraIdentifier, WorkflowId = arrival.SessionId,
            CreatedAt = reader.GetDateTime(2), UpdatedAt = reader.GetDateTime(3) };
    }
}
