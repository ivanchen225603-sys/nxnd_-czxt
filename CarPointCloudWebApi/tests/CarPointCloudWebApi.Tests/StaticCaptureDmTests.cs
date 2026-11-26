using Dm;
using Microsoft.Extensions.Configuration;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class StaticCaptureDmTests
{
    [DatabaseFact]
    public async Task Both_whole_scans_must_be_confirmed_before_dimension_insert()
    {
        var dsn = TestConfiguration.GetDatabaseConnectionString()!;
        var repository = new IntegrationRepository(new ConfigurationBuilder().AddInMemoryCollection(
            new Dictionary<string, string?> { ["ConnectionStrings:CarPointCloudDb"] = dsn }).Build());
        using var db = new DmConnection(dsn);
        db.Open();
        var stop = DateTimeOffset.UtcNow.AddSeconds(-10);
        var measuredAt = BusinessClock.Now;
        Task<List<VehicleDimensionRecord>> ReadPartnerPage() => repository.GetPartnerVehiclePageAsync(
            100, measuredAt.AddSeconds(-1), measuredAt.AddSeconds(1), null, null, CancellationToken.None);
        foreach (var moving in new[] { false, true })
        {
            var workflow = "static-" + Guid.NewGuid().ToString("N");
            var pairId = "p-" + workflow;
            using var insert = new DmCommand(@"INSERT INTO VehicleArrival(EventId,DeviceId,CameraIdentifier,EventType,SessionId,ArrivalTime)
                VALUES (?, 'LOAD_RADAR','CAM_004','ARRIVAL',?,SYSDATE)", db);
            insert.Parameters.Add("@e", DmDbType.VarChar).Value = workflow;
            insert.Parameters.Add("@w", DmDbType.VarChar).Value = workflow;
            insert.ExecuteNonQuery();
            var arrival = repository.GetVehicleArrivalBySessionId(workflow)!;
            var gate = new StaticCapture { SegmentId = "s", PrimaryCamera = "CAM_004", StoppedAt = stop,
                CaptureUntil = stop.AddSeconds(10) };
            Assert.True(repository.TryClaimStaticScan(arrival.Id, workflow, DateTime.Now, gate));
            Assert.False(repository.TryClaimStaticScan(arrival.Id, workflow, DateTime.Now, gate));
            var evidence = new StaticPairEvidence(pairId, "s",
                new("CAM_004", stop.AddSeconds(7), stop.AddSeconds(2), stop.AddSeconds(7.1)),
                new("CAM_005", stop.AddSeconds(8), stop.AddSeconds(3), stop.AddSeconds(8.1)));
            var pending = repository.UpdateStaticScan(workflow, g => g.Offer(evidence, DateTimeOffset.UtcNow));
            Assert.Equal("pending", pending.Status);
            using var dimensions = new DmCommand(@"INSERT INTO VehicleDimensionRecord
                (IdempotencyKey,WorkflowId,PairId,LicensePlate,CameraIdentifier,GetTime)
                VALUES (?,?,?,'TEST','CAM_004',?)", db);
            foreach (var (key, value) in new[] { ("@i",workflow), ("@w",workflow), ("@p",pairId) })
                dimensions.Parameters.Add(key, DmDbType.VarChar).Value = value;
            dimensions.Parameters.Add("@measured", DmDbType.DateTime).Value = measuredAt;
            Assert.Equal(-20061, Assert.Throws<DmException>(() => dimensions.ExecuteNonQuery()).Number);
            Assert.DoesNotContain(await ReadPartnerPage(), row => row.WorkflowId == workflow);
            var observation = new StandstillObservation("node","CH4","passage","s",1,stop,
                stop.AddSeconds(moving ? 6.8 : 9), stop.AddSeconds(moving ? 7 : 10), "movement");
            var result = repository.UpdateStaticScan(workflow, g => g.Observe(observation, DateTimeOffset.UtcNow));
            Assert.Equal(moving ? "rejected" : "confirmed", result.Status);
            if (moving)
            {
                Assert.Equal(-20061, Assert.Throws<DmException>(() => dimensions.ExecuteNonQuery()).Number);
                Assert.DoesNotContain(await ReadPartnerPage(), row => row.WorkflowId == workflow);
            }
            else
            {
                Assert.Equal(1, dimensions.ExecuteNonQuery());
                var published = Assert.Single(await ReadPartnerPage(), row => row.WorkflowId == workflow);
                Assert.Equal(pairId, published.PairId);
            }
        }
    }
}
