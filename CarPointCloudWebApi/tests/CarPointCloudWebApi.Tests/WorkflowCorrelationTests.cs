using Microsoft.Extensions.Configuration;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public class WorkflowCorrelationTests
{
    [Fact]
    public void MockPlateGenerator_UsesConfiguredPrefixAndValidSuffix()
    {
        var plate = MockLicensePlateGenerator.Generate("粤A");

        Assert.Matches("^粤A[A-HJ-NP-Z0-9]{5}$", plate);
    }

    [Fact]
    public void WorkflowPlate_ReturnsPersistedPlateForExactWorkflow()
    {
        var workflowId = "sess_20260812120000000_abcdef123456";
        var repository = new WorkflowRepository(new VehicleArrivalRecord
        {
            SessionId = workflowId,
            LicensePlate = "粤A64727",
            PlateStatus = VehicleArrivalPlateStatus.Succeeded,
            PlateSource = "MOCK",
            PlateCapturedAt = new DateTime(2026, 8, 12, 12, 0, 0, DateTimeKind.Local)
        });
        var service = new IntegrationQueryService(
            repository,
            new ConfigurationBuilder().Build());

        var response = service.GetVehicleWorkflowPlate(workflowId);

        Assert.Equal(workflowId, response.WorkflowId);
        Assert.Equal("粤A64727", response.LicensePlate);
        Assert.Equal("MOCK", response.Source);
        Assert.Equal(1, repository.LookupCount);
    }

    [Fact]
    public void WorkflowPlate_WhenPlateIsPending_IsNotReturned()
    {
        var repository = new WorkflowRepository(new VehicleArrivalRecord
        {
            SessionId = "workflow-pending",
            PlateStatus = VehicleArrivalPlateStatus.Pending
        });
        var service = new IntegrationQueryService(
            repository,
            new ConfigurationBuilder().Build());

        Assert.Throws<InvalidOperationException>(() =>
            service.GetVehicleWorkflowPlate("workflow-pending"));
    }

    [Fact]
    public async Task DimensionIngest_UsesWorkflowPlateAndKeepsFingerprintStableAcrossPlateReadiness()
    {
        var repository = new DimensionWorkflowRepository();
        var service = new IntegrationQueryService(
            repository,
            new ConfigurationBuilder()
                .AddInMemoryCollection(new Dictionary<string, string?>
                {
                    ["Database:Enabled"] = "true"
                })
                .Build());

        await service.IngestPointCloudDimensionsAsync(DimensionRequest("UNKNOWN", "dimension-before"));
        repository.SucceededPlate = "粤B48322";
        await service.IngestPointCloudDimensionsAsync(DimensionRequest("错误车牌", "dimension-after"));

        Assert.Equal(2, repository.Records.Count);
        Assert.Equal("UNKNOWN", repository.Records[0].LicensePlate);
        Assert.Equal("粤B48322", repository.Records[1].LicensePlate);
        Assert.Equal(repository.Records[0].PayloadHash, repository.Records[1].PayloadHash);
    }

    private static PointCloudDimensionIngestRequest DimensionRequest(
        string licensePlate,
        string idempotencyKey) => new()
        {
            IdempotencyKey = idempotencyKey,
            WorkflowId = "workflow-dimension-001",
            PairId = "pair-dimension-001",
            CameraIdentifier = "CAM_004",
            LicensePlate = licensePlate,
            DetectStatus = "success",
            SourceFile = "/data/workflow-dimension-001.pcd",
            Dimensions = new PointCloudDimensionsPayload
            {
                TrailerLength = 960.0m,
                TrailerWidth = 255.0m
            }
        };

    private sealed class WorkflowRepository(VehicleArrivalRecord record)
        : NullIntegrationRepository
    {
        public int LookupCount { get; private set; }

        public override VehicleArrivalRecord? GetVehicleArrivalBySessionId(string sessionId)
        {
            LookupCount++;
            return string.Equals(record.SessionId, sessionId, StringComparison.Ordinal)
                ? record
                : null;
        }
    }

    private sealed class DimensionWorkflowRepository : NullIntegrationRepository
    {
        public List<VehicleDimensionRecord> Records { get; } = new();

        public string? SucceededPlate { get; set; }

        public override string? GetSucceededWorkflowLicensePlate(string workflowId) => SucceededPlate;

        public override Task<long> InsertVehicleDimensionRecordAsync(
            VehicleDimensionRecord record,
            CancellationToken cancellationToken = default)
        {
            Records.Add(record);
            return Task.FromResult((long)Records.Count);
        }
    }
}
