using System.Net;
using System.Text;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class WorkflowIdempotencyTests
{
    [Fact]
    public async Task CoalBusinessKey_IsStableWhenCallerChangesIdempotencyKey()
    {
        var repository = new CaptureCoalRepository();
        var configuration = new ConfigurationBuilder()
            .AddInMemoryCollection(new Dictionary<string, string?>
            {
                ["Database:Enabled"] = "true"
            })
            .Build();
        var service = new IntegrationQueryService(repository, configuration);
        var measureTime = new DateTime(2026, 8, 14, 9, 30, 0);

        await service.IngestCoalVolumeAsync(CreateCoalRequest("coal-retry-1", measureTime));
        await service.IngestCoalVolumeAsync(CreateCoalRequest("coal-retry-2", measureTime));

        Assert.Equal(2, repository.Records.Count);
        Assert.NotEqual(repository.Records[0].IdempotencyKey, repository.Records[1].IdempotencyKey);
        Assert.Equal(repository.Records[0].BusinessKey, repository.Records[1].BusinessKey);
        Assert.Equal("workflow-20260814-001", repository.Records[0].WorkflowId);
    }

    [Theory]
    [InlineData(null, "success", "unit 不能为空")]
    [InlineData("", "success", "unit 不能为空")]
    [InlineData("m³", null, "status 不能为空")]
    [InlineData("m³", "", "status 不能为空")]
    public async Task CoalVolume_RequiresUnitAndStatus(string? unit, string? status, string expectedMessage)
    {
        var service = CreateCoalService(new CaptureCoalRepository());
        var request = CreateCoalRequest("coal-required-fields", new DateTime(2026, 8, 14, 9, 30, 0));
        request.Unit = unit!;
        request.Status = status!;

        var exception = await Assert.ThrowsAsync<ArgumentException>(
            () => service.IngestCoalVolumeAsync(request));

        Assert.Equal(expectedMessage, exception.Message);
    }

    private static PointCloudCoalVolumeIngestRequest CreateCoalRequest(
        string idempotencyKey,
        DateTime measureTime) => new()
        {
            IdempotencyKey = idempotencyKey,
            CameraIdentifier = "CAM_004",
            LicensePlate = "宁A25771",
            MeasureTime = measureTime,
            Volume = 42.3m,
            Unit = "m³",
            Status = "success",
            Confidence = 0.95m,
            SourceFile = "/data/carpointcloud/results/coal-001.json"
        };

    private static IntegrationQueryService CreateCoalService(IIntegrationRepository repository)
    {
        var configuration = new ConfigurationBuilder()
            .AddInMemoryCollection(new Dictionary<string, string?>
            {
                ["Database:Enabled"] = "true"
            })
            .Build();
        return new IntegrationQueryService(repository, configuration);
    }

    private sealed class CaptureCoalRepository : NullIntegrationRepository
    {
        public List<CoalVolumeRecord> Records { get; } = new();

        public override Task<long> InsertCoalVolumeRecordAsync(
            CoalVolumeRecord record,
            CancellationToken cancellationToken = default)
        {
            Records.Add(record);
            return Task.FromResult((long)Records.Count);
        }

        public override string? FindWorkflowIdForMeasurement(
            string licensePlate,
            string cameraIdentifier,
            DateTime measureTime) => "workflow-20260814-001";
    }

    private sealed class QueryCoalRepository(string status) : NullIntegrationRepository
    {
        public override List<CoalVolumeRecord> GetCoalVolumesPage(
            int take,
            DateTime? windowStart,
            DateTime? beforeTime,
            long? beforeId) =>
        [
            new CoalVolumeRecord
            {
                Id = 1,
                LicensePlate = "宁A25771",
                CameraIdentifier = "CAM_004",
                Volume = 42.3m,
                Unit = "m³",
                Status = status,
                MeasureTime = new DateTime(2026, 8, 14, 9, 30, 0)
            }
        ];
    }

    private sealed class PlateJobRepository : NullIntegrationRepository
    {
        private int _claimed;
        private PlateAcquisitionJob? _retry;
        public TaskCompletionSource<(PlateAcquisitionJob Job, LicensePlateRecord Record)> Completed { get; } =
            new(TaskCreationOptions.RunContinuationsAsynchronously);

        public override PlateAcquisitionJob? TryClaimDuePlateAcquisitionJob(DateTime now, DateTime staleBefore)
        {
            if (_retry is { } pending && pending.NextAttemptAt > now) return null;
            if (Interlocked.Exchange(ref _claimed, 1) != 0)
            {
                return null;
            }

            if (_retry is { } retry) return retry;
            return new PlateAcquisitionJob
            {
                Id = 1,
                WorkflowId = "workflow-plate-001",
                InteractionId = "interaction-plate-001",
                CameraIdentifier = "CAM_004",
                Status = "PROCESSING",
                NextAttemptAt = now,
                CreatedAt = now,
                UpdatedAt = now
            };
        }

        public override bool SavePlateAcquisitionProgress(PlateAcquisitionJob job, string? candidateJson,
            DateTime nextAttemptAt, string? error, DateTime now)
        {
            base.SavePlateAcquisitionProgress(job, candidateJson, nextAttemptAt, error, now);
            _retry = job;
            Volatile.Write(ref _claimed, 0);
            return true;
        }

        public override bool TryCompletePlateAcquisitionJob(PlateAcquisitionJob job,
            IReadOnlyList<LicensePlateRecord> candidates) => Completed.TrySetResult((job, Assert.Single(candidates)));

        public override void FailPlateAcquisitionJob(
            PlateAcquisitionJob job,
            string error,
            DateTime nextAttemptAt,
            int maxAttempts,
            DateTime updatedAt) => Completed.TrySetException(new InvalidOperationException(error));
    }

    private sealed class StaticPartnerHandler : HttpMessageHandler
    {
        public bool SawInternalCorrelationHeader { get; private set; }
        public Uri? RequestUri { get; private set; }
        public int RequestCount { get; private set; }

        protected override Task<HttpResponseMessage> SendAsync(
            HttpRequestMessage request,
            CancellationToken cancellationToken)
        {
            RequestUri = request.RequestUri;
            RequestCount++;
            SawInternalCorrelationHeader = request.Headers.Any(header =>
                header.Key is "X-Interaction-Id" or "X-Workflow-Id" or "X-Event-Id" or
                    "X-Caller-System" or "X-Integration-Type");
            var captureTime = BusinessClock.Now.ToString("yyyy-MM-dd HH:mm:ss");
            var body = $$"""
                {"code":200,"message":"success","data":[
                  {"cameraId":"CAM_004","licensePlate":"宁A69352","captureTime":"{{captureTime}}","enterID":"business-1"}
                ]}
                """;
            return Task.FromResult(new HttpResponseMessage(HttpStatusCode.OK)
            {
                Content = new StringContent(body, Encoding.UTF8, "application/json")
            });
        }
    }

    private sealed class TestOptionsMonitor<T>(T value) : IOptionsMonitor<T>, IOptions<T> where T : class
    {
        public T CurrentValue => value;
        public T Value => value;
        public T Get(string? name) => value;
        public IDisposable? OnChange(Action<T, string?> listener) => null;
    }
}
