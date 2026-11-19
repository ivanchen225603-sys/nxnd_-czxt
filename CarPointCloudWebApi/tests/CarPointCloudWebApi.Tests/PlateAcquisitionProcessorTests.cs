using Xunit;
using System.Globalization;
using System.Net;
using System.Text;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Tests;

public sealed class PlateAcquisitionProcessorTests
{
    private const string Camera = "CAM_001";

    [Fact]
    public async Task Capture_inside_arrival_window_completes_the_job_with_the_latest_plate()
    {
        var job = Job(arrivedSecondsAgo: 20, attempts: 0);
        var repository = new JobRepository();
        var processor = Processor(repository,
            Capture("宁A09185", job.CreatedAt.AddSeconds(-30)),
            Capture("宁A27066", job.CreatedAt.AddSeconds(2)),
            Capture("宁A49586", job.CreatedAt.AddSeconds(5)));

        await processor.RunAsync(job, CancellationToken.None);

        Assert.Equal("宁A49586", repository.Completed?.LicensePlate);
        Assert.Equal(job.WorkflowId, repository.Completed?.WorkflowId);
        Assert.Equal(1, job.AttemptCount);
    }

    [Fact]
    public async Task No_capture_is_retried_until_the_third_attempt_fails()
    {
        var job = Job(arrivedSecondsAgo: 20, attempts: 0);
        var repository = new JobRepository();
        var processor = Processor(repository);

        await processor.RunAsync(job, CancellationToken.None);
        Assert.StartsWith(PlateFailureCode.NoCapture, repository.ProgressError);
        Assert.Null(repository.FailedError);

        job.AttemptCount = 2;
        await processor.RunAsync(job, CancellationToken.None);
        Assert.StartsWith(PlateFailureCode.NoCapture, repository.FailedError);
        Assert.Null(repository.Completed);
    }

    [Fact]
    public async Task Partner_is_not_called_before_the_first_call_delay()
    {
        var job = Job(arrivedSecondsAgo: 1, attempts: 0);
        var repository = new JobRepository();

        await Processor(repository).RunAsync(job, CancellationToken.None);

        Assert.Equal(0, job.AttemptCount);
        Assert.Equal(job.CreatedAt.AddSeconds(10), repository.NextAttemptAt);
    }

    private static PlateAcquisitionJob Job(int arrivedSecondsAgo, int attempts) => new()
    {
        Id = 1,
        WorkflowId = "sess_test",
        InteractionId = "interaction",
        CameraIdentifier = Camera,
        AttemptCount = attempts,
        CreatedAt = BusinessClock.Now.AddSeconds(-arrivedSecondsAgo)
    };

    private static object Capture(string plate, DateTime capturedAt) => new
    {
        cameraId = Camera,
        licensePlate = plate,
        captureTime = capturedAt.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture),
        enterID = "enter-" + plate
    };

    private static PlateAcquisitionProcessor Processor(JobRepository repository, params object[] captures)
    {
        var body = System.Text.Json.JsonSerializer.Serialize(new { code = 200, message = "success", data = captures });
        var http = new HttpClient(new PartnerHandler(body));
        var configuration = new ConfigurationBuilder().AddInMemoryCollection(new Dictionary<string, string?>
        {
            ["Database:Enabled"] = "false"
        }).Build();
        var client = new PartnerLicensePlateClient(http, Options.Create(new IntegrationOptions
        {
            AppSecret = "test-secret",
            PartnerBaseUrl = "http://partner.local",
            LicensePlatePath = "/api/v1/licensePlateRecords",
            PartnerTimeoutMs = 3000
        }), repository, configuration, NullLogger<PartnerLicensePlateClient>.Instance, new HttpContextAccessor());
        return new PlateAcquisitionProcessor(repository, client, configuration, NullLogger.Instance);
    }

    private sealed class PartnerHandler(string body) : HttpMessageHandler
    {
        protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken) =>
            Task.FromResult(new HttpResponseMessage(HttpStatusCode.OK)
            {
                Content = new StringContent(body, Encoding.UTF8, "application/json")
            });
    }

    private sealed class JobRepository : NullIntegrationRepository
    {
        public LicensePlateRecord? Completed;
        public string? ProgressError;
        public string? FailedError;
        public DateTime? NextAttemptAt;

        public override bool TryBeginPlateAcquisitionAttempt(PlateAcquisitionJob job, int maxAttempts, DateTime now)
        {
            job.AttemptCount++;
            return true;
        }

        public override bool SavePlateAcquisitionProgress(PlateAcquisitionJob job, string? candidateJson,
            DateTime nextAttemptAt, string? error, DateTime now)
        {
            NextAttemptAt = nextAttemptAt;
            ProgressError = error;
            return true;
        }

        public override void CompletePlateAcquisitionJob(PlateAcquisitionJob job, LicensePlateRecord record) => Completed = record;

        public override void FailPlateAcquisitionJob(PlateAcquisitionJob job, string error, DateTime nextAttemptAt,
            int maxAttempts, DateTime updatedAt) => FailedError = error;
    }
}
