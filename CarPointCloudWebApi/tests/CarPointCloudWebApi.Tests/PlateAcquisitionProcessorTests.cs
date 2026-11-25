using Xunit;
using System.Globalization;
using System.Net;
using System.Text;
using System.Text.Json;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Tests;

public sealed class PlateAcquisitionProcessorTests
{
    private const string Camera = "CAM_001";

    [Fact]
    public async Task Outbound_audit_preserves_response_order_and_redacts_credentials()
    {
        var now = BusinessClock.Now;
        var body = JsonSerializer.Serialize(new
        {
            code = 200, message = "success", appSecret = "must-not-appear",
            data = new[] { Capture("宁A49586", now), Capture("宁A27066", now.AddSeconds(-1), camera: "CAM_002") }
        });
        using var http = new HttpClient(new PartnerHandler(body));
        var repository = new JobRepository();
        var client = new PartnerLicensePlateClient(http, Options.Create(new IntegrationOptions
        {
            AppSecret = "test-secret", PartnerBaseUrl = "http://partner.local"
        }), repository, new ConfigurationBuilder().Build(), NullLogger<PartnerLicensePlateClient>.Instance,
            new HttpContextAccessor());
        var candidates = await client.GetLicensePlateSnapshotForWorkflowAsync(Camera, "workflow", "interaction", 2, CancellationToken.None);
        Assert.Single(candidates);
        var audit = Assert.IsType<ApiAccessLogRecord>(repository.Audit);
        Assert.Equal("workflow", audit.WorkflowId);
        Assert.Equal("interaction", audit.InteractionId);
        Assert.Equal(2, audit.AttemptNo);
        Assert.DoesNotContain("must-not-appear", audit.ResponseBody);
        using var json = JsonDocument.Parse(audit.ResponseBody!);
        Assert.Equal(new[] { "宁A49586", "宁A27066" }, json.RootElement.GetProperty("data").EnumerateArray()
            .Select(item => item.GetProperty("licensePlate").GetString()).ToArray());
    }

    [Fact]
    public async Task Only_the_capture_inside_the_arrival_window_completes_the_job()
    {
        var job = Job(arrivedSecondsAgo: 20, attempts: 0);
        var repository = new JobRepository();
        var processor = Processor(repository,
            Capture("宁A09185", job.CreatedAt.AddSeconds(-30)),
            Capture("宁A27066", job.CreatedAt.AddSeconds(2), camera: "CAM_002"),
            Capture("宁A49586", job.CreatedAt.AddSeconds(5)));

        await processor.RunAsync(job, CancellationToken.None);

        Assert.Equal("宁A49586", repository.Completed?.LicensePlate);
        Assert.Equal(job.WorkflowId, repository.Completed?.WorkflowId);
        Assert.Equal(1, job.AttemptCount);
    }

    [Theory]
    [InlineData(42)]
    [InlineData(-37)]
    public async Task Calibrated_match_uses_the_arrival_time_with_positive_or_negative_clock_offset(int offset)
    {
        var job = Job(arrivedSecondsAgo: 90, attempts: 0);
        var arrival = job.CreatedAt.AddSeconds(-14);
        var repository = new JobRepository { ArrivalTime = arrival, OffsetSeconds = [offset, offset, offset, offset + 2, offset + 80] };
        var capture = arrival.AddSeconds(offset);

        await Processor(repository, Capture("宁A27066", capture, "000042")).RunAsync(job, CancellationToken.None);

        var record = Assert.IsType<LicensePlateRecord>(repository.Completed);
        Assert.Equal("000042", record.EnterId);
        Assert.Equal(Second(capture), record.CaptureTime);
    }

    [Fact]
    public async Task A_single_fresh_capture_is_usable_after_a_long_gap()
    {
        var job = Job(arrivedSecondsAgo: 12, attempts: 0);
        var repository = new JobRepository { OffsetSeconds = [11], ClaimedIds = ["prior-trip"] };

        await Processor(repository, Capture("宁A01210", job.CreatedAt.AddSeconds(11), "new-trip"))
            .RunAsync(job, CancellationToken.None);

        Assert.Equal("new-trip", repository.Completed?.EnterId);
    }

    [Theory]
    [InlineData(10, "capture")]
    [InlineData(22, null)]
    public async Task Cold_start_retry_only_uses_captures_from_the_first_vehicle_window(
        int captureOffset, string? expected)
    {
        var job = Job(arrivedSecondsAgo: 29, attempts: 1);
        var repository = new JobRepository();
        await Processor(repository, Capture("宁A01210", job.CreatedAt.AddSeconds(captureOffset), "capture"))
            .RunAsync(job, CancellationToken.None);

        Assert.Equal(expected, repository.Completed?.EnterId);
    }

    [Fact]
    public async Task Returning_truck_uses_the_new_record_even_if_the_previous_capture_is_newer_in_the_array()
    {
        var job = Job(arrivedSecondsAgo: 20, attempts: 0);
        var repository = new JobRepository { OffsetSeconds = [11], ClaimedIds = ["old-id"] };
        await Processor(repository,
            Capture("宁A49586", job.CreatedAt.AddSeconds(12), "old-id"),
            Capture("宁A49586", job.CreatedAt.AddSeconds(11), "000038"))
            .RunAsync(job, CancellationToken.None);

        Assert.Equal("000038", repository.Completed?.EnterId);
    }

    [Theory]
    [InlineData(4, "this-trip")]
    [InlineData(1, null)]
    public async Task Consecutive_arrivals_bind_only_when_the_capture_windows_distinguish_the_trips(
        int interval, string? expectedEnterId)
    {
        var job = Job(arrivedSecondsAgo: 25, attempts: 0);
        var next = Job(arrivedSecondsAgo: 25 - interval, attempts: 0);
        next.CreatedAt = job.CreatedAt.AddSeconds(interval);
        next.WorkflowId = "next-workflow";
        var repository = new JobRepository { OffsetSeconds = [11, 11, 11], Neighbors = [job.CreatedAt.AddSeconds(interval)] };
        object[] captures = [
            Capture("宁A56751", job.CreatedAt.AddSeconds(11), "this-trip"),
            Capture("宁A45125", job.CreatedAt.AddSeconds(11 + interval), "next-trip")];
        await Processor(repository, captures).RunAsync(job, CancellationToken.None);

        var nextRepository = new JobRepository { OffsetSeconds = [11, 11, 11], Neighbors = [job.CreatedAt] };
        if (repository.Completed?.EnterId is { } claimed) nextRepository.ClaimedIds.Add(claimed);
        await Processor(nextRepository, captures).RunAsync(next, CancellationToken.None);

        Assert.Equal(expectedEnterId, repository.Completed?.EnterId);
        Assert.Equal(expectedEnterId is null ? null : "next-trip", nextRepository.Completed?.EnterId);
        if (expectedEnterId is null) Assert.StartsWith(PlateFailureCode.NoCapture, repository.ProgressError);
    }

    [Theory]
    [InlineData("", "宁A50473", 11, false)]
    [InlineData("first", "宁A50473", 11, true)]
    [InlineData("second", "宁A27066", 11, true)]
    [InlineData("second", "宁A50473", 11, true)]
    public async Task Missing_ids_are_discarded_and_conflicting_eligible_grabs_are_not_bound(
        string enterId, string plate, int second, bool includeFirst)
    {
        var job = Job(arrivedSecondsAgo: 20, attempts: 0);
        var rows = new List<object> { Capture(plate, job.CreatedAt.AddSeconds(second), enterId) };
        if (includeFirst) rows.Add(Capture("宁A27066", job.CreatedAt.AddSeconds(second), "first"));
        var repository = new JobRepository();

        await Processor(repository, rows.ToArray()).RunAsync(job, CancellationToken.None);

        Assert.Null(repository.Completed);
        Assert.StartsWith(PlateFailureCode.NoCapture, repository.ProgressError);
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

        Assert.Equal(1, job.AttemptCount);
        Assert.InRange(repository.RetryDelay!.Value.TotalMilliseconds, 4900, 5000);
        await processor.RunAsync(job, CancellationToken.None);
        Assert.Equal(2, job.AttemptCount);
        Assert.Null(repository.FailedError);
        await processor.RunAsync(job, CancellationToken.None);
        Assert.StartsWith(PlateFailureCode.NoCapture, repository.FailedError);
        Assert.Null(repository.Completed);
        await processor.RunAsync(job, CancellationToken.None);
        Assert.Equal(3, job.AttemptCount);
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

    private static DateTime Second(DateTime time) => time.AddTicks(-(time.Ticks % TimeSpan.TicksPerSecond));

    private static object Capture(string plate, DateTime capturedAt, string? enterId = null, string camera = Camera) => new
    {
        cameraId = camera,
        licensePlate = plate,
        captureTime = capturedAt.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture),
        enterID = enterId ?? "enter-" + plate
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
        public ApiAccessLogRecord? Audit;
        public LicensePlateRecord? Completed;
        public string? ProgressError;
        public string? FailedError;
        public DateTime? NextAttemptAt;
        public TimeSpan? RetryDelay;
        public DateTime? ArrivalTime;
        public int[] OffsetSeconds = [];
        public DateTime[] Neighbors = [];
        public HashSet<string> ClaimedIds = [];

        public override Task InsertApiAccessLogAsync(ApiAccessLogRecord record, CancellationToken cancellationToken = default)
        {
            Audit = record;
            return Task.CompletedTask;
        }

        public override bool TryBeginPlateAcquisitionAttempt(PlateAcquisitionJob job, int maxAttempts, DateTime now)
        {
            if (job.AttemptCount >= maxAttempts) return false;
            job.AttemptCount++;
            return true;
        }

        public override bool SavePlateAcquisitionProgress(PlateAcquisitionJob job, string? candidateJson,
            DateTime nextAttemptAt, string? error, DateTime now)
        {
            NextAttemptAt = nextAttemptAt;
            ProgressError = error;
            if (error is not null) RetryDelay = nextAttemptAt - now;
            return true;
        }

        public override bool TryCompletePlateAcquisitionJob(PlateAcquisitionJob job,
            IReadOnlyList<LicensePlateRecord> candidates)
        {
            var window = PlateCaptureWindow.FromHistory(OffsetSeconds.Select(x => TimeSpan.FromSeconds(x)).ToArray(), 5, 30, 5);
            Completed = window.MatchUnique(candidates, ArrivalTime ?? job.CreatedAt, Neighbors,
                OffsetSeconds.Length == 0 ? BusinessClock.Now.AddSeconds(5) : null,
                candidate => ClaimedIds.Contains(candidate.EnterId!));
            return Completed is not null;
        }

        public override void FailPlateAcquisitionJob(PlateAcquisitionJob job, string error, DateTime nextAttemptAt,
            int maxAttempts, DateTime updatedAt) => FailedError = error;
    }
}
