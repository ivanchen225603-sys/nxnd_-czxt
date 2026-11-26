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
    public async Task Field_boolean_response_binds_one_capture_and_audits_the_original_payload()
    {
        var job = Job(20, 0);
        var body = JsonSerializer.Serialize(new { success = true, msg = "ok", data = new[]
        {
            new { CAMERAID = Camera, LICENSEPLATE = "宁A12345",
                CAPTURETIME = job.CreatedAt.AddSeconds(8).ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture),
                ENTERID = 9007199254740993L }
        } });
        using var http = new HttpClient(new PartnerHandler(body));
        var repo = new JobRepository();
        var config = new ConfigurationBuilder().Build();
        var client = new PartnerLicensePlateClient(http, Options.Create(new IntegrationOptions
        {
            AppSecret = "test-secret", PartnerBaseUrl = "http://partner.local"
        }), repo, config, NullLogger<PartnerLicensePlateClient>.Instance, new HttpContextAccessor());
        await new PlateAcquisitionProcessor(repo, client, config, NullLogger.Instance).RunAsync(job, default);
        var bound = Assert.IsType<LicensePlateRecord>(repo.Completed);
        Assert.Equal(job.WorkflowId, bound.WorkflowId);
        Assert.Equal("宁A12345", bound.LicensePlate);
        Assert.Equal("9007199254740993", bound.EnterId);
        var audit = Assert.IsType<ApiAccessLogRecord>(repo.Audit);
        Assert.Equal(200, audit.BusinessCode);
        Assert.Equal("PartnerResponseReceived", audit.LifecycleStage);
        using var original = JsonDocument.Parse(audit.ResponseBody!);
        Assert.True(original.RootElement.GetProperty("success").GetBoolean());
        Assert.Equal(9007199254740993L, original.RootElement.GetProperty("data")[0].GetProperty("ENTERID").GetInt64());
    }

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

        var nextRepository = new JobRepository { OffsetSeconds = [11, 11, 11],
            NeighborRows = [new(job.CreatedAt, repository.Completed is not null)] };
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

    [Theory]
    [InlineData(16)]
    [InlineData(21)]
    public async Task Unresolved_previous_passage_cannot_lose_a_late_capture_to_the_next_car(int capturedSeconds)
    {
        var next = Job(arrivedSecondsAgo: 20, attempts: 0);
        var prior = next.CreatedAt.AddSeconds(-20);
        var repo = new JobRepository { NeighborRows = [new(prior, false)] };
        await Processor(repo, Capture("宁A12345", prior.AddSeconds(capturedSeconds), "prior"))
            .RunAsync(next, default);
        Assert.Null(repo.Completed);
        Assert.Contains("prior_passage_capture", repo.Decisions);
    }

    [Fact]
    public async Task Cold_start_checks_older_unresolved_passages_even_after_an_identified_neighbor()
    {
        var job = Job(20, 0);
        var repo = new JobRepository { NeighborRows = [
            new(job.CreatedAt.AddSeconds(-40), false), new(job.CreatedAt.AddSeconds(-20), true)] };
        await Processor(repo, Capture("宁A12345", job.CreatedAt.AddSeconds(1), "old"))
            .RunAsync(job, default);
        Assert.Null(repo.Completed);
        Assert.Contains("prior_passage_capture", repo.Decisions);
    }

    [Theory]
    [InlineData(24)]
    [InlineData(93)]
    [InlineData(1936)]
    public async Task Cold_start_can_pass_an_older_failure_when_a_confirmed_capture_separates_it(int priorDistance)
    {
        var job = Job(20, 0);
        var at = job.CreatedAt;
        var repo = new JobRepository { NeighborRows = [
            new(at.AddSeconds(-priorDistance), false),
            new(at.AddSeconds(-12), true, Second(at.AddSeconds(-13)))] };
        var response = new[] {
            Capture("宁A06241F", at.AddSeconds(-3), "0001"),
            Capture("宁A09185", at.AddSeconds(-21), "9999") };
        await Processor(repo, response).RunAsync(job, default);
        Assert.Equal("0001", repo.Completed?.EnterId);
    }

    [Fact]
    public async Task Partner_only_trips_after_an_anchor_cannot_force_a_rank_match()
    {
        var job = Job(20, 0);
        var at = job.CreatedAt;
        var repo = new JobRepository { NeighborRows = [
            new(at.AddMinutes(-15), false), new(at.AddSeconds(-12), true, Second(at.AddSeconds(-10)))] };
        await Processor(repo, Capture("宁A09185", at.AddSeconds(-2), "9000"),
            Capture("宁A27066", at.AddSeconds(1), "0001")).RunAsync(job, default);
        Assert.Null(repo.Completed);
        Assert.Contains("ambiguous", repo.Decisions);
    }

    [Fact]
    public async Task An_unresolved_trip_after_the_last_anchor_still_competes_for_a_late_capture()
    {
        var job = Job(20, 0);
        var at = job.CreatedAt;
        var repo = new JobRepository { NeighborRows = [
            new(at.AddSeconds(-40), true, Second(at.AddSeconds(-42))), new(at.AddSeconds(-20), false)] };
        await Processor(repo, Capture("宁A09185", at.AddSeconds(1), "001")).RunAsync(job, default);
        Assert.Null(repo.Completed);
        Assert.Contains("prior_passage_capture", repo.Decisions);
    }

    [Fact]
    public async Task Unresolved_old_passage_outside_partner_snapshot_does_not_block_an_isolated_new_capture()
    {
        var job = Job(20, 0);
        var repo = new JobRepository { NeighborRows = [new(job.CreatedAt.AddHours(-2), false)] };
        await Processor(repo, Capture("宁A12345", job.CreatedAt.AddSeconds(3), "new")).RunAsync(job, default);
        Assert.Equal("new", repo.Completed?.EnterId);
    }

    [Fact]
    public void Calibration_requires_a_unique_consistent_group_of_three()
    {
        static PlateCaptureWindow Window(params int[] offsets) => PlateCaptureWindow.FromHistory(
            offsets.Select(x => TimeSpan.FromSeconds(x)).ToArray(), 5, 30, 5);
        Assert.False(Window(-37, -37).IsCalibrated);
        Assert.False(Window(0, 9, 18, 27, 36).IsCalibrated);
        Assert.False(Window(0, 2, 4, 6).IsCalibrated);
        Assert.True(Window(-38, -37, -37, -36, 80).IsCalibrated);
    }

    [Fact]
    public async Task Reduced_budget_finishes_a_recovered_job_without_another_request()
    {
        var job = Job(20, 1);
        var repo = new JobRepository { MaxAttempts = 1 };
        await Processor(repo, Capture("宁A12345", job.CreatedAt.AddSeconds(3))).RunAsync(job, default);
        Assert.StartsWith(PlateFailureCode.NoCapture, repo.FailedError);
        Assert.Equal(1, job.AttemptCount);
        Assert.Null(repo.Completed);
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
            PartnerTimeoutMs = 3000,
            PartnerRetryCount = repository.MaxAttempts
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
        public PlateArrivalNeighbor[] NeighborRows = [];
        public List<string> Decisions = [];
        public int MaxAttempts = 3;
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
            Completed = window.MatchUnique(candidates, ArrivalTime ?? job.CreatedAt,
                [.. NeighborRows, .. Neighbors.Select(at => new PlateArrivalNeighbor(at, false))],
                window.IsCalibrated ? null : BusinessClock.Now.AddSeconds(5),
                candidate => ClaimedIds.Contains(candidate.EnterId!), (_, reason) => Decisions.Add(reason));
            return Completed is not null;
        }

        public override void FailPlateAcquisitionJob(PlateAcquisitionJob job, string error, DateTime nextAttemptAt,
            int maxAttempts, DateTime updatedAt) => FailedError = error;
    }
}
