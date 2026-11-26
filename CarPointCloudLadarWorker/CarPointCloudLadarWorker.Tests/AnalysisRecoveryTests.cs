using System.Collections.Concurrent;
using System.Text.Json.Nodes;
using CarPointCloudLadarWorker.Analysis;
using CarPointCloudLadarWorker.Api;
using CarPointCloudWebApi.Application;
using CarPointCloudWebApi.Domain;

namespace CarPointCloudLadarWorker.Tests;

public sealed class AnalysisRecoveryTests
{
    private const string Success = """
        {"success":true,"outputs":{"loadStatus":{"status":"Empty"},"dimensions":{"carriage":{"length":10.6}},
         "measurementStatus":{"reviewStatus":"pending"}},
         "integrationPayloads":{"vehicleDimensions":{"dimensions":{"trailerLength":1060}}},
         "persistence":{"status":"inserted"}}
        """;

    [Fact]
    public async Task HttpFailures_RetryThenSucceed_WithoutChangingPairOrRequest()
    {
        var root = TestSupport.TempDirectory();
        var calls = 0;
        var handler = new Handler((_, _) => Task.FromResult(
            ++calls < 3 ? TestSupport.Json("busy", 503) : TestSupport.Json(Success)));
        using var worker = Create(root, handler);
        await worker.StartAsync(CancellationToken.None);
        worker.Enqueue(Pair(root));

        var result = await WaitFor(worker, r => r.Stage == AnalysisStage.Completed);
        await worker.StopAsync(CancellationToken.None);

        Assert.Equal(3, calls);
        Assert.Single(handler.Bodies.Distinct());
        Assert.Null(result.Error);
        Assert.True(result.HasDimensionPayload);
        Assert.False(result.PermanentFailure);
        Assert.Equal("pending", (string?)result.MeasurementStatus!["reviewStatus"]);
        Assert.Equal("inserted", (string?)result.Persistence!["status"]);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public async Task PlateLookupFailureDoesNotConsumeAnAnalysisAttempt(bool timesOut)
    {
        var root = TestSupport.TempDirectory();
        var options = Options.Create(TestSupport.Options(o =>
        {
            o.CaptureDirectory = root;
            o.EnableApiRequests = true;
            o.FetchLicensePlateOnCapture = true;
            o.WebApiUrl = "http://127.0.0.1:1";
            o.AppSecret = "test-secret";
        }));
        var handler = new Handler((_, _) => Task.FromResult(TestSupport.Json(Success)));
        using var analyzerHttp = new HttpClient(handler);
        using var plateHttp = new HttpClient(new FailedPlateLookup(timesOut));
        using var worker = new AnalysisWorker(new AnalyzerClient(analyzerHttp, options),
            new PlateClient(plateHttp, options, NullLogger<PlateClient>.Instance),
            new AnalysisTaskStore(options), options, TimeProvider.System, NullLogger<AnalysisWorker>.Instance);
        await worker.StartAsync(CancellationToken.None);
        worker.Enqueue(Pair(root));

        Assert.Null((await WaitFor(worker, r => r.Stage == AnalysisStage.Completed)).Error);
        await worker.StopAsync(CancellationToken.None);
        var sent = JsonNode.Parse(Assert.Single(handler.Bodies))!;
        Assert.Equal(string.Empty, sent["plate"]!.GetValue<string>());
        Assert.Equal(0, Assert.Single(new AnalysisTaskStore(options).Load()).FailureCount);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    public async Task PendingTask_RecoversWithOriginalIdentityAndStoredCaptureTime(bool legacyTask, bool primaryIsLater)
    {
        var root = TestSupport.TempDirectory();
        var pair = Pair(root);
        if (primaryIsLater)
            pair = pair with { Primary = pair.Primary with { CapturedAt = pair.Secondary.CapturedAt.AddMilliseconds(250) } };
        var expectedTime = legacyTask || primaryIsLater ? pair.Primary.CapturedAt : pair.Secondary.CapturedAt;
        var handler = new Handler((_, _) => Task.FromResult(TestSupport.Json(Success)));
        using (var producer = Create(root, handler))
        {
            producer.Enqueue(pair);
            var path = Assert.Single(Directory.GetFiles(Path.Combine(root, "analysis-tasks"), "*.json"));
            if (legacyTask)
            {
                var task = JsonNode.Parse(await File.ReadAllTextAsync(path))!;
                task["Record"]!["CapturedAt"] = JsonValue.Create(pair.Primary.CapturedAt);
                task["UnavailablePublicationPending"] = true;
                await File.WriteAllTextAsync(path, task.ToJsonString());
            }
            Assert.Empty(handler.Bodies);
        }

        using var recovered = Create(root, handler);
        await recovered.StartAsync(CancellationToken.None);
        var result = await WaitFor(recovered, r => r.Stage == AnalysisStage.Completed);
        await recovered.StopAsync(CancellationToken.None);

        var sent = JsonNode.Parse(Assert.Single(handler.Bodies))!;
        Assert.Equal(pair.WorkflowId, (string?)sent["workflowId"]);
        Assert.Equal(pair.VehicleCaptureId, (string?)sent["pairId"]);
        Assert.Equal(pair.Primary.PcdPath, (string?)sent["pcdPathA"]);
        Assert.Equal(pair.Secondary.PcdPath, (string?)sent["pcdPathB"]);
        Assert.Equal(expectedTime, sent["capturedAt"]!.GetValue<DateTime>());
        Assert.Equal(expectedTime, result.CapturedAt);
        Assert.Equal(pair.VehicleCaptureId, result.PairId);
    }

    [Fact]
    public async Task CorruptedTaskFile_IsQuarantined_AndDoesNotBlockOtherRecovery()
    {
        var root = TestSupport.TempDirectory();
        var handler = new Handler((_, _) => Task.FromResult(TestSupport.Json(Success)));

        // 先落一份正常的待恢复任务，再人为写入一份截断 JSON。
        using (var producer = Create(root, handler))
            producer.Enqueue(Pair(root));

        var taskDirectory = Path.Combine(root, "analysis-tasks");
        var broken = Path.Combine(taskDirectory, "broken.json");
        const string truncated = "{\"pair\":";
        File.WriteAllText(broken, truncated);
        Assert.Equal(2, Directory.GetFiles(taskDirectory, "*.json").Length);

        using var recovered = Create(root, handler);
        await recovered.StartAsync(CancellationToken.None);
        var result = await WaitFor(recovered, r => r.Stage == AnalysisStage.Completed);
        await recovered.StopAsync(CancellationToken.None);

        Assert.Null(result.Error);
        // 正常任务仍然恢复并完成；损坏文件保留原始内容但已移出任务目录。
        Assert.False(File.Exists(broken));
        var quarantined = Assert.Single(Directory.GetFiles(
            Path.Combine(taskDirectory, "_quarantine"), "*.json"));
        Assert.Equal(truncated, File.ReadAllText(quarantined));
        Assert.Single(Directory.GetFiles(taskDirectory, "*.json"));
    }

    [Fact]
    public async Task TemporarilyUnreadableTask_IsNotQuarantined_AndRecoversAfterUnlock()
    {
        var root = TestSupport.TempDirectory();
        var handler = new Handler((_, _) => Task.FromResult(TestSupport.Json(Success)));
        using (var producer = Create(root, handler))
            producer.Enqueue(Pair(root));

        var directory = Path.Combine(root, "analysis-tasks");
        var path = Assert.Single(Directory.GetFiles(directory, "*.json"));
        var original = File.ReadAllBytes(path);
        // Windows 允许改名但不允许读取，复现正常文件被误移入隔离目录。
        var share = OperatingSystem.IsWindows() ? FileShare.Delete : FileShare.None;
        using (var held = new FileStream(path, FileMode.Open, FileAccess.Read, share))
        using (var blocked = Create(root, handler))
            await Assert.ThrowsAnyAsync<IOException>(() => blocked.StartAsync(CancellationToken.None));

        Assert.Equal(original, File.ReadAllBytes(path));
        Assert.False(Directory.Exists(Path.Combine(directory, "_quarantine")));
        using var recovered = Create(root, handler);
        await recovered.StartAsync(CancellationToken.None);
        var result = await WaitFor(recovered, r => r.Stage == AnalysisStage.Completed);
        await recovered.StopAsync(CancellationToken.None);
        Assert.Null(result.Error);
        Assert.Single(handler.Bodies);
    }

    [Fact]
    public void AtomicSave_LeavesNoTemporaryFileInTaskDirectory()
    {
        var root = TestSupport.TempDirectory();
        var handler = new Handler((_, _) => Task.FromResult(TestSupport.Json(Success)));
        using var worker = Create(root, handler);
        worker.Enqueue(Pair(root));

        var directory = Path.Combine(root, "analysis-tasks");
        Assert.Single(Directory.GetFiles(directory, "*.json"));
        Assert.Empty(Directory.GetFiles(directory, "*.tmp"));
        Assert.DoesNotContain(Directory.EnumerateFiles(directory),
            path => Path.GetFileName(path).StartsWith(".", StringComparison.Ordinal));
    }

    [Fact]
    public async Task RestartDuringRetry_PreservesFailureCountAndDueTime_ThenExhausts()
    {
        var root = TestSupport.TempDirectory();
        var calls = 0;
        var handler = new Handler((_, _) =>
        {
            calls++;
            throw new HttpRequestException("connection refused");
        });
        using (var first = Create(root, handler, retryDelay: 1))
        {
            await first.StartAsync(CancellationToken.None);
            first.Enqueue(Pair(root));
            var waiting = await WaitFor(first, r => r.Stage == AnalysisStage.Queued && r.Error is not null);
            Assert.False(waiting.PermanentFailure);
            var progress = CaptureProgressBuilder.Build("wf-recovery", new VehicleArrivalRecord(), null,
                System.Text.Json.JsonSerializer.SerializeToNode(ApiResponses.Analysis(waiting)), null);
            Assert.False(progress.Terminal);
            await first.StopAsync(CancellationToken.None);
        }

        var options = Options.Create(TestSupport.Options(o => o.CaptureDirectory = root));
        var saved = Assert.Single(new AnalysisTaskStore(options).Load());
        Assert.Equal(1, saved.FailureCount);
        Assert.NotNull(saved.NextAttemptAt);

        DateTimeOffset? resumedAt = null;
        var resumedHandler = new Handler((_, _) =>
        {
            resumedAt ??= DateTimeOffset.UtcNow;
            calls++;
            return Task.FromResult(TestSupport.Json("""{"success":false,"error":"failed"}"""));
        });
        // 即使重启后配置改成零等待，已保存的这次到期时间仍须遵守。
        using var second = Create(root, resumedHandler);
        await second.StartAsync(CancellationToken.None);
        var failed = await WaitFor(second, r => r.Stage == AnalysisStage.Failed);
        await second.StopAsync(CancellationToken.None);

        Assert.True(resumedAt >= saved.NextAttemptAt);
        Assert.Equal(3, calls);
        Assert.True(failed.PermanentFailure);
        Assert.Equal(3, Assert.Single(new AnalysisTaskStore(options).Load()).FailureCount);

        using var third = Create(root, resumedHandler);
        await third.StartAsync(CancellationToken.None);
        var terminal = third.Find("wf-recovery")!;
        Assert.Equal(failed.Stage, terminal.Stage);
        Assert.Equal(failed.Error, terminal.Error);
        Assert.True(terminal.PermanentFailure);
        await third.StopAsync(CancellationToken.None);
        Assert.Equal(3, calls);
    }

    [Fact]
    public async Task RestartDuringCall_ReplaysOriginalRequest_AndCompletedResultIsRestoredWithoutCallingAgain()
    {
        var root = TestSupport.TempDirectory();
        var entered = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var interrupted = new Handler(async (_, token) =>
        {
            entered.SetResult();
            await Task.Delay(Timeout.InfiniteTimeSpan, token);
            return TestSupport.Json(Success);
        });
        using (var first = Create(root, interrupted))
        {
            await first.StartAsync(CancellationToken.None);
            first.Enqueue(Pair(root));
            await entered.Task.WaitAsync(TimeSpan.FromSeconds(5));
            await first.StopAsync(CancellationToken.None);
        }

        var handler = new Handler((_, _) => Task.FromResult(TestSupport.Json(Success)));
        AnalysisRecord completed;
        using (var second = Create(root, handler))
        {
            await second.StartAsync(CancellationToken.None);
            completed = await WaitFor(second, r => r.Stage == AnalysisStage.Completed);
            await second.StopAsync(CancellationToken.None);
        }
        Assert.Equal(Assert.Single(interrupted.Bodies), Assert.Single(handler.Bodies));

        using var third = Create(root, handler);
        await third.StartAsync(CancellationToken.None);
        var restored = third.Find("wf-recovery")!;
        Assert.Equal(AnalysisStage.Completed, restored.Stage);
        Assert.Equal(completed.PairId, restored.PairId);
        Assert.True(JsonNode.DeepEquals(completed.Dimensions, restored.Dimensions));
        Assert.True(JsonNode.DeepEquals(completed.Persistence, restored.Persistence));
        Assert.Equal(1060, restored.DimensionPayload?["dimensions"]?["trailerLength"]?.GetValue<int>());
        Assert.True(JsonNode.DeepEquals(completed.DimensionPayload, restored.DimensionPayload));
        await third.StopAsync(CancellationToken.None);
        Assert.Single(handler.Bodies);
    }

    [Fact]
    public async Task SuccessfulAnalysisWithoutDimensions_IsNotRetried()
    {
        var root = TestSupport.TempDirectory();
        var handler = new Handler((_, _) => Task.FromResult(TestSupport.Json(
            """{"success":true,"outputs":{"loadStatus":{"status":"Loaded"}}}""")));
        using var worker = Create(root, handler);
        await worker.StartAsync(CancellationToken.None);
        worker.Enqueue(Pair(root));
        var result = await WaitFor(worker, r => r.Stage == AnalysisStage.Completed);
        await worker.StopAsync(CancellationToken.None);
        Assert.False(result.HasDimensionPayload);
        Assert.Single(handler.Bodies);
    }

    [Fact]
    public async Task DisabledAnalysis_RestoresStatusButDoesNotCallAnalyzer()
    {
        var root = TestSupport.TempDirectory();
        var handler = new Handler((_, _) => Task.FromResult(TestSupport.Json(Success)));
        using (var producer = Create(root, handler))
            producer.Enqueue(Pair(root));
        using var worker = Create(root, handler, enabled: false);
        await worker.StartAsync(CancellationToken.None);
        Assert.Equal(AnalysisStage.Queued, worker.Find("wf-recovery")!.Stage);
        await worker.StopAsync(CancellationToken.None);
        Assert.Empty(handler.Bodies);
    }

    [Theory]
    [InlineData(-1, 10)]
    [InlineData(2, -1)]
    public void RetryConfiguration_IsValidatedAtStartup(int retries, int delay)
    {
        var options = TestSupport.Options(o => { o.AnalysisRetryCount = retries; o.AnalysisRetryDelaySeconds = delay; });
        Assert.Throws<InvalidOperationException>(options.Validate);
    }

    private static AnalysisWorker Create(string root, Handler handler, int retryDelay = 0, bool enabled = true)
    {
        var options = Options.Create(TestSupport.Options(o =>
        {
            o.CaptureDirectory = root;
            o.AnalysisRetryDelaySeconds = retryDelay;
            o.EnablePythonAnalyzer = enabled;
        }));
        options.Value.Validate();
        return new AnalysisWorker(new AnalyzerClient(new HttpClient(handler, disposeHandler: false), options),
            new PlateClient(new HttpClient(), options, NullLogger<PlateClient>.Instance),
            new AnalysisTaskStore(options), options, TimeProvider.System, NullLogger<AnalysisWorker>.Instance);
    }

    private static PairedPcdCapture Pair(string root, string workflow = "wf-recovery") => PairedPcdCapture.Create(
        new PcdCapture(Path.Combine(root, workflow + "-a.pcd"), "CAM_004", new DateTime(2026, 11, 3, 10, 0, 0), workflow),
        new PcdCapture(Path.Combine(root, workflow + "-b.pcd"), "CAM_005", new DateTime(2026, 11, 3, 10, 0, 1), workflow),
        new DateTime(2026, 11, 3, 10, 0, 2));

    [Fact]
    public async Task RetryWaitReleasesOnlyAnalyzerSlotForNextVehicle()
    {
        var root = TestSupport.TempDirectory();
        var handler = new Handler(async (request, token) =>
        {
            var workflow = JsonNode.Parse(await request.Content!.ReadAsStringAsync(token))!["workflowId"]!.GetValue<string>();
            return workflow == "wf-recovery" ? TestSupport.Json("busy", 503) : TestSupport.Json(Success);
        });
        using var worker = Create(root, handler, retryDelay: 60);
        await worker.StartAsync(CancellationToken.None);
        worker.Enqueue(Pair(root));
        await WaitFor(worker, record => record.Stage == AnalysisStage.Queued && record.Error is not null);
        worker.Enqueue(Pair(root, "next"));
        await WaitFor(worker, record => record.Stage == AnalysisStage.Completed, "next");
        await worker.StopAsync(CancellationToken.None);
        Assert.Equal(2, handler.Bodies.Count);
    }

    [Fact]
    public async Task TwoConfiguredConsumersRunTwoIndependentPairsAndQueueTheThird()
    {
        var root = TestSupport.TempDirectory();
        var options = Options.Create(TestSupport.Options(o => { o.CaptureDirectory = root; o.MaxConcurrentAnalyses = 2; }));
        var firstTwo = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var release = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var entered = 0;
        var handler = new Handler(async (_, token) =>
        {
            if (Interlocked.Increment(ref entered) == 2) firstTwo.TrySetResult();
            await release.Task.WaitAsync(token);
            return TestSupport.Json(Success);
        });
        using var analyzerHttp = new HttpClient(handler);
        using var plateHttp = new HttpClient();
        using var worker = new AnalysisWorker(new AnalyzerClient(analyzerHttp, options),
            new PlateClient(plateHttp, options, NullLogger<PlateClient>.Instance),
            new AnalysisTaskStore(options), options, TimeProvider.System, NullLogger<AnalysisWorker>.Instance);
        await worker.StartAsync(CancellationToken.None);
        try
        {
            foreach (var id in new[] { "one", "two", "three" }) worker.Enqueue(Pair(root, id));
            await firstTwo.Task.WaitAsync(TimeSpan.FromSeconds(5));
            Assert.Equal(2, Volatile.Read(ref entered));
            Assert.Equal(AnalysisStage.Queued, worker.Find("three")!.Stage);
            release.SetResult();
            foreach (var id in new[] { "one", "two", "three" })
                await WaitFor(worker, record => record.Stage == AnalysisStage.Completed, id);
            Assert.Equal(3, entered);
        }
        finally { await worker.StopAsync(CancellationToken.None); }
    }

    [Theory]
    [InlineData("confirmed", AnalysisStage.Completed, 1)]
    [InlineData("rejected", AnalysisStage.Failed, 0)]
    [InlineData("insufficient", AnalysisStage.Failed, 0)]
    public async Task PendingStaticEvidenceIsDurableAndNeverStartsAnalyzerEarly(
        string finalStatus, AnalysisStage finalStage, int expectedAnalyses)
    {
        var root = TestSupport.TempDirectory();
        var options = Options.Create(TestSupport.Options(o =>
        {
            o.CaptureDirectory = root;
            o.EnableApiRequests = true;
            o.AppSecret = "test-only";
        }));
        var now = DateTimeOffset.UtcNow;
        var window = new StaticCaptureWindow("same-stop", now.AddSeconds(10));
        var sensor = new LadarDeviceOptions
        {
            VerifiedMaxScanDurationMs = 4000, VerifiedMaxCallbackDelayMs = 50, VerifiedClockUncertaintyMs = 10
        };
        var frame = StaticFrameEvidence.FromCallback(window, now, sensor);
        Assert.Equal(now.AddMilliseconds(-4060), frame.ScanStartUtc);
        Assert.Equal(now.AddMilliseconds(10), frame.ScanEndUtc);
        Assert.Null(StaticFrameEvidence.FromCallback(window, now, new()).ScanStartUtc);
        var source = Pair(root);
        var pair = source with { Primary = source.Primary with { StaticEvidence = frame },
            Secondary = source.Secondary with { StaticEvidence = frame } };
        var attempts = 0;
        using var evidenceHttp = new HttpClient(new Handler(async (request, token) =>
        {
            Assert.Equal(HttpMethod.Post, request.Method);
            Assert.EndsWith("/vehicleWorkflows/wf-recovery/staticEvidence", request.RequestUri!.AbsolutePath);
            Assert.True(request.Headers.Contains("Signature"));
            var data = JsonNode.Parse(await request.Content!.ReadAsStringAsync(token))!;
            Assert.Equal(pair.VehicleCaptureId, data["pairId"]!.GetValue<string>());
            Assert.Equal("same-stop", data["segmentId"]!.GetValue<string>());
            Assert.Equal("CAM_005", data["secondary"]!["cameraIdentifier"]!.GetValue<string>());
            Assert.Equal(frame.ScanStartUtc, data["primary"]!["scanStartUtc"]!.GetValue<DateTimeOffset>());
            var state = Interlocked.Increment(ref attempts) == 1 ? "pending" : finalStatus;
            return TestSupport.Json(System.Text.Json.JsonSerializer.Serialize(new
                { code = 200, data = new { status = state, reason = "observed" } }));
        }));
        var handler = new Handler((_, _) => Task.FromResult(TestSupport.Json(Success)));
        using var analyzerHttp = new HttpClient(handler);
        using var plateHttp = new HttpClient();
        AnalysisWorker Start() => new(new AnalyzerClient(analyzerHttp, options),
            new PlateClient(plateHttp, options, NullLogger<PlateClient>.Instance), new AnalysisTaskStore(options),
            options, TimeProvider.System, NullLogger<AnalysisWorker>.Instance, new StaticEvidenceClient(evidenceHttp, options));
        using (var first = Start())
        {
            await first.StartAsync(CancellationToken.None);
            first.Enqueue(pair);
            await WaitFor(first, record => record.StaticReason == "observed");
            await first.StopAsync(CancellationToken.None);
            Assert.Empty(handler.Bodies);
        }
        using var second = Start();
        await second.StartAsync(CancellationToken.None);
        var final = await WaitFor(second, record => record.Stage == finalStage);
        await second.StopAsync(CancellationToken.None);
        Assert.Equal(finalStatus, final.StaticStatus);
        Assert.Equal(expectedAnalyses, handler.Bodies.Count);
        Assert.Equal(0, Assert.Single(new AnalysisTaskStore(options).Load()).FailureCount);
    }

    private static async Task<AnalysisRecord> WaitFor(AnalysisWorker worker, Func<AnalysisRecord, bool> done,
        string workflow = "wf-recovery")
    {
        var deadline = DateTime.UtcNow.AddSeconds(5);
        while (true)
        {
            if (worker.Find(workflow) is { } record && done(record))
                return record;
            Assert.True(DateTime.UtcNow < deadline, "等待分析状态超时");
            await Task.Delay(10);
        }
    }

    private sealed class Handler(Func<HttpRequestMessage, CancellationToken, Task<HttpResponseMessage>> respond) : HttpMessageHandler
    {
        public ConcurrentQueue<string> Bodies { get; } = new();

        protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
        {
            Bodies.Enqueue(await request.Content!.ReadAsStringAsync(cancellationToken));
            return await respond(request, cancellationToken);
        }
    }

    private sealed class FailedPlateLookup(bool timesOut) : HttpMessageHandler
    {
        protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken) =>
            timesOut ? Task.FromException<HttpResponseMessage>(new TaskCanceledException("lookup timed out"))
                     : Task.FromResult(TestSupport.Json("""{"code":200,"data":false}"""));
    }
}
