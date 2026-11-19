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
        await handler.UnavailableSent.Task.WaitAsync(TimeSpan.FromSeconds(5));
        await worker.StopAsync(CancellationToken.None);
        Assert.False(result.HasDimensionPayload);
        Assert.Single(handler.Bodies);
    }

    [Fact]
    public async Task UnreportedTerminalFailure_ResumesPublicationWithoutRerunningAnalysis()
    {
        var root = TestSupport.TempDirectory();
        var pair = Pair(root);
        var outage = new Handler((_, _) => Task.FromResult(TestSupport.Json("analysis failed", 503)),
            unavailableStatus: 503);
        var options = Options.Create(TestSupport.Options(o => o.CaptureDirectory = root));
        var store = new AnalysisTaskStore(options);
        using (var worker = Create(root, outage))
        {
            await worker.StartAsync(CancellationToken.None);
            worker.Enqueue(pair);
            await WaitFor(worker, r => r.PermanentFailure);
            await outage.UnavailableSent.Task.WaitAsync(TimeSpan.FromSeconds(5));
            await worker.StopAsync(CancellationToken.None);
        }
        Assert.Equal(3, outage.Bodies.Count);
        Assert.True(Assert.Single(store.Load()).UnavailablePublicationPending);

        var online = new Handler((_, _) => throw new InvalidOperationException("不得重新分析已终局的任务"));
        using var resumed = Create(root, online);
        await resumed.StartAsync(CancellationToken.None);
        await online.UnavailableSent.Task.WaitAsync(TimeSpan.FromSeconds(5));
        var deadline = DateTime.UtcNow.AddSeconds(5);
        while (Assert.Single(store.Load()).UnavailablePublicationPending)
        {
            Assert.True(DateTime.UtcNow < deadline, "等待缺测上报回执落盘超时");
            await Task.Delay(10);
        }
        await resumed.StopAsync(CancellationToken.None);
        Assert.Empty(online.Bodies);
        Assert.True(resumed.Find(pair.WorkflowId)!.PermanentFailure);
        var report = JsonNode.Parse(Assert.Single(online.UnavailableBodies))!;
        Assert.Equal(pair.WorkflowId, report["workflowId"]!.GetValue<string>());
        Assert.Equal(pair.Primary.CameraIdentifier, report["cameraA"]!.GetValue<string>());
        Assert.Equal(pair.Secondary.CapturedAt, report["capturedAt"]!.GetValue<DateTime>());
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

    private static PairedPcdCapture Pair(string root) => PairedPcdCapture.Create(
        new PcdCapture(Path.Combine(root, "a.pcd"), "CAM_004", new DateTime(2026, 11, 3, 10, 0, 0), "wf-recovery"),
        new PcdCapture(Path.Combine(root, "b.pcd"), "CAM_005", new DateTime(2026, 11, 3, 10, 0, 1), "wf-recovery"),
        new DateTime(2026, 11, 3, 10, 0, 2));

    private static async Task<AnalysisRecord> WaitFor(AnalysisWorker worker, Func<AnalysisRecord, bool> done)
    {
        var deadline = DateTime.UtcNow.AddSeconds(5);
        while (true)
        {
            if (worker.Find("wf-recovery") is { } record && done(record))
                return record;
            Assert.True(DateTime.UtcNow < deadline, "等待分析状态超时");
            await Task.Delay(10);
        }
    }

    private sealed class Handler(Func<HttpRequestMessage, CancellationToken, Task<HttpResponseMessage>> respond,
        int unavailableStatus = 200) : HttpMessageHandler
    {
        public ConcurrentQueue<string> Bodies { get; } = new();
        public ConcurrentQueue<string> UnavailableBodies { get; } = new();
        public TaskCompletionSource UnavailableSent { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);

        protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
        {
            var body = await request.Content!.ReadAsStringAsync(cancellationToken);
            if (request.RequestUri!.AbsolutePath == "/analyze-pair/unavailable")
            {
                UnavailableBodies.Enqueue(body);
                UnavailableSent.TrySetResult();
                return TestSupport.Json("""{"status":"recorded"}""", unavailableStatus);
            }
            Bodies.Enqueue(body);
            return await respond(request, cancellationToken);
        }
    }
}
