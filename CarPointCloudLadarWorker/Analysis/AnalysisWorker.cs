using System.Threading.Channels;
using System.Diagnostics;
using CarPointCloudLadarWorker.Config;
using CarPointCloudLadarWorker.Domain;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Analysis;

/// <summary>
/// 逐对调用 Analyzer，并在内存中保留最近的分析记录供 <c>/api/radars/analysis</c> 查询。
/// 任务原子写入 JSON，失败固定重试，启动时恢复未完成任务及最近的终局；
/// 损坏文件被隔离并告警，不阻断其余任务恢复。
/// </summary>
public sealed class AnalysisWorker(
    AnalyzerClient analyzer,
    PlateClient plates,
    AnalysisTaskStore tasks,
    IOptions<LadarCaptureOptions> options,
    TimeProvider time,
    ILogger<AnalysisWorker> logger,
    StaticEvidenceClient? evidence = null) : BackgroundService
{
    private const int MaxRetainedWorkflows = 200;

    private readonly LadarCaptureOptions _options = options.Value;
    private readonly Channel<AnalysisTask> _queue = Channel.CreateUnbounded<AnalysisTask>();
    private readonly Channel<AnalysisTask> _ready = Channel.CreateUnbounded<AnalysisTask>();
    private readonly Channel<AnalysisTask> _pendingEvidence = Channel.CreateUnbounded<AnalysisTask>();
    private readonly object _gate = new();
    private readonly Dictionary<string, AnalysisRecord> _records = [];
    private readonly Queue<string> _order = new();

    /// <summary>排队分析一对 PCD。</summary>
    public void Enqueue(PairedPcdCapture pair)
    {
        var capturedAt = pair.Secondary.CapturedAt > pair.Primary.CapturedAt
            ? pair.Secondary.CapturedAt : pair.Primary.CapturedAt;
        var record = new AnalysisRecord(
            pair.WorkflowId, pair.VehicleCaptureId, AnalysisStage.Queued, time.GetLocalNow().DateTime,
            pair.Primary.CameraIdentifier, pair.Secondary.CameraIdentifier, capturedAt,
            [pair.Primary.PcdPath, pair.Secondary.PcdPath])
        { StaticStatus = pair.Primary.StaticEvidence is null && pair.Secondary.StaticEvidence is null ? null : "pending" };
        var task = new AnalysisTask(pair, record, 0, null);
        Persist(task);
        _queue.Writer.TryWrite(task);
    }

    public override Task StartAsync(CancellationToken cancellationToken)
    {
        foreach (var task in tasks.Load().OrderBy(x => x.Pair.PairedAt))
        {
            var pending = task.Record.Stage is AnalysisStage.Queued or AnalysisStage.Analyzing;
            var restored = pending
                ? task with { Record = task.Record with { Stage = AnalysisStage.Queued } }
                : task;
            Save(restored.Record);
            if (pending)
                _queue.Writer.TryWrite(restored);
        }
        // 损坏文件已由任务存储移入隔离目录；登记告警，不能静默当作没有这个任务。
        foreach (var detail in tasks.QuarantinedPaths)
            logger.LogWarning("分析任务文件损坏，已隔离且未参与本次恢复：{Detail}", detail);
        return base.StartAsync(cancellationToken);
    }

    /// <summary>流程的最新分析记录；没有则为 null。</summary>
    public AnalysisRecord? Find(string workflowId)
    {
        lock (_gate)
            return _records.GetValueOrDefault(workflowId);
    }

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        if (!_options.AnalysisEnabled) return;
        using var active = CancellationTokenSource.CreateLinkedTokenSource(stoppingToken);
        var loops = Enumerable.Range(0, _options.MaxConcurrentAnalyses)
            .Select(_ => AnalyzeAsync(active.Token)).Append(CheckEvidenceAsync(active.Token))
            .Append(ScheduleAsync(active.Token)).ToArray();
        // 若文件读写或调度发生致命错误，一起停止其他消费者，让宿主收到错误。
        await Task.WhenAny(loops);
        await active.CancelAsync();
        await Task.WhenAll(loops);
    }

    private async Task ScheduleAsync(CancellationToken token)
    {
        var delayed = new PriorityQueue<AnalysisTask, DateTimeOffset>();
        while (!token.IsCancellationRequested)
        {
            while (_queue.Reader.TryRead(out var incoming))
            {
                if (incoming.NextAttemptAt is { } due && due > time.GetUtcNow()) delayed.Enqueue(incoming, due);
                else Dispatch(incoming);
            }
            while (delayed.TryPeek(out _, out var due) && due <= time.GetUtcNow()) Dispatch(delayed.Dequeue());

            if (!delayed.TryPeek(out _, out var earliest))
            {
                await _queue.Reader.WaitToReadAsync(token);
                continue;
            }
            var wait = earliest - time.GetUtcNow();
            if (wait <= TimeSpan.Zero) continue;
            using var alarm = new CancellationTokenSource(wait, time);
            using var wake = CancellationTokenSource.CreateLinkedTokenSource(token, alarm.Token);
            try { await _queue.Reader.WaitToReadAsync(wake.Token); }
            catch (OperationCanceledException) when (alarm.IsCancellationRequested && !token.IsCancellationRequested) { }
        }
    }

    private void Dispatch(AnalysisTask task)
    {
        var needsEvidence = task.Pair.Primary.StaticEvidence is not null || task.Pair.Secondary.StaticEvidence is not null;
        var target = needsEvidence && task.Record.StaticStatus != "confirmed" ? _pendingEvidence : _ready;
        target.Writer.TryWrite(task);
    }

    private async Task CheckEvidenceAsync(CancellationToken token)
    {
        await foreach (var task in _pendingEvidence.Reader.ReadAllAsync(token))
        {
            var decision = evidence is null ? new StaticDecision("insufficient", "静态确认服务未配置") :
                await evidence.CheckAsync(task.Pair, token);
            var captured = task.Pair.Primary.StaticEvidence ?? task.Pair.Secondary.StaticEvidence!;
            var deadline = captured.Window.CaptureUntil.AddSeconds(30);
            var now = time.GetUtcNow();
            if (decision.Status == "pending" && now >= deadline)
                decision = new("insufficient", $"静态确认超时：{decision.Reason ?? "证据未覆盖两路扫描"}");
            var terminal = decision.Status is "rejected" or "insufficient";
            var record = task.Record with
            {
                Stage = terminal ? AnalysisStage.Failed : AnalysisStage.Queued, StaticStatus = decision.Status,
                StaticReason = decision.Reason, Error = terminal ? $"静态未确认：{decision.Reason ?? decision.Status}" : null,
                UpdatedAt = time.GetLocalNow().DateTime
            };
            var retryAt = now.AddSeconds(1);
            var next = task with { Record = record, NextAttemptAt = decision.Status == "pending" ?
                (retryAt < deadline ? retryAt : deadline) : null };
            Persist(next);
            if (task.Record.StaticStatus != decision.Status || task.Record.StaticReason != decision.Reason)
                logger.LogInformation("扫描静态确认 workflowId={WorkflowId} pairId={PairId} status={Status} reason={Reason}",
                    record.WorkflowId, record.PairId, decision.Status, decision.Reason);
            if (!terminal) _queue.Writer.TryWrite(next);
        }
    }

    private async Task AnalyzeAsync(CancellationToken stoppingToken)
    {
        await foreach (var queued in _ready.Reader.ReadAllAsync(stoppingToken))
        {
            var task = queued;
            var record = task.Record with
            {
                Stage = AnalysisStage.Analyzing,
                Error = null,
                UpdatedAt = time.GetLocalNow().DateTime
            };
            task = task with { Record = record, NextAttemptAt = null };
            Persist(task);
            var startedAt = DateTime.Now;
            var stage = "plate";
            var stageTimer = Stopwatch.StartNew();
            try
            {
                var plate = await plates.FetchAsync(task.Pair.WorkflowId, stoppingToken);
                logger.LogInformation("Worker分析阶段 workflowId={WorkflowId} pairId={PairId} stage=plate elapsedMs={ElapsedMs}",
                    task.Pair.WorkflowId, task.Pair.VehicleCaptureId, stageTimer.ElapsedMilliseconds);
                stage = "analyzer-http";
                stageTimer.Restart();
                record = await analyzer.AnalyzePairAsync(record, task.Pair, plate, stoppingToken);
            }
            catch (Exception ex) when (!stoppingToken.IsCancellationRequested)
            {
                // 只处理本次调用失败；停机取消保留 analyzing，重启后继续。
                record = record with { Stage = AnalysisStage.Failed, Error = ex.Message };
            }
            logger.LogInformation("Worker分析阶段 workflowId={WorkflowId} pairId={PairId} stage={Stage} elapsedMs={ElapsedMs} result={Result}",
                task.Pair.WorkflowId, task.Pair.VehicleCaptureId, stage, stageTimer.ElapsedMilliseconds, record.Stage);

            if (record.Stage == AnalysisStage.Failed)
            {
                task = task with { FailureCount = task.FailureCount + 1 };
                if (task.FailureCount <= _options.AnalysisRetryCount)
                {
                    record = record with { Stage = AnalysisStage.Queued };
                    task = task with { NextAttemptAt = time.GetUtcNow().AddSeconds(_options.AnalysisRetryDelaySeconds) };
                    logger.LogWarning("分析失败，等待重试: workflowId={WorkflowId}, 失败次数={Failures}, 下次调用={NextAttemptAt}, error={Error}",
                        task.Pair.WorkflowId, task.FailureCount, task.NextAttemptAt, record.Error);
                }
            }
            task = task with { Record = record with { UpdatedAt = time.GetLocalNow().DateTime } };
            Persist(task);
            LogTiming(task.Pair, startedAt, record);
            if (task.Record.Stage == AnalysisStage.Queued) _queue.Writer.TryWrite(task);
        }
    }

    private void Persist(AnalysisTask task)
    {
        tasks.Save(task);
        Save(task.Record);
    }

    private AnalysisRecord Save(AnalysisRecord record)
    {
        lock (_gate)
        {
            if (_records.TryAdd(record.WorkflowId, record))
            {
                _order.Enqueue(record.WorkflowId);
                if (_order.Count > MaxRetainedWorkflows)
                    _records.Remove(_order.Dequeue());
            }
            else
            {
                _records[record.WorkflowId] = record;
            }
        }
        return record;
    }

    private void LogTiming(PairedPcdCapture pair, DateTime startedAt, AnalysisRecord record)
    {
        var now = DateTime.Now;
        var lastCapturedAt = pair.Primary.CapturedAt > pair.Secondary.CapturedAt
            ? pair.Primary.CapturedAt
            : pair.Secondary.CapturedAt;
        logger.Log(record.Stage == AnalysisStage.Completed ? LogLevel.Information : LogLevel.Warning,
            "双雷达分阶段耗时 workflowId={WorkflowId} pairId={PairId} stage={Stage} " +
            "A采集={PrimaryCapturedAt:HH:mm:ss.fff} B采集={SecondaryCapturedAt:HH:mm:ss.fff} " +
            "采集到配对={CaptureToPairMs}ms 配对到开始分析={QueueWaitMs}ms Analyzer调用={AnalyzerMs}ms " +
            "采集到分析返回={CaptureToResultMs}ms error={Error}",
            pair.WorkflowId, pair.VehicleCaptureId, record.Stage,
            pair.Primary.CapturedAt, pair.Secondary.CapturedAt,
            (long)(pair.PairedAt - lastCapturedAt).TotalMilliseconds,
            (long)(startedAt - pair.PairedAt).TotalMilliseconds,
            (long)(now - startedAt).TotalMilliseconds,
            (long)(now - lastCapturedAt).TotalMilliseconds,
            record.Error);
    }
}
