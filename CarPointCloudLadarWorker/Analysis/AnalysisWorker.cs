using System.Collections.Concurrent;
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
    ILogger<AnalysisWorker> logger) : BackgroundService
{
    private const int MaxRetainedWorkflows = 200;

    private readonly LadarCaptureOptions _options = options.Value;
    private readonly Channel<AnalysisTask> _queue = Channel.CreateUnbounded<AnalysisTask>();
    private readonly ConcurrentDictionary<string, AnalysisTask> _unavailable = new();
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
            [pair.Primary.PcdPath, pair.Secondary.PcdPath]);
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
            else if (task.UnavailablePublicationPending)
                _unavailable[task.Record.PairId] = task;
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
        if (!_options.AnalysisEnabled)
            return;

        using var lifetime = CancellationTokenSource.CreateLinkedTokenSource(stoppingToken);
        var analysis = RunAnalysisAsync(lifetime.Token);
        var publication = PublishUnavailableAsync(lifetime.Token);
        await Task.WhenAny(analysis, publication);
        await lifetime.CancelAsync();
        await Task.WhenAll(analysis, publication);
    }

    private async Task RunAnalysisAsync(CancellationToken stoppingToken)
    {
        await foreach (var queued in _queue.Reader.ReadAllAsync(stoppingToken))
        {
            var task = queued;
            do
            {
                if (task.NextAttemptAt is { } next)
                {
                    var delay = next - time.GetUtcNow();
                    if (delay > TimeSpan.Zero)
                        await Task.Delay(delay, time, stoppingToken);
                }
                stoppingToken.ThrowIfCancellationRequested();

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
                task = task with
                {
                    Record = record with { UpdatedAt = time.GetLocalNow().DateTime },
                    UnavailablePublicationPending = record.Stage == AnalysisStage.Failed ||
                        record.Stage == AnalysisStage.Completed && !record.HasDimensionPayload
                };
                Persist(task);
                if (task.UnavailablePublicationPending)
                    _unavailable[record.PairId] = task;
                LogTiming(task.Pair, startedAt, record);
            }
            while (task.Record.Stage == AnalysisStage.Queued);
        }
    }

    private async Task PublishUnavailableAsync(CancellationToken stoppingToken)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromSeconds(2), time);
        do
        {
            foreach (var task in _unavailable.Values)
            {
                stoppingToken.ThrowIfCancellationRequested();
                try
                {
                    if (!await analyzer.ReportUnavailableAsync(task.Record, stoppingToken))
                        continue;
                    tasks.Save(task with { UnavailablePublicationPending = false });
                    _unavailable.TryRemove(task.Record.PairId, out _);
                }
                catch (Exception ex) when (!stoppingToken.IsCancellationRequested)
                {
                    logger.LogWarning(ex, "尺寸缺测状态待上报：workflowId={WorkflowId}", task.Record.WorkflowId);
                }
            }
        }
        while (await timer.WaitForNextTickAsync(stoppingToken));
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
