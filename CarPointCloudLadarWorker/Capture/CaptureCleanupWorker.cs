using System.Globalization;
using CarPointCloudLadarWorker.Config;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Capture;

/// <summary>
/// 删除采集目录中过期的 PCD 及 Analyzer 附属文件。
/// 配置 <see cref="LadarCaptureOptions.CaptureCleanupDailyAt"/> 时每天该时刻删除此前写入的文件，
/// 否则按 <see cref="LadarCaptureOptions.CaptureRetentionHours"/> 定期清理；两者都未配置时不清理。
/// </summary>
public sealed class CaptureCleanupWorker(
    IOptions<LadarCaptureOptions> options,
    TimeProvider time,
    ILogger<CaptureCleanupWorker> logger) : BackgroundService
{
    private readonly LadarCaptureOptions _options = options.Value;

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        var daily = _options.CaptureCleanupDailyAt.Length > 0;
        if (!daily && _options.CaptureRetentionHours <= 0)
        {
            logger.LogInformation("采集点云清理未启用: root={Root}", _options.CaptureDirectory);
            return;
        }

        var dailyAt = daily ? TimeOnly.ParseExact(_options.CaptureCleanupDailyAt, "HH:mm", CultureInfo.InvariantCulture) : default;
        logger.LogInformation("采集点云清理已启用: root={Root} dailyAt={DailyAt} retentionHours={RetentionHours} timezone={Timezone}",
            _options.CaptureDirectory, _options.CaptureCleanupDailyAt, _options.CaptureRetentionHours, time.LocalTimeZone.Id);
        DateTime? lastDailyCutoff = null;
        using var timer = new PeriodicTimer(
            TimeSpan.FromMinutes(daily ? 1 : _options.CaptureCleanupIntervalMinutes), time);
        do
        {
            var now = time.GetLocalNow().DateTime;
            var cutoff = now.AddHours(-_options.CaptureRetentionHours);
            if (daily)
            {
                cutoff = now.Date + dailyAt.ToTimeSpan();
                if (cutoff > now)
                    cutoff = cutoff.AddDays(-1);
                if (cutoff == lastDailyCutoff)
                    continue;
                lastDailyCutoff = cutoff;
            }
            DeleteOlderThan(cutoff);
        }
        while (await timer.WaitForNextTickAsync(stoppingToken));
    }

    internal void DeleteOlderThan(DateTime cutoff)
    {
        var root = _options.CaptureDirectory;
        var runId = Guid.NewGuid().ToString("N");
        logger.LogInformation("采集点云清理开始: runId={RunId} root={Root} cutoff={Cutoff:yyyy-MM-dd HH:mm:ss}",
            runId, root, cutoff);
        if (!Directory.Exists(root))
        {
            logger.LogWarning("采集点云清理跳过: runId={RunId} root={Root} reason=directory_missing", runId, root);
            return;
        }

        var keep = _options.PermanentPointCloudDirectories
            .Select(x => Path.TrimEndingDirectorySeparator(Path.GetFullPath(x)) + Path.DirectorySeparatorChar)
            .ToArray();
        var deleted = 0;
        var artifactsDeleted = 0;
        var recent = 0;
        var protectedFiles = 0;
        var failed = 0;
        long deletedBytes = 0;
        try
        {
            foreach (var file in new DirectoryInfo(root).EnumerateFiles("*.pcd", SearchOption.AllDirectories))
            {
                if (file.Name.EndsWith("_fused_empty.pcd", StringComparison.OrdinalIgnoreCase) ||
                    keep.Any(directory => file.FullName.StartsWith(directory, StringComparison.OrdinalIgnoreCase)))
                {
                    protectedFiles++;
                    continue;
                }
                var modifiedAt = file.LastWriteTime;
                if (modifiedAt >= cutoff)
                {
                    recent++;
                    continue;
                }

                var stem = Path.GetFileNameWithoutExtension(file.Name);
                try
                {
                    var bytes = file.Length;
                    file.Delete();
                    deleted++;
                    deletedBytes += bytes;
                    logger.LogInformation("已删除过期 PCD: runId={RunId} path={Path} modifiedAt={ModifiedAt:yyyy-MM-dd HH:mm:ss} bytes={Bytes}",
                        runId, file.FullName, modifiedAt, bytes);
                    foreach (var pattern in new[] { $"{stem}_rib_coords_*.json", $"{stem}_ribs_*.png" })
                        foreach (var artifact in file.Directory!.EnumerateFiles(pattern))
                        {
                            artifact.Delete();
                            artifactsDeleted++;
                        }
                }
                catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
                {
                    failed++;
                    logger.LogWarning(ex, "PCD 或其附属文件清理失败: runId={RunId} path={Path}", runId, file.FullName);
                }
            }
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
        {
            failed++;
            logger.LogWarning(ex, "采集点云目录扫描中断: runId={RunId} root={Root}", runId, root);
        }
        logger.LogInformation("采集点云清理结果: runId={RunId} cutoff={Cutoff:yyyy-MM-dd HH:mm:ss} " +
            "deletedPcd={DeletedPcd} deletedArtifacts={DeletedArtifacts} pcdBytes={PcdBytes} " +
            "keptRecent={KeptRecent} keptProtected={KeptProtected} failures={Failures}",
            runId, cutoff, deleted, artifactsDeleted, deletedBytes, recent, protectedFiles, failed);
    }
}
