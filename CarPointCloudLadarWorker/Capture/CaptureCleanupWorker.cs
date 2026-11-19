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
            return;

        var dailyAt = daily ? TimeOnly.ParseExact(_options.CaptureCleanupDailyAt, "HH:mm", CultureInfo.InvariantCulture) : default;
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
        if (!Directory.Exists(_options.CaptureDirectory))
            return;

        var keep = _options.PermanentPointCloudDirectories
            .Select(x => Path.TrimEndingDirectorySeparator(Path.GetFullPath(x)) + Path.DirectorySeparatorChar)
            .ToArray();
        var deleted = 0;
        foreach (var file in new DirectoryInfo(_options.CaptureDirectory).EnumerateFiles("*.pcd", SearchOption.AllDirectories))
        {
            if (file.LastWriteTime >= cutoff ||
                file.Name.EndsWith("_fused_empty.pcd", StringComparison.OrdinalIgnoreCase) ||
                keep.Any(root => file.FullName.StartsWith(root, StringComparison.OrdinalIgnoreCase)))
                continue;

            var stem = Path.GetFileNameWithoutExtension(file.Name);
            try
            {
                file.Delete();
                foreach (var pattern in new[] { $"{stem}_rib_coords_*.json", $"{stem}_ribs_*.png" })
                    foreach (var artifact in file.Directory!.EnumerateFiles(pattern))
                        artifact.Delete();
                deleted++;
            }
            catch (IOException ex)
            {
                // 文件正被其他进程占用时跳过，下次清理再删；不能让一个文件拖停整个 Worker。
                logger.LogWarning(ex, "删除过期 PCD 失败: {Path}", file.FullName);
            }
        }
        if (deleted > 0)
            logger.LogInformation("过期 PCD 清理完成: 已删除={Deleted}, 截止时间={Cutoff}", deleted, cutoff);
    }
}
