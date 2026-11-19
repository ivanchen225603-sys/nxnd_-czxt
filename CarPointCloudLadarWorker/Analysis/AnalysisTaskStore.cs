using System.Text.Json;
using System.Text.Json.Serialization;
using CarPointCloudLadarWorker.Config;
using CarPointCloudLadarWorker.Domain;
using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Analysis;

/// <summary>每个配对任务一个 JSON 文件，原子写入；未完成任务与终局使用同一种格式。</summary>
public sealed class AnalysisTaskStore
{
    private const string QuarantineDirectoryName = "_quarantine";

    private readonly string _directory;
    private readonly string _quarantineDirectory;
    private readonly List<string> _quarantined = [];

    public AnalysisTaskStore(IOptions<LadarCaptureOptions> options)
    {
        var o = options.Value;
        _directory = Path.GetFullPath(string.IsNullOrWhiteSpace(o.AnalysisTaskDirectory)
            ? Path.Combine(o.CaptureDirectory, "analysis-tasks") : o.AnalysisTaskDirectory);
        _quarantineDirectory = Path.Combine(_directory, QuarantineDirectoryName);
        Directory.CreateDirectory(_directory);
    }

    /// <summary>本次 Load 中已隔离的损坏任务说明；由宿主记录告警，不参与恢复。</summary>
    public IReadOnlyList<string> QuarantinedPaths => _quarantined;

    /// <summary>先写同目录临时文件再原子替换，避免进程中断留下截断 JSON。</summary>
    internal void Save(AnalysisTask task)
    {
        var target = Path.Combine(_directory, $"{task.Pair.VehicleCaptureId}.json");
        var temp = Path.Combine(_directory, $".{Path.GetRandomFileName()}.tmp");
        try
        {
            File.WriteAllText(temp, JsonSerializer.Serialize(task));
            File.Move(temp, target, overwrite: true);
        }
        finally
        {
            try
            {
                if (File.Exists(temp))
                    File.Delete(temp);
            }
            catch (IOException)
            {
                // 临时文件清理失败不影响本次写入结果；留待下次清理或人工处理。
            }
        }
    }

    /// <summary>
    /// 磁盘输入仅在这里校验一次；损坏文件被移入隔离目录并登记，不猜测、不修复，
    /// 也不让单个坏文件阻断其余任务的恢复。读取或权限错误原样上抛，不能据此判为损坏。
    /// </summary>
    internal IEnumerable<AnalysisTask> Load()
    {
        _quarantined.Clear();
        var loaded = new List<AnalysisTask>();
        foreach (var path in Directory.EnumerateFiles(_directory, "*.json"))
        {
            try
            {
                loaded.Add(Parse(path));
            }
            catch (Exception ex) when (ex is JsonException or InvalidDataException)
            {
                _quarantined.Add(Quarantine(path, ex.Message));
            }
        }
        return loaded;
    }

    private static AnalysisTask Parse(string path)
    {
        var task = JsonSerializer.Deserialize<AnalysisTask>(File.ReadAllText(path));
        if (task is not
            {
                Pair:
                {
                    VehicleCaptureId.Length: > 0,
                    Primary: { PcdPath.Length: > 0, WorkflowId.Length: > 0, CameraIdentifier.Length: > 0 },
                    Secondary: { PcdPath.Length: > 0, WorkflowId.Length: > 0, CameraIdentifier.Length: > 0 }
                },
                Record: { WorkflowId.Length: > 0 }
            } || task.FailureCount < 0 || !Enum.IsDefined(task.Record.Stage))
            throw new InvalidDataException($"分析任务缺少必要字段或状态无效: {path}");
        return task;
    }

    /// <summary>把损坏文件原样移出任务目录；无法移动时保留现场并登记，仍不阻断恢复。</summary>
    private string Quarantine(string path, string reason)
    {
        try
        {
            Directory.CreateDirectory(_quarantineDirectory);
            var name = Path.GetFileName(path);
            var target = Path.Combine(_quarantineDirectory, name);
            if (File.Exists(target))
                target = Path.Combine(_quarantineDirectory,
                    $"{Path.GetFileNameWithoutExtension(name)}_{Path.GetRandomFileName()}.json");
            File.Move(path, target);
            return $"{path} -> {target}（{reason}）";
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
        {
            return $"{path} 无法隔离，已跳过本次恢复，文件保持原样（{reason}；{ex.Message}）";
        }
    }
}

internal sealed record AnalysisTask(
    [property: JsonRequired] PairedPcdCapture Pair,
    [property: JsonRequired] AnalysisRecord Record,
    [property: JsonRequired] int FailureCount,
    [property: JsonRequired] DateTimeOffset? NextAttemptAt,
    bool UnavailablePublicationPending = false);
