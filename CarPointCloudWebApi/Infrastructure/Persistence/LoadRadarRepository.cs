using System.Data;
using System.Diagnostics;
using System.Globalization;
using System.Text.Json;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>所有正式发布均在同一达梦连接和事务内完成，不调用旧时间邻近匹配。</summary>
public sealed partial class LoadRadarRepository : ILoadRadarRepository, ILoadRadarPublicationQueue
{
    private readonly string _connectionString;
    private readonly int _timeout;
    // 与 LoadRadarService 同一个配置项；null = 不设宽度门槛（见 LoadRadarPublicationPolicy）。
    private readonly decimal? _boundSpreadMax;
    private static readonly JsonSerializerOptions Json = new(JsonSerializerDefaults.Web);

    public LoadRadarRepository(IConfiguration configuration)
    {
        _connectionString = configuration.GetConnectionString("CarPointCloudDb") ?? "";
        _boundSpreadMax = configuration.GetValue<decimal?>("LoadRadar:VolumeBoundSpreadMax");
        if (string.IsNullOrWhiteSpace(_connectionString))
            throw new InvalidOperationException("载货雷达仓储缺少数据库连接配置");
        _timeout = Math.Clamp(configuration.GetValue("Database:CommandTimeoutSeconds", 30), 1, 300);
    }

    public LoadMeasurementRecord SaveMeasurement(LoadMeasurementRecord record)
    {
        var id = Text(record.Measurement.MeasurementId, 128);
        Text(record.PassageKey, 64);
        Text(record.PayloadHash, 64);
        // 多条测量可能同时首次建立同一 passage，唯一冲突后重新读取已提交结果。
        for (var attempt = 0; ; attempt++)
        {
            try
            {
                return Transaction((connection, transaction) =>
                {
                    var existing = ReadMeasurement(connection, transaction, id);
                    if (existing is not null) return SameMeasurement(existing, record);
                    using (var find = Command(connection, transaction,
                        "SELECT PassageKey FROM LoadRadarPassage WHERE PassageKey = ?"))
                    {
                        V(find, "key", record.PassageKey, 64);
                        if (find.ExecuteScalar() is null)
                        {
                            using var create = Command(connection, transaction,
                                "INSERT INTO LoadRadarPassage (PassageKey, NodeId, ChannelId, PassageId) VALUES (?, ?, ?, ?)");
                            V(create, "key", record.PassageKey, 64);
                            V(create, "node", record.Measurement.NodeId, 64);
                            V(create, "channel", record.Measurement.ChannelId, 64);
                            V(create, "passage", record.Measurement.PassageId, 128);
                            create.ExecuteNonQuery();
                        }
                    }
                    ValidatePassage(connection, transaction, record);
                    var started = ReadPassageStart(connection, transaction, record.PassageKey);
                    if (started is not null && (started.Start.NodeId != record.Measurement.NodeId ||
                        started.Start.ChannelId != record.Measurement.ChannelId || started.Start.PassageId != record.Measurement.PassageId ||
                        started.Start.CalibrationId != record.Measurement.CalibrationId ||
                        started.Start.ConfigurationHash != record.Measurement.ConfigurationHash ||
                        started.Start.SourceMode != record.Measurement.SourceMode || record.Measurement.MeasuredAt < started.Start.ArrivedAt))
                        throw Conflict("测量与原到车上下文不一致");
                    using var insert = Command(connection, transaction,
                        "INSERT INTO LoadRadarMeasurement (MeasurementId, PassageKey, NodeId, ChannelId, PayloadHash, PayloadJson, ReceivedAt) VALUES (?, ?, ?, ?, ?, ?, ?)");
                    V(insert, "id", id, 128);
                    V(insert, "key", record.PassageKey, 64);
                    V(insert, "node", record.Measurement.NodeId, 64);
                    V(insert, "channel", record.Measurement.ChannelId, 64);
                    V(insert, "hash", record.PayloadHash, 64);
                    J(insert, "payload", record.Measurement);
                    V(insert, "received", Stamp(record.ReceivedAt), 40);
                    insert.ExecuteNonQuery();
                    // 新测量及自动入库意图同事务；历史重放不会补建任务或批量发布旧数据。
                    if (LoadRadarPublicationPolicy.IsCandidate(record.Measurement, _boundSpreadMax))
                        EnqueuePublication(connection, transaction, id, record.ReceivedAt);
                    return ReadMeasurement(connection, transaction, id)!;
                });
            }
            catch (DmException exception) when (exception.Number == -6602)
            {
                var existing = GetMeasurement(id);
                if (existing is not null) return SameMeasurement(existing, record);
                if (attempt >= 2) throw Conflict("测量或车辆通过编号并发冲突，请以原编号重试");
            }
        }
    }

    public LoadMeasurementRecord? GetMeasurement(string id)
    {
        using var connection = Open();
        return ReadMeasurement(connection, null, Text(id, 128));
    }

    public IReadOnlyList<LoadMeasurementRecord> GetHistory(string? nodeId, string? channelId,
        string? licensePlate, string? beforeId, int take, DateTimeOffset? from = null, DateTimeOffset? to = null)
    {
        if (from > to) throw new LoadRadarException(400, "开始时间不能晚于结束时间");
        take = Math.Clamp(take, 1, 100);
        using var connection = Open();
        var filters = new List<string>();
        if (!string.IsNullOrWhiteSpace(nodeId)) filters.Add("m.NodeId = ?");
        if (!string.IsNullOrWhiteSpace(channelId)) filters.Add("m.ChannelId = ?");
        if (!string.IsNullOrWhiteSpace(licensePlate)) filters.Add("p.LicensePlate = ?");
        if (!string.IsNullOrWhiteSpace(beforeId))
            filters.Add("m.SequenceId < (SELECT SequenceId FROM LoadRadarMeasurement WHERE MeasurementId = ?)");
        var where = filters.Count == 0 ? "" : " WHERE " + string.Join(" AND ", filters);
        var byTime = from is not null || to is not null;
        // 旧表只有不可变载荷中的测量时间；不能拿接收时间冒充，也不能只筛前 100 条。
        // 有时间条件时逐行读取原时间，够一页即停止；不迁移或改写历史载荷。
        var columns = byTime ? "m.MeasurementId, m.PayloadJson" : $"TOP {take} m.MeasurementId";
        using var command = Command(connection, null,
            $"SELECT {columns} FROM LoadRadarMeasurement m JOIN LoadRadarPassage p ON p.PassageKey = m.PassageKey{where} ORDER BY m.SequenceId DESC");
        if (!string.IsNullOrWhiteSpace(nodeId)) V(command, "node", nodeId, 64);
        if (!string.IsNullOrWhiteSpace(channelId)) V(command, "channel", channelId, 64);
        if (!string.IsNullOrWhiteSpace(licensePlate)) V(command, "plate", licensePlate, 32);
        if (!string.IsNullOrWhiteSpace(beforeId)) V(command, "before", beforeId, 128);
        var ids = new List<string>();
        var elapsed = Stopwatch.StartNew();
        int RemainingSeconds()
        {
            var remaining = _timeout - elapsed.Elapsed.TotalSeconds;
            if (remaining <= 0) throw new LoadRadarException(503, "历史查询超时，请缩小车牌或通道范围后重试");
            return Math.Max(1, (int)Math.Ceiling(remaining));
        }
        using (var reader = command.ExecuteReader())
            while (ids.Count < take && reader.Read())
            {
                RemainingSeconds();
                if (!byTime || WithinMeasuredTime(reader.GetString(1), from, to)) ids.Add(reader.GetString(0));
            }
        RemainingSeconds();
        var rows = new List<LoadMeasurementRecord>();
        foreach (var id in ids)
        {
            RemainingSeconds();
            rows.Add(ReadMeasurement(connection, null, id, RemainingSeconds)!);
            RemainingSeconds();
        }
        return rows;
    }

    private static bool WithinMeasuredTime(string payload, DateTimeOffset? from, DateTimeOffset? to)
    {
        using var document = JsonDocument.Parse(payload);
        // 缺失或损坏数据报错，不能静默跳过并声称历史完整。
        var measuredAt = document.RootElement.GetProperty("measuredAt").GetDateTimeOffset();
        return (from is null || measuredAt >= from) && (to is null || measuredAt <= to);
    }

    public LoadPassageBinding BindPassage(LoadMeasurementRecord measurement, LoadPassageBinding binding)
    {
        try
        {
            return Transaction((connection, transaction) =>
            {
                var authoritative = LockWorkflow(connection, transaction, binding.WorkflowId);
                LockPassage(connection, transaction, measurement.PassageKey);
                var persisted = RequireMeasurement(connection, transaction, measurement);
                if (binding.PassageKey != persisted.PassageKey ||
                    authoritative.Plate != binding.LicensePlate ||
                    !string.Equals(authoritative.Camera, binding.CameraIdentifier, StringComparison.OrdinalIgnoreCase))
                    throw Conflict("流程的权威车牌或点位与本次绑定不一致");
                if (persisted.Binding is { } existing)
                {
                    if (SameBinding(existing, binding)) return existing;
                    throw Conflict("该车辆通过已绑定另一流程或挂车身份，不能覆盖");
                }
                using (var occupied = Command(connection, transaction,
                    "SELECT PassageKey FROM LoadRadarPassage WHERE WorkflowId = ?"))
                {
                    V(occupied, "workflow", binding.WorkflowId, 128);
                    if (occupied.ExecuteScalar() is not null)
                        throw Conflict("该流程已绑定另一车辆通过");
                }
                using var update = Command(connection, transaction,
                    "UPDATE LoadRadarPassage SET WorkflowId = ?, LicensePlate = ?, BindingJson = ? WHERE PassageKey = ? AND WorkflowId IS NULL");
                V(update, "workflow", binding.WorkflowId, 128);
                V(update, "plate", authoritative.Plate, 32);
                J(update, "binding", binding);
                V(update, "key", binding.PassageKey, 64);
                if (update.ExecuteNonQuery() != 1) throw Conflict("车辆通过绑定已被其它请求修改");
                Audit(connection, transaction, "bind", binding.PassageKey, binding.Actor, binding.Reason, binding);
                return binding;
            });
        }
        catch (DmException exception) when (exception.Number == -6602)
        {
            var existing = GetMeasurement(measurement.Measurement.MeasurementId)?.Binding;
            if (existing is not null && SameBinding(existing, binding)) return existing;
            throw Conflict("车辆通过或 workflow 已被其它关联占用");
        }
    }

    public LoadBaselineVersion CreateBaseline(LoadBaselineVersion version)
    {
        for (var attempt = 0; ; attempt++)
        {
            try
            {
                return Transaction((connection, transaction) =>
                {
                    var existing = ReadBaseline(connection, transaction, version.VersionId);
                    if (existing is not null)
                    {
                        if (existing.ScopeKey == version.ScopeKey && existing.NodeId == version.NodeId &&
                            existing.ChannelId == version.ChannelId && existing.CalibrationId == version.CalibrationId &&
                            existing.VehicleIdentity == version.VehicleIdentity && existing.LicensePlate == version.LicensePlate &&
                            existing.MeasurementId == version.MeasurementId && existing.AssetSha256 == version.AssetSha256)
                            return existing;
                        throw Conflict("同一基线版本编号对应不同内容");
                    }
                    var measurement = ReadMeasurement(connection, transaction, version.MeasurementId)
                        ?? throw new LoadRadarException(404, "空车测量不存在");
                    ValidateBaselineSource(version, measurement);
                    using (var find = Command(connection, transaction,
                        "SELECT ScopeKey FROM LoadRadarBaselineHead WHERE ScopeKey = ?"))
                    {
                        V(find, "scope", version.ScopeKey, 64);
                        if (find.ExecuteScalar() is null)
                        {
                            using var head = Command(connection, transaction,
                                "INSERT INTO LoadRadarBaselineHead (ScopeKey, ActiveVersionId) VALUES (?, NULL)");
                            V(head, "scope", version.ScopeKey, 64);
                            head.ExecuteNonQuery();
                        }
                    }
                    using var insert = Command(connection, transaction,
                        "INSERT INTO LoadRadarBaseline (VersionId, ScopeKey, MeasurementId, VersionJson, ActivatedAt) VALUES (?, ?, ?, ?, NULL)");
                    V(insert, "id", version.VersionId, 128);
                    V(insert, "scope", version.ScopeKey, 64);
                    V(insert, "measurement", version.MeasurementId, 128);
                    J(insert, "version", version with { Active = false, ActivatedAt = null });
                    insert.ExecuteNonQuery();
                    Audit(connection, transaction, "baseline-create", version.VersionId, version.Actor, version.Reason, version);
                    return version with { Active = false, ActivatedAt = null };
                });
            }
            catch (DmException exception) when (exception.Number == -6602)
            {
                if (attempt >= 2) throw Conflict("基线版本或范围并发冲突，请用原编号重试");
            }
        }
    }

    public LoadBaselineVersion? GetBaseline(string id)
    {
        using var connection = Open();
        return ReadBaseline(connection, null, Text(id, 128));
    }

    public IReadOnlyList<LoadBaselineVersion> GetBaselines()
    {
        using var connection = Open();
        using var command = Command(connection, null,
            "SELECT VersionId FROM LoadRadarBaseline ORDER BY SequenceId DESC");
        var ids = new List<string>();
        using (var reader = command.ExecuteReader())
            while (reader.Read()) ids.Add(reader.GetString(0));
        return ids.Select(id => ReadBaseline(connection, null, id)!).ToArray();
    }

    public LoadBaselineVersion ActivateBaseline(string id, string? expectedVersionId, string actor, string reason)
        => ActivateBaselineCore(id, expectedVersionId, actor, reason, false);

    public LoadBaselineVersion ActivateAutomaticBaseline(string id, string actor, string reason)
        => ActivateBaselineCore(id, null, actor, reason, true);

    private LoadBaselineVersion ActivateBaselineCore(string id, string? expectedVersionId, string actor, string reason, bool automatic)
    {
        return Transaction((connection, transaction) =>
        {
            var version = ReadBaseline(connection, transaction, Text(id, 128))
                ?? throw new LoadRadarException(404, "基线版本不存在");
            using (var acquire = Command(connection, transaction,
                "UPDATE LoadRadarBaselineHead SET ScopeKey = ScopeKey WHERE ScopeKey = ?"))
            {
                V(acquire, "scope", version.ScopeKey, 64);
                if (acquire.ExecuteNonQuery() != 1) throw Conflict("基线范围不存在");
            }
            var current = ReadPlateBaselineAt(connection, transaction, version.NodeId, version.LicensePlate, DateTimeOffset.UtcNow)?.VersionId;
            if (!automatic && !string.Equals(current, expectedVersionId, StringComparison.Ordinal))
                throw Conflict("当前激活版本已变化，请刷新后重新确认");
            if (current == id) return ReadBaseline(connection, transaction, id)!;
            var measurement = ReadMeasurement(connection, transaction, version.MeasurementId)
                ?? throw Conflict("基线原始测量不存在");
            ValidateBaselineSource(version, measurement);
            var currentVersion = current is null ? null : ReadBaseline(connection, transaction, current);
            var currentSource = currentVersion is null ? null : ReadMeasurement(connection, transaction, currentVersion.MeasurementId);
            var moveHead = !automatic || currentSource is null || currentSource.Measurement.MeasuredAt <= measurement.Measurement.MeasuredAt;
            // 重启后迟到的昨天空车只补昨天复算，不把今天已经生效的基线回退。
            if (moveHead)
            {
                // 同牌升级后只有一个活动头；保留旧版本和激活审计，供原任务按时刻恢复。
                using (var clear = Command(connection, transaction,
                    @"UPDATE LoadRadarBaselineHead SET ActiveVersionId=NULL WHERE ActiveVersionId IN
                      (SELECT b.VersionId FROM LoadRadarBaseline b
                       JOIN LoadRadarMeasurement m ON m.MeasurementId=b.MeasurementId
                       JOIN LoadRadarPassage p ON p.PassageKey=m.PassageKey
                       WHERE m.NodeId=? AND m.ChannelId='CH4' AND p.LicensePlate=?)"))
                {
                    V(clear, "node", version.NodeId, 64);
                    V(clear, "plate", version.LicensePlate, 32);
                    clear.ExecuteNonQuery();
                }
                using (var update = Command(connection, transaction,
                    "UPDATE LoadRadarBaselineHead SET ActiveVersionId = ? WHERE ScopeKey = ?"))
                {
                    V(update, "id", id, 128);
                    V(update, "scope", version.ScopeKey, 64);
                    update.ExecuteNonQuery();
                }
            }
            var effectiveAt = DateTimeOffset.UtcNow;
            using (var stamp = Command(connection, transaction,
                "UPDATE LoadRadarBaseline SET ActivatedAt = ? WHERE VersionId = ? AND ActivatedAt IS NULL"))
            {
                V(stamp, "at", Stamp(effectiveAt), 40);
                V(stamp, "id", id, 128);
                stamp.ExecuteNonQuery();
            }
            if (moveHead)
                Audit(connection, transaction, "baseline-activate", id, actor, reason,
                    new { previousVersionId = current, versionId = id }, effectiveAt);
            Audit(connection, transaction, "baseline-recalculate", id, actor, reason,
                new
                {
                    previousVersionId = current,
                    versionId = id,
                    automatic,
                    source = measurement.Measurement with { Points = [] }
                }, effectiveAt);
            return ReadBaseline(connection, transaction, id)!;
        });
    }

    public LoadBaselineVersion? GetPlateBaselineAt(string nodeId, string licensePlate, DateTimeOffset measuredAt)
    {
        using var connection = Open();
        return ReadPlateBaselineAt(connection, null, nodeId, licensePlate, measuredAt);
    }

    private LoadBaselineVersion? ReadPlateBaselineAt(DmConnection connection, DmTransaction? transaction,
        string nodeId, string licensePlate, DateTimeOffset measuredAt)
    {
        // 按激活事件顺序兼容旧标定作用域；重新激活旧版也以实际事件时间生效。
        using var command = Command(connection, transaction,
            @"SELECT TOP 1 b.VersionId FROM LoadRadarAudit a
              JOIN LoadRadarBaseline b ON b.VersionId=a.EntityId
              JOIN LoadRadarMeasurement m ON m.MeasurementId=b.MeasurementId
              JOIN LoadRadarPassage p ON p.PassageKey=m.PassageKey
              WHERE a.ActionName='baseline-activate' AND m.NodeId=? AND m.ChannelId='CH4' AND p.LicensePlate=? AND a.RecordedAt<=?
              ORDER BY a.RecordedAt DESC, a.Id DESC");
        V(command, "node", nodeId, 64);
        V(command, "plate", licensePlate, 32);
        V(command, "at", Stamp(measuredAt), 40);
        var id = command.ExecuteScalar() as string;
        return id is null ? null : ReadBaseline(connection, transaction, id);
    }

    private LoadBaselineVersion? ReadBaselineAt(DmConnection connection, DmTransaction? transaction,
        string scopeKey, DateTimeOffset measuredAt)
    {
        // 激活审计与head变更同事务提交，保留切换和重新激活的每一次事件。
        // 缺历史凭据时不将现在的head倒套给过去的观测。
        using var command = Command(connection, transaction,
            @"SELECT TOP 1 b.VersionId FROM LoadRadarAudit a
              JOIN LoadRadarBaseline b ON b.VersionId=a.EntityId
              WHERE a.ActionName='baseline-activate' AND b.ScopeKey=? AND a.RecordedAt<=?
              ORDER BY a.RecordedAt DESC, a.Id DESC");
        V(command, "scope", scopeKey, 64);
        V(command, "at", Stamp(measuredAt), 40);
        var id = command.ExecuteScalar() as string;
        return id is null ? null : ReadBaseline(connection, transaction, id);
    }

    public LoadCoalPublication Publish(LoadMeasurementRecord measurement, CoalVolumeRecord coal,
        string actor, string reason)
    {
        try
        {
            return Transaction((connection, transaction) =>
            {
                var initial = RequireMeasurement(connection, transaction, measurement);
                var binding = initial.Binding ?? throw Conflict("测量尚未绑定车辆流程");
                var workflow = LockWorkflow(connection, transaction, binding.WorkflowId);
                LockPassage(connection, transaction, initial.PassageKey);
                var stored = RequireMeasurement(connection, transaction, measurement);
                if (stored.Binding is null || !SameBinding(stored.Binding, binding) ||
                    workflow.Plate != binding.LicensePlate ||
                    !string.Equals(workflow.Camera, binding.CameraIdentifier, StringComparison.OrdinalIgnoreCase))
                    throw Conflict("发布时流程绑定、车牌或点位已变化");
                var prior = ReadPublication(connection, transaction, stored.PassageKey, binding.WorkflowId);
                if (prior is not null) return SamePublication(prior, stored, binding);
                var input = stored.Measurement;
                // 与入队、与 LoadRadarService.Publish 同一条规则，不再各写一份数字。
                if (!LoadRadarPublicationPolicy.IsCandidate(input, _boundSpreadMax))
                    throw Conflict("测量不满足正式煤量发布条件");
                var baseline = ReadBaseline(connection, transaction, input.BaselineVersionId!)
                    ?? throw Conflict("体积所用基线版本不存在");
                if (ReadPlateBaselineAt(connection, transaction, input.NodeId, binding.LicensePlate, input.MeasuredAt)?.VersionId != baseline.VersionId)
                    throw Conflict("体积未使用原测量时刻激活的基线版本");
                var baselineSource = ReadMeasurement(connection, transaction, baseline.MeasurementId)
                    ?? throw Conflict("正式基线来源测量不存在");
                if (!LoadRadarPublicationPolicy.MatchesBaseline(input, binding, baseline, baselineSource.Measurement))
                    throw Conflict("体积基线未确认激活，或车辆、CH4来源及配准引用不匹配");
                using (var legacy = Command(connection, transaction,
                    "SELECT Id FROM CoalVolumeRecord WHERE WorkflowId = ?"))
                {
                    V(legacy, "workflow", binding.WorkflowId, 128);
                    if (legacy.ExecuteScalar() is not null)
                        throw Conflict("该 workflow 已有煤量记录，禁止旧来源与载货雷达重复发布");
                }
                if (coal.WorkflowId != binding.WorkflowId || coal.LicensePlate != workflow.Plate ||
                    coal.CameraIdentifier != binding.CameraIdentifier || coal.Unit is not ("m3" or "m³") ||
                    coal.Status != "success" || coal.Volume != input.VolumeM3!.Value.RoundCoalVolume())
                    throw Conflict("煤量载荷与持久测量或权威流程不一致");
                var coalId = InsertCoal(connection, transaction, coal);
                var result = new LoadCoalPublication(stored.PassageKey, binding.WorkflowId,
                    input.MeasurementId, coalId, actor, reason, DateTimeOffset.UtcNow);
                using var insert = Command(connection, transaction,
                    "INSERT INTO LoadRadarPublication (PassageKey, WorkflowId, MeasurementId, CoalVolumeId, PublicationJson) VALUES (?, ?, ?, ?, ?)");
                V(insert, "key", result.PassageKey, 64);
                V(insert, "workflow", result.WorkflowId, 128);
                V(insert, "measurement", result.MeasurementId, 128);
                insert.Parameters.Add("coal", DmDbType.Int64).Value = coalId;
                J(insert, "publication", result);
                insert.ExecuteNonQuery();
                Audit(connection, transaction, "publish", result.PassageKey, actor, reason, result);
                return result;
            });
        }
        catch (DmException exception) when (exception.Number == -6602)
        {
            var stored = GetMeasurement(measurement.Measurement.MeasurementId);
            if (stored?.Binding is { } binding && stored.Publication is { } publication)
                return SamePublication(publication, stored, binding);
            throw Conflict("车辆通过、workflow 或煤量幂等键已被占用");
        }
    }

    private LoadMeasurementRecord? ReadMeasurement(DmConnection connection, DmTransaction? transaction, string id,
        Func<int>? remainingSeconds = null)
    {
        using var command = Command(connection, transaction,
            "SELECT m.PassageKey, m.PayloadHash, m.PayloadJson, m.ReceivedAt, p.BindingJson FROM LoadRadarMeasurement m JOIN LoadRadarPassage p ON p.PassageKey=m.PassageKey WHERE m.MeasurementId = ?");
        V(command, "id", id, 128);
        if (remainingSeconds is not null) command.CommandTimeout = remainingSeconds();
        LoadMeasurementRecord result;
        using (var reader = command.ExecuteReader())
        {
            if (!reader.Read()) return null;
            result = new(reader.GetString(0), reader.GetString(1),
                Decode<LoadMeasurementInput>(reader.GetString(2)), ParseStamp(reader.GetString(3)),
                reader.IsDBNull(4) ? null : Decode<LoadPassageBinding>(reader.GetString(4)));
        }
        using (var revision = Command(connection, transaction,
            "SELECT TOP 1 RevisionJson FROM LoadRadarVolumeRevision WHERE MeasurementId=? ORDER BY ActivationId DESC"))
        {
            V(revision, "id", id, 128);
            if (revision.ExecuteScalar() is string json) result = result with { LatestRevision = Decode<LoadVolumeRevision>(json) };
        }
        return result with { Publication = ReadPublication(connection, transaction, result.PassageKey, result.Binding?.WorkflowId, remainingSeconds) };
    }

    private LoadMeasurementRecord RequireMeasurement(DmConnection connection, DmTransaction transaction,
        LoadMeasurementRecord expected)
    {
        var stored = ReadMeasurement(connection, transaction, expected.Measurement.MeasurementId)
            ?? throw new LoadRadarException(404, "测量不存在");
        return SameMeasurement(stored, expected);
    }

    private static LoadMeasurementRecord SameMeasurement(LoadMeasurementRecord existing, LoadMeasurementRecord incoming)
    {
        if (existing.PassageKey != incoming.PassageKey || existing.PayloadHash != incoming.PayloadHash ||
            JsonSerializer.Serialize(existing.Measurement, Json) != JsonSerializer.Serialize(incoming.Measurement, Json))
            throw Conflict("相同 measurementId 已用于不同载荷");
        return existing;
    }

    private static bool SameBinding(LoadPassageBinding first, LoadPassageBinding second) =>
        first.PassageKey == second.PassageKey && first.WorkflowId == second.WorkflowId &&
        first.LicensePlate == second.LicensePlate && first.CameraIdentifier == second.CameraIdentifier &&
        first.VehicleIdentity == second.VehicleIdentity;

    private void ValidatePassage(DmConnection connection, DmTransaction transaction, LoadMeasurementRecord record)
    {
        using var command = Command(connection, transaction,
            "SELECT NodeId, ChannelId, PassageId FROM LoadRadarPassage WHERE PassageKey = ?");
        V(command, "key", record.PassageKey, 64);
        using var reader = command.ExecuteReader();
        if (!reader.Read() || reader.GetString(0) != record.Measurement.NodeId ||
            reader.GetString(1) != record.Measurement.ChannelId || reader.GetString(2) != record.Measurement.PassageId)
            throw Conflict("PassageKey 已用于不同节点、车道或车辆通过");
    }

    private void LockPassage(DmConnection connection, DmTransaction transaction, string key)
    {
        using var command = Command(connection, transaction,
            "UPDATE LoadRadarPassage SET PassageKey=PassageKey WHERE PassageKey = ?");
        V(command, "key", key, 64);
        if (command.ExecuteNonQuery() != 1) throw new LoadRadarException(404, "车辆通过不存在");
    }

    private (string Plate, string Camera) LockWorkflow(DmConnection connection, DmTransaction transaction, string id)
    {
        using (var command = Command(connection, transaction,
            "UPDATE VehicleArrival SET UpdatedAt=UpdatedAt WHERE SessionId = ?"))
        {
            V(command, "id", id, 128);
            if (command.ExecuteNonQuery() != 1) throw new LoadRadarException(404, "车辆流程不存在");
        }
        using var select = Command(connection, transaction,
            "SELECT LicensePlate, PlateStatus, CameraIdentifier, PlateSource FROM VehicleArrival WHERE SessionId = ?");
        V(select, "id", id, 128);
        using var reader = select.ExecuteReader();
        // 车牌来源（含 MOCK）不再判断：真假由 /api/v1/radar/licensePlate 的取牌来源负责（2026-09-28 现场定）。
        if (!reader.Read() || reader.IsDBNull(0) || reader.GetString(1) != "SUCCEEDED" ||
            !ManualLicensePlate.TryNormalize(reader.GetString(0), out var plate, out _))
            throw Conflict("车辆流程尚无确认成功的车牌");
        return (plate, reader.GetString(2));
    }

    private LoadBaselineVersion? ReadBaseline(DmConnection connection, DmTransaction? transaction, string id)
    {
        using var command = Command(connection, transaction,
            "SELECT b.VersionJson, h.ActiveVersionId, b.ActivatedAt FROM LoadRadarBaseline b JOIN LoadRadarBaselineHead h ON h.ScopeKey=b.ScopeKey WHERE b.VersionId = ?");
        V(command, "id", id, 128);
        using var reader = command.ExecuteReader();
        if (!reader.Read()) return null;
        return Decode<LoadBaselineVersion>(reader.GetString(0)) with
        {
            Active = !reader.IsDBNull(1) && reader.GetString(1) == id,
            ActivatedAt = reader.IsDBNull(2) ? null : ParseStamp(reader.GetString(2))
        };
    }

    private static void ValidateBaselineSource(LoadBaselineVersion version, LoadMeasurementRecord source)
    {
        var input = source.Measurement;
        if (input.ChannelId != "CH4" || input.SourceMode != "live" || input.Verdict != "empty" || input.FrameDegraded ||
            source.Binding is null || source.Binding.VehicleIdentity != version.VehicleIdentity ||
            source.Binding.LicensePlate != version.LicensePlate || input.NodeId != version.NodeId ||
            input.ChannelId != version.ChannelId || input.CalibrationId != version.CalibrationId ||
            version.AssetSha256 != input.BaselineCandidateSha256)
            throw Conflict("基线必须来自已绑定、真实完整空车测量，且身份及标定一致");
    }

    private LoadCoalPublication? ReadPublication(DmConnection connection, DmTransaction? transaction,
        string key, string? workflowId, Func<int>? remainingSeconds = null)
    {
        using var command = Command(connection, transaction,
            "SELECT PublicationJson FROM LoadRadarPublication WHERE PassageKey = ? OR WorkflowId = ?");
        V(command, "key", key, 64);
        V(command, "workflow", workflowId, 128, optional: true);
        if (remainingSeconds is not null) command.CommandTimeout = remainingSeconds();
        using var reader = command.ExecuteReader();
        if (!reader.Read()) return null;
        var publication = Decode<LoadCoalPublication>(reader.GetString(0));
        if (reader.Read() || publication.PassageKey != key ||
            (workflowId is not null && publication.WorkflowId != workflowId))
            throw Conflict("车辆通过与 workflow 对应不同发布槽位");
        return publication;
    }

    private static LoadCoalPublication SamePublication(LoadCoalPublication existing,
        LoadMeasurementRecord measurement, LoadPassageBinding binding)
    {
        if (existing.PassageKey != measurement.PassageKey || existing.WorkflowId != binding.WorkflowId ||
            existing.MeasurementId != measurement.Measurement.MeasurementId)
            throw Conflict("该车辆通过或 workflow 已发布其它测量，禁止替换或重复计量");
        return existing;
    }

    private long InsertCoal(DmConnection connection, DmTransaction transaction, CoalVolumeRecord record)
    {
        // Publish 已锁定并核实权威 workflow；只有这次成功抓拍能提供对外业务身份。
        using (var plate = Command(connection, transaction,
            @"SELECT TOP 1 Id FROM LicensePlateRecord
              WHERE WorkflowId = ? AND CameraIdentifier = ? AND LicensePlate = ? AND IsSuccess = 1"))
        {
            V(plate, "workflow", record.WorkflowId, 128);
            V(plate, "camera", record.CameraIdentifier, 64);
            V(plate, "license", record.LicensePlate, 32);
            var id = plate.ExecuteScalar();
            record.LicensePlateRecordId = id is null or DBNull ? null : Convert.ToInt64(id);
        }
        using var command = Command(connection, transaction,
            @"INSERT INTO CoalVolumeRecord
              (IdempotencyKey, BusinessKey, WorkflowId, PayloadHash, LicensePlate, CameraIdentifier,
               Volume, Unit, MeasureTime, LicensePlateRecordId, Status, Confidence, SourceFile, BaselineFile, CreatedAt,
               MeasurementStartedAt, MeasurementEndedAt)
              VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?); SELECT SCOPE_IDENTITY()");
        V(command, "idempotency", record.IdempotencyKey, 64);
        V(command, "business", record.BusinessKey, 64);
        V(command, "workflow", record.WorkflowId, 128);
        V(command, "hash", record.PayloadHash, 64);
        V(command, "plate", record.LicensePlate, 32);
        V(command, "camera", record.CameraIdentifier, 64);
        Decimal(command, "volume", record.Volume, 10, 1);
        V(command, "unit", record.Unit, 16);
        command.Parameters.Add("time", DmDbType.DateTime).Value = record.MeasureTime;
        command.Parameters.Add("plateId", DmDbType.Int64).Value = (object?)record.LicensePlateRecordId ?? DBNull.Value;
        V(command, "status", record.Status, 32);
        Decimal(command, "confidence", record.Confidence, 5, 2);
        V(command, "source", record.SourceFile, 512, optional: true);
        V(command, "baseline", record.BaselineFile, 512, optional: true);
        command.Parameters.Add("created", DmDbType.DateTime).Value = record.CreatedAt;
        command.Parameters.Add("started", DmDbType.DateTime).Value = (object?)record.MeasurementStartedAt ?? DBNull.Value;
        command.Parameters.Add("ended", DmDbType.DateTime).Value = (object?)record.MeasurementEndedAt ?? DBNull.Value;
        return Convert.ToInt64(command.ExecuteScalar());
    }

    private void Audit(DmConnection connection, DmTransaction transaction, string action, string entity,
        string actor, string reason, object payload, DateTimeOffset? recordedAt = null)
    {
        using var command = Command(connection, transaction,
            "INSERT INTO LoadRadarAudit (ActionName, EntityId, Actor, Reason, RecordedAt, PayloadJson) VALUES (?, ?, ?, ?, ?, ?)");
        V(command, "action", action, 32);
        V(command, "entity", entity, 128);
        V(command, "actor", actor, 128);
        V(command, "reason", reason, 1024);
        V(command, "at", Stamp(recordedAt ?? DateTimeOffset.UtcNow), 40);
        J(command, "payload", payload);
        command.ExecuteNonQuery();
    }

    private T Transaction<T>(Func<DmConnection, DmTransaction, T> operation)
    {
        using var connection = Open();
        using var transaction = (DmTransaction)connection.BeginTransaction();
        try
        {
            var value = operation(connection, transaction);
            transaction.Commit();
            return value;
        }
        catch
        {
            try { transaction.Rollback(); } catch { /* 保留原始失败，不能伪造提交成功。 */ }
            throw;
        }
    }

    private DmConnection Open()
    {
        var connection = new DmConnection(_connectionString);
        try { connection.Open(); return connection; }
        catch { connection.Dispose(); throw; }
    }

    private DmCommand Command(DmConnection connection, DmTransaction? transaction, string sql) =>
        new(sql, connection, transaction) { CommandTimeout = _timeout };

    private static string Text(string? value, int limit)
    {
        if (string.IsNullOrWhiteSpace(value) || value.Length > limit || value.Any(char.IsControl))
            throw new LoadRadarException(400, $"持久化字段为空、超长或含控制字符（上限 {limit}）");
        return value;
    }

    private static void V(DmCommand command, string name, string? value, int size, bool optional = false) =>
        command.Parameters.Add(name, DmDbType.VarChar, size).Value =
            optional && value is null ? DBNull.Value : Text(value, size);

    private static void J(DmCommand command, string name, object value)
    {
        // 原始测量可携带万级点云，必须显式使用 CLOB，不能依赖字符串参数推断 VARCHAR。
        var parameter = command.Parameters.Add(name, DmDbType.Clob);
        parameter.Value = JsonSerializer.Serialize(value, Json);
    }

    private static void Decimal(DmCommand command, string name, decimal? value, byte precision, byte scale)
    {
        var parameter = command.Parameters.Add(name, DmDbType.Decimal);
        parameter.Value = (object?)value ?? DBNull.Value;
        parameter.Precision = precision;
        parameter.Scale = scale;
    }

    private static T Decode<T>(string value) => JsonSerializer.Deserialize<T>(value, Json)
        ?? throw new InvalidOperationException("载货雷达数据库 JSON 内容不完整");
    private static string Stamp(DateTimeOffset value) => value.ToUniversalTime().ToString("O", CultureInfo.InvariantCulture);
    private static DateTimeOffset ParseStamp(string value) => DateTimeOffset.Parse(value, CultureInfo.InvariantCulture, DateTimeStyles.RoundtripKind);
    private static LoadRadarException Conflict(string message) => new(409, message);
}
