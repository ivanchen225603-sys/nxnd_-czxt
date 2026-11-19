using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public sealed partial class LoadRadarRepository
{
    // 激活审计和原测量本身就是持久任务来源，晚绑定车牌及重启不依赖内存通知。
    public IReadOnlyList<LoadRecalculationTarget> GetRecalculations(string nodeId, string channelId, string? afterId, DateOnly fromDate)
    {
        using var connection = Open();
        return RecalculationTargets(connection, null, nodeId, channelId, afterId, fromDate: fromDate);
    }

    private IReadOnlyList<LoadRecalculationTarget> RecalculationTargets(DmConnection connection,
        DmTransaction? transaction, string nodeId, string channelId, string? afterId = null, string? measurementId = null,
        DateOnly? fromDate = null)
    {
        var since = new DateTimeOffset((fromDate ?? DateOnly.FromDateTime(BusinessClock.Now)).ToDateTime(TimeOnly.MinValue),
            TimeSpan.FromHours(8)).AddMinutes(-5).ToUniversalTime();
        var baselines = new Dictionary<(string Plate, DateOnly Day), (long Event, LoadBaselineVersion Version, LoadMeasurementInput Source)>();
        using (var command = Command(connection, transaction, """
            SELECT a.Id, b.VersionJson, b.ActivatedAt, a.PayloadJson FROM LoadRadarAudit a
            JOIN LoadRadarBaseline b ON b.VersionId=a.EntityId
            JOIN LoadRadarMeasurement m ON m.MeasurementId=b.MeasurementId
            WHERE a.ActionName='baseline-recalculate' AND m.NodeId=? AND m.ChannelId='CH4' AND a.RecordedAt>=?
            ORDER BY a.Id
            """))
        {
            V(command, "node", nodeId, 64);
            V(command, "since", Stamp(since), 40);
            using var reader = command.ExecuteReader();
            while (reader.Read())
            {
                var version = Decode<LoadBaselineVersion>(reader.GetString(1)) with
                { ActivatedAt = ParseStamp(reader.GetString(2)) };
                using var intent = System.Text.Json.JsonDocument.Parse(reader.GetString(3));
                var source = System.Text.Json.JsonSerializer.Deserialize<LoadMeasurementInput>(intent.RootElement.GetProperty("source"), Json)!;
                var day = DateOnly.FromDateTime(source.MeasuredAt.ToOffset(TimeSpan.FromHours(8)).DateTime);
                if (fromDate is not null && day < fromDate) continue;
                if (intent.RootElement.GetProperty("automatic").GetBoolean() &&
                    baselines.TryGetValue((version.LicensePlate, day), out var newer) && newer.Source.MeasuredAt > source.MeasuredAt)
                    continue;
                baselines[(version.LicensePlate, day)] = (reader.GetInt64(0), version, source);
            }
        }
        if (baselines.Count == 0) return [];
        var inputs = new List<(string Id, string Plate, LoadMeasurementInput Input)>();
        var plates = baselines.Keys.Select(k => k.Plate).Distinct().ToArray();
        var activations = baselines.Values.Select(v => v.Event).ToArray();
        using (var command = Command(connection, transaction, $"""
            SELECT m.MeasurementId, p.LicensePlate, m.PayloadJson FROM LoadRadarMeasurement m
            JOIN LoadRadarPassage p ON p.PassageKey=m.PassageKey
            LEFT JOIN LoadRadarPublication q ON q.PassageKey=m.PassageKey
            WHERE m.NodeId=? AND m.ChannelId=? AND p.BindingJson IS NOT NULL AND m.ReceivedAt>=?
              AND p.LicensePlate IN ({string.Join(",", plates.Select(_ => "?"))})
              AND (q.MeasurementId IS NULL OR q.MeasurementId=m.MeasurementId)
              AND NOT EXISTS (SELECT 1 FROM LoadRadarVolumeRevision r WHERE r.MeasurementId=m.MeasurementId
                  AND r.ActivationId IN ({string.Join(",", activations.Select(_ => "?"))}))
              {(afterId is null ? "" : "AND m.MeasurementId>?")}
              {(measurementId is null ? "" : "AND m.MeasurementId=?")}
            ORDER BY m.MeasurementId
            """))
        {
            V(command, "node", nodeId, 64);
            V(command, "channel", channelId, 64);
            V(command, "since", Stamp(since), 40);
            foreach (var plate in plates) V(command, "plate" + command.Parameters.Count, plate, 32);
            foreach (var activation in activations)
                command.Parameters.Add("activation" + command.Parameters.Count, DmDbType.Int64).Value = activation;
            if (afterId is not null) V(command, "after", afterId, 128);
            if (measurementId is not null) V(command, "measurement", measurementId, 128);
            using var reader = command.ExecuteReader();
            while (reader.Read())
            {
                var id = reader.GetString(0);
                var input = Decode<LoadMeasurementInput>(reader.GetString(2));
                var day = DateOnly.FromDateTime(input.MeasuredAt.ToOffset(TimeSpan.FromHours(8)).DateTime);
                if (input.SourceMode == "live" && input.Verdict == "loaded" && baselines.ContainsKey((reader.GetString(1), day)))
                {
                    inputs.Add((id, reader.GetString(1), input));
                    if (inputs.Count == 16) break;
                }
            }
        }
        var targets = new List<LoadRecalculationTarget>();
        foreach (var row in inputs)
        {
            var day = DateOnly.FromDateTime(row.Input.MeasuredAt.ToOffset(TimeSpan.FromHours(8)).DateTime);
            if (!baselines.TryGetValue((row.Plate, day), out var baseline)) continue;
            var id = LoadRadarService.Key("volume-revision", row.Id, baseline.Event.ToString(System.Globalization.CultureInfo.InvariantCulture));
            using var exists = Command(connection, transaction, "SELECT Id FROM LoadRadarVolumeRevision WHERE Id=?");
            V(exists, "id", id, 64);
            if (exists.ExecuteScalar() is not null) continue;
            targets.Add(new(id, baseline.Event, ReadMeasurement(connection, transaction, row.Id)!, baseline.Version, baseline.Source));
            if (targets.Count == 16) break;
        }
        return targets;
    }

    public IReadOnlyList<LoadVolumeRevision> GetVolumeRevisions(string measurementId)
    {
        using var connection = Open();
        using var command = Command(connection, null,
            "SELECT RevisionJson FROM LoadRadarVolumeRevision WHERE MeasurementId=? ORDER BY ActivationId DESC");
        V(command, "id", measurementId, 128);
        var rows = new List<LoadVolumeRevision>();
        using var reader = command.ExecuteReader();
        while (reader.Read()) rows.Add(Decode<LoadVolumeRevision>(reader.GetString(0)));
        return rows;
    }

    public LoadRecalculationReceipt ApplyRecalculation(string nodeId, LoadRecalculationInput request, string resultHash)
    {
        return Transaction((connection, transaction) =>
        {
            var original = ReadMeasurement(connection, transaction, request.MeasurementId)
                ?? throw new LoadRadarException(404, "原始测量不存在");
            var binding = original.Binding ?? throw Conflict("原车次尚未确认车牌");
            var workflow = LockWorkflow(connection, transaction, binding.WorkflowId);
            if (workflow.Plate != binding.LicensePlate || workflow.Camera != binding.CameraIdentifier)
                throw Conflict("原测量的权威车牌或点位已变化");
            LockPassage(connection, transaction, original.PassageKey);
            var publication = ReadPublication(connection, transaction, original.PassageKey, binding.WorkflowId);
            if (publication is not null && publication.MeasurementId != request.MeasurementId)
                return new LoadRecalculationReceipt(request.Id, "not-selected", null);
            using (var existing = Command(connection, transaction, "SELECT RevisionJson FROM LoadRadarVolumeRevision WHERE Id=?"))
            {
                V(existing, "id", request.Id, 64);
                if (existing.ExecuteScalar() is string json)
                {
                    var revision = Decode<LoadVolumeRevision>(json);
                    if (revision.ResultHash != resultHash) throw Conflict("同一复算编号不允许更换结果");
                    return new LoadRecalculationReceipt(request.Id, "applied", revision);
                }
            }
            // 同日多次CH4空车更新很常见；迟到的旧基线结果不能盖过最新修订。
            var target = RecalculationTargets(connection, transaction, nodeId, original.Measurement.ChannelId,
                measurementId: request.MeasurementId,
                fromDate: DateOnly.FromDateTime(original.Measurement.MeasuredAt.ToOffset(TimeSpan.FromHours(8)).DateTime))
                .FirstOrDefault(t => t.Id == request.Id);
            if (target is null || target.Original.Measurement.MeasurementId != request.MeasurementId ||
                target.ActivationId != request.ActivationId || target.Baseline.VersionId != request.BaselineVersionId)
                return new LoadRecalculationReceipt(request.Id, "superseded", null);
            var input = request.Measurement;
            if (!LoadRadarPublicationPolicy.IsCandidate(input, _boundSpreadMax) ||
                !LoadRadarPublicationPolicy.MatchesBaseline(input, binding, target.Baseline, target.BaselineSource, retrospective: true))
                throw Conflict("复算未满足实车、质量或CH4基线来源要求");
            var volume = input.VolumeM3!.Value.RoundCoalVolume();
            var baselineFile = "load-radar:baseline:" + target.Baseline.VersionId;
            decimal? previousVolume = null;
            string? previousBaseline = null;
            long coalId;
            if (publication is not null)
            {
                coalId = publication.CoalVolumeId;
                using (var read = Command(connection, transaction, "SELECT Volume, BaselineFile FROM CoalVolumeRecord WHERE Id=?"))
                {
                    read.Parameters.Add("id", DmDbType.Int64).Value = coalId;
                    using var reader = read.ExecuteReader();
                    if (!reader.Read()) throw Conflict("正式煤量记录不存在");
                    previousVolume = reader.GetDecimal(0);
                    previousBaseline = reader.IsDBNull(1) ? null : reader.GetString(1);
                }
                using var update = Command(connection, transaction,
                    "UPDATE CoalVolumeRecord SET Volume=?, BaselineFile=?, MeasurementStartedAt=?, MeasurementEndedAt=? WHERE Id=?");
                Decimal(update, "volume", volume, 10, 1);
                V(update, "baseline", baselineFile, 1000);
                update.Parameters.Add("started", DmDbType.DateTime).Value = input.MeasurementStartedAt!.Value.ToOffset(TimeSpan.FromHours(8)).DateTime;
                update.Parameters.Add("ended", DmDbType.DateTime).Value = input.MeasurementEndedAt!.Value.ToOffset(TimeSpan.FromHours(8)).DateTime;
                update.Parameters.Add("id", DmDbType.Int64).Value = coalId;
                update.ExecuteNonQuery();
            }
            else
            {
                using (var legacy = Command(connection, transaction, "SELECT Id FROM CoalVolumeRecord WHERE WorkflowId=?"))
                {
                    V(legacy, "workflow", binding.WorkflowId, 128);
                    if (legacy.ExecuteScalar() is not null) throw Conflict("原车次已有其他来源煤量，不能重复发布");
                }
                var coal = new CoalVolumeRecord
                {
                    WorkflowId = binding.WorkflowId,
                    LicensePlate = binding.LicensePlate,
                    CameraIdentifier = binding.CameraIdentifier,
                    Volume = volume,
                    Status = "success",
                    IdempotencyKey = LoadRadarService.Key("load-radar", original.PassageKey),
                    BusinessKey = LoadRadarService.Key("load-radar-workflow", binding.WorkflowId),
                    PayloadHash = original.PayloadHash,
                    MeasureTime = input.MeasuredAt.ToOffset(TimeSpan.FromHours(8)).DateTime,
                    MeasurementStartedAt = input.MeasurementStartedAt?.ToOffset(TimeSpan.FromHours(8)).DateTime,
                    MeasurementEndedAt = input.MeasurementEndedAt?.ToOffset(TimeSpan.FromHours(8)).DateTime,
                    SourceFile = "load-radar:measurement:" + request.MeasurementId,
                    BaselineFile = baselineFile,
                    CreatedAt = BusinessClock.Now
                };
                coalId = InsertCoal(connection, transaction, coal);
                publication = new(original.PassageKey, binding.WorkflowId, request.MeasurementId, coalId,
                    "system:load-radar", "CH4当日空车基线事后复算首次入库", DateTimeOffset.UtcNow);
                using var insert = Command(connection, transaction,
                    "INSERT INTO LoadRadarPublication (PassageKey, WorkflowId, MeasurementId, CoalVolumeId, PublicationJson) VALUES (?,?,?,?,?)");
                V(insert, "key", original.PassageKey, 64); V(insert, "workflow", binding.WorkflowId, 128);
                V(insert, "measurement", request.MeasurementId, 128);
                insert.Parameters.Add("coal", DmDbType.Int64).Value = coalId; J(insert, "json", publication); insert.ExecuteNonQuery();
            }
            var result = new LoadVolumeRevision(request.Id, request.MeasurementId, request.ActivationId,
                request.BaselineVersionId, coalId, previousVolume, volume, previousBaseline, resultHash, input, request.Provenance, DateTimeOffset.UtcNow);
            using (var insert = Command(connection, transaction,
                "INSERT INTO LoadRadarVolumeRevision (Id,MeasurementId,ActivationId,BaselineVersionId,CoalVolumeId,ResultHash,RevisionJson) VALUES (?,?,?,?,?,?,?)"))
            {
                V(insert, "id", result.Id, 64); V(insert, "measurement", result.MeasurementId, 128);
                insert.Parameters.Add("event", DmDbType.Int64).Value = result.ActivationId;
                V(insert, "baseline", result.BaselineVersionId, 128);
                insert.Parameters.Add("coal", DmDbType.Int64).Value = coalId;
                V(insert, "hash", resultHash, 64); J(insert, "json", result); insert.ExecuteNonQuery();
            }
            Audit(connection, transaction, "volume-revision", result.Id, "system:load-radar", "CH4同牌当日体积复算", result);
            return new LoadRecalculationReceipt(request.Id, "applied", result);
        });
    }
}
