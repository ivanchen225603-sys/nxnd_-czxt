using Dm;
using Microsoft.Extensions.Configuration;
using System.Reflection;
using System.Text.Json;
using Xunit;

namespace CarPointCloudWebApi.Tests;

/// <summary>仅使用显式配置的隔离 DM8；增量 SQL 须由测试管理员预先手动执行。</summary>
[Collection("Integration")]
public sealed class LoadRadarRepositoryTests
{
    [DatabaseFact]
    public void Later_CH4_baseline_revises_all_lanes_once_and_repository_restart_keeps_history()
    {
        using var store = new IsolatedStore();
        var date = DateOnly.FromDateTime(BusinessClock.Now);
        var start = new DateTimeOffset(date.ToDateTime(TimeOnly.MinValue), TimeSpan.FromHours(8));
        var loaded = new List<LoadMeasurementRecord>();
        foreach (var channel in new[] { "CH1", "CH2", "CH3", "CH4" })
        {
            var row = store.Measurement("morning-" + channel, "loaded", channel: channel);
            row = row with
            {
                Measurement = row.Measurement with
                {
                    MeasuredAt = start.AddHours(8),
                    VolumeStatus = "unavailable",
                    VolumeM3 = null
                }
            };
            row = row with { PayloadHash = LoadRadarService.Key(row.Measurement) };
            store.Repository.SaveMeasurement(row);
            store.Repository.BindPassage(row, store.Binding(row, store.Workflow()));
            loaded.Add(store.Repository.GetMeasurement(row.Measurement.MeasurementId)!);
        }
        var empty = store.Measurement("afternoon-empty");
        empty = empty with { Measurement = empty.Measurement with { MeasuredAt = start.AddHours(16) } };
        empty = empty with { PayloadHash = LoadRadarService.Key(empty.Measurement) };
        store.Repository.SaveMeasurement(empty);
        store.Repository.BindPassage(empty, store.Binding(empty, store.Workflow()));
        empty = store.Repository.GetMeasurement(empty.Measurement.MeasurementId)!;
        var version = store.Baseline(empty);
        store.Repository.ActivateBaseline(version.VersionId, null, "tester", "卸煤后CH4空车基线");
        foreach (var row in loaded)
        {
            var target = Assert.Single(store.NewRepository().GetRecalculations(store.Node, row.Measurement.ChannelId, null, date));
            var result = row.Measurement with
            {
                VolumeStatus = "success",
                VolumeM3 = 51.25m,
                BaselineVersionId = version.VersionId,
                BaselineUse = new("CH4", "calibration-test", new string('c', 64),
                    "ch4-shared-v1", "baseline_fallback", "按基准计算", 0, 0, 0, 0, null)
            };
            var request = new LoadRecalculationInput(target.Id, target.ActivationId, result.MeasurementId,
                version.VersionId, result, new(new string('d', 64), new string('e', 64), new string('f', 64), new string('a', 64)));
            var first = store.Repository.ApplyRecalculation(store.Node, request, LoadRadarService.Key(request));
            Assert.Equal("applied", first.Status);
            Assert.Equal(first, store.NewRepository().ApplyRecalculation(store.Node, request, LoadRadarService.Key(request)));
            Assert.Single(store.NewRepository().GetVolumeRevisions(result.MeasurementId));
            Assert.Equal(1, store.CountCoal(row.Binding!.WorkflowId));
            Assert.Empty(store.NewRepository().GetRecalculations(store.Node, result.ChannelId, null, date));
            Assert.Null(store.NewRepository().GetMeasurement(result.MeasurementId)!.Measurement.VolumeM3);
            Assert.Equal(51.3m, store.NewRepository().GetMeasurement(result.MeasurementId)!.LatestRevision!.VolumeM3);
        }
    }

    [DatabaseFact]
    public void Delayed_older_automatic_empty_keeps_newer_active_baseline_and_intent_is_durable()
    {
        using var store = new IsolatedStore();
        var older = store.Bound("older-empty", "empty");
        var oldVersion = store.Baseline(older);
        var newer = store.Measurement("newer-empty");
        newer = newer with { Measurement = newer.Measurement with { MeasuredAt = older.Measurement.MeasuredAt.AddHours(1) } };
        newer = newer with { PayloadHash = LoadRadarService.Key(newer.Measurement) };
        store.Repository.SaveMeasurement(newer);
        store.Repository.BindPassage(newer, store.Binding(newer, store.Workflow()));
        var newVersion = store.Baseline(store.Repository.GetMeasurement(newer.Measurement.MeasurementId)!);
        store.Repository.ActivateAutomaticBaseline(newVersion.VersionId, "tester", "先收到较新的空车");
        store.NewRepository().ActivateAutomaticBaseline(oldVersion.VersionId, "tester", "恢复后收到更早的空车");
        Assert.Equal(newVersion.VersionId, store.NewRepository().GetPlateBaselineAt(store.Node, store.Plate, DateTimeOffset.UtcNow)!.VersionId);
        Assert.NotNull(store.NewRepository().GetBaseline(oldVersion.VersionId)!.ActivatedAt);
    }

    [DatabaseFact]
    public void Already_published_volume_updates_in_place_with_two_revisions_and_old_retry_never_reverts_it()
    {
        using var store = new IsolatedStore();
        var initialEmpty = store.Bound("initial-empty", "empty");
        var baseline = store.Baseline(initialEmpty);
        store.Repository.ActivateBaseline(baseline.VersionId, null, "tester", "初始空车");
        var loaded = store.Bound("published-loaded", "loaded", baseline.VersionId);
        var publication = store.Repository.Publish(loaded, store.Coal(loaded), "tester", "初次正式发布");
        LoadRecalculationInput? first = null;
        foreach (var nextVolume in new[] { 60m, 70m })
        {
            var empty = store.Bound("next-empty-" + nextVolume, "empty");
            var next = store.Baseline(empty);
            store.Repository.ActivateBaseline(next.VersionId, baseline.VersionId, "tester", "卸煤后更新空车");
            baseline = next;
            var target = Assert.Single(store.NewRepository().GetRecalculations(store.Node, "CH4", null,
                DateOnly.FromDateTime(BusinessClock.Now)));
            var result = loaded.Measurement with
            {
                VolumeM3 = nextVolume,
                BaselineVersionId = next.VersionId,
                MeasurementStartedAt = loaded.Measurement.MeasuredAt.AddSeconds(-1),
                MeasurementEndedAt = loaded.Measurement.MeasuredAt,
                BaselineUse = new("CH4", "calibration-test", new string('c', 64), "ch4-shared-v1", "baseline_fallback",
                    "原基准", 0, 0, 0, 0, null)
            };
            var request = new LoadRecalculationInput(target.Id, target.ActivationId, result.MeasurementId, next.VersionId,
                result, new(new string('d', 64), new string('e', 64), new string('f', 64), new string('a', 64)));
            first ??= request;
            var revision = store.NewRepository().ApplyRecalculation(store.Node, request, LoadRadarService.Key(request)).Revision!;
            Assert.Equal(publication.CoalVolumeId, revision.CoalVolumeId);
            Assert.Equal(nextVolume == 60m ? 45.6m : 60m, revision.PreviousVolumeM3);
            Assert.Equal(1, store.CountCoal(loaded.Binding!.WorkflowId));
        }
        store.NewRepository().ApplyRecalculation(store.Node, first!, LoadRadarService.Key(first!));
        var restored = store.NewRepository().GetMeasurement(loaded.Measurement.MeasurementId)!;
        Assert.Equal(70m, restored.LatestRevision!.VolumeM3);
        Assert.Equal(loaded.Measurement, restored.Measurement);
        Assert.Equal(2, store.NewRepository().GetVolumeRevisions(loaded.Measurement.MeasurementId).Count);
    }
    [DatabaseFact]
    public void Recalculation_keeps_the_selected_measurement_of_an_already_published_passage()
    {
        using var store = new IsolatedStore();
        var empty = store.Bound("empty", "empty");
        var baseline = store.Baseline(empty);
        store.Repository.ActivateBaseline(baseline.VersionId, null, "tester", "初始基线");
        var loaded = store.Bound("loaded", "loaded", baseline.VersionId);
        var publication = store.Repository.Publish(loaded, store.Coal(loaded), "tester", "选定测量");
        var replacement = store.Save("loaded", "loaded", baseline.VersionId);
        var nextEmpty = store.Bound("next-empty", "empty");
        var next = store.Baseline(nextEmpty);
        store.Repository.ActivateBaseline(next.VersionId, baseline.VersionId, "tester", "更新基线");
        var target = Assert.Single(store.NewRepository().GetRecalculations(store.Node, "CH4", null,
            DateOnly.FromDateTime(BusinessClock.Now)));
        Assert.Equal(loaded.Measurement.MeasurementId, target.Original.Measurement.MeasurementId);
        var request = new LoadRecalculationInput("unused", target.ActivationId, replacement.Measurement.MeasurementId,
            next.VersionId, replacement.Measurement,
            new(new string('d', 64), new string('e', 64), new string('f', 64), new string('a', 64)));
        var receipt = store.NewRepository().ApplyRecalculation(store.Node, request, "unused");
        Assert.Equal("not-selected", receipt.Status);
        Assert.Null(receipt.Revision);
        Assert.Equal(publication.CoalVolumeId, store.Repository.GetMeasurement(loaded.Measurement.MeasurementId)!.Publication!.CoalVolumeId);
        Assert.Empty(store.Repository.GetVolumeRevisions(replacement.Measurement.MeasurementId));
        Assert.Equal(1, store.CountCoal(loaded.Binding!.WorkflowId));
    }

    [Fact]
    public void MissingDatabaseConfiguration_DoesNotBecomeInMemorySuccess()
    {
        var configuration = new ConfigurationBuilder().AddInMemoryCollection().Build();
        Assert.Throws<InvalidOperationException>(() => new LoadRadarRepository(configuration));
    }

    [Fact]
    public void JsonParameter_UsesExplicitClobWithoutTruncatingTenThousandPoints()
    {
        var input = new LoadMeasurementInput { Points = LargePointCloud() };
        using var command = new DmCommand();
        var helper = typeof(LoadRadarRepository).GetMethod("J", BindingFlags.NonPublic | BindingFlags.Static)!;
        helper.Invoke(null, [command, "payload", input]);
        var parameter = Assert.IsType<DmParameter>(Assert.Single(command.Parameters.Cast<DmParameter>()));
        Assert.Equal(DmDbType.Clob, parameter.DmSqlType);
        var json = Assert.IsType<string>(parameter.Value);
        Assert.True(json.Length > 32767);
        using var document = JsonDocument.Parse(json);
        var points = document.RootElement.GetProperty("points");
        Assert.Equal(10_000, points.GetArrayLength());
        Assert.Equal(9999.125f, points[9999][0].GetSingle());
    }

    [DatabaseFact]
    public async Task ImmutableMeasurement_ConcurrentReplayKeepsOriginalReceiptAndRejectsChangedPayload()
    {
        using var store = new IsolatedStore();
        var measurement = store.Measurement("passage");
        var input = measurement.Measurement with { Points = LargePointCloud(), PointCount = 10_000 };
        measurement = measurement with { Measurement = input, PayloadHash = LoadRadarService.Key(input) };
        var results = await Task.WhenAll(Enumerable.Range(0, 6).Select(_ =>
            Task.Run(() => store.Repository.SaveMeasurement(measurement))));
        Assert.All(results, result => Assert.Equal(measurement.ReceivedAt, result.ReceivedAt));
        var replay = store.Repository.SaveMeasurement(measurement with { ReceivedAt = DateTimeOffset.UtcNow.AddHours(1) });
        Assert.Equal(measurement.ReceivedAt, replay.ReceivedAt);
        Assert.Equal(10_000, replay.Measurement.Points.Length);
        Assert.Equal(input.Points[^1], replay.Measurement.Points[^1]);
        var changed = measurement with { Measurement = measurement.Measurement with { PointCount = 99 } };
        Assert.Equal(409, Assert.Throws<LoadRadarException>(() => store.Repository.SaveMeasurement(changed)).StatusCode);
        Assert.Single(store.Repository.GetHistory(store.Node, null, null, null, 100));
    }

    [DatabaseFact]
    public void Ch4PlateHistoryAllowsOtherLaneAfterRepositoryRestart()
    {
        using var store = new IsolatedStore();
        var source = store.Bound("ch4-source", "empty");
        var baseline = store.Baseline(source);
        store.Repository.ActivateBaseline(baseline.VersionId, null, "tester", "CH4按牌共用");
        var proposed = store.Measurement("ch2-loaded", "loaded", baseline.VersionId);
        var input = proposed.Measurement with
        {
            ChannelId = "CH2",
            CalibrationId = "ch2-calibration",
            ConfigurationHash = new string('d', 64),
            BoundSpread = null,
            BaselineUse = new("CH4", source.Measurement.CalibrationId, source.Measurement.ConfigurationHash,
                "ch4-shared-v1", "baseline_fallback", "特征不足按CH4原基准", 0, 0, 0, 0, null)
        };
        var row = proposed with
        {
            PassageKey = LoadRadarService.Key(input.NodeId, input.ChannelId, input.PassageId),
            Measurement = input,
            PayloadHash = LoadRadarService.Key(input)
        };
        var saved = store.Repository.SaveMeasurement(row);
        store.Repository.BindPassage(saved, store.Binding(saved, store.Workflow()));
        var restarted = store.NewRepository();
        Assert.Equal(baseline.VersionId, restarted.GetPlateBaselineAt(store.Node, store.Plate, input.MeasuredAt)!.VersionId);
        var bound = restarted.GetMeasurement(input.MeasurementId)!;
        var first = restarted.Publish(bound, store.Coal(bound), "tester", "跨道回退正式发布");
        Assert.Equal(first, restarted.Publish(bound, store.Coal(bound), "tester", "重启后同编号重试"));
        Assert.Equal(1, store.CountCoal(bound.Binding!.WorkflowId));
    }

    private static float[][] LargePointCloud() => Enumerable.Range(0, 10_000)
        .Select(index => new[] { index + 0.125f, index + 0.25f, -3.5f }).ToArray();

    [DatabaseFact]
    public void PublicationUsesBaselineAtOriginalMeasurementTimeAfterHeadChanges()
    {
        using var store = new IsolatedStore();
        var source = store.Bound("empty-for-history", "empty");
        var first = store.Baseline(source);
        store.Repository.ActivateBaseline(first.VersionId, null, "tester", "activate-v1");
        var original = store.Measurement("late-loaded", "loaded", first.VersionId);
        var second = store.Baseline(store.Save("empty-for-history", "empty"));
        store.Repository.ActivateBaseline(second.VersionId, first.VersionId, "tester", "activate-v2");
        var saved = store.Repository.SaveMeasurement(original);
        store.Repository.BindPassage(saved, store.Binding(saved, store.Workflow()));
        var bound = store.Repository.GetMeasurement(saved.Measurement.MeasurementId)!;
        var published = store.Repository.Publish(bound, store.Coal(bound), "tester", "late identity using original v1");
        Assert.Equal(bound.Measurement.MeasurementId, published.MeasurementId);
        Assert.Equal(1, store.CountCoal(bound.Binding!.WorkflowId));

        var invalid = store.Measurement("wrong-late-baseline", "loaded", second.VersionId);
        invalid = invalid with { Measurement = invalid.Measurement with { MeasuredAt = original.Measurement.MeasuredAt } };
        invalid = invalid with { PayloadHash = LoadRadarService.Key(invalid.Measurement) };
        saved = store.Repository.SaveMeasurement(invalid);
        store.Repository.BindPassage(saved, store.Binding(saved, store.Workflow()));
        bound = store.Repository.GetMeasurement(saved.Measurement.MeasurementId)!;
        Assert.Equal(409, Assert.Throws<LoadRadarException>(() => store.Repository.Publish(bound, store.Coal(bound), "tester", "wrong-v2")).StatusCode);
        Assert.Equal(0, store.CountCoal(bound.Binding!.WorkflowId));
        Assert.Null(store.Repository.GetMeasurement(saved.Measurement.MeasurementId)!.Publication);
    }

    [Theory]
    [InlineData("2026-01-01T08:00:00+08:00", true)]
    [InlineData("2026-01-01T00:00:00Z", true)]
    [InlineData("2026-01-01T00:01:00Z", true)]
    [InlineData("2025-12-31T23:59:59.999Z", false)]
    [InlineData("2026-01-01T00:01:00.001Z", false)]
    public void HistoricalPayloadTimeIsComparedAsAnInstant(string timestamp, bool expected)
    {
        var helper = typeof(LoadRadarRepository).GetMethod("WithinMeasuredTime", BindingFlags.NonPublic | BindingFlags.Static)!;
        var start = DateTimeOffset.Parse("2026-01-01T00:00:00Z");
        var payload = JsonSerializer.Serialize(new { measuredAt = timestamp, receivedAt = "2026-02-01T00:00:00Z" });
        Assert.Equal(expected, helper.Invoke(null, [payload, start, start.AddMinutes(1)]));
    }

    [Theory]
    [InlineData("{}")]
    [InlineData("{\"measuredAt\":null}")]
    [InlineData("{\"measuredAt\":\"bad\"}")]
    public void DamagedHistoryTimeIsNotSilentlySkipped(string payload)
    {
        var helper = typeof(LoadRadarRepository).GetMethod("WithinMeasuredTime", BindingFlags.NonPublic | BindingFlags.Static)!;
        Assert.Throws<TargetInvocationException>(() => helper.Invoke(null, [payload, null, DateTimeOffset.UtcNow]));
    }

    [DatabaseFact]
    public void HistoryFiltersOriginalTimeBeforeTakingPageAndKeepsSequenceCursor()
    {
        using var store = new IsolatedStore();
        var start = DateTimeOffset.Parse("2026-01-01T00:00:00Z");
        var saved = new List<LoadMeasurementRecord>();
        foreach (var minute in new[] { 0, 1, 2, -2, 10 })
        {
            var row = store.Measurement("time-" + minute);
            var input = row.Measurement with { MeasuredAt = start.AddMinutes(minute) };
            saved.Add(store.Repository.SaveMeasurement(row with { Measurement = input, PayloadHash = LoadRadarService.Key(input) }));
        }
        var first = Assert.Single(store.Repository.GetHistory(store.Node, "CH4", null, null, 1, start, start.AddMinutes(2)));
        Assert.Equal(saved[2].Measurement.MeasurementId, first.Measurement.MeasurementId);
        var second = store.Repository.GetHistory(store.Node, "CH4", null, first.Measurement.MeasurementId, 2, start, start.AddMinutes(2));
        Assert.Equal(new[] { saved[1].Measurement.MeasurementId, saved[0].Measurement.MeasurementId }, second.Select(r => r.Measurement.MeasurementId));
    }

    [DatabaseFact]
    public void ExactWorkflowBinding_RejectsOtherCameraAndExclusiveWorkflowReuse()
    {
        using var store = new IsolatedStore();
        var first = store.Save("first");
        var workflow = store.Workflow();
        var binding = store.Binding(first, workflow);
        Assert.Equal(409, Assert.Throws<LoadRadarException>(() =>
            store.Repository.BindPassage(first, binding with { CameraIdentifier = "OTHER_CAMERA" })).StatusCode);
        Assert.Null(store.Repository.GetMeasurement(first.Measurement.MeasurementId)!.Binding);
        Assert.Equal(binding, store.Repository.BindPassage(first, binding));
        Assert.Equal(binding, store.Repository.BindPassage(first, binding with { BoundAt = DateTimeOffset.UtcNow }));
        var other = store.Save("second");
        Assert.Equal(409, Assert.Throws<LoadRadarException>(() =>
            store.Repository.BindPassage(other, store.Binding(other, workflow))).StatusCode);
        var anotherMeasurement = store.Save("first");
        Assert.Equal(binding, anotherMeasurement.Binding);
        Assert.Equal(2, store.Repository.GetHistory(store.Node, null, store.Plate, null, 100).Count);
    }

    [DatabaseFact]
    public void CoalPublication_RollsBackAuditFailureAndRejectsReplacementAndLegacySource()
    {
        using var store = new IsolatedStore();
        var empty = store.Bound("empty", "empty");
        var baseline = store.Baseline(empty);
        store.Repository.ActivateBaseline(baseline.VersionId, null, "tester", "确认基线");
        var loaded = store.Bound("loaded", "loaded", baseline.VersionId);
        var coal = store.Coal(loaded);
        // 审计参数验证在煤量和发布槽位 INSERT 后失败，必须整笔回滚。
        Assert.Throws<LoadRadarException>(() =>
            store.Repository.Publish(loaded, coal, new string('x', 129), "故障注入"));
        Assert.Equal(0, store.CountCoal(loaded.Binding!.WorkflowId));
        Assert.Null(store.Repository.GetMeasurement(loaded.Measurement.MeasurementId)!.Publication);
        var publication = store.Repository.Publish(loaded, coal, "tester", "确认发布");
        var replay = store.Repository.Publish(loaded, coal, "tester", "响应丢失后的重试");
        Assert.Equal(publication.CoalVolumeId, replay.CoalVolumeId);
        Assert.Equal(1, store.CountCoal(loaded.Binding.WorkflowId));
        var replacement = store.Save("loaded", "loaded", baseline.VersionId);
        Assert.Equal(409, Assert.Throws<LoadRadarException>(() =>
            store.Repository.Publish(replacement, store.Coal(replacement), "tester", "错误替换")).StatusCode);

        var legacy = store.Bound("legacy", "loaded", baseline.VersionId);
        store.InsertLegacyCoal(legacy.Binding!.WorkflowId);
        Assert.Equal(409, Assert.Throws<LoadRadarException>(() =>
            store.Repository.Publish(legacy, store.Coal(legacy), "tester", "阻止双源")).StatusCode);
        Assert.Equal(1, store.CountCoal(legacy.Binding.WorkflowId));
    }

    [DatabaseFact]
    public async Task PublicationIntentPersistsOnceAndStaleAttemptCannotOverwriteOutcome()
    {
        using var store = new IsolatedStore();
        var row = store.Measurement("queued", "loaded", "baseline-not-yet-validated");
        await Task.WhenAll(Enumerable.Range(0, 6).Select(_ =>
            Task.Run(() => store.Repository.SaveMeasurement(row))));
        var id = row.Measurement.MeasurementId;
        Assert.Equal(("pending", 0), store.PublicationState(id));
        var original = new LoadRadarPublicationJob(id, 0);
        var next = DateTimeOffset.UtcNow.AddMinutes(1);
        store.Repository.SetPublicationOutcome(original, "pending", "awaiting_identity", next);
        store.Repository.SetPublicationOutcome(original, "rejected", "stale_attempt", next);
        Assert.Equal(("pending", 1), store.PublicationState(id));
        // 新仓储重新读持久层；同 ID 重放不重新建任务或清零尝试次数。
        var restarted = store.NewRepository();
        restarted.SaveMeasurement(row);
        Assert.Equal(("pending", 1), store.PublicationState(id));
        restarted.SetPublicationOutcome(new(id, 1), "published", "published", next);
        store.Repository.SetPublicationOutcome(new(id, 1), "pending", "late_ack", next);
        Assert.Equal(("published", 2), store.PublicationState(id));
    }

    private sealed class IsolatedStore : IDisposable
    {
        private readonly string _connectionString = TestConfiguration.GetDatabaseConnectionString()!;
        private readonly string _suffix = Guid.NewGuid().ToString("N");
        private readonly List<string> _workflows = [];
        private readonly List<string> _baselines = [];
        private readonly List<string> _passages = [];
        public string Node { get; }
        public string Plate => "宁A25771";
        public LoadRadarRepository Repository { get; }

        public IsolatedStore()
        {
            Node = "load-repo-" + _suffix;
            Repository = NewRepository();
        }

        public LoadRadarRepository NewRepository() => new(new ConfigurationBuilder().AddInMemoryCollection(
            new Dictionary<string, string?> { ["ConnectionStrings:CarPointCloudDb"] = _connectionString }).Build());

        public (string Status, int Attempts) PublicationState(string id)
        {
            using var connection = Open();
            using var command = new DmCommand(
                "SELECT Status, AttemptCount FROM LoadRadarPublishJob WHERE MeasurementId = ?", connection);
            Add(command, "id", id);
            using var reader = command.ExecuteReader();
            Assert.True(reader.Read());
            var state = (reader.GetString(0), reader.GetInt32(1));
            Assert.False(reader.Read());
            return state;
        }

        public LoadMeasurementRecord Measurement(string passage, string verdict = "empty", string? baseline = null, string channel = "CH4")
        {
            var input = new LoadMeasurementInput
            {
                MeasurementId = Guid.NewGuid().ToString("N"),
                NodeId = Node,
                ChannelId = channel,
                PassageId = passage,
                CalibrationId = "calibration-test",
                SourceMode = "live",
                AlgorithmVersion = "algorithm-test",
                ConfigurationHash = new string('c', 64),
                BaselineCandidateSha256 = verdict == "empty" ? new string('a', 64) : null,
                MeasuredAt = DateTimeOffset.UtcNow,
                Verdict = verdict,
                VolumeStatus = verdict == "loaded" ? "success" : "unavailable",
                VolumeM3 = verdict == "loaded" ? 45.55m : null,
                BaselineVersionId = baseline,
                Reason = "仓储隔离测试",
                PointCount = 100,
                FilledRatio = 0.03m,
                BoundSpread = 0.02m,
                HoleRatio = 0.05m
            };
            var key = LoadRadarService.Key(Node, channel, passage);
            if (!_passages.Contains(key)) _passages.Add(key);
            return new(key, LoadRadarService.Key(input), input, DateTimeOffset.UtcNow);
        }

        public LoadMeasurementRecord Save(string passage, string verdict = "empty", string? baseline = null) =>
            Repository.SaveMeasurement(Measurement(passage, verdict, baseline));

        public string Workflow()
        {
            var id = ReserveWorkflowId();
            using var connection = Open();
            using var command = new DmCommand(
                @"INSERT INTO VehicleArrival (EventId, DeviceId, CameraIdentifier, EventType, SessionId,
                  LicensePlate, PlateStatus, PlateSource, ArrivalTime, RadarStatus, CreatedAt, UpdatedAt)
                  VALUES (?, 'TEST_LOAD', 'CAM_LOAD_TEST', 'ARRIVAL', ?, ?, 'SUCCEEDED', 'MANUAL', SYSDATE, 'NOT_REQUIRED', SYSDATE, SYSDATE)", connection);
            Add(command, "event", id);
            Add(command, "workflow", id);
            Add(command, "plate", Plate);
            command.ExecuteNonQuery();
            return id;
        }

        public string ReserveWorkflowId()
        {
            var id = "load-test-" + Guid.NewGuid().ToString("N");
            _workflows.Add(id);
            return id;
        }

        public int CountArrivalOrJob(string table, string column, string workflow)
        {
            using var connection = Open();
            using var command = new DmCommand($"SELECT COUNT(*) FROM {table} WHERE {column}=?", connection);
            Add(command, "workflow", workflow);
            return Convert.ToInt32(command.ExecuteScalar());
        }

        public LoadPassageBinding Binding(LoadMeasurementRecord row, string workflow) =>
            new(row.PassageKey, workflow, Plate, "CAM_LOAD_TEST", "trailer-test-" + _suffix,
                "tester", "确认测试车辆", DateTimeOffset.UtcNow);

        public LoadMeasurementRecord Bound(string passage, string verdict, string? baseline = null)
        {
            var record = Save(passage, verdict, baseline);
            Repository.BindPassage(record, Binding(record, Workflow()));
            return Repository.GetMeasurement(record.Measurement.MeasurementId)!;
        }

        public LoadBaselineVersion Baseline(LoadMeasurementRecord row)
        {
            var id = LoadRadarService.Key("baseline", row.Measurement.MeasurementId, new string('a', 64));
            _baselines.Add(id);
            var binding = row.Binding!;
            return Repository.CreateBaseline(new(id,
                LoadRadarService.Key(Node, "CH4", row.Measurement.CalibrationId, binding.VehicleIdentity),
                Node, "CH4", row.Measurement.CalibrationId, binding.VehicleIdentity, Plate,
                row.Measurement.MeasurementId, new string('a', 64), "tester", "确认空车", DateTimeOffset.UtcNow));
        }

        public CoalVolumeRecord Coal(LoadMeasurementRecord row) => new()
        {
            IdempotencyKey = LoadRadarService.Key("load-radar", row.PassageKey),
            BusinessKey = LoadRadarService.Key("load-radar-workflow", row.Binding!.WorkflowId),
            PayloadHash = LoadRadarService.Key(row.PayloadHash, row.Binding.WorkflowId, row.Measurement.BaselineVersionId!),
            WorkflowId = row.Binding.WorkflowId,
            LicensePlate = Plate,
            CameraIdentifier = "CAM_LOAD_TEST",
            Volume = row.Measurement.VolumeM3!.Value.RoundCoalVolume(),
            Status = "success",
            MeasureTime = BusinessClock.Now,
            CreatedAt = BusinessClock.Now
        };

        public int CountCoal(string workflow)
        {
            using var connection = Open();
            using var command = new DmCommand("SELECT COUNT(*) FROM CoalVolumeRecord WHERE WorkflowId = ?", connection);
            Add(command, "workflow", workflow);
            return Convert.ToInt32(command.ExecuteScalar());
        }

        public void InsertLegacyCoal(string workflow)
        {
            using var connection = Open();
            using var command = new DmCommand(
                @"INSERT INTO CoalVolumeRecord (IdempotencyKey, BusinessKey, WorkflowId, PayloadHash, LicensePlate,
                  CameraIdentifier, Volume, Unit, MeasureTime, Status, CreatedAt)
                  VALUES (?, ?, ?, ?, ?, 'CAM_LOAD_TEST', 50, 'm3', SYSDATE, 'success', SYSDATE)", connection);
            Add(command, "id", Guid.NewGuid().ToString("N"));
            Add(command, "business", Guid.NewGuid().ToString("N"));
            Add(command, "workflow", workflow);
            Add(command, "hash", new string('b', 64));
            Add(command, "plate", Plate);
            command.ExecuteNonQuery();
        }

        public void Dispose()
        {
            // 仅清理本用例生成的精确 ID，不使用全库 TEST% 清理规则。
            using var connection = Open();
            using var transaction = (DmTransaction)connection.BeginTransaction();
            void Delete(string sql, string id)
            {
                using var command = new DmCommand(sql, connection, transaction);
                Add(command, "id", id);
                command.ExecuteNonQuery();
            }
            Delete("DELETE FROM LoadRadarAudit WHERE EntityId IN (SELECT Id FROM LoadRadarVolumeRevision WHERE MeasurementId IN (SELECT MeasurementId FROM LoadRadarMeasurement WHERE NodeId=?))", Node);
            Delete("DELETE FROM LoadRadarVolumeRevision WHERE MeasurementId IN (SELECT MeasurementId FROM LoadRadarMeasurement WHERE NodeId=?)", Node);
            // 跨道用例会重建 PassageKey，按本节点实际保存的测量清理发布槽，不能只用构造时的旧 key。
            Delete("DELETE FROM LoadRadarPublication WHERE MeasurementId IN (SELECT MeasurementId FROM LoadRadarMeasurement WHERE NodeId=?)", Node);
            foreach (var passage in _passages)
            {
                Delete("DELETE FROM LoadRadarPublication WHERE PassageKey = ?", passage);
                Delete("DELETE FROM LoadRadarAudit WHERE EntityId = ?", passage);
            }
            foreach (var workflow in _workflows) Delete("DELETE FROM CoalVolumeRecord WHERE WorkflowId = ?", workflow);
            foreach (var baseline in _baselines)
            {
                Delete("UPDATE LoadRadarBaselineHead SET ActiveVersionId=NULL WHERE ActiveVersionId = ?", baseline);
                Delete("DELETE FROM LoadRadarAudit WHERE EntityId = ?", baseline);
            }
            foreach (var baseline in _baselines) Delete("DELETE FROM LoadRadarBaseline WHERE VersionId = ?", baseline);
            Delete("DELETE FROM LoadRadarPublishJob WHERE MeasurementId IN (SELECT MeasurementId FROM LoadRadarMeasurement WHERE NodeId = ?)", Node);
            Delete("DELETE FROM LoadRadarBaselineHead WHERE ScopeKey = ?",
                LoadRadarService.Key(Node, "CH4", "calibration-test", "trailer-test-" + _suffix));
            Delete("DELETE FROM LoadRadarMeasurement WHERE NodeId = ?", Node);
            Delete("DELETE FROM LoadRadarPassageStart WHERE PassageKey IN (SELECT PassageKey FROM LoadRadarPassage WHERE NodeId = ?)", Node);
            Delete("DELETE FROM LoadRadarPassage WHERE NodeId = ?", Node);
            foreach (var workflow in _workflows)
            {
                Delete("DELETE FROM PlateAcquisitionJob WHERE WorkflowId = ?", workflow);
                Delete("DELETE FROM LicensePlateRecord WHERE WorkflowId = ?", workflow);
                Delete("DELETE FROM VehicleArrival WHERE SessionId = ?", workflow);
            }
            transaction.Commit();
        }

        private DmConnection Open()
        {
            var connection = new DmConnection(_connectionString);
            connection.Open();
            return connection;
        }

        private static void Add(DmCommand command, string name, string value) =>
            command.Parameters.Add(name, DmDbType.VarChar, 128).Value = value;
    }
}
