using System.IO.Compression;
using System.Security.Cryptography;
using System.Runtime.CompilerServices;
using System.Net;
using System.Net.Http.Json;
using System.Text.Json;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.TestHost;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Options;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class LoadRadarServiceTests : IDisposable
{
    private readonly string _root = Path.Combine(Path.GetTempPath(), "load-radar-tests-" + Guid.NewGuid().ToString("N"));
    private readonly MemoryRepository _repository = new();
    private readonly WorkflowRepository _workflow = new();
    private readonly PlateSource _plates = new();
    private readonly LoadRadarOptions _options;
    private readonly LoadRadarAssetStore _assets;
    private readonly LoadRadarService _service;

    public LoadRadarServiceTests()
    {
        _repository.Workflow = _workflow;
        _options = new LoadRadarOptions
        {
            Enabled = true,
            StorageRoot = _root,
            // 本类里的质量边界用例（0.03 上一个 ulp 不能被舍进去）需要**配了**宽度门槛
            // 才有意义。默认值已改成 null = 不设门槛，那一档由
            // LoadRadarBoundSpreadGateTests 覆盖。
            VolumeBoundSpreadMax = 0.03m,
            Nodes = [new LoadRadarNodeOptions { NodeId = "n1", AgentKey = new string('a', 40),
                Channels = [new LoadRadarLaneOptions { ChannelId = "CH4", Name = "空车通道", CameraIdentifier = "CAM010" }] }]
        };
        _assets = new(Options.Create(_options));
        _service = new(_repository, _workflow, Options.Create(_options), _assets);
    }

    private static LoadMeasurementInput Input(string id = "m1") => new()
    {
        MeasurementId = id,
        NodeId = "n1",
        ChannelId = "CH4",
        PassageId = "p1",
        CalibrationId = "cal1",
        SourceMode = "live",
        AlgorithmVersion = "clean-room-v1",
        ConfigurationHash = new string('c', 64),
        MeasuredAt = DateTimeOffset.UtcNow.AddMinutes(-1),
        Verdict = "empty",
        Reason = "人工复核前的算法结论",
        PointCount = 3,
        Points = [[1, 2, 3]]
    };

    [Fact]
    public void ReplayIsImmutable_AndOffsetEquivalent()
    {
        var input = Input();
        var first = _service.Ingest("n1", input);
        var again = _service.Ingest("n1", input with { MeasuredAt = input.MeasuredAt.ToOffset(TimeSpan.FromHours(8)) });
        Assert.Equal(first, again);
        Assert.Equal(409, Assert.Throws<LoadRadarException>(() => _service.Ingest("n1", input with { VolumeM3 = 1 })).StatusCode);
        Assert.Single(_repository.Measurements);
    }

    [Fact]
    public void HistoryDoesNotTransportPointArraysOrMutateStoredMeasurement()
    {
        _service.Ingest("n1", Input());
        Assert.Empty(Assert.Single(_service.History(null, null, null, null, 50)).Measurement.Points);
        Assert.Single(_service.Get("m1").Measurement.Points);
    }

    [Fact]
    public void HistoryTimeWindowUsesMeasurementTimeAndIncludesBothBoundaries()
    {
        var start = DateTimeOffset.Parse("2026-01-01T08:00:00+08:00");
        foreach (var minute in new[] { -1, 0, 1, 2, 3 })
            _service.Ingest("n1", Input("m" + (minute + 1)) with { MeasuredAt = start.AddMinutes(minute) });
        var rows = _service.History(null, null, null, null, 50, start.ToUniversalTime(), start.AddMinutes(2));
        Assert.Equal(new[] { "m1", "m2", "m3" }, rows.Select(r => r.Measurement.MeasurementId));
        Assert.All(rows, row => Assert.Empty(row.Measurement.Points));
        Assert.Equal(4, _service.History(null, null, null, null, 50, start).Count);
        Assert.Equal(2, _service.History(null, null, null, null, 50, to: start).Count);
        Assert.Equal(400, Assert.Throws<LoadRadarException>(() =>
            _service.History(null, null, null, null, 50, start.AddMinutes(1), start)).StatusCode);
    }

    [Fact]
    public void NodeAndLaneAreServerControlled()
    {
        Assert.Equal(403, Assert.Throws<LoadRadarException>(() => _service.Ingest("n2", Input())).StatusCode);
        Assert.Throws<LoadRadarException>(() => _service.Ingest("n1", Input() with { ChannelId = "CH1" }));
        _options.Enabled = false;
        Assert.Equal(503, Assert.Throws<LoadRadarException>(() => _service.Ingest("n1", Input())).StatusCode);
    }

    [Fact]
    public void BindingUsesExactSuccessfulWorkflow_NoNearbyLookup()
    {
        _service.Ingest("n1", Input());
        var binding = _service.Bind("m1", new("workflow-exact", "宁A25771/挂车001", "现场核对车辆挂车"), "admin");
        Assert.Equal("宁A25771", binding.LicensePlate);
        Assert.Equal("workflow-exact", _workflow.LastWorkflowId);
        _workflow.Plate = null;
        Assert.Equal(409, Assert.Throws<LoadRadarException>(() =>
            _service.Bind("m1", new("pending", "挂车001", "核对"), "admin")).StatusCode);
        _workflow.Plate = "宁A25771";
        _workflow.Camera = "CAM999";
        Assert.Throws<LoadRadarException>(() => _service.Bind("m1", new("wrong-lane", "挂车001", "核对"), "admin"));
    }

    [Fact]
    public async Task BaselineRequiresOwnAssetExplicitConfirmation_AndStableVersion()
    {
        var hash = await UploadAsset();
        _service.Ingest("n1", Input() with { BaselineCandidateSha256 = hash });
        _service.Bind("m1", new("workflow-exact", "挂车001", "已核对挂车"), "admin");
        Assert.Throws<LoadRadarException>(() => _service.CreateBaseline(new("m1", hash, false, "尚未确认"), "admin"));
        Assert.Throws<LoadRadarException>(() => _service.CreateBaseline(new("m1", new string('b', 64), true, "不属于本次测量"), "admin"));
        var baseline = _service.CreateBaseline(new("m1", hash, true, "现场确认空车"), "admin");
        Assert.Equal(baseline.VersionId, _service.CreateBaseline(new("m1", hash, true, "重试"), "admin").VersionId);
        Assert.False(baseline.Active);
        Assert.Single(_repository.Versions);
    }

    [Fact]
    public async Task PublishPreservesPrecisionAndUsesExactWorkflow_WithoutFakeConfidence()
    {
        var row = await ReadyLoaded();
        var result = _service.Publish(row.Measurement.MeasurementId, new("核对基线及载货体积"), "admin");
        Assert.Equal("loaded-workflow", result.WorkflowId);
        Assert.Equal(12.4m, _repository.Coal!.Volume);
        Assert.Null(_repository.Coal.Confidence);
        Assert.Equal(DateTimeKind.Unspecified, _repository.Coal.MeasureTime.Kind);
        Assert.Equal(12.35m, _service.Get("loaded").Measurement.VolumeM3);
        Assert.Equal(result, _service.Publish("loaded", new("响应丢失后重试"), "admin"));
        Assert.Equal(1, _repository.PublishCount);
    }

    [Theory]
    [InlineData("demo", false, "success", "loaded", true)]
    [InlineData("live", false, "unavailable", "loaded", false)]
    [InlineData("live", false, "success", "empty", true)]
    public async Task UnsafeMeasurementCannotPublish(string mode, bool degraded, string status, string verdict, bool hasVolume)
    {
        var row = await ReadyLoaded();
        _repository.Measurements["loaded"] = row with
        {
            Measurement = row.Measurement with
            {
                SourceMode = mode,
                FrameDegraded = degraded,
                VolumeStatus = status,
                Verdict = verdict,
                VolumeM3 = hasVolume ? 3 : null
            }
        };
        Assert.Equal(409, Assert.Throws<LoadRadarException>(() => _service.Publish("loaded", new("审核"), "admin")).StatusCode);
        Assert.Equal(0, _repository.PublishCount);
    }

    [Theory]
    [InlineData("vehicle")]
    [InlineData("calibration")]
    [InlineData("activation-time")]
    public async Task BaselineIdentityAndActivationTimeMustMatch(string mismatch)
    {
        var row = await ReadyLoaded();
        var baseline = _repository.Versions[row.Measurement.BaselineVersionId!];
        _repository.Versions[baseline.VersionId] = mismatch switch
        {
            "vehicle" => baseline with { VehicleIdentity = "另一辆挂车" },
            "calibration" => baseline with { CalibrationId = "changed" },
            _ => baseline with { ActivatedAt = row.Measurement.MeasuredAt.AddSeconds(1) }
        };
        Assert.Throws<LoadRadarException>(() => _service.Publish("loaded", new("审核"), "admin"));
    }

    [Theory]
    [InlineData(null, 0.01)]
    [InlineData(0.05, null)]
    [InlineData(0.16, 0.01)]
    [InlineData(0.05, 0.031)]
    public async Task DiagnosticMetricsDoNotBlockComputedVolume(double? hole, double? spread)
    {
        var row = await ReadyLoaded();
        _repository.Measurements["loaded"] = row with
        {
            Measurement = row.Measurement with
            {
                HoleRatio = (decimal?)hole,
                BoundSpread = (decimal?)spread,
                FrameDegraded = true,
                VolumeStatus = "low_confidence"
            }
        };
        _service.Publish("loaded", new("自动发布"), "admin");
        Assert.Equal(1, _repository.PublishCount);
    }

    [Theory]
    [InlineData("0.0300000000000000000000000001", false)]
    [InlineData("0.030000000000000002", false)]
    [InlineData("0.1500000000000000000000000001", true)]
    public async Task DiagnosticThresholdDoesNotAffectPublication(string raw, bool isHole)
    {
        var row = await ReadyLoaded();
        var number = System.Text.Json.JsonSerializer.Deserialize<LoadRadarNumber>(raw);
        var input = isHole ? row.Measurement with { HoleRatio = number } : row.Measurement with { BoundSpread = number };
        _repository.Measurements["loaded"] = row with { Measurement = input };
        _service.Publish("loaded", new("自动发布"), "admin");
        Assert.Equal(1, _repository.PublishCount);
    }

    [Fact]
    public async Task HistoricalQualityFlagPublishesAutomaticallyWithoutReview()
    {
        var row = await ReadyLoaded();
        _repository.Measurements["loaded"] = row with
        {
            Measurement = row.Measurement with
            { VolumeStatus = "low_confidence", HoleRatio = 0.9m, BoundSpread = null, FrameDegraded = true }
        };
        new LoadRadarPublicationProcessor(_repository, _service).RunBatch(DateTimeOffset.UtcNow, default);
        Assert.Equal(1, _repository.PublishCount);
        Assert.Equal("published", _repository.Jobs["loaded"].Status);
        // 再启动处理器不会重复写煤量，原诊断载荷保持不变。
        new LoadRadarPublicationProcessor(_repository, _service).RunBatch(DateTimeOffset.UtcNow, default);
        Assert.Equal(1, _repository.PublishCount);
        Assert.Equal("low_confidence", _repository.Measurements["loaded"].Measurement.VolumeStatus);
    }

    [Theory]
    [InlineData("45.14999999999999", "45.1")]
    [InlineData("45.15", "45.2")]
    [InlineData("45.150000000000006", "45.2")]
    public async Task PublicationUsesRoundTripPrecisionWithoutChangingOriginal(string raw, string expected)
    {
        var row = await ReadyLoaded();
        var number = System.Text.Json.JsonSerializer.Deserialize<LoadRadarNumber>(raw);
        _repository.Measurements["loaded"] = row with { Measurement = row.Measurement with { VolumeM3 = number } };
        _service.Publish("loaded", new("审核"), "admin");
        Assert.Equal(decimal.Parse(expected, System.Globalization.CultureInfo.InvariantCulture), _repository.Coal!.Volume);
        Assert.Equal(raw, System.Text.Json.JsonSerializer.Serialize(_service.Get("loaded").Measurement.VolumeM3));
    }

    [Fact]
    public void ValidationRejectsUnknownsAndNonFinitePoints()
    {
        foreach (var invalid in new[]
        {
            Input() with { SourceMode = "" }, Input() with { ConfigurationHash = "" },
            Input() with { VolumeStatus = "success", VolumeM3 = null }, Input() with { VolumeM3 = -1 },
            Input() with { Points = [[float.NaN, 1, 2]] }, Input() with { Points = [[1, 2]] },
            Input() with { MeasuredAt = DateTimeOffset.UtcNow.AddDays(1) }, Input() with { HoleRatio = 1.1m }
        }) Assert.Throws<LoadRadarException>(() => LoadRadarService.ValidateInput(invalid));
    }

    [Fact]
    public async Task LegacyCoalIngressCannotWriteMigratedLane()
    {
        var configuration = new ConfigurationBuilder().AddInMemoryCollection(new Dictionary<string, string?>
        {
            ["LoadRadar:Enabled"] = "true",
            ["LoadRadar:Nodes:0:Channels:0:CameraIdentifier"] = "CAM010"
        }).Build();
        var legacy = new IntegrationQueryService(_workflow, configuration);
        var error = await Assert.ThrowsAsync<LoadRadarException>(() => legacy.IngestCoalVolumeAsync(new()
        {
            IdempotencyKey = "legacy",
            CameraIdentifier = "CAM_010",
            LicensePlate = "宁A25771",
            Volume = 10,
            Unit = "m³",
            Status = "success"
        }));
        Assert.Equal(409, error.StatusCode);
    }

    private async Task<string> UploadAsset()
    {
        using var body = new MemoryStream("baseline-npz"u8.ToArray());
        var hash = Convert.ToHexString(SHA256.HashData(body.ToArray())).ToLowerInvariant();
        body.Position = 0;
        await _assets.Upload(hash, body, default);
        return hash;
    }

    [Fact]
    public async Task AutomaticPublicationUsesExistingPublishBoundaryAndOriginalMeasurement()
    {
        var row = await ReadyLoaded();
        var processor = new LoadRadarPublicationProcessor(_repository, _service);
        Assert.Equal(1, processor.RunBatch(DateTimeOffset.UtcNow, default));
        Assert.Equal("published", _repository.Jobs["loaded"].Status);
        Assert.Equal("system:load-radar", _service.Get("loaded").Publication!.Actor);
        Assert.Equal(12.4m, _repository.Coal!.Volume);
        Assert.Equal(row.Measurement, _service.Get("loaded").Measurement);
        Assert.Equal(0, processor.RunBatch(DateTimeOffset.UtcNow.AddHours(1), default));
        Assert.Equal(1, _repository.PublishCount);
        // 原始空车和历史同 ID 重放不产生新任务。
        _service.Ingest("n1", row.Measurement);
        Assert.Single(_repository.Jobs);
        Assert.Equal("published", _repository.Jobs["loaded"].Status);
    }

    [Fact]
    public async Task AutomaticPublicationWaitsForExactIdentityWithoutGuessing()
    {
        var row = await ReadyLoaded();
        _repository.Measurements["loaded"] = row with { Binding = null };
        var now = DateTimeOffset.UtcNow;
        var processor = new LoadRadarPublicationProcessor(_repository, _service);
        processor.RunBatch(now, default);
        Assert.Equal("awaiting_identity", _repository.Jobs["loaded"].Reason);
        Assert.Equal(0, _repository.PublishCount);
        Assert.Equal(0, processor.RunBatch(now.AddSeconds(4), default));
        _repository.Measurements["loaded"] = row;
        // 新处理器模拟重启：任务来自仓储，不依赖上个实例的内存。
        new LoadRadarPublicationProcessor(_repository, _service).RunBatch(now.AddSeconds(5), default);
        Assert.Equal(1, _repository.PublishCount);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task AutomaticPublicationRecoversBeforeAndAfterCommitFailure(bool afterCommit)
    {
        await ReadyLoaded();
        _repository.FailPublishBeforeCommit = !afterCommit;
        _repository.FailPublishAfterCommit = afterCommit;
        var now = DateTimeOffset.UtcNow;
        var processor = new LoadRadarPublicationProcessor(_repository, _service);
        processor.RunBatch(now, default);
        Assert.Equal("pending", _repository.Jobs["loaded"].Status);
        Assert.Equal("retry_storage", _repository.Jobs["loaded"].Reason);
        new LoadRadarPublicationProcessor(_repository, _service).RunBatch(now.AddSeconds(5), default);
        Assert.Equal("published", _repository.Jobs["loaded"].Status);
        Assert.Equal(1, _repository.PublishCount);
    }

    [Fact]
    public async Task AutomaticPublicationRecoversWhenJobAcknowledgementFails()
    {
        await ReadyLoaded();
        _repository.FailOutcome = true;
        var processor = new LoadRadarPublicationProcessor(_repository, _service);
        Assert.Throws<IOException>(() => processor.RunBatch(DateTimeOffset.UtcNow, default));
        Assert.Equal(1, _repository.PublishCount);
        Assert.Equal("pending", _repository.Jobs["loaded"].Status);
        processor.RunBatch(DateTimeOffset.UtcNow, default);
        Assert.Equal(1, _repository.PublishCount);
        Assert.Equal("published", _repository.Jobs["loaded"].Status);
    }

    [Theory]
    [InlineData("plate")]
    [InlineData("trailer")]
    [InlineData("camera")]
    [InlineData("missing_volume")]
    [InlineData("configuration")]
    [InlineData("baseline")]
    public async Task AutomaticPublicationCannotBypassValidation(string invalid)
    {
        var row = await ReadyLoaded();
        _repository.Measurements["loaded"] = invalid switch
        {
            "plate" => row with { Binding = row.Binding! with { LicensePlate = "宁B08669" } },
            "trailer" => row with { Binding = row.Binding! with { VehicleIdentity = "其他挂车" } },
            "camera" => row with { Binding = row.Binding! with { CameraIdentifier = "CAM999" } },
            "missing_volume" => row with { Measurement = row.Measurement with { VolumeM3 = null } },
            "configuration" => row with { Measurement = row.Measurement with { ConfigurationHash = new string('d', 64) } },
            _ => row with { Measurement = row.Measurement with { BaselineVersionId = "missing" } }
        };
        new LoadRadarPublicationProcessor(_repository, _service).RunBatch(DateTimeOffset.UtcNow, default);
        Assert.Equal("rejected", _repository.Jobs["loaded"].Status);
        Assert.Equal(0, _repository.PublishCount);
    }

    [Fact]
    public async Task AutomaticPublicationHonorsDisabledModuleAndCancellation()
    {
        await ReadyLoaded();
        var processor = new LoadRadarPublicationProcessor(_repository, _service);
        _options.Enabled = false;
        Assert.Equal(503, Assert.Throws<LoadRadarException>(() => processor.RunBatch(DateTimeOffset.UtcNow, default)).StatusCode);
        _options.Enabled = true;
        Assert.Throws<OperationCanceledException>(() => processor.RunBatch(DateTimeOffset.UtcNow, new CancellationToken(true)));
        Assert.Equal(0, _repository.PublishCount);
        Assert.Equal(0, _repository.Jobs["loaded"].Attempts);
    }

    [Fact]
    public async Task AutomaticPublicationWaitsForOriginalAssetAndPublishesAfterRestore()
    {
        var row = await ReadyLoaded();
        var baseline = _repository.GetBaseline(row.Measurement.BaselineVersionId!)!;
        var path = _assets.PathFor(baseline.AssetSha256);
        // 仅移动本测试独占目录内的资产，不接触真实资产盘。
        var held = path + ".held";
        File.Move(path, held);
        var now = DateTimeOffset.UtcNow;
        var processor = new LoadRadarPublicationProcessor(_repository, _service);
        processor.RunBatch(now, default);
        Assert.Equal("pending", _repository.Jobs["loaded"].Status);
        Assert.Equal(0, _repository.PublishCount);
        File.Move(held, path);
        processor.RunBatch(now.AddSeconds(5), default);
        Assert.Equal("published", _repository.Jobs["loaded"].Status);
        Assert.Equal(1, _repository.PublishCount);
        Assert.Equal(row.Measurement, _service.Get("loaded").Measurement);
    }

    [Theory]
    [InlineData("CH1", "CAM_001")]
    [InlineData("CH2", "CAM_002")]
    [InlineData("CH3", "CAM_003")]
    [InlineData("CH4", "CAM_004")]
    public async Task FourLanesUseConfirmedCameraForBindingAndCoalStorage(string channel, string camera)
    {
        var example = new ConfigurationBuilder().AddJsonFile(ExamplePath()).Build()
            .GetSection("LoadRadar").Get<LoadRadarOptions>()!;
        Assert.False(example.Enabled);
        Assert.Empty(example.Nodes[0].AgentKey);
        _options.Nodes[0].Channels = example.Nodes[0].Channels;
        _workflow.Camera = "CAM_004";
        var hash = await UploadAsset();
        _service.Ingest("n1", Input() with { ChannelId = "CH4", BaselineCandidateSha256 = hash });
        var binding = _service.Bind("m1", new("empty-workflow", "挂车001", "测试已确认身份"), "test");
        Assert.Equal("CAM_004", binding.CameraIdentifier);
        var baseline = _service.CreateBaseline(new("m1", hash, true, "测试确认空车"), "test");
        _service.Activate(baseline.VersionId, new(null, "测试激活"), "test");
        _service.Ingest("n1", Input("loaded") with
        {
            ChannelId = channel,
            PassageId = "p2",
            Verdict = "loaded",
            VolumeStatus = "success",
            VolumeM3 = 12.35m,
            HoleRatio = 0.05m,
            BoundSpread = 0.02m,
            BaselineVersionId = baseline.VersionId,
            MeasuredAt = DateTimeOffset.UtcNow,
            BaselineUse = SharedUse()
        });
        _workflow.Camera = camera == "CAM_004" ? "CAM_001" : "CAM_004";
        Assert.Equal(409, Assert.Throws<LoadRadarException>(() =>
            _service.Bind("loaded", new("loaded-workflow", "挂车001", "测试串道拒绝"), "test")).StatusCode);
        _workflow.Camera = camera;
        _service.Bind("loaded", new("loaded-workflow", "挂车001", "测试已确认身份"), "test");
        new LoadRadarPublicationProcessor(_repository, _service).RunBatch(DateTimeOffset.UtcNow, default);
        Assert.Equal(camera, _repository.Coal!.CameraIdentifier);
        Assert.Equal("宁A25771", _repository.Coal.LicensePlate);
        Assert.Equal("loaded-workflow", _repository.Coal.WorkflowId);
        Assert.Equal(12.4m, _repository.Coal.Volume);
        Assert.Equal(1, _repository.PublishCount);
    }

    private static string ExamplePath([CallerFilePath] string source = "") =>
        Path.GetFullPath(Path.Combine(Path.GetDirectoryName(source)!, "../../config/load-radar.example.json"));

    [Fact]
    public async Task WaitingIdentityBatchDoesNotStarveNextReadyMeasurement()
    {
        var row = await ReadyLoaded();
        var now = DateTimeOffset.UtcNow;
        for (var i = 0; i < 16; i++)
        {
            var id = "waiting-" + i;
            _repository.Measurements.Add(id, row with
            { Binding = null, Measurement = row.Measurement with { MeasurementId = id } });
            _repository.Jobs.Add(id, new("pending", "awaiting_processing", 0, now.AddMinutes(-1)));
        }
        var processor = new LoadRadarPublicationProcessor(_repository, _service);
        Assert.Equal(16, processor.RunBatch(now, default));
        Assert.Equal(0, _repository.PublishCount);
        Assert.Equal(1, processor.RunBatch(now, default));
        Assert.Equal("published", _repository.Jobs["loaded"].Status);
        Assert.Equal(1, _repository.PublishCount);
    }

    [Theory]
    [InlineData("demo")]
    [InlineData("replay")]
    [InlineData("degraded")]
    [InlineData("unknown")]
    [InlineData("low_confidence")]
    [InlineData("null_volume")]
    [InlineData("null_hole")]
    [InlineData("high_hole")]
    [InlineData("null_spread")]
    [InlineData("high_spread")]
    [InlineData("no_baseline")]
    public async Task AutomaticPublicationCandidateDoesNotAuthorizeInvalidMeasurements(string invalid)
    {
        // 这条用例的语义是"**配了** 3% 门槛时，下面这些都要被拒"，所以显式传门槛。
        // 门槛本身默认已改成不设（null），另见 LoadRadarBoundSpreadGateTests。
        var good = (await ReadyLoaded()).Measurement;
        Assert.True(LoadRadarPublicationPolicy.IsCandidate(good with { VolumeM3 = 0, HoleRatio = 0.15m, BoundSpread = 0.03m }, 0.03m));
        var bad = invalid switch
        {
            "demo" or "replay" => good with { SourceMode = invalid },
            "degraded" => good with { FrameDegraded = true },
            "unknown" => good with { Verdict = "unknown" },
            "low_confidence" => good with { VolumeStatus = "low_confidence" },
            "null_volume" => good with { VolumeM3 = null },
            "null_hole" => good with { HoleRatio = null },
            "high_hole" => good with { HoleRatio = 0.150001m },
            "null_spread" => good with { BoundSpread = null },
            "high_spread" => good with { BoundSpread = 0.030001m },
            _ => good with { BaselineVersionId = null }
        };
        Assert.Equal(invalid is "degraded" or "low_confidence" or "null_hole" or "high_hole" or "null_spread" or "high_spread",
            LoadRadarPublicationPolicy.IsCandidate(bad, 0.03m));
    }

    private async Task<LoadMeasurementRecord> ReadyLoaded()
    {
        var hash = await UploadAsset();
        _service.Ingest("n1", Input() with { BaselineCandidateSha256 = hash });
        _service.Bind("m1", new("empty-workflow", "挂车001", "确认空车身份"), "admin");
        var baseline = _service.CreateBaseline(new("m1", hash, true, "确认空车"), "admin");
        _service.Activate(baseline.VersionId, new(null, "启用基线"), "admin");
        _service.Ingest("n1", Input("loaded") with
        {
            PassageId = "p2",
            Verdict = "loaded",
            VolumeStatus = "success",
            VolumeM3 = 12.35m,
            HoleRatio = 0.05m,
            BoundSpread = 0.02m,
            BaselineVersionId = baseline.VersionId,
            MeasuredAt = DateTimeOffset.UtcNow
        });
        _service.Bind("loaded", new("loaded-workflow", "挂车001", "同一车辆挂车"), "admin");
        return _service.Get("loaded");
    }

    private LoadRadarPassageService PassageService(IRadarLicensePlateSource? plates = null) =>
        new(_repository, _workflow, _service, _assets, plates ?? _plates, new ConfigurationBuilder().Build());

    private sealed class ScanWorker(HttpStatusCode status, string body) : HttpMessageHandler, IHttpClientFactory
    {
        public readonly List<string> Paths = [];
        public HttpClient CreateClient(string name) => new(this, false) { BaseAddress = new Uri("http://127.0.0.1:8000") };
        protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
        {
            Paths.Add(request.RequestUri!.PathAndQuery);
            return Task.FromResult(new HttpResponseMessage(status) { Content = new StringContent(body) });
        }
    }

    private static ScanWorker AcceptingWorker(string workflowId) => new(HttpStatusCode.Accepted,
        "{\"success\":true,\"code\":\"scanning\",\"started\":true,\"idempotent\":false," +
        "\"scan\":{\"requestId\":\"" + workflowId + "\"}}");

    private LoadRadarStandstillService StandstillService(ScanWorker worker, string? dimensionChannel = "CH4") =>
        new(_repository, _workflow, _service,
            new RadarLiveClient(worker),
            new ConfigurationBuilder().AddInMemoryCollection(new Dictionary<string, string?>
            { ["LoadRadar:DimensionScanChannelId"] = dimensionChannel }).Build(),
            Microsoft.Extensions.Logging.Abstractions.NullLogger<LoadRadarStandstillService>.Instance);

    [Fact]
    public async Task StandstillDispatchesOnceWithOriginalWorkflowAndKeepsVolumeChain()
    {
        var arrival = Arrival("standstill-p1");
        var prepared = PassageService().Prepare("n1", arrival);
        var worker = AcceptingWorker(prepared.WorkflowId);
        var standstill = StandstillService(worker);

        var first = await standstill.ReportAsync("n1", new("n1", "CH4", "standstill-p1", arrival.ArrivedAt));
        Assert.Equal("dispatched", first.Status);
        Assert.Equal(prepared.WorkflowId, first.WorkflowId);
        Assert.Equal(VehicleArrivalRadarStatus.Succeeded, _workflow.Arrivals[prepared.WorkflowId].RadarStatus);
        Assert.StartsWith($"/api/radars/scan-once?requestId={Uri.EscapeDataString(prepared.WorkflowId)}&captureUntil=",
            Assert.Single(worker.Paths));
        Assert.Equal(arrival.ArrivedAt.AddSeconds(10),
            DateTimeOffset.Parse(Uri.UnescapeDataString(worker.Paths[0].Split("&captureUntil=")[1])));

        // 重复通知不再扫描
        Assert.Equal("already", (await standstill.ReportAsync("n1", new("n1", "CH4", "standstill-p1", arrival.ArrivedAt))).Status);
        Assert.Single(worker.Paths);
        // 已触发扫描的原车次，JT128 到车登记照常（体积链不受影响）
        Assert.Equal(prepared.WorkflowId, PassageService().Prepare("n1", arrival).WorkflowId);
    }

    /// <summary>
    /// 生产路径：到车时牌未就绪（awaiting_plate / PENDING），停稳约 10s 后第三方才回牌。
    /// 停稳派发不得等牌；必须用原 workflowId 调 scan-once，且不因无牌丢派发。
    /// </summary>
    [Fact]
    public async Task StandstillDispatchesWhilePlateStillPendingUsingOriginalWorkflow()
    {
        var arrival = Arrival("standstill-pending-plate");
        var prepared = PassageService(NoPlate()).Prepare("n1", arrival);
        Assert.Equal("awaiting_plate", prepared.Status);
        Assert.Equal(VehicleArrivalPlateStatus.Pending, _workflow.Arrivals[prepared.WorkflowId].PlateStatus);
        Assert.Null(_workflow.Arrivals[prepared.WorkflowId].LicensePlate);

        var worker = AcceptingWorker(prepared.WorkflowId);
        var first = await StandstillService(worker).ReportAsync("n1", new("n1", "CH4", "standstill-pending-plate", arrival.ArrivedAt));
        Assert.Equal("dispatched", first.Status);
        Assert.Equal(prepared.WorkflowId, first.WorkflowId);
        Assert.Equal(VehicleArrivalRadarStatus.Succeeded, _workflow.Arrivals[prepared.WorkflowId].RadarStatus);
        Assert.StartsWith($"/api/radars/scan-once?requestId={Uri.EscapeDataString(prepared.WorkflowId)}&captureUntil=",
            Assert.Single(worker.Paths));
        // 派发后牌仍可能 PENDING——绑定靠 workflowId，不靠即时车牌。
        Assert.Equal(VehicleArrivalPlateStatus.Pending, _workflow.Arrivals[prepared.WorkflowId].PlateStatus);
        Assert.Equal("already", (await StandstillService(worker).ReportAsync("n1",
            new("n1", "CH4", "standstill-pending-plate", arrival.ArrivedAt))).Status);
        Assert.Single(worker.Paths);
    }

    [Fact]
    public async Task DepartureClosesOnlyOriginalWorkflowAndOldStandstillIsNotDispatched()
    {
        var arrival = Arrival("standstill-old") with { ArrivedAt = DateTimeOffset.UtcNow.AddSeconds(-20) };
        var prepared = PassageService().Prepare("n1", arrival);
        var worker = AcceptingWorker(prepared.WorkflowId);
        var standstill = StandstillService(worker);

        var closed = await standstill.DepartAsync("n1", new("n1", "CH4", arrival.PassageId));
        Assert.Equal("closed", closed.Status);
        Assert.Equal($"/api/radars/scan-once/passage-ended?requestId={Uri.EscapeDataString(prepared.WorkflowId)}",
            Assert.Single(worker.Paths));
        var late = await standstill.ReportAsync("n1", new("n1", "CH4", arrival.PassageId, arrival.ArrivedAt));
        Assert.Equal("expired", late.Status);
        Assert.Single(worker.Paths);
        Assert.Equal(VehicleArrivalRadarStatus.NotRequired, _workflow.Arrivals[prepared.WorkflowId].RadarStatus);
    }

    [Theory]
    [InlineData(null)]
    [InlineData("")]
    [InlineData("CH1")]
    public async Task StandstillWithoutMatchingConfigurationDoesNothing(string? dimensionChannel)
    {
        var prepared = PassageService().Prepare("n1", Arrival("standstill-off"));
        var worker = AcceptingWorker(prepared.WorkflowId);
        var receipt = await StandstillService(worker, dimensionChannel).ReportAsync("n1", new("n1", "CH4", "standstill-off"));
        Assert.Equal("not_required", receipt.Status);
        Assert.Empty(worker.Paths);
        Assert.Equal(VehicleArrivalRadarStatus.NotRequired, _workflow.Arrivals[prepared.WorkflowId].RadarStatus);
    }

    [Fact]
    public async Task StandstillForUnregisteredPassageIsRejected()
    {
        var worker = AcceptingWorker("none");
        var error = await Assert.ThrowsAsync<LoadRadarException>(() =>
            StandstillService(worker).ReportAsync("n1", new("n1", "CH4", "never-registered")));
        Assert.Equal(409, error.StatusCode);
        Assert.Empty(worker.Paths);
    }

    private static LoadBaselineUse SharedUse(string status = "baseline_fallback") =>
        new("CH4", "cal1", new string('c', 64), "ch4-shared-v1", status,
            "测试：车厢特征不足，未解分量使用CH4基准", 0, 0, 0, 0, null);

    /// <summary>取牌来源不可用（例如生产环境未开模拟车牌）。</summary>
    private static PlateSource NoPlate() => new() { Plate = null };

    private static LoadPassageStart Arrival(string passage = "automatic-p1") =>
        new("n1", "CH4", passage, "cal1", new string('c', 64), DateTimeOffset.UtcNow, "live");

    [Fact]
    public async Task ArrivalPreparesIdentityAndBaselineBeforeFirstLoadedMeasurement()
    {
        var service = PassageService();
        var empty = Arrival();
        var first = service.Prepare("n1", empty);
        // 到车当场从 /api/v1/radar/licensePlate 同一来源取牌：没有 awaiting_plate 这一拍，MOCK 车牌照用。
        Assert.Equal("baseline_required", first.Status);
        Assert.Equal("plate:宁A25771", first.Binding!.VehicleIdentity);
        Assert.Empty(_repository.Measurements);
        Assert.Single(_workflow.Arrivals);
        var arrival = _workflow.Arrivals[first.WorkflowId];
        Assert.Equal(VehicleArrivalRadarStatus.NotRequired, arrival.RadarStatus);
        Assert.Equal(VehicleArrivalPlateStatus.Succeeded, arrival.PlateStatus);
        Assert.Equal("宁A25771", arrival.LicensePlate);
        Assert.Equal("MOCK", arrival.PlateSource);
        Assert.Equal(1, _plates.Calls);
        var bound = service.Prepare("n1", empty);
        Assert.Equal(first.Binding, bound.Binding);
        Assert.Equal(bound.Binding, service.Prepare("n1", empty).Binding);
        // 同一车次重放沿用原来车记录、不再取牌；来源之后不可用也不影响已绑定的车次。
        Assert.Equal(bound.Binding, PassageService(NoPlate()).Prepare("n1", empty).Binding);
        Assert.Equal(1, _plates.Calls);
        var hash = await UploadAsset();
        _service.Ingest("n1", Input("auto-empty") with
        { PassageId = empty.PassageId, MeasuredAt = DateTimeOffset.UtcNow, BaselineCandidateSha256 = hash });
        var baseline = _service.CreateBaseline(new("auto-empty", hash, true, "维护确认空车"), "admin");
        _service.Activate(baseline.VersionId, new(null, "维护激活"), "admin");
        var loaded = Arrival("automatic-p2");
        var waiting = service.Prepare("n1", loaded);
        Assert.Equal("ready", waiting.Status);
        var ready = service.Prepare("n1", loaded);
        Assert.Equal("ready", ready.Status);
        Assert.Equal(baseline.VersionId, ready.Baseline!.VersionId);
        Assert.Single(_repository.Measurements); // 此时还没有任何载货测量。
        _service.Ingest("n1", Input("auto-loaded") with
        {
            PassageId = loaded.PassageId,
            MeasuredAt = DateTimeOffset.UtcNow,
            Verdict = "loaded",
            VolumeStatus = "success",
            VolumeM3 = 12.35m,
            HoleRatio = 0.05m,
            BoundSpread = 0.02m,
            BaselineVersionId = ready.Baseline.VersionId
        });
        new LoadRadarPublicationProcessor(_repository, _service).RunBatch(DateTimeOffset.UtcNow, default);
        Assert.Equal(waiting.WorkflowId, _repository.Coal!.WorkflowId);
        Assert.Equal(12.4m, _repository.Coal.Volume);
        Assert.NotNull(_service.Get("auto-loaded").Publication);
        Assert.Equal(2, _service.History(null, null, null, null, 50).Count);
        Assert.Equal(2, _workflow.Arrivals.Count);
        var changedConfiguration = Arrival("automatic-p3") with { ConfigurationHash = new string('d', 64) };
        Assert.Equal("ready", service.Prepare("n1", changedConfiguration).Status);
    }

    [Fact]
    public void ArrivalRetriesRecoverOriginalWorkflowAndRejectChangedContext()
    {
        var input = Arrival();
        _workflow.FailAfterArrival = true;
        Assert.Throws<IOException>(() => PassageService().Prepare("n1", input));
        var replay = PassageService().Prepare("n1", input);
        Assert.Single(_workflow.Arrivals);
        Assert.Single(_repository.Starts);
        Assert.Equal(replay.WorkflowId, PassageService().Prepare("n1", input with
        { ArrivedAt = input.ArrivedAt.ToOffset(TimeSpan.FromHours(8)) }).WorkflowId);
        Assert.Equal(409, Assert.Throws<LoadRadarException>(() => PassageService().Prepare("n1", input with
        { ConfigurationHash = new string('d', 64) })).StatusCode);
        Assert.Equal(403, Assert.Throws<LoadRadarException>(() => PassageService().Prepare("other", input)).StatusCode);
    }

    [Theory]
    [InlineData("CH1", "CAM_001")]
    [InlineData("CH2", "CAM_002")]
    [InlineData("CH3", "CAM_003")]
    [InlineData("CH4", "CAM_004")]
    public async Task AutomaticArrivalThroughMachineHttpKeepsExactLaneAndPublishesOnce(string channel, string camera)
    {
        _options.Nodes[0].Channels = Enumerable.Range(1, 4).Select(i => new LoadRadarLaneOptions
        { ChannelId = "CH" + i, CameraIdentifier = "CAM_00" + i, Name = "CH" + i }).ToList();
        _workflow.Camera = camera;
        using var server = new TestServer(new WebHostBuilder().ConfigureServices(services =>
        {
            services.AddSingleton(Options.Create(_options));
            services.AddSingleton(_service);
            services.AddSingleton(_assets);
            services.AddSingleton(PassageService());
            services.AddControllers().AddApplicationPart(typeof(LoadRadarAgentController).Assembly);
        }).Configure(app =>
        {
            app.UseMiddleware<ApiExceptionMiddleware>();
            app.UseMiddleware<LoadRadarAgentAuthMiddleware>();
            app.UseRouting();
            app.UseEndpoints(endpoints => endpoints.MapControllers());
        }));
        using var client = server.CreateClient();
        const string route = "/api/v1/load-radar-agent";
        var input = Arrival();
        Assert.Equal(HttpStatusCode.Unauthorized, (await client.PostAsJsonAsync(route + "/passages/prepare", input)).StatusCode);
        client.DefaultRequestHeaders.Add(LoadRadarAgentAuthMiddleware.NodeHeader, "n1");
        client.DefaultRequestHeaders.Add(LoadRadarAgentAuthMiddleware.KeyHeader, new string('a', 40));
        var crossNodeResponse = await client.PostAsJsonAsync(route + "/passages/prepare", input with { NodeId = "other" });
        Assert.True(crossNodeResponse.StatusCode == HttpStatusCode.Forbidden, await crossNodeResponse.Content.ReadAsStringAsync());
        async Task<LoadPassagePreparation> Prepare(LoadPassageStart start)
        {
            var response = await client.PostAsJsonAsync(route + "/passages/prepare", start);
            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
            using var json = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
            return json.RootElement.GetProperty("data").Deserialize<LoadPassagePreparation>(new JsonSerializerOptions(JsonSerializerDefaults.Web))!;
        }
        void Recognized(LoadPassagePreparation response)
        {
            // 到车当场按本车道点位取牌（与 /api/v1/radar/licensePlate 同一来源）。
            var workflow = _workflow.Arrivals[response.WorkflowId];
            Assert.Equal(response.CameraIdentifier, workflow.CameraIdentifier);
            Assert.Equal(VehicleArrivalRadarStatus.NotRequired, workflow.RadarStatus);
            Assert.Equal(VehicleArrivalPlateStatus.Succeeded, workflow.PlateStatus);
            Assert.Equal(response.CameraIdentifier, _plates.LastCamera);
        }
        var first = await Prepare(input);
        Assert.Equal(input, first.Start);
        Assert.Equal("baseline_required", first.Status);
        Recognized(first);
        Assert.Equal("baseline_required", (await Prepare(input)).Status);
        var hash = await UploadAsset();
        var empty = Input("http-empty") with
        {
            PassageId = input.PassageId,
            MeasuredAt = DateTimeOffset.UtcNow,
            BaselineCandidateSha256 = hash
        };
        Assert.Equal(HttpStatusCode.OK, (await client.PostAsJsonAsync(route + "/measurements", empty)).StatusCode);
        var baseline = _service.CreateBaseline(new(empty.MeasurementId, hash, true, "测试维护确认"), "test-admin");
        _service.Activate(baseline.VersionId, new(null, "测试维护激活"), "test-admin");
        var next = input with
        {
            ChannelId = channel,
            CalibrationId = "cal-" + channel,
            ConfigurationHash = new string('d', 64),
            PassageId = "http-loaded-passage",
            ArrivedAt = DateTimeOffset.UtcNow
        };
        var awaiting = await Prepare(next);
        Assert.Equal("ready", awaiting.Status);
        Recognized(awaiting);
        var prepared = await Prepare(next);
        Assert.Equal("ready", prepared.Status);
        Assert.Equal(camera, prepared.Binding!.CameraIdentifier);
        Assert.Single(_repository.Measurements); // 正式基线在载货首测前已返回。
        var loaded = Input("http-loaded") with
        {
            ChannelId = channel,
            PassageId = next.PassageId,
            CalibrationId = next.CalibrationId,
            ConfigurationHash = next.ConfigurationHash,
            BaselineUse = SharedUse(),
            MeasuredAt = DateTimeOffset.UtcNow,
            Verdict = "loaded",
            VolumeStatus = "success",
            VolumeM3 = 9.6m,
            HoleRatio = 0,
            BoundSpread = 0,
            BaselineVersionId = baseline.VersionId
        };
        for (var retry = 0; retry < 2; retry++)
            Assert.Equal(HttpStatusCode.OK, (await client.PostAsJsonAsync(route + "/measurements", loaded)).StatusCode);
        var processor = new LoadRadarPublicationProcessor(_repository, _service);
        processor.RunBatch(DateTimeOffset.UtcNow, default);
        processor.RunBatch(DateTimeOffset.UtcNow.AddMinutes(1), default);
        Assert.Equal(1, _repository.PublishCount);
        Assert.Equal(awaiting.WorkflowId, _repository.Coal!.WorkflowId);
        Assert.Equal(9.6m, _repository.Coal.Volume);
        Assert.NotNull(_service.History(null, null, null, null, 50).Single(r => r.Measurement.MeasurementId == loaded.MeasurementId).Publication);
        Assert.Equal(HttpStatusCode.Conflict, (await client.PostAsJsonAsync(route + "/passages/prepare",
            next with { ConfigurationHash = new string('e', 64) })).StatusCode);
        Assert.Equal(2, _workflow.Arrivals.Count);
    }

    [Theory]
    [InlineData("CH1")]
    [InlineData("CH2")]
    [InlineData("CH3")]
    [InlineData("CH4")]
    [InlineData("aligned")]
    public async Task PythonSharedBaselinePublishesAndReplaysThroughMachineHttp(string fixtureKey)
    {
        var channel = fixtureKey == "aligned" ? "CH1" : fixtureKey;
        var existing = await ReadyLoaded();
        var baseline = existing.Measurement.BaselineVersionId!;
        if (channel != "CH4") _options.Nodes[0].Channels.Add(new() { ChannelId = channel, Name = channel, CameraIdentifier = "CAM_" + channel });
        using var fixture = JsonDocument.Parse(File.ReadAllText(Path.Combine(SourceDirectory(), "Fixtures", "load-radar-ch4-shared-v1.json")));
        var input = fixture.RootElement.GetProperty(fixtureKey).Deserialize<LoadMeasurementInput>(new JsonSerializerOptions(JsonSerializerDefaults.Web))! with
        { BaselineVersionId = baseline, MeasuredAt = DateTimeOffset.UtcNow };
        _workflow.Camera = _options.Nodes[0].Channels.Single(c => c.ChannelId == channel).CameraIdentifier;
        using var server = new TestServer(new WebHostBuilder().ConfigureServices(services =>
        {
            services.AddSingleton(Options.Create(_options));
            services.AddSingleton(_service);
            services.AddSingleton(_assets);
            services.AddControllers().AddApplicationPart(typeof(LoadRadarAgentController).Assembly);
        }).Configure(app =>
        {
            app.UseMiddleware<ApiExceptionMiddleware>();
            app.UseMiddleware<LoadRadarAgentAuthMiddleware>();
            app.UseRouting();
            app.UseEndpoints(endpoints => endpoints.MapControllers());
        }));
        using var client = server.CreateClient();
        client.DefaultRequestHeaders.Add(LoadRadarAgentAuthMiddleware.NodeHeader, "n1");
        client.DefaultRequestHeaders.Add(LoadRadarAgentAuthMiddleware.KeyHeader, new string('a', 40));
        var response = await client.PostAsJsonAsync("/api/v1/load-radar-agent/measurements", input);
        Assert.Equal(HttpStatusCode.OK, response.StatusCode);
        using var reply = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
        var echoed = reply.RootElement.GetProperty("data").GetProperty("measurement");
        var request = JsonSerializer.SerializeToElement(input, new JsonSerializerOptions(JsonSerializerDefaults.Web));
        foreach (var field in request.EnumerateObject())
            Assert.True(System.Text.Json.Nodes.JsonNode.DeepEquals(System.Text.Json.Nodes.JsonNode.Parse(field.Value.GetRawText()),
                System.Text.Json.Nodes.JsonNode.Parse(echoed.GetProperty(field.Name).GetRawText())), field.Name);
        _service.Bind(input.MeasurementId, new("cross-" + channel, "plate:宁A25771", "同牌跨道共享"), "test");
        var restarted = new LoadRadarService(_repository, _workflow, Options.Create(_options), _assets);
        var first = restarted.Publish(input.MeasurementId, new("配准失败按基准发布"), "test");
        Assert.Equal(first, restarted.Publish(input.MeasurementId, new("原编号重试"), "test"));
        Assert.Equal(1, _repository.PublishCount);
        var stored = restarted.Get(input.MeasurementId).Measurement;
        Assert.Equal(channel, stored.ChannelId);
        Assert.Equal("CH4", stored.BaselineUse!.SourceChannelId);
        Assert.Equal(fixtureKey == "aligned" ? "resolved" : "baseline_fallback", stored.BaselineUse.RegistrationStatus);
        if (fixtureKey == "aligned") Assert.NotNull(stored.BaselineUse.Pose);
        Assert.Null(stored.BoundSpread);
        Assert.Equal("success", stored.VolumeStatus);
    }

    [Theory]
    [InlineData("CH1")]
    [InlineData("CH2")]
    [InlineData("CH3")]
    public async Task OnlyCh4CanCreateOrActivateBaseline(string channel)
    {
        _options.Nodes[0].Channels.Add(new() { ChannelId = channel, Name = channel, CameraIdentifier = "CAM010" });
        var hash = await UploadAsset();
        _service.Ingest("n1", Input() with { ChannelId = channel, BaselineCandidateSha256 = hash });
        _service.Bind("m1", new("wf-other", "vehicle", "隔离基线来源"), "test");
        Assert.Throws<LoadRadarException>(() => _service.CreateBaseline(new("m1", hash, true, "非CH4禁止"), "test"));
        var legacy = new LoadBaselineVersion("legacy", "scope", "n1", channel, "cal1", "vehicle", "宁A25771",
            "m1", hash, "test", "旧车道基线", DateTimeOffset.UtcNow);
        _repository.Versions.Add(legacy.VersionId, legacy);
        Assert.Throws<LoadRadarException>(() => _service.Activate(legacy.VersionId, new(null, "非CH4禁止"), "test"));
    }

    [Fact]
    public async Task BaselineReplacementAndReactivationRespectOriginalTimeAcrossRestart()
    {
        var row = await ReadyLoaded();
        var first = _repository.Versions[row.Measurement.BaselineVersionId!];
        // 模拟升级前按标定分域的CH4基线，激活审计仍需被新策略识别。
        _repository.Versions[first.VersionId] = first with { ScopeKey = new string('a', 64) };
        var retried = _service.CreateBaseline(new(first.MeasurementId, first.AssetSha256, true, "升级后重试原登记"), "test");
        Assert.Equal(new string('a', 64), retried.ScopeKey);
        var oldTime = row.Measurement.MeasuredAt;
        _service.Ingest("n1", Input("empty-v2") with
        {
            PassageId = "empty-p2",
            CalibrationId = "cal-v2",
            ConfigurationHash = new string('e', 64),
            BaselineCandidateSha256 = first.AssetSha256
        });
        _service.Bind("empty-v2", new("empty-workflow-v2", "plate:宁A25771", "同牌新空车"), "test");
        var second = _service.CreateBaseline(new("empty-v2", first.AssetSha256, true, "CH4更新基线"), "test");
        second = _service.Activate(second.VersionId, new(first.VersionId, "启用新版本"), "test");
        var restarted = new LoadRadarService(_repository, _workflow, Options.Create(_options), _assets);
        Assert.Equal(first.VersionId, _repository.GetPlateBaselineAt("n1", first.LicensePlate, oldTime)!.VersionId);
        Assert.Equal(second.VersionId, Assert.Single(restarted.Baselines().Where(b => b.Active)).VersionId);
        restarted.Publish("loaded", new("重启后提交原时刻结果"), "test");
        var current = row.Measurement with { MeasurementId = "wrong-version", PassageId = "wrong-version-p", MeasuredAt = DateTimeOffset.UtcNow };
        restarted.Ingest("n1", current);
        restarted.Bind(current.MeasurementId, new("wrong-version-wf", "挂车001", "原版本不适用新观测"), "test");
        Assert.Throws<LoadRadarException>(() => restarted.Publish(current.MeasurementId, new("禁止旧版套新车"), "test"));
        restarted.Activate(first.VersionId, new(second.VersionId, "显式重新启用旧版本"), "test");
        Assert.Equal(second.VersionId, _repository.GetPlateBaselineAt("n1", first.LicensePlate, second.ActivatedAt!.Value)!.VersionId);
        Assert.Equal(first.VersionId, Assert.Single(restarted.Baselines().Where(b => b.Active)).VersionId);
    }

    [Theory]
    [InlineData("source-config")]
    [InlineData("source-calibration")]
    [InlineData("plate")]
    public async Task SharedReferenceCannotPublishAnotherSourceOrPlate(string mismatch)
    {
        var row = await ReadyLoaded();
        var use = SharedUse();
        if (mismatch == "source-config") use = use with { SourceConfigurationHash = new string('f', 64) };
        if (mismatch == "source-calibration") use = use with { SourceCalibrationId = "another-calibration" };
        var changed = row with
        {
            Measurement = row.Measurement with { BaselineUse = use },
            Binding = mismatch == "plate" ? row.Binding! with { LicensePlate = "宁A02621" } : row.Binding
        };
        _repository.Measurements["loaded"] = changed;
        Assert.Throws<LoadRadarException>(() => _service.Publish("loaded", new("引用不匹配"), "test"));
        Assert.Equal(0, _repository.PublishCount);
    }

    [Theory]
    [InlineData("source-channel")]
    [InlineData("policy")]
    [InlineData("status")]
    [InlineData("hash")]
    [InlineData("offset")]
    [InlineData("pose")]
    public void SharedInputIsValidatedAtIngestBoundary(string invalid)
    {
        var use = SharedUse();
        use = invalid switch
        {
            "source-channel" => use with { SourceChannelId = "CH1" },
            "policy" => use with { Policy = "unknown" },
            "status" => use with { RegistrationStatus = "unknown" },
            "hash" => use with { SourceConfigurationHash = "not-a-sha" },
            "offset" => use with { DxM = double.NaN },
            _ => use with { Pose = new([1], [1, 2], -1, 1, 0) }
        };
        Assert.Equal(400, Assert.Throws<LoadRadarException>(() => LoadRadarService.ValidateInput(
            Input() with { BaselineVersionId = "version", BaselineUse = use })).StatusCode);
    }

    private static string SourceDirectory([CallerFilePath] string path = "") => Path.GetDirectoryName(path)!;

    public void Dispose()
    {
        // 测试只清理自身随机生成的独占临时目录。
        if (Directory.Exists(_root)) Directory.Delete(_root, true);
    }

    /// <summary>可控的取牌来源：默认给 MOCK 车牌；Plate=null 表示来源不可用。</summary>
    private sealed class PlateSource : IRadarLicensePlateSource
    {
        public string? Plate = "宁A25771";
        public int Calls;
        public string? LastCamera;
        public bool Enabled => Plate is not null;
        public RadarLicensePlate? Resolve(string cameraIdentifier)
        {
            Calls++;
            LastCamera = cameraIdentifier;
            return Plate is null ? null : new(cameraIdentifier, Plate, DateTime.Now, "MOCK");
        }
    }

    private sealed class WorkflowRepository : NullIntegrationRepository
    {
        public readonly Dictionary<string, VehicleArrivalRecord> Arrivals = [];
        public bool FailAfterArrival;
        public override VehicleArrivalRecord? FindVehicleArrivalByEventId(string eventId) => Arrivals.GetValueOrDefault(eventId);
        public override bool TryClaimStaticScan(long recordId, string workflowId, DateTime updatedAt, StaticCapture gate) =>
            TryClaimVehicleArrivalAutoScan(recordId, updatedAt);
        public override bool TryClaimVehicleArrivalAutoScan(long recordId, DateTime updatedAt)
        {
            var arrival = Arrivals.Values.Single(a => a.Id == recordId);
            if (arrival.RadarStatus != VehicleArrivalRadarStatus.NotRequired) return false;
            arrival.RadarStatus = VehicleArrivalRadarStatus.Pending;
            return true;
        }
        public override void UpdateVehicleArrivalRadarResult(long recordId, string radarStatus, string? radarError, DateTime updatedAt)
        {
            var arrival = Arrivals.Values.Single(a => a.Id == recordId);
            arrival.RadarStatus = radarStatus;
            arrival.RadarError = radarError;
        }
        public override VehicleArrivalInsertResult InsertVehicleArrivalRecord(VehicleArrivalRecord record)
        {
            var added = Arrivals.TryAdd(record.SessionId, record);
            if (FailAfterArrival) { FailAfterArrival = false; throw new IOException("模拟到车提交后丢回执"); }
            return new() { Record = Arrivals[record.SessionId], IsNew = added };
        }
        public string? Plate = "宁A25771";
        public string Camera = "CAM010";
        public string? LastWorkflowId;
        public override string? GetSucceededWorkflowLicensePlate(string workflowId) { LastWorkflowId = workflowId; return Plate; }
        public override VehicleArrivalRecord? GetVehicleArrivalBySessionId(string sessionId) => Arrivals.GetValueOrDefault(sessionId) ?? new()
        { SessionId = sessionId, CameraIdentifier = Camera, LicensePlate = Plate };
    }

    // 仅为业务边界测试使用；不注册到应用，不替代真实达梦事务验证。
    private sealed class MemoryRepository : ILoadRadarRepository, ILoadRadarPublicationQueue, ILoadRadarPassageRepository
    {
        public WorkflowRepository Workflow { private get; set; } = null!;
        public readonly Dictionary<string, LoadPassageStartRecord> Starts = [];
        public LoadPassageStartRecord RegisterPassageStart(LoadPassageStartRecord record, VehicleArrivalRecord arrival, int registrationMaxSeconds)
        {
            if (Starts.ContainsKey(record.PassageKey))
            {
                if (!Workflow.Arrivals.ContainsKey(record.WorkflowId)) throw new LoadRadarException(409, "原到车登记不完整");
                return SavePassageStart(record);
            }
            var stored = SavePassageStart(record);
            Workflow.InsertVehicleArrivalRecord(arrival);
            return stored;
        }
        public LoadPassageStartRecord? GetPassageStart(string key) => Starts.GetValueOrDefault(key);
        public LoadPassageStartRecord SavePassageStart(LoadPassageStartRecord record)
        {
            if (Starts.TryGetValue(record.PassageKey, out var prior))
            {
                if (prior.Start != record.Start || prior.CameraIdentifier != record.CameraIdentifier)
                    throw new LoadRadarException(409, "到车上下文冲突");
                return prior;
            }
            Starts.Add(record.PassageKey, record);
            return record;
        }
        public LoadPassageBinding BindStartedPassage(LoadPassageStartRecord record, LoadPassageBinding binding)
        {
            var prior = Starts[record.PassageKey];
            if (prior.Binding is { } existing)
            {
                if (existing.LicensePlate != binding.LicensePlate || existing.WorkflowId != binding.WorkflowId)
                    throw new LoadRadarException(409, "关联冲突");
                return existing;
            }
            Starts[record.PassageKey] = prior with { Binding = binding };
            return binding;
        }
        public sealed record JobState(string Status, string Reason, int Attempts, DateTimeOffset Due);
        public readonly Dictionary<string, JobState> Jobs = [];
        public bool FailPublishBeforeCommit;
        public bool FailPublishAfterCommit;
        public bool FailOutcome;
        public readonly Dictionary<string, LoadMeasurementRecord> Measurements = [];
        public readonly Dictionary<string, LoadBaselineVersion> Versions = [];
        public CoalVolumeRecord? Coal;
        public int PublishCount;
        public LoadMeasurementRecord SaveMeasurement(LoadMeasurementRecord record)
        {
            if (Measurements.TryGetValue(record.Measurement.MeasurementId, out var existing))
            {
                if (existing.PayloadHash != record.PayloadHash) throw new LoadRadarException(409, "重复载荷冲突");
                return existing;
            }
            record = record with { Binding = Starts.GetValueOrDefault(record.PassageKey)?.Binding ?? record.Binding };
            Measurements.Add(record.Measurement.MeasurementId, record);
            if (LoadRadarPublicationPolicy.IsCandidate(record.Measurement, null))
                Jobs.Add(record.Measurement.MeasurementId, new("pending", "awaiting_processing", 0, record.ReceivedAt));
            return record;
        }
        public IReadOnlyList<LoadRadarPublicationJob> GetDuePublications(DateTimeOffset now, int take) => Jobs
            .Where(pair => pair.Value.Status == "pending" && pair.Value.Due <= now)
            .OrderBy(pair => pair.Value.Due).Take(take)
            .Select(pair => new LoadRadarPublicationJob(pair.Key, pair.Value.Attempts)).ToArray();
        public void SetPublicationOutcome(LoadRadarPublicationJob job, string status, string reasonCode, DateTimeOffset nextAttemptAt)
        {
            if (FailOutcome) { FailOutcome = false; throw new IOException("模拟任务回执失败"); }
            var stored = Jobs[job.MeasurementId];
            if (stored.Status == "pending" && stored.Attempts == job.AttemptCount)
                Jobs[job.MeasurementId] = new(status, reasonCode, stored.Attempts + 1, nextAttemptAt);
        }
        public LoadMeasurementRecord? GetMeasurement(string id) => Measurements.GetValueOrDefault(id);
        public IReadOnlyList<LoadMeasurementRecord> GetHistory(string? n, string? c, string? p, string? b, int take,
            DateTimeOffset? from = null, DateTimeOffset? to = null) => Measurements.Values
            .Where(row => (from is null || row.Measurement.MeasuredAt >= from) &&
                (to is null || row.Measurement.MeasuredAt <= to)).Take(take).ToArray();
        public LoadPassageBinding BindPassage(LoadMeasurementRecord measurement, LoadPassageBinding binding)
        {
            Measurements[measurement.Measurement.MeasurementId] = measurement with { Binding = binding };
            return binding;
        }
        public LoadBaselineVersion CreateBaseline(LoadBaselineVersion version)
        {
            Versions.TryAdd(version.VersionId, version);
            return Versions[version.VersionId];
        }
        public LoadBaselineVersion? GetBaseline(string id) => Versions.GetValueOrDefault(id);
        public IReadOnlyList<LoadBaselineVersion> GetBaselines() => Versions.Values.ToArray();
        private readonly List<(DateTimeOffset At, string Id)> _activations = [];
        public LoadBaselineVersion? GetBaselineAt(string scopeKey, DateTimeOffset at) => _activations
            .Where(x => x.At <= at && Versions[x.Id].ScopeKey == scopeKey)
            .OrderByDescending(x => x.At).Select(x => Versions[x.Id]).FirstOrDefault();
        public LoadBaselineVersion? GetPlateBaselineAt(string node, string plate, DateTimeOffset at) => _activations
            .Where(x => x.At <= at && Versions[x.Id].NodeId == node && Versions[x.Id].ChannelId == "CH4" && Versions[x.Id].LicensePlate == plate)
            .OrderByDescending(x => x.At).Select(x => Versions[x.Id]).FirstOrDefault();
        public LoadBaselineVersion ActivateBaseline(string id, string? expected, string actor, string reason)
        {
            var version = Versions[id];
            var current = GetPlateBaselineAt(version.NodeId, version.LicensePlate, DateTimeOffset.UtcNow);
            if (current?.VersionId != expected) throw new LoadRadarException(409, "版本已变化");
            if (current is not null) Versions[current.VersionId] = current with { Active = false };
            var effectiveAt = DateTimeOffset.UtcNow;
            _activations.Add((effectiveAt, id));
            return Versions[id] = Versions[id] with { Active = true, ActivatedAt = Versions[id].ActivatedAt ?? effectiveAt };
        }
        public LoadCoalPublication Publish(LoadMeasurementRecord measurement, CoalVolumeRecord coal, string actor, string reason)
        {
            if (FailPublishBeforeCommit) { FailPublishBeforeCommit = false; throw new IOException("模拟提交前故障"); }
            PublishCount++;
            Coal = coal;
            var result = new LoadCoalPublication(measurement.PassageKey, coal.WorkflowId!, measurement.Measurement.MeasurementId,
                42, actor, reason, DateTimeOffset.UtcNow);
            Measurements[measurement.Measurement.MeasurementId] = measurement with { Publication = result };
            if (FailPublishAfterCommit) { FailPublishAfterCommit = false; throw new IOException("模拟提交成功后回执丢失"); }
            return result;
        }
    }
}
