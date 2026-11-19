using System.Net;
using System.Net.Http.Json;
using System.Runtime.CompilerServices;
using System.Text.Json;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.TestHost;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Options;
using Xunit;

/// <summary>真实Python计算夹具→HTTP模型绑定、节点认证与业务边界；持久层为替身，不连接DM8。</summary>
namespace CarPointCloudWebApi.Tests;

public sealed class LoadRadarRecalculationTests : IDisposable
{
    private readonly string _root = Path.Combine(Path.GetTempPath(), "day-recalc-api-" + Guid.NewGuid().ToString("N"));
    private static readonly JsonSerializerOptions Json = new(JsonSerializerDefaults.Web);
    private readonly JsonDocument _fixture;
    private readonly MemoryRepository _repository = new();
    private readonly LoadRadarService _service;
    private readonly LoadRadarAssetStore _assets;
    private readonly TestServer _server;
    private readonly HttpClient _client;

    public LoadRadarRecalculationTests()
    {
        _fixture = JsonDocument.Parse(File.ReadAllText(Path.Combine(SourceDirectory(), "Fixtures", "load-radar-day-recalculation-v1.json")));
        foreach (var channel in new[] { "CH1", "CH2", "CH3", "CH4" })
        {
            var target = _fixture.RootElement.GetProperty(channel).GetProperty("target").Deserialize<LoadRecalculationTarget>(Json)!;
            _repository.Targets.Add(target);
            _repository.Rows.Add(target.Original.Measurement.MeasurementId, target.Original);
            _repository.Versions[target.Baseline.VersionId] = target.Baseline;
        }
        var baseline = _repository.Targets[0];
        _repository.Rows[baseline.BaselineSource.MeasurementId] = new("empty-passage", "a", baseline.BaselineSource, DateTimeOffset.UtcNow);
        var options = Options.Create(new LoadRadarOptions
        {
            Enabled = true,
            StorageRoot = _root,
            Nodes =
            [new() { NodeId = "test-node", AgentKey = new string('a', 40), Channels = Enumerable.Range(1, 4)
                .Select(i => new LoadRadarLaneOptions { ChannelId = "CH" + i, CameraIdentifier = "CAM_00" + i }).ToList() },
             new() { NodeId = "other-node", AgentKey = new string('b', 40), Channels = [new() { ChannelId = "CH1" }] }]
        });
        _assets = new(options);
        _service = new(_repository, new NullIntegrationRepository(), options, _assets);
        _server = new TestServer(new WebHostBuilder().ConfigureServices(services =>
        {
            services.AddLogging();
            services.AddSingleton(options); services.AddSingleton(_service); services.AddSingleton(_assets);
            services.AddControllers().AddApplicationPart(typeof(LoadRadarAgentController).Assembly);
        }).Configure(app =>
        {
            app.UseMiddleware<ApiExceptionMiddleware>();
            app.UseMiddleware<LoadRadarAgentAuthMiddleware>();
            app.UseRouting(); app.UseEndpoints(endpoints => endpoints.MapControllers());
        }));
        _client = _server.CreateClient();
        _client.DefaultRequestHeaders.Add(LoadRadarAgentAuthMiddleware.NodeHeader, "test-node");
        _client.DefaultRequestHeaders.Add(LoadRadarAgentAuthMiddleware.KeyHeader, new string('a', 40));
    }

    private LoadRecalculationInput Request(string channel = "CH1") =>
        _fixture.RootElement.GetProperty(channel).GetProperty("request").Deserialize<LoadRecalculationInput>(Json)!;

    private async Task Upload()
    {
        using var bytes = new MemoryStream(Convert.FromBase64String(_fixture.RootElement.GetProperty("assetBase64").GetString()!));
        await _assets.Upload(_repository.Targets[0].Baseline.AssetSha256, bytes, default);
    }

    [Theory]
    [InlineData("CH1")]
    [InlineData("CH2")]
    [InlineData("CH3")]
    [InlineData("CH4")]
    public async Task Python_future_CH4_baseline_result_is_accepted_without_changing_original(string channel)
    {
        await Upload();
        var request = Request(channel);
        var original = _repository.Rows[request.MeasurementId];
        var response = await _client.PostAsJsonAsync("/api/v1/load-radar-agent/recalculations", request);
        Assert.Equal(HttpStatusCode.OK, response.StatusCode);
        var result = (await response.Content.ReadFromJsonAsync<ApiResponse<LoadRecalculationReceipt>>())!.Data!;
        Assert.Equal("applied", result.Status);
        Assert.Equal(request.Id, result.Id);
        Assert.Equal(request.Measurement.VolumeM3!.Value.RoundCoalVolume(), result.Revision!.VolumeM3);
        Assert.Equal(request.Measurement.MeasuredAt, result.Revision.Calculation.MeasuredAt);
        Assert.Equal(channel, result.Revision.Calculation.ChannelId);
        Assert.Equal(original.Measurement, _repository.Rows[request.MeasurementId].Measurement);
        Assert.True(_repository.Versions[request.BaselineVersionId].ActivatedAt > request.Measurement.MeasuredAt);
        var again = await _client.PostAsJsonAsync("/api/v1/load-radar-agent/recalculations", request);
        Assert.Equal(HttpStatusCode.OK, again.StatusCode);
        Assert.Single(_repository.Revisions);
        Assert.Single(_repository.Volumes);
    }

    [Fact]
    public async Task Poll_passes_original_day_and_channel_and_requires_machine_credentials()
    {
        var response = await _client.GetAsync("/api/v1/load-radar-agent/recalculations?channelId=CH2&fromDate=2026-10-21&afterId=m-before");
        response.EnsureSuccessStatusCode();
        Assert.Equal(("test-node", "CH2", "m-before", new DateOnly(2026, 10, 21)), _repository.LastPoll);
        _client.DefaultRequestHeaders.Remove(LoadRadarAgentAuthMiddleware.KeyHeader);
        Assert.Equal(HttpStatusCode.Unauthorized,
            (await _client.GetAsync("/api/v1/load-radar-agent/recalculations?channelId=CH1")).StatusCode);
    }

    [Theory]
    [InlineData("channelId=CH5")]
    [InlineData("channelId=CH1&fromDate=not-a-date")]
    [InlineData("channelId=CH1&fromDate=0001-01-01")]
    [InlineData("channelId=CH1&afterId=../escape")]
    public async Task Invalid_poll_is_rejected_at_boundary(string query)
    {
        Assert.Equal(HttpStatusCode.BadRequest,
            (await _client.GetAsync("/api/v1/load-radar-agent/recalculations?" + query)).StatusCode);
        Assert.Null(_repository.LastPoll);
    }

    [Theory]
    [InlineData("time")]
    [InlineData("channel")]
    [InlineData("passage")]
    [InlineData("configuration")]
    [InlineData("window")]
    [InlineData("degraded")]
    [InlineData("points")]
    public async Task Revision_cannot_replace_original_observation_identity(string mutation)
    {
        var request = Request();
        var m = request.Measurement;
        m = mutation switch
        {
            "time" => m with { MeasuredAt = m.MeasuredAt.AddSeconds(1) },
            "channel" => m with { ChannelId = "CH2" },
            "passage" => m with { PassageId = "another-passage" },
            "configuration" => m with { ConfigurationHash = new string('a', 64) },
            "window" => m with { MeasurementStartedAt = m.MeasurementStartedAt!.Value.AddMilliseconds(50) },
            "degraded" => m with { FrameDegraded = true },
            _ => m with { PointCount = m.PointCount + 1 }
        };
        var response = await _client.PostAsJsonAsync("/api/v1/load-radar-agent/recalculations", request with { Measurement = m });
        Assert.Equal(HttpStatusCode.Conflict, response.StatusCode);
        Assert.Empty(_repository.Revisions);
    }

    [Fact]
    public async Task Another_node_and_inconsistent_revision_key_are_rejected()
    {
        var request = Request();
        Assert.Equal(HttpStatusCode.BadRequest, (await _client.PostAsJsonAsync("/api/v1/load-radar-agent/recalculations",
            request with { Id = "wrong-key" })).StatusCode);
        _client.DefaultRequestHeaders.Remove(LoadRadarAgentAuthMiddleware.NodeHeader);
        _client.DefaultRequestHeaders.Remove(LoadRadarAgentAuthMiddleware.KeyHeader);
        _client.DefaultRequestHeaders.Add(LoadRadarAgentAuthMiddleware.NodeHeader, "other-node");
        _client.DefaultRequestHeaders.Add(LoadRadarAgentAuthMiddleware.KeyHeader, new string('b', 40));
        Assert.Equal(HttpStatusCode.Forbidden,
            (await _client.PostAsJsonAsync("/api/v1/load-radar-agent/recalculations", request)).StatusCode);
        Assert.Empty(_repository.Revisions);
    }

    [Fact]
    public async Task Original_motion_frames_can_expand_calculation_window_without_changing_capture_time()
    {
        await Upload();
        var request = Request();
        var result = request.Measurement with
        {
            MeasurementStartedAt = request.Measurement.MeasurementStartedAt!.Value.AddSeconds(-2),
            MeasurementEndedAt = request.Measurement.MeasurementEndedAt!.Value.AddSeconds(2)
        };
        var receipt = _service.Revise("test-node", request with { Measurement = result });
        Assert.Equal("applied", receipt.Status);
        Assert.Equal(request.Measurement.MeasuredAt, receipt.Revision!.Calculation.MeasuredAt);
        Assert.Equal(result.MeasurementEndedAt, receipt.Revision.Calculation.MeasurementEndedAt);
        Assert.Equal(request.Measurement.MeasurementEndedAt, _repository.Rows[request.MeasurementId].Measurement.MeasurementEndedAt);
    }

    [Fact]
    public async Task Retry_changed_payload_is_rejected_and_superseded_payload_does_not_apply()
    {
        await Upload();
        var request = Request();
        Assert.Equal("applied", _service.Revise("test-node", request).Status);
        Assert.Equal(409, Assert.Throws<LoadRadarException>(() => _service.Revise("test-node", request with
        { Measurement = request.Measurement with { VolumeM3 = 50m } })).StatusCode);
        _repository.CurrentActivation = 99;
        var stale = Request("CH2");
        Assert.Equal("superseded", _service.Revise("test-node", stale).Status);
        Assert.Single(_repository.Revisions);
        Assert.Single(_repository.Volumes);
    }

    [Fact]
    public void Retrospective_policy_allows_later_baseline_without_relaxing_normal_publication()
    {
        var target = _repository.Targets[0];
        var result = Request().Measurement;
        Assert.False(LoadRadarPublicationPolicy.MatchesBaseline(result, target.Original.Binding!, target.Baseline, target.BaselineSource));
        Assert.True(LoadRadarPublicationPolicy.MatchesBaseline(result, target.Original.Binding!, target.Baseline, target.BaselineSource, retrospective: true));
        Assert.False(LoadRadarPublicationPolicy.MatchesBaseline(result, target.Original.Binding!, target.Baseline with
        { LicensePlate = "宁A52522S" }, target.BaselineSource, retrospective: true));
    }

    [Fact]
    public void Automatic_baseline_flag_requires_real_complete_CH4_empty_asset()
    {
        var input = _repository.Targets[0].BaselineSource with { AutomaticBaseline = true, Reason = "合成完整空车" };
        LoadRadarService.ValidateInput(input);
        foreach (var invalid in new[] { input with { ChannelId = "CH1" }, input with { Verdict = "loaded" },
            input with { FrameDegraded = true }, input with { SourceMode = "demo" }, input with { BaselineCandidateSha256 = null } })
            Assert.Equal(400, Assert.Throws<LoadRadarException>(() => LoadRadarService.ValidateInput(invalid)).StatusCode);
    }

    [Fact]
    public async Task Automatic_empty_asset_upload_activates_once_and_retry_keeps_same_version()
    {
        var target = _repository.Targets[3];
        var input = target.BaselineSource with { AutomaticBaseline = true, Reason = "合成自动空车候选" };
        _repository.Rows[input.MeasurementId] = new("empty-passage", "a", input, DateTimeOffset.UtcNow,
            target.Original.Binding! with { WorkflowId = "empty-workflow" });
        var bytes = Convert.FromBase64String(_fixture.RootElement.GetProperty("assetBase64").GetString()!);
        var path = "/api/v1/load-radar-agent/baseline-assets/" + input.BaselineCandidateSha256 + "?measurementId=" + input.MeasurementId;
        for (var i = 0; i < 2; i++)
        {
            var response = await _client.PutAsync(path, new ByteArrayContent(bytes));
            Assert.Equal(HttpStatusCode.OK, response.StatusCode);
        }
        Assert.Equal(1, _repository.AutomaticActivations);
    }

    private sealed class MemoryRepository : ILoadRadarRepository
    {
        public readonly List<LoadRecalculationTarget> Targets = [];
        public readonly Dictionary<string, LoadMeasurementRecord> Rows = [];
        public readonly Dictionary<string, LoadBaselineVersion> Versions = [];
        public readonly Dictionary<string, LoadVolumeRevision> Revisions = [];
        public readonly Dictionary<string, decimal> Volumes = [];
        public (string, string, string?, DateOnly)? LastPoll;
        public long CurrentActivation = 17;
        public int AutomaticActivations;
        public IReadOnlyList<LoadRecalculationTarget> GetRecalculations(string node, string channel, string? after, DateOnly from)
        { LastPoll = (node, channel, after, from); return Targets.Where(t => t.Original.Measurement.ChannelId == channel).ToArray(); }
        public LoadRecalculationReceipt ApplyRecalculation(string node, LoadRecalculationInput input, string hash)
        {
            if (Revisions.TryGetValue(input.Id, out var prior))
            {
                if (hash != prior.ResultHash) throw new LoadRadarException(409, "相同复算ID不能改变结果");
                return new(input.Id, "applied", prior);
            }
            if (input.ActivationId != CurrentActivation) return new(input.Id, "superseded", null);
            var old = Volumes.TryGetValue(input.MeasurementId, out var value) ? (decimal?)value : null;
            var volume = input.Measurement.VolumeM3!.Value.RoundCoalVolume();
            var revision = new LoadVolumeRevision(input.Id, input.MeasurementId, input.ActivationId, input.BaselineVersionId,
                1, old, volume, null, hash, input.Measurement, input.Provenance, DateTimeOffset.UtcNow);
            Revisions.Add(input.Id, revision); Volumes[input.MeasurementId] = volume;
            return new(input.Id, "applied", revision);
        }
        public IReadOnlyList<LoadVolumeRevision> GetVolumeRevisions(string id) => Revisions.Values.Where(r => r.MeasurementId == id).ToArray();
        public LoadMeasurementRecord? GetMeasurement(string id) => Rows.GetValueOrDefault(id);
        public LoadBaselineVersion? GetBaseline(string id) => Versions.GetValueOrDefault(id);
        public IReadOnlyList<LoadBaselineVersion> GetBaselines() => Versions.Values.ToArray();
        public LoadMeasurementRecord SaveMeasurement(LoadMeasurementRecord row) => Rows[row.Measurement.MeasurementId] = row;
        public IReadOnlyList<LoadMeasurementRecord> GetHistory(string? node, string? channel, string? plate, string? before,
            int take, DateTimeOffset? from = null, DateTimeOffset? to = null) => [];
        public LoadPassageBinding BindPassage(LoadMeasurementRecord row, LoadPassageBinding binding) => binding;
        public LoadBaselineVersion CreateBaseline(LoadBaselineVersion version) =>
            Versions.TryGetValue(version.VersionId, out var old) ? old : Versions[version.VersionId] = version;
        public LoadBaselineVersion ActivateBaseline(string id, string? expected, string actor, string reason) => Versions[id];
        public LoadBaselineVersion ActivateAutomaticBaseline(string id, string actor, string reason)
        { AutomaticActivations++; return Versions[id] = Versions[id] with { Active = true, ActivatedAt = DateTimeOffset.UtcNow }; }
        public LoadCoalPublication Publish(LoadMeasurementRecord row, CoalVolumeRecord coal, string actor, string reason) => throw new NotSupportedException();
    }

    private static string SourceDirectory([CallerFilePath] string file = "") => Path.GetDirectoryName(file)!;
    public void Dispose()
    {
        _client.Dispose(); _server.Dispose(); _fixture.Dispose();
        if (Directory.Exists(_root) && Path.GetFullPath(_root).StartsWith(Path.Combine(Path.GetTempPath(), "day-recalc-api-"), StringComparison.OrdinalIgnoreCase))
            Directory.Delete(_root, true);
    }
}
