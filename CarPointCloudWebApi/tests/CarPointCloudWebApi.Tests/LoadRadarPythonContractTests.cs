using System.Net;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.TestHost;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Options;
using Xunit;

namespace CarPointCloudWebApi.Tests;

/// <summary>Python 实际生成的 JSON/NumPy NPZ 进入真实 MVC/服务/资产校验链；不加载 Program 或生产配置。</summary>
public sealed class LoadRadarPythonContractTests
{
    private static readonly JsonSerializerOptions Json = new(JsonSerializerDefaults.Web);
    private const string Prefix = "/api/v1/load-radar-agent";

    [Theory]
    [InlineData("1.2345678901234567e-16")]
    [InlineData("1e-29")]
    [InlineData("0.030000000000000002")]
    public async Task SmallQualityNumbers_KeepExactDoubleAckAndImmutableReplay(string raw)
    {
        using var fixture = new Fixture();
        using var python = ReadPythonFixture();
        var payload = JsonNode.Parse(python.RootElement.GetProperty("loaded").GetRawText())!;
        payload["boundSpread"] = JsonNode.Parse(raw);
        var body = payload.ToJsonString();
        var first = await Receipt(await fixture.Client.PostAsync(Prefix + "/measurements", Body(body)));
        var echoed = first.GetProperty("measurement").GetProperty("boundSpread").GetDouble();
        Assert.Equal(double.Parse(raw, System.Globalization.CultureInfo.InvariantCulture), echoed);
        var again = await Receipt(await fixture.Client.PostAsync(Prefix + "/measurements", Body(body)));
        Assert.Equal(first.GetProperty("receivedAt").GetString(), again.GetProperty("receivedAt").GetString());
        Assert.Equal(first.GetProperty("payloadHash").GetString(), again.GetProperty("payloadHash").GetString());
        payload["boundSpread"] = 0;
        Assert.Equal(HttpStatusCode.Conflict, (await fixture.Client.PostAsync(Prefix + "/measurements", Body(payload.ToJsonString()))).StatusCode);
    }

    [Theory]
    [InlineData("volumeM3")]
    [InlineData("filledRatio")]
    [InlineData("boundSpread")]
    [InlineData("holeRatio")]
    public async Task AllMachineNumericFields_KeepSmallestChangesAndRejectInvalidNumbers(string field)
    {
        using var fixture = new Fixture();
        using var python = ReadPythonFixture();
        var payload = JsonNode.Parse(python.RootElement.GetProperty("loaded").GetRawText())!;
        foreach (var raw in new[] { "1e-29", "1.2345678901234567e-16" })
        {
            payload["measurementId"] = "numeric-" + field + "-" + (raw == "1e-29" ? "1" : "2");
            payload[field] = JsonNode.Parse(raw);
            var first = await Receipt(await fixture.Client.PostAsync(Prefix + "/measurements", Body(payload.ToJsonString())));
            var expected = double.Parse(raw, System.Globalization.CultureInfo.InvariantCulture);
            Assert.Equal(expected, first.GetProperty("measurement").GetProperty(field).GetDouble());
            payload[field] = Math.BitIncrement(expected);
            Assert.Equal(HttpStatusCode.Conflict, (await fixture.Client.PostAsync(Prefix + "/measurements", Body(payload.ToJsonString()))).StatusCode);
        }
        payload["measurementId"] = "invalid-" + field;
        foreach (var raw in new[] { "-1e-29", "\"0.1\"", "1e400" })
        {
            payload[field] = JsonNode.Parse(raw);
            Assert.Equal(HttpStatusCode.BadRequest, (await fixture.Client.PostAsync(Prefix + "/measurements", Body(payload.ToJsonString()))).StatusCode);
        }
        if (field != "boundSpread")
        {
            payload[field] = JsonNode.Parse(field == "volumeM3" ? "1000.000000000000000000000001" : "1.0000000000000000000000000001");
            Assert.Equal(HttpStatusCode.BadRequest, (await fixture.Client.PostAsync(Prefix + "/measurements", Body(payload.ToJsonString()))).StatusCode);
        }
        Assert.Equal(2, fixture.Repository.Measurements.Count);
    }

    [Theory]
    [InlineData("empty")]
    [InlineData("loaded")]
    public async Task PythonJson_UsesRealMvcBindingAndIngest_WithImmutableReplay(string kind)
    {
        using var fixture = new Fixture();
        using var python = ReadPythonFixture();
        var payload = python.RootElement.GetProperty(kind);
        var response = await fixture.Client.PostAsync(Prefix + "/measurements", Body(payload.GetRawText()));
        var receipt = await Receipt(response);
        var stored = receipt.Deserialize<LoadMeasurementRecord>(Json)!;
        Assert.Equal("node1", stored.Measurement.NodeId);
        Assert.Equal("CH1", stored.Measurement.ChannelId);
        Assert.Equal("cal1", stored.Measurement.CalibrationId);
        Assert.Equal("jt128-v1", stored.Measurement.AlgorithmVersion);
        Assert.Equal("live", stored.Measurement.SourceMode);
        Assert.Equal(DateTimeOffset.Parse("2024-01-01T00:00:00Z"), stored.Measurement.MeasuredAt);
        Assert.Equal(kind, stored.Measurement.Verdict);
        Assert.Equal(50, stored.Measurement.PointCount);
        Assert.Equal(2, stored.Measurement.Points.Length);
        Assert.Equal(new float[] { -1, -4, -2.5f }, stored.Measurement.Points[1]);
        Assert.Null(stored.Binding);
        Assert.Null(stored.Publication);
        if (kind == "loaded")
        {
            Assert.Equal("success", stored.Measurement.VolumeStatus);
            Assert.Equal(45.1234m, stored.Measurement.VolumeM3);
            Assert.Equal(.1m, stored.Measurement.HoleRatio);
            Assert.Equal(.02m, stored.Measurement.BoundSpread);
            Assert.Equal(.4m, stored.Measurement.FilledRatio);
            Assert.Equal("baseline-fixture", stored.Measurement.BaselineVersionId);
        }
        else
        {
            Assert.Equal("unavailable", stored.Measurement.VolumeStatus);
            Assert.Null(stored.Measurement.VolumeM3);
            Assert.Null(stored.Measurement.HoleRatio);
        }
        var replay = await Receipt(await fixture.Client.PostAsync(Prefix + "/measurements", Body(payload.GetRawText())));
        Assert.Equal(receipt.GetProperty("receivedAt").GetString(), replay.GetProperty("receivedAt").GetString());
        Assert.Equal(receipt.GetProperty("payloadHash").GetString(), replay.GetProperty("payloadHash").GetString());
        var changed = JsonNode.Parse(payload.GetRawText())!;
        changed["reason"] = "同一 ID 不得替换原始载荷";
        var conflict = await fixture.Client.PostAsync(Prefix + "/measurements", Body(changed.ToJsonString()));
        Assert.Equal(HttpStatusCode.Conflict, conflict.StatusCode);
        var read = await Receipt(await fixture.Client.GetAsync(Prefix + "/measurements/" + stored.Measurement.MeasurementId));
        Assert.Equal("cross-language fixture", read.GetProperty("measurement").GetProperty("reason").GetString());
        Assert.Single(fixture.Repository.Measurements);
    }

    [Fact]
    public async Task PythonCandidate_RequiresMeasurementReceiptThenVerifiedBinaryArchiveReceipt()
    {
        using var fixture = new Fixture();
        using var python = ReadPythonFixture();
        var payload = python.RootElement.GetProperty("empty");
        var bytes = Convert.FromBase64String(python.RootElement.GetProperty("assetBase64").GetString()!);
        var hash = payload.GetProperty("baselineCandidateSha256").GetString()!;
        Assert.Equal(hash, Convert.ToHexString(SHA256.HashData(bytes)).ToLowerInvariant());
        var path = Prefix + "/baseline-assets/" + hash + "?measurementId=python-empty-1";
        Assert.Equal(HttpStatusCode.NotFound, (await fixture.Client.PutAsync(path, new ByteArrayContent(bytes))).StatusCode);
        var measurement = await Receipt(await fixture.Client.PostAsync(Prefix + "/measurements", Body(payload.GetRawText())));
        Assert.Equal(hash, measurement.GetProperty("measurement").GetProperty("baselineCandidateSha256").GetString());
        using var binary = new ByteArrayContent(bytes);
        binary.Headers.ContentType = new("application/octet-stream");
        var archive = await Receipt(await fixture.Client.PutAsync(path, binary));
        Assert.True(archive.GetProperty("archived").GetBoolean());
        Assert.Equal(hash, archive.GetProperty("assetSha256").GetString());
        fixture.Assets.Verify(hash); // 包括实际 Python 生成 NPZ 的摘要与 NPY 数组校验。
        using var archived = fixture.Assets.Open(hash);
        using var copy = new MemoryStream();
        await archived.CopyToAsync(copy);
        Assert.Equal(bytes, copy.ToArray());
    }

    [Fact]
    public async Task RealMachineAuthentication_RejectsMissingKeyAndCrossNodePayload()
    {
        using var fixture = new Fixture();
        using var python = ReadPythonFixture();
        var payload = python.RootElement.GetProperty("empty").GetRawText();
        fixture.Client.DefaultRequestHeaders.Remove(LoadRadarAgentAuthMiddleware.KeyHeader);
        Assert.Equal(HttpStatusCode.Unauthorized,
            (await fixture.Client.PostAsync(Prefix + "/measurements", Body(payload))).StatusCode);
        fixture.Client.DefaultRequestHeaders.Add(LoadRadarAgentAuthMiddleware.KeyHeader, Fixture.AgentKey);
        var crossNode = JsonNode.Parse(payload)!;
        crossNode["nodeId"] = "other-node";
        Assert.Equal(HttpStatusCode.Forbidden,
            (await fixture.Client.PostAsync(Prefix + "/measurements", Body(crossNode.ToJsonString()))).StatusCode);
        Assert.Empty(fixture.Repository.Measurements);
    }

    private static StringContent Body(string json) => new(json, Encoding.UTF8, "application/json");

    private static async Task<JsonElement> Receipt(HttpResponseMessage response)
    {
        Assert.Equal(HttpStatusCode.OK, response.StatusCode);
        using var document = JsonDocument.Parse(await response.Content.ReadAsStringAsync());
        Assert.True(document.RootElement.GetProperty("success").GetBoolean());
        Assert.Equal(200, document.RootElement.GetProperty("code").GetInt32());
        return document.RootElement.GetProperty("data").Clone();
    }

    private static JsonDocument ReadPythonFixture([CallerFilePath] string sourceFile = "") =>
        JsonDocument.Parse(File.ReadAllText(Path.Combine(Path.GetDirectoryName(sourceFile)!, "Fixtures", "load-radar-python-v1.json")));

    private sealed class Fixture : IDisposable
    {
        public const string AgentKey = "python-contract-only-not-production-key-0123456789";
        private readonly string _root = Path.Combine(Path.GetTempPath(), "load-python-contract-" + Guid.NewGuid().ToString("N"));
        private readonly TestServer _server;
        public ContractRepository Repository { get; }
        public LoadRadarAssetStore Assets { get; }
        public HttpClient Client { get; }

        public Fixture()
        {
            var options = Options.Create(new LoadRadarOptions
            {
                Enabled = true,
                StorageRoot = _root,
                Nodes = [new()
            {
                NodeId = "node1", AgentKey = AgentKey,
                Channels = [new() { ChannelId = "CH1", CameraIdentifier = "CAM_TEST", Name = "合同测试" }]
            }]
            });
            var repository = DispatchProxy.Create<ILoadRadarRepository, ContractRepository>();
            Repository = (ContractRepository)repository;
            Assets = new(options);
            _server = new TestServer(new WebHostBuilder().ConfigureServices(services =>
            {
                services.AddSingleton(options);
                services.AddSingleton<ILoadRadarRepository>(repository);
                services.AddSingleton<IIntegrationRepository>(new NullIntegrationRepository());
                services.AddSingleton(Assets);
                services.AddSingleton<LoadRadarService>();
                services.AddControllers().AddApplicationPart(typeof(LoadRadarAgentController).Assembly);
            }).Configure(app =>
            {
                app.UseMiddleware<ApiExceptionMiddleware>();
                app.UseMiddleware<LoadRadarAgentAuthMiddleware>();
                app.UseRouting();
                app.UseEndpoints(endpoints => endpoints.MapControllers());
            }));
            Client = _server.CreateClient();
            Client.DefaultRequestHeaders.Add(LoadRadarAgentAuthMiddleware.NodeHeader, "node1");
            Client.DefaultRequestHeaders.Add(LoadRadarAgentAuthMiddleware.KeyHeader, AgentKey);
        }

        public void Dispose()
        {
            Client.Dispose();
            _server.Dispose();
            if (Directory.Exists(_root)) Directory.Delete(_root, true);
        }
    }

    // 此仓储仅限本 TestServer，不替代达梦事务或发布集成验证。
    public class ContractRepository : DispatchProxy
    {
        public readonly Dictionary<string, LoadMeasurementRecord> Measurements = [];
        protected override object? Invoke(MethodInfo? method, object?[]? args)
        {
            if (method?.Name == nameof(ILoadRadarRepository.GetMeasurement))
                return Measurements.GetValueOrDefault((string)args![0]!);
            if (method?.Name != nameof(ILoadRadarRepository.SaveMeasurement)) throw new NotSupportedException();
            var record = (LoadMeasurementRecord)args![0]!;
            if (Measurements.TryGetValue(record.Measurement.MeasurementId, out var prior))
            {
                if (prior.PayloadHash != record.PayloadHash) throw new LoadRadarException(409, "原始载荷冲突");
                return prior;
            }
            Measurements.Add(record.Measurement.MeasurementId, record);
            return record;
        }
    }
}
