using System.Net;
using System.Net.Sockets;
using System.Text.Json.Nodes;
using CarPointCloudLadarWorker.Analysis;
using CarPointCloudLadarWorker.Hosting;
using CarPointCloudWebApi.Infrastructure;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;

namespace CarPointCloudLadarWorker.Tests;

/// <summary>合成雷达 → HTTP scan-once → PCD 落盘与配对 → Analyzer（桩）→ 分析查询，全部走正式组合；响应用 WebApi 的 RadarWorkerResponse 读取。</summary>
public sealed class EndToEndTests : IAsyncLifetime
{
    private readonly string _captureDirectory = TestSupport.TempDirectory();
    private readonly int _port = FreePort();
    private readonly TestSupport.StubHandler _analyzer = new(_ => TestSupport.Json("""
        {"success":true,"fusion":{"mode":"fused"},"outputs":{"loadStatus":{"status":"Empty"}},
         "integrationPayloads":{"vehicleDimensions":{"dimensions":{"trailerLength":1060.0}}},
         "persistence":{"status":"inserted"}}
        """));
    private readonly HttpClient _http = new();
    private IHost _host = null!;

    public async Task InitializeAsync()
    {
        var options = TestSupport.Options(o =>
        {
            o.EnableSyntheticRadar = true;
            o.SyntheticFrameIntervalSeconds = 0.05;
            o.SyntheticPointCount = 200;
            o.SyntheticInterDeviceDelayMilliseconds = 5;
            o.MaximumFrameDeliveryDelayMilliseconds = 0;
            o.CaptureDirectory = _captureDirectory;
            o.HttpApiPort = _port;
            o.HealthLogIntervalSeconds = 0;
        });
        var builder = Host.CreateApplicationBuilder();
        builder.Services.AddWorker(options);
        builder.Services.AddHttpClient<AnalyzerClient>().ConfigurePrimaryHttpMessageHandler(() => _analyzer);
        _host = builder.Build();
        await _host.StartAsync();
        _http.BaseAddress = new Uri($"http://127.0.0.1:{_port}");
    }

    public async Task DisposeAsync()
    {
        await _host.StopAsync();
        _host.Dispose();
    }

    [Fact]
    public async Task ScanOnce_CapturesBothRadars_AnalyzesPair_AndAnswersWebApiDispatch()
    {
        var start = await SendAsync(HttpMethod.Post, "/api/radars/scan-once?requestId=wf-e2e");
        var repeat = await SendAsync(HttpMethod.Post, "/api/radars/scan-once?requestId=wf-e2e");
        var busy = await SendAsync(HttpMethod.Post, "/api/radars/scan-once?requestId=wf-other");

        // WebApi 的 RadarScanDispatcher 以 2xx 判定受理。
        Assert.Equal((202, true), (start.StatusCode, start.IsSuccess));
        Assert.Equal((200, true), (repeat.StatusCode, repeat.IsSuccess));
        Assert.Equal("wf-e2e", (string?)repeat.Body!["scan"]!["requestId"]);
        Assert.Equal((409, "busy"), (busy.StatusCode, (string?)busy.Body!["code"]));

        var analysis = await WaitForAsync("/api/radars/analysis?workflowId=wf-e2e", x => (string?)x["stage"] == "completed");
        Assert.True((bool)analysis["hasDimensionPayload"]!);
        Assert.Equal(1060.0, analysis["dimensionPayload"]?["dimensions"]?["trailerLength"]?.GetValue<double>());
        Assert.Equal(["CAM_004", "CAM_005"], analysis["sourcePcdPaths"]!.AsArray()
            .Select(x => Path.GetFileName((string)x!)[..7]));
        Assert.All(analysis["sourcePcdPaths"]!.AsArray(), x => Assert.True(File.Exists((string)x!)));

        var sent = JsonNode.Parse(Assert.Single(_analyzer.Requests).Body)!;
        Assert.Equal("wf-e2e", (string?)sent["workflowId"]);
        Assert.Equal((string?)analysis["pairId"], (string?)sent["pairId"]);

        var lookup = await SendAsync(HttpMethod.Get, "/api/radars/scan-once/lookup?requestId=wf-e2e");
        Assert.True((bool)lookup.Body!["analysisEnabled"]!);
        var scan = lookup.Body!["scan"]!;
        Assert.Equal(("idle", "completed", "saved"), ((string?)scan["state"], (string?)scan["outcome"], (string?)scan["persistenceState"]));
        Assert.Equal(404, (await SendAsync(HttpMethod.Get, "/api/radars/scan-once/lookup?requestId=nope")).StatusCode);
    }

    [Fact]
    public async Task StatusEndpoints_ReportRadarsAndPreview()
    {
        var live = await WaitForAsync("/api/radars/live/status",
            x => x["radars"]!.AsArray().All(r => (long)r!["rawFrameCount"]! > 0));

        Assert.Equal(["CAM_004", "CAM_005"], live["radars"]!.AsArray().Select(r => (string?)r!["cameraIdentifier"]));
        Assert.True((bool)live["radars"]![0]!["connected"]!);
        Assert.False((bool)live["preview"]!["enabled"]!);
        Assert.Equal("idle", (string?)live["scan"]!["state"]);
        Assert.Equal(404, (await SendAsync(HttpMethod.Get, "/api/radar/status?camera=CAM_999")).StatusCode);
        Assert.Equal(409, (await SendAsync(HttpMethod.Post, "/api/radar/standby?mode=1")).StatusCode);
        Assert.Equal(200, (await SendAsync(HttpMethod.Post, "/api/radars/scan-once/stop")).StatusCode);
        Assert.Equal(400, (await SendAsync(HttpMethod.Get, "/api/radars/live/fused-frame?afterVersion=-1")).StatusCode);
    }

    private async Task<RadarWorkerResponse> SendAsync(HttpMethod method, string path)
    {
        using var response = await _http.SendAsync(new HttpRequestMessage(method, path));
        return new RadarWorkerResponse((int)response.StatusCode, JsonNode.Parse(await response.Content.ReadAsStringAsync()), null);
    }

    private async Task<JsonNode> WaitForAsync(string path, Func<JsonNode, bool> done)
    {
        var deadline = DateTime.UtcNow.AddSeconds(15);
        while (true)
        {
            var response = await SendAsync(HttpMethod.Get, path);
            if (response.StatusCode == 200 && done(response.Body!))
                return response.Body!;
            Assert.True(DateTime.UtcNow < deadline, $"等待 {path} 超时: {response.Body?.ToJsonString()}");
            await Task.Delay(50);
        }
    }

    private static int FreePort()
    {
        using var listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start();
        return ((IPEndPoint)listener.LocalEndpoint).Port;
    }
}
