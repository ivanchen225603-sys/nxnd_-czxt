using System.Net;
using System.Text;
using System.Text.Json.Nodes;
using Microsoft.Extensions.Logging.Abstractions;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class RadarCompletionTests
{
    [Theory]
    [InlineData(false, false, 200)]
    [InlineData(false, true, 503)]
    public async Task CompletionIsBoundToFreshSourceSession(bool restart, bool old, int expected)
    {
        var handler = new PreviewHandler { Restart = restart, Old = old };
        var factory = new Factory(handler);
        var radar = new RadarLiveClient(factory);
        var service = new RadarCompletionClient(radar, factory);
        var response = await service.GetAsync(0, default);
        Assert.Equal(expected, response.StatusCode);
        Assert.DoesNotContain(handler.Paths, p => p.Contains("scan-once") || p.Contains("captures") || p.Contains("analyze-pair"));
        if (expected == 200)
        {
            Assert.Equal(3, response.Body!["version"]!.GetValue<int>());
            Assert.Equal(1, response.Body["syntheticPointCount"]!.GetValue<int>());
            Assert.Null(response.Body["dimensions"]);
            Assert.Equal(200, (await service.GetAsync(0, default)).StatusCode);
            Assert.Equal(1, handler.Completions);
        }
    }

    [Fact]
    public async Task NonIdleScanNeverCallsAnalyzer()
    {
        var handler = new PreviewHandler { State = "scanning" };
        var factory = new Factory(handler);
        var service = new RadarCompletionClient(new RadarLiveClient(factory), factory);
        Assert.Equal(503, (await service.GetAsync(0, default)).StatusCode);
        Assert.Equal(0, handler.Completions);
    }

    [Theory]
    [InlineData(0, 10, 200)]
    public async Task CleanShellPreservesSyntheticProvenanceAndCountLimits(int measured, int synthetic, int expected)
    {
        var handler = new PreviewHandler { Measured = measured, Synthetic = synthetic, Appearance = true };
        var factory = new Factory(handler);
        var service = new RadarCompletionClient(new RadarLiveClient(factory), factory);
        var response = await service.GetAsync(0, default);
        Assert.Equal(expected, response.StatusCode);
        if (expected == 200)
        {
            Assert.Equal(0, response.Body!["pointCountA"]!.GetValue<int>());
            Assert.Equal(synthetic, response.Body["syntheticPointCount"]!.GetValue<int>());
            Assert.True(response.Body["wallCompletion"]!["appearance"]!["displayOnly"]!.GetValue<bool>());
            Assert.True(response.Body["referenceOnly"]!.GetValue<bool>());
        }
    }

    private sealed class Factory(PreviewHandler handler) : IHttpClientFactory
    {
        public HttpClient CreateClient(string name) => new(handler, false) { BaseAddress = new Uri("http://127.0.0.1") };
    }

    private sealed class PreviewHandler : HttpMessageHandler
    {
        public bool Restart { get; init; }
        public bool Old { get; init; }
        public string State { get; init; } = "idle";
        public int Measured { get; init; } = 1;
        public int Synthetic { get; init; } = 1;
        public bool Appearance { get; init; }
        public JsonNode? Mesh { get; init; }
        public int Completions { get; private set; }
        public List<string> Paths { get; } = [];
        protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
        {
            var path = request.RequestUri!.AbsolutePath;
            Paths.Add(path);
            JsonNode body;
            if (path.EndsWith("status"))
            {
                body = new JsonObject
                {
                    ["preview"] = new JsonObject { ["available"] = true, ["sessionId"] = Restart && Completions > 0 ? "new" : "old" },
                    ["scan"] = new JsonObject { ["state"] = State }
                };
            }
            else if (path.EndsWith("fused-frame"))
            {
                var time = DateTimeOffset.UtcNow.AddSeconds(Old ? -30 : -1).ToString("O");
                body = new JsonObject
                {
                    ["updated"] = true,
                    ["version"] = 3,
                    ["sessionId"] = "old",
                    ["capturedAtA"] = time,
                    ["capturedAtB"] = time,
                    ["points"] = Convert.ToBase64String(new byte[24]),
                    ["pointCountA"] = 1,
                    ["pointCountB"] = 1,
                    ["pointsEncoding"] = "float32le-xyz-base64"
                };
            }
            else
            {
                Assert.Equal("/preview-completion", path);
                var sent = await request.Content!.ReadAsStringAsync(cancellationToken);
                Assert.DoesNotContain("workflow", sent);
                Assert.DoesNotContain("plate", sent);
                Completions++;
                body = new JsonObject
                {
                    ["referenceOnly"] = true,
                    ["displayMesh"] = Mesh?.DeepClone(),
                    ["measuredPointCount"] = Measured,
                    ["syntheticPointCount"] = Synthetic,
                    ["points"] = Convert.ToBase64String(new byte[Math.Max(0, Measured + Synthetic) * 12]),
                    ["wallCompletion"] = Appearance ? new JsonObject
                    {
                        ["appearance"] = new JsonObject { ["applied"] = true, ["displayOnly"] = true }
                    } : null
                };
            }
            return new(HttpStatusCode.OK) { Content = new StringContent(body.ToJsonString(), Encoding.UTF8, "application/json") };
        }
    }
}
