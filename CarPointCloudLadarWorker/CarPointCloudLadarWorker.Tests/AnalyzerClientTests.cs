using System.Text.Json.Nodes;
using CarPointCloudLadarWorker.Analysis;

namespace CarPointCloudLadarWorker.Tests;

public sealed class AnalyzerClientTests
{
    private static readonly PairedPcdCapture Pair = PairedPcdCapture.Create(
        new PcdCapture(@"C:\captures\a.pcd", "CAM_004", new DateTime(2026, 11, 3, 10, 0, 0, DateTimeKind.Local), "wf-1"),
        new PcdCapture(@"C:\captures\b.pcd", "CAM_005", new DateTime(2026, 11, 3, 10, 0, 1, DateTimeKind.Local), "wf-1"),
        DateTime.Now);

    private static readonly AnalysisRecord Analyzing = new(
        "wf-1", Pair.VehicleCaptureId, AnalysisStage.Analyzing, DateTime.Now, "CAM_004", "CAM_005",
        Pair.Primary.CapturedAt, [Pair.Primary.PcdPath, Pair.Secondary.PcdPath]);

    private static (AnalyzerClient Client, TestSupport.StubHandler Handler) Create(HttpResponseMessage response)
    {
        var handler = new TestSupport.StubHandler(_ => response);
        var options = TestSupport.Options(o =>
        {
            o.AnalyzerServiceUrl = "http://analyzer:8010/";
            o.DualRadarExtrinsicPath = @"D:\data\extrinsic.json";
            o.AnalyzerBaselineDirectory = @"D:\data\baselines";
        });
        return (new AnalyzerClient(new HttpClient(handler), Microsoft.Extensions.Options.Options.Create(options)), handler);
    }

    [Fact]
    public async Task AnalyzePair_PostsPairContractAndKeepsAnalyzerOutputs()
    {
        var (client, handler) = Create(TestSupport.Json("""
            {"success":true,"pairId":"p","workflowId":"wf-1",
             "fusion":{"mode":"fused","qualityPassed":true,"pointsMerged":10},
             "outputs":{"dimensions":{"carriage":{"length":10.6}},"measurementStatus":{"valid":true,"lengthMeasurement":{"side":"left"}},
                        "loadStatus":{"status":"Empty"},"reviewMeasurements":{"unit":"m"}},
             "integrationPayloads":{"vehicleDimensions":{"dimensions":{"trailerLength":1060.0}}},
             "persistence":{"status":"inserted"}}
            """));

        var record = await client.AnalyzePairAsync(Analyzing, Pair, "宁A25771", CancellationToken.None);

        var (request, body) = Assert.Single(handler.Requests);
        Assert.Equal("http://analyzer:8010/analyze-pair", request.RequestUri!.ToString());
        var sent = JsonNode.Parse(body)!.AsObject();
        Assert.Equal(
            ["pcdPathA", "cameraA", "pcdPathB", "cameraB", "pairId", "workflowId", "plate", "extrinsicPath", "baselineDir", "inputUnit", "capturedAt"],
            sent.Select(x => x.Key));
        Assert.Equal(Pair.VehicleCaptureId, (string?)sent["pairId"]);
        Assert.Equal("宁A25771", (string?)sent["plate"]);
        Assert.Equal("m", (string?)sent["inputUnit"]);

        Assert.Equal(AnalysisStage.Completed, record.Stage);
        Assert.True(record.HasDimensionPayload);
        var response = System.Text.Json.JsonSerializer.SerializeToNode(CarPointCloudLadarWorker.Api.ApiResponses.Analysis(record))!;
        Assert.Equal(1060.0, response["dimensionPayload"]?["dimensions"]?["trailerLength"]?.GetValue<double>());
        Assert.Equal("left", (string?)record.MeasurementStatus!["lengthMeasurement"]!["side"]);
        Assert.Equal("Empty", (string?)record.LoadStatus!["status"]);
        Assert.Equal("inserted", (string?)record.Persistence!["status"]);
        Assert.Equal("fused", (string?)record.Fusion!["mode"]);
    }

    [Fact]
    public async Task AnalyzePair_CompletedWithoutDimensionPayload()
    {
        var (client, _) = Create(TestSupport.Json("""{"success":true,"outputs":{"loadStatus":{"status":"Loaded"}}}"""));

        var record = await client.AnalyzePairAsync(Analyzing, Pair, null, CancellationToken.None);

        Assert.Equal(AnalysisStage.Completed, record.Stage);
        Assert.False(record.HasDimensionPayload);
    }

    [Fact]
    public async Task AnalyzePair_HttpErrorAndServiceFailureAreTerminalFailures()
    {
        var (httpError, _) = Create(TestSupport.Json("""{"detail":"busy"}""", 503));
        var (serviceFailure, _) = Create(TestSupport.Json("""{"success":false,"error":"bad pcd"}"""));

        var a = await httpError.AnalyzePairAsync(Analyzing, Pair, null, CancellationToken.None);
        var b = await serviceFailure.AnalyzePairAsync(Analyzing, Pair, null, CancellationToken.None);

        Assert.Equal((AnalysisStage.Failed, true), (a.Stage, a.PermanentFailure));
        Assert.StartsWith("status=HttpFailure http=503", a.Error);
        Assert.Equal(AnalysisStage.Failed, b.Stage);
        Assert.Contains("bad pcd", b.Error);
    }
}
