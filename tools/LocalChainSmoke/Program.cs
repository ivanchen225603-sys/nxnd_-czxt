// Test-only console: does not launch either production .NET host or contact DM8/radars.
using System.Net;
using System.Net.Http.Json;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Text.Json.Serialization;
using CarPointCloudLadarWorker.Analysis;
using CarPointCloudLadarWorker.Config;
using CarPointCloudLadarWorker.Domain;
using CarPointCloudWebApi.Api;
using CarPointCloudWebApi.Application;
using CarPointCloudWebApi.Domain;
using CarPointCloudWebApi.Infrastructure;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;

var uri = new Uri(args[0]);
if (uri.Host != "127.0.0.1") throw new ArgumentException("test host must be loopback");
var json = new JsonSerializerOptions(JsonSerializerDefaults.Web);
json.Converters.Add(new JsonStringEnumConverter(JsonNamingPolicy.CamelCase));
var cases = JsonNode.Parse(File.ReadAllText(args[1]))!.AsArray();
var analyzerResponses = new CapturingHandler();
using var http = new HttpClient(analyzerResponses) { BaseAddress = uri, Timeout = TimeSpan.FromMinutes(10) };
var analyzer = new AnalyzerClient(http, Options.Create(new LadarCaptureOptions { AnalyzerServiceUrl = uri.ToString() }));
foreach (var item in cases)
{
    string Read(string key) => item![key]!.GetValue<string>();
    var workflow = Read("workflow");
    var plate = Read("plate");
    var expected = item!["expected"]!.AsObject();
    var terminal = expected["terminal"]!.GetValue<string>();
    // Recorded PCD timestamps do not prove a live same-session acquisition. Simulate
    // metadata only; the actual original files remain untouched and independently hashed.
    var captured = new DateTime(2026, 9, 9, 10, 0, 0);
    var pair = PairedPcdCapture.Create(
        new PcdCapture(Read("a"), "CAM_004", captured, workflow),
        new PcdCapture(Read("b"), "CAM_005", captured.AddSeconds(2), workflow),
        DateTime.Now);
    var results = new SmokeResults();
    Console.WriteLine($"Analyzing {Read("name")} through real loopback HTTP...");
    var run = await analyzer.AnalyzePairAsync(
        new AnalysisRecord(workflow, pair.VehicleCaptureId, AnalysisStage.Analyzing, DateTime.Now, "CAM_004", "CAM_005",
            captured, [pair.Primary.PcdPath, pair.Secondary.PcdPath]),
        pair, null, CancellationToken.None);
    var analyzerBody = JsonNode.Parse(analyzerResponses.LastBody!)!;
    File.WriteAllText(Path.Combine(Path.GetDirectoryName(args[1])!, workflow + "-analyzer.json"),
        analyzerBody.ToJsonString());
    Require(run.Stage == AnalysisStage.Completed, $"Analyzer failed: {run.Error}");
    Require(run.Fusion?["qualityPassed"]?.GetValue<bool>() == true, "fusion quality failed");
    results.Record = run;
    var record = await http.GetFromJsonAsync<VehicleDimensionRecord>($"/_test/persisted/{workflow}", json);
    var profile = await http.GetFromJsonAsync<EmptyVehicleProfileRecord>($"/_test/profile/{plate}", json);
    var repository = new SmokeRepository(workflow, plate, record);
    var configuration = new ConfigurationBuilder().AddInMemoryCollection(new Dictionary<string, string?>
    { ["Database:Enabled"] = "true" }).Build();
    var factory = new SmokeWorkerFactory(results, json);
    var controller = new RadarLiveController(new RadarLiveClient(factory),
        repository, new SmokeProfiles(profile), new IntegrationQueryService(repository, configuration),
        configuration, NullLogger<RadarLiveController>.Instance);
    // An Analyzer success alone must not manufacture a completed database result.
    repository.Visible = false;
    var before = await Progress(controller, workflow);
    Require(before.Stage != "completed", "completion incorrectly inferred from Analyzer alone");
    if (run.HasDimensionPayload)
    {
        Require(before.Stage == "completed_unpersisted" && before.Terminal, "completed analysis must not keep waiting for persistence");
        Require(JsonNode.DeepEquals(before.Analysis?["dimensionPayload"], analyzerBody["integrationPayloads"]?["vehicleDimensions"]),
            "current calculation payload lost before page display");
    }
    repository.Visible = true;
    var after = await Progress(controller, workflow);
    Require(after.Stage == terminal && after.Terminal, $"unexpected terminal: {after.Stage}");
    if (terminal == "completed_without_dimensions")
    {
        Require(!run.HasDimensionPayload && run.Dimensions is null, "dimension payload unexpectedly emitted");
        Require(record is null && profile is null, "no-dimension case wrote a record or archive");
        Require(after.Dimensions is null && after.EmptyVehiclePointCloudUrl is null,
            "no-dimension progress exposed stale dimensions or archive");
        File.WriteAllText(Path.Combine(Path.GetDirectoryName(args[1])!, workflow + "-result.json"),
            JsonSerializer.Serialize(new
            {
                name = Read("name"),
                workflow,
                expected = terminal,
                actual = after.Stage,
                reason = run.MeasurementStatus?["reason"],
                hasDimensionPayload = run.HasDimensionPayload
            }, json));
        Console.WriteLine($"PASS {Read("name")}: {after.Stage}; no record or archive");
        continue;
    }
    Require(terminal == "completed", "unknown expected terminal");
    Require(run.Persistence?["status"]?.GetValue<string>() == "inserted", "direct-write acknowledgement missing");
    Require(run.HasDimensionPayload && record is not null && profile is not null, "missing dimensions/record/archive");
    if (record is null || profile is null)
        throw new Exception("SQL driver has no saved record or profile");
    Require(record.LicensePlate == plate, "confirmed workflow plate not used");
    var payload = analyzerBody["integrationPayloads"]!["vehicleDimensions"]!["dimensions"]!;
    Require(record.TrailerLength == payload["trailerLength"]!.GetValue<decimal>(), "saved length differs");
    Require(record.TrailerWidth == payload["trailerWidth"]!.GetValue<decimal>(), "saved width differs");
    Require(record.TrailerBodyHeight == payload["trailerBodyHeight"]!.GetValue<decimal>(), "saved height differs");
    Require(record.CabToTrailerFrontDist == payload["cabToTrailerFrontDist"]?.GetValue<decimal>(),
        "saved cab distance differs");
    var queried = (VehicleDimensionsDto)after.Dimensions!.Dimensions;
    var ribPayload = analyzerBody["integrationPayloads"]!["vehicleDimensions"]!;
    Require(record.TrailerBottomToGroundHeight == payload["trailerBottomToGroundHeight"]?.GetValue<decimal>()
        && record.HighLowPlateHeight == payload["highLowPlateHeight"]?.GetValue<decimal>()
        && record.MaximumWidthCm == ribPayload["maximumWidthCm"]?.GetValue<decimal>(),
        "saved floor height/step height/outer width differs");
    Require(after.Dimensions.MaximumWidthCm == record.MaximumWidthCm, "WebApi outer width differs");
    Require(record.HighLowPlates.Select(x => (x.PlateId, x.HighestToGround, x.LowestToGround, x.LengthCm)).SequenceEqual(
        ribPayload["highLowPlates"]!.AsArray().Select(x => (x!["plateId"]!.GetValue<int>(),
            x["highestToGround"]?.GetValue<decimal>(), x["lowestToGround"]?.GetValue<decimal>(), x["lengthCm"]?.GetValue<decimal>()))),
        "saved high-low plate data differs");
    Require(queried.HighLowPlates.Select(x => (x.PlateId, x.HighestToGround, x.LowestToGround, x.LengthCm)).SequenceEqual(
        record.HighLowPlates.Select(x => (x.PlateId, x.HighestToGround, x.LowestToGround, x.LengthCm))),
        "WebApi high-low plate data differs");
    Require(record.Wires.Select(x => (x.WireId, x.DistToPrev)).SequenceEqual(
        ribPayload["wires"]!.AsArray().Select(x => (x!["wireId"]!.GetValue<int>(), x["distToPrev"]?.GetValue<decimal>()))),
        "saved wire data differs");
    Require(queried.Wires.Select(x => (x.WireId, x.DistToPrev)).SequenceEqual(
        record.Wires.Select(x => (x.WireId, x.DistToPrev))), "WebApi wire data differs");
    Require(JsonNode.DeepEquals(run.Dimensions, analyzerBody["outputs"]?["dimensions"])
        && JsonNode.DeepEquals(run.MeasurementStatus, analyzerBody["outputs"]?["measurementStatus"])
        && JsonNode.DeepEquals(run.ReviewMeasurements, analyzerBody["outputs"]?["reviewMeasurements"]),
        "Worker changed Analyzer measurements or evidence");
    Require(record.ActualRibCount == ribPayload["actualRibCount"]?.GetValue<int>(), "saved rib count differs");
    Require(after.Dimensions.ActualRibCount == record.ActualRibCount, "WebApi rib count differs");
    Require(record.Ribs.Select(x => (x.RibId, x.DistToPrev)).SequenceEqual(
        ribPayload["ribs"]!.AsArray().Select(x => (x!["ribId"]!.GetValue<int>(), x["distToPrev"]?.GetValue<decimal>()))),
        "saved rib segments differ");
    Require(queried.Ribs.Select(x => (x.RibId, x.DistToPrev)).SequenceEqual(
        record.Ribs.Select(x => (x.RibId, x.DistToPrev))), "WebApi rib segments differ");
    if (expected["actualRibCount"] is JsonNode ribCount)
        Require(record.ActualRibCount == ribCount.GetValue<int>(), "case rib count regression");
    Require(queried.TrailerLength == record.TrailerLength
        && queried.CabToTrailerFrontDist == record.CabToTrailerFrontDist
        && queried.TrailerWidth == record.TrailerWidth
        && queried.TrailerBodyHeight == record.TrailerBodyHeight
        && queried.TrailerBottomToGroundHeight == record.TrailerBottomToGroundHeight
        && queried.HighLowPlateHeight == record.HighLowPlateHeight,
        "WebApi dimensions differ from saved record");
    Require(profile.WorkflowId == workflow, "saved profile workflow mismatch");
    Require(after.EmptyVehiclePointCloudUrl?.Contains(workflow) == true, "matching archive link missing");
    var pending = record.DetectStatus == "low_confidence";
    // Since the 2026-09-16 side-wall length line, ordinary carriages may also be pending
    // (Analyzer docs/side_wall_length_fix_20260916.md); branch by length method like verify_pair_http.
    var lengthMethod = run.MeasurementStatus?["lengthMeasurement"]?["method"]?.GetValue<string>();
    var sideLength = lengthMethod is "observed_lower_side_inner_edges" or "completed_lower_side_inner_edges";
    if (expected["review"] is JsonNode review)
        Require(after.Dimensions?.ReviewStatus == review.GetValue<string>(), "review status lost");
    if (pending && !sideLength)
    {
        Require(run.ReviewMeasurements?["carriageOpening"]?["applied"]?.GetValue<bool>() == true, "opening inference lost");
        Require(record.TrailerLength is > 538 and < 546 && record.TrailerWidth is > 222 and < 230,
            "new vehicle opening regression");
        Require(record.TrailerBodyHeight is > 260 and < 270, "inner height replaced by 2.9 m exterior");
    }
    if (expected["lengthMethod"] is JsonNode method)
        Require(lengthMethod == method.GetValue<string>(), "length source changed");
    factory.Offline = true;
    var offline = await Progress(controller, workflow);
    Require(offline.Stage == "completed" && (!pending || offline.Dimensions?.ReviewStatus == "pending"),
        "stored result must survive Worker offline");
    File.WriteAllText(Path.Combine(Path.GetDirectoryName(args[1])!, workflow + "-progress.json"),
        JsonSerializer.Serialize(new { before, after, offline }, json));
    var output = new
    {
        name = Read("name"),
        workflow,
        pending,
        record.TrailerLength,
        record.TrailerWidth,
        record.TrailerBodyHeight,
        record.CabToTrailerFrontDist,
        record.ActualRibCount,
        record.MaximumWidthCm,
        record.TrailerBottomToGroundHeight,
        record.HighLowPlateHeight,
        before = before.Stage,
        after = after.Stage,
        workerOffline = offline.Stage,
        dimensions = run.Dimensions,
        queriedDimensions = queried,
        review = run.ReviewMeasurements
    };
    File.WriteAllText(Path.Combine(Path.GetDirectoryName(args[1])!, workflow + "-result.json"), JsonSerializer.Serialize(output, json));
    // 先保存实际 SQL/WebApi 结果，长度失败时仍能核对其他字段；原断言不变。
    var lengthRange = expected["lengthCm"]!.AsArray();
    Require(record.TrailerLength >= lengthRange[0]!.GetValue<decimal>()
        && record.TrailerLength <= lengthRange[1]!.GetValue<decimal>(), "case length regression");
    foreach (var (key, value) in new (string, decimal?)[] {
        ("widthCm", record.TrailerWidth), ("cabDistanceCm", record.CabToTrailerFrontDist),
        ("heightCm", record.TrailerBodyHeight),
        ("bottomHeightCm", record.TrailerBottomToGroundHeight), ("stepHeightCm", record.HighLowPlateHeight) })
    {
        if (expected[key] is JsonArray range)
            Require(value >= range[0]!.GetValue<decimal>() && value <= range[1]!.GetValue<decimal>(),
                $"case {key} regression: {value}");
    }
    Console.WriteLine($"PASS {Read("name")}: {record.TrailerLength} x {record.TrailerWidth} x {record.TrailerBodyHeight} cm; pending={pending}");
}

static void Require(bool condition, string message) { if (!condition) throw new Exception(message); }
static async Task<CaptureProgressResponse> Progress(RadarLiveController controller, string workflow)
{
    var result = await controller.GetCapture(workflow, CancellationToken.None);
    return ((ApiResponse<CaptureProgressResponse>)((OkObjectResult)result.Result!).Value!).Data!;
}

sealed class SmokeRepository(string workflow, string plate, VehicleDimensionRecord? record) : NullIntegrationRepository, IIntegrationRepository
{
    public bool Visible { get; set; }
    public override VehicleArrivalRecord? GetVehicleArrivalBySessionId(string sessionId) => new()
    { SessionId = sessionId, LicensePlate = plate, PlateStatus = "SUCCEEDED", RadarStatus = "SUCCEEDED" };
    public VehicleDimensionRecord? GetVehicleDimensionByWorkflowId(string workflowId) => Visible && workflowId == workflow ? record : null;
}
sealed class SmokeProfiles(EmptyVehicleProfileRecord? profile) : IEmptyVehicleProfileRepository
{
    public EmptyVehicleProfileRecord? GetByLicensePlate(string value) => value == profile?.LicensePlate ? profile : null;
}
sealed class SmokeResults
{
    public AnalysisRecord? Record { get; set; }
    public AnalysisRecord? Find(string workflowId) => Record?.WorkflowId == workflowId ? Record : null;
}
sealed class CapturingHandler() : DelegatingHandler(new HttpClientHandler { UseProxy = false })
{
    public string? LastBody { get; private set; }
    protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
    {
        var response = await base.SendAsync(request, cancellationToken);
        LastBody = await response.Content.ReadAsStringAsync(cancellationToken);
        return response;
    }
}
sealed class SmokeWorkerFactory(SmokeResults store, JsonSerializerOptions json) : IHttpClientFactory
{
    public bool Offline { get; set; }
    public HttpClient CreateClient(string name) => new(new SmokeHandler(this, store, json)) { BaseAddress = new Uri("http://127.0.0.1:8000") };
}
sealed class SmokeHandler(SmokeWorkerFactory factory, SmokeResults store, JsonSerializerOptions json) : HttpMessageHandler
{
    protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
    {
        if (factory.Offline) return Task.FromResult(new HttpResponseMessage(HttpStatusCode.ServiceUnavailable));
        var query = Microsoft.AspNetCore.WebUtilities.QueryHelpers.ParseQuery(request.RequestUri!.Query);
        var workflow = query.TryGetValue("workflowId", out var value) ? value.ToString() : null;
        var body = workflow is null ? "{}" : JsonSerializer.Serialize(store.Find(workflow), json);
        return Task.FromResult(new HttpResponseMessage(HttpStatusCode.OK) { Content = new StringContent(body) });
    }
}
