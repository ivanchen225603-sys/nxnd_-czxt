using System.Net;
using System.Text;
using System.Text.Json.Nodes;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Logging.Abstractions;
using Xunit;

namespace CarPointCloudWebApi.Tests;

/// <summary>
/// 磅房实时点云接口的契约约束。
/// </summary>
/// <remarks>
/// 全部为无数据库用例：它们校验的是鉴权归属、审计归类、车牌规范化和阶段合成这四段
/// 纯逻辑，这些恰恰是最容易在重构中被悄悄改坏、而真实 DM8 测试又覆盖不到的地方。
/// </remarks>
public sealed class RadarLiveContractTests
{
    private const string TestRecorderToken = "test-only-recorder-token-0123456789abcdef";
    // ------------------------------------------------------------ 鉴权归属

    /// <summary>
    /// 未携带固定 Token 的 radar-live 请求必须要求 JWT。
    /// </summary>
    /// <remarks>
    /// 这条断言看起来显然，实际非常脆：JwtAuthMiddleware 的匿名白名单里有
    /// <c>/api/v1/radar</c>，而新路由叫 <c>/api/v1/radar-live</c>。两者只差一个连字符，
    /// 全靠 <c>StartsWithSegments</c> 按<b>整段</b>匹配才没有被放行。任何人把它改成
    /// <c>StartsWith</c>，或者把新路由挂到 <c>/api/v1/radar/live</c> 下，
    /// 都会让磅房的采集按钮变成匿名可调用——必须由测试钉死。
    /// </remarks>
    [Theory]
    [InlineData("/api/v1/radar-live/status")]
    [InlineData("/api/v1/radar-live/fused-frame")]
    [InlineData("/api/v1/radar-live/completed-frame")]
    [InlineData("/api/v1/radar-live/captures")]
    [InlineData("/api/v1/radar-live/captures/sess_test")]
    [InlineData("/api/v1/radar-live/captures/sess_test/cancel")]
    public async Task RadarLiveRoutes_RequireJwt(string path)
    {
        var context = await InvokeJwtMiddlewareAsync(path);

        Assert.Equal(StatusCodes.Status401Unauthorized, context.Response.StatusCode);
    }

    [Theory]
    [InlineData("GET", "/api/v1/radar-live/status", true)]
    [InlineData("POST", "/api/v1/radar-live/captures", true)]
    [InlineData("GET", "/api/v1/radar-live/captures/sess_test", true)]
    [InlineData("POST", "/api/v1/radar-live/captures/sess_test/cancel", true)]
    [InlineData("GET", "/api/v1/radar-live/fused-frame", false)]
    [InlineData("GET", "/api/v1/radar-live/completed-frame", false)]
    [InlineData("GET", "/api/v1/radar-live/captures", false)]
    [InlineData("POST", "/api/v1/radar-live/status", false)]
    public async Task RecorderToken_OnlyAllowsCaptureWorkflowRoutes(string method, string path, bool allowed)
    {
        var context = await InvokeJwtMiddlewareAsync(path, method, TestRecorderToken);
        Assert.Equal(allowed ? StatusCodes.Status200OK : StatusCodes.Status401Unauthorized,
                     context.Response.StatusCode);
    }

    [Theory]
    [InlineData("")]
    [InlineData("wrong-token")]
    [InlineData(TestRecorderToken + " ")]
    public async Task RecorderToken_RejectsMissingOrIncorrectValue(string value)
    {
        var context = await InvokeJwtMiddlewareAsync(
            "/api/v1/radar-live/captures", HttpMethods.Post, value);
        Assert.Equal(StatusCodes.Status401Unauthorized, context.Response.StatusCode);
    }

    /// <summary>既有的匿名来车事件接口不受新路由影响。</summary>
    [Theory]
    [InlineData("/api/v1/radar/vehicle-arrival")]
    [InlineData("/api/v1/coalVolumes")]
    public async Task ExistingAnonymousRoutes_StayAnonymous(string path)
    {
        var context = await InvokeJwtMiddlewareAsync(path);

        Assert.Equal(StatusCodes.Status200OK, context.Response.StatusCode);
    }

    private static async Task<DefaultHttpContext> InvokeJwtMiddlewareAsync(
        string path, string method = "GET", string? recorderToken = null,
        string? configuredToken = TestRecorderToken, bool duplicateHeader = false)
    {
        var nextInvoked = false;
        var middleware = new JwtAuthMiddleware(
            context =>
            {
                nextInvoked = true;
                context.Response.StatusCode = StatusCodes.Status200OK;
                return Task.CompletedTask;
            },
            new JwtTokenService(Microsoft.Extensions.Options.Options.Create(new JwtOptions
            {
                JwtSecret = "radar-live-contract-test-secret-0123456789",
                TokenExpireMinutes = 60
            })));

        using var services = new ServiceCollection()
            .AddSingleton<IConfiguration>(new ConfigurationBuilder().AddInMemoryCollection(
                new Dictionary<string, string?> { ["RadarLive:RecorderToken"] = configuredToken })
                .Build()).BuildServiceProvider();
        var context = new DefaultHttpContext { RequestServices = services };
        context.Request.Path = path;
        context.Request.Method = method;
        if (!string.IsNullOrEmpty(recorderToken))
            context.Request.Headers[RadarLiveRecorderAccess.HeaderName] = recorderToken;
        if (duplicateHeader)
            context.Request.Headers.Append(RadarLiveRecorderAccess.HeaderName, recorderToken);
        context.Response.Body = new MemoryStream();

        await middleware.InvokeAsync(context);
        _ = nextInvoked;
        return context;
    }

    // ------------------------------------------------------------ 审计归类

    [Fact]
    public void RecorderToken_IsRedactedFromAuditHeaders()
    {
        var headers = new HeaderDictionary { [RadarLiveRecorderAccess.HeaderName] = TestRecorderToken };
        var method = typeof(ApiAccessLogMiddleware).GetMethod("SerializeHeaders",
            System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.NonPublic)!;
        var serialized = Assert.IsType<string>(method.Invoke(null, new object[] { headers }));
        Assert.DoesNotContain(TestRecorderToken, serialized);
        Assert.Contains("***REDACTED***", serialized);
    }

    [Fact]
    public async Task RecorderToken_UsesConfiguredValueAndRejectsOldValueAfterRotation()
    {
        const string rotated = "rotated-test-only-recorder-token-0123456789";
        var denied = await InvokeJwtMiddlewareAsync(
            "/api/v1/radar-live/status", recorderToken: TestRecorderToken, configuredToken: rotated);
        var accepted = await InvokeJwtMiddlewareAsync(
            "/api/v1/radar-live/status", recorderToken: rotated, configuredToken: rotated);
        Assert.Equal(StatusCodes.Status401Unauthorized, denied.Response.StatusCode);
        Assert.Equal(StatusCodes.Status200OK, accepted.Response.StatusCode);
    }

    /// <summary>
    /// 手动采集与第三方来车归入同一审计类型；进度查询不进审计。
    /// </summary>
    /// <remarks>
    /// 进度查询在采集期间会被页面按秒轮询。一旦它被纳入审计，ApiAccessLog 会被单次
    /// 采集写进几十条无状态变化的记录，把真正的交互淹没掉。
    /// </remarks>
    [Theory]
    [InlineData("/api/v1/radar-live/captures", "VehicleArrival")]
    [InlineData("/api/v1/radar/vehicle-arrival", "VehicleArrival")]
    [InlineData("/api/v1/radar-live/captures/sess_test", null)]
    [InlineData("/api/v1/radar-live/status", null)]
    [InlineData("/api/v1/radar-live/fused-frame", null)]
    [InlineData("/api/v1/radar-live/completed-frame", null)]
    public void ResolveIntegrationType_ClassifiesRadarLiveRoutes(string path, string? expected)
    {
        Assert.Equal(expected, IntegrationAudit.ResolveIntegrationType(path));
    }

    // -------------------------------------------------------- 车牌规范化

    [Theory]
    [InlineData("宁A10333F", "宁A10333F")]
    [InlineData("  宁a10333f  ", "宁A10333F")]
    [InlineData("宁A 10333 F", "宁A10333F")]
    public void ManualPlate_IsNormalizedToUppercaseWithoutWhitespace(string input, string expected)
    {
        Assert.True(ManualLicensePlate.TryNormalize(input, out var plate, out _));
        Assert.Equal(expected, plate);
    }

    /// <summary>
    /// 占位车牌必须在入口就被拒绝。
    /// </summary>
    /// <remarks>
    /// Analyzer 落库时对车牌只检查长度和控制字符：<c>UNKNOWN</c> 会被当成真实车牌写进
    /// VehicleDimensionRecord，并随合作方的 <c>GET /api/v1/vehicleDimensions</c> 发出去。
    /// 手动输入是这条链路上唯一由人直接决定车牌的地方，必须在这里挡住。
    /// </remarks>
    [Theory]
    [InlineData("UNKNOWN")]
    [InlineData("unknown")]
    [InlineData("N/A")]
    [InlineData("无车牌")]
    [InlineData("   ")]
    [InlineData("")]
    [InlineData(null)]
    public void ManualPlate_RejectsPlaceholders(string? input)
    {
        Assert.False(ManualLicensePlate.TryNormalize(input, out _, out var error));
        Assert.NotEmpty(error);
    }

    [Fact]
    public void ManualPlate_RejectsOverlongValue()
    {
        var input = new string('粤', ManualLicensePlate.MaxLength + 1);

        Assert.False(ManualLicensePlate.TryNormalize(input, out _, out var error));
        Assert.Contains("长度", error);
    }

    // -------------------------------------------------------- 阶段合成

    private static JsonNode PendingReviewAnalysis() => JsonNode.Parse("""
        {"stage":"completed","hasDimensionPayload":false,
         "measurementStatus":{"reviewStatus":"pending","calculationSucceeded":true},
         "loadStatus":{"status":"Uncertain"},
         "reviewMeasurements":{"unit":"m","dimensions":{"carriage":{
             "length":5.958,"width":2.561,"height":2.645}},
             "cabToTrailerFrontDist":8.685,"ribCount":null,"ribs":[],
             "syntheticPointCount":15932}}
        """)!;

    [Fact]
    public async Task CaptureEndpoint_PreservesWorkerReviewDataWithoutRedispatch()
    {
        var repository = new RecordingArrivalRepository(Arrival("wf-review"));
        var analysis = PendingReviewAnalysis();
        var radar = new CountingHandler(request => new HttpResponseMessage(HttpStatusCode.OK)
        {
            Content = new StringContent(
                request.RequestUri!.AbsolutePath.Contains("analysis", StringComparison.Ordinal)
                    ? analysis.ToJsonString()
                    : new JsonObject { ["success"] = true, ["known"] = true,
                        ["scan"] = Scan("wf-review", outcome: "completed", persistenceState: "saved",
                            persisted: ["CAM_004", "CAM_005"]) }.ToJsonString(),
                Encoding.UTF8, "application/json")
        });
        var response = await CreateController(repository, radar).GetCapture("wf-review", CancellationToken.None);
        var payload = Assert.IsType<ApiResponse<CaptureProgressResponse>>(
            Assert.IsType<OkObjectResult>(response.Result).Value);
        Assert.Equal(CaptureStages.PendingReview, payload.Data!.Stage);
        Assert.True(payload.Data.Terminal);
        Assert.True(JsonNode.DeepEquals(analysis["reviewMeasurements"], payload.Data.Analysis!["reviewMeasurements"]));
        Assert.Null(payload.Data.Dimensions);
        Assert.Equal(0, radar.PostCount);
    }

    [Fact]
    public void Progress_CompletedReviewStopsWaitingWithoutClaimingDatabasePersistence()
    {
        var analysis = PendingReviewAnalysis();
        var progress = CaptureProgressBuilder.Build("wf-review", Arrival("wf-review"),
            Scan("wf-review", outcome: "completed", persistenceState: "saved",
                persisted: ["CAM_004", "CAM_005"]), analysis, null);

        Assert.Equal(CaptureStages.PendingReview, progress.Stage);
        Assert.True(progress.Terminal);
        Assert.Null(progress.Dimensions);
        Assert.Contains("待人工审核", progress.Message);
        Assert.Contains("尚未入库", progress.Message);
        Assert.Equal(2, progress.PersistedCameras.Count);
        Assert.True(JsonNode.DeepEquals(analysis, progress.Analysis));
        analysis["reviewMeasurements"]!["cabToTrailerFrontDist"] = 0;
        Assert.Equal(8.685, progress.Analysis!["reviewMeasurements"]!["cabToTrailerFrontDist"]!.GetValue<double>());
    }

    [Theory]
    [InlineData("Loaded")]
    [InlineData("Uncertain")]
    public void Progress_RestoredNoDimensionOutcomeEndsOldWorkflowAfterWorkerStartsNewScan(string load)
    {
        // Worker 从 outcomes 恢复后仍返回原分析契约；scan 快照可以已属于下一辆车。
        var restored = JsonNode.Parse("""
            {"workflowId":"wf-original","pairId":"pair-original","stage":"completed",
             "hasDimensionPayload":false,"measurementStatus":{"valid":false},"loadStatus":{}}
            """)!;
        restored["loadStatus"]!["status"] = load;
        var progress = CaptureProgressBuilder.Build("wf-original", Arrival("wf-original"),
            Scan("wf-next", outcome: "completed", persistenceState: "saved", persisted: ["CAM_004", "CAM_005"]),
            JsonNode.Parse(restored.ToJsonString()), null);
        Assert.Equal(CaptureStages.CompletedWithoutDimensions, progress.Stage);
        Assert.True(progress.Terminal);
        Assert.Null(progress.Dimensions);
        Assert.Empty(progress.PersistedCameras);
        Assert.Equal(load, progress.Analysis!["loadStatus"]!["status"]!.GetValue<string>());
    }

    [Theory]
    [InlineData("2.3876")]
    [InlineData("null")]
    public void Progress_PartialReviewShowsAvailableFieldsAndMissingReasonsWithoutRequiringThreeDimensions(string width)
    {
        var analysis = PendingReviewAnalysis();
        analysis["reviewMeasurements"] = JsonNode.Parse("""
            {"unit":"m","partial":true,"calculationSucceeded":true,"reviewStatus":"pending",
             "dimensions":{"carriage":{"length":null,"width":null,"height":null}},
             "fields":{"dimensions.carriage.length":{"value":null,"reason":"inner_wall_boundaries_unverified"}},
             "ribCount":null,"ribIntervals":null}
            """);
        analysis["reviewMeasurements"]!["dimensions"]!["carriage"]!["width"] = JsonNode.Parse(width);
        var progress = CaptureProgressBuilder.Build("wf-review", Arrival("wf-review"), null, analysis, null);
        Assert.Equal(width == "null" ? CaptureStages.CompletedWithoutDimensions : CaptureStages.PendingReview, progress.Stage);
        Assert.True(progress.Terminal);
        Assert.Null(progress.Dimensions);
        Assert.Contains("缺测原因", progress.Message);
        Assert.Contains(width == "null" ? "未生成完整尺寸入库记录" : "尚未入库", progress.Message);
        Assert.DoesNotContain("完整计算数据", progress.Message);
        Assert.True(JsonNode.DeepEquals(analysis["reviewMeasurements"], progress.Analysis!["reviewMeasurements"]));
    }

    [Theory]
    [InlineData("Loaded")]
    [InlineData("Uncertain")]
    public void Progress_PartialDisplayDoesNotRequireBusinessMeasurementSuccessOrChangeLoadState(string load)
    {
        var analysis = AnalysisWithoutDimensions(load, "vehicle_not_empty");
        analysis["measurementStatus"]!["calculationSucceeded"] = false;
        analysis["reviewMeasurements"] = JsonNode.Parse("""
            {"unit":"m","partial":true,"calculationSucceeded":true,"reviewStatus":"pending",
             "dimensions":{"vehicle":{"length":15.7},"carriage":{"length":null,"width":null,"height":null}},
             "fields":{"dimensions.carriage.height":{"value":null,"reason":"current_capture_not_confirmed_empty"}}}
            """);
        var progress = CaptureProgressBuilder.Build("wf-review", Arrival("wf-review"), null, analysis, null);
        Assert.Equal(CaptureStages.PendingReview, progress.Stage);
        Assert.Equal(load, progress.Analysis!["loadStatus"]!["status"]!.GetValue<string>());
        Assert.False(progress.Analysis["measurementStatus"]!["calculationSucceeded"]!.GetValue<bool>());
        Assert.Null(progress.Dimensions);
        Assert.Contains("尚未入库", progress.Message);
    }

    [Fact]
    public void Progress_DatabaseRecordWinsOverLegacyNoDimensionAnalysis()
    {
        var analysis = AnalysisWithoutDimensions("Empty", "inner_wall_boundaries_unverified");
        var progress = CaptureProgressBuilder.Build("wf-review", Arrival("wf-review"), null,
            analysis, new VehicleDimensionDto { WorkflowId = "wf-review", ReviewStatus = "pending" });
        Assert.Equal(CaptureStages.Completed, progress.Stage);
        Assert.Contains("尺寸已写入数据库", progress.Message);
    }

    [Fact]
    public void Progress_DatabaseAndFailureTakePriorityOverReview()
    {
        var analysis = PendingReviewAnalysis();
        var stored = CaptureProgressBuilder.Build("wf-review", Arrival("wf-review"), null,
            analysis, new VehicleDimensionDto { WorkflowId = "wf-review" });
        Assert.Equal(CaptureStages.Completed, stored.Stage);
        analysis["stage"] = "failed";
        analysis["permanentFailure"] = true;
        var failed = CaptureProgressBuilder.Build("wf-review", Arrival("wf-review"), null, analysis, null);
        Assert.Equal(CaptureStages.AnalysisFailed, failed.Stage);
    }

    [Fact]
    public void Progress_ReviewDoesNotOverrideExpectedDatabasePayload()
    {
        var analysis = PendingReviewAnalysis();
        analysis["hasDimensionPayload"] = true;
        var progress = CaptureProgressBuilder.Build("wf-review", Arrival("wf-review"), null, analysis, null);
        Assert.Equal(CaptureStages.CompletedUnpersisted, progress.Stage);
        Assert.True(progress.Terminal);
        analysis["hasDimensionPayload"] = false;
        analysis["measurementStatus"]!["calculationSucceeded"] = false;
        progress = CaptureProgressBuilder.Build("wf-review", Arrival("wf-review"), null, analysis, null);
        Assert.NotEqual(CaptureStages.PendingReview, progress.Stage);
    }

    [Fact]
    public void Progress_IsCompletedOnlyWhenDatabaseRecordExists()
    {
        // Analyzer 已返回成功但 DM8 里还查不到：结束计算等待，但绝不能宣称已入库。
        var analyzed = CaptureProgressBuilder.Build(
            "wf-1",
            Arrival("wf-1"),
            Scan("wf-1", outcome: "completed", persistenceState: "saved", persisted: ["CAM_004", "CAM_005"]),
            Analysis("completed"),
            dimensions: null);

        Assert.Equal(CaptureStages.CompletedUnpersisted, analyzed.Stage);
        Assert.True(analyzed.Terminal);

        var persisted = CaptureProgressBuilder.Build(
            "wf-1",
            Arrival("wf-1"),
            Scan("wf-1", outcome: "completed", persistenceState: "saved", persisted: ["CAM_004", "CAM_005"]),
            Analysis("completed"),
            new VehicleDimensionDto { WorkflowId = "wf-1" });

        Assert.Equal(CaptureStages.Completed, persisted.Stage);
        Assert.True(persisted.Terminal);
    }

    [Fact]
    public void Progress_LegacyToleranceDoesNotReplaceDatabaseEvidenceOrReview()
    {
        var analysis = Analysis("completed");
        analysis["hasDimensionPayload"] = true;
        analysis["measurementStatus"] = JsonNode.Parse("""{"reviewStatus":"pending","calculationPolicy":{"approved":true,"thresholdM":1.0}}""");
        analysis["loadStatus"] = JsonNode.Parse("""{"status":"Uncertain"}""");
        var waiting = CaptureProgressBuilder.Build("wf-policy", Arrival("wf-policy"), null, analysis, null);
        Assert.True(waiting.Terminal);
        var saved = CaptureProgressBuilder.Build("wf-policy", Arrival("wf-policy"), null, analysis,
            new VehicleDimensionDto { WorkflowId = "wf-policy", ReviewStatus = "pending" });
        Assert.Equal(CaptureStages.Completed, saved.Stage);
        Assert.True(saved.Terminal);
        Assert.DoesNotContain("业务高度容差", saved.Message);
        Assert.Contains("待人工审核", saved.Message);
        Assert.Contains("已写入数据库", saved.Message);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void Progress_LowerSideLengthReviewStillRequiresDatabaseEvidenceAndPreservesDiagnostics(bool upperReferenceComplete)
    {
        // 普通车型的待审核原因来自端缘敏感性/同侧筋位投影，不是矩形模型或1m容差。
        var analysis = Analysis("completed");
        analysis["hasDimensionPayload"] = true;
        analysis["loadStatus"] = JsonNode.Parse("""{"status":"Empty"}""");
        analysis["dimensions"] = JsonNode.Parse("""
            {"carriage":{"length":10.72,"width":2.35,"height":2.1}}
            """);
        analysis["measurementStatus"] = JsonNode.Parse("""
            {"valid":true,"reviewStatus":"pending","calculationSucceeded":true,
             "requiresManualReview":true,
             "lengthMeasurement":{"valid":true,"method":"observed_lower_side_inner_edges",
               "selectedSide":"right","heightAboveFloorM":[0.15,0.35],
               "lengthM":10.72,"ribPositionsValid":true,"requiresManualReview":true,
               "lowerEndpoints":{"front":{"valid":true,"positionM":10.82},
                                 "rear":{"valid":true,"positionM":0.1}},
               "ribIntervalsM":[3.66,3.55,3.51],
               "upperReference":{"endpoints":{"front":{"valid":true,"positionM":10.56},
                                               "rear":{"valid":true,"positionM":0.1}},
                                 "lengthM":10.46,"ribIntervalsM":[3.4,3.55,3.51]},
               "ribProjections":[
                 {"ribId":1,"valid":true,"positionM":7.16,"extrapolationM":0.04,
                  "observedWRangeM":[0.7,1.0],"supportCellCount":7,
                  "supportCellsWLH":[[0.7,7.16,1.55],[0.75,7.16,1.55],[0.8,7.16,1.55],
                                     [0.85,7.16,1.55],[0.9,7.16,1.55],[0.95,7.16,1.55],[1.0,7.16,1.55]],
                  "intersectionWLH":[1.04,7.16,1.55],"referencePlaneWLH":[0.0,0.0,1.04],
                  "residualP90M":[0.011,0.008],"fitSensitivityM":[0.007,0.006]},
                 {"ribId":2,"valid":true,"positionM":3.61,"extrapolationM":0.02,
                  "observedWRangeM":[0.72,1.02],"supportCellCount":7,
                  "supportCellsWLH":[[0.72,3.61,1.54],[0.77,3.61,1.54],[0.82,3.61,1.54],
                                     [0.87,3.61,1.54],[0.92,3.61,1.54],[0.97,3.61,1.54],[1.02,3.61,1.54]],
                  "intersectionWLH":[1.04,3.61,1.54],"referencePlaneWLH":[0.0,0.0,1.04],
                  "residualP90M":[0.012,0.009],"fitSensitivityM":[0.008,0.007]}]}}
            """);
        var sourceLength = analysis["measurementStatus"]!["lengthMeasurement"]!;
        var sourceUpper = sourceLength["upperReference"]!;
        if (!upperReferenceComplete)
        {
            sourceUpper["endpoints"]!["rear"]!["valid"] = false;
            sourceUpper["endpoints"]!["rear"]!["positionM"] = null;
            sourceUpper["lengthM"] = null;
            sourceUpper["ribIntervalsM"]![2] = null;
        }
        var expectedAnalysis = analysis.DeepClone();
        var waiting = CaptureProgressBuilder.Build("wf-side-length", Arrival("wf-side-length"),
            null, analysis, null);
        Assert.Equal(CaptureStages.CompletedUnpersisted, waiting.Stage);
        Assert.True(waiting.Terminal);
        Assert.Null(waiting.Dimensions);
        Assert.True(JsonNode.DeepEquals(expectedAnalysis, waiting.Analysis));

        var saved = CaptureProgressBuilder.Build("wf-side-length", Arrival("wf-side-length"),
            null, analysis, new VehicleDimensionDto
            {
                WorkflowId = "wf-side-length",
                ReviewStatus = "pending"
            });
        Assert.Equal(CaptureStages.Completed, saved.Stage);
        Assert.True(saved.Terminal);
        Assert.Contains("已写入数据库", saved.Message);
        Assert.Contains("待人工审核", saved.Message);
        Assert.Equal("pending", saved.Dimensions!.ReviewStatus);
        Assert.Equal("pending", saved.Analysis!["measurementStatus"]!["reviewStatus"]!.GetValue<string>());
        Assert.True(JsonNode.DeepEquals(expectedAnalysis, saved.Analysis));
        Assert.True(JsonNode.DeepEquals(analysis["measurementStatus"], saved.Analysis!["measurementStatus"]));
        Assert.Null(saved.Analysis["reviewMeasurements"]);

        var preserved = saved.Analysis["measurementStatus"]!["lengthMeasurement"]!;
        var preservedUpper = preserved["upperReference"]!;
        Assert.Equal(10.82, preserved["lowerEndpoints"]!["front"]!["positionM"]!.GetValue<double>());
        Assert.Equal(10.72, preserved["lengthM"]!.GetValue<double>());
        Assert.Equal(new[] { 3.66, 3.55, 3.51 }, preserved["ribIntervalsM"]!.AsArray().Select(value => value!.GetValue<double>()));
        Assert.Equal(10.56, preservedUpper["endpoints"]!["front"]!["positionM"]!.GetValue<double>());
        Assert.Equal(upperReferenceComplete, preservedUpper["endpoints"]!["rear"]!["valid"]!.GetValue<bool>());
        Assert.Equal(upperReferenceComplete ? 0.1 : (double?)null, preservedUpper["endpoints"]!["rear"]!["positionM"]?.GetValue<double>());
        Assert.Equal(upperReferenceComplete ? 10.46 : (double?)null, preservedUpper["lengthM"]?.GetValue<double>());
        Assert.Equal(new double?[] { 3.4, 3.55, upperReferenceComplete ? 3.51 : null },
            preservedUpper["ribIntervalsM"]!.AsArray().Select(value => value?.GetValue<double>()));
        var expectedPositions = new[] { 7.16, 3.61 };
        var expectedHeights = new[] { 1.55, 1.54 };
        var expectedWidths = new[]
        {
            new[] { 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0 },
            new[] { 0.72, 0.77, 0.82, 0.87, 0.92, 0.97, 1.02 }
        };
        var expectedResiduals = new[] { new[] { 0.011, 0.008 }, new[] { 0.012, 0.009 } };
        var expectedSensitivities = new[] { new[] { 0.007, 0.006 }, new[] { 0.008, 0.007 } };
        Assert.Equal(2, preserved["ribProjections"]!.AsArray().Count);
        for (var i = 0; i < expectedPositions.Length; i++)
        {
            var projection = preserved["ribProjections"]![i]!;
            Assert.Equal(expectedPositions[i], projection["positionM"]!.GetValue<double>());
            Assert.Equal(i == 0 ? 0.04 : 0.02, projection["extrapolationM"]!.GetValue<double>());
            Assert.Equal(new[] { expectedWidths[i][0], expectedWidths[i][6] },
                projection["observedWRangeM"]!.AsArray().Select(value => value!.GetValue<double>()));
            Assert.Equal(7, projection["supportCellCount"]!.GetValue<int>());
            var cells = projection["supportCellsWLH"]!.AsArray();
            Assert.Equal(7, cells.Count);
            for (var cell = 0; cell < cells.Count; cell++)
            {
                Assert.Equal(new[] { expectedWidths[i][cell], expectedPositions[i], expectedHeights[i] },
                    cells[cell]!.AsArray().Select(value => value!.GetValue<double>()));
            }
            Assert.Equal(new[] { 1.04, expectedPositions[i], expectedHeights[i] },
                projection["intersectionWLH"]!.AsArray().Select(value => value!.GetValue<double>()));
            Assert.Equal(new[] { 0.0, 0.0, 1.04 },
                projection["referencePlaneWLH"]!.AsArray().Select(value => value!.GetValue<double>()));
            Assert.Equal(expectedResiduals[i], projection["residualP90M"]!.AsArray().Select(value => value!.GetValue<double>()));
            Assert.Equal(expectedSensitivities[i], projection["fitSensitivityM"]!.AsArray().Select(value => value!.GetValue<double>()));
        }

        sourceLength["lowerEndpoints"]!["front"]!["positionM"] = 999.0;
        sourceLength["ribIntervalsM"]![0] = 999.0;
        sourceUpper["endpoints"]!["rear"]!["valid"] = !upperReferenceComplete;
        sourceUpper["endpoints"]!["rear"]!["positionM"] = 999.0;
        sourceUpper["lengthM"] = 999.0;
        sourceUpper["ribIntervalsM"]![2] = 999.0;
        foreach (var projection in sourceLength["ribProjections"]!.AsArray())
        {
            projection!["positionM"] = 999.0;
            projection["supportCellCount"] = 999;
            projection["supportCellsWLH"]![0]![1] = 999.0;
            projection["intersectionWLH"]![1] = 999.0;
            projection["referencePlaneWLH"]![2] = 999.0;
            projection["residualP90M"]![0] = 999.0;
            projection["fitSensitivityM"]![1] = 999.0;
        }
        Assert.False(JsonNode.DeepEquals(expectedAnalysis, analysis));
        Assert.True(JsonNode.DeepEquals(expectedAnalysis, saved.Analysis));
        Assert.True(JsonNode.DeepEquals(expectedAnalysis, waiting.Analysis));
    }

    [Fact]
    public void Progress_ExplainsMissingLowerSideInnerEdgesWithoutClaimingVehicleLoaded()
    {
        var analysis = AnalysisWithoutDimensions("Empty", "longitudinal_inner_edges_unavailable");
        analysis["measurementStatus"]!["lengthMeasurement"] = JsonNode.Parse("""
            {"valid":false,"reason":"lower_inner_edges_unavailable","ribProjections":null}
            """);
        var progress = CaptureProgressBuilder.Build("wf-side-missing", Arrival("wf-side-missing"),
            Scan("wf-side-missing", outcome: "completed", persistenceState: "saved"),
            analysis, dimensions: null);

        Assert.Equal(CaptureStages.CompletedWithoutDimensions, progress.Stage);
        Assert.True(progress.Terminal);
        Assert.Contains("完整侧壁下部内缘缺测", progress.Message);
        Assert.Contains("无法可靠计算内长", progress.Message);
        Assert.DoesNotContain("车厢非空", progress.Message);
        Assert.DoesNotContain("车厢载货", progress.Message);
        Assert.DoesNotContain("空车时重新采集", progress.Message);
        Assert.Contains("已计算项目", progress.Message);
        Assert.Null(progress.Dimensions);
    }

    /// <summary>
    /// 分析成功但按算法结论不产出尺寸时，必须给出终局而不是一直等数据库记录。
    /// </summary>
    /// <remarks>
    /// 载货、载荷不确定或端壁内表面未通过时，Analyzer 正常返回却不写
    /// VehicleDimensionRecord。把它当成中间态，页面就会永远停在"计算中"。
    /// </remarks>
    [Fact]
    public void Progress_TerminatesWhenAnalysisProducesNoDimensions()
    {
        var progress = CaptureProgressBuilder.Build(
            "wf-nodim",
            Arrival("wf-nodim"),
            Scan("wf-nodim", outcome: "completed", persistenceState: "saved"),
            AnalysisWithoutDimensions("Loaded", "vehicle_not_empty"),
            dimensions: null);

        Assert.Equal(CaptureStages.CompletedWithoutDimensions, progress.Stage);
        Assert.True(progress.Terminal);
        Assert.Contains("车厢非空", progress.Message);
    }

    /// <summary>有尺寸载荷但数据库未查到时结束等待，保留未入库状态。</summary>
    [Fact]
    public void Progress_StopsWaitingWithoutClaimingPersistenceWhenDimensionPayloadExists()
    {
        var analysis = Analysis("completed");
        analysis["hasDimensionPayload"] = true;

        var progress = CaptureProgressBuilder.Build(
            "wf-wait",
            Arrival("wf-wait"),
            Scan("wf-wait", outcome: "completed", persistenceState: "saved"),
            analysis,
            dimensions: null);

        Assert.Equal(CaptureStages.CompletedUnpersisted, progress.Stage);
        Assert.True(progress.Terminal);
    }

    /// <summary>载荷不确定时的说明必须能让操作员判断是否要重扫。</summary>
    [Theory]
    [InlineData(null)]
    [InlineData("vehicle_not_empty")]
    [InlineData("load_status_uncertain")]
    public void Progress_ExplainsUncertainLoadStatus(string? reason)
    {
        var progress = CaptureProgressBuilder.Build(
            "wf-uncertain",
            Arrival("wf-uncertain"),
            Scan("wf-uncertain", outcome: "completed", persistenceState: "saved"),
            AnalysisWithoutDimensions("Uncertain", reason),
            dimensions: null);

        Assert.Equal(CaptureStages.CompletedWithoutDimensions, progress.Stage);
        Assert.True(progress.Terminal);
        Assert.Contains("载荷状态不确定", progress.Message);
    }

    [Fact]
    public void Progress_DistinguishesCapturedFromSaved()
    {
        // 只收到点云、尚未确认落盘时不能显示成"已保存"：这正是最容易让现场
        // 以为数据已经安全的一步。
        var captured = CaptureProgressBuilder.Build(
            "wf-2",
            Arrival("wf-2"),
            Scan("wf-2", outcome: "inprogress", persistenceState: "pending", captured: ["CAM_004", "CAM_005"]),
            analysis: null,
            dimensions: null);

        Assert.Equal(CaptureStages.Captured, captured.Stage);

        var saved = CaptureProgressBuilder.Build(
            "wf-2",
            Arrival("wf-2"),
            Scan(
                "wf-2",
                outcome: "completed",
                persistenceState: "saved",
                captured: ["CAM_004", "CAM_005"],
                persisted: ["CAM_004", "CAM_005"]),
            analysis: null,
            dimensions: null);

        Assert.Equal(CaptureStages.Saved, saved.Stage);
    }

    [Fact]
    public void Progress_SingleSidedPersistenceIsNotSaved()
    {
        var progress = CaptureProgressBuilder.Build(
            "wf-3",
            Arrival("wf-3"),
            Scan(
                "wf-3",
                outcome: "inprogress",
                persistenceState: "saved",
                captured: ["CAM_004"],
                persisted: ["CAM_004"]),
            analysis: null,
            dimensions: null);

        Assert.Equal(CaptureStages.Captured, progress.Stage);
        Assert.Contains("等待另一台", progress.Message);
    }

    /// <summary>
    /// 雷达已经开始下一辆车的会话时，不得把别人的快照当成本流程的进度。
    /// </summary>
    [Fact]
    public void Progress_IgnoresScanSnapshotFromAnotherWorkflow()
    {
        var progress = CaptureProgressBuilder.Build(
            "wf-7",
            Arrival("wf-7"),
            Scan("wf-OTHER", outcome: "inprogress", captured: ["CAM_004", "CAM_005"]),
            analysis: null,
            dimensions: null);

        Assert.Equal(CaptureStages.Unknown, progress.Stage);
        Assert.Empty(progress.CapturedCameras);
    }

    // ------------------------------------------------- 预览帧缓存与扇出

    /// <summary>不同版本的调用方不会被别人的否定应答挡住。</summary>
    [Fact]
    public async Task FusedFrame_DifferentVersionStillReachesWorker()
    {
        var handler = new CountingHandler(request =>
            request.RequestUri!.Query.Contains("afterVersion=7")
                ? NoUpdateResponse(version: 7)
                : FrameResponse(version: 7));
        var client = new RadarLiveClient(
            new StubHttpClientFactory(handler));

        await client.GetFusedFrameAsync(7, CancellationToken.None);
        var behind = await client.GetFusedFrameAsync(3, CancellationToken.None);

        Assert.True(behind.Body!["updated"]!.GetValue<bool>());
        Assert.Equal(2, handler.CallCount);
    }

    /// <summary>
    /// 取消必须把 workflowId 交给 Worker 判定，不能自己先查再停。
    /// </summary>
    /// <remarks>
    /// "先查当前 requestId、再发不带参数的停止"两步之间随时可能换车，
    /// 那样停掉的是下一辆车正在进行的采集，而且返回值看起来完全成功。
    /// 因此停止请求必须自带 requestId，由 Worker 在协调器锁内原子校验。
    /// </remarks>
    [Fact]
    public async Task StopScan_SendsWorkflowIdForAtomicOwnershipCheck()
    {
        string? capturedQuery = null;
        var handler = new CountingHandler(request =>
        {
            capturedQuery = request.RequestUri!.Query;
            return new HttpResponseMessage(HttpStatusCode.OK)
            {
                Content = new StringContent("{\"success\":true}", Encoding.UTF8, "application/json")
            };
        });
        var client = new RadarLiveClient(
            new StubHttpClientFactory(handler));

        await client.StopScanAsync("sess_abc", CancellationToken.None);

        Assert.Equal(1, handler.CallCount);
        Assert.Contains("requestId=sess_abc", capturedQuery);
    }

    // ------------------------------------------------ 幂等重放的派发语义

    /// <summary>
    /// 已确认受理的重放不得再向 Worker 发指令。
    /// </summary>
    /// <remarks>
    /// Worker 只在进程内记住最近若干个 requestId，重启或淘汰后同一个编号会被当成新请求，
    /// 于是再采一次、再写一条记录。因此"是否已经派发过"必须由数据库里的 RadarStatus 回答，
    /// 不能依赖 Worker 的内存去重。
    /// </remarks>
    [Fact]
    public async Task Replay_OfAcceptedCapture_DoesNotDispatchAgain()
    {
        var repository = new RecordingArrivalRepository(
            Arrival("wf-1", VehicleArrivalRadarStatus.Succeeded));
        var radar = new CountingHandler(_ => throw new InvalidOperationException(
            "已受理的重放不应再向 Worker 发送任何请求"));
        var controller = CreateController(repository, radar);

        var result = await controller.CreateCapture(
            new ManualCaptureRequest { LicensePlate = "宁A10333F", IdempotencyKey = "key-1" },
            CancellationToken.None);

        var payload = Assert.IsType<ApiResponse<ManualCaptureResponse>>(
            Assert.IsType<OkObjectResult>(result.Result).Value);
        Assert.True(payload.Data!.Duplicate);
        Assert.Equal("already_accepted", payload.Data.Status);
        // SUCCEEDED 已经是确定结论，连证据查询都不必发起。
        Assert.Equal(0, radar.CallCount);
        Assert.Equal(0, repository.InsertCount);
    }

    /// <summary>
    /// 空车档案确认只对新采集生效，不得拦截原请求的正常重放。
    /// </summary>
    /// <remarks>
    /// 首次请求已经完成并写出档案、响应却在网络上丢失时，同键重试若还去做覆盖确认，
    /// 拿到的会是 409"请确认覆盖"而不是原流程 —— 一次正常重试被变成了故障。
    /// </remarks>
    [Fact]
    public async Task Replay_IsNotBlockedByEmptyProfileConfirmation()
    {
        var repository = new RecordingArrivalRepository(
            Arrival("wf-4", VehicleArrivalRadarStatus.Succeeded));
        var profiles = new StubEmptyProfileRepository(hasProfile: true);
        var controller = CreateController(
            repository,
            new CountingHandler(ScanAcceptedResponse),
            profiles);

        var result = await controller.CreateCapture(
            new ManualCaptureRequest
            {
                LicensePlate = "宁A10333F",
                IdempotencyKey = "key-4",
                ConfirmOverwriteEmptyProfile = false
            },
            CancellationToken.None);


        var payload = Assert.IsType<ApiResponse<ManualCaptureResponse>>(
            Assert.IsType<OkObjectResult>(result.Result).Value);
        Assert.True(payload.Data!.Duplicate);
        Assert.Equal("wf-4", payload.Data.WorkflowId);
    }

    /// <summary>新采集在命中已有档案且未确认时仍必须拦截。</summary>
    [Fact]
    public async Task NewCapture_WithExistingProfile_StillRequiresConfirmation()
    {
        var repository = new RecordingArrivalRepository(existing: null);
        var profiles = new StubEmptyProfileRepository(hasProfile: true);
        var controller = CreateController(
            repository,
            new CountingHandler(ScanAcceptedResponse),
            profiles);

        var result = await controller.CreateCapture(
            new ManualCaptureRequest
            {
                LicensePlate = "宁A10333F",
                IdempotencyKey = "key-5",
                ConfirmOverwriteEmptyProfile = false
            },
            CancellationToken.None);

        Assert.IsType<ConflictObjectResult>(result.Result);
        Assert.Equal(0, repository.InsertCount);
    }

    [Theory]
    [InlineData(409)]
    [InlineData(503)]
    public async Task RejectedCapture_RetryDispatchesOriginalWorkflowOnceWorkerIsAvailable(int status)
    {
        var repository = new RecordingArrivalRepository(null);
        var attempts = 0;
        string? firstQuery = null;
        var radar = new CountingHandler(request =>
        {
            Assert.NotNull(repository.Current);
            if (++attempts == 1)
            {
                firstQuery = request.RequestUri!.Query;
                return JsonResponse(status, """{"message":"已有扫描正在执行"}""");
            }
            Assert.Equal(firstQuery, request.RequestUri!.Query);
            return ScanAcceptedResponse(request);
        });
        var controller = CreateController(repository, radar, new StubEmptyProfileRepository(true));
        var request = new ManualCaptureRequest
        {
            LicensePlate = "宁A10333F", IdempotencyKey = "retry-rejected", ConfirmOverwriteEmptyProfile = true
        };
        var first = await controller.CreateCapture(request, CancellationToken.None);
        Assert.Equal(status == 409 ? 409 : 502, Assert.IsType<ObjectResult>(first.Result).StatusCode);
        Assert.Equal(VehicleArrivalRadarStatus.Failed, repository.Current!.RadarStatus);
        var workflow = repository.Current.SessionId;
        var second = await controller.CreateCapture(request, CancellationToken.None);
        var data = Assert.IsType<ApiResponse<ManualCaptureResponse>>(Assert.IsType<OkObjectResult>(second.Result).Value).Data!;
        Assert.Equal(workflow, data.WorkflowId);
        Assert.False(data.Duplicate);
        Assert.Equal(VehicleArrivalRadarStatus.Succeeded, repository.Current.RadarStatus);
        Assert.Equal(2, radar.PostCount);
        Assert.Equal(1, repository.InsertCount);
        await controller.CreateCapture(request, CancellationToken.None);
        Assert.Equal(2, radar.PostCount);
    }

    [Fact]
    public async Task InterruptedDispatch_RetryUsesWorkerIdempotencyAndKeepsOriginalWorkflow()
    {
        var repository = new RecordingArrivalRepository(null);
        var attempts = 0;
        string? firstQuery = null;
        var radar = new CountingHandler(request =>
        {
            if (++attempts == 1)
            {
                firstQuery = request.RequestUri!.Query;
                throw new HttpRequestException("受理回执未到达");
            }
            Assert.Equal(firstQuery, request.RequestUri!.Query);
            return JsonResponse(200, """{"success":true,"started":false,"idempotent":true,"scan":{"state":"scanning"}}""");
        });
        var controller = CreateController(repository, radar);
        var request = new ManualCaptureRequest { LicensePlate = "宁A10333F", IdempotencyKey = "retry-pending", ConfirmOverwriteEmptyProfile = true };
        await Assert.ThrowsAsync<HttpRequestException>(() => controller.CreateCapture(request, CancellationToken.None));
        var workflow = repository.Current!.SessionId;
        var retry = await controller.CreateCapture(request, CancellationToken.None);
        var data = Assert.IsType<ApiResponse<ManualCaptureResponse>>(Assert.IsType<OkObjectResult>(retry.Result).Value).Data!;
        Assert.True(data.Duplicate);
        Assert.Equal(workflow, data.WorkflowId);
        Assert.Equal(1, repository.InsertCount);
        Assert.Equal(2, radar.PostCount);
    }

    [Theory]
    [InlineData("timedout")]
    [InlineData("cancelled")]
    public async Task CaptureProgress_LooksUpOriginalSessionInsteadOfCurrentScan(string outcome)
    {
        var repository = new RecordingArrivalRepository(Arrival("wf-old"));
        var radar = new CountingHandler(request =>
        {
            if (request.RequestUri!.AbsolutePath.Contains("analysis", StringComparison.Ordinal))
                return JsonResponse(404, "{}");
            Assert.Equal("/api/radars/scan-once/lookup", request.RequestUri.AbsolutePath);
            Assert.Equal("?requestId=wf-old", request.RequestUri.Query);
            return JsonResponse(200, new JsonObject { ["success"] = true, ["known"] = true,
                ["scan"] = Scan("wf-old", outcome) }.ToJsonString());
        });
        var response = await CreateController(repository, radar).GetCapture("wf-old", CancellationToken.None);
        var data = Assert.IsType<ApiResponse<CaptureProgressResponse>>(Assert.IsType<OkObjectResult>(response.Result).Value).Data!;
        Assert.Equal(CaptureStages.CaptureFailed, data.Stage);
        Assert.True(data.Terminal);
        Assert.Equal(0, radar.PostCount);
    }

    [Fact]
    public async Task StoredResult_DoesNotWaitForWorkerOrUseAnotherWorkflow()
    {
        var repository = new RecordingArrivalRepository(Arrival("wf-result"))
        {
            DimensionRecord = new VehicleDimensionRecord { WorkflowId = "wf-result", TrailerLength = 1000 }
        };
        var calls = 0;
        var radar = new CountingHandler(_ => { calls++; throw new HttpRequestException("Worker unavailable"); });
        var result = await CreateController(repository, radar).GetCapture("wf-result", CancellationToken.None);
        var data = Assert.IsType<ApiResponse<CaptureProgressResponse>>(Assert.IsType<OkObjectResult>(result.Result).Value).Data!;
        Assert.Equal(CaptureStages.Completed, data.Stage);
        Assert.True(data.Terminal);
        Assert.NotNull(data.Dimensions);
        Assert.Equal(0, calls);
    }

    [Fact]
    public async Task AnalysisCompletesDuringQuery_ReadsCommittedResultAgain()
    {
        var repository = new RecordingArrivalRepository(Arrival("wf-result"));
        var radar = new CountingHandler(request =>
        {
            if (request.RequestUri!.AbsolutePath.EndsWith("analysis"))
            {
                repository.DimensionRecord = new VehicleDimensionRecord { WorkflowId = "wf-result", TrailerLength = 1000 };
                return JsonResponse(200, """{"stage":"completed","hasDimensionPayload":true}""");
            }
            return JsonResponse(404, "{}");
        });
        var result = await CreateController(repository, radar).GetCapture("wf-result", CancellationToken.None);
        var data = Assert.IsType<ApiResponse<CaptureProgressResponse>>(Assert.IsType<OkObjectResult>(result.Result).Value).Data!;
        Assert.Equal(CaptureStages.Completed, data.Stage);
        Assert.True(repository.DimensionReads >= 2);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task MissingOrUnreachableWorker_ExplainsUnavailableProgressInsteadOfEndlessWaiting(bool unreachable)
    {
        var repository = new RecordingArrivalRepository(Arrival("wf-lost", VehicleArrivalRadarStatus.Succeeded));
        var radar = new CountingHandler(_ => unreachable ? throw new HttpRequestException("connection refused") : JsonResponse(404, "{}"));
        var result = await CreateController(repository, radar).GetCapture("wf-lost", CancellationToken.None);
        var data = Assert.IsType<ApiResponse<CaptureProgressResponse>>(Assert.IsType<OkObjectResult>(result.Result).Value).Data!;
        Assert.Equal("progress_unavailable", data.Stage);
        Assert.True(data.Terminal);
        Assert.Null(data.Dimensions);
        Assert.Equal(0, radar.PostCount);
    }

    [Fact]
    public async Task SavedPcdWithAnalysisDisabled_DoesNotWaitForAnAnalysisThatWillNotRun()
    {
        var repository = new RecordingArrivalRepository(Arrival("wf-off"));
        var radar = new CountingHandler(request => request.RequestUri!.AbsolutePath.EndsWith("analysis")
            ? JsonResponse(404, "{}") : JsonResponse(200, new JsonObject { ["analysisEnabled"] = false,
                ["scan"] = Scan("wf-off", "completed", "saved", persisted: ["CAM_004", "CAM_005"]) }.ToJsonString()));
        var result = await CreateController(repository, radar).GetCapture("wf-off", CancellationToken.None);
        var data = Assert.IsType<ApiResponse<CaptureProgressResponse>>(Assert.IsType<OkObjectResult>(result.Result).Value).Data!;
        Assert.True(data.Terminal);
        Assert.Contains("未开启", data.Message);
        Assert.Null(data.Dimensions);
    }

    // ------------------------------------------------------------ 测试夹具

    [Fact]
    public async Task IdempotentWorkerReceipt_IsAcceptedWithoutRequiringANewScan()
    {
        var repository = new RecordingArrivalRepository(null);
        var radar = new CountingHandler(request =>
        {
            var accepted = ScanAcceptedResponse(request);
            var body = JsonNode.Parse(accepted.Content.ReadAsStringAsync().GetAwaiter().GetResult())!;
            accepted.Dispose();
            body["code"] = "existing"; body["started"] = false; body["idempotent"] = true;
            return JsonResponse(200, body.ToJsonString());
        });
        var controller = CreateController(repository, radar);
        var result = await controller.CreateCapture(new ManualCaptureRequest
        {
            LicensePlate = "宁A10333F",
            IdempotencyKey = "accepted"
        }, CancellationToken.None);
        Assert.IsType<OkObjectResult>(result.Result);
        Assert.Equal(VehicleArrivalRadarStatus.Succeeded, repository.Current!.RadarStatus);
    }

    private static HttpResponseMessage JsonResponse(int status, string body) => new((HttpStatusCode)status)
    {
        Content = new StringContent(body, Encoding.UTF8, "application/json")
    };

    private static VehicleArrivalRecord Arrival(string workflowId) => new()
    {
        SessionId = workflowId,
        LicensePlate = "宁A10333F",
        PlateStatus = VehicleArrivalPlateStatus.Succeeded,
        PlateSource = "MANUAL"
    };

    private static JsonNode Scan(
        string requestId,
        string outcome,
        string? persistenceState = null,
        string[]? captured = null,
        string[]? persisted = null,
        string? safetyBlockReason = null,
        string? persistenceError = null)
    {
        var node = new JsonObject
        {
            ["requestId"] = requestId,
            ["outcome"] = outcome,
            ["capturedCameras"] = new JsonArray((captured ?? []).Select(x => (JsonNode)x!).ToArray()),
            ["persistedCameras"] = new JsonArray((persisted ?? []).Select(x => (JsonNode)x!).ToArray())
        };
        if (persistenceState is not null) node["persistenceState"] = persistenceState;
        if (safetyBlockReason is not null) node["safetyBlockReason"] = safetyBlockReason;
        if (persistenceError is not null) node["persistenceError"] = persistenceError;
        return node;
    }

    private static JsonNode Analysis(string stage, string? error = null, bool permanentFailure = false)
    {
        var node = new JsonObject
        {
            ["stage"] = stage,
            ["permanentFailure"] = permanentFailure
        };
        if (error is not null) node["error"] = error;
        return node;
    }

    private static JsonNode AnalysisWithoutDimensions(string loadStatus, string? reason)
    {
        var node = new JsonObject
        {
            ["stage"] = "completed",
            ["permanentFailure"] = false,
            ["hasDimensionPayload"] = false,
            ["loadStatus"] = new JsonObject { ["status"] = loadStatus }
        };
        if (reason is not null)
        {
            node["measurementStatus"] = new JsonObject { ["valid"] = false, ["reason"] = reason };
        }

        return node;
    }

    private static HttpResponseMessage NoUpdateResponse(long version) => new(HttpStatusCode.OK)
    {
        Content = new StringContent(
            $"{{\"success\":true,\"updated\":false,\"version\":{version}}}",
            Encoding.UTF8,
            "application/json")
    };

    private static HttpResponseMessage FrameResponse(long version) => new(HttpStatusCode.OK)
    {
        Content = new StringContent(
            $"{{\"success\":true,\"updated\":true,\"version\":{version},\"pointCountA\":1,\"pointCountB\":1,\"points\":\"\"}}",
            Encoding.UTF8,
            "application/json")
    };

    private static VehicleArrivalRecord Arrival(string workflowId, string radarStatus) => new()
    {
        Id = 42,
        SessionId = workflowId,
        LicensePlate = "宁A10333F",
        PlateStatus = VehicleArrivalPlateStatus.Succeeded,
        PlateSource = "MANUAL",
        RadarStatus = radarStatus
    };

    private static HttpResponseMessage ScanAcceptedResponse(HttpRequestMessage request) => new(HttpStatusCode.Accepted)
    {
        Content = new StringContent(
            new JsonObject
            {
                ["success"] = true,
                ["code"] = "scanning",
                ["started"] = true,
                ["idempotent"] = false,
                ["message"] = "ok",
                ["scan"] = new JsonObject
                {
                    ["state"] = "scanning",
                    ["requestId"] = Uri.UnescapeDataString(request.RequestUri!.Query.Split('=', 2)[1])
                }
            }.ToJsonString(),
            Encoding.UTF8,
            "application/json")
    };

    private static RadarLiveController CreateController(
        RecordingArrivalRepository repository,
        CountingHandler radarHandler,
        StubEmptyProfileRepository? profiles = null)
    {
        var client = new RadarLiveClient(
            new StubHttpClientFactory(radarHandler));
        var configuration = new ConfigurationBuilder().AddInMemoryCollection().Build();
        var controller = new RadarLiveController(
            client,
            repository,
            profiles ?? new StubEmptyProfileRepository(hasProfile: false),
            new IntegrationQueryService(repository, configuration),
            configuration,
            NullLogger<RadarLiveController>.Instance)
        {
            ControllerContext = new ControllerContext { HttpContext = new DefaultHttpContext() }
        };
        return controller;
    }

    /// <summary>只实现本组用例需要的来车记录行为，其余沿用接口默认实现。</summary>
    private sealed class RecordingArrivalRepository : NullIntegrationRepository, IIntegrationRepository
    {
        private VehicleArrivalRecord? _existing;

        public VehicleArrivalRecord? Current => _existing;

        public RecordingArrivalRepository(VehicleArrivalRecord? existing) => _existing = existing;

        public int InsertCount { get; private set; }

        public bool ClaimResult { get; set; }
        public VehicleDimensionRecord? DimensionRecord { get; set; }
        public int DimensionReads { get; private set; }
        public VehicleDimensionRecord? GetVehicleDimensionByWorkflowId(string workflowId)
        {
            DimensionReads++;
            return DimensionRecord?.WorkflowId == workflowId ? DimensionRecord : null;
        }

        public override VehicleArrivalRecord? FindVehicleArrivalByEventId(string eventId) => _existing;

        public override VehicleArrivalRecord? GetVehicleArrivalBySessionId(string sessionId) =>
            _existing?.SessionId == sessionId ? _existing : null;

        public override VehicleArrivalInsertResult InsertVehicleArrivalRecord(VehicleArrivalRecord record)
        {
            InsertCount++;
            record.Id = 42;
            _existing = record;
            return new VehicleArrivalInsertResult { Record = record, IsNew = true };
        }

        public override void UpdateVehicleArrivalRadarResult(
            long recordId,
            string radarStatus,
            string? radarError,
            DateTime updatedAt)
        {
            Assert.NotNull(_existing);
            _existing.RadarStatus = radarStatus;
            _existing.RadarError = radarError;
        }
    }

    private sealed class StubEmptyProfileRepository : IEmptyVehicleProfileRepository
    {
        private readonly bool _hasProfile;

        public StubEmptyProfileRepository(bool hasProfile) => _hasProfile = hasProfile;

        public EmptyVehicleProfileRecord? GetByLicensePlate(string licensePlate) =>
            _hasProfile
                ? new EmptyVehicleProfileRecord
                {
                    LicensePlate = licensePlate,
                    ScanTime = new DateTime(2026, 9, 1, 8, 0, 0)
                }
                : null;
    }

    private sealed class StubHttpClientFactory : IHttpClientFactory
    {
        private readonly HttpMessageHandler _handler;

        public StubHttpClientFactory(HttpMessageHandler handler) => _handler = handler;

        public HttpClient CreateClient(string name) => new(_handler, disposeHandler: false)
        {
            BaseAddress = new Uri("http://radar.test")
        };
    }

    private sealed class CountingHandler : HttpMessageHandler
    {
        private readonly Func<HttpRequestMessage, HttpResponseMessage> _responder;

        public CountingHandler(Func<HttpRequestMessage, HttpResponseMessage> responder) =>
            _responder = responder;

        public int CallCount { get; private set; }

        /// <summary>只统计真正会驱动硬件的 POST，读证据的 GET 不计入。</summary>
        public int PostCount { get; private set; }

        protected override Task<HttpResponseMessage> SendAsync(
            HttpRequestMessage request,
            CancellationToken cancellationToken)
        {
            CallCount++;
            if (request.Method == HttpMethod.Post)
            {
                PostCount++;
            }

            return Task.FromResult(_responder(request));
        }
    }
}
