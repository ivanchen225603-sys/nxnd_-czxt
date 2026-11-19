using System.Diagnostics;
using System.Net;
using System.Text.Json;
using Microsoft.Extensions.DependencyInjection;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class PartnerQueryContractLoggingTests
{
    public static IEnumerable<object[]> InputCases()
    {
        var cases = new (string Query, int Status, int PageSize, int Minutes)[]
        {
            ("?limit=1", 200, 1, 0), ("?limit=100", 200, 100, 0),
            ("?limit=0", 200, 100, 0), ("?limit=-1", 200, 100, 0),
            ("?limit=-2147483648", 200, 100, 0),
            ("?limit=101", 200, 100, 0), ("?limit=2147483647", 200, 100, 0),
            ("?timeWindow=1", 200, 100, 1), ("?timeWindow=120", 200, 100, 120),
            ("?timeWindow=0", 200, 100, 120), ("?timeWindow=-1", 200, 100, 120),
            ("?timeWindow=121", 200, 100, 120), ("", 400, 0, 0),
            ("?limit=1&timeWindow=1", 400, 0, 0), ("?limit=", 400, 0, 0),
            ("?limit=1&timeWindow=", 400, 0, 0), ("?limit=&timeWindow=1", 400, 0, 0),
            ("?limit=1&limit=2", 400, 0, 0), ("?limit=1&extra=1", 400, 0, 0),
            ("?limit=abc", 400, 0, 0), ("?limit=1.5", 400, 0, 0),
            ("?limit=2147483648", 400, 0, 0), ("?timeWindow=abc", 400, 0, 0),
            ("?timeWindow=", 400, 0, 0), ("?timeWindow=1.5", 400, 0, 0), ("?timeWindow=2147483648", 400, 0, 0),
            ("?cursor=bad-cursor", 400, 0, 0), ("?cursor=bad&limit=1", 400, 0, 0)
        };
        foreach (var path in new[] { "/api/v1/vehicleDimensions", "/api/v1/coalVolumes" })
            foreach (var item in cases) yield return [path, item.Query, item.Status, item.PageSize, item.Minutes];
    }

    [Theory]
    [MemberData(nameof(InputCases))]
    public async Task Contract_inputs_and_actual_execution_are_audited(string path, string query, int status, int pageSize, int minutes)
    {
        using var host = new PartnerQueryContractHost();
        var before = BusinessClock.Now;
        var result = await SendAsync(host, path + query);
        var after = BusinessClock.Now;
        Assert.Equal(status, result.Status);
        var json = Envelope(result.Body, status);
        if (status == 200)
        {
            Assert.Single(json.GetProperty("data").EnumerateArray());
            var executed = Assert.Single(host.Repository.Queries);
            Assert.Equal(101, executed.Take); // 批量筛选后再决定对外条数。
            if (minutes == 0) Assert.Null(executed.WindowStart);
            else Assert.InRange(executed.WindowStart!.Value, before.AddMinutes(-minutes), after.AddMinutes(-minutes));
            var step = Assert.Single(host.Repository.Records.Where(row => row.LifecycleStage == "QueryExecuted"));
            Assert.Contains($"每页={pageSize}", step.LogMessage);
        }
        else
        {
            Assert.Equal(JsonValueKind.Null, json.GetProperty("data").ValueKind);
            Assert.Empty(host.Repository.Queries);
            Assert.DoesNotContain(host.Repository.Records, row => row.LifecycleStage == "QueryExecuted");
        }
        AssertAudit(host, result);
        host.SaveEvidence(path.Split('/')[^1] + "-input", path + query, result.Interaction, result.Status, result.Body);
    }

    [Fact]
    public async Task Coal_output_uses_contract_fields_units_rounding_and_original_business_times()
    {
        using var host = new PartnerQueryContractHost();
        var source = host.Repository.Coals.Single();
        source.EnterId = "000042-业务";
        source.CaptureTime = new DateTime(2026, 10, 12, 9, 0, 1);
        source.MeasurementStartedAt = source.CaptureTime.Value.AddSeconds(10);
        source.MeasurementEndedAt = source.CaptureTime.Value.AddSeconds(17);
        var result = await SendAsync(host, "/api/v1/coalVolumes?limit=1");
        var row = Envelope(result.Body, 200).GetProperty("data")[0];
        AssertFields(row, "licensePlate", "cameraIdentifier", "captureTime", "enterID", "volume", "unit", "measureTime", "measureEndTime");
        Assert.Equal("宁A48394Q", row.GetProperty("licensePlate").GetString());
        Assert.Equal("CAM_001", row.GetProperty("cameraIdentifier").GetString());
        Assert.Equal(52.4m, row.GetProperty("volume").GetDecimal());
        Assert.Equal("m³", row.GetProperty("unit").GetString());
        Assert.Equal("000042-业务", row.GetProperty("enterID").GetString());
        Assert.Equal("2026-10-12 09:00:01", row.GetProperty("captureTime").GetString());
        Assert.Equal("2026-10-12 09:00:11", row.GetProperty("measureTime").GetString());
        Assert.Equal("2026-10-12 09:00:18", row.GetProperty("measureEndTime").GetString());
        AssertAudit(host, result);
        host.SaveEvidence("coal-output", "/api/v1/coalVolumes?limit=1", result.Interaction, result.Status, result.Body);
    }

    [Fact]
    public async Task Vehicle_output_separates_external_width_plate_length_and_internal_metadata()
    {
        using var host = new PartnerQueryContractHost();
        var result = await SendAsync(host, "/api/v1/vehicleDimensions?limit=1");
        var row = Envelope(result.Body, 200).GetProperty("data")[0];
        AssertFields(row, "licensePlate", "cameraIdentifier", "getTime", "dimensions");
        Assert.Equal(host.Repository.Vehicles[0].GetTime, row.GetProperty("getTime").GetDateTime());
        var dimensions = row.GetProperty("dimensions");
        AssertFields(dimensions, "cabToTrailerFrontDist", "trailerLength", "trailerWidth", "trailerBodyHeight",
            "trailerBottomToGroundHeight", "highLowPlateHeight", "highLowPlates", "ribs", "wires");
        Assert.Equal(440.2m, dimensions.GetProperty("cabToTrailerFrontDist").GetDecimal());
        Assert.Equal(1050.2m, dimensions.GetProperty("trailerLength").GetDecimal());
        Assert.Equal(255.4m, dimensions.GetProperty("trailerWidth").GetDecimal());
        Assert.Equal(JsonValueKind.Null, dimensions.GetProperty("trailerBodyHeight").ValueKind);
        Assert.Equal(180.2m, dimensions.GetProperty("highLowPlates")[0].GetProperty("highestToGround").GetDecimal());
        Assert.Equal(2, dimensions.GetProperty("ribs").GetArrayLength());
        Assert.Equal(1, dimensions.GetProperty("ribs")[0].GetProperty("ribId").GetInt32());
        Assert.Equal(350.2m, dimensions.GetProperty("ribs")[0].GetProperty("distToPrev").GetDecimal());
        Assert.Equal(JsonValueKind.Null, dimensions.GetProperty("ribs")[1].GetProperty("distToPrev").ValueKind);
        Assert.Empty(dimensions.GetProperty("wires").EnumerateArray());
        Assert.DoesNotContain("synthetic-workflow", result.Body);
        Assert.DoesNotContain("internal-only.pcd", result.Body);
        AssertAudit(host, result);
        host.SaveEvidence("vehicle-output", "/api/v1/vehicleDimensions?limit=1", result.Interaction, result.Status, result.Body);
    }

    [Fact]
    public async Task Vehicle_details_respect_document_array_caps_and_failed_dimensions_are_empty()
    {
        using var host = new PartnerQueryContractHost();
        var vehicle = host.Repository.Vehicles[0];
        vehicle.ActualRibCount = 9;
        vehicle.Ribs = Enumerable.Range(1, 10).Reverse().Select(i => new RibInfo { RibId = i, DistToPrev = 100 }).ToList();
        vehicle.HighLowPlates = Enumerable.Range(1, 4).Select(i => new HighLowPlateInfo { PlateId = i, LengthCm = 100 }).ToList();
        var result = await SendAsync(host, "/api/v1/vehicleDimensions?limit=1");
        var dimensions = Envelope(result.Body, 200).GetProperty("data")[0].GetProperty("dimensions");
        Assert.Equal(6, dimensions.GetProperty("ribs").GetArrayLength());
        Assert.Equal(3, dimensions.GetProperty("highLowPlates").GetArrayLength());
        Assert.Empty(dimensions.GetProperty("wires").EnumerateArray());
        vehicle.DetectStatus = "failed";
        var failed = await SendAsync(host, "/api/v1/vehicleDimensions?limit=1");
        Assert.Empty(Envelope(failed.Body, 200).GetProperty("data")[0].GetProperty("dimensions").EnumerateObject());
    }

    [Theory]
    [InlineData("/api/v1/vehicleDimensions")]
    [InlineData("/api/v1/coalVolumes")]
    public async Task Empty_data_time_windows_and_latest_limit_follow_contract_without_public_cursor(string path)
    {
        using var host = new PartnerQueryContractHost();
        var now = BusinessClock.Now.AddSeconds(-1);
        host.Repository.Vehicles.Clear(); host.Repository.Coals.Clear();
        var empty = await SendAsync(host, path + "?limit=1");
        Assert.Empty(Envelope(empty.Body, 200).GetProperty("data").EnumerateArray());
        for (var id = 1; id <= 105; id++)
        {
            var time = id == 1 ? now.AddMinutes(-121) : now.AddSeconds(id - 105);
            var vehicle = PartnerQueryContractHost.Vehicle(id, time); vehicle.LicensePlate = "宁A" + id.ToString("D5");
            var coal = PartnerQueryContractHost.Coal(id, time); coal.LicensePlate = vehicle.LicensePlate;
            host.Repository.Vehicles.Add(vehicle); host.Repository.Coals.Add(coal);
        }
        var capped = await SendAsync(host, path + "?limit=999");
        var data = Envelope(capped.Body, 200).GetProperty("data");
        Assert.Equal(100, data.GetArrayLength());
        Assert.Equal("宁A34741", data[0].GetProperty("licensePlate").GetString());
        Assert.Equal("宁A91308", data[99].GetProperty("licensePlate").GetString());
        // 换客户端来源以隔离限流窗口；请求业务参数和日志仍走同一条生产管线。
        Assert.Null(capped.Cursor);
        var continued = await SendAsync(host, path + "?cursor=unsupported", newSource: true);
        Envelope(continued.Body, 400);
        host.Repository.Vehicles.Clear(); host.Repository.Coals.Clear();
        for (var id = 1; id <= 3; id++)
        {
            var time = id == 1 ? now.AddMinutes(-121) : now;
            var vehicle = PartnerQueryContractHost.Vehicle(id, time); vehicle.LicensePlate = "宁A" + id.ToString("D5");
            var coal = PartnerQueryContractHost.Coal(id, time); coal.LicensePlate = vehicle.LicensePlate;
            host.Repository.Vehicles.Add(vehicle); host.Repository.Coals.Add(coal);
        }
        var window = await SendAsync(host, path + "?timeWindow=120", newSource: true);
        Assert.Equal(2, Envelope(window.Body, 200).GetProperty("data").GetArrayLength());
        Assert.DoesNotContain(Envelope(window.Body, 200).GetProperty("data").EnumerateArray(),
            row => row.GetProperty("licensePlate").GetString() == "宁A96523");
        AssertAudit(host, window);
        host.SaveEvidence(path.Split('/')[^1] + "-pagination", path + "?timeWindow=120", window.Interaction, window.Status, window.Body);
    }

    [Fact]
    public async Task Incomplete_coal_is_not_published_or_counted_against_limit()
    {
        using var host = new PartnerQueryContractHost();
        var coal = host.Repository.Coals[0];
        coal.EnterId = null; coal.CaptureTime = null; coal.MeasurementStartedAt = null; coal.MeasurementEndedAt = null;
        var invalid = PartnerQueryContractHost.Coal(2, coal.MeasureTime); invalid.LicensePlate = "UNKNOWN";
        host.Repository.Coals.Add(invalid);
        var complete = PartnerQueryContractHost.Coal(3, coal.MeasureTime.AddSeconds(-5));
        host.Repository.Coals.Add(complete);
        var result = await SendAsync(host, "/api/v1/coalVolumes?limit=1");
        var row = Assert.Single(Envelope(result.Body, 200).GetProperty("data").EnumerateArray());
        foreach (var field in new[] { "enterID", "captureTime", "measureTime", "measureEndTime" })
            Assert.Equal(JsonValueKind.String, row.GetProperty(field).ValueKind);
        Assert.Equal(PartnerContract.FormatTime(complete.MeasurementStartedAt), row.GetProperty("measureTime").GetString());
        var access = Access(host, result.Interaction);
        Assert.Contains("实际出参 data 条数=1", access.LogMessage);
        Assert.Contains("按合同排除=2 条", host.Repository.Records.Single(item => item.LifecycleStage == "QueryExecuted").LogMessage);
    }

    [Theory]
    [InlineData("/api/v1/vehicleDimensions", "missing")]
    [InlineData("/api/v1/vehicleDimensions", "wrong")]
    [InlineData("/api/v1/coalVolumes", "expired")]
    [InlineData("/api/v1/vehicleDimensions", "missing-timestamp")]
    [InlineData("/api/v1/coalVolumes", "missing-signature")]
    [InlineData("/api/v1/vehicleDimensions", "seconds")]
    [InlineData("/api/v1/vehicleDimensions", "future")]
    [InlineData("/api/v1/coalVolumes", "uppercase-signature")]
    [InlineData("/api/v1/vehicleDimensions", "multiple-timestamps")]
    [InlineData("/api/v1/coalVolumes", "leading-zero")]
    [InlineData("/api/v1/vehicleDimensions", "leading-zero")]
    [InlineData("/api/v1/coalVolumes", "leading-plus")]
    [InlineData("/api/v1/coalVolumes", "min-long")]
    public async Task Authentication_failure_is_logged_without_executing_query(string path, string auth)
    {
        using var host = new PartnerQueryContractHost();
        var result = await SendAsync(host, path + "?limit=1", auth);
        Envelope(result.Body, 401);
        Assert.Empty(host.Repository.Queries);
        Assert.DoesNotContain(host.Repository.Records, row => row.LifecycleStage == "QueryExecuted");
        AssertAudit(host, result);
        host.SaveEvidence(path.Split('/')[^1] + "-auth-" + auth, path + "?limit=1", result.Interaction, result.Status, result.Body);
    }

    [Theory]
    [InlineData("/api/v1/coalVolumes")]
    [InlineData("/api/v1/vehicleDimensions")]
    public async Task Missing_query_subpath_returns_contract_404(string path)
    {
        using var host = new PartnerQueryContractHost();
        var uri = path + "/not-found";
        var result = await SendAsync(host, uri);
        Assert.Equal(404, result.Status);
        Assert.Equal(JsonValueKind.Null, Envelope(result.Body, 404).GetProperty("data").ValueKind);
        Assert.Empty(host.Repository.Queries);
        Assert.Equal(404, Access(host, result.Interaction).HttpStatus);
        Assert.Null(Access(host, result.Interaction).ResponseBody);
        host.SaveEvidence(path.Split('/')[^1] + "-not-found", uri, result.Interaction, result.Status, result.Body);
    }

    [Fact]
    public async Task Blocked_audit_does_not_change_a_successful_coal_query()
    {
        using var host = new PartnerQueryContractHost();
        using var warmup = host.CreateClient();
        var audit = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        host.Repository.AuditBlock = audit;
        try
        {
            var result = await SendAsync(host, "/api/v1/coalVolumes?limit=1").WaitAsync(TimeSpan.FromSeconds(3));
            Assert.Equal(200, result.Status);
            Assert.Single(Envelope(result.Body, 200).GetProperty("data").EnumerateArray());
        }
        finally { audit.TrySetResult(); }
    }

    [Theory]
    [InlineData("/api/v1/coalVolumes")]
    [InlineData("/api/v1/vehicleDimensions")]
    public async Task Blocked_query_and_audit_return_contract_timeout_within_three_seconds(string path)
    {
        using var host = new PartnerQueryContractHost();
        using var warmup = host.CreateClient();
        var query = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var audit = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        host.Repository.QueryBlock = query;
        host.Repository.AuditBlock = audit;
        var uri = path + "?limit=1";
        try
        {
            var started = Stopwatch.GetTimestamp();
            var result = await SendAsync(host, uri).WaitAsync(TimeSpan.FromSeconds(3));
            var elapsed = Stopwatch.GetElapsedTime(started);
            Assert.Equal(500, result.Status);
            var json = Envelope(result.Body, 500);
            Assert.Equal("查询超时", json.GetProperty("message").GetString());
            Assert.Equal(JsonValueKind.Null, json.GetProperty("data").ValueKind);
            Assert.Contains(host.Logs.Entries, row => row.InteractionId == result.Interaction && row.Stage == "RequestFailed");
            host.SaveEvidence(path.Split('/')[^1] + "-timeout", uri,
                result.Interaction, result.Status, result.Body, elapsed.TotalMilliseconds);
        }
        finally
        {
            query.TrySetResult();
            audit.TrySetResult();
        }
    }

    [Theory]
    [InlineData("/api/v1/vehicleDimensions", 429)]
    [InlineData("/api/v1/coalVolumes", 400)]
    public async Task Rate_limit_and_query_failure_preserve_error_responses_and_logs(string path, int limitStatus)
    {
        using var host = new PartnerQueryContractHost();
        for (var i = 0; i < 3; i++) Assert.Equal(200, (await SendAsync(host, path + "?limit=1")).Status);
        var limited = await SendAsync(host, path + "?limit=1");
        Envelope(limited.Body, limitStatus);
        Assert.Equal(3, host.Repository.Queries.Count);
        AssertAudit(host, limited);
        host.SaveEvidence(path.Split('/')[^1] + "-rate-limit", path + "?limit=1", limited.Interaction, limited.Status, limited.Body);
        host.Repository.QueryError = path.EndsWith("coalVolumes", StringComparison.Ordinal)
            ? new ArgumentException("isolated-query-failure")
            : new InvalidOperationException("isolated-query-failure");
        var failure = await SendAsync(host, path + "?limit=1", newSource: true);
        Assert.Equal(500, failure.Status);
        Envelope(failure.Body, 500);
        Assert.DoesNotContain("isolated-query-failure", failure.Body);
        Assert.Equal("RequestFailed", Access(host, failure.Interaction).LifecycleStage);
        Assert.DoesNotContain(host.Repository.Records, row => row.InteractionId == failure.Interaction && row.LifecycleStage == "QueryExecuted");
        AssertAudit(host, failure);
        host.SaveEvidence(path.Split('/')[^1] + "-query-failure", path + "?limit=1", failure.Interaction, failure.Status, failure.Body);
    }

    private static JsonElement Envelope(string body, int status)
    {
        using var document = JsonDocument.Parse(body);
        var json = document.RootElement.Clone();
        AssertFields(json, "code", "message", "data");
        Assert.Equal(status, json.GetProperty("code").GetInt32());
        Assert.Equal(status == 200, json.GetProperty("message").GetString() == "success");
        return json;
    }

    [Theory]
    [InlineData("/api/v1/vehicleDimensions")]
    [InlineData("/api/v1/coalVolumes")]
    public async Task Invalid_latest_batches_cannot_hide_older_valid_data(string path)
    {
        using var host = new PartnerQueryContractHost();
        var now = BusinessClock.Now.AddSeconds(-1);
        host.Repository.Vehicles.Clear(); host.Repository.Coals.Clear();
        for (var id = 1; id <= 205; id++)
        {
            var vehicle = PartnerQueryContractHost.Vehicle(id, now);
            var coal = PartnerQueryContractHost.Coal(id, now);
            if (id != 1) { vehicle.LicensePlate = "UNKNOWN"; coal.EnterId = null; }
            host.Repository.Vehicles.Add(vehicle); host.Repository.Coals.Add(coal);
        }
        var result = await SendAsync(host, path + "?limit=1");
        Assert.Single(Envelope(result.Body, 200).GetProperty("data").EnumerateArray());
        Assert.Equal(3, host.Repository.Queries.Count);
        AssertAudit(host, result);
        Assert.Contains("按合同排除=204 条", host.Repository.Records.Single(row => row.LifecycleStage == "QueryExecuted").LogMessage);
        host.SaveEvidence(path.Split('/')[^1] + "-filter-before-limit", path + "?limit=1",
            result.Interaction, result.Status, result.Body);
    }

    [Theory]
    [InlineData("enterID")]
    [InlineData("captureTime")]
    [InlineData("measureTime")]
    [InlineData("measureEndTime")]
    [InlineData("reversed")]
    [InlineData("blank-enter")]
    public async Task Required_coal_fields_are_never_null_or_fabricated(string missing)
    {
        using var host = new PartnerQueryContractHost();
        var row = host.Repository.Coals[0];
        switch (missing)
        {
            case "enterID": row.EnterId = null; break;
            case "captureTime": row.CaptureTime = null; break;
            case "measureTime": row.MeasurementStartedAt = null; break;
            case "measureEndTime": row.MeasurementEndedAt = null; break;
            case "reversed": row.MeasurementEndedAt = row.MeasurementStartedAt!.Value.AddSeconds(-1); break;
            case "blank-enter": row.EnterId = " "; break;
        }
        var result = await SendAsync(host, "/api/v1/coalVolumes?limit=1");
        Assert.Empty(Envelope(result.Body, 200).GetProperty("data").EnumerateArray());
        AssertAudit(host, result);
        host.SaveEvidence("coal-required-" + missing, "/api/v1/coalVolumes?limit=1",
            result.Interaction, result.Status, result.Body);
    }

    [Fact]
    public async Task Coal_window_uses_the_measureTime_returned_to_partner()
    {
        using var host = new PartnerQueryContractHost();
        var now = BusinessClock.Now.AddSeconds(-1);
        var excluded = host.Repository.Coals[0];
        excluded.MeasureTime = now; excluded.MeasurementStartedAt = now.AddMinutes(-121);
        var included = PartnerQueryContractHost.Coal(2, now.AddMinutes(-121));
        included.MeasurementStartedAt = now.AddMinutes(-1); included.MeasurementEndedAt = now;
        included.LicensePlate = "宁B77617N";
        host.Repository.Coals.Add(included);
        var result = await SendAsync(host, "/api/v1/coalVolumes?timeWindow=120");
        Assert.Equal("宁B77617N", Assert.Single(Envelope(result.Body, 200).GetProperty("data").EnumerateArray())
            .GetProperty("licensePlate").GetString());
    }

    [Theory]
    [InlineData("AppSecret")]
    [InlineData("Signature")]
    [InlineData("access_token")]
    [InlineData("password")]
    public async Task Unsupported_secret_query_values_are_rejected_and_redacted(string key)
    {
        using var host = new PartnerQueryContractHost();
        const string value = "query-credential-must-never-be-logged";
        var result = await SendAsync(host, "/api/v1/coalVolumes?limit=1&" + key + "=" + value);
        Envelope(result.Body, 400);
        var logs = JsonSerializer.Serialize(new { audit = host.Repository.Records, process = host.Logs.Entries });
        Assert.DoesNotContain(value, logs);
        Assert.Contains("REDACTED", logs);
        Assert.Empty(host.Repository.Queries);
    }

    [Theory]
    [InlineData("{\"x\":1,\"x\":2}")]
    [InlineData("not-json")]
    public async Task Extra_GET_body_cannot_break_auditing_or_authentication(string body)
    {
        using var host = new PartnerQueryContractHost();
        var result = await SendAsync(host, "/api/v1/coalVolumes?limit=1", body: body);
        Envelope(result.Body, 400);
        AssertAudit(host, result);
        Assert.Null(Access(host, result.Interaction).RequestBody);
    }

    private static void AssertFields(JsonElement element, params string[] fields) =>
        Assert.Equal(fields.OrderBy(name => name), element.EnumerateObject().Select(field => field.Name).OrderBy(name => name));

    private static ApiAccessLogRecord Access(PartnerQueryContractHost host, string interaction) =>
        Assert.Single(host.Repository.Records.Where(row => row.InteractionId == interaction && row.RecordKind == "Access"));

    private static void AssertAudit(PartnerQueryContractHost host, QueryResult result)
    {
        var access = Access(host, result.Interaction);
        Assert.Equal(result.Status, access.HttpStatus);
        Assert.Equal(result.Status, access.BusinessCode);
        Assert.Equal(result.Query, access.QueryString ?? "");
        Assert.False(string.IsNullOrWhiteSpace(access.ResponseBody));
        using var logged = JsonDocument.Parse(access.ResponseBody!);
        using var actual = JsonDocument.Parse(result.Body);
        Assert.Equal(JsonSerializer.Serialize(actual.RootElement), JsonSerializer.Serialize(logged.RootElement));
        var logs = host.Logs.Entries.Where(log => log.InteractionId == result.Interaction).ToList();
        Assert.Contains(logs, log => log.Stage == "RequestReceived");
        Assert.Contains(logs, log => log.Stage == "QueryResponse" && log.Message.Contains("responseBody="));
        Assert.Contains(logs, log => log.Stage == (result.Status >= 500 ? "RequestFailed" : result.Status >= 400 ? "RequestRejected" : "RequestCompleted"));
        if (result.Status == 200)
            Assert.Equal(new[] { "RequestReceived", "QueryExecuted", "QueryResponse", "RequestCompleted" },
                logs.Where(log => !string.IsNullOrEmpty(log.Stage)).Select(log => log.Stage));
        var evidence = JsonSerializer.Serialize(new { audit = host.Repository.Records.ToArray(), logs = host.Logs.Entries.ToArray() });
        Assert.DoesNotContain(PartnerQueryContractHost.Secret, evidence);
        if (!string.IsNullOrEmpty(result.Signature)) Assert.DoesNotContain(result.Signature, evidence);
        Assert.Equal(0, host.Outbound.Calls);
    }

    private sealed record QueryResult(int Status, string Body, string Interaction, string Query, string? Signature, string? Cursor);

    private static async Task<QueryResult> SendAsync(PartnerQueryContractHost host, string uri, string auth = "valid", bool newSource = false, string? body = null)
    {
        using var client = host.CreateClient();
        using var request = new HttpRequestMessage(HttpMethod.Get, uri);
        if (body is not null) request.Content = new StringContent(body, System.Text.Encoding.UTF8, "application/json");
        var interaction = Guid.NewGuid().ToString("N");
        request.Headers.Add("X-Interaction-Id", interaction);
        // 测试专用启动过滤器读取此头，不影响生产配置或正式来源校验。
        if (newSource) request.Headers.Add("X-Query-Test-Source", Guid.NewGuid().ToString("N"));
        string? signature = null;
        if (auth != "missing")
        {
            var timestamp = (auth == "expired" ? DateTimeOffset.UtcNow.AddMinutes(-2) : auth == "future" ? DateTimeOffset.UtcNow.AddMinutes(2) : DateTimeOffset.UtcNow)
                .ToUnixTimeMilliseconds().ToString(System.Globalization.CultureInfo.InvariantCulture);
            if (auth == "seconds") timestamp = DateTimeOffset.UtcNow.ToUnixTimeSeconds().ToString(System.Globalization.CultureInfo.InvariantCulture);
            if (auth == "leading-zero") timestamp = "0" + timestamp;
            if (auth == "leading-plus") timestamp = "+" + timestamp;
            if (auth == "min-long") timestamp = "-9223372036854775808";
            signature = ApiSignatureHelper.GenerateSignature(timestamp, auth == "wrong" ? "wrong-test-secret" : PartnerQueryContractHost.Secret);
            if (auth == "uppercase-signature") signature = signature.ToUpperInvariant();
            if (auth != "missing-timestamp") request.Headers.Add("Timestamp", timestamp);
            if (auth == "multiple-timestamps") request.Headers.Add("Timestamp", timestamp);
            if (auth != "missing-signature") request.Headers.Add("Signature", signature);
        }
        using var response = await client.SendAsync(request);
        Assert.Equal(interaction, response.Headers.GetValues("X-Interaction-Id").Single());
        return new((int)response.StatusCode, await response.Content.ReadAsStringAsync(), interaction,
            new Uri(client.BaseAddress!, uri).Query, signature,
            response.Headers.TryGetValues("X-Next-Cursor", out var cursor) ? cursor.Single() : null);
    }
}
