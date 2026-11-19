using System.Reflection;
using System.Text.Json;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Swashbuckle.AspNetCore.SwaggerGen;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class LoadRadarNumberTests
{
    private static readonly JsonSerializerOptions Json = new(JsonSerializerDefaults.Web);

    [Fact]
    public void SwaggerDescribesMachineNumbersAsNullableNumbers_NotInternalObjects()
    {
        var services = new ServiceCollection();
        services.AddLogging();
        services.AddControllers();
        services.AddSwaggerGen();
        services.AddLoadRadar(new ConfigurationBuilder().AddInMemoryCollection().Build());
        using var provider = services.BuildServiceProvider();
        var schemas = new SchemaRepository();
        var schema = provider.GetRequiredService<ISchemaGenerator>().GenerateSchema(typeof(LoadMeasurementInput), schemas);
        var model = schemas.Schemas[schema.Reference.Id];
        foreach (var field in new[] { "volumeM3", "filledRatio", "boundSpread", "holeRatio" })
        {
            Assert.Equal("number", model.Properties[field].Type);
            Assert.Equal("double", model.Properties[field].Format);
            Assert.True(model.Properties[field].Nullable);
        }
    }

    // 由修改前 DLL 90181c925b74ff3c4f165809836a3ab1c7bba2e873de9e2e6a0086f8eb2bfe5c
    // 经独立 .NET 8.0.31 回环 HTTP 宿主固化，不从新实现推导预期哈希。
    public static IEnumerable<object[]> LegacyCases()
    {
        yield return ["scale", new[] { "12.3500", "0.10", "0.0200", "0.050" },
            new[] { "12.3500", "0.10", "0.0200", "0.050" }, "9d9bafe36c9565285eb7b608474cbc8bf29ef8fc979acc74d8ed9cfb702748b7", false];
        yield return ["exponent", new[] { "1e-06", "1.0", "2e-02", "1e-1" },
            new[] { "0.000001", "1.0", "0.02", "0.1" }, "661d5080b4fe79259635498e58771329b4119af010a260beb1dbee454a686b71", false];
        yield return ["exact-threshold", new[] { "12.35", "0.4", "0.0300000000000000000000000001", "0.1" },
            new[] { "12.35", "0.4", "0.0300000000000000000000000001", "0.1" }, "066bcc59f17b14ccba5133fea05f0413c349b3fb6bf9069dd3d85bf772818f8c", false];
        yield return ["tiny-spread", new[] { "12.35", "0.4", "1.2345678901234567e-16", "0.1" },
            new[] { "12.35", "0.4", "0.0000000000000001234567890123", "0.1" }, "e4a7df907f1ed06ad8ddc4a5a314a95ee33a46e0756e25660d19747c516c408c", true];
        yield return ["underflow-spread", new[] { "12.35", "0.4", "1e-29", "0.1" },
            new[] { "12.35", "0.4", "0.0000000000000000000000000000", "0.1" }, "4d86c85e722e8c6c97623d4adb98fc7fcac050cdba6493fd32aea87369caeb5e", true];
        yield return ["null", new[] { "null", "null", "null", "null" },
            new[] { "null", "null", "null", "null" }, "4a11d7c50c5f5d8888a6d4d93b76952c5ca86781d9f2501107764d7e2f662174", false];
    }

    private static string LegacyJson(string name, string[] n) =>
        $$"""{"measurementId":"m-{{name}}","nodeId":"probe-node","channelId":"CH1","passageId":"p-legacy","calibrationId":"cal-legacy","sourceMode":"live","algorithmVersion":"v1","configurationHash":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","baselineCandidateSha256":null,"measuredAt":"2024-01-01T00:00:00+00:00","verdict":"loaded","frameDegraded":false,"volumeStatus":"{{(name == "null" ? "unavailable" : "success")}}","volumeM3":{{n[0]}},"baselineVersionId":"b-legacy","filledRatio":{{n[1]}},"boundSpread":{{n[2]}},"holeRatio":{{n[3]}},"pointCount":1,"reason":"legacy","points":[[0.1,0.2,-3.5]]}""";

    [Theory]
    [MemberData(nameof(LegacyCases))]
    public void LegacyStoredJsonAndHashRemainExact_OnlyLostPrecisionRequestsConflict(
        string name, string[] requestNumbers, string[] storedNumbers, string hash, bool precisionWasLost)
    {
        var storedJson = LegacyJson(name, storedNumbers);
        var stored = JsonSerializer.Deserialize<LoadMeasurementInput>(storedJson, Json)!;
        Assert.Equal(storedJson, JsonSerializer.Serialize(stored, Json));
        Assert.Equal(hash, LoadRadarService.Key(stored));
        var input = JsonSerializer.Deserialize<LoadMeasurementInput>(LegacyJson(name, requestNumbers), Json)!;
        LoadRadarService.ValidateInput(input);
        var key = LoadRadarService.Key(input.NodeId, input.ChannelId, input.PassageId);
        Assert.Equal("0cd364a29d56a111afc56d6ccfb7b5d3a6922b32aad52c77f9570585dae50d25", key);
        var original = new LoadMeasurementRecord(key, hash, stored, DateTimeOffset.Parse("2024-01-01T00:01:00Z"));
        var replay = new LoadMeasurementRecord(key, LoadRadarService.Key(input), input, DateTimeOffset.UtcNow);
        var method = typeof(LoadRadarRepository).GetMethod("SameMeasurement", BindingFlags.NonPublic | BindingFlags.Static)!;
        if (precisionWasLost)
        {
            Assert.NotEqual(hash, replay.PayloadHash);
            var failure = Assert.Throws<TargetInvocationException>(() => method.Invoke(null, [original, replay]));
            Assert.Equal(409, Assert.IsType<LoadRadarException>(failure.InnerException).StatusCode);
        }
        else
        {
            Assert.Equal(hash, replay.PayloadHash);
            Assert.Same(original, method.Invoke(null, [original, replay]));
        }
        Assert.Equal(storedJson, JsonSerializer.Serialize(original.Measurement, Json));
    }

    [Theory]
    [InlineData("1.2345678901234567e-16")]
    [InlineData("1e-29")]
    [InlineData("5e-324")]
    [InlineData("1e100")]
    [InlineData("0.030000000000000002")]
    public void FiniteDouble_RoundTripsAndPersistsWithoutTolerance(string raw)
    {
        var number = JsonSerializer.Deserialize<LoadRadarNumber>(raw);
        var json = JsonSerializer.Serialize(number);
        Assert.Equal(double.Parse(raw, System.Globalization.CultureInfo.InvariantCulture), JsonDocument.Parse(json).RootElement.GetDouble());
        Assert.Equal(json, JsonSerializer.Serialize(JsonSerializer.Deserialize<LoadRadarNumber>(json)));
    }

    [Theory]
    [InlineData("\"0.1\"")]
    [InlineData("NaN")]
    [InlineData("Infinity")]
    [InlineData("1e400")]
    public void InvalidOrNonfiniteJson_IsRejected(string raw) =>
        Assert.Throws<JsonException>(() => JsonSerializer.Deserialize<LoadRadarNumber>(raw));

    [Theory]
    [InlineData("0.0300000000000000000000000001", "0.03")]
    [InlineData("0.1500000000000000000000000001", "0.15")]
    [InlineData("1.0000000000000000000000000001", "1")]
    [InlineData("-1e-29", "1")]
    public void DecimalPrecisionAndNegativeTinyValues_CannotCrossBusinessGates(string raw, string maximum)
    {
        Assert.False(LoadRadarNumber.InRange(JsonSerializer.Deserialize<LoadRadarNumber>(raw), 0m,
            decimal.Parse(maximum, System.Globalization.CultureInfo.InvariantCulture)));
    }

    [Theory]
    [InlineData("45.14999999999999", "45.1")]
    [InlineData("45.15", "45.2")]
    [InlineData("45.150000000000006", "45.2")]
    [InlineData("1e-29", "0.0")]
    public void CoalRounding_DoesNotPreRoundDoubleToFifteenDigits(string raw, string expected)
    {
        var parsed = JsonSerializer.Deserialize<LoadRadarNumber>(raw);
        Assert.Equal(decimal.Parse(expected, System.Globalization.CultureInfo.InvariantCulture), parsed.RoundCoalVolume());
        var fromDouble = LoadRadarNumber.FromDouble(double.Parse(raw, System.Globalization.CultureInfo.InvariantCulture));
        Assert.Equal(parsed.RoundCoalVolume(), fromDouble.RoundCoalVolume());
    }

    [Fact]
    public void ProgrammaticNonfiniteValuesAreRejected()
    {
        foreach (var value in new[] { double.NaN, double.NegativeInfinity, double.PositiveInfinity })
            Assert.Throws<ArgumentOutOfRangeException>(() => LoadRadarNumber.FromDouble(value));
    }
}
