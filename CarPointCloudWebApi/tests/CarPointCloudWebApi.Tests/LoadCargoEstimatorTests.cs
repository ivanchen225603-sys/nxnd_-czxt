using Dm;
using Microsoft.Extensions.Configuration;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class LoadCargoEstimatorTests
{
    private static EmptyVehicleProfileRecord Profile(decimal lengthCm = 1000m, decimal widthCm = 250m, decimal heightCm = 200m) => new()
    {
        LicensePlate = "宁A88801",
        TrailerLength = lengthCm,
        TrailerWidth = widthCm,
        TrailerBodyHeight = heightCm
    }; // 10m * 2.5m * 2m = 50.000 m³

    private static LoadMeasurementRecord Record(
        string verdict = "loaded",
        string volumeStatus = "success",
        decimal? volumeM3 = 25m,
        string channelId = "CH1",
        decimal? revisedVolumeM3 = null)
    {
        var input = new LoadMeasurementInput
        {
            MeasurementId = "m-1",
            NodeId = "jt128-01",
            ChannelId = channelId,
            PassageId = "p-1",
            Verdict = verdict,
            VolumeStatus = volumeStatus,
            VolumeM3 = volumeM3 is { } v ? new LoadRadarNumber(v) : null,
            MeasuredAt = DateTimeOffset.UtcNow
        };
        LoadVolumeRevision? revision = revisedVolumeM3 is { } rv
            ? new LoadVolumeRevision("rev-1", "m-1", 1, "base-1", 10, volumeM3, rv, null, "hash-rev",
                input with { VolumeM3 = new LoadRadarNumber(rv), VolumeStatus = "success", Verdict = "loaded" },
                new LoadRecalculationProvenance("a", "b", "c", "d"), DateTimeOffset.UtcNow)
            : null;
        return new LoadMeasurementRecord("pk-1", "hash-1", input, DateTimeOffset.UtcNow, LatestRevision: revision);
    }

    [Theory]
    [InlineData(5, "low", "少量", 0.10)]
    [InlineData(16.5, "one_third", "约 1/3", 0.33)]
    [InlineData(27.5, "half", "半载", 0.55)]
    [InlineData(45, "full", "全载", 0.90)]
    [InlineData(55, "overloaded", "满载超冒", 1.10)]
    public void Classifies_cargo_fullness_from_measured_volume(
        double volume, string expectedLevel, string expectedLabel, double expectedRatio)
    {
        var estimate = LoadCargoEstimator.Estimate(Record(volumeM3: (decimal)volume), Profile());

        Assert.Equal(50.000m, estimate.BoxCapacityM3);
        Assert.Equal((decimal)expectedRatio, estimate.LoadRatio);
        Assert.Equal(expectedLevel, estimate.LoadLevel);
        Assert.Equal(expectedLabel, estimate.LoadLevelLabel);
    }

    [Fact]
    public void Empty_vehicle_has_no_load_ratio()
    {
        var emptyEstimate = LoadCargoEstimator.Estimate(
            Record(verdict: "empty", volumeStatus: "unavailable", volumeM3: null),
            Profile());
        Assert.Equal("empty", emptyEstimate.LoadLevel);
        Assert.Equal("空车", emptyEstimate.LoadLevelLabel);
        Assert.Null(emptyEstimate.LoadRatio);
    }

    [Fact]
    public void Missing_empty_profile_leaves_fullness_unknown()
    {
        var estimate = LoadCargoEstimator.Estimate(
            Record(volumeM3: 30m), null);

        Assert.Null(estimate.BoxCapacityM3);
        Assert.Null(estimate.LoadRatio);
        Assert.Equal("loaded_unknown_capacity", estimate.LoadLevel);
        Assert.Equal("已载货（缺容积）", estimate.LoadLevelLabel);
    }

    [Fact]
    public void Revised_volume_takes_precedence_for_ch4()
    {
        var estimate = LoadCargoEstimator.Estimate(
            Record(volumeM3: 10m, channelId: "CH4", revisedVolumeM3: 40m), Profile());

        Assert.Equal("full", estimate.LoadLevel);
        Assert.Equal(0.8000m, estimate.LoadRatio);
    }

    [Theory]
    [InlineData(9.99999, "low")]
    [InlineData(10, "one_third")]
    [InlineData(22.49999, "one_third")]
    [InlineData(22.5, "half")]
    [InlineData(37.49999, "half")]
    [InlineData(37.5, "full")]
    [InlineData(52.5, "full")]
    [InlineData(52.50001, "overloaded")]
    public void Classifies_against_unrounded_thresholds(double volume, string level)
    {
        var cargo = LoadCargoEstimator.Estimate(Record(volumeM3: (decimal)volume), Profile());
        Assert.Equal(level, cargo.LoadLevel);
    }

    [Fact]
    public void Uses_full_capacity_and_exact_decimal_for_classification()
    {
        var capacity = 1001m * 249.9m * 199.9m / 1_000_000m;
        var cargo = LoadCargoEstimator.Estimate(Record(volumeM3: capacity * 0.75m), Profile(1001m, 249.9m, 199.9m));
        Assert.Equal(capacity, cargo.BoxCapacityM3);
        Assert.Equal("full", cargo.LoadLevel);
        Assert.Equal(0.75m, cargo.LoadRatio);
        var below = LoadCargoEstimator.Estimate(Record(volumeM3: 9.9999999999999999999999999m), Profile());
        Assert.Equal("low", below.LoadLevel);
    }

    [Theory]
    [InlineData(null, "success")]
    [InlineData(0d, "success")]
    [InlineData(-1d, "success")]
    [InlineData(1001d, "success")]
    [InlineData(25d, "unavailable")]
    public void Missing_or_unusable_volume_has_no_load_ratio(double? volume, string status)
    {
        var cargo = LoadCargoEstimator.Estimate(Record(volumeStatus: status, volumeM3: (decimal?)volume),
            Profile());
        Assert.Equal("loaded_pending_volume", cargo.LoadLevel);
        Assert.Null(cargo.LoadRatio);
    }

    [Theory]
    [InlineData(double.Epsilon)]
    [InlineData(-double.Epsilon)]
    public void Subnormal_double_cannot_be_decimalised_into_a_false_zero_ratio(double volume)
    {
        var row = Record();
        row = row with { Measurement = row.Measurement with { VolumeM3 = LoadRadarNumber.FromDouble(volume) } };
        var cargo = LoadCargoEstimator.Estimate(row, Profile());
        Assert.Equal("loaded_pending_volume", cargo.LoadLevel);
        Assert.Null(cargo.LoadRatio);
    }

    [Fact]
    public void Missing_nonpositive_or_unreviewed_dimensions_leave_fullness_unknown()
    {
        var profiles = new[] { Profile(), Profile(widthCm: 0m), Profile(heightCm: -1m), Profile() };
        profiles[0].TrailerLength = null;
        profiles[3].DetectStatus = "low_confidence";
        foreach (var profile in profiles)
        {
            var cargo = LoadCargoEstimator.Estimate(Record(volumeStatus: "low_confidence"), profile);
            Assert.Null(cargo.BoxCapacityM3);
            Assert.Equal("loaded_unknown_capacity", cargo.LoadLevel);
            Assert.Null(cargo.LoadRatio);
        }
    }

    [DatabaseFact]
    public void Batch_profile_query_reads_nullable_dimensions_and_detect_status_from_dm8()
    {
        var dsn = TestConfiguration.GetDatabaseConnectionString()!;
        var configuration = new ConfigurationBuilder().AddInMemoryCollection(
            new Dictionary<string, string?> { ["ConnectionStrings:CarPointCloudDb"] = dsn }).Build();
        var repository = new EmptyVehicleProfileRepository(configuration);
        var prefix = "cargo-" + Guid.NewGuid().ToString("N")[..16];
        using var db = new DmConnection(dsn);
        db.Open();

        void Execute(string sql, string plate)
        {
            using var command = new DmCommand(sql, db);
            command.Parameters.Add("@Plate", DmDbType.VarChar, 32).Value = plate;
            command.ExecuteNonQuery();
        }

        try
        {
            Execute("""
                INSERT INTO VehicleDimensionRecord (LicensePlate, CameraIdentifier, GetTime, DetectStatus)
                VALUES (?, 'CAM_004', SYSDATE, 'low_confidence')
                """, prefix);
            Execute("""
                INSERT INTO EmptyVehicleProfile
                    (LicensePlate, DimensionRecordId, PairId, CameraIdentifier, PointCloudPath, PointCloudSha256,
                     PointCloudSize, PointCount, ScanTime, TrailerLength, TrailerWidth, TrailerBodyHeight)
                SELECT LicensePlate, Id, 'cargo-test', 'CAM_004', 'cargo-test.pcd', 'hash',
                       1, 1, SYSDATE, 1000.1, 249.9, NULL
                FROM VehicleDimensionRecord WHERE LicensePlate=?
                """, prefix);

            var profiles = repository.GetDimensionsByLicensePlates([prefix, prefix + "-missing"]);
            var profile = Assert.Single(profiles).Value;
            Assert.Equal("low_confidence", profile.DetectStatus);
            Assert.Equal(1000.1m, profile.TrailerLength);
            Assert.Equal(249.9m, profile.TrailerWidth);
            Assert.Null(profile.TrailerBodyHeight);
        }
        finally
        {
            Execute("DELETE FROM EmptyVehicleProfile WHERE LicensePlate=?", prefix);
            Execute("DELETE FROM VehicleDimensionRecord WHERE LicensePlate=?", prefix);
        }
    }
}
