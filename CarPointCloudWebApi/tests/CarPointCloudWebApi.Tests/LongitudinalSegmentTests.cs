using System.Globalization;
using Microsoft.Extensions.Configuration;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public class LongitudinalSegmentTests
{
    private sealed class RecordingRepository : NullIntegrationRepository
    {
        public VehicleDimensionRecord? Record { get; private set; }
        public override Task<long> InsertVehicleDimensionRecordAsync(VehicleDimensionRecord record, CancellationToken cancellationToken = default)
        {
            Record = record;
            return Task.FromResult(1L);
        }
    }

    [Theory]
    [InlineData("1134.4", "1134.4", null, true)]
    [InlineData("3000", "3000", null, true)]
    [InlineData("1134.4", "500", "634.4", true)]
    [InlineData("960.04", "960.04", null, true)]
    [InlineData("100", "40", null, true)]
    [InlineData(null, "2000", null, true)]
    [InlineData("0", "0", null, true)]
    [InlineData("100", "100.1", null, false)]
    [InlineData("100", "60", "60", false)]
    [InlineData("0", "0.1", null, false)]
    [InlineData(null, "2000", "1000.1", false)]
    [InlineData("3000", "3000.01", null, false)]
    [InlineData("100", "-0.1", null, false)]
    [InlineData("3000.1", "1", null, false)]
    public async Task SegmentsUseCarriageBoundsForBothDetailTypes(string? length, string? first, string? second, bool accepted)
    {
        foreach (var wire in new[] { false, true })
        {
            var repository = new RecordingRepository();
            var config = new ConfigurationBuilder().AddInMemoryCollection(new Dictionary<string, string?> { ["Database:Enabled"] = "true" }).Build();
            var service = new IntegrationQueryService(repository, config);
            static decimal? Parse(string? value) => value is null ? null : decimal.Parse(value, CultureInfo.InvariantCulture);
            var request = new PointCloudDimensionIngestRequest
            {
                IdempotencyKey = Guid.NewGuid().ToString("N"),
                CameraIdentifier = "CAM_004",
                DetectStatus = "success",
                Dimensions = new PointCloudDimensionsPayload { TrailerLength = Parse(length) },
                Ribs = wire ? [] : [new() { RibId = 1, DistToPrev = Parse(first) }, new() { RibId = 2, DistToPrev = Parse(second) }],
                Wires = wire ? [new() { WireId = 1, DistToPrev = Parse(first) }, new() { WireId = 2, DistToPrev = Parse(second) }] : []
            };
            if (accepted)
            {
                Assert.Equal(1, await service.IngestPointCloudDimensionsAsync(request));
                Assert.NotNull(repository.Record);
                Assert.Equal(Math.Round(Parse(first)!.Value, 1, MidpointRounding.AwayFromZero),
                    wire ? repository.Record.Wires[0].DistToPrev : repository.Record.Ribs[0].DistToPrev);
            }
            else
            {
                await Assert.ThrowsAsync<ArgumentException>(() => service.IngestPointCloudDimensionsAsync(request));
                Assert.Null(repository.Record);
            }
        }
    }
}
