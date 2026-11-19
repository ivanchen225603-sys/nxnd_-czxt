using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Logging.Abstractions;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class EmptyVehicleProfileControllerTests : IDisposable
{
    private readonly string _storageRoot = Path.Combine(
        Path.GetTempPath(),
        $"carpoint-empty-profile-{Guid.NewGuid():N}");

    [Fact]
    public void Stored_review_profile_keeps_quality_label_and_dimensions()
    {
        var record = CreateRecord();
        record.DetectStatus = "low_confidence";
        var result = CreateController(record).GetLatest("川A12047");
        var response = Assert.IsType<ApiResponse<EmptyVehicleProfileDto>>(Assert.IsType<OkObjectResult>(result.Result).Value);
        Assert.Equal("pending", response.Data!.ReviewStatus);
        Assert.Equal(record.TrailerLength, response.Data.Dimensions.TrailerLength);
        Assert.NotEmpty(response.Data.PointCloudUrl);
    }

    [Fact]
    public void GetLatest_returns_latest_profile_metadata_and_dimensions()
    {
        var record = CreateRecord();
        var controller = CreateController(record);

        var action = controller.GetLatest(" 川a12047 ");

        var ok = Assert.IsType<OkObjectResult>(action.Result);
        var response = Assert.IsType<ApiResponse<EmptyVehicleProfileDto>>(ok.Value);
        Assert.Equal("川A12047", response.Data!.LicensePlate);
        Assert.Equal(960.0m, response.Data.Dimensions.TrailerLength);
        Assert.Single(response.Data.Dimensions.HighLowPlates);
        Assert.EndsWith("/pointcloud", response.Data.PointCloudUrl);
    }

    [Fact]
    public void GetPointCloud_serves_the_profile_file_with_range_support()
    {
        Directory.CreateDirectory(_storageRoot);
        var pcdPath = Path.Combine(_storageRoot, "plate", "latest.pcd");
        Directory.CreateDirectory(Path.GetDirectoryName(pcdPath)!);
        File.WriteAllText(pcdPath, "PCD");
        var record = CreateRecord();
        record.PointCloudPath = pcdPath;
        var controller = CreateController(record);

        var result = controller.GetPointCloud("川A12047");

        var file = Assert.IsType<PhysicalFileResult>(result);
        Assert.Equal(Path.GetFullPath(pcdPath), file.FileName);
        Assert.True(file.EnableRangeProcessing);
    }

    [Fact]
    public void GetPointCloud_rejects_an_archive_from_another_workflow()
    {
        var controller = CreateController(CreateRecord());
        var result = Assert.IsType<ConflictObjectResult>(controller.GetPointCloud("川A12047", "previous-workflow"));
        Assert.Equal(409, result.StatusCode);
    }

    private EmptyVehicleProfileController CreateController(EmptyVehicleProfileRecord? record)
    {
        var configuration = new ConfigurationBuilder()
            .AddInMemoryCollection(new Dictionary<string, string?>
            {
                ["EmptyVehicleProfile:StorageRoot"] = _storageRoot
            })
            .Build();
        return new EmptyVehicleProfileController(new StubRepository(record));
    }

    private static EmptyVehicleProfileRecord CreateRecord() => new()
    {
        Id = 1,
        LicensePlate = "川A12047",
        DimensionRecordId = 18,
        WorkflowId = "workflow-18",
        PairId = "pair-18",
        CameraIdentifier = "CAM_004",
        PointCloudPath = "/data/carpointcloud/empty-vehicle-profiles/test.pcd",
        PointCloudSha256 = new string('a', 64),
        PointCloudSize = 1024,
        PointCount = 200,
        ScanTime = new DateTime(2026, 8, 13, 9, 0, 0),
        TrailerLength = 960.0m,
        TrailerWidth = 255.0m,
        TrailerBodyHeight = 180.0m,
        HighLowPlates =
        [
            new HighLowPlateInfo
            {
                PlateId = 1,
                HighestToGround = 142.0m,
                LowestToGround = 95.0m
            }
        ],
        CreatedAt = DateTime.Now,
        UpdatedAt = DateTime.Now
    };

    public void Dispose()
    {
        if (Directory.Exists(_storageRoot))
        {
            Directory.Delete(_storageRoot, recursive: true);
        }
    }

    private sealed class StubRepository(EmptyVehicleProfileRecord? record)
        : IEmptyVehicleProfileRepository
    {
        public EmptyVehicleProfileRecord? GetByLicensePlate(string licensePlate)
        {
            Assert.Equal("川A12047", licensePlate);
            return record;
        }
    }
}
