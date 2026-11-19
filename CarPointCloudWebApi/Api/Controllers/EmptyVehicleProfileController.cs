using Microsoft.AspNetCore.Mvc;

namespace CarPointCloudWebApi.Api;

/// <summary>按车牌读取最新一次有效空车的双雷达融合云图和尺寸。</summary>
[ApiController]
[Route("api/empty-vehicle-profiles")]
[RequireRole(Roles = "admin,viewer")]
public sealed class EmptyVehicleProfileController(IEmptyVehicleProfileRepository repository) : ControllerBase
{
    /// <summary>查询该车牌最新空车档案；不会改变车辆历史数据接口。</summary>
    [HttpGet("{licensePlate}")]
    public ActionResult<ApiResponse<EmptyVehicleProfileDto>> GetLatest(string licensePlate)
    {
        if (!TryNormalizeLicensePlate(licensePlate, out var normalized, out var error))
        {
            return BadRequest(ApiResponse<EmptyVehicleProfileDto>.Fail(400, error));
        }

        var record = repository.GetByLicensePlate(normalized);
        if (record is null)
        {
            return NotFound(ApiResponse<EmptyVehicleProfileDto>.Fail(
                404,
                $"未找到车牌 {normalized} 的有效空车云图"));
        }

        return Ok(ApiResponse<EmptyVehicleProfileDto>.Success(ToDto(record)));
    }

    /// <summary>以二进制 PCD 流返回该车牌最新融合空车云图。</summary>
    [HttpGet("{licensePlate}/pointcloud")]
    public IActionResult GetPointCloud(string licensePlate, [FromQuery] string? expectedWorkflowId = null)
    {
        if (!TryNormalizeLicensePlate(licensePlate, out var normalized, out var error))
        {
            return BadRequest(ApiResponse<object>.Fail(400, error));
        }

        var record = repository.GetByLicensePlate(normalized);
        if (record is null)
        {
            return NotFound(ApiResponse<object>.Fail(404, $"未找到车牌 {normalized} 的有效空车云图"));
        }

        if (expectedWorkflowId is not null && !string.Equals(record.WorkflowId, expectedWorkflowId, StringComparison.Ordinal))
            return Conflict(ApiResponse<object>.Fail(409, "该车牌的最新档案已更新，不是本次采集结果"));

        return PhysicalFile(Path.GetFullPath(record.PointCloudPath), "application/octet-stream",
            $"{normalized}_fused_empty.pcd", enableRangeProcessing: true);
    }

    private EmptyVehicleProfileDto ToDto(EmptyVehicleProfileRecord record) => new()
    {
        ReviewStatus = string.Equals(record.DetectStatus, "low_confidence", StringComparison.OrdinalIgnoreCase) ? "pending" : null,
        LicensePlate = record.LicensePlate,
        SourceRecordId = record.DimensionRecordId,
        WorkflowId = record.WorkflowId,
        PairId = record.PairId,
        CameraIdentifier = record.CameraIdentifier,
        ScanTime = record.ScanTime,
        UpdatedAt = record.UpdatedAt,
        PointCount = record.PointCount,
        FileSize = record.PointCloudSize,
        PointCloudSha256 = record.PointCloudSha256,
        PointCloudUrl = $"/api/empty-vehicle-profiles/{Uri.EscapeDataString(record.LicensePlate)}/pointcloud",
        Dimensions = new VehicleDimensionsDto
        {
            CabToTrailerFrontDist = record.CabToTrailerFrontDist,
            TrailerLength = record.TrailerLength,
            TrailerWidth = record.TrailerWidth,
            TrailerBodyHeight = record.TrailerBodyHeight,
            TrailerBottomToGroundHeight = record.TrailerBottomToGroundHeight,
            HighLowPlateHeight = record.HighLowPlateHeight,
            HighLowPlates = record.HighLowPlates.Select(item => new HighLowPlateDto
            {
                PlateId = item.PlateId,
                HighestToGround = item.HighestToGround,
                LowestToGround = item.LowestToGround
            }).ToList(),
            Ribs = record.Ribs.Select(item => new RibDto
            {
                RibId = item.RibId,
                DistToPrev = item.DistToPrev
            }).ToList(),
            Wires = record.Wires.Select(item => new WireDto
            {
                WireId = item.WireId,
                DistToPrev = item.DistToPrev
            }).ToList()
        }
    };

    private static bool TryNormalizeLicensePlate(
        string? licensePlate,
        out string normalized,
        out string error)
    {
        normalized = licensePlate?.Trim().ToUpperInvariant() ?? string.Empty;
        if (normalized.Length is < 1 or > 32 || normalized.Any(char.IsControl))
        {
            error = "车牌号要求 1-32 位且不能包含控制字符";
            return false;
        }

        error = string.Empty;
        return true;
    }
}
