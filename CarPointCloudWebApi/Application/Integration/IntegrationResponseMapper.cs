
namespace CarPointCloudWebApi.Application;

/// <summary>查询记录到现有响应 DTO 的纯映射。</summary>
internal static class IntegrationResponseMapper
{
    internal static VehicleDimensionDto ToVehicleDimensionDto(VehicleDimensionRecord record, bool includeReview = false)
    {
        var pendingReview = includeReview && string.Equals(record.DetectStatus, "low_confidence", StringComparison.OrdinalIgnoreCase);
        object dimensions = new { };
        if (pendingReview || string.Equals(record.DetectStatus, "success", StringComparison.OrdinalIgnoreCase))
        {
            dimensions = new VehicleDimensionsDto
            {
                CabToTrailerFrontDist = Round1(record.CabToTrailerFrontDist),
                TrailerLength = Round1(record.TrailerLength),
                TrailerWidth = Round1(record.TrailerWidth),
                TrailerBodyHeight = Round1(record.TrailerBodyHeight),
                TrailerBottomToGroundHeight = Round1(record.TrailerBottomToGroundHeight),
                HighLowPlateHeight = Round1(record.HighLowPlateHeight),
                HighLowPlates = record.HighLowPlates.OrderBy(x => x.PlateId)
                    .Select(x => new HighLowPlateDto { PlateId = x.PlateId, HighestToGround = Round1(x.HighestToGround), LengthCm = Round1(x.LengthCm), LowestToGround = Round1(x.LowestToGround) })
                    .ToList(),
                Ribs = record.Ribs.OrderBy(x => x.RibId)
                    .Select(x => new RibDto { RibId = x.RibId, DistToPrev = Round1(x.DistToPrev) })
                    .ToList(),
                Wires = record.Wires.OrderBy(x => x.WireId)
                    .Select(x => new WireDto { WireId = x.WireId, DistToPrev = Round1(x.DistToPrev) })
                    .ToList()
            };
        }

        return new VehicleDimensionDto
        {
            ActualRibCount = record.ActualRibCount,
            MaximumWidthCm = Round1(record.MaximumWidthCm),
            ReviewStatus = pendingReview ? "pending" : null,
            RecordId = record.Id,
            WorkflowId = record.WorkflowId,
            PairId = record.PairId,
            LicensePlate = record.LicensePlate,
            CameraIdentifier = record.CameraIdentifier,
            GetTime = record.GetTime,
            Dimensions = dimensions
        };
    }

    internal static CoalVolumeDto ToCoalVolumeDto(CoalVolumeRecord record)
    {
        return new CoalVolumeDto
        {
            RecordId = record.Id,
            LicensePlate = record.LicensePlate,
            CameraIdentifier = record.CameraIdentifier,
            Volume = Round1(record.Volume),
            Unit = string.IsNullOrWhiteSpace(record.Unit) ? "m³" : record.Unit,
            Status = record.Status,
            IsAbnormal = !string.Equals(record.Status, "success", StringComparison.OrdinalIgnoreCase),
            MeasureTime = record.MeasureTime,
            MeasurementStartedAt = record.MeasurementStartedAt,
            MeasurementEndedAt = record.MeasurementEndedAt,
            EnterId = record.EnterId,
            CaptureTime = record.CaptureTime
        };
    }

    internal static decimal? Round1(decimal? value)
    {
        return value.HasValue ? Round1(value.Value) : null;
    }

    /// <summary>
    /// 四舍五入保留1位小数。
    /// </summary>
    /// <remarks>使用银行家舍入（ToEven）</remarks>
    /// <param name="value">数值</param>
    /// <returns>四舍五入后的数值</returns>
    internal static decimal Round1(decimal value)
    {
        return decimal.Round(value, 1, MidpointRounding.ToEven);
    }

}
