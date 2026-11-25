
namespace CarPointCloudWebApi.Application;

/// <summary>合同 v4 投影；内部测量、审计元数据与旧最高点高度保持独立。</summary>
public static class PartnerQueryMapper
{
    public static PartnerVehicleDimensionDto Vehicle(VehicleDimensionDto row)
    {
        object dimensions = new { };
        if (row.Dimensions is VehicleDimensionsDto source)
        {
            dimensions = new PartnerVehicleDimensionsDto
            {
                CabToTrailerFrontDist = source.CabToTrailerFrontDist,
                TrailerLength = source.TrailerLength,
                TrailerWidth = row.MaximumWidthCm,
                TrailerBodyHeight = source.TrailerBodyHeight,
                TrailerBottomToGroundHeight = source.TrailerBottomToGroundHeight,
                HighLowPlateHeight = source.HighLowPlateHeight,
                HighLowPlates = source.HighLowPlates.OrderBy(x => x.PlateId).Take(3)
                    .Select((x, index) => new PartnerHighLowPlateDto
                    {
                        PlateId = index + 1,
                        HighestToGround = x.LengthCm,
                        LowestToGround = x.LowestToGround
                    }).ToList(),
                Ribs = source.Ribs.Where(x => x.RibId > 0).OrderBy(x => x.RibId)
                    .Take(Math.Min(row.ActualRibCount ?? source.Ribs.Count, 6))
                    .Select((x, index) => new RibDto
                    {
                        RibId = index + 1,
                        // 旧编号缺口不能假称距离是到“已返回的前一条”。
                        DistToPrev = x.RibId == index + 1 ? x.DistToPrev : null
                    }).ToList()
            };
        }
        return new PartnerVehicleDimensionDto
        {
            LicensePlate = row.LicensePlate,
            CameraIdentifier = row.CameraIdentifier,
            GetTime = PartnerContract.FormatTime(row.GetTime)!,
            Dimensions = dimensions
        };
    }

    public static PartnerCoalVolumeDto Coal(CoalVolumeDto row) => new()
    {
        LicensePlate = row.LicensePlate,
        CameraIdentifier = row.CameraIdentifier,
        CaptureTime = PartnerContract.FormatTime(row.CaptureTime) ?? throw new ArgumentException("煤量缺少实际抓拍时间"),
        EnterId = !string.IsNullOrWhiteSpace(row.EnterId) ? row.EnterId : throw new ArgumentException("煤量缺少原业务 enterID"),
        Volume = row.Volume,
        MeasureTime = PartnerContract.FormatTime(row.MeasurementStartedAt) ?? throw new ArgumentException("煤量缺少实际测量开始时间"),
        MeasureEndTime = PartnerContract.FormatTime(row.MeasurementEndedAt) ?? throw new ArgumentException("煤量缺少实际测量结束时间")
    };
}
