namespace CarPointCloudWebApi.Domain;

/// <summary>基于实测煤体积和空车车厢内尺寸计算装载程度。</summary>
public static class LoadCargoEstimator
{
    private const decimal LowUpperExclusive = 0.20m;
    private const decimal OneThirdUpperExclusive = 0.45m;
    private const decimal HalfUpperExclusive = 0.75m;
    private const decimal FullUpperInclusive = 1.05m;

    public static LoadCargoEstimate Estimate(LoadMeasurementRecord row, EmptyVehicleProfileRecord? profile)
    {
        var effective = row.LatestRevision?.Calculation ?? row.Measurement;
        var boxCapacity = ComputeBoxCapacityM3(profile);

        if (effective.Verdict == "empty")
            return new(boxCapacity, null, "empty", "空车");

        if (effective.Verdict != "loaded")
            return new(boxCapacity, null, "unknown", "无法确认");

        if (effective.VolumeStatus is not ("success" or "low_confidence")
            || !LoadRadarNumber.InRange(effective.VolumeM3, 0m, LoadRadarPublicationPolicy.VolumeMaxM3)
            || effective.VolumeM3!.Value.ToDecimalForVolume() is not (> 0m and var volumeM3))
        {
            return new(boxCapacity, null, "loaded_pending_volume", "已载货（待体积）");
        }

        if (boxCapacity is not { } capacity)
            return new(null, null, "loaded_unknown_capacity", "已载货（缺容积）");

        var (level, label) = ClassifyVolume(volumeM3, capacity);
        return new(capacity, volumeM3 / capacity, level, label);
    }

    private static decimal? ComputeBoxCapacityM3(EmptyVehicleProfileRecord? profile)
    {
        if (profile is not { DetectStatus: "success", TrailerLength: > 0m and var length,
                TrailerWidth: > 0m and var width,
                TrailerBodyHeight: > 0m and var height })
            return null;

        var capacity = length * width * height / 1_000_000m;
        return capacity > 0m ? capacity : null;
    }

    private static (string Level, string Label) ClassifyVolume(decimal volume, decimal capacity) => volume switch
    {
        _ when volume < capacity * LowUpperExclusive => ("low", "少量"),
        _ when volume < capacity * OneThirdUpperExclusive => ("one_third", "约 1/3"),
        _ when volume < capacity * HalfUpperExclusive => ("half", "半载"),
        _ when volume <= capacity * FullUpperInclusive => ("full", "全载"),
        _ => ("overloaded", "满载超冒")
    };
}
