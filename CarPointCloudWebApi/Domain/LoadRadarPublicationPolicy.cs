
namespace CarPointCloudWebApi.Domain;

/// <summary>筛选已计算体积；发布事务继续核对身份、基线和数值有效性。</summary>
/// <remarks>
/// ★ 质量门槛此前在三处各写一份（本类、<c>LoadRadarService.Publish</c>、
/// <c>LoadRadarRepository.Publish</c>），数字一模一样。改一处另两处不会跟着动，
/// 现象是"候选进了队列却在发布事务里被拒"，而两边报的理由不一样。
/// 现在统一走本类，质量指标不再作为发布门槛。
/// </remarks>
public static class LoadRadarPublicationPolicy
{
    /// <summary>体积上限（m^3）。超出即认为不是一车煤。</summary>
    public const decimal VolumeMaxM3 = 1000m;

    /// <summary>历史诊断参考值，保留常量兼容性，不作为发布条件。</summary>
    public const decimal HoleRatioMax = 0.15m;

    // 2026-11-03：已计算体积直接发布；保留签名以兼容原调用方和配置。
    // 空洞比例、上下界宽度是诊断数据，不参与是否入库的判断。
    public static bool MeetsQuality(LoadMeasurementInput input, decimal? boundSpreadMax) =>
        LoadRadarNumber.InRange(input.VolumeM3, 0m, VolumeMaxM3);

    public static bool MatchesBaseline(LoadMeasurementInput input, LoadPassageBinding binding,
        LoadBaselineVersion baseline, LoadMeasurementInput source, bool retrospective = false) =>
        baseline.ChannelId == "CH4" && baseline.NodeId == input.NodeId && baseline.LicensePlate == binding.LicensePlate &&
        baseline.ActivatedAt is { } activated && (retrospective || activated <= input.MeasuredAt) &&
        source.ChannelId == baseline.ChannelId && source.CalibrationId == baseline.CalibrationId &&
        source.SourceMode == "live" && source.Verdict == "empty" && !source.FrameDegraded &&
        source.BaselineCandidateSha256 == baseline.AssetSha256 &&
        (input.BaselineUse is { } use
            ? use.SourceChannelId == baseline.ChannelId && use.SourceCalibrationId == baseline.CalibrationId &&
              use.SourceConfigurationHash == source.ConfigurationHash
            : input.ChannelId == "CH4" && input.CalibrationId == baseline.CalibrationId &&
              input.ConfigurationHash == source.ConfigurationHash && binding.VehicleIdentity == baseline.VehicleIdentity);

    public static bool IsCandidate(LoadMeasurementInput input, decimal? boundSpreadMax) =>
        input.SourceMode == "live" && input.Verdict == "loaded" &&
        input.VolumeStatus is "success" or "low_confidence" && !string.IsNullOrWhiteSpace(input.BaselineVersionId) &&
        MeetsQuality(input, boundSpreadMax);
}
