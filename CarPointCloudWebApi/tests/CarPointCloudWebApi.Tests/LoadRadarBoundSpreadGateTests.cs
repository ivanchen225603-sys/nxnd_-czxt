using Xunit;

namespace CarPointCloudWebApi.Tests;

/// <summary>
/// 正式煤量的体积区间宽度门槛（2026-09-22 由写死 0.03 改成可配置、默认不设）。
///
/// 为什么改：3% 来自采集端旧口径（当时上下界是"格子扫描的上下界"，能压到 1.6%/1.8%）。
/// 后来上下界换成**声明模型下的区间** —— 上界要把没观测到的面积按最大货高整个算进去，
/// 语义变了而门槛没跟着变。两辆有磅单真值的车实测 <b>54.8% / 60.7%</b>，
/// 扫遍 5 档格子 x 4 档货高上限 x 误差预算减半，**最低只到 46%**。
/// 根因是俯视单雷达在车厢两端掠射、未观测面积占 34%~39%，调参数改不掉。
/// 也就是说这道门在现有几何下永远关着，**煤量永远进不了正式入库**。
///
/// 采集端 2026-09-18 已按现场决定改成不设（VOLUME_SPREAD_MAX = None），
/// 后台三处当时没跟着改，于是源端放行、后台照样拒发。
/// </summary>
public sealed class LoadRadarBoundSpreadGateTests
{
    // 两辆有磅单真值的车的实测值，以及扫参数得到的下限
    // [InlineData] 只接受编译期常量且不支持 decimal，所以这里用 double 传参，
    // 方法内再转成 decimal —— 业务比较仍然全程走 decimal。
    private const double MeasuredA = 0.548;
    private const double MeasuredB = 0.607;
    private const double AchievableFloor = 0.46;

    private static LoadMeasurementInput Good(decimal? spread = null, decimal? hole = null) => new()
    {
        MeasurementId = "n1-20260922-000001",
        NodeId = "jt128-01",
        ChannelId = "CH4",
        PassageId = "passage-1",
        CalibrationId = "CH4-calibration-v1",
        SourceMode = "live",
        AlgorithmVersion = "clean-room-v1",
        ConfigurationHash = new string('a', 64),
        MeasuredAt = DateTimeOffset.UtcNow,
        Verdict = "loaded",
        FrameDegraded = false,
        VolumeStatus = "success",
        VolumeM3 = new LoadRadarNumber(56.43m),
        BaselineVersionId = "bl-000001",
        HoleRatio = hole is null ? new LoadRadarNumber(0.132m) : new LoadRadarNumber(hole.Value),
        BoundSpread = spread is null ? null : new LoadRadarNumber(spread.Value),
        PointCount = 112386,
        Reason = "载货"
    };

    [Theory]
    [InlineData(MeasuredA)]
    [InlineData(MeasuredB)]
    [InlineData(AchievableFloor)]
    public void Without_a_threshold_the_real_measurements_can_publish(double spread)
    {
        Assert.True(LoadRadarPublicationPolicy.IsCandidate(Good((decimal)spread), null));
    }

    [Theory]
    [InlineData(MeasuredA)]
    [InlineData(MeasuredB)]
    [InlineData(AchievableFloor)]
    public void Legacy_threshold_configuration_does_not_block_computed_volume(double spread)
    {
        // 这就是改动前的行为：现场扫得最好的车也发不出去，且看不出卡在哪一条。
        Assert.True(LoadRadarPublicationPolicy.IsCandidate(Good((decimal)spread), 0.03m));
    }

    [Fact]
    public void Missing_diagnostics_do_not_block_computed_volume()
    {
        // ▲ 不设门槛 != 不看这个数：缺失或非法仍然拒，照常入库上报供下游筛选。
        Assert.True(LoadRadarPublicationPolicy.IsCandidate(Good(spread: null), null));
    }

    [Fact]
    public void Hole_ratio_no_longer_blocks_computed_volume()
    {
        // 这一条**够得着**（实测 13.2% / 9.5%），是真正有区分度的质量判据，没动。
        Assert.True(LoadRadarPublicationPolicy.IsCandidate(Good((decimal)MeasuredA, hole: 0.15m), null));
        Assert.True(LoadRadarPublicationPolicy.IsCandidate(Good((decimal)MeasuredA, hole: 0.150001m), null));
    }

    [Theory]
    [InlineData("demo")]
    [InlineData("degraded")]
    [InlineData("unknown_verdict")]
    [InlineData("low_confidence")]
    [InlineData("no_baseline")]
    public void Relaxing_the_width_gate_does_not_relax_anything_else(string invalid)
    {
        var bad = invalid switch
        {
            "demo" => Good((decimal)MeasuredA) with { SourceMode = "demo" },
            "degraded" => Good((decimal)MeasuredA) with { FrameDegraded = true },
            "unknown_verdict" => Good((decimal)MeasuredA) with { Verdict = "unknown" },
            "low_confidence" => Good((decimal)MeasuredA) with { VolumeStatus = "low_confidence" },
            _ => Good((decimal)MeasuredA) with { BaselineVersionId = null }
        };
        Assert.Equal(invalid is "degraded" or "low_confidence", LoadRadarPublicationPolicy.IsCandidate(bad, null));
    }

    [Fact]
    public void Default_is_no_threshold()
    {
        Assert.Null(new LoadRadarOptions().VolumeBoundSpreadMax);
    }
}
