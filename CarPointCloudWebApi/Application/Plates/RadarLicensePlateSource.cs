using System.Collections.Concurrent;

namespace CarPointCloudWebApi.Application;

/// <summary>
/// <c>GET /api/v1/radar/licensePlate</c> 背后的取车牌来源；载货雷达到车登记共用同一个实例。
/// </summary>
/// <remarks>
/// <para>
/// ★ 车牌真假由这里（API 侧）负责，调用方拿到什么车牌就用什么 —— 载货雷达只管测量、存盘、上报
/// （2026-09-28 现场定）。以后换成真实识牌来源，只改这里，接口与载货雷达正式模式一起跟着变。
/// </para>
/// <para>
/// 2026-09-28 现场：载货雷达正式模式原来只认乙方识牌任务，而现场未接乙方
/// （<c>Integration:ExternalPartnerEnabled=false</c>），到车登记一律 503，停稳的车一辆都没测；
/// 同一时刻本地模式调本接口是通的。
/// </para>
/// </remarks>
public interface IRadarLicensePlateSource
{
    /// <summary>本来源是否对外可用；与 <c>/api/v1/radar/licensePlate</c> 返回 404 的判断同一口径。</summary>
    bool Enabled { get; }

    /// <summary>取点位当前车牌；不可用时返回 null。</summary>
    RadarLicensePlate? Resolve(string cameraIdentifier);
}

/// <summary>一次取牌结果。<paramref name="Source"/> 原样记入来车记录的 PlateSource。</summary>
public sealed record RadarLicensePlate(string CameraId, string LicensePlate, DateTime CaptureTime, string Source);

/// <summary>当前唯一实现：模拟车牌（<c>Radar:LicensePlateMock*</c>）。从 <c>IntegrationController</c> 原样搬来。</summary>
public sealed class MockRadarLicensePlateSource(IConfiguration configuration, IHostEnvironment environment) : IRadarLicensePlateSource
{
    /// <summary>
    /// 模拟车牌按点位缓存：<c>cameraIdentifier -> (车牌, 过期时刻)</c>。
    /// </summary>
    /// <remarks>
    /// ★ 为什么不是"每次调用都随机"：载货雷达的空车基线<b>按车牌存</b>，
    /// 一辆车停留期间可能读好几次牌（到车登记一次、开窗预取一次、标定再读一次）。
    /// 每次都换牌的话，同一辆车会被拆成好几辆，基线落不到对的地方。
    /// 所以同一点位在一个停留窗口内返回同一块牌，窗口过了（下一辆车）才换新牌。
    /// 本类按单例注册，接口与到车登记读到的是同一份缓存。
    /// </remarks>
    private readonly ConcurrentDictionary<string, (string Plate, DateTime ExpiresUtc)> _plateByCamera = new();

    /// <summary>
    /// 可见性由 <c>Radar:LicensePlateMockEnabled</c> 控制（默认 <c>false</c>）；开发环境无需配置即可用。
    /// </summary>
    public bool Enabled => configuration.GetValue("Radar:LicensePlateMockEnabled", false) || environment.IsDevelopment();

    /// <inheritdoc />
    public RadarLicensePlate? Resolve(string cameraIdentifier) =>
        Enabled ? new(cameraIdentifier, ResolvePlate(cameraIdentifier), BusinessClock.Now, "MOCK") : null;

    /// <remarks>
    /// <para>
    /// <c>Radar:LicensePlateMockRandom</c> 为 <c>true</c> 时用
    /// <see cref="MockLicensePlateGenerator"/> 随机生成（前缀取
    /// <c>Radar:LicensePlateMockPrefix</c>，默认 <c>宁A</c>），并在
    /// <c>Radar:LicensePlateMockHoldSeconds</c>（默认 60 秒，长于一辆车的停留）
    /// 内对同一点位保持不变；为 <c>false</c> 时保持原来的固定车牌。
    /// </para>
    /// <para>
    /// ★ 固定车牌的坏处（2026-09-23）：载货雷达空车基线改为按车牌存之后，
    /// 所有车都读到同一块牌 = 共用同一份基线 = 体积算出来但<b>全错且不报错</b>。
    /// 随机车牌至少让不同的车落到不同的基线上；跨车道配对（空车 CH2、装货 CH4）
    /// 仍然需要真实识别源 —— 模拟值认不出"是同一辆车"，<c>source</c> 仍标 <c>MOCK</c>。
    /// </para>
    /// </remarks>
    private string ResolvePlate(string cameraIdentifier)
    {
        if (!configuration.GetValue("Radar:LicensePlateMockRandom", false))
        {
            return "宁A10333F";
        }

        var prefix = configuration.GetValue<string>("Radar:LicensePlateMockPrefix") ?? "宁A";
        var hold = Math.Clamp(configuration.GetValue("Radar:LicensePlateMockHoldSeconds", 60), 1, 3600);
        var now = DateTime.UtcNow;
        var entry = _plateByCamera.AddOrUpdate(
            cameraIdentifier,
            _ => (MockLicensePlateGenerator.Generate(prefix), now.AddSeconds(hold)),
            (_, existing) => existing.ExpiresUtc > now
                ? existing
                : (MockLicensePlateGenerator.Generate(prefix), now.AddSeconds(hold)));
        return entry.Plate;
    }
}
