using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Api;

/// <summary>分别控制乙方只读查询与合作方完整链路，保留现有自有雷达入口。</summary>
public sealed class ExternalPartnerGateMiddleware(RequestDelegate next, IOptions<IntegrationOptions> options)
{
    /// <remarks>
    /// <c>/api/v1/internal/pointcloud</c> 下的两个端点归属不同：<c>coalVolumes</c> 是第三方推送煤量的入口，
    /// 属合作方链路；<c>vehicleDimensions</c> 是 Worker 自己回传车厢尺寸的入口，属自有链路。
    /// 因此这里只拦前者。按整个前缀拦会把 Worker 的上传一并挡掉——而那正是
    /// <see cref="Controllers.IntegrationController.IngestPointCloudDimensions"/> 注释里写明的
    /// 「DM8 直写出问题时把 EnableResultUploadToWebApi 翻回 true」的应急通道，
    /// 恰好只在自有链路模式下才需要，堵死它等于同时失去两条入库路径。
    /// </remarks>
    private static readonly string[] ExternalRoutes = [
        "/api/v1/radar", "/api/v1/licensePlateRecords", "/api/v1/vehicleDimensions",
        "/api/v1/coalVolumes", "/api/v1/internal/pointcloud/coalVolumes"
    ];

    /// <remarks>
    /// <c>/api/v1/radar</c> 下的端点方向相反，不能一起拦：
    /// <c>vehicle-arrival</c> 是合作方<b>推</b>来车，合作方关闭时必须堵死；
    /// <c>licensePlate</c> 是我方设备（JT128 载货雷达）按点位<b>拉</b>车牌，属自有链路。
    /// 按整个 <c>/api/v1/radar</c> 前缀拦会把自有链路的取数口一并挡掉，
    /// 表现为载货雷达每次取牌都收到 503「合作方接入已关闭」——
    /// 而那条提示会把排查方向引到合作方，实际和合作方无关。
    ///
    /// 这里只放行拉取口；它自身的可见性另有
    /// <c>Radar:LicensePlateMockEnabled</c> 开关和
    /// <c>Radar:AllowedDeviceAddresses</c> 来源白名单把关，不是无条件敞开。
    /// </remarks>
    private static readonly string[] SelfOwnedRoutes = [
        "/api/v1/radar/licensePlate"
    ];

    /// <summary>请求被阻止时不执行控制器、不创建来车记录、不调用合作方。</summary>
    public async Task InvokeAsync(HttpContext context)
    {
        var settings = options.Value;
        // 合同 v4 的乙方入站只有两个 GET 查询。精确匹配路径及方法，不能连带打开
        // 来车触发、煤量写入或 /licensePlateRecords/fetch 的甲方识牌外呼。
        var path = context.Request.Path;
        var isQuery = HttpMethods.IsGet(context.Request.Method) && PartnerResponseFactory.IsPartnerQueryPath(path);
        if (isQuery && !PartnerResponseFactory.IsPartnerQuery(path))
        {
            context.Response.StatusCode = StatusCodes.Status404NotFound;
            await context.Response.WriteAsJsonAsync(PartnerApiResponse<object>.Fail(404, "查询资源不存在"));
            return;
        }
        var enabled = isQuery ? settings.ExternalQueriesEnabled ?? settings.ExternalPartnerEnabled
            : settings.ExternalPartnerEnabled;
        if (!enabled
            && !SelfOwnedRoutes.Any(prefix =>
                context.Request.Path.StartsWithSegments(prefix, StringComparison.OrdinalIgnoreCase))
            && ExternalRoutes.Any(prefix =>
                context.Request.Path.StartsWithSegments(prefix, StringComparison.OrdinalIgnoreCase)))
        {
            context.Response.StatusCode = StatusCodes.Status503ServiceUnavailable;
            await context.Response.WriteAsJsonAsync(PartnerResponseFactory.Fail(context.Request.Path, 503, "合作方接入已关闭，请使用磅房手动采集"));
            return;
        }
        await next(context);
    }
}
