namespace CarPointCloudWebApi.Api;

/// <summary>合同查询的三字段包裹；内部 ApiResponse 的 success 兼容字段不对外发送。</summary>
public sealed record PartnerApiResponse<T>(int Code, string Message, T? Data)
{
    public static PartnerApiResponse<T> Success(T data) => new(200, "success", data);
    public static PartnerApiResponse<T> Fail(int code, string message) => new(code, message, default);
}

/// <summary>公共错误路径按查询契约投影，内部入口保留原响应。</summary>
public static class PartnerResponseFactory
{
    /// <summary>合同规定的两个查询资源，大小写和尾斜线与路由匹配保持一致。</summary>
    public static bool IsPartnerQuery(PathString path)
    {
        var route = (path.Value ?? string.Empty).TrimEnd('/');
        return route.Equals("/api/v1/coalVolumes", StringComparison.OrdinalIgnoreCase) ||
               route.Equals("/api/v1/vehicleDimensions", StringComparison.OrdinalIgnoreCase);
    }

    /// <summary>含合作方查询的不存在子路径，保证这些路径上的错误格式一致。</summary>
    public static bool IsPartnerQueryPath(PathString path) =>
        path.StartsWithSegments("/api/v1/coalVolumes", StringComparison.OrdinalIgnoreCase) ||
        path.StartsWithSegments("/api/v1/vehicleDimensions", StringComparison.OrdinalIgnoreCase);

    public static object Fail(PathString path, int code, string message) => IsPartnerQueryPath(path)
        ? PartnerApiResponse<object>.Fail(code, message)
        : ApiResponse<object>.Fail(code, message);
}
