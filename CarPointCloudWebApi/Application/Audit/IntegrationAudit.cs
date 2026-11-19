using System.Text.Json;
using Microsoft.AspNetCore.Http;

namespace CarPointCloudWebApi.Application;

/// <summary>
/// 三方交互审计的统一字段和关联规则。
/// </summary>
public static class IntegrationAudit
{
    /// <summary>在 <see cref="HttpContext.Items"/> 中保存交互链路编号所使用的键。</summary>
    public const string InteractionIdItem = "CarPointCloud.IntegrationInteractionId";

    private const string QueryDiagnosticsItem = "CarPointCloud.IntegrationQueryDiagnostics";

    /// <summary>
    /// 让控制器把"这次查询实际怎么执行的"留给访问日志中间件落库。
    /// </summary>
    /// <remarks>
    /// 走 <c>HttpContext.Items</c> 而不是让控制器自己写库，是为了让 ApiAccessLog 的
    /// 写入口径只留在中间件一处：控制器抛异常时不会留下一条"查询成功"的孤立记录。
    /// </remarks>
    public static void SetQueryDiagnostics(HttpContext? context, string description)
    {
        if (context is null || string.IsNullOrWhiteSpace(description))
        {
            return;
        }

        context.Items[QueryDiagnosticsItem] = description;
    }

    /// <summary>
    /// 读取控制器为当前请求记录的实际查询诊断信息。
    /// </summary>
    /// <param name="context">当前 HTTP 上下文；后台任务场景可为空。</param>
    /// <returns>分页模式、时间窗和命中数等诊断描述；未设置时返回 <see langword="null"/>。</returns>
    public static string? GetQueryDiagnostics(HttpContext? context)
    {
        if (context is null)
        {
            return null;
        }

        return context.Items.TryGetValue(QueryDiagnosticsItem, out var value)
            ? value as string
            : null;
    }

    /// <summary>
    /// 根据请求路径识别第三方交互业务类型。
    /// </summary>
    /// <param name="path">待分类的请求路径。</param>
    /// <returns>审计系统使用的固定业务类型；非集成接口返回 <see langword="null"/>。</returns>
    public static string? ResolveIntegrationType(PathString path)
    {
        if (path.StartsWithSegments("/api/v1/licensePlateRecords/fetch", StringComparison.OrdinalIgnoreCase) ||
            path.StartsWithSegments("/api/v1/radar/licensePlate", StringComparison.OrdinalIgnoreCase) ||
            path.StartsWithSegments("/api/v1/vehicleWorkflows", StringComparison.OrdinalIgnoreCase))
        {
            return "LicensePlate";
        }

        // 手动采集与第三方来车事件建立的是同一种业务对象（VehicleArrival 一条记录 + 一次雷达
        // 采集），因此归入同一审计类型，现场按 interactionId 排障时不需要区分两套口径。
        // 这里用精确匹配而不是 StartsWithSegments：captures/{id} 的进度查询在采集期间会被页面
        // 按秒轮询，纳入审计会把 ApiAccessLog 打满，而它本身不改变任何状态。
        if (path.StartsWithSegments("/api/v1/radar/vehicle-arrival", StringComparison.OrdinalIgnoreCase) ||
            path.Equals("/api/v1/radar-live/captures", StringComparison.OrdinalIgnoreCase))
        {
            return "VehicleArrival";
        }

        if (path.StartsWithSegments("/api/v1/vehicleDimensions", StringComparison.OrdinalIgnoreCase) ||
            path.StartsWithSegments("/api/v1/internal/pointcloud/vehicleDimensions", StringComparison.OrdinalIgnoreCase))
        {
            return "VehicleDimensions";
        }

        if (path.StartsWithSegments("/api/v1/coalVolumes", StringComparison.OrdinalIgnoreCase) ||
            path.StartsWithSegments("/api/v1/internal/pointcloud/coalVolumes", StringComparison.OrdinalIgnoreCase))
        {
            return "CoalVolume";
        }

        return null;
    }

    /// <summary>
    /// 获取调用方提供的安全交互编号，或为当前请求生成新的编号。
    /// </summary>
    /// <param name="context">当前 HTTP 请求上下文。</param>
    /// <returns>写入请求上下文和响应头的交互链路编号。</returns>
    /// <remarks>不符合长度或控制字符约束的外部编号会被忽略，防止污染日志与响应头。</remarks>
    public static string GetOrCreateInteractionId(HttpContext context)
    {
        var supplied = context.Request.Headers["X-Interaction-Id"].FirstOrDefault();
        var interactionId = IsSafeIdentifier(supplied, 64)
            ? supplied!
            : Guid.NewGuid().ToString("N");

        context.Items[InteractionIdItem] = interactionId;
        if (!context.Response.HasStarted)
        {
            context.Response.Headers["X-Interaction-Id"] = interactionId;
        }

        return interactionId;
    }

    /// <summary>
    /// 读取当前请求已经建立的交互链路编号。
    /// </summary>
    /// <param name="context">当前 HTTP 上下文。</param>
    /// <returns>上下文或请求头中的交互编号；不存在时返回 <see langword="null"/>。</returns>
    public static string? GetInteractionId(HttpContext? context)
    {
        if (context is null)
        {
            return null;
        }

        return context.Items.TryGetValue(InteractionIdItem, out var value)
            ? value as string
            : context.Request.Headers["X-Interaction-Id"].FirstOrDefault();
    }

    /// <summary>
    /// 从候选值中选择第一个非空字符串并去除首尾空白。
    /// </summary>
    /// <param name="values">按业务优先级排列的候选值。</param>
    /// <returns>首个有效值；全部为空时返回 <see langword="null"/>。</returns>
    public static string? FirstNonEmpty(params string?[] values)
    {
        return values.FirstOrDefault(value => !string.IsNullOrWhiteSpace(value))?.Trim();
    }

    /// <summary>
    /// 从 JSON 对象的顶层字段中提取字符串值。
    /// </summary>
    /// <param name="body">待解析的 JSON 正文。</param>
    /// <param name="names">允许匹配的字段名，比较时忽略大小写。</param>
    /// <returns>首个匹配的字符串；正文非法或字段不存在时返回 <see langword="null"/>。</returns>
    /// <remarks>解析失败会静默降级，原始业务接口仍负责返回其自身的格式错误。</remarks>
    public static string? JsonString(string? body, params string[] names)
    {
        if (string.IsNullOrWhiteSpace(body))
        {
            return null;
        }

        try
        {
            using var document = JsonDocument.Parse(body);
            if (document.RootElement.ValueKind != JsonValueKind.Object)
            {
                return null;
            }

            foreach (var property in document.RootElement.EnumerateObject())
            {
                if (names.Any(name => string.Equals(property.Name, name, StringComparison.OrdinalIgnoreCase)) &&
                    property.Value.ValueKind == JsonValueKind.String)
                {
                    return property.Value.GetString();
                }
            }
        }
        catch (JsonException)
        {
            // 请求体格式错误由控制器记录为 400；审计链路不应再抛异常。
        }

        return null;
    }

    /// <summary>
    /// 从 JSON 对象的顶层字段中提取整数值。
    /// </summary>
    /// <param name="body">待解析的 JSON 正文。</param>
    /// <param name="names">允许匹配的字段名，比较时忽略大小写。</param>
    /// <returns>数值或可转换字符串对应的整数；无法提取时返回 <see langword="null"/>。</returns>
    public static int? JsonInt(string? body, params string[] names)
    {
        if (string.IsNullOrWhiteSpace(body))
        {
            return null;
        }

        try
        {
            using var document = JsonDocument.Parse(body);
            if (document.RootElement.ValueKind != JsonValueKind.Object)
            {
                return null;
            }

            foreach (var property in document.RootElement.EnumerateObject())
            {
                if (!names.Any(name => string.Equals(property.Name, name, StringComparison.OrdinalIgnoreCase)))
                {
                    continue;
                }

                if (property.Value.ValueKind == JsonValueKind.Number && property.Value.TryGetInt32(out var number))
                {
                    return number;
                }

                if (property.Value.ValueKind == JsonValueKind.String &&
                    int.TryParse(property.Value.GetString(), out number))
                {
                    return number;
                }
            }
        }
        catch (JsonException)
        {
            // 非法响应正文仍按 HTTP 状态记录。
        }

        return null;
    }

    /// <summary>
    /// 递归查找 JSON 对象或数组中首次出现的指定字符串字段。
    /// </summary>
    /// <param name="body">待遍历的 JSON 正文。</param>
    /// <param name="name">目标字段名。</param>
    /// <returns>深度优先遍历发现的首个字符串值；正文非法或未命中时返回 <see langword="null"/>。</returns>
    public static string? JsonFirstString(string? body, string name)
    {
        if (string.IsNullOrWhiteSpace(body))
        {
            return null;
        }

        try
        {
            using var document = JsonDocument.Parse(body);
            return FindFirstString(document.RootElement, name);
        }
        catch (JsonException)
        {
            return null;
        }
    }

    private static string? FindFirstString(JsonElement element, string name)
    {
        if (element.ValueKind == JsonValueKind.Object)
        {
            foreach (var property in element.EnumerateObject())
            {
                if (string.Equals(property.Name, name, StringComparison.OrdinalIgnoreCase) &&
                    property.Value.ValueKind == JsonValueKind.String)
                {
                    return property.Value.GetString();
                }

                var nested = FindFirstString(property.Value, name);
                if (!string.IsNullOrWhiteSpace(nested))
                {
                    return nested;
                }
            }
        }
        else if (element.ValueKind == JsonValueKind.Array)
        {
            foreach (var item in element.EnumerateArray())
            {
                var nested = FindFirstString(item, name);
                if (!string.IsNullOrWhiteSpace(nested))
                {
                    return nested;
                }
            }
        }

        return null;
    }

    /// <summary>
    /// 确定审计日志中的调用方系统名称。
    /// </summary>
    /// <param name="context">当前 HTTP 上下文。</param>
    /// <param name="integrationType">已识别的集成业务类型。</param>
    /// <returns>安全的显式调用方名称，或按业务类型推导的默认系统名称。</returns>
    public static string? ResolveCallerSystem(HttpContext context, string integrationType)
    {
        var declared = context.Request.Headers["X-Caller-System"].FirstOrDefault();
        if (IsSafeIdentifier(declared, 64))
        {
            return declared;
        }

        return integrationType switch
        {
            "LicensePlate" => "Worker",
            "VehicleDimensions" => "Partner",
            "CoalVolume" => "Partner",
            "VehicleArrival" => "Partner",
            _ => "Unknown"
        };
    }

    /// <summary>
    /// 检查外部标识符能否安全写入日志和 HTTP 头。
    /// </summary>
    /// <param name="value">待检查的标识符。</param>
    /// <param name="maxLength">允许的最大字符数。</param>
    /// <returns>非空、未超长且不含控制字符时返回 <see langword="true"/>。</returns>
    public static bool IsSafeIdentifier(string? value, int maxLength)
    {
        return !string.IsNullOrWhiteSpace(value) &&
               value.Length <= maxLength &&
               value.All(ch => !char.IsControl(ch) && ch != '\r' && ch != '\n');
    }
}
