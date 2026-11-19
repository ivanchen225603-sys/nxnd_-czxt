using Microsoft.AspNetCore.Mvc;

namespace CarPointCloudWebApi.Api;

/// <summary>
/// 日志查询控制器。
/// </summary>
/// <remarks>
/// 数据来源：ApiAccessLog 表（由 ApiAccessLogMiddleware 自动记录）。
/// 供前端日志查看页（logs.html）调用，返回结构对齐前端期望的 { id, time, level, source, message }。
/// 级别映射：HTTP 500+ → error；400-499 → warn；其余 → info。
/// </remarks>
[ApiController]
[Route("api/logs")]
public class LogController : ControllerBase
{
    /// <summary>分页页码上限，防止偏移量计算整数溢出。</summary>
    private const int MaxPage = 100_000;

    private readonly ILogRepository _logRepository;

    /// <summary>创建第三方交互审计日志查询控制器。</summary>
    /// <param name="logRepository">提供分页和链路维度查询的日志仓储。</param>
    public LogController(ILogRepository logRepository)
    {
        _logRepository = logRepository;
    }

    /// <summary>
    /// 分页查询 API 访问日志。
    /// </summary>
    /// <param name="keyword">关键字，匹配路径/来源/错误消息（模糊匹配）。</param>
    /// <param name="startTime">起始时间（yyyy-MM-ddTHH:mm，含）。</param>
    /// <param name="endTime">结束时间（yyyy-MM-ddTHH:mm，含）。</param>
    /// <param name="level">级别过滤：debug/info/warn/error，不传为全部。</param>
    /// <param name="source">来源过滤（按请求路径/方法模糊匹配），不传为全部。</param>
    /// <param name="interactionId">完整交互链路 ID，精确匹配。</param>
    /// <param name="workflowId">业务流程 ID，精确匹配。</param>
    /// <param name="licensePlate">车牌号，精确匹配。</param>
    /// <param name="integrationType">交互类型：LicensePlate、VehicleDimensions、CoalVolume、VehicleArrival。</param>
    /// <param name="page">页码，从 1 开始，默认 1。</param>
    /// <param name="pageSize">每页条数，默认 20，最大 100。</param>
    /// <returns>日志分页数据（items + total + page + pageSize）。</returns>
    [HttpGet]
    [RequireRole(Roles = "admin,viewer")]
    public ActionResult<ApiResponse<LogPageResult>> GetLogs(
        [FromQuery] string? keyword,
        [FromQuery] DateTime? startTime,
        [FromQuery] DateTime? endTime,
        [FromQuery] string? level,
        [FromQuery] string? source,
        [FromQuery] string? interactionId,
        [FromQuery] string? workflowId,
        [FromQuery] string? licensePlate,
        [FromQuery] string? integrationType,
        [FromQuery] int page = 1,
        [FromQuery] int pageSize = 20)
    {
        if (startTime.HasValue && endTime.HasValue && startTime > endTime)
        {
            return BadRequest(ApiResponse<LogPageResult>.Fail(400, "开始时间不能晚于结束时间"));
        }

        // 同时钳上限：仅 Math.Max(1, page) 时，page=int.MaxValue 会让 (page-1)*pageSize
        // 整数溢出为负，进而拼出 TOP -N 让数据库报错并返回 500。
        page = Math.Clamp(page, 1, MaxPage);
        pageSize = Math.Clamp(pageSize, 1, 100);

        var items = _logRepository.QueryLogs(
            keyword,
            startTime,
            endTime,
            level,
            source,
            interactionId,
            workflowId,
            licensePlate,
            integrationType,
            page,
            pageSize,
            out var total);

        return Ok(ApiResponse<LogPageResult>.Success(new LogPageResult
        {
            Items = items,
            Total = total,
            Page = page,
            PageSize = pageSize
        }));
    }

    /// <summary>
    /// 按交互 ID 返回完整的第三方请求链路，包括入站、出站、重试、响应和异常。
    /// </summary>
    [HttpGet("interaction/{interactionId}")]
    [RequireRole(Roles = "admin,viewer")]
    public ActionResult<ApiResponse<List<LogEntryDto>>> GetInteraction([FromRoute] string interactionId)
    {
        var items = _logRepository.QueryInteraction(interactionId);
        return Ok(ApiResponse<List<LogEntryDto>>.Success(items));
    }
}
