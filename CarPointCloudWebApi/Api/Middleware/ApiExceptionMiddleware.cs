
namespace CarPointCloudWebApi.Api;

/// <summary>
/// 全局 API 异常处理中间件。
/// </summary>
/// <remarks>
/// 功能说明：捕获后续中间件和控制器未处理的异常，记录日志并返回统一 JSON 错误响应。
/// 业务关键点：对外不暴露内部异常堆栈，统一返回 code=500/message=服务器内部错误。
/// </remarks>
public class ApiExceptionMiddleware
{
    private readonly RequestDelegate _next;
    private readonly ILogger<ApiExceptionMiddleware> _logger;

    /// <summary>
    /// 初始化全局异常中间件。
    /// </summary>
    /// <param name="next">下一个中间件委托。</param>
    /// <param name="logger">日志组件。</param>
    public ApiExceptionMiddleware(RequestDelegate next, ILogger<ApiExceptionMiddleware> logger)
    {
        _next = next;
        _logger = logger;
    }

    /// <summary>
    /// 执行异常捕获逻辑。
    /// </summary>
    /// <remarks>
    /// 异常分类处理策略：
    /// - ArgumentException → 400 错误（客户端参数错误）
    /// - InvalidOperationException → 根据具体情况返回 400/502
    /// - HttpRequestException → 502 错误（上游服务错误）
    /// - 其他异常 → 500 错误（服务器内部错误）
    /// </remarks>
    /// <param name="context">当前 HTTP 上下文。</param>
    /// <returns>异步任务。</returns>
    public async Task InvokeAsync(HttpContext context)
    {
        try
        {
            await _next(context);
        }
        catch (OperationCanceledException) when (PartnerResponseFactory.IsPartnerQueryPath(context.Request.Path) &&
                                                 context.RequestAborted.IsCancellationRequested)
        {
            throw;
        }
        catch (Exception ex) when (PartnerResponseFactory.IsPartnerQueryPath(context.Request.Path))
        {
            _logger.LogError(ex, "合作方查询失败：{Path} trace={TraceId}", context.Request.Path, context.TraceIdentifier);
            var message = ex is TimeoutException or OperationCanceledException ? "查询超时" : "服务器内部错误";
            await WriteErrorResponseAsync(context, StatusCodes.Status500InternalServerError, 500, message);
        }
        catch (LoadRadarException ex)
        {
            await WriteErrorResponseAsync(context, ex.StatusCode, ex.StatusCode, ex.Message);
        }
        catch (ArgumentException ex)
        {
            _logger.LogWarning(ex, "Invalid argument: {Method} {Path} trace={TraceId} - {Message}",
                context.Request.Method, context.Request.Path, context.TraceIdentifier, ex.Message);
            await WriteErrorResponseAsync(context, StatusCodes.Status400BadRequest, 400, ex.Message);
        }
        catch (InvalidOperationException ex)
        {
            _logger.LogWarning(ex, "Invalid operation: {Method} {Path} trace={TraceId} - {Message}",
                context.Request.Method, context.Request.Path, context.TraceIdentifier, ex.Message);
            await WriteErrorResponseAsync(context, StatusCodes.Status400BadRequest, 400, ex.Message);
        }
        catch (HttpRequestException ex)
        {
            _logger.LogError(ex, "Upstream service error: {Method} {Path} trace={TraceId}",
                context.Request.Method, context.Request.Path, context.TraceIdentifier);
            await WriteErrorResponseAsync(context, StatusCodes.Status502BadGateway, 502, "上游服务错误");
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "API request failed: {Method} {Path} trace={TraceId}",
                context.Request.Method, context.Request.Path, context.TraceIdentifier);
            await WriteErrorResponseAsync(context, StatusCodes.Status500InternalServerError, 500, "服务器内部错误");
        }
    }

    /// <summary>
    /// 写入错误响应。
    /// </summary>
    /// <param name="context">HTTP 上下文。</param>
    /// <param name="statusCode">HTTP 状态码。</param>
    /// <param name="code">业务错误码。</param>
    /// <param name="message">错误消息。</param>
    private static async Task WriteErrorResponseAsync(HttpContext context, int statusCode, int code, string message)
    {
        context.Response.StatusCode = statusCode;
        context.Response.ContentType = "application/json; charset=utf-8";
        await context.Response.WriteAsJsonAsync(PartnerResponseFactory.Fail(context.Request.Path, code, message));
    }
}
