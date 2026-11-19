using System.Collections.Concurrent;
using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Api;

/// <summary>
/// 合同 v4 规定的查询限流：同一来源 IP 每秒最多 min(RateLimitPerSecond, 3) 次；
/// 超限时煤量查询返回 400，尺寸查询返回 429。
/// </summary>
public class PartnerRateLimitMiddleware(RequestDelegate next, IOptions<IntegrationOptions> options)
{
    private static readonly ConcurrentDictionary<string, Queue<long>> Windows = new();

    public async Task InvokeAsync(HttpContext context)
    {
        var path = (context.Request.Path.Value ?? string.Empty).TrimEnd('/').ToLowerInvariant();
        if (path is not ("/api/v1/vehicledimensions" or "/api/v1/coalvolumes"))
        {
            await next(context);
            return;
        }

        var limit = Math.Min(options.Value.RateLimitPerSecond, 3);
        var now = Environment.TickCount64;
        var accepted = Windows.GetOrAdd($"{context.Connection.RemoteIpAddress}:{path}", _ => new Queue<long>());
        bool allowed;
        lock (accepted)
        {
            while (accepted.Count > 0 && accepted.Peek() <= now - 1000)
                accepted.Dequeue();
            allowed = accepted.Count < limit;
            if (allowed)
                accepted.Enqueue(now);
        }
        if (!allowed)
        {
            var status = path == "/api/v1/coalvolumes" ? StatusCodes.Status400BadRequest : StatusCodes.Status429TooManyRequests;
            context.Response.StatusCode = status;
            await context.Response.WriteAsJsonAsync(PartnerResponseFactory.Fail(context.Request.Path, status, "请求过于频繁"));
            return;
        }
        await next(context);
    }
}
