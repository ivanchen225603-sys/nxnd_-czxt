using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Api;

/// <summary>
/// 载货雷达采集端的机器认证：<c>/api/v1/load-radar-agent/*</c> 必须带 X-Load-Radar-Node 与对应节点的 X-Load-Radar-Key，
/// 通过后节点编号放入 <see cref="NodeIdKey"/>。
/// </summary>
public sealed class LoadRadarAgentAuthMiddleware(RequestDelegate next, IOptions<LoadRadarOptions> options)
{
    public const string NodeIdKey = "LoadRadarAgentNodeId";
    public const string NodeHeader = "X-Load-Radar-Node";
    public const string KeyHeader = "X-Load-Radar-Key";

    public static bool IsAuthenticatedAgent(HttpContext context) => context.Items.ContainsKey(NodeIdKey);

    public async Task InvokeAsync(HttpContext context)
    {
        if (!context.Request.Path.StartsWithSegments("/api/v1/load-radar-agent", StringComparison.OrdinalIgnoreCase))
        {
            await next(context);
            return;
        }
        if (!options.Value.Enabled)
        {
            await RejectAsync(context, StatusCodes.Status503ServiceUnavailable, "载货雷达接入未启用");
            return;
        }

        var nodeId = context.Request.Headers[NodeHeader].ToString();
        var node = options.Value.Nodes.FirstOrDefault(n => n.NodeId == nodeId);
        if (node is null || node.AgentKey != context.Request.Headers[KeyHeader].ToString())
        {
            await RejectAsync(context, StatusCodes.Status401Unauthorized, "机器认证失败");
            return;
        }
        context.Items[NodeIdKey] = node.NodeId;
        await next(context);
    }

    private static async Task RejectAsync(HttpContext context, int status, string message)
    {
        context.Response.StatusCode = status;
        await context.Response.WriteAsJsonAsync(ApiResponse<object>.Fail(status, message));
    }
}
