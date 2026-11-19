using System.Reflection;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.Mvc.Abstractions;
using Microsoft.AspNetCore.Mvc.Filters;
using Microsoft.AspNetCore.Routing;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using Microsoft.Extensions.Primitives;
using Xunit;

namespace CarPointCloudWebApi.Tests;

/// <summary>不连接数据库，覆盖机器凭据与操作员角色的隔离边界。</summary>
public sealed class LoadRadarAuthorizationTests
{
    private const string AgentPath = "/api/v1/load-radar-agent/measurements";
    private const string HumanPath = "/api/v1/load-radar/measurements";
    private const string AgentKey = "load-radar-test-node-secret-0123456789";

    [Theory]
    [InlineData(null, null)]
    [InlineData("node-1", null)]
    [InlineData(null, AgentKey)]
    [InlineData("unknown-node", AgentKey)]
    [InlineData("node-1", "wrong-key")]
    [InlineData("node-1", "load-radar-test-node-secret-0123456780")]
    [InlineData("node-1", AgentKey + " ")]
    public async Task Missing_or_invalid_machine_credentials_are_rejected(string? node, string? key)
    {
        var context = Context(AgentPath, node, key);
        var called = await InvokePipelineAsync(context);
        Assert.False(called);
        Assert.Equal(401, context.Response.StatusCode);
        Assert.False(LoadRadarAgentAuthMiddleware.IsAuthenticatedAgent(context));
        context.Response.Body.Position = 0;
        Assert.DoesNotContain(AgentKey, await new StreamReader(context.Response.Body).ReadToEndAsync());
    }

    [Fact]
    public async Task Disabled_module_rejects_even_valid_machine_credentials()
    {
        var context = Context(AgentPath, "node-1", AgentKey);
        Assert.False(await InvokePipelineAsync(context, enabled: false));
        Assert.Equal(503, context.Response.StatusCode);
    }

    [Theory]
    [InlineData(AgentPath)]
    [InlineData("/API/V1/LOAD-RADAR-AGENT/measurements")]
    [InlineData("/api/v1/load-radar-agent")]
    public async Task Valid_key_authenticates_only_a_machine_node(string path)
    {
        var context = Context(path, "node-1", AgentKey);
        Assert.True(await InvokePipelineAsync(context));
        Assert.Equal("node-1", context.Items[LoadRadarAgentAuthMiddleware.NodeIdKey]);
        Assert.True(LoadRadarAgentAuthMiddleware.IsAuthenticatedAgent(context));
        Assert.False(context.Items.ContainsKey(JwtAuthMiddleware.UserInfoKey));
        Assert.False(context.User.Identity?.IsAuthenticated ?? false);
    }

    [Theory]
    [InlineData(HumanPath)]
    [InlineData("/api/v1/load-radar-agent-extra/measurements")]
    [InlineData("/api/v1/load-radar-agents/measurements")]
    [InlineData("/api/v1/load-radar-agentx")]
    [InlineData("/api/v1/radar-live/status")]
    public async Task Machine_key_does_not_authorize_human_or_similar_prefix_paths(string path)
    {
        var context = Context(path, "node-1", AgentKey);
        Assert.False(await InvokePipelineAsync(context));
        Assert.Equal(401, context.Response.StatusCode);
        Assert.False(LoadRadarAgentAuthMiddleware.IsAuthenticatedAgent(context));
    }

    [Fact]
    public async Task Valid_admin_jwt_cannot_replace_machine_key()
    {
        var context = Context(AgentPath);
        using var provider = UserProvider("admin");
        context.RequestServices = provider;
        SetToken(context, "admin");
        Assert.False(await InvokePipelineAsync(context));
        Assert.Equal(401, context.Response.StatusCode);
    }

    [Fact]
    public async Task Machine_identity_has_no_operator_role()
    {
        var context = Context(AgentPath, "node-1", AgentKey);
        Assert.False(await InvokePipelineAsync(context, requiredRoles: "admin"));
        Assert.Equal(401, context.Response.StatusCode);
    }

    [Theory]
    [InlineData("admin", "admin,viewer", true, 200)]
    [InlineData("viewer", "admin,viewer", true, 200)]
    [InlineData("admin", "admin", true, 200)]
    [InlineData("viewer", "admin", false, 403)]
    [InlineData("operator", "admin,viewer", false, 403)]
    public async Task Human_api_preserves_jwt_and_role_requirements(string role, string allowed, bool expected, int status)
    {
        var context = Context(HumanPath, "node-1", AgentKey);
        using var provider = UserProvider(role);
        context.RequestServices = provider;
        SetToken(context, role);
        var called = await InvokePipelineAsync(context, requiredRoles: allowed);
        Assert.Equal(expected, called);
        Assert.Equal(status, context.Response.StatusCode);
        Assert.False(LoadRadarAgentAuthMiddleware.IsAuthenticatedAgent(context));
    }

    [Fact]
    public async Task Human_api_requires_login_even_for_read_role()
    {
        var context = Context(HumanPath);
        Assert.False(await InvokePipelineAsync(context, requiredRoles: "admin,viewer"));
        Assert.Equal(401, context.Response.StatusCode);
    }

    private static async Task<bool> InvokePipelineAsync(DefaultHttpContext context, bool enabled = true, string? requiredRoles = null)
    {
        var called = false;
        var jwt = Jwt(async current =>
        {
            if (requiredRoles is null) { called = true; return; }
            called = await InvokeRoleAsync(current, requiredRoles);
        });
        var options = Options.Create(new LoadRadarOptions
        {
            Enabled = enabled,
            Nodes = [new() { NodeId = "node-1", AgentKey = AgentKey }]
        });
        await new LoadRadarAgentAuthMiddleware(jwt.InvokeAsync, options).InvokeAsync(context);
        return called;
    }

    private static async Task<bool> InvokeRoleAsync(HttpContext context, string requiredRoles)
    {
        var called = false;
        var action = new ActionContext(context, new RouteData(), new ActionDescriptor());
        var executing = new ActionExecutingContext(action, new List<IFilterMetadata>(),
            new Dictionary<string, object?>(), new object());
        await new RequireRoleAttribute { Roles = requiredRoles }.OnActionExecutionAsync(executing, () =>
        {
            called = true;
            return Task.FromResult(new ActionExecutedContext(action, new List<IFilterMetadata>(), new object()));
        });
        if (executing.Result is ObjectResult result) context.Response.StatusCode = result.StatusCode ?? 200;
        return called;
    }

    private static JwtTokenService TokenService() => new(Options.Create(new JwtOptions
    {
        JwtSecret = "load-radar-human-test-jwt-secret-0123456789",
        TokenExpireMinutes = 60
    }));

    private static JwtAuthMiddleware Jwt(RequestDelegate next) =>
        new(next, TokenService());

    private static void SetToken(DefaultHttpContext context, string role)
    {
        var jwt = TokenService();
        context.Request.Headers.Authorization = "Bearer " + jwt.IssueToken(new TokenUserInfo
        {
            UserId = 7,
            Username = "operator",
            RoleCode = role
        }, DateTime.UtcNow);
    }

    private static ServiceProvider UserProvider(string role) => new ServiceCollection().BuildServiceProvider();

    private static DefaultHttpContext Context(string path, string? node = null, string? key = null)
    {
        var context = new DefaultHttpContext();
        context.Request.Path = path;
        context.Response.Body = new MemoryStream();
        if (node is not null) context.Request.Headers[LoadRadarAgentAuthMiddleware.NodeHeader] = node;
        if (key is not null) context.Request.Headers[LoadRadarAgentAuthMiddleware.KeyHeader] = key;
        return context;
    }
}
