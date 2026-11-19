using System.Reflection;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.Mvc.Abstractions;
using Microsoft.AspNetCore.Mvc.Filters;
using Microsoft.AspNetCore.Routing;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class RadarLiveWriteAuthorizationTests
{
    [Theory]
    [InlineData(nameof(RadarLiveController.CreateCapture), "viewer", false)]
    [InlineData(nameof(RadarLiveController.CancelCapture), "viewer", false)]
    [InlineData(nameof(RadarLiveController.CreateCapture), "admin", true)]
    [InlineData(nameof(RadarLiveController.CancelCapture), "admin", true)]
    [InlineData(nameof(RadarLiveController.GetCapture), "viewer", true)]
    public async Task Actual_action_authorization_keeps_viewer_read_only(string actionName, string role, bool allowed)
    {
        var filter = typeof(RadarLiveController).GetMethod(actionName)!.GetCustomAttribute<RequireRoleAttribute>()!;
        var http = new DefaultHttpContext();
        http.Items[JwtAuthMiddleware.UserInfoKey] = new TokenUserInfo { Username = "isolated-test", RoleCode = role };
        var action = new ActionContext(http, new RouteData(), new ActionDescriptor());
        var context = new ActionExecutingContext(action, [], new Dictionary<string, object?>(), new object());
        var executed = false;
        await filter.OnActionExecutionAsync(context, () =>
        {
            executed = true;
            return Task.FromResult(new ActionExecutedContext(action, [], context.Controller));
        });
        Assert.Equal(allowed, executed);
        if (!allowed) Assert.Equal(403, Assert.IsType<ObjectResult>(context.Result).StatusCode);
    }
}
