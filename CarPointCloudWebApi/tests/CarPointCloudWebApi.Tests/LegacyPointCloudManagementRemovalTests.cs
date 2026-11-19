using System.Reflection;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.Mvc.Routing;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class LegacyPointCloudManagementRemovalTests
{
    private static readonly Assembly ApplicationAssembly = typeof(Program).Assembly;

    [Fact]
    public void LegacyPointCloudManagementTypes_AreRemoved()
    {
        var removedTypes = new[]
        {
            "CarPointCloudWebApi.Controllers.PointCloudController",
            "CarPointCloudWebApi.Repositories.IPointCloudRepository",
            "CarPointCloudWebApi.Repositories.PointCloudRepository",
            "CarPointCloudWebApi.Models.PointCloudFileRecord",
            "CarPointCloudWebApi.Models.PointCloudFileDto",
            "CarPointCloudWebApi.Models.UploadPointCloudResponse",
            "CarPointCloudWebApi.Services.PcdHeaderParser"
        };

        Assert.All(removedTypes, typeName => Assert.Null(ApplicationAssembly.GetType(typeName)));
    }

    [Fact]
    public void LegacyPointCloudRoute_IsNotExposed()
    {
        var controllerRoutes = ApplicationAssembly
            .GetTypes()
            .Where(type => typeof(ControllerBase).IsAssignableFrom(type))
            .SelectMany(type => type.GetCustomAttributes<RouteAttribute>())
            .Select(attribute => attribute.Template);

        Assert.DoesNotContain(
            controllerRoutes,
            template => string.Equals(template, "api/pointclouds", StringComparison.OrdinalIgnoreCase));
    }

    [Fact]
    public void EmptyVehicleProfileController_RemainsReadOnly()
    {
        var controllerType = ApplicationAssembly.GetType(
            "CarPointCloudWebApi.Api.EmptyVehicleProfileController");
        Assert.NotNull(controllerType);

        var actions = controllerType!
            .GetMethods(BindingFlags.Instance | BindingFlags.Public | BindingFlags.DeclaredOnly)
            .SelectMany(method => method.GetCustomAttributes<HttpMethodAttribute>())
            .ToArray();

        Assert.Equal(2, actions.Length);
        Assert.All(actions, action => Assert.Equal(new[] { "GET" }, action.HttpMethods));
        Assert.Contains(actions, action => action.Template == "{licensePlate}");
        Assert.Contains(actions, action => action.Template == "{licensePlate}/pointcloud");
    }
}
