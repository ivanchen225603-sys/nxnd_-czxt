using Microsoft.AspNetCore.Http;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class IntegrationAuditTests
{
    [Theory]
    [InlineData("/api/v1/licensePlateRecords/fetch", "LicensePlate")]
    [InlineData("/api/v1/vehicleWorkflows/wf-001/licensePlate", "LicensePlate")]
    [InlineData("/api/v1/vehicleDimensions", "VehicleDimensions")]
    [InlineData("/api/v1/internal/pointcloud/vehicleDimensions", "VehicleDimensions")]
    [InlineData("/api/v1/coalVolumes", "CoalVolume")]
    [InlineData("/api/v1/internal/pointcloud/coalVolumes", "CoalVolume")]
    [InlineData("/api/v1/radar/vehicle-arrival", "VehicleArrival")]
    public void ResolveIntegrationType_MapsSupportedEndpoints(string path, string expected)
    {
        Assert.Equal(expected, IntegrationAudit.ResolveIntegrationType(new PathString(path)));
    }

    [Fact]
    public void GetOrCreateInteractionId_ReusesSafeHeaderAndSetsResponseHeader()
    {
        var context = new DefaultHttpContext();
        context.Request.Headers["X-Interaction-Id"] = "interaction-001";

        var actual = IntegrationAudit.GetOrCreateInteractionId(context);

        Assert.Equal("interaction-001", actual);
        Assert.Equal("interaction-001", context.Response.Headers["X-Interaction-Id"].ToString());
        Assert.Equal(actual, IntegrationAudit.GetInteractionId(context));
    }

    [Fact]
    public void GetOrCreateInteractionId_RejectsHeaderWithNewline()
    {
        var context = new DefaultHttpContext();
        context.Request.Headers["X-Interaction-Id"] = "bad\ninteraction";

        var actual = IntegrationAudit.GetOrCreateInteractionId(context);

        Assert.NotEqual("bad\ninteraction", actual);
        Assert.Matches("^[0-9a-f]{32}$", actual);
    }

    [Fact]
    public void JsonString_ReadsCorrelationFieldCaseInsensitivelyAndIgnoresInvalidJson()
    {
        Assert.Equal("wf-001", IntegrationAudit.JsonString("{\"WorkflowId\":\"wf-001\"}", "workflowId"));
        Assert.Null(IntegrationAudit.JsonString("not-json", "workflowId"));
    }
}
