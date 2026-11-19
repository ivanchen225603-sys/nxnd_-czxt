using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using Xunit;

namespace CarPointCloudWebApi.Tests;

public sealed class InternalChainTests
{
    [Fact]
    public async Task Stored_review_returns_dimensions_and_archive_even_when_worker_is_offline()
    {
        var repository = new CompletedRepository("low_confidence");
        var configuration = new ConfigurationBuilder().AddInMemoryCollection(new Dictionary<string, string?>
        { ["Database:Enabled"] = "true" }).Build();
        var controller = new RadarLiveController(
            new RadarLiveClient(new OfflineWorkerFactory()),
            repository, new ProfileRepository("workflow-a"), new IntegrationQueryService(repository, configuration),
            configuration, NullLogger<RadarLiveController>.Instance);
        var result = await controller.GetCapture("workflow-a", CancellationToken.None);
        var response = Assert.IsType<ApiResponse<CaptureProgressResponse>>(Assert.IsType<OkObjectResult>(result.Result).Value);
        Assert.Equal("completed", response.Data!.Stage);
        Assert.True(response.Data.Terminal);
        Assert.Equal("pending", response.Data.Dimensions!.ReviewStatus);
        Assert.Equal(290m, Assert.IsType<VehicleDimensionsDto>(response.Data.Dimensions.Dimensions).TrailerBodyHeight);
        Assert.Contains("待人工审核", response.Data.Message);
        Assert.NotNull(response.Data.EmptyVehiclePointCloudUrl);
    }
    [Theory]
    [InlineData("workflow-a", true)]
    [InlineData("workflow-newer", false)]
    [InlineData(null, false)]
    public async Task Completed_capture_only_links_its_own_archive(string? archiveWorkflow, bool linked)
    {
        var repository = new CompletedRepository();
        var configuration = new ConfigurationBuilder().AddInMemoryCollection(new Dictionary<string, string?>
        { ["Database:Enabled"] = "true" }).Build();
        var controller = new RadarLiveController(
            new RadarLiveClient(new OfflineWorkerFactory()),
            repository, new ProfileRepository(archiveWorkflow), new IntegrationQueryService(repository, configuration),
            configuration, NullLogger<RadarLiveController>.Instance);
        var result = await controller.GetCapture("workflow-a", CancellationToken.None);
        var response = Assert.IsType<ApiResponse<CaptureProgressResponse>>(Assert.IsType<OkObjectResult>(result.Result).Value);
        Assert.Equal("completed", response.Data!.Stage);
        Assert.Equal(linked, response.Data.EmptyVehiclePointCloudUrl is not null);
        if (linked) Assert.EndsWith("?expectedWorkflowId=workflow-a", response.Data.EmptyVehiclePointCloudUrl);
    }

    private sealed class CompletedRepository(string status = "success") : NullIntegrationRepository, IIntegrationRepository
    {
        VehicleArrivalRecord? IIntegrationRepository.GetVehicleArrivalBySessionId(string sessionId) => new()
        { SessionId = sessionId, LicensePlate = "川A12047", PlateStatus = "SUCCEEDED", RadarStatus = "SUCCEEDED" };
        public VehicleDimensionRecord? GetVehicleDimensionByWorkflowId(string workflowId) => new()
        { WorkflowId = workflowId, LicensePlate = "川A12047", DetectStatus = status, TrailerLength = 595.8m, TrailerBodyHeight = 290m };
    }

    private sealed class ProfileRepository(string? workflow) : IEmptyVehicleProfileRepository
    {
        public EmptyVehicleProfileRecord? GetByLicensePlate(string plate)
        {
            if (workflow == "unavailable") throw new IOException("test archive unavailable");
            return workflow is null ? null : new() { LicensePlate = plate, WorkflowId = workflow };
        }
    }

    private sealed class OfflineWorkerFactory : IHttpClientFactory
    {
        public HttpClient CreateClient(string name) => new(new OfflineWorkerHandler()) { BaseAddress = new Uri("http://127.0.0.1:8000") };
    }

    private sealed class OfflineWorkerHandler : HttpMessageHandler
    {
        protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken) =>
            Task.FromResult(new HttpResponseMessage(System.Net.HttpStatusCode.ServiceUnavailable));
    }

    [Theory]
    [InlineData("/api/v1/radar/vehicle-arrival", false)]
    [InlineData("/api/v1/licensePlateRecords/fetch", false)]
    // Worker 回传车厢尺寸的入口属自有链路，自有链路模式下必须保持可达：
    // 它是 Analyzer 直写 DM8 失效时唯一的入库退路，拦掉会让两条入库路径同时不通。
    [InlineData("/api/v1/internal/pointcloud/vehicleDimensions", true)]
    // 煤量由第三方推送，属合作方链路。
    [InlineData("/api/v1/internal/pointcloud/coalVolumes", false)]
    [InlineData("/api/v1/vehicleDimensions", false)]
    [InlineData("/api/v1/radar-live/status", true)]
    [InlineData("/api/v1/radar-live/fused-frame", true)]
    [InlineData("/api/v1/radar-live/captures", true)]
    [InlineData("/api/v1/vehicleWorkflows/workflow/licensePlate", true)]
    [InlineData("/api/empty-vehicle-profiles/TEST/pointcloud", true)]
    public async Task Internal_chain_never_dispatches_external_endpoints(string path, bool expected)
    {
        var called = false;
        var context = new DefaultHttpContext();
        context.Request.Path = path;
        context.Response.Body = new MemoryStream();
        await new ExternalPartnerGateMiddleware(_ => { called = true; return Task.CompletedTask; },
            Options.Create(new IntegrationOptions { ExternalPartnerEnabled = false })).InvokeAsync(context);
        Assert.Equal(expected, called);
        Assert.Equal(expected ? 200 : 503, context.Response.StatusCode);
    }
}
