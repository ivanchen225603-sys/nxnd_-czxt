using Microsoft.AspNetCore.Mvc;

namespace CarPointCloudWebApi.Api;

/// <summary>人工命令只接受既有测量标识，节点、通道和过车均从不可变记录解析。</summary>
[ApiController]
[Route("api/v1/load-radar/measurements/{id}/commands")]
[RequireRole(Roles = "admin")]
public sealed class LoadRadarCommandController(LoadRadarService service, LoadRadarCommandClient commands) : ControllerBase
{
    [HttpPost]
    public async Task<IActionResult> Submit(string id, LoadRadarCommandRequest request, CancellationToken cancellationToken) =>
        Ok(ApiResponse<object>.Success(await commands.Submit(service.Get(id), request, cancellationToken)));

    [HttpGet("{requestId}")]
    public async Task<IActionResult> Status(string id, string requestId, CancellationToken cancellationToken) =>
        Ok(ApiResponse<object>.Success(await commands.Status(service.Get(id), requestId, cancellationToken)));
}
