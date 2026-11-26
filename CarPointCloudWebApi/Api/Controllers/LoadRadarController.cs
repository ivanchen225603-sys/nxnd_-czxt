using Microsoft.AspNetCore.Mvc;

namespace CarPointCloudWebApi.Api;

/// <summary>统一后台入口；写操作仅管理员，机器密钥不能访问这里。</summary>
[ApiController]
[Route("api/v1/load-radar")]
[RequireRole(Roles = "admin,viewer")]
public sealed class LoadRadarController(LoadRadarService service, LoadRadarLiveClient live,
    LoadRadarAssetStore assets) : ControllerBase
{
    private string Actor => ((TokenUserInfo)HttpContext.Items[JwtAuthMiddleware.UserInfoKey]!).Username;

    [HttpGet("state")]
    public async Task<IActionResult> State() => Ok(ApiResponse<object>.Success(new
    {
        service.Options.Enabled,
        Lanes = await live.State()
    }));

    [HttpGet("history")]
    public IActionResult History([FromQuery] string? nodeId, [FromQuery] string? channelId,
        [FromQuery] string? licensePlate, [FromQuery] string? beforeId, [FromQuery] int take = 50,
        [FromQuery] DateTimeOffset? from = null, [FromQuery] DateTimeOffset? to = null) =>
        Ok(ApiResponse<object>.Success(service.History(nodeId, channelId, licensePlate, beforeId, take, from, to)));

    [HttpGet("measurements/{id}")]
    public IActionResult Measurement(string id) => Ok(ApiResponse<object>.Success(service.GetForDisplay(id)));

    [HttpGet("measurements/{id}/revisions")]
    public IActionResult Revisions(string id) => Ok(ApiResponse<object>.Success(service.Revisions(id)));

    [HttpPost("measurements/{id}/binding")]
    [RequireRole(Roles = "admin")]
    public IActionResult Bind(string id, LoadBindRequest request) =>
        Ok(ApiResponse<object>.Success(service.Bind(id, request, Actor)));

    [HttpGet("baselines")]
    public IActionResult Baselines() => Ok(ApiResponse<object>.Success(service.Baselines()));

    [HttpPost("baselines")]
    [RequireRole(Roles = "admin")]
    public IActionResult CreateBaseline(LoadBaselineCreateRequest request) =>
        Ok(ApiResponse<object>.Success(service.CreateBaseline(request, Actor)));

    [HttpPost("baselines/{id}/activate")]
    [RequireRole(Roles = "admin")]
    public IActionResult Activate(string id, LoadBaselineActivateRequest request) =>
        Ok(ApiResponse<object>.Success(service.Activate(id, request, Actor)));

    [HttpGet("baselines/{id}/asset")]
    public IActionResult Download(string id)
    {
        var baseline = service.Baseline(id);
        return File(assets.Open(baseline.AssetSha256), "application/octet-stream", id + ".npz");
    }

    [HttpPost("measurements/{id}/publish")]
    [RequireRole(Roles = "admin")]
    public IActionResult Publish(string id, LoadPublishRequest request) =>
        Ok(ApiResponse<object>.Success(service.Publish(id, request, Actor)));
}
