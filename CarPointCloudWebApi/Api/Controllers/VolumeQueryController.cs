using Microsoft.AspNetCore.Mvc;

namespace CarPointCloudWebApi.Api;

/// <summary>后台正式体积记录查询，不触发采集、计算或第三方调用。</summary>
[ApiController]
[Route("api/volume-query")]
[RequireRole(Roles = "admin,viewer")]
public sealed class VolumeQueryController(IIntegrationRepository repository) : ControllerBase
{
    [HttpGet]
    public async Task<IActionResult> Get([FromQuery] DateOnly? startDate = null,
        [FromQuery] DateOnly? endDate = null, [FromQuery] string? licensePlate = null,
        [FromQuery] string? channelId = null, [FromQuery] int page = 1,
        [FromQuery] int pageSize = 10, CancellationToken cancellationToken = default)
    {
        var today = DateOnly.FromDateTime(BusinessClock.Now);
        var start = startDate ?? today;
        var end = endDate ?? today;
        var plate = licensePlate?.Trim().ToUpperInvariant();
        var channel = channelId?.Trim().ToUpperInvariant();
        if (start > end || end == DateOnly.MaxValue)
            return BadRequest(ApiResponse<object>.Fail(400, "请选择有效的起止日期，开始日期不能晚于结束日期"));
        if (plate?.Length > 64)
            return BadRequest(ApiResponse<object>.Fail(400, "车牌号不能超过64个字符"));
        if (!string.IsNullOrEmpty(channel) && channel is not ("CH1" or "CH2" or "CH3" or "CH4"))
            return BadRequest(ApiResponse<object>.Fail(400, "通道只能为全部或CH1至CH4"));
        if (page is < 1 or > 100_000 || pageSize is < 1 or > 100)
            return BadRequest(ApiResponse<object>.Fail(400, "页码范围为1至100000，每页条数范围为1至100"));

        var query = new VolumeQuery(start.ToDateTime(TimeOnly.MinValue),
            end.AddDays(1).ToDateTime(TimeOnly.MinValue),
            string.IsNullOrEmpty(plate) ? null : plate,
            string.IsNullOrEmpty(channel) ? null : "CAM_00" + channel[2], page, pageSize);
        return Ok(ApiResponse<VolumeQueryPage>.Success(await repository.QueryVolumesAsync(query, cancellationToken)));
    }
}
