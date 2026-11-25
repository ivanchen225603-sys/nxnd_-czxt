using System.Collections.Concurrent;
using System.Text.Json;
using System.Text.RegularExpressions;
using Microsoft.AspNetCore.Mvc;

namespace CarPointCloudWebApi.Api;

/// <summary>
/// 第三方数据交互控制器。
/// </summary>
/// <remarks>
/// 职责：对外提供车辆尺寸、煤炭体积、车牌拉取接口，接收第三方推送的煤炭体积，
/// 并保留一条当前未启用的点云尺寸入库通道（详见各端点 remarks）。
/// 异常策略：参数错误返回 400；乙方接口异常返回 502；未捕获异常由全局异常中间件处理。
/// </remarks>
[ApiController]
[Route("api/v1")]
public class IntegrationController : ControllerBase
{
    private static readonly Regex IdempotencyKeyRegex = new("^[A-Za-z0-9_\\-:.]{1,64}$", RegexOptions.Compiled);
    private readonly IntegrationQueryService _queryService;
    private readonly PartnerLicensePlateClient _licensePlateClient;
    private readonly IRadarLicensePlateSource _plateSource;
    private readonly IConfiguration _configuration;

    /// <summary>
    /// 初始化第三方数据交互控制器。
    /// </summary>
    /// <param name="queryService">车辆尺寸、煤炭体积查询和点云入库服务。</param>
    /// <param name="licensePlateClient">乙方车牌接口客户端。</param>
    /// <param name="plateSource">
    /// <c>/api/v1/radar/licensePlate</c> 的取车牌来源（单例，与载货雷达到车登记共用，
    /// 可见性开关 Radar:LicensePlateMockEnabled 也在它里面）。
    /// </param>
    /// <param name="configuration">运行开关。</param>
    public IntegrationController(
        IntegrationQueryService queryService,
        PartnerLicensePlateClient licensePlateClient,
        IRadarLicensePlateSource plateSource,
        IConfiguration configuration)
    {
        _queryService = queryService;
        _licensePlateClient = licensePlateClient;
        _plateSource = plateSource;
        _configuration = configuration;
    }

    /// <summary>
    /// 获取车辆尺寸数据。
    /// </summary>
    /// <param name="limit">按最新条数查询，范围 1-100；越界采用 100。与 timeWindow 必须且只能传一项。</param>
    /// <param name="timeWindow">按最近时间窗口查询，单位分钟，范围 1-120；与 limit 互斥。</param>
    /// <param name="cancellationToken">客户端断开时取消查询。</param>
    /// <returns>标准响应，data 为车辆尺寸列表。</returns>
    /// <exception cref="ArgumentException">查询参数不符合互斥规则时由服务层抛出。</exception>
    [HttpGet("vehicleDimensions")]
    public async Task<ActionResult<PartnerApiResponse<List<PartnerVehicleDimensionDto>>>> GetVehicleDimensions(
        [FromQuery] int? limit,
        [FromQuery] int? timeWindow,
        CancellationToken cancellationToken = default)
    {
        try
        {
            ValidatePartnerQuery(Request);
        }
        catch (ArgumentException ex)
        {
            return BadRequest(PartnerApiResponse<object>.Fail(400, ex.Message));
        }
        var page = await _queryService.GetPartnerVehiclesAsync(limit, timeWindow, cancellationToken);
        RecordQueryDiagnostics("vehicleDimensions", page.Diagnostics);
        return Ok(PartnerApiResponse<List<PartnerVehicleDimensionDto>>.Success(
            page.Items.Select(PartnerQueryMapper.Vehicle).ToList()));
    }

    /// <summary>按 workflowId 获取本次来车唯一绑定的车牌；Worker 每个流程只调用一次。</summary>
    [HttpGet("vehicleWorkflows/{workflowId}/licensePlate")]
    public ActionResult<ApiResponse<VehicleWorkflowPlateResponse>> GetWorkflowLicensePlate(
        [FromRoute] string workflowId)
    {
        try
        {
            return Ok(ApiResponse<VehicleWorkflowPlateResponse>.Success(
                _queryService.GetVehicleWorkflowPlate(workflowId)));
        }
        catch (KeyNotFoundException ex)
        {
            return NotFound(ApiResponse<VehicleWorkflowPlateResponse>.Fail(404, ex.Message));
        }
        catch (InvalidOperationException ex)
        {
            return Conflict(ApiResponse<VehicleWorkflowPlateResponse>.Fail(409, ex.Message));
        }
    }

    /// <summary>
    /// 获取煤炭体积数据。
    /// </summary>
    /// <param name="limit">按最新条数查询，范围 1-100；越界采用 100。与 timeWindow 必须且只能传一项。</param>
    /// <param name="timeWindow">按最近时间窗口查询，单位分钟，范围 1-120；与 limit 互斥。</param>
    /// <param name="cancellationToken">客户端断开时取消查询。</param>
    /// <returns>标准响应，data 为煤炭体积列表。</returns>
    [HttpGet("coalVolumes")]
    public async Task<ActionResult<PartnerApiResponse<List<PartnerCoalVolumeDto>>>> GetCoalVolumes(
        [FromQuery] int? limit,
        [FromQuery] int? timeWindow,
        CancellationToken cancellationToken = default)
    {
        try
        {
            ValidatePartnerQuery(Request);
        }
        catch (ArgumentException ex)
        {
            return BadRequest(PartnerApiResponse<object>.Fail(400, ex.Message));
        }
        var page = await _queryService.GetPartnerCoalsAsync(limit, timeWindow, cancellationToken);
        RecordQueryDiagnostics("coalVolumes", page.Diagnostics);
        return Ok(PartnerApiResponse<List<PartnerCoalVolumeDto>>.Success(
            page.Items.Select(PartnerQueryMapper.Coal).ToList()));
    }

    /// <summary>
    /// 主动调用乙方系统获取指定点位最新车牌号。
    /// </summary>
    /// <param name="cameraIdentifier">摄像头/雷达点位编码，仅允许字母、数字、下划线，最大 32 位。</param>
    /// <param name="cancellationToken">请求取消令牌。</param>
    /// <returns>乙方返回的车牌识别数据。</returns>
    /// <exception cref="ArgumentException">cameraIdentifier 缺失或格式非法。</exception>
    /// <exception cref="InvalidOperationException">乙方接口未配置、超时、返回失败或 cameraId 不一致。</exception>
    [HttpPost("licensePlateRecords/fetch")]
    public async Task<ActionResult<ApiResponse<List<PartnerLicensePlateData>>>> FetchLicensePlate([FromQuery] string cameraIdentifier, CancellationToken cancellationToken)
    {
        try
        {
            var data = await _licensePlateClient.GetLicensePlateAsync(cameraIdentifier, cancellationToken);
            return Ok(ApiResponse<List<PartnerLicensePlateData>>.Success(data));
        }
        catch (ArgumentException ex)
        {
            return BadRequest(ApiResponse<object>.Fail(400, ex.Message));
        }
        catch (InvalidOperationException)
        {
            return StatusCode(StatusCodes.Status502BadGateway, ApiResponse<object>.Fail(502, "乙方车牌服务暂时不可用"));
        }
        catch (HttpRequestException)
        {
            return StatusCode(StatusCodes.Status502BadGateway, ApiResponse<object>.Fail(502, "乙方车牌服务暂时不可用"));
        }
        catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
        {
            return StatusCode(StatusCodes.Status502BadGateway, ApiResponse<object>.Fail(502, "乙方车牌服务调用超时"));
        }
        catch (JsonException)
        {
            return StatusCode(StatusCodes.Status502BadGateway, ApiResponse<object>.Fail(502, "乙方车牌服务响应格式错误"));
        }
    }

    /// <summary>
    /// 雷达调用获取车牌号接口。
    /// </summary>
    /// <remarks>
    /// JT128 载货雷达按点位拉取当前车牌（CH1~CH4 对应 CAM_001~CAM_004）。
    ///
    /// <para>
    /// ★★ <b>当前实现返回的是模拟车牌，不是真实识别结果</b>，响应里 <c>source</c> 为 <c>MOCK</c>。
    /// 车牌真假由本接口（<see cref="IRadarLicensePlateSource"/>）负责，调用方拿到什么用什么：
    /// 载货雷达正式模式的到车登记与本接口取自同一来源，不再另判真假（2026-09-28 现场定）。
    /// 接入真实识别源只改 <see cref="IRadarLicensePlateSource"/> 的实现，调用方无需改协议。
    /// </para>
    ///
    /// <para>
    /// 可见性由 <c>Radar:LicensePlateMockEnabled</c> 控制（默认 <c>false</c>）。
    /// 以前写死成"仅 Development 可见"，导致麒麟生产环境一律 404，
    /// 而联调又确实需要它——所以改成显式开关：
    /// 生产要用就必须在 appsettings.json 里写明白，不会因为环境名不同而悄悄变化。
    /// 开发环境保持原行为（无需配置即可见），避免打断现有用例。
    /// </para>
    ///
    /// <para>
    /// 另外两道门不在本方法里，排查时别只看这里：
    /// <c>RadarDeviceAuthorizationMiddleware</c> 按 <c>Radar:AllowedDeviceAddresses</c>
    /// 校验实际 TCP 对端（不是雷达自身 IP，是调用方进程所在主机的 IP）；
    /// <c>ExternalPartnerGateMiddleware</c> 在合作方关闭时拦 <c>/api/v1/radar</c>，
    /// 但本端点已列入例外（自有链路的拉取口，不是合作方推送口）。
    /// </para>
    /// </remarks>
    /// <param name="cameraIdentifier">雷达点位编码，仅允许字母、数字、下划线，最大 32 位。</param>
    /// <param name="cancellationToken">请求取消令牌。</param>
    /// <returns>车牌号信息。</returns>
    [HttpGet("radar/licensePlate")]
    public async Task<ActionResult<ApiResponse<object>>> GetLicensePlateForRadar([FromQuery] string cameraIdentifier, CancellationToken cancellationToken)
    {
        if (!_plateSource.Enabled)
        {
            return NotFound(ApiResponse<object>.Fail(404, "接口不存在"));
        }

        if (string.IsNullOrWhiteSpace(cameraIdentifier) || cameraIdentifier.Length > 32 || !Regex.IsMatch(cameraIdentifier, "^[A-Za-z0-9_]+$"))
        {
            return BadRequest(ApiResponse<object>.Fail(400, "cameraIdentifier 格式非法，要求 1-32 位字母、数字或下划线"));
        }

        // 模拟异步延迟
        await Task.Delay(100, cancellationToken);
        var plate = _plateSource.Resolve(cameraIdentifier);
        if (plate is null)
        {
            return NotFound(ApiResponse<object>.Fail(404, "接口不存在"));
        }

        return Ok(ApiResponse<object>.Success(new
        {
            cameraId = plate.CameraId,
            licensePlate = plate.LicensePlate,
            captureTime = plate.CaptureTime.ToString("yyyy-MM-dd HH:mm:ss"),
            source = plate.Source
        }));
    }

    /// <summary>
    /// 接收 Python 点云解析模块上报的车辆尺寸数据。
    /// </summary>
    /// <remarks>
    /// 当前部署下这条路不通电：Analyzer 分析完直接写 DM8，Worker 的
    /// EnableResultUploadToWebApi=false，从不调用本端点。保留它是为了在 DM8 直写
    /// 出问题时把该开关翻回 true 应急，以及给其它可能用 HTTP 入库的客户端使用。
    /// </remarks>
    /// <param name="request">点云解析后的尺寸、筋条、钢丝绳和追溯路径信息。</param>
    /// <returns>入库成功后的记录 ID；数据库关闭时返回 0。</returns>
    /// <exception cref="ArgumentException">cameraIdentifier 为空或请求体不合法。</exception>
    [HttpPost("internal/pointcloud/vehicleDimensions")]
    public async Task<ActionResult<ApiResponse<object>>> IngestPointCloudDimensions(
        [FromBody] PointCloudDimensionIngestRequest? request,
        CancellationToken cancellationToken)
    {
        if (request == null)
        {
            return BadRequest(ApiResponse<object>.Fail(400, "请求体不能为空"));
        }

        try
        {
            request.IdempotencyKey = ResolveIdempotencyKey(request.IdempotencyKey);
            var id = await _queryService.IngestPointCloudDimensionsAsync(request, cancellationToken);
            return Ok(ApiResponse<object>.Success(new { id }));
        }
        catch (ArgumentException ex)
        {
            return BadRequest(ApiResponse<object>.Fail(400, ex.Message));
        }
        catch (IdempotencyConflictException ex)
        {
            return Conflict(ApiResponse<object>.Fail(409, ex.Message));
        }
    }

    /// <summary>
    /// 接收第三方推送的煤炭体积计算结果。
    /// </summary>
    /// <remarks>
    /// 本项目自身不产出煤量：Analyzer 只算车厢尺寸，体积由第三方系统主动推送到这里。
    /// 因此这是本服务唯一仍在使用的入库端点，鉴权同其它合作方接口
    /// （Timestamp + Signature = MD5(Timestamp + AppSecret)）。
    /// </remarks>
    /// <param name="request">煤炭体积、置信度、追溯文件和状态信息。</param>
    /// <returns>入库成功后的记录 ID；幂等重复时返回已有记录 ID。</returns>
    [HttpPost("internal/pointcloud/coalVolumes")]
    public async Task<ActionResult<ApiResponse<object>>> IngestCoalVolume(
        [FromBody] PointCloudCoalVolumeIngestRequest? request,
        CancellationToken cancellationToken)
    {
        if (request == null)
        {
            return BadRequest(ApiResponse<object>.Fail(400, "请求体不能为空"));
        }

        try
        {
            request.IdempotencyKey = ResolveIdempotencyKey(request.IdempotencyKey);
            var id = await _queryService.IngestCoalVolumeAsync(request, cancellationToken);
            return Ok(ApiResponse<object>.Success(new { id }));
        }
        catch (ArgumentException ex)
        {
            return BadRequest(ApiResponse<object>.Fail(400, ex.Message));
        }
        catch (IdempotencyConflictException ex)
        {
            return Conflict(ApiResponse<object>.Fail(409, ex.Message));
        }
    }

    private string? ResolveIdempotencyKey(string? bodyValue)
    {
        var bodyKey = NormalizeOptionalIdempotencyKey(bodyValue);
        string? headerKey = null;
        if (Request.Headers.TryGetValue("Idempotency-Key", out var headerValue))
        {
            if (headerValue.Count != 1)
            {
                throw new ArgumentException("Idempotency-Key 请求头只能出现一次");
            }

            headerKey = NormalizeOptionalIdempotencyKey(headerValue[0]);
        }

        if (bodyKey is not null && headerKey is not null &&
            !string.Equals(bodyKey, headerKey, StringComparison.Ordinal))
        {
            throw new ArgumentException("Idempotency-Key 请求头与请求体 idempotencyKey 不一致");
        }

        return bodyKey ?? headerKey;
    }

    private static string? NormalizeOptionalIdempotencyKey(string? raw)
    {
        raw = raw?.Trim();
        if (string.IsNullOrWhiteSpace(raw))
        {
            return null;
        }

        if (!IdempotencyKeyRegex.IsMatch(raw))
        {
            throw new ArgumentException("Idempotency-Key 格式非法，要求 1-64 位字母、数字、下划线、连字符、冒号或点");
        }

        return raw;
    }

    /// <summary>
    /// 把本次查询的实际执行参数交给访问日志中间件，作为一条 QueryExecuted 生命周期记录。
    /// </summary>
    /// <remarks>不写任何响应头或响应体，第三方看到的请求/返回契约完全不变。</remarks>
    private void RecordQueryDiagnostics(string endpoint, CursorQueryDiagnostics? diagnostics)
    {
        if (diagnostics is null)
        {
            return;
        }

        IntegrationAudit.SetQueryDiagnostics(HttpContext, diagnostics.Describe(endpoint));
    }

    private static void ValidatePartnerQuery(HttpRequest request)
    {
        if (request.Query.Count != 1 ||
            !(request.Query.ContainsKey("limit") || request.Query.ContainsKey("timeWindow")))
            throw new ArgumentException("必须且只能传入 limit 或 timeWindow，不接受其他查询参数");
        var value = request.Query.First().Value;
        if (value.Count != 1 || string.IsNullOrEmpty(value[0]) ||
            !int.TryParse(value[0], System.Globalization.NumberStyles.AllowLeadingSign,
                System.Globalization.CultureInfo.InvariantCulture, out _))
            throw new ArgumentException("查询参数必须是单个非空整数");
        if (request.ContentLength > 0 || request.Headers.ContainsKey("Transfer-Encoding"))
            throw new ArgumentException("GET 查询仅接受 URL Query 参数，不接受请求正文");
    }
}
