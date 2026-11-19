using System.Globalization;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Api;

/// <summary>地感/抓拍设备的来车通知：登记 VehicleArrival，车牌由后台识牌任务补全。</summary>
[ApiController]
[Route("api/v1/radar")]
public class RadarController(
    IIntegrationRepository repository,
    IOptions<IntegrationOptions> integrationOptions,
    ILogger<RadarController> logger) : ControllerBase
{
    private const int MaxIdentifierLength = 64;
    private const int EventIdMaxLength = 128;

    [HttpPost("vehicle-arrival")]
    public ActionResult<ApiResponse<VehicleArrivalResponse>> VehicleArrival([FromBody] VehicleArrivalRequest? request)
    {
        var deviceId = request?.DeviceId?.Trim() ?? string.Empty;
        var cameraIdentifier = request?.CameraIdentifier?.Trim() ?? string.Empty;
        if (deviceId.Length is 0 or > MaxIdentifierLength)
            return Fail($"device_id 不能为空且长度不能超过 {MaxIdentifierLength}");
        if (cameraIdentifier.Length is 0 or > MaxIdentifierLength)
            return Fail($"cameraIdentifier 不能为空且长度不能超过 {MaxIdentifierLength}");
        if (!DateTime.TryParseExact(request!.Timestamp, "yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture,
                DateTimeStyles.None, out var arrivalTime))
            return Fail("timestamp 格式错误，应为 yyyy-MM-dd HH:mm:ss");
        if (request.EventType?.Trim() != "ARRIVAL")
            return Fail("event_type 必须为 ARRIVAL");

        var bodyEventId = Normalize(request.EventId);
        var headerEventId = Normalize(Request.Headers["Idempotency-Key"].ToString());
        if (bodyEventId is not null && headerEventId is not null && bodyEventId != headerEventId)
            return Fail("event_id 与 Idempotency-Key 请求头不一致");
        var callerEventId = bodyEventId ?? headerEventId;
        if (callerEventId?.Length > EventIdMaxLength)
            return Fail($"event_id 长度不能超过 {EventIdMaxLength}");

        // CAM004 的来车由载货雷达 CH4 登记，这里只记日志。
        if (cameraIdentifier == "CAM004")
        {
            logger.LogInformation("CAM004 来车通知已停用，仅记录: eventId={EventId}, deviceId={DeviceId}, timestamp={Timestamp}",
                callerEventId, deviceId, request.Timestamp);
            return Ok(ApiResponse<VehicleArrivalResponse>.Success(ArrivalResponse(string.Empty, "handled_by_load_radar", false, callerEventId)));
        }

        var now = BusinessClock.Now;
        var options = integrationOptions.Value;
        var mockPlate = options.EnableMockLicensePlate ? MockLicensePlateGenerator.Generate(options.MockLicensePlatePrefix) : null;
        var candidate = new VehicleArrivalRecord
        {
            EventId = callerEventId ?? $"auto_{Guid.NewGuid():N}",
            DeviceId = deviceId,
            CameraIdentifier = cameraIdentifier,
            EventType = "ARRIVAL",
            SessionId = $"sess_{now:yyyyMMddHHmmssfff}_{Guid.NewGuid().ToString("N")[..12]}",
            LicensePlate = mockPlate,
            PlateStatus = mockPlate is null ? VehicleArrivalPlateStatus.Pending : VehicleArrivalPlateStatus.Succeeded,
            PlateSource = mockPlate is null ? null : "MOCK",
            PlateCapturedAt = mockPlate is null ? null : now,
            ArrivalTime = arrivalTime,
            RadarStatus = VehicleArrivalRadarStatus.NotRequired,
            CreatedAt = now,
            UpdatedAt = now
        };
        var inserted = repository.InsertVehicleArrivalRecord(candidate);
        var record = inserted.Record;
        if (!inserted.IsNew &&
            (record.DeviceId != candidate.DeviceId || record.CameraIdentifier != candidate.CameraIdentifier ||
             record.EventType != candidate.EventType || record.ArrivalTime != candidate.ArrivalTime))
            return Conflict(ApiResponse<VehicleArrivalResponse>.Fail(409, "相同 event_id 已用于不同的来车事件"));

        return Ok(ApiResponse<VehicleArrivalResponse>.Success(ArrivalResponse(
            record.SessionId, inserted.IsNew ? "accepted" : "already_processed", !inserted.IsNew, callerEventId,
            record.LicensePlate)));
    }

    private BadRequestObjectResult Fail(string message) =>
        BadRequest(ApiResponse<VehicleArrivalResponse>.Fail(400, message));

    private static string? Normalize(string? value) => string.IsNullOrWhiteSpace(value) ? null : value.Trim();

    private static VehicleArrivalResponse ArrivalResponse(
        string sessionId, string status, bool duplicate, string? callerEventId, string? licensePlate = null) => new()
        {
            SessionId = sessionId,
            WorkflowId = sessionId,
            LicensePlate = licensePlate,
            Status = status,
            Duplicate = duplicate,
            EventId = callerEventId
        };
}
