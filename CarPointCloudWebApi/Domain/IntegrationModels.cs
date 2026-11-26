using System.Text.Json;
using System.Text.Json.Serialization;

namespace CarPointCloudWebApi.Domain;

/// <summary>
/// 车牌识别调用记录。
/// </summary>
/// <remarks>
/// 业务用途：保存我方主动调用乙方车牌接口的请求结果，用于追溯识别结果、异常原因和后续数据关联。
/// </remarks>
public class LicensePlateRecord
{
    /// <summary>数据库自增主键。</summary>
    public long Id { get; set; }
    /// <summary>内部车辆流程 ID；每个流程最多保留一条车牌业务记录。</summary>
    public string? WorkflowId { get; set; }
    /// <summary>内部交互链路 ID，仅用于日志和故障追踪，不发送给乙方。</summary>
    public string? InteractionId { get; set; }
    /// <summary>我方摄像头/雷达点位编码，请求乙方接口时传入。</summary>
    public string CameraIdentifier { get; set; } = string.Empty;
    /// <summary>乙方返回的摄像头编码，应与 CameraIdentifier 一致。</summary>
    public string CameraId { get; set; } = string.Empty;
    /// <summary>乙方识别出的车牌号；失败时为空字符串。</summary>
    public string LicensePlate { get; set; } = string.Empty;
    /// <summary>乙方识别或抓拍时间；乙方未返回时为空。</summary>
    public DateTime? CaptureTime { get; set; }
    /// <summary>乙方原始业务 ID；旧数据缺失时为空，不以 workflowId 代替。</summary>
    public string? EnterId { get; set; }
    /// <summary>同点位首次在我方收到该 enterID 的业务时间；不使用乙方时钟。</summary>
    [System.Text.Json.Serialization.JsonIgnore]
    public DateTime? FirstSeenAt { get; set; }
    [System.Text.Json.Serialization.JsonIgnore]
    public bool ConflictingIdentity { get; set; }
    /// <summary>我方发起请求时间。</summary>
    public DateTime RequestTime { get; set; }
    /// <summary>乙方业务响应码。</summary>
    public int ResponseCode { get; set; }
    /// <summary>乙方响应描述。</summary>
    public string ResponseMessage { get; set; } = string.Empty;
    /// <summary>本次调用是否成功，要求乙方 code=200 且 data 不为空。</summary>
    public bool IsSuccess { get; set; }
    /// <summary>调用失败、解析失败或业务校验失败时的错误详情。</summary>
    public string? ErrorMessage { get; set; }
    /// <summary>记录创建时间。</summary>
    public DateTime CreatedAt { get; set; } = DateTime.Now;
}

/// <summary>
/// 煤炭体积测量记录。
/// </summary>
/// <remarks>
/// 当前接口仅负责对外提供煤炭体积数据；实际体积来源可由后续算法或人工系统写入该表。
/// </remarks>
public class CoalVolumeRecord
{
    /// <summary>数据库自增主键。</summary>
    public long Id { get; set; }
    /// <summary>幂等键，用于防止 Worker 重试导致重复入库。</summary>
    public string IdempotencyKey { get; set; } = string.Empty;
    /// <summary>由现有请求字段计算的稳定业务键，防止调用方更换幂等键后重复入库。</summary>
    public string BusinessKey { get; set; } = string.Empty;
    /// <summary>根据车牌、点位和测量时间在内部匹配的车辆流程 ID。</summary>
    public string? WorkflowId { get; set; }
    /// <summary>规范化客户端业务载荷的 SHA-256 指纹；仅用于幂等冲突检测。</summary>
    public string? PayloadHash { get; set; }
    /// <summary>车辆车牌号。</summary>
    public string LicensePlate { get; set; } = string.Empty;
    /// <summary>采集点位编码。</summary>
    public string CameraIdentifier { get; set; } = string.Empty;
    /// <summary>煤炭体积，单位由 Unit 指定，接口返回前保留 1 位小数。</summary>
    public decimal Volume { get; set; }
    /// <summary>体积单位，默认立方米 m³。</summary>
    public string Unit { get; set; } = "m³";
    /// <summary>体积测量时间。</summary>
    public DateTime MeasureTime { get; set; }
    /// <summary>已确认的实际测量开始时间；旧测量时间不能冒充开始时间。</summary>
    public DateTime? MeasurementStartedAt { get; set; }
    /// <summary>实际测量结束时间；旧记录缺失时为空。</summary>
    public DateTime? MeasurementEndedAt { get; set; }
    /// <summary>从精确关联的成功车牌记录读取的乙方业务 ID。</summary>
    public string? EnterId { get; set; }
    /// <summary>从同一车牌记录读取的实际抓拍时间。</summary>
    public DateTime? CaptureTime { get; set; }
    /// <summary>可选关联车牌识别记录主键。</summary>
    public long? LicensePlateRecordId { get; set; }
    /// <summary>体积计算状态，例如 Success、LowConfidence、Failed。</summary>
    public string Status { get; set; } = "Success";
    /// <summary>体积计算置信度，范围 0-1。</summary>
    public decimal? Confidence { get; set; }
    /// <summary>源 PCD 文件路径，仅用于追溯。</summary>
    public string? SourceFile { get; set; }
    /// <summary>空车基准文件路径，仅用于追溯。</summary>
    public string? BaselineFile { get; set; }
    /// <summary>记录创建时间。</summary>
    public DateTime CreatedAt { get; set; } = DateTime.Now;
}

/// <summary>
/// 车辆尺寸测量主记录。
/// </summary>
/// <remarks>
/// 业务关键点：所有尺寸字段单位均为 CM，保留 1 位小数；DetectStatus 非 success 时对外返回 dimensions 为空对象。
/// </remarks>
public class VehicleDimensionRecord
{
    /// <summary>独立实测最大外宽，单位 CM；内部 TrailerWidth 仍为净宽。</summary>
    public decimal? MaximumWidthCm { get; set; }
    /// <summary>实际拉筋数；N+1 尾段不是拉筋，旧记录未标注时为空。</summary>
    public int? ActualRibCount { get; set; }
    /// <summary>数据库自增主键。</summary>
    public long Id { get; set; }
    /// <summary>幂等键，用于防止 Worker 重试导致重复入库。</summary>
    public string IdempotencyKey { get; set; } = string.Empty;
    /// <summary>贯穿来车、扫描、分析和入库的车辆流程 ID。</summary>
    public string? WorkflowId { get; set; }
    /// <summary>双雷达 PCD 配对 ID。</summary>
    public string? PairId { get; set; }
    /// <summary>规范化客户端业务载荷的 SHA-256 指纹；仅用于幂等冲突检测。</summary>
    public string? PayloadHash { get; set; }
    /// <summary>车辆车牌号；无车牌时使用 UNKNOWN。</summary>
    public string LicensePlate { get; set; } = string.Empty;
    /// <summary>雷达/摄像头点位编码。</summary>
    public string CameraIdentifier { get; set; } = string.Empty;
    /// <summary>数据获取时间。</summary>
    public DateTime GetTime { get; set; }
    /// <summary>检测状态；success 表示尺寸可用，其他状态表示检测失败或异常。</summary>
    public string DetectStatus { get; set; } = "success";
    /// <summary>车头到车厢前沿距离，单位 CM。</summary>
    public decimal? CabToTrailerFrontDist { get; set; }
    /// <summary>车厢长度，单位 CM。</summary>
    public decimal? TrailerLength { get; set; }
    /// <summary>车厢宽度，单位 CM。</summary>
    public decimal? TrailerWidth { get; set; }
    /// <summary>车厢厢体高度，单位 CM。</summary>
    public decimal? TrailerBodyHeight { get; set; }
    /// <summary>车厢底部离地高度，单位 CM。</summary>
    public decimal? TrailerBottomToGroundHeight { get; set; }
    /// <summary>高低板高度差，单位 CM。</summary>
    public decimal? HighLowPlateHeight { get; set; }
    /// <summary>高低板明细集合，按 PlateId 升序输出。</summary>
    public List<HighLowPlateInfo> HighLowPlates { get; set; } = new List<HighLowPlateInfo>();
    /// <summary>记录创建时间。</summary>
    public DateTime CreatedAt { get; set; } = DateTime.Now;
    /// <summary>记录更新时间。</summary>
    public DateTime UpdatedAt { get; set; } = DateTime.Now;
    /// <summary>源 PCD 文件路径，仅用于追溯，不对外返回。</summary>
    public string? SourceFile { get; set; }
    /// <summary>Python 解析结果 JSON 路径，仅用于追溯，不对外返回。</summary>
    public string? ResultJsonPath { get; set; }
    /// <summary>Python 可视化图片路径，仅用于追溯，不对外返回。</summary>
    public string? ResultImagePath { get; set; }
    /// <summary>车厢筋条明细集合，按 RibId 升序输出。</summary>
    public List<RibInfo> Ribs { get; set; } = new List<RibInfo>();
    /// <summary>钢丝绳明细集合，按 WireId 升序输出。</summary>
    public List<WireInfo> Wires { get; set; } = new List<WireInfo>();
}

/// <summary>车厢筋条间距明细。</summary>
public class RibInfo
{
    /// <summary>数据库自增主键。</summary>
    public long Id { get; set; }
    /// <summary>所属车辆尺寸主记录 ID。</summary>
    public long DimensionRecordId { get; set; }
    /// <summary>筋条序号。</summary>
    public int RibId { get; set; }
    /// <summary>与上一筋条或车厢前沿的距离，单位 CM。</summary>
    public decimal? DistToPrev { get; set; }
}

/// <summary>车厢钢丝绳间距明细。</summary>
public class WireInfo
{
    /// <summary>数据库自增主键。</summary>
    public long Id { get; set; }
    /// <summary>所属车辆尺寸主记录 ID。</summary>
    public long DimensionRecordId { get; set; }
    /// <summary>钢丝绳序号。</summary>
    public int WireId { get; set; }
    /// <summary>与上一钢丝绳或基准位置的距离，单位 CM。</summary>
    public decimal? DistToPrev { get; set; }
}

/// <summary>高低板明细。</summary>
public class HighLowPlateInfo
{
    /// <summary>数据库自增主键。</summary>
    public long Id { get; set; }
    /// <summary>所属车辆尺寸主记录 ID。</summary>
    public long DimensionRecordId { get; set; }
    /// <summary>高低板编号。</summary>
    public int PlateId { get; set; }
    /// <summary>该高低板最高点离地高度，单位 CM。</summary>
    public decimal? HighestToGround { get; set; }
    /// <summary>独立实测高低板长度，单位 CM；原最高点高度不迁入此字段。</summary>
    public decimal? LengthCm { get; set; }
    /// <summary>该高低板最低点离地高度，单位 CM。</summary>
    public decimal? LowestToGround { get; set; }
}

/// <summary>车辆尺寸接口响应对象。</summary>
public class VehicleDimensionDto
{
    /// <summary>对外最大外宽，旧记录缺失时保持 null。</summary>
    [System.Text.Json.Serialization.JsonIgnore]
    public decimal? MaximumWidthCm { get; set; }
    /// <summary>供对外投影剔除 N+1 尾段，内部明细不裁剪。</summary>
    [System.Text.Json.Serialization.JsonIgnore]
    public int? ActualRibCount { get; set; }
    /// <summary>磅房内部查询的待核验标记；已入库不等于审核通过。</summary>
    [System.Text.Json.Serialization.JsonIgnore(Condition = System.Text.Json.Serialization.JsonIgnoreCondition.WhenWritingNull)]
    public string? ReviewStatus { get; set; }
    /// <summary>稳定数据库记录 ID，可用于跨页去重。</summary>
    public long RecordId { get; set; }
    /// <summary>关联的来车业务流程 ID；旧数据或未匹配记录可为空。</summary>
    public string? WorkflowId { get; set; }
    /// <summary>双雷达分析配对标识；单雷达记录可为空。</summary>
    public string? PairId { get; set; }
    /// <summary>车辆车牌号。</summary>
    public string LicensePlate { get; set; } = string.Empty;
    /// <summary>雷达/摄像头点位编码。</summary>
    public string CameraIdentifier { get; set; } = string.Empty;
    /// <summary>数据获取时间。</summary>
    public DateTime GetTime { get; set; }
    /// <summary>尺寸数据；检测成功时为 VehicleDimensionsDto，失败时为空对象。</summary>
    public object Dimensions { get; set; } = new();
}

/// <summary>车辆尺寸明细响应对象，尺寸单位均为 CM。</summary>
public class VehicleDimensionsDto
{
    /// <summary>车头到车厢前沿距离，单位 CM。</summary>
    public decimal? CabToTrailerFrontDist { get; set; }
    /// <summary>车厢长度，单位 CM。</summary>
    public decimal? TrailerLength { get; set; }
    /// <summary>车厢宽度，单位 CM。</summary>
    public decimal? TrailerWidth { get; set; }
    /// <summary>车厢厢体高度，单位 CM。</summary>
    public decimal? TrailerBodyHeight { get; set; }
    /// <summary>车厢底部离地高度，单位 CM。</summary>
    public decimal? TrailerBottomToGroundHeight { get; set; }
    /// <summary>高低板高度差，单位 CM。</summary>
    public decimal? HighLowPlateHeight { get; set; }
    /// <summary>高低板信息数组，元素包含 plateId、highestToGround 和 lowestToGround。</summary>
    public List<HighLowPlateDto> HighLowPlates { get; set; } = new List<HighLowPlateDto>();
    /// <summary>筋条间距列表。</summary>
    public List<RibDto> Ribs { get; set; } = new List<RibDto>();
    /// <summary>钢丝绳间距列表。</summary>
    public List<WireDto> Wires { get; set; } = new List<WireDto>();
}

/// <summary>筋条响应对象。</summary>
public class RibDto
{
    /// <summary>筋条序号。</summary>
    public int RibId { get; set; }
    /// <summary>与上一筋条或车厢前沿的距离，单位 CM。</summary>
    public decimal? DistToPrev { get; set; }
}

/// <summary>钢丝绳响应对象。</summary>
public class WireDto
{
    /// <summary>钢丝绳序号。</summary>
    public int WireId { get; set; }
    /// <summary>与上一钢丝绳或基准位置的距离，单位 CM。</summary>
    public decimal? DistToPrev { get; set; }
}

/// <summary>高低板响应对象。</summary>
public class HighLowPlateDto
{
    /// <summary>高低板编号。</summary>
    public int PlateId { get; set; }
    /// <summary>该高低板最高点到地面的高度，单位 CM。</summary>
    public decimal? HighestToGround { get; set; }
    /// <summary>供合作方 v4 投影使用的独立实测高低板长度，单位 CM。</summary>
    [System.Text.Json.Serialization.JsonIgnore(Condition = System.Text.Json.Serialization.JsonIgnoreCondition.WhenWritingNull)]
    public decimal? LengthCm { get; set; }
    /// <summary>该高低板最低点到地面的高度，单位 CM。</summary>
    public decimal? LowestToGround { get; set; }
}

/// <summary>煤炭体积接口响应对象。</summary>
public class CoalVolumeDto
{
    /// <summary>稳定数据库记录 ID，可用于跨页去重。</summary>
    public long RecordId { get; set; }
    /// <summary>车辆车牌号。</summary>
    public string LicensePlate { get; set; } = string.Empty;
    /// <summary>采集点位编码。</summary>
    public string CameraIdentifier { get; set; } = string.Empty;
    /// <summary>煤炭体积，保留 1 位小数。</summary>
    public decimal Volume { get; set; }
    /// <summary>体积单位。</summary>
    public string Unit { get; set; } = "m³";
    /// <summary>计算状态：success、failed、low_confidence 或 unknown。</summary>
    public string Status { get; set; } = string.Empty;
    /// <summary>是否为异常结果；除 success 外均为异常。</summary>
    public bool IsAbnormal { get; set; }
    /// <summary>测量时间。</summary>
    public DateTime MeasureTime { get; set; }
    /// <summary>确认过的实际测量开始时间；缺失时为空。</summary>
    public DateTime? MeasurementStartedAt { get; set; }
    /// <summary>实际测量结束时间；缺失时为空。</summary>
    public DateTime? MeasurementEndedAt { get; set; }
    /// <summary>乙方业务 ID，来自同次车牌抓拍。</summary>
    public string? EnterId { get; set; }
    /// <summary>乙方同次抓拍时间。</summary>
    public DateTime? CaptureTime { get; set; }
}

/// <summary>乙方车牌接口 data 节点。</summary>
public class PartnerLicensePlateData
{
    [System.Text.Json.Serialization.JsonIgnore]
    public DateTime? FirstSeenAt { get; set; }
    [System.Text.Json.Serialization.JsonIgnore]
    public bool ConflictingIdentity { get; set; }
    /// <summary>乙方返回的摄像头编码。</summary>
    public string CameraId { get; set; } = string.Empty;
    /// <summary>乙方识别出的车牌号。</summary>
    public string LicensePlate { get; set; } = string.Empty;
    /// <summary>乙方识别或抓拍时间，格式 "yyyy-MM-dd HH:mm:ss"。</summary>
    public string CaptureTime { get; set; } = string.Empty;
    /// <summary>乙方业务 ID，必须原样传递；字段名按合同保留大写 ID。</summary>
    [System.Text.Json.Serialization.JsonPropertyName("enterID")]
    [JsonConverter(typeof(PartnerEnterIdConverter))]
    public string EnterId { get; set; } = string.Empty;
}

internal sealed class PartnerEnterIdConverter : JsonConverter<string>
{
    public override string Read(ref Utf8JsonReader reader, Type type, JsonSerializerOptions options)
    {
        if (reader.TokenType == JsonTokenType.String) return reader.GetString()!;
        if (reader.TokenType == JsonTokenType.Number)
        {
            // 按 JSON 原文读取整数，避免长 ID 转为浮点或受 Int64 上限影响。
            using var token = JsonDocument.ParseValue(ref reader);
            var raw = token.RootElement.GetRawText();
            if (raw.IndexOfAny(['.', 'e', 'E']) < 0) return raw;
        }
        throw new JsonException("乙方 enterID 应为字符串或整数");
    }

    public override void Write(Utf8JsonWriter writer, string value, JsonSerializerOptions options) =>
        writer.WriteStringValue(value);
}

/// <summary>乙方车牌接口响应对象。</summary>
public class PartnerLicensePlateResponse
{
    /// <summary>乙方业务响应码。</summary>
    public JsonElement Code { get; set; }
    /// <summary>现场乙方返回的成功标志。</summary>
    public JsonElement Success { get; set; }
    internal int? BusinessCode => Code.ValueKind == JsonValueKind.Number && Code.TryGetInt32(out var code) ? code : null;
    internal bool IsSuccessful => Code.ValueKind == JsonValueKind.Undefined
        ? Success.ValueKind == JsonValueKind.True
        : BusinessCode == 200 && Success.ValueKind is JsonValueKind.Undefined or JsonValueKind.True;
    /// <summary>乙方业务提示信息。</summary>
    public string Message { get; set; } = string.Empty;
    /// <summary>乙方车牌数据数组；失败时可能为空。</summary>
    public List<PartnerLicensePlateData>? Data { get; set; }
}

/// <summary>API 访问日志记录。</summary>
public class ApiAccessLogRecord
{
    /// <summary>记录类型：Access（完整 HTTP 快照）或 Lifecycle（运行阶段事件）。</summary>
    public string RecordKind { get; set; } = "Access";
    /// <summary>运行日志级别：debug / info / warn / error。</summary>
    public string LogLevel { get; set; } = "info";
    /// <summary>接口生命周期阶段，如 RequestReceived、RequestCompleted、PartnerRetryScheduled。</summary>
    public string? LifecycleStage { get; set; }
    /// <summary>面向运维人员的结构化运行日志说明。</summary>
    public string? LogMessage { get; set; }
    /// <summary>同一次第三方交互链路的稳定标识；同一链路可包含多个 HTTP 跳。</summary>
    public string? InteractionId { get; set; }
    /// <summary>交互类型：LicensePlate、VehicleDimensions、CoalVolume 或 VehicleArrival。</summary>
    public string? IntegrationType { get; set; }
    /// <summary>请求方向：Inbound 或 Outbound。</summary>
    public string? Direction { get; set; }
    /// <summary>发起方系统。</summary>
    public string? CallerSystem { get; set; }
    /// <summary>目标系统。</summary>
    public string? TargetSystem { get; set; }
    /// <summary>业务流程 ID。</summary>
    public string? WorkflowId { get; set; }
    /// <summary>业务来车事件 ID。</summary>
    public string? EventId { get; set; }
    /// <summary>车牌号；无法从当前跳提取时为空。</summary>
    public string? LicensePlate { get; set; }
    /// <summary>雷达/摄像头点位。</summary>
    public string? CameraIdentifier { get; set; }
    /// <summary>本次请求在同一链路内的尝试序号。</summary>
    public int? AttemptNo { get; set; }
    /// <summary>HTTP 请求方法。</summary>
    public string HttpMethod { get; set; } = string.Empty;
    /// <summary>请求协议，例如 http 或 https。</summary>
    public string? Scheme { get; set; }
    /// <summary>调用目标主机及端口。</summary>
    public string? Host { get; set; }
    /// <summary>ASP.NET Core 请求跟踪标识。</summary>
    public string? TraceIdentifier { get; set; }
    /// <summary>调用方 User-Agent。</summary>
    public string? UserAgent { get; set; }
    /// <summary>请求体媒体类型。</summary>
    public string? RequestContentType { get; set; }
    /// <summary>脱敏后的合作方 Signature 请求头。</summary>
    public string? SignatureHeader { get; set; }
    /// <summary>调用方提供的幂等键请求头。</summary>
    public string? IdempotencyKeyHeader { get; set; }
    /// <summary>脱敏后的请求头 JSON。</summary>
    public string? RequestHeadersJson { get; set; }
    /// <summary>按审计长度上限截断后的请求体。</summary>
    public string? RequestBody { get; set; }
    /// <summary>响应体媒体类型。</summary>
    public string? ResponseContentType { get; set; }
    /// <summary>脱敏后的响应头 JSON。</summary>
    public string? ResponseHeadersJson { get; set; }
    /// <summary>按审计长度上限截断后的响应体。</summary>
    public string? ResponseBody { get; set; }
    /// <summary>请求体长度，未知时为空。</summary>
    public long? RequestContentLength { get; set; }
    /// <summary>响应体长度，未知时为空。</summary>
    public long? ResponseContentLength { get; set; }
    /// <summary>请求路径。</summary>
    public string RequestPath { get; set; } = string.Empty;
    /// <summary>查询字符串。</summary>
    public string? QueryString { get; set; }
    /// <summary>调用方 IP。</summary>
    public string? ClientIp { get; set; }
    /// <summary>调用方传入的 Timestamp 请求头。</summary>
    public string? TimestampHeader { get; set; }
    /// <summary>HTTP 状态码。</summary>
    public int HttpStatus { get; set; }
    /// <summary>业务状态码，当前与 HTTP 状态码一致。</summary>
    public int BusinessCode { get; set; }
    /// <summary>接口耗时，单位毫秒。</summary>
    public int ElapsedMs { get; set; }
    /// <summary>异常信息；正常请求为空。</summary>
    public string? ErrorMessage { get; set; }
    /// <summary>记录创建时间。</summary>
    public DateTime CreatedAt { get; set; } = DateTime.Now;
}

/// <summary>
/// Python 点云解析结果写入 WebApi 的内部请求对象。
/// </summary>
/// <remarks>
/// 该接口属于系统内部联动入口，同样使用 Timestamp + Signature 鉴权，防止未授权写入车辆尺寸数据。
/// </remarks>
public class PointCloudDimensionIngestRequest
{
    /// <summary>独立实测最大外宽，单位 CM；不以内部净宽回填。</summary>
    public decimal? MaximumWidthCm { get; set; }
    /// <summary>实际拉筋数，区分拉筋与到后壁的 N+1 尾段。</summary>
    public int? ActualRibCount { get; set; }
    /// <summary>幂等键；为空时服务端尝试从 Idempotency-Key 请求头补齐。</summary>
    public string? IdempotencyKey { get; set; }
    /// <summary>来车业务流程 ID，用于把分析结果绑定到触发本次扫描的车辆。</summary>
    public string? WorkflowId { get; set; }
    /// <summary>双雷达配对标识，用于保证两份点云属于同一次融合分析。</summary>
    public string? PairId { get; set; }
    /// <summary>雷达/摄像头点位编码，必填。</summary>
    public string CameraIdentifier { get; set; } = string.Empty;
    /// <summary>车牌号；为空时服务端写入 UNKNOWN。</summary>
    public string? LicensePlate { get; set; }
    /// <summary>点云获取时间；为空时服务端使用当前 本地时间（DateTime.Now）。</summary>
    public DateTime? GetTime { get; set; }
    /// <summary>检测状态；默认 success。</summary>
    public string DetectStatus { get; set; } = "success";
    /// <summary>车辆尺寸主体数据，单位 CM。</summary>
    public PointCloudDimensionsPayload Dimensions { get; set; } = new();
    /// <summary>高低板间距列表，单位 CM。</summary>
    public List<HighLowPlateDto> HighLowPlates { get; set; } = new List<HighLowPlateDto>();
    /// <summary>筋条间距列表，单位 CM。</summary>
    public List<RibDto> Ribs { get; set; } = new List<RibDto>();
    /// <summary>钢丝绳间距列表，单位 CM。</summary>
    public List<WireDto> Wires { get; set; } = new List<WireDto>();
    /// <summary>源 PCD 文件路径，仅用于追溯，不对外返回。</summary>
    public string? SourceFile { get; set; }
    /// <summary>Python 解析结果 JSON 路径，仅用于追溯，不对外返回。</summary>
    public string? ResultJsonPath { get; set; }
    /// <summary>Python 可视化图片路径，仅用于追溯，不对外返回。</summary>
    public string? ResultImagePath { get; set; }
}

/// <summary>
/// 点云算法计算出的煤炭体积写入请求。
/// </summary>
public class PointCloudCoalVolumeIngestRequest
{
    /// <summary>幂等键；为空时服务端尝试从 Idempotency-Key 请求头补齐。</summary>
    public string? IdempotencyKey { get; set; }
    /// <summary>雷达/摄像头点位编码，必填。</summary>
    public string CameraIdentifier { get; set; } = string.Empty;
    /// <summary>车牌号；为空时服务端写入 UNKNOWN。</summary>
    public string? LicensePlate { get; set; }
    /// <summary>与 measureEndTime 成对时表示实际开始；旧单时间载荷只作内部排序时间。</summary>
    public DateTime? MeasureTime { get; set; }
    /// <summary>实际测量结束时间；与 measureTime 配对，不以接收时间替代。</summary>
    public DateTime? MeasureEndTime { get; set; }
    /// <summary>煤炭体积，单位 m³。</summary>
    public decimal Volume { get; set; }
    /// <summary>体积单位，必填，例如 m³。</summary>
    public string Unit { get; set; } = string.Empty;
    /// <summary>计算状态，必填：success、failed、low_confidence 或 unknown。</summary>
    public string Status { get; set; } = string.Empty;
    /// <summary>计算置信度，范围 0-1。</summary>
    public decimal? Confidence { get; set; }
    /// <summary>源 PCD 文件路径。</summary>
    public string? SourceFile { get; set; }
    /// <summary>空车基准文件路径。</summary>
    public string? BaselineFile { get; set; }
}

/// <summary>Python 点云解析出的车辆尺寸数据，单位均为 CM。</summary>
public class PointCloudDimensionsPayload
{
    /// <summary>车头到车厢前沿距离，单位 CM。</summary>
    public decimal? CabToTrailerFrontDist { get; set; }
    /// <summary>车厢长度，单位 CM。</summary>
    public decimal? TrailerLength { get; set; }
    /// <summary>车厢宽度，单位 CM。</summary>
    public decimal? TrailerWidth { get; set; }
    /// <summary>车厢厢体高度，单位 CM。</summary>
    public decimal? TrailerBodyHeight { get; set; }
    /// <summary>车厢底部离地高度，单位 CM。</summary>
    public decimal? TrailerBottomToGroundHeight { get; set; }
    /// <summary>高低板高度差，单位 CM。</summary>
    public decimal? HighLowPlateHeight { get; set; }
}

/// <summary>
/// 雷达来车上报请求。
/// </summary>
public class VehicleArrivalRequest
{
    /// <summary>雷达设备编号，如 RADAR_001。</summary>
    [System.Text.Json.Serialization.JsonPropertyName("device_id")]
    public string DeviceId { get; set; } = string.Empty;
    /// <summary>摄像头/雷达点位编码，如 CAM003。</summary>
    [System.Text.Json.Serialization.JsonPropertyName("cameraIdentifier")]
    public string CameraIdentifier { get; set; } = string.Empty;
    /// <summary>检测时间戳，格式 yyyy-MM-dd HH:mm:ss。</summary>
    [System.Text.Json.Serialization.JsonPropertyName("timestamp")]
    public string Timestamp { get; set; } = string.Empty;
    /// <summary>事件类型，固定值 ARRIVAL。</summary>
    [System.Text.Json.Serialization.JsonPropertyName("event_type")]
    public string EventType { get; set; } = string.Empty;

    /// <summary>
    /// Optional caller-supplied event identifier used for idempotent retries.
    /// The same value may also be supplied through the Idempotency-Key header.
    /// </summary>
    [System.Text.Json.Serialization.JsonPropertyName("event_id")]
    public string? EventId { get; set; }
}

/// <summary>
/// 来车上报响应数据。
/// </summary>
public class VehicleArrivalResponse
{
    /// <summary>会话ID，格式 sess_{yyyyMMddHHmmss}_{序号}。</summary>
    [System.Text.Json.Serialization.JsonPropertyName("session_id")]
    public string SessionId { get; set; } = string.Empty;

    /// <summary>跨 WebApi、Worker、Analyzer 的业务流程 ID，与 session_id 相同。</summary>
    [System.Text.Json.Serialization.JsonPropertyName("workflow_id")]
    public string WorkflowId { get; set; } = string.Empty;

    /// <summary>该流程唯一绑定的车牌号。</summary>
    [System.Text.Json.Serialization.JsonPropertyName("license_plate")]
    public string? LicensePlate { get; set; }

    /// <summary>Processing state for new clients; existing clients may ignore this field.</summary>
    [System.Text.Json.Serialization.JsonPropertyName("status")]
    public string Status { get; set; } = "accepted";

    /// <summary>Whether this response belongs to an already persisted event.</summary>
    [System.Text.Json.Serialization.JsonPropertyName("duplicate")]
    public bool Duplicate { get; set; }

    /// <summary>The caller-supplied idempotency key, when present.</summary>
    [System.Text.Json.Serialization.JsonPropertyName("event_id")]
    [System.Text.Json.Serialization.JsonIgnore(
        Condition = System.Text.Json.Serialization.JsonIgnoreCondition.WhenWritingNull)]
    public string? EventId { get; set; }
}

/// <summary>Persisted radar processing states for a vehicle-arrival event.</summary>
public static class VehicleArrivalRadarStatus
{
    /// <summary>该点位无需触发雷达控制。</summary>
    public const string NotRequired = "NOT_REQUIRED";
    /// <summary>雷达控制已占用，等待完成。</summary>
    public const string Pending = "PENDING";
    /// <summary>雷达控制成功完成。</summary>
    public const string Succeeded = "SUCCEEDED";
    /// <summary>雷达控制失败或超时。</summary>
    public const string Failed = "FAILED";
}

/// <summary>持久化的车牌获取任务；不属于任何第三方接口契约。</summary>
public sealed class PlateAcquisitionJob
{
    /// <summary>待确认或已确认候选的持久检查点；不依赖消费者内存完成重启恢复。</summary>
    public string? CandidateJson { get; set; }
    /// <summary>车牌获取任务表自增主键。</summary>
    public long Id { get; set; }
    /// <summary>任务归属的来车业务流程 ID。</summary>
    public string WorkflowId { get; set; } = string.Empty;
    /// <summary>与来车请求审计链路一致的交互标识。</summary>
    public string InteractionId { get; set; } = string.Empty;
    /// <summary>需要查询车牌的摄像头点位。</summary>
    public string CameraIdentifier { get; set; } = string.Empty;
    /// <summary>任务状态：PENDING、PROCESSING、RETRY、SUCCEEDED 或 FAILED。</summary>
    public string Status { get; set; } = "PENDING";
    /// <summary>已经执行的外呼尝试次数。</summary>
    public int AttemptCount { get; set; }
    /// <summary>任务允许再次被后台 Worker 占用的时间。</summary>
    public DateTime NextAttemptAt { get; set; }
    /// <summary>最近一次失败摘要。</summary>
    public string? LastError { get; set; }
    /// <summary>任务创建时间。</summary>
    public DateTime CreatedAt { get; set; }
    /// <summary>任务状态最后更新时间。</summary>
    public DateTime UpdatedAt { get; set; }
}

/// <summary>Confirmed候选等待相邻来车登记窗口关闭后，在同一workflow中复核提交。</summary>
public sealed record PlateAcquisitionCandidate(LicensePlateRecord Record, bool Confirmed, DateTime ReadyAt);

/// <summary>来车流程的车牌获取状态常量。</summary>
public static class VehicleArrivalPlateStatus
{
    /// <summary>等待后台任务查询车牌。</summary>
    public const string Pending = "PENDING";
    /// <summary>已经取得并绑定唯一车牌。</summary>
    public const string Succeeded = "SUCCEEDED";
    /// <summary>达到最大尝试次数后仍未取得车牌。</summary>
    public const string Failed = "FAILED";
}

/// <summary>
/// 来车事件数据库记录。
/// </summary>
public class VehicleArrivalRecord
{
    /// <summary>数据库自增主键。</summary>
    public long Id { get; set; }
    /// <summary>Internal or caller-supplied unique event identifier.</summary>
    public string EventId { get; set; } = string.Empty;
    /// <summary>雷达设备编号。</summary>
    public string DeviceId { get; set; } = string.Empty;
    /// <summary>摄像头/雷达点位编码。</summary>
    public string CameraIdentifier { get; set; } = string.Empty;
    /// <summary>事件类型，固定值 ARRIVAL。</summary>
    public string EventType { get; set; } = string.Empty;
    /// <summary>会话ID，用于关联后续数据。</summary>
    public string SessionId { get; set; } = string.Empty;
    /// <summary>该流程唯一绑定的车牌号；识别完成前为空。</summary>
    public string? LicensePlate { get; set; }
    /// <summary>车牌获取状态。</summary>
    public string PlateStatus { get; set; } = VehicleArrivalPlateStatus.Pending;
    /// <summary>车牌来源，例如合作方识别服务。</summary>
    public string? PlateSource { get; set; }
    /// <summary>合作方实际抓拍或识别车牌的时间。</summary>
    public DateTime? PlateCapturedAt { get; set; }
    /// <summary>雷达检测时间。</summary>
    public DateTime ArrivalTime { get; set; }
    /// <summary>Current state of the optional CAM004 radar call.</summary>
    public string RadarStatus { get; set; } = VehicleArrivalRadarStatus.NotRequired;
    /// <summary>Last radar error, truncated to the database column limit.</summary>
    public string? RadarError { get; set; }
    /// <summary>记录创建时间。</summary>
    public DateTime CreatedAt { get; set; } = DateTime.Now;
    /// <summary>Last persistence or radar-state update time.</summary>
    public DateTime UpdatedAt { get; set; } = DateTime.Now;
}

/// <summary>Worker 按业务流程读取唯一车牌时使用的响应数据。</summary>
public sealed class VehicleWorkflowPlateResponse
{
    /// <summary>请求对应的车辆业务流程 ID。</summary>
    public string WorkflowId { get; set; } = string.Empty;
    /// <summary>已经成功绑定到流程的车牌号。</summary>
    public string LicensePlate { get; set; } = string.Empty;
    /// <summary>车牌数据来源。</summary>
    public string Source { get; set; } = string.Empty;
    /// <summary>合作方抓拍或识别车牌的时间。</summary>
    public DateTime? CapturedAt { get; set; }
}

/// <summary>Atomic insert result used to identify idempotent duplicate events.</summary>
public sealed class VehicleArrivalInsertResult
{
    /// <summary>新建或幂等命中的来车记录。</summary>
    public required VehicleArrivalRecord Record { get; init; }
    /// <summary>是否由本次调用首次创建；false 表示幂等重复。</summary>
    public bool IsNew { get; init; }
}
