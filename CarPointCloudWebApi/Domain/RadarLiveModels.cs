using System.ComponentModel.DataAnnotations;
using System.Text.Json.Nodes;

namespace CarPointCloudWebApi.Domain;

/// <summary>磅房页面发起一次"保存 PCD 并计算"的请求。</summary>
/// <remarks>
/// 车牌由操作员在页面输入：本次采集不经过乙方车牌接口，因此不会占用车牌配额，
/// 也不存在"分析已落库但车牌还没绑定好"的竞态。代价是车牌正确性完全由操作员负责，
/// 所以服务端的校验必须比合作方入库接口更严（见 <c>RadarLiveController</c>）。
/// </remarks>
public sealed class ManualCaptureRequest
{
    /// <summary>本次过车的车牌号，由操作员输入。</summary>
    [Required(ErrorMessage = "licensePlate 不能为空")]
    public string? LicensePlate { get; set; }

    /// <summary>
    /// 操作员确认已经知晓该车牌存在空车档案、并同意覆盖。
    /// </summary>
    /// <remarks>
    /// 空车档案按车牌唯一，较新扫描会顶替较旧的。车牌打错一位就会覆盖另一辆车的永久档案，
    /// 而这种错误事后无法从档案本身看出来，因此必须由操作员显式确认一次。
    /// </remarks>
    public bool ConfirmOverwriteEmptyProfile { get; set; }

    /// <summary>
    /// 客户端幂等键，也可用 <c>Idempotency-Key</c> 请求头传递；两者同时给出时必须一致。
    /// </summary>
    /// <remarks>
    /// 页面必须提供。请求超时时客户端无法判断 Worker 是否已经受理，
    /// 不带幂等键重试会再采一次、再写一条来车与尺寸记录。
    /// </remarks>
    public string? IdempotencyKey { get; set; }

    /// <summary>操作员补充的备注，仅写入审计日志。</summary>
    public string? Note { get; set; }
}

/// <summary>发起手动采集后返回给页面的受理结果。</summary>
public sealed class ManualCaptureResponse
{
    /// <summary>本次采集的流程编号，贯穿 WebApi → Worker → Analyzer → DM8。</summary>
    public string WorkflowId { get; set; } = string.Empty;

    /// <summary>已绑定到本次流程的车牌（已规范化为大写、去空白）。</summary>
    public string LicensePlate { get; set; } = string.Empty;

    /// <summary>
    /// 本次是否为同一幂等键的重放。
    /// </summary>
    /// <remarks>
    /// 为 <see langword="true"/> 时没有新建来车记录、也没有再次触发采集，
    /// 返回的是原流程。页面据此提示"已在进行中"而不是"又发起了一次"。
    /// </remarks>
    public bool Duplicate { get; set; }

    /// <summary>Worker 对本次扫描指令的受理状态。</summary>
    public string Status { get; set; } = string.Empty;

    /// <summary>面向操作员的中文说明。</summary>
    public string Message { get; set; } = string.Empty;

    /// <summary>Worker 返回的扫描会话快照原文。</summary>
    public JsonNode? Scan { get; set; }
}

/// <summary>一次手动采集在页面上的完整进度与结果。</summary>
/// <remarks>
/// 三个来源合成，且<b>不能互相替代</b>：
/// <list type="bullet">
/// <item>采集与落盘来自 Worker 的 scan-once 快照；</item>
/// <item>融合质量与空载判定来自 Worker 留存的分析结果（DM8 不保存这两项）；</item>
/// <item>已入库尺寸来自 DM8；未入库的本次计算值单独标注，不冒充数据库记录。</item>
/// </list>
/// </remarks>
public sealed class CaptureProgressResponse
{
    /// <summary>属于本流程的永久融合空车 PCD；没有有效档案时为空。</summary>
    public string? EmptyVehiclePointCloudUrl { get; set; }
    /// <summary>流程编号。</summary>
    public string WorkflowId { get; set; } = string.Empty;

    /// <summary>合成后的总体阶段，取值见 <see cref="CaptureStages"/>。</summary>
    public string Stage { get; set; } = CaptureStages.Unknown;

    /// <summary>面向操作员的中文阶段说明。</summary>
    public string Message { get; set; } = string.Empty;

    /// <summary>是否已经走到终局（成功或失败），页面据此停止轮询。</summary>
    public bool Terminal { get; set; }

    /// <summary>本次流程绑定的车牌。</summary>
    public string? LicensePlate { get; set; }

    /// <summary>已捕获点云的雷达点位。</summary>
    public IReadOnlyList<string> CapturedCameras { get; set; } = [];

    /// <summary>已确认 PCD 落盘的雷达点位。</summary>
    public IReadOnlyList<string> PersistedCameras { get; set; } = [];

    /// <summary>Worker 的落盘状态：pending / saved / failed。</summary>
    public string? PersistenceState { get; set; }

    /// <summary>失败原因；成功时为空。</summary>
    public string? Error { get; set; }

    /// <summary>Worker 扫描会话快照原文，供排障查看全部字段。</summary>
    public JsonNode? Scan { get; set; }

    /// <summary>Worker 留存的分析记录原文，含融合质量与空载判定。</summary>
    public JsonNode? Analysis { get; set; }

    /// <summary>DM8 中的权威尺寸记录；尚未落库时为空。</summary>
    public VehicleDimensionDto? Dimensions { get; set; }
}

/// <summary>手动采集在页面上呈现的阶段常量。</summary>
/// <remarks>
/// 阶段刻意区分 <see cref="Captured"/>、<see cref="Saved"/>、<see cref="Analyzing"/> 和
/// <see cref="Completed"/>：把"收到点云"当成"已落盘"，或把"分析返回成功"当成"已入库"，
/// 都会让现场以为数据已经安全，而实际上可能什么都没留下。
/// </remarks>
public static class CaptureStages
{
    /// <summary>无法确定当前阶段，通常是 Worker 已经开始了另一次会话。</summary>
    public const string Unknown = "unknown";

    /// <summary>Worker 已受理指令，正在等待安全的完整场。</summary>
    public const string Waiting = "waiting";

    /// <summary>已收到点云，尚未确认落盘。</summary>
    public const string Captured = "captured";

    /// <summary>两侧 PCD 均已确认落盘。</summary>
    public const string Saved = "saved";

    /// <summary>已配对并排队等待分析。</summary>
    public const string Queued = "queued";

    /// <summary>Analyzer 正在融合与计算。</summary>
    public const string Analyzing = "analyzing";

    /// <summary>已完成并在 DM8 中查到尺寸记录。</summary>
    public const string Completed = "completed";

    /// <summary>计算已经结束，展示本次计算值，但数据库中尚未查到记录。</summary>
    public const string CompletedUnpersisted = "completed_unpersisted";

    /// <summary>本次任务状态不可获取，结束自动查询，可按原流程手动刷新。</summary>
    public const string ProgressUnavailable = "progress_unavailable";

    /// <summary>计算值已返回供人工审核，自动计算结束，但不表示尺寸已入库。</summary>
    public const string PendingReview = "pending_review";

    /// <summary>
    /// 分析正常结束，但按算法结论本次不产生车辆尺寸记录。
    /// </summary>
    /// <remarks>
    /// 载货、载荷状态不确定或端壁内表面未通过时，Analyzer 成功返回却<b>不写</b>
    /// <c>VehicleDimensionRecord</c>。这是终局，不是中间态：把它当成"还在计算"
    /// 会让页面永远等一条不会出现的记录。
    /// </remarks>
    public const string CompletedWithoutDimensions = "completed_without_dimensions";

    /// <summary>采集阶段失败（超时、安全门禁、雷达错误）。</summary>
    public const string CaptureFailed = "capture_failed";


    /// <summary>分析或入库失败。</summary>
    public const string AnalysisFailed = "analysis_failed";
}
