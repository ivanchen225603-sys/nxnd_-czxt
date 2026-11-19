using System.Text.RegularExpressions;

namespace CarPointCloudWebApi.Application;

/// <summary>外部尺寸和煤量写入的单一校验入口，转换完成后交给原仓储事务。</summary>
internal sealed class MeasurementIngestionService(IIntegrationRepository repository, IConfiguration configuration)
{
    private readonly IIntegrationRepository _repository = repository;
    private readonly IConfiguration _configuration = configuration;
    private bool DatabaseEnabled => _configuration.GetValue<bool>("Database:Enabled", true);

    /// <summary>最大筋条数量</summary>
    private const int MaxRibCount = 50;
    /// <summary>最大钢丝绳数量</summary>
    private const int MaxWireCount = 50;
    // 这两个上限只约束本服务的 HTTP 入库端点 internal/pointcloud/*。
    // 当前部署下尺寸数据不走这条路：Analyzer 分析完直接写 DM8，Worker 的
    // EnableResultUploadToWebApi=false，从不调用该端点。真正生效的同名校验在
    // CarPointCloud_Analyzer_fixed/carpointcloud_analyzer/persistence.py
    // （_MAX_DISTANCE_CM / _MAX_TRAILER_LENGTH_CM）。
    // 因此改业务上限要改的是那一份；这里只是给仍用 HTTP 入库的客户端兜底。
    /// <summary>最大距离（厘米）</summary>
    private const decimal MaxDistanceCm = 1000m;
    /// <summary>最大挂车长度（厘米）；正常半挂车可能超过 10 米。</summary>
    private const decimal MaxTrailerLengthCm = 3000m;
    /// <summary>最大煤炭体积（立方米）</summary>
    /// <remarks>
    /// 与上面两个上限不同，这一个是真正生效的：煤量由第三方推送到
    /// internal/pointcloud/coalVolumes，本服务是其唯一入库校验点。
    /// </remarks>
    private const decimal MaxVolume = 1000m;

    /// <summary>摄像头标识符正则表达式：1-32位字母数字下划线</summary>
    private static readonly Regex CameraIdentifierRegex = new("^[A-Za-z0-9_]{1,32}$", RegexOptions.Compiled);
    /// <summary>幂等键正则表达式：1-64位字母数字下划线连字符冒号点</summary>
    private static readonly Regex IdempotencyKeyRegex = new("^[A-Za-z0-9_\\-:.]{1,64}$", RegexOptions.Compiled);
    /// <summary>车牌号正则表达式：排除控制字符和代理项，最多32位</summary>
    private static readonly Regex LicensePlateRegex = new("^[^\\p{Cc}\\p{Cs}]{1,32}$", RegexOptions.Compiled);

    /// <summary>
    /// 入库点云车辆尺寸数据。
    /// </summary>
    /// <remarks>
    /// 验证内容：
    /// - 幂等键、摄像头标识、车牌号格式验证
    /// - 检测状态标准化
    /// - 筋条和钢丝绳数量、序号、距离验证
    /// - 尺寸参数范围验证（0-1000cm）
    /// - 所有数值保留1位小数
    /// Database:Enabled=false 时返回0
    /// </remarks>
    /// <param name="request">点云尺寸入库请求</param>
    /// <param name="cancellationToken">取消并发幂等等待的请求令牌。</param>
    /// <returns>数据库记录ID，禁用时返回0</returns>
    public async Task<long> IngestPointCloudDimensionsAsync(
        PointCloudDimensionIngestRequest request,
        CancellationToken cancellationToken = default)
    {
        if (request is null)
        {
            throw new ArgumentException("请求体不能为空");
        }

        if (request.Dimensions is null)
        {
            throw new ArgumentException("dimensions 不能为空");
        }

        request.Ribs ??= new List<RibDto>();
        request.Wires ??= new List<WireDto>();
        request.HighLowPlates ??= new List<HighLowPlateDto>();

        request.IdempotencyKey = NormalizeIdempotencyKey(request.IdempotencyKey);
        var cameraIdentifier = NormalizeCameraIdentifier(request.CameraIdentifier);
        var workflowId = NormalizeOptionalText(request.WorkflowId, 128, "workflowId");
        var authoritativePlate = workflowId is null || !DatabaseEnabled
            ? null
            : _repository.GetSucceededWorkflowLicensePlate(workflowId);
        var licensePlate = NormalizeLicensePlate(authoritativePlate ?? request.LicensePlate);
        var detectStatus = NormalizeStatus(request.DetectStatus);
        var trailerLength = Round1(ValidateNullableDistance(request.Dimensions.TrailerLength, "trailerLength", MaxTrailerLengthCm));
        ValidateDetails(request.Ribs, "Ribs", MaxRibCount, trailerLength);
        ValidateDetails(request.Wires, "Wires", MaxWireCount, trailerLength);
        ValidateHighLowPlates(request.HighLowPlates);
        if (request.ActualRibCount is < 0 or > MaxRibCount || request.ActualRibCount > request.Ribs.Count)
        {
            throw new ArgumentException("actualRibCount 必须是明细内的实际拉筋数量");
        }

        if (!DatabaseEnabled)
        {
            return 0;
        }

        // 鉴权使用 UTC Unix 毫秒；无偏移业务时间统一按北京时间解释。
        var now = BusinessClock.Now;
        var record = new VehicleDimensionRecord
        {
            IdempotencyKey = request.IdempotencyKey,
            WorkflowId = workflowId,
            PairId = NormalizeOptionalText(request.PairId, 64, "pairId"),
            CameraIdentifier = cameraIdentifier,
            LicensePlate = licensePlate,
            GetTime = NormalizeInputTime(request.GetTime, now),
            DetectStatus = detectStatus,
            CabToTrailerFrontDist = Round1(ValidateNullableDistance(request.Dimensions.CabToTrailerFrontDist, "cabToTrailerFrontDist")),
            TrailerLength = trailerLength,
            TrailerWidth = Round1(ValidateNullableDistance(request.Dimensions.TrailerWidth, "trailerWidth")),
            MaximumWidthCm = Round1(ValidateNullableDistance(request.MaximumWidthCm, "maximumWidthCm")),
            TrailerBodyHeight = Round1(ValidateNullableDistance(request.Dimensions.TrailerBodyHeight, "trailerBodyHeight")),
            TrailerBottomToGroundHeight = Round1(ValidateNullableDistance(request.Dimensions.TrailerBottomToGroundHeight, "trailerBottomToGroundHeight")),
            HighLowPlateHeight = Round1(ValidateNullableDistance(request.Dimensions.HighLowPlateHeight, "highLowPlateHeight")),
            CreatedAt = now,
            UpdatedAt = now,
            SourceFile = NormalizeOptionalText(request.SourceFile, 512, "sourceFile"),
            ResultJsonPath = NormalizeOptionalText(request.ResultJsonPath, 512, "resultJsonPath"),
            ResultImagePath = NormalizeOptionalText(request.ResultImagePath, 512, "resultImagePath"),
            Ribs = request.Ribs.Select(x => new RibInfo { RibId = x.RibId, DistToPrev = Round1(x.DistToPrev) }).ToList(),
            Wires = request.Wires.Select(x => new WireInfo { WireId = x.WireId, DistToPrev = Round1(x.DistToPrev) }).ToList(),
            HighLowPlates = request.HighLowPlates.Select(x => new HighLowPlateInfo { PlateId = x.PlateId, HighestToGround = Round1(x.HighestToGround), LengthCm = Round1(x.LengthCm), LowestToGround = Round1(x.LowestToGround) }).ToList(),
            ActualRibCount = request.ActualRibCount
        };

        record.PayloadHash = PayloadFingerprint.Compute(new
        {
            record.WorkflowId,
            record.PairId,
            record.CameraIdentifier,
            LicensePlate = record.WorkflowId is null ? record.LicensePlate : null,
            GetTime = request.GetTime.HasValue ? record.GetTime : (DateTime?)null,
            record.DetectStatus,
            record.CabToTrailerFrontDist,
            record.TrailerLength,
            record.TrailerWidth,
            record.TrailerBodyHeight,
            record.TrailerBottomToGroundHeight,
            record.HighLowPlateHeight,
            record.SourceFile,
            Ribs = record.Ribs.OrderBy(x => x.RibId).Select(x => new { x.RibId, x.DistToPrev }),
            Wires = record.Wires.OrderBy(x => x.WireId).Select(x => new { x.WireId, x.DistToPrev }),
            HighLowPlates = record.HighLowPlates.OrderBy(x => x.PlateId)
                .Select(x => new { x.PlateId, x.HighestToGround, x.LowestToGround })
        });

        if (record.ActualRibCount.HasValue || record.MaximumWidthCm.HasValue || record.HighLowPlates.Any(x => x.LengthCm.HasValue))
        {
            record.PayloadHash = PayloadFingerprint.Compute(new
            {
                OriginalHash = record.PayloadHash,
                record.ActualRibCount,
                record.MaximumWidthCm,
                Lengths = record.HighLowPlates.OrderBy(x => x.PlateId).Select(x => new { x.PlateId, x.LengthCm })
            });
        }

        return await _repository.InsertVehicleDimensionRecordAsync(record, cancellationToken)
            .ConfigureAwait(false);
    }

    /// <summary>
    /// 入库煤炭体积数据。
    /// </summary>
    /// <remarks>
    /// 验证内容：
    /// - 体积范围 0-1000 m³
    /// - 置信度范围 0-1
    /// - 状态标准化
    /// Database:Enabled=false 时返回0
    /// </remarks>
    /// <param name="request">煤炭体积入库请求</param>
    /// <param name="cancellationToken">取消并发幂等等待的请求令牌。</param>
    /// <returns>数据库记录ID，禁用时返回0</returns>
    public async Task<long> IngestCoalVolumeAsync(
        PointCloudCoalVolumeIngestRequest request,
        CancellationToken cancellationToken = default)
    {
        if (request is null)
        {
            throw new ArgumentException("请求体不能为空");
        }

        request.IdempotencyKey = NormalizeIdempotencyKey(request.IdempotencyKey);
        var cameraIdentifier = NormalizeCameraIdentifier(request.CameraIdentifier);
        // 整体重启切换写入所有权；已接管点位不得同时走旧合作方煤量入口。
        var loadRadar = _configuration.GetSection("LoadRadar").Get<LoadRadarOptions>();
        if (loadRadar?.Enabled == true && loadRadar.Nodes.SelectMany(n => n.Channels).Any(c =>
                string.Equals(c.CameraIdentifier.Replace("_", ""), cameraIdentifier.Replace("_", ""), StringComparison.OrdinalIgnoreCase)))
            throw new LoadRadarException(409, "该点位煤量已由载货雷达后台接管，请使用测量审核入库流程");
        var licensePlate = NormalizeLicensePlate(request.LicensePlate);
        if (string.IsNullOrWhiteSpace(request.Unit))
        {
            throw new ArgumentException("unit 不能为空");
        }

        var unit = NormalizeOptionalText(request.Unit, 16, "unit")!;
        if (string.IsNullOrWhiteSpace(request.Status))
        {
            throw new ArgumentException("status 不能为空");
        }

        var status = NormalizeStatus(request.Status);

        if (request.Volume < 0 || request.Volume > MaxVolume)
        {
            throw new ArgumentException($"volume 必须在 0-{MaxVolume} m³ 范围内");
        }

        if (request.Confidence is < 0 or > 1)
        {
            throw new ArgumentException("confidence 必须在 0-1 范围内");
        }

        if (request.MeasureEndTime.HasValue && (!request.MeasureTime.HasValue ||
            NormalizeInputTime(request.MeasureEndTime, BusinessClock.Now) <
            NormalizeInputTime(request.MeasureTime, BusinessClock.Now)))
        {
            throw new ArgumentException("measureEndTime 必须与 measureTime 配对且不早于开始时间");
        }

        if (!DatabaseEnabled)
        {
            return 0;
        }

        var record = new CoalVolumeRecord
        {
            IdempotencyKey = request.IdempotencyKey,
            CameraIdentifier = cameraIdentifier,
            LicensePlate = licensePlate,
            MeasureTime = NormalizeInputTime(request.MeasureTime, BusinessClock.Now),
            MeasurementStartedAt = request.MeasureEndTime.HasValue
                ? NormalizeInputTime(request.MeasureTime, BusinessClock.Now) : null,
            MeasurementEndedAt = request.MeasureEndTime.HasValue
                ? NormalizeInputTime(request.MeasureEndTime, BusinessClock.Now) : null,
            Volume = Round1(request.Volume),
            Unit = unit,
            Status = status,
            // 修复：置信度为 0-1 的高精度值，不能按 1 位小数四舍五入（原代码会把 0.92 变成 0.9、0.96 变成 1.0），
            // 改为保留 2 位小数，既能去除浮点噪声又能保留有效精度。
            Confidence = Round2(request.Confidence),
            SourceFile = NormalizeOptionalText(request.SourceFile, 512, "sourceFile"),
            BaselineFile = NormalizeOptionalText(request.BaselineFile, 512, "baselineFile"),
            CreatedAt = BusinessClock.Now
        };

        object businessIdentity;
        if (record.SourceFile is not null)
        {
            businessIdentity = new
            {
                Kind = "coal-volume-source",
                record.CameraIdentifier,
                record.LicensePlate,
                SourceFile = record.SourceFile
            };
        }
        else if (request.MeasureTime.HasValue)
        {
            businessIdentity = new
            {
                Kind = "coal-volume-time",
                record.CameraIdentifier,
                record.LicensePlate,
                MeasureTime = record.MeasureTime
            };
        }
        else
        {
            businessIdentity = new
            {
                Kind = "coal-volume-idempotency-fallback",
                record.CameraIdentifier,
                record.LicensePlate,
                record.IdempotencyKey
            };
        }
        record.BusinessKey = PayloadFingerprint.Compute(businessIdentity);
        record.WorkflowId = string.Equals(record.LicensePlate, "UNKNOWN", StringComparison.OrdinalIgnoreCase)
            ? null
            : _repository.FindWorkflowIdForMeasurement(
                record.LicensePlate,
                record.CameraIdentifier,
                record.MeasureTime);

        record.PayloadHash = PayloadFingerprint.Compute(new
        {
            record.CameraIdentifier,
            record.LicensePlate,
            MeasureTime = request.MeasureTime.HasValue ? record.MeasureTime : (DateTime?)null,
            record.Volume,
            record.Unit,
            record.Status,
            record.Confidence,
            record.SourceFile,
            record.BaselineFile,
            record.LicensePlateRecordId
        });

        // 旧调用方不传结束时间时保留原指纹，避免同键历史重试冲突。
        if (record.MeasurementEndedAt.HasValue)
        {
            record.PayloadHash = PayloadFingerprint.Compute(new
            {
                OriginalHash = record.PayloadHash,
                record.MeasurementEndedAt
            });
        }

        return await _repository.InsertCoalVolumeRecordAsync(record, cancellationToken)
            .ConfigureAwait(false);
    }

    /// <summary>
    /// 规范化幂等键。
    /// </summary>
    /// <param name="raw">原始幂等键</param>
    /// <returns>规范化后的幂等键</returns>
    private static string NormalizeIdempotencyKey(string? raw)
    {
        raw = raw?.Trim();
        if (string.IsNullOrWhiteSpace(raw))
        {
            throw new ArgumentException("idempotencyKey 不能为空");
        }

        if (!IdempotencyKeyRegex.IsMatch(raw))
        {
            throw new ArgumentException("idempotencyKey 格式非法，要求 1-64 位字母、数字、下划线、连字符、冒号或点");
        }

        return raw;
    }

    /// <summary>
    /// 规范化摄像头标识符。
    /// </summary>
    /// <param name="raw">原始摄像头标识符</param>
    /// <returns>规范化后的摄像头标识符</returns>
    private static string NormalizeCameraIdentifier(string? raw)
    {
        raw = raw?.Trim();
        if (string.IsNullOrWhiteSpace(raw))
        {
            throw new ArgumentException("cameraIdentifier 不能为空");
        }

        if (!CameraIdentifierRegex.IsMatch(raw))
        {
            throw new ArgumentException("cameraIdentifier 格式非法，要求 1-32 位字母、数字或下划线");
        }

        return raw;
    }

    /// <summary>
    /// 规范化车牌号。
    /// </summary>
    /// <remarks>
    /// 空值时返回"UNKNOWN"
    /// 自动转为大写
    /// </remarks>
    /// <param name="raw">原始车牌号</param>
    /// <returns>规范化后的车牌号</returns>
    private static string NormalizeLicensePlate(string? raw)
    {
        if (string.IsNullOrWhiteSpace(raw))
        {
            return "UNKNOWN";
        }

        raw = raw.Trim().ToUpperInvariant();
        if (!LicensePlateRegex.IsMatch(raw))
        {
            throw new ArgumentException("licensePlate 格式非法或超长");
        }

        return raw;
    }

    /// <summary>
    /// 规范化检测状态。
    /// </summary>
    /// <remarks>
    /// 支持的状态值：
    /// - success（默认）
    /// - failed / fail
    /// - lowconfidence / low_confidence
    /// - unknown
    /// </remarks>
    /// <param name="raw">原始状态</param>
    /// <returns>规范化后的状态</returns>
    private static string NormalizeStatus(string? raw)
    {
        return raw?.Trim().ToLowerInvariant() switch
        {
            null or "" or "success" => "success",
            "failed" or "fail" => "failed",
            "lowconfidence" or "low_confidence" => "low_confidence",
            "unknown" => "unknown",
            _ => throw new ArgumentException($"未知状态: {raw}")
        };
    }

    /// <summary>
    /// 规范化可选文本字段。
    /// </summary>
    /// <param name="raw">原始文本</param>
    /// <param name="maxLength">最大长度</param>
    /// <param name="fieldName">字段名（用于错误信息）</param>
    /// <returns>规范化后的文本，空值返回null</returns>
    private static string? NormalizeOptionalText(string? raw, int maxLength, string fieldName)
    {
        if (string.IsNullOrWhiteSpace(raw))
        {
            return null;
        }

        raw = raw.Trim();
        if (raw.Length > maxLength)
        {
            throw new ArgumentException($"{fieldName} 长度不能超过 {maxLength}");
        }

        if (raw.Any(char.IsControl))
        {
            throw new ArgumentException($"{fieldName} 不能包含控制字符");
        }

        return raw;
    }

    /// <summary>
    /// 规范化输入时间。
    /// </summary>
    /// <param name="value">输入时间</param>
    /// <param name="fallbackLocal">回退值（本地时间）</param>
    /// <returns>规范化后的时间</returns>
    private static DateTime NormalizeInputTime(DateTime? value, DateTime fallbackLocal)
    {
        if (!value.HasValue)
        {
            return fallbackLocal;
        }

        return value.Value;
    }

    /// <summary>
    /// 验证可空距离值。
    /// </summary>
    /// <param name="value">距离值</param>
    /// <param name="fieldName">字段名</param>
    /// <param name="maximum">该字段允许的最大厘米值。</param>
    /// <returns>验证后的距离值</returns>
    private static decimal? ValidateNullableDistance(
        decimal? value,
        string fieldName,
        decimal maximum = MaxDistanceCm)
    {
        if (value.HasValue && (value.Value < 0 || value.Value > maximum))
        {
            throw new ArgumentException($"{fieldName} 必须在 0-{maximum} cm 范围内");
        }

        return value;
    }

    /// <summary>
    /// 验证筋条/钢丝绳明细。
    /// </summary>
    /// <remarks>
    /// 验证内容：
    /// - 数量不超过maxCount
    /// - 序号不重复
    /// - 序号范围 0-999
    /// - 纵向距离范围 0-3000cm；已知分段总和不得超过车厢内长
    /// </remarks>
    /// <param name="items">明细列表</param>
    /// <param name="fieldName">字段名</param>
    /// <param name="maxCount">最大数量</param>
    /// <param name="trailerLength">量化后的内长；缺失时仅使用 3000cm 硬上限。</param>
    private static void ValidateDetails<T>(List<T> items, string fieldName, int maxCount, decimal? trailerLength) where T : class
    {
        if (items.Count > maxCount)
        {
            throw new ArgumentException($"{fieldName} 数量不能超过 {maxCount}");
        }

        var details = items.Select(item => item switch
        {
            RibDto rib => (Id: rib.RibId, Dist: rib.DistToPrev),
            WireDto wire => (Id: wire.WireId, Dist: wire.DistToPrev),
            _ => throw new ArgumentException($"{fieldName} 类型非法")
        }).ToList();

        if (details.Select(x => x.Id).Distinct().Count() != details.Count)
        {
            throw new ArgumentException($"{fieldName} 存在重复序号");
        }

        if (details.Any(x => x.Id is < 0 or > 999 || x.Dist is < 0 or > MaxTrailerLengthCm))
        {
            throw new ArgumentException($"{fieldName} 含非法值，序号范围 0-999，距离范围 0-{MaxTrailerLengthCm} cm");
        }
        // 与 Analyzer 保持一致：比较实际入库的一位小数，兼容部分明细和 null，不伪造完整分段。
        var maximum = trailerLength ?? MaxTrailerLengthCm;
        if (details.Sum(x => Round1(x.Dist) ?? 0m) > maximum)
            throw new ArgumentException($"{fieldName} distToPrev 已知分段总和不能超过车厢长度 {maximum} cm");
    }

    /// <summary>
    /// 验证高低板明细。
    /// </summary>
    /// <remarks>
    /// 验证内容：
    /// - 数量不超过 50
    /// - plateId 不重复
    /// - plateId 范围 1-999
    /// - highestToGround / lowestToGround 范围 0-1000cm
    /// </remarks>
    /// <param name="plates">高低板列表</param>
    private static void ValidateHighLowPlates(List<HighLowPlateDto> plates)
    {
        if (plates.Count > 50)
        {
            throw new ArgumentException("HighLowPlates 数量不能超过 50");
        }

        if (plates.Select(x => x.PlateId).Distinct().Count() != plates.Count)
        {
            throw new ArgumentException("HighLowPlates 存在重复 plateId");
        }

        if (plates.Any(x => x.PlateId is < 1 or > 999))
        {
            throw new ArgumentException("HighLowPlates plateId 范围 1-999");
        }

        if (plates.Any(x => x.HighestToGround is < 0 or > MaxDistanceCm || x.LowestToGround is < 0 or > MaxDistanceCm ||
            x.LengthCm is < 0 or > MaxTrailerLengthCm))
        {
            throw new ArgumentException($"HighLowPlates 含非法值，高度范围 0-{MaxDistanceCm} cm，长度范围 0-{MaxTrailerLengthCm} cm");
        }
    }

    /// <summary>
    /// 四舍五入保留1位小数（可空版本）。
    /// </summary>
    /// <param name="value">数值</param>
    /// <returns>四舍五入后的数值</returns>
    private static decimal? Round1(decimal? value)
    {
        return value.HasValue ? Round1(value.Value) : null;
    }

    /// <summary>
    /// 四舍五入保留1位小数。
    /// </summary>
    /// <remarks>使用银行家舍入（ToEven）</remarks>
    /// <param name="value">数值</param>
    /// <returns>四舍五入后的数值</returns>
    private static decimal Round1(decimal value)
    {
        return decimal.Round(value, 1, MidpointRounding.ToEven);
    }

    /// <summary>
    /// 四舍五入保留2位小数（可空版本）。
    /// </summary>
    /// <remarks>
    /// 用于置信度等 0-1 高精度字段，避免按 1 位小数舍入造成精度丢失。
    /// </remarks>
    /// <param name="value">数值</param>
    /// <returns>四舍五入后的数值，空值返回 null。</returns>
    private static decimal? Round2(decimal? value)
    {
        return value.HasValue ? decimal.Round(value.Value, 2, MidpointRounding.ToEven) : null;
    }
}
