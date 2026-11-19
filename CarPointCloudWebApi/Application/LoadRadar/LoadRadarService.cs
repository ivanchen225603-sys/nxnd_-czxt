using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;
using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Application;

/// <summary>载货雷达业务边界；不触发双雷达扫描，不按最近车牌猜测流程。</summary>
public sealed class LoadRadarService(ILoadRadarRepository repository, IIntegrationRepository integration,
    IOptions<LoadRadarOptions> options, LoadRadarAssetStore assets)
{
    public LoadRadarOptions Options => options.Value;

    public IReadOnlyList<LoadRecalculationTarget> Recalculations(string nodeId, string channelId, string? afterId, DateOnly? fromDate)
    {
        Lane(nodeId, channelId);
        if (channelId is not ("CH1" or "CH2" or "CH3" or "CH4"))
            throw new LoadRadarException(400, "复算只允许CH1至CH4");
        if (afterId is not null) Identifier(afterId, "afterId");
        if (fromDate is { Year: < 2020 }) throw new LoadRadarException(400, "复算日期不能早于2020年");
        return repository.GetRecalculations(nodeId, channelId, afterId, fromDate ?? DateOnly.FromDateTime(BusinessClock.Now));
    }

    public LoadRecalculationReceipt Revise(string nodeId, LoadRecalculationInput request)
    {
        Identifier(request.Id, "revisionId");
        Identifier(request.MeasurementId, "measurementId");
        Identifier(request.BaselineVersionId, "baselineVersionId");
        if (request.ActivationId <= 0) throw new LoadRadarException(400, "缺少基线激活事件");
        if (request.Id != Key("volume-revision", request.MeasurementId,
                request.ActivationId.ToString(System.Globalization.CultureInfo.InvariantCulture)))
            throw new LoadRadarException(400, "复算编号与原测量、激活事件不一致");
        ValidateInput(request.Measurement);
        foreach (var hash in new[] { request.Provenance.InputSha256, request.Provenance.OriginalAlgorithmSha256,
            request.Provenance.CalculationAlgorithmSha256, request.Provenance.ExitFramesSha256 })
            if (!Regex.IsMatch(hash ?? "", "\\A[a-f0-9]{64}\\z"))
                throw new LoadRadarException(400, "复算必须记录原输入、原算法及本次算法的SHA-256");
        var row = Get(request.MeasurementId);
        var original = row.Measurement;
        if (original.NodeId != nodeId) throw new LoadRadarException(403, "不能复算其他节点测量");
        var result = request.Measurement;
        // 只允许改变计算输出，车次、原采集时间及原采集配置仍必须逐项相同。
        if (result.MeasurementId != original.MeasurementId || result.NodeId != original.NodeId ||
            result.ChannelId != original.ChannelId || result.PassageId != original.PassageId ||
            result.CalibrationId != original.CalibrationId || result.ConfigurationHash != original.ConfigurationHash ||
            result.FrameDegraded != original.FrameDegraded || result.PointCount != original.PointCount ||
            result.AlgorithmVersion != original.AlgorithmVersion ||
            result.MeasuredAt != original.MeasuredAt || result.SourceMode != original.SourceMode ||
            result.BaselineVersionId != request.BaselineVersionId)
            throw new LoadRadarException(409, "复算不得改变原车次、观测时间或采集配置");
        // 首测无基线时未使用进/离场帧；复算实际采用原留样中的补充帧，计算窗口可扩大。
        if (result.MeasurementStartedAt is null || result.MeasurementEndedAt is null ||
            original.MeasurementStartedAt is { } begin && result.MeasurementStartedAt > begin ||
            original.MeasurementEndedAt is { } end && result.MeasurementEndedAt < end)
            throw new LoadRadarException(409, "复算必须保留原观测窗口，不能缩小实际采集范围");
        assets.Verify(Baseline(request.BaselineVersionId).AssetSha256);
        return repository.ApplyRecalculation(nodeId, request, Key(JsonSerializer.Serialize(request)));
    }

    public IReadOnlyList<LoadVolumeRevision> Revisions(string id)
    {
        Get(id);
        return repository.GetVolumeRevisions(id);
    }

    public void EnsureEnabled()
    {
        if (!Options.Enabled) throw new LoadRadarException(503, "载货雷达模块未启用");
    }

    public void ActivateAutomaticBaseline(string id)
    {
        var row = Get(id);
        if (row.Binding is null) throw new LoadRadarException(503, "空车原流程车牌尚未就绪，保留候选稍后重试");
        var version = CreateBaseline(new(id, row.Measurement.BaselineCandidateSha256!, true,
            "CH4完整实车空车自动建立基线"), "system:load-radar");
        if (version.ActivatedAt is not null) return;
        repository.ActivateAutomaticBaseline(version.VersionId, "system:load-radar", "CH4空车基线生效并重算同牌当日载重");
    }

    public LoadRadarLaneOptions Lane(string nodeId, string channelId)
    {
        EnsureEnabled();
        return Options.Nodes.SingleOrDefault(n => n.NodeId == nodeId)?.Channels
            .SingleOrDefault(c => c.ChannelId == channelId)
            ?? throw new LoadRadarException(400, "节点或通道未登记");
    }

    public LoadMeasurementRecord Ingest(string authenticatedNode, LoadMeasurementInput input)
    {
        if (input.NodeId != authenticatedNode) throw new LoadRadarException(403, "节点身份与测量不符");
        Lane(input.NodeId, input.ChannelId);
        ValidateInput(input);
        // 相同时刻的时区表达先归一；所有字段均由强类型序列化固定顺序。
        input = input with
        {
            MeasuredAt = input.MeasuredAt.ToUniversalTime(),
            MeasurementStartedAt = input.MeasurementStartedAt?.ToUniversalTime(),
            MeasurementEndedAt = input.MeasurementEndedAt?.ToUniversalTime()
        };
        return repository.SaveMeasurement(new LoadMeasurementRecord(
            Key(input.NodeId, input.ChannelId, input.PassageId), Key(input), input, DateTimeOffset.UtcNow));
    }

    public LoadMeasurementRecord Get(string id)
    {
        EnsureEnabled();
        Identifier(id, "measurementId");
        return repository.GetMeasurement(id) ?? throw new LoadRadarException(404, "测量不存在");
    }

    public IReadOnlyList<LoadMeasurementRecord> History(string? nodeId, string? channelId,
        string? licensePlate, string? beforeId, int take, DateTimeOffset? from = null, DateTimeOffset? to = null)
    {
        EnsureEnabled();
        if (take is < 1 or > 100) throw new LoadRadarException(400, "take 必须在 1-100 内");
        if (nodeId is not null) Identifier(nodeId, "nodeId");
        if (channelId is not null) Identifier(channelId, "channelId");
        if (beforeId is not null) Identifier(beforeId, "beforeId");
        if (licensePlate is not null) Text(licensePlate, "licensePlate", 32);
        if (from > to) throw new LoadRadarException(400, "开始时间不能晚于结束时间");
        // 列表不重复运输大点云；选中测量后通过详情接口按需读取。
        return repository.GetHistory(nodeId, channelId, licensePlate, beforeId, take, from, to)
            .Select(row => row with
            {
                Measurement = row.Measurement with { Points = [] },
                LatestRevision = row.LatestRevision is { } revision
                    ? revision with { Calculation = revision.Calculation with { Points = [] } } : null
            }).ToArray();
    }

    public LoadPassageBinding Bind(string id, LoadBindRequest request, string actor)
    {
        var row = Get(id);
        Text(request.WorkflowId, "workflowId", 128);
        Text(request.VehicleIdentity, "vehicleIdentity（含挂车身份）", 128);
        Text(request.Reason, "关联理由", 500);
        var lane = Lane(row.Measurement.NodeId, row.Measurement.ChannelId);
        var workflow = integration.GetVehicleArrivalBySessionId(request.WorkflowId)
            ?? throw new LoadRadarException(404, "过车流程不存在");
        var plate = integration.GetSucceededWorkflowLicensePlate(request.WorkflowId);
        if (string.IsNullOrWhiteSpace(plate) || plate.Equals("UNKNOWN", StringComparison.OrdinalIgnoreCase))
            throw new LoadRadarException(409, "该过车流程尚未确认车牌");
        if (!string.Equals(workflow.CameraIdentifier, lane.CameraIdentifier, StringComparison.Ordinal))
            throw new LoadRadarException(409, "过车点位与登记通道不一致");
        return repository.BindPassage(row, new LoadPassageBinding(row.PassageKey,
            request.WorkflowId, plate, lane.CameraIdentifier, request.VehicleIdentity, actor,
            request.Reason, DateTimeOffset.UtcNow));
    }

    public LoadBaselineVersion CreateBaseline(LoadBaselineCreateRequest request, string actor)
    {
        var row = Get(request.MeasurementId);
        var measurement = row.Measurement;
        var binding = row.Binding ?? throw new LoadRadarException(409, "请先关联流程并确认车辆及挂车身份");
        if (measurement.ChannelId != "CH4") throw new LoadRadarException(409, "正式空车基线只能在CH4建立");
        if (!request.ConfirmedEmpty || measurement.Verdict != "empty" || measurement.FrameDegraded || measurement.SourceMode != "live")
            throw new LoadRadarException(409, "基线必须来自人工确认的完整实车空车测量");
        Text(request.Reason, "基线确认理由", 500);
        if (request.AssetSha256 != measurement.BaselineCandidateSha256)
            throw new LoadRadarException(409, "该资产不是此测量声明的空车基线候选，禁止串用其他车辆文件");
        assets.Verify(request.AssetSha256);
        var id = Key("baseline", measurement.MeasurementId, request.AssetSha256);
        var original = repository.GetBaseline(id);
        var version = repository.CreateBaseline(new LoadBaselineVersion(id,
            original?.ScopeKey ?? Key("ch4-plate", measurement.NodeId, binding.LicensePlate),
            measurement.NodeId, measurement.ChannelId, measurement.CalibrationId, binding.VehicleIdentity,
            binding.LicensePlate, measurement.MeasurementId, request.AssetSha256, actor, request.Reason,
            DateTimeOffset.UtcNow));
        return version with { Active = repository.GetPlateBaselineAt(version.NodeId, version.LicensePlate, DateTimeOffset.UtcNow)?.VersionId == id };
    }

    public IReadOnlyList<LoadBaselineVersion> Baselines()
    {
        EnsureEnabled();
        var versions = repository.GetBaselines();
        var current = versions.Where(v => v.ChannelId == "CH4").GroupBy(v => (v.NodeId, v.LicensePlate))
            .ToDictionary(g => g.Key, g => repository.GetPlateBaselineAt(g.Key.NodeId, g.Key.LicensePlate, DateTimeOffset.UtcNow)?.VersionId);
        return versions.Select(v => v with { Active = current.GetValueOrDefault((v.NodeId, v.LicensePlate)) == v.VersionId }).ToArray();
    }

    public LoadBaselineVersion Baseline(string id)
    {
        EnsureEnabled();
        Identifier(id, "versionId");
        var version = repository.GetBaseline(id) ?? throw new LoadRadarException(404, "基线版本不存在");
        return version with { Active = repository.GetPlateBaselineAt(version.NodeId, version.LicensePlate, DateTimeOffset.UtcNow)?.VersionId == id };
    }

    public LoadBaselineVersion Activate(string id, LoadBaselineActivateRequest request, string actor)
    {
        var baseline = Baseline(id);
        if (baseline.ChannelId != "CH4") throw new LoadRadarException(409, "只允许激活CH4空车基线");
        Text(request.Reason, "激活理由", 500);
        if (request.ExpectedVersionId is not null) Identifier(request.ExpectedVersionId, "expectedVersionId");
        assets.Verify(baseline.AssetSha256);
        return repository.ActivateBaseline(id, request.ExpectedVersionId, actor, request.Reason);
    }

    public LoadCoalPublication Publish(string id, LoadPublishRequest request, string actor)
    {
        var row = Get(id);
        Text(request.Reason, "入库理由", 500);
        // 已提交结果的重放不受文件后续迁移影响；不允许换测量覆盖。
        if (row.Publication is { } published)
        {
            if (published.MeasurementId != id) throw new LoadRadarException(409, "同次过车已有其他测量入库");
            return published;
        }
        var measurement = row.Measurement;
        var binding = row.Binding ?? throw new LoadRadarException(409, "尚未绑定权威过车流程");
        // 门槛统一走 LoadRadarPublicationPolicy，不在这里再写一份数字。
        if (!LoadRadarPublicationPolicy.IsCandidate(measurement, Options.VolumeBoundSpreadMax))
            throw new LoadRadarException(409, "仅允许完整实车载货测量、有效基线和成功体积发布煤量");
        var baseline = Baseline(measurement.BaselineVersionId!);
        if (repository.GetPlateBaselineAt(measurement.NodeId, binding.LicensePlate, measurement.MeasuredAt)?.VersionId != baseline.VersionId)
            throw new LoadRadarException(409, "体积未使用原测量时刻激活的基线版本");
        if (!LoadRadarPublicationPolicy.MatchesBaseline(measurement, binding, baseline, Get(baseline.MeasurementId).Measurement))
            throw new LoadRadarException(409, "基线未激活或车辆、挂车、通道、标定不匹配");
        assets.Verify(baseline.AssetSha256);
        var coal = new CoalVolumeRecord
        {
            IdempotencyKey = Key("load-radar", row.PassageKey),
            BusinessKey = Key("load-radar-workflow", binding.WorkflowId),
            WorkflowId = binding.WorkflowId,
            PayloadHash = Key(row.PayloadHash, binding.WorkflowId, baseline.VersionId),
            LicensePlate = binding.LicensePlate,
            CameraIdentifier = binding.CameraIdentifier,
            Volume = measurement.VolumeM3!.Value.RoundCoalVolume(),
            MeasureTime = measurement.MeasuredAt.ToOffset(TimeSpan.FromHours(8)).DateTime,
            MeasurementStartedAt = measurement.MeasurementStartedAt?.ToOffset(TimeSpan.FromHours(8)).DateTime,
            MeasurementEndedAt = measurement.MeasurementEndedAt?.ToOffset(TimeSpan.FromHours(8)).DateTime,
            Status = "success",
            // 覆盖率/上下界差不是概率置信度，正式表保持 NULL。
            Confidence = null,
            SourceFile = "load-radar:measurement:" + id,
            BaselineFile = "load-radar:baseline:" + baseline.VersionId,
            CreatedAt = BusinessClock.Now
        };
        return repository.Publish(row, coal, actor, request.Reason);
    }

    public static void ValidateInput(LoadMeasurementInput input)
    {
        Identifier(input.MeasurementId, "measurementId");
        Identifier(input.NodeId, "nodeId");
        Identifier(input.ChannelId, "channelId");
        Identifier(input.PassageId, "passageId");
        Identifier(input.CalibrationId, "calibrationId");
        Text(input.AlgorithmVersion, "algorithmVersion", 64);
        if (!Regex.IsMatch(input.ConfigurationHash ?? "", "\\A[a-f0-9]{64}\\z"))
            throw new LoadRadarException(400, "configurationHash 必须为完整运行配置的 SHA-256");
        if (input.BaselineCandidateSha256 is not null && !Regex.IsMatch(input.BaselineCandidateSha256, "\\A[a-f0-9]{64}\\z"))
            throw new LoadRadarException(400, "baselineCandidateSha256 格式错误");
        if (input.SourceMode is not ("live" or "demo" or "replay")) throw new LoadRadarException(400, "sourceMode 必须显式声明 live/demo/replay");
        if (input.MeasuredAt.Year < 2020 || input.MeasuredAt > DateTimeOffset.UtcNow.AddMinutes(5))
            throw new LoadRadarException(400, "测量时间缺失或超出允许时钟偏差");
        if (input.MeasurementStartedAt.HasValue != input.MeasurementEndedAt.HasValue ||
            input.MeasurementStartedAt is { } start && input.MeasurementEndedAt is { } end &&
            (start.Year < 2020 || end < start || end > DateTimeOffset.UtcNow.AddMinutes(5)))
            throw new LoadRadarException(400, "实际测量起止时间必须成对提供且结束不早于开始");
        if (input.Verdict is not ("empty" or "loaded" or "unknown")) throw new LoadRadarException(400, "未知空载结论");
        if (input.AutomaticBaseline && (input.ChannelId != "CH4" || input.Verdict != "empty" ||
            input.FrameDegraded || input.SourceMode != "live" || input.BaselineCandidateSha256 is null))
            throw new LoadRadarException(400, "自动空车基线只接受CH4完整实车空车候选");
        if (input.VolumeStatus is not ("success" or "unavailable" or "low_confidence" or "failed")) throw new LoadRadarException(400, "未知体积状态");
        if (input.VolumeM3 is not null && !LoadRadarNumber.InRange(input.VolumeM3, 0m, 1000m) || input.VolumeStatus == "success" && input.VolumeM3 is null)
            throw new LoadRadarException(400, "有效体积必须在 0-1000 立方米内，缺失不可代入 0");
        if (input.FilledRatio is not null && !LoadRadarNumber.InRange(input.FilledRatio, 0m, 1m) ||
            input.HoleRatio is not null && !LoadRadarNumber.InRange(input.HoleRatio, 0m, 1m) ||
            input.BoundSpread is not null && !LoadRadarNumber.InRange(input.BoundSpread, 0m))
            throw new LoadRadarException(400, "体积质量指标越界");
        if (input.BaselineVersionId is not null) Identifier(input.BaselineVersionId, "baselineVersionId");
        if (input.BaselineUse is { } use)
        {
            if (use.SourceChannelId != "CH4" || use.Policy != "ch4-shared-v1" ||
                use.RegistrationStatus is not ("resolved" or "baseline_fallback") || input.BaselineVersionId is null)
                throw new LoadRadarException(400, "共享基线必须声明CH4来源、版本及配准结果");
            Identifier(use.SourceCalibrationId, "sourceCalibrationId");
            if (!Regex.IsMatch(use.SourceConfigurationHash ?? "", "\\A[a-f0-9]{64}\\z") ||
                new[] { use.DxM, use.DyM, use.YawSlope, use.YawPivotYM }.Any(v => !double.IsFinite(v)))
                throw new LoadRadarException(400, "基线来源摘要或配准参数无效");
            if (use.Pose is { } pose && (pose.Left is not { Length: 2 } || pose.Right is not { Length: 2 } ||
                !(pose.XLeft < pose.XRight) || pose.Left.Concat(pose.Right).Concat(new[] { pose.XLeft, pose.XRight, pose.YMid }).Any(v => !double.IsFinite(v))))
                throw new LoadRadarException(400, "基线姿态参数无效");
            Text(use.RegistrationReason, "registrationReason", 2000);
        }
        Text(input.Reason, "reason", 2000);
        if (input.Points is null || input.Points.Length > 10000 || input.PointCount < input.Points.Length || input.PointCount < 0 ||
            input.Points.Any(p => p is null || p.Length != 3 || p.Any(v => !float.IsFinite(v))))
            throw new LoadRadarException(400, "预览点云最多 10000 个有限 XYZ 点，且不能超过原始点数");
    }

    public static void Identifier(string? value, string name)
    {
        if (value is null || !Regex.IsMatch(value, "\\A[A-Za-z0-9_.:-]{1,64}\\z"))
            throw new LoadRadarException(400, name + " 必须为 1-64 位字母、数字、下划线、点、冒号或连字符");
    }

    public static void Text(string? value, string name, int max)
    {
        if (string.IsNullOrWhiteSpace(value) || value.Length > max || value != value.Trim() || value.Any(char.IsControl))
            throw new LoadRadarException(400, name + " 不能为空、超长、含首尾空格或控制字符");
    }

    public static string Key(params object[] values) => Convert.ToHexString(
        SHA256.HashData(Encoding.UTF8.GetBytes(JsonSerializer.Serialize(values)))).ToLowerInvariant();
}
