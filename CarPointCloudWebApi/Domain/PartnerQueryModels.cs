using System.Text.Json.Serialization;

namespace CarPointCloudWebApi.Domain;

/// <summary>合同 v4 对外车辆尺寸响应；内部记录编号、流程及审核信息不对外发送。</summary>
public sealed class PartnerVehicleDimensionDto
{
    public string LicensePlate { get; init; } = string.Empty;
    public string CameraIdentifier { get; init; } = string.Empty;
    public string GetTime { get; init; } = string.Empty;
    public object Dimensions { get; init; } = new { };
}

/// <summary>合同 v4 对外煤量响应；全部必填字段仅由已完整关联的原始记录提供。</summary>
public sealed class PartnerCoalVolumeDto
{
    public string LicensePlate { get; init; } = string.Empty;
    public string CameraIdentifier { get; init; } = string.Empty;
    public string CaptureTime { get; init; } = string.Empty;
    [JsonPropertyName("enterID")]
    public string EnterId { get; init; } = string.Empty;
    public decimal Volume { get; init; }
    public string Unit { get; init; } = "m³";
    public string MeasureTime { get; init; } = string.Empty;
    public string MeasureEndTime { get; init; } = string.Empty;
}

/// <summary>对外尺寸单位均为 CM；不与内部高度语义混用。</summary>
public sealed class PartnerVehicleDimensionsDto
{
    public decimal? CabToTrailerFrontDist { get; init; }
    public decimal? TrailerLength { get; init; }
    public decimal? TrailerWidth { get; init; }
    public decimal? TrailerBodyHeight { get; init; }
    public decimal? TrailerBottomToGroundHeight { get; init; }
    public decimal? HighLowPlateHeight { get; init; }
    public List<PartnerHighLowPlateDto> HighLowPlates { get; init; } = [];
    public List<RibDto> Ribs { get; init; } = [];
    public List<WireDto> Wires { get; init; } = [];
}

public sealed class PartnerHighLowPlateDto
{
    public int PlateId { get; init; }
    /// <summary>合同 v4 及用户确认：该高低板长度，单位 CM。旧高度不会映射到这里。</summary>
    public decimal? HighestToGround { get; init; }
    /// <summary>该高低板离地高度，单位 CM。</summary>
    public decimal? LowestToGround { get; init; }
}
