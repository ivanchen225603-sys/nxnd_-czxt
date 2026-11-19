namespace CarPointCloudWebApi.Domain;

/// <summary>每个车牌唯一的最新空车档案；历史扫描仍保存在 VehicleDimensionRecord。</summary>
public sealed class EmptyVehicleProfileRecord
{
    /// <summary>关联尺寸记录的质量状态，无新增数据库列。</summary>
    public string DetectStatus { get; set; } = "success";
    /// <summary>空车档案表自增主键。</summary>
    public long Id { get; set; }
    /// <summary>档案归属的标准化车牌号。</summary>
    public string LicensePlate { get; set; } = string.Empty;
    /// <summary>生成该档案的车辆尺寸记录主键。</summary>
    public long DimensionRecordId { get; set; }
    /// <summary>贯穿来车、采集和分析链路的业务流程 ID。</summary>
    public string? WorkflowId { get; set; }
    /// <summary>双雷达配对标识；单雷达档案可为空字符串。</summary>
    public string PairId { get; set; } = string.Empty;
    /// <summary>产生档案的雷达或摄像头点位编码。</summary>
    public string CameraIdentifier { get; set; } = string.Empty;
    /// <summary>服务端保存的空车点云绝对路径。</summary>
    public string PointCloudPath { get; set; } = string.Empty;
    /// <summary>点云文件 SHA-256，用于下载完整性校验。</summary>
    public string PointCloudSha256 { get; set; } = string.Empty;
    /// <summary>点云文件大小，单位字节。</summary>
    public long PointCloudSize { get; set; }
    /// <summary>点云包含的有效点数量。</summary>
    public long PointCount { get; set; }
    /// <summary>空车点云实际采集时间。</summary>
    public DateTime ScanTime { get; set; }
    /// <summary>车头到车厢前沿距离，单位厘米。</summary>
    public decimal? CabToTrailerFrontDist { get; set; }
    /// <summary>车厢长度，单位厘米。</summary>
    public decimal? TrailerLength { get; set; }
    /// <summary>车厢宽度，单位厘米。</summary>
    public decimal? TrailerWidth { get; set; }
    /// <summary>车厢厢体高度，单位厘米。</summary>
    public decimal? TrailerBodyHeight { get; set; }
    /// <summary>车厢底部离地高度，单位厘米。</summary>
    public decimal? TrailerBottomToGroundHeight { get; set; }
    /// <summary>高低板整体高度差，单位厘米。</summary>
    public decimal? HighLowPlateHeight { get; set; }
    /// <summary>档案首次创建时间。</summary>
    public DateTime CreatedAt { get; set; }
    /// <summary>档案最后一次被新空车扫描替换的时间。</summary>
    public DateTime UpdatedAt { get; set; }
    /// <summary>与尺寸主记录关联的高低板测量明细。</summary>
    public List<HighLowPlateInfo> HighLowPlates { get; set; } = new();
    /// <summary>与尺寸主记录关联的筋条测量明细。</summary>
    public List<RibInfo> Ribs { get; set; } = new();
    /// <summary>与尺寸主记录关联的钢丝绳测量明细。</summary>
    public List<WireInfo> Wires { get; set; } = new();
}

/// <summary>后台点云查看器按车牌查询的最新空车档案。</summary>
public sealed class EmptyVehicleProfileDto
{
    /// <summary>pending 表示已保存、待人工核验。</summary>
    public string? ReviewStatus { get; set; }
    /// <summary>用于查询和展示的车牌号。</summary>
    public string LicensePlate { get; set; } = string.Empty;
    /// <summary>档案来源车辆尺寸记录的主键。</summary>
    public long SourceRecordId { get; set; }
    /// <summary>来源记录关联的车辆业务流程 ID。</summary>
    public string? WorkflowId { get; set; }
    /// <summary>来源记录的双雷达配对标识。</summary>
    public string PairId { get; set; } = string.Empty;
    /// <summary>来源雷达或摄像头点位编码。</summary>
    public string CameraIdentifier { get; set; } = string.Empty;
    /// <summary>空车点云实际采集时间。</summary>
    public DateTime ScanTime { get; set; }
    /// <summary>该车牌档案最后更新时间。</summary>
    public DateTime UpdatedAt { get; set; }
    /// <summary>空车点云有效点数量。</summary>
    public long PointCount { get; set; }
    /// <summary>空车点云文件大小，单位字节。</summary>
    public long FileSize { get; set; }
    /// <summary>点云文件 SHA-256 校验值。</summary>
    public string PointCloudSha256 { get; set; } = string.Empty;
    /// <summary>需要登录后访问的点云下载地址。</summary>
    public string PointCloudUrl { get; set; } = string.Empty;
    /// <summary>空车扫描对应的尺寸及结构明细。</summary>
    public VehicleDimensionsDto Dimensions { get; set; } = new();
}
