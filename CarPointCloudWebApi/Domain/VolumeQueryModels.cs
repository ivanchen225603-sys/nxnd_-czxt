namespace CarPointCloudWebApi.Domain;

/// <summary>后台只读体积查询；时间范围为业务日期的左闭右开区间。</summary>
public sealed record VolumeQuery(DateTime From, DateTime Until, string? LicensePlate,
    string? CameraIdentifier, int Page, int PageSize);

public sealed record VolumeQueryRow(DateTime CollectedAt, string LicensePlate,
    string? ChannelId, string ScaleName, decimal VolumeM3);

public sealed record VolumeQueryPage(IReadOnlyList<VolumeQueryRow> Items, long Total, int Page, int PageSize);
