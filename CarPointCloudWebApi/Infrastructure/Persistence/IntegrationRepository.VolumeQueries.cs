using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public partial class IntegrationRepository
{
    /// <inheritdoc />
    public async Task<VolumeQueryPage> QueryVolumesAsync(VolumeQuery query, CancellationToken cancellationToken)
    {
        await using var connection = CreateConnection();
        await connection.OpenAsync(cancellationToken);
        // 正式记录按实际测量时间筛选，不以识牌或接收入库时间代替；旧记录仍可查询。
        var filter = "MeasureTime >= ? AND MeasureTime < ? AND LOWER(Status) = 'success' " +
            "AND Unit IN ('m3', 'm³') AND Volume >= 0";
        if (query.LicensePlate is not null) filter += " AND LicensePlate = ?";
        if (query.CameraIdentifier is not null) filter += " AND CameraIdentifier = ?";

        // 两条SQL使用相同条件和参数顺序；HTTP输入不拼接进SQL。
        using var count = new DmCommand($"SELECT COUNT(*) FROM CoalVolumeRecord WHERE {filter}", connection)
        { CommandTimeout = _commandTimeoutSeconds };
        using var command = new DmCommand($"""
            SELECT MeasureTime, LicensePlate, CameraIdentifier, Volume FROM (
                SELECT MeasureTime, LicensePlate, CameraIdentifier, Volume,
                    ROW_NUMBER() OVER (ORDER BY MeasureTime DESC, Id DESC) AS RowNo
                FROM CoalVolumeRecord WHERE {filter}
            ) records WHERE RowNo > ? AND RowNo <= ? ORDER BY RowNo
            """, connection) { CommandTimeout = _commandTimeoutSeconds };
        foreach (var cmd in new[] { count, command })
        {
            cmd.Parameters.Add("@From", DmDbType.DateTime).Value = query.From;
            cmd.Parameters.Add("@Until", DmDbType.DateTime).Value = query.Until;
            if (query.LicensePlate is not null)
                cmd.Parameters.Add("@Plate", DmDbType.VarChar, 64).Value = query.LicensePlate;
            if (query.CameraIdentifier is not null)
                cmd.Parameters.Add("@Camera", DmDbType.VarChar, 64).Value = query.CameraIdentifier;
        }
        var offset = (query.Page - 1) * query.PageSize;
        command.Parameters.Add("@Offset", DmDbType.Int32).Value = offset;
        command.Parameters.Add("@End", DmDbType.Int32).Value = offset + query.PageSize;
        var total = Convert.ToInt64(await count.ExecuteScalarAsync(cancellationToken));
        var rows = new List<VolumeQueryRow>();
        await using var reader = await command.ExecuteReaderAsync(cancellationToken);
        while (await reader.ReadAsync(cancellationToken))
        {
            var camera = reader.GetString(2);
            var channel = camera switch
            {
                "CAM_001" => "CH1",
                "CAM_002" => "CH2",
                "CAM_003" => "CH3",
                "CAM_004" => "CH4",
                _ => null
            };
            rows.Add(new(reader.GetDateTime(0), reader.GetString(1), channel,
                channel is null ? $"未配置（{camera}）" : $"{channel[2]}号磅", reader.GetDecimal(3)));
        }
        return new(rows, total, query.Page, query.PageSize);
    }
}
