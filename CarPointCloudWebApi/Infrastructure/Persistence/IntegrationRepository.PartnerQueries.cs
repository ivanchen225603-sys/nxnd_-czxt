using System.Data.Common;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public partial class IntegrationRepository
{
    /// <inheritdoc />
    public async Task<List<VehicleDimensionRecord>> GetPartnerVehiclePageAsync(int take, DateTime? windowStart,
        DateTime asOf, DateTime? beforeTime, long? beforeId, CancellationToken token)
    {
        ValidatePageArguments(ref take, beforeTime, beforeId);
        await using var connection = CreateConnection();
        await connection.OpenAsync(token);
        using var command = PartnerPageCommand(connection, "VehicleDimensionRecord", VehicleDimensionColumns,
            "GetTime", take, windowStart, asOf, beforeTime, beforeId);
        using var cancellation = CancelPartnerCommand(command, token);
        var records = new List<VehicleDimensionRecord>();
        await using (var reader = await command.ExecuteReaderAsync(token))
            while (await reader.ReadAsync(token)) records.Add(MapVehicleDimension(reader));
        await LoadPartnerDimensionDetailsAsync(connection, records, token);
        return records;
    }

    /// <inheritdoc />
    public async Task<List<CoalVolumeRecord>> GetPartnerCoalPageAsync(int take, DateTime? windowStart,
        DateTime asOf, DateTime? beforeTime, long? beforeId, CancellationToken token)
    {
        ValidatePageArguments(ref take, beforeTime, beforeId);
        await using var connection = CreateConnection();
        await connection.OpenAsync(token);
        using var command = PartnerPageCommand(connection, "CoalVolumeRecord", CoalVolumeColumns,
            "MeasurementStartedAt", take, windowStart, asOf, beforeTime, beforeId,
            "MeasurementEndedAt IS NOT NULL AND MeasurementEndedAt >= MeasurementStartedAt " +
            "AND LicensePlateRecordId IS NOT NULL AND Volume > 0 AND LOWER(Status) = 'success' AND Unit IN ('m3', 'm³')");
        using var cancellation = CancelPartnerCommand(command, token);
        var records = new List<CoalVolumeRecord>();
        await using var reader = await command.ExecuteReaderAsync(token);
        while (await reader.ReadAsync(token)) records.Add(MapCoalVolume(reader));
        return records;
    }

    private static DmCommand PartnerPageCommand(DmConnection connection, string table, string columns, string clock,
        int take, DateTime? windowStart, DateTime asOf, DateTime? beforeTime, long? beforeId, string? eligibility = null)
    {
        // 表名、列名、时钟字段均由上面的固定调用点给定，不接受 HTTP 输入。
        var filters = new List<string> { $"{clock} <= ?" };
        if (eligibility is not null) filters.Add(eligibility);
        if (windowStart.HasValue) filters.Add($"{clock} >= ?");
        if (beforeTime.HasValue) filters.Add($"({clock} < ? OR ({clock} = ? AND Id < ?))");
        var command = new DmCommand($"SELECT TOP {take} {columns} FROM {table} " +
            $"WHERE {string.Join(" AND ", filters)} ORDER BY {clock} DESC, Id DESC", connection)
        { CommandTimeout = 2 };
        command.Parameters.Add("@AsOf", DmDbType.DateTime).Value = asOf;
        if (windowStart.HasValue) command.Parameters.Add("@Window", DmDbType.DateTime).Value = windowStart.Value;
        if (beforeTime.HasValue)
        {
            command.Parameters.Add("@Before1", DmDbType.DateTime).Value = beforeTime.Value;
            command.Parameters.Add("@Before2", DmDbType.DateTime).Value = beforeTime.Value;
            command.Parameters.Add("@BeforeId", DmDbType.Int64).Value = beforeId!.Value;
        }
        return command;
    }

    private static CancellationTokenRegistration CancelPartnerCommand(DmCommand command, CancellationToken token) =>
        token.Register(() =>
        {
            try { command.Cancel(); }
            catch (Exception) { /* 请求取消与连接释放可并发，外层预算仍会终止等待。 */ }
        });

    private async Task LoadPartnerDimensionDetailsAsync(DmConnection connection,
        List<VehicleDimensionRecord> records, CancellationToken token)
    {
        if (records.Count == 0) return;
        var byId = records.ToDictionary(record => record.Id);
        var ids = string.Join(",", records.Select(_ => "?"));
        async Task Read(string table, string fields, string order, Action<DbDataReader> accept)
        {
            using var command = new DmCommand($"SELECT {fields} FROM {table} WHERE DimensionRecordId IN ({ids}) " +
                $"ORDER BY DimensionRecordId, {order}", connection)
            { CommandTimeout = 2 };
            AddIdParameters(command, records);
            using var cancellation = CancelPartnerCommand(command, token);
            await using var reader = await command.ExecuteReaderAsync(token);
            while (await reader.ReadAsync(token)) accept(reader);
        }
        await Read("VehicleDimensionRib", "Id, DimensionRecordId, RibId, DistToPrev", "RibId", reader =>
        {
            var rib = MapRib(reader);
            if (byId.TryGetValue(rib.DimensionRecordId, out var record)) record.Ribs.Add(rib);
        });
        await Read("VehicleDimensionHighLowPlate", "Id, DimensionRecordId, PlateId, HighestToGround, LowestToGround, LengthCm", "PlateId", reader =>
        {
            var plate = MapHighLowPlate(reader);
            if (byId.TryGetValue(plate.DimensionRecordId, out var record)) record.HighLowPlates.Add(plate);
        });
    }
}
