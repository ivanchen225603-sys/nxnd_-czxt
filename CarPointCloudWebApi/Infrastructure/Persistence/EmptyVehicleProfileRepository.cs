using System.Data.Common;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>读取每个车牌唯一的最新空车融合云图档案。</summary>
public sealed class EmptyVehicleProfileRepository : IEmptyVehicleProfileRepository
{
    private readonly string _connectionString;
    private readonly int _commandTimeoutSeconds;

    /// <summary>创建从达梦数据库读取空车基准档案的仓储。</summary>
    /// <param name="configuration">数据库连接串和命令超时配置。</param>
    public EmptyVehicleProfileRepository(IConfiguration configuration)
    {
        _connectionString = configuration.GetConnectionString("CarPointCloudDb")
            ?? throw new InvalidOperationException("缺少数据库连接字符串 ConnectionStrings:CarPointCloudDb");
        _commandTimeoutSeconds = configuration.GetValue<int>("Database:CommandTimeoutSeconds", 30);
    }

    /// <inheritdoc />
    public EmptyVehicleProfileRecord? GetByLicensePlate(string licensePlate)
    {
        using var connection = new DmConnection(_connectionString);
        connection.Open();
        using var command = new DmCommand(
            @"SELECT p.*, r.DetectStatus
              FROM EmptyVehicleProfile p
              JOIN VehicleDimensionRecord r ON r.Id = p.DimensionRecordId
              WHERE p.LicensePlate = ?",
            connection);
        command.CommandTimeout = _commandTimeoutSeconds;
        command.Parameters.Add("@LicensePlate", DmDbType.VarChar, 32).Value = licensePlate;
        using var reader = command.ExecuteReader();
        if (!reader.Read())
        {
            return null;
        }

        var record = Map(reader);
        reader.Close();
        LoadDetails(connection, record);
        return record;
    }

    /// <inheritdoc />
    public IReadOnlyDictionary<string, EmptyVehicleProfileRecord> GetDimensionsByLicensePlates(IReadOnlyList<string> licensePlates)
    {
        var profiles = new Dictionary<string, EmptyVehicleProfileRecord>(StringComparer.Ordinal);
        if (licensePlates.Count == 0) return profiles;

        using var connection = new DmConnection(_connectionString);
        connection.Open();
        var placeholders = string.Join(", ", Enumerable.Repeat("?", licensePlates.Count));
        using var command = new DmCommand($"""
            SELECT p.LicensePlate, p.TrailerLength, p.TrailerWidth, p.TrailerBodyHeight, r.DetectStatus
            FROM EmptyVehicleProfile p JOIN VehicleDimensionRecord r ON r.Id = p.DimensionRecordId
            WHERE p.LicensePlate IN ({placeholders})
            """, connection)
        { CommandTimeout = _commandTimeoutSeconds };
        for (var index = 0; index < licensePlates.Count; index++)
            command.Parameters.Add($"@Plate{index}", DmDbType.VarChar, 32).Value = licensePlates[index];
        using var reader = command.ExecuteReader();
        while (reader.Read())
        {
            var plate = reader.GetString(0);
            profiles.Add(plate, new EmptyVehicleProfileRecord
            {
                LicensePlate = plate,
                DetectStatus = reader.GetString(reader.GetOrdinal("DetectStatus")),
                TrailerLength = GetNullableDecimal(reader, "TrailerLength"),
                TrailerWidth = GetNullableDecimal(reader, "TrailerWidth"),
                TrailerBodyHeight = GetNullableDecimal(reader, "TrailerBodyHeight")
            });
        }
        return profiles;
    }

    private static EmptyVehicleProfileRecord Map(DbDataReader reader) => new()
    {
        DetectStatus = reader.GetString(reader.GetOrdinal("DetectStatus")),
        Id = reader.GetInt64(reader.GetOrdinal("Id")),
        LicensePlate = reader.GetString(reader.GetOrdinal("LicensePlate")),
        DimensionRecordId = reader.GetInt64(reader.GetOrdinal("DimensionRecordId")),
        WorkflowId = GetNullableString(reader, "WorkflowId"),
        PairId = reader.GetString(reader.GetOrdinal("PairId")),
        CameraIdentifier = reader.GetString(reader.GetOrdinal("CameraIdentifier")),
        PointCloudPath = reader.GetString(reader.GetOrdinal("PointCloudPath")),
        PointCloudSha256 = reader.GetString(reader.GetOrdinal("PointCloudSha256")),
        PointCloudSize = reader.GetInt64(reader.GetOrdinal("PointCloudSize")),
        PointCount = reader.GetInt64(reader.GetOrdinal("PointCount")),
        ScanTime = reader.GetDateTime(reader.GetOrdinal("ScanTime")),
        CabToTrailerFrontDist = GetNullableDecimal(reader, "CabToTrailerFrontDist"),
        TrailerLength = GetNullableDecimal(reader, "TrailerLength"),
        TrailerWidth = GetNullableDecimal(reader, "TrailerWidth"),
        TrailerBodyHeight = GetNullableDecimal(reader, "TrailerBodyHeight"),
        TrailerBottomToGroundHeight = GetNullableDecimal(reader, "TrailerBottomToGroundHeight"),
        HighLowPlateHeight = GetNullableDecimal(reader, "HighLowPlateHeight"),
        CreatedAt = reader.GetDateTime(reader.GetOrdinal("CreatedAt")),
        UpdatedAt = reader.GetDateTime(reader.GetOrdinal("UpdatedAt"))
    };

    private void LoadDetails(DmConnection connection, EmptyVehicleProfileRecord record)
    {
        using (var command = new DmCommand(
            @"SELECT Id, DimensionRecordId, PlateId, HighestToGround, LowestToGround
              FROM VehicleDimensionHighLowPlate WHERE DimensionRecordId = ? ORDER BY PlateId",
            connection))
        {
            command.CommandTimeout = _commandTimeoutSeconds;
            command.Parameters.Add("@DimensionRecordId", DmDbType.Int64).Value = record.DimensionRecordId;
            using var reader = command.ExecuteReader();
            while (reader.Read())
            {
                record.HighLowPlates.Add(new HighLowPlateInfo
                {
                    Id = reader.GetInt64(0),
                    DimensionRecordId = reader.GetInt64(1),
                    PlateId = reader.GetInt32(2),
                    HighestToGround = reader.IsDBNull(3) ? null : reader.GetDecimal(3),
                    LowestToGround = reader.IsDBNull(4) ? null : reader.GetDecimal(4)
                });
            }
        }

        LoadLinearDetails(connection, record);
    }

    private void LoadLinearDetails(DmConnection connection, EmptyVehicleProfileRecord record)
    {
        using (var command = new DmCommand(
            @"SELECT Id, DimensionRecordId, RibId, DistToPrev
              FROM VehicleDimensionRib WHERE DimensionRecordId = ? ORDER BY RibId",
            connection))
        {
            command.CommandTimeout = _commandTimeoutSeconds;
            command.Parameters.Add("@DimensionRecordId", DmDbType.Int64).Value = record.DimensionRecordId;
            using var reader = command.ExecuteReader();
            while (reader.Read())
            {
                record.Ribs.Add(new RibInfo
                {
                    Id = reader.GetInt64(0),
                    DimensionRecordId = reader.GetInt64(1),
                    RibId = reader.GetInt32(2),
                    DistToPrev = reader.IsDBNull(3) ? null : reader.GetDecimal(3)
                });
            }
        }

        using var wireCommand = new DmCommand(
            @"SELECT Id, DimensionRecordId, WireId, DistToPrev
              FROM VehicleDimensionWire WHERE DimensionRecordId = ? ORDER BY WireId",
            connection);
        wireCommand.CommandTimeout = _commandTimeoutSeconds;
        wireCommand.Parameters.Add("@DimensionRecordId", DmDbType.Int64).Value = record.DimensionRecordId;
        using var wireReader = wireCommand.ExecuteReader();
        while (wireReader.Read())
        {
            record.Wires.Add(new WireInfo
            {
                Id = wireReader.GetInt64(0),
                DimensionRecordId = wireReader.GetInt64(1),
                WireId = wireReader.GetInt32(2),
                DistToPrev = wireReader.IsDBNull(3) ? null : wireReader.GetDecimal(3)
            });
        }
    }

    private static string? GetNullableString(DbDataReader reader, string name)
    {
        var ordinal = reader.GetOrdinal(name);
        return reader.IsDBNull(ordinal) ? null : reader.GetString(ordinal);
    }

    private static decimal? GetNullableDecimal(DbDataReader reader, string name)
    {
        var ordinal = reader.GetOrdinal(name);
        return reader.IsDBNull(ordinal) ? null : reader.GetDecimal(ordinal);
    }
}
