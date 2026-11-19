using System.Data;
using System.Data.Common;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

public partial class IntegrationRepository
{

    private void LoadDimensionDetails(DmConnection conn, List<VehicleDimensionRecord> records)
    {
        if (records.Count == 0)
        {
            return;
        }

        var byId = records.ToDictionary(x => x.Id);
        var parameters = records.Select(_ => "?").ToArray();
        var whereIn = string.Join(", ", parameters);

        using (var cmd = new DmCommand(
            $@"SELECT Id, DimensionRecordId, RibId, DistToPrev
               FROM VehicleDimensionRib
               WHERE DimensionRecordId IN ({whereIn})
               ORDER BY DimensionRecordId, RibId", conn))
        {
            AddIdParameters(cmd, records);
            using var reader = cmd.ExecuteReader();
            while (reader.Read())
            {
                var rib = MapRib(reader);
                if (byId.TryGetValue(rib.DimensionRecordId, out var record))
                {
                    record.Ribs.Add(rib);
                }
            }
        }

        using (var cmd = new DmCommand(
            $@"SELECT Id, DimensionRecordId, WireId, DistToPrev
               FROM VehicleDimensionWire
               WHERE DimensionRecordId IN ({whereIn})
               ORDER BY DimensionRecordId, WireId", conn))
        {
            AddIdParameters(cmd, records);
            using var reader = cmd.ExecuteReader();
            while (reader.Read())
            {
                var wire = MapWire(reader);
                if (byId.TryGetValue(wire.DimensionRecordId, out var record))
                {
                    record.Wires.Add(wire);
                }
            }
        }

        using (var cmd = new DmCommand(
            $@"SELECT Id, DimensionRecordId, PlateId, HighestToGround, LowestToGround, LengthCm
               FROM VehicleDimensionHighLowPlate
               WHERE DimensionRecordId IN ({whereIn})
               ORDER BY DimensionRecordId, PlateId", conn))
        {
            AddIdParameters(cmd, records);
            using var reader = cmd.ExecuteReader();
            while (reader.Read())
            {
                var plate = MapHighLowPlate(reader);
                if (byId.TryGetValue(plate.DimensionRecordId, out var record))
                {
                    record.HighLowPlates.Add(plate);
                }
            }
        }
    }

    private static VehicleDimensionRecord MapVehicleDimension(DbDataReader reader)
    {
        return new VehicleDimensionRecord
        {
            Id = reader.GetInt64(reader.GetOrdinal("Id")),
            IdempotencyKey = reader.GetString(reader.GetOrdinal("IdempotencyKey")),
            WorkflowId = GetNullableString(reader, "WorkflowId"),
            PairId = GetNullableString(reader, "PairId"),
            PayloadHash = GetNullableString(reader, "PayloadHash"),
            LicensePlate = reader.GetString(reader.GetOrdinal("LicensePlate")),
            CameraIdentifier = reader.GetString(reader.GetOrdinal("CameraIdentifier")),
            GetTime = reader.GetDateTime(reader.GetOrdinal("GetTime")),
            ActualRibCount = reader.IsDBNull(reader.GetOrdinal("ActualRibCount"))
                ? null : reader.GetInt32(reader.GetOrdinal("ActualRibCount")),
            DetectStatus = reader.GetString(reader.GetOrdinal("DetectStatus")),
            CabToTrailerFrontDist = GetNullableDecimal(reader, "CabToTrailerFrontDist"),
            TrailerLength = GetNullableDecimal(reader, "TrailerLength"),
            TrailerWidth = GetNullableDecimal(reader, "TrailerWidth"),
            MaximumWidthCm = GetNullableDecimal(reader, "MaximumWidthCm"),
            TrailerBodyHeight = GetNullableDecimal(reader, "TrailerBodyHeight"),
            TrailerBottomToGroundHeight = GetNullableDecimal(reader, "TrailerBottomToGroundHeight"),
            HighLowPlateHeight = GetNullableDecimal(reader, "HighLowPlateHeight"),
            SourceFile = GetNullableString(reader, "SourceFile"),
            ResultJsonPath = GetNullableString(reader, "ResultJsonPath"),
            ResultImagePath = GetNullableString(reader, "ResultImagePath"),
            CreatedAt = reader.GetDateTime(reader.GetOrdinal("CreatedAt")),
            UpdatedAt = reader.GetDateTime(reader.GetOrdinal("UpdatedAt"))
        };
    }

    private static CoalVolumeRecord MapCoalVolume(DbDataReader reader)
    {
        return new CoalVolumeRecord
        {
            Id = reader.GetInt64(reader.GetOrdinal("Id")),
            IdempotencyKey = reader.GetString(reader.GetOrdinal("IdempotencyKey")),
            BusinessKey = reader.GetString(reader.GetOrdinal("BusinessKey")),
            WorkflowId = GetNullableString(reader, "WorkflowId"),
            PayloadHash = GetNullableString(reader, "PayloadHash"),
            LicensePlate = reader.GetString(reader.GetOrdinal("LicensePlate")),
            CameraIdentifier = reader.GetString(reader.GetOrdinal("CameraIdentifier")),
            Volume = reader.GetDecimal(reader.GetOrdinal("Volume")),
            Unit = reader.GetString(reader.GetOrdinal("Unit")),
            MeasureTime = reader.GetDateTime(reader.GetOrdinal("MeasureTime")),
            MeasurementStartedAt = reader.IsDBNull(reader.GetOrdinal("MeasurementStartedAt"))
                ? null : reader.GetDateTime(reader.GetOrdinal("MeasurementStartedAt")),
            MeasurementEndedAt = reader.IsDBNull(reader.GetOrdinal("MeasurementEndedAt"))
                ? null : reader.GetDateTime(reader.GetOrdinal("MeasurementEndedAt")),
            EnterId = GetNullableString(reader, "EnterId"),
            CaptureTime = reader.IsDBNull(reader.GetOrdinal("CaptureTime"))
                ? null : reader.GetDateTime(reader.GetOrdinal("CaptureTime")),
            LicensePlateRecordId = GetNullableInt64(reader, "LicensePlateRecordId"),
            Status = reader.GetString(reader.GetOrdinal("Status")),
            Confidence = GetNullableDecimal(reader, "Confidence"),
            SourceFile = GetNullableString(reader, "SourceFile"),
            BaselineFile = GetNullableString(reader, "BaselineFile"),
            CreatedAt = reader.GetDateTime(reader.GetOrdinal("CreatedAt"))
        };
    }

    private static RibInfo MapRib(DbDataReader reader)
    {
        return new RibInfo
        {
            Id = reader.GetInt64(reader.GetOrdinal("Id")),
            DimensionRecordId = reader.GetInt64(reader.GetOrdinal("DimensionRecordId")),
            RibId = reader.GetInt32(reader.GetOrdinal("RibId")),
            DistToPrev = GetNullableDecimal(reader, "DistToPrev")
        };
    }

    private static WireInfo MapWire(DbDataReader reader)
    {
        return new WireInfo
        {
            Id = reader.GetInt64(reader.GetOrdinal("Id")),
            DimensionRecordId = reader.GetInt64(reader.GetOrdinal("DimensionRecordId")),
            WireId = reader.GetInt32(reader.GetOrdinal("WireId")),
            DistToPrev = GetNullableDecimal(reader, "DistToPrev")
        };
    }

    private static HighLowPlateInfo MapHighLowPlate(DbDataReader reader)
    {
        return new HighLowPlateInfo
        {
            Id = reader.GetInt64(reader.GetOrdinal("Id")),
            DimensionRecordId = reader.GetInt64(reader.GetOrdinal("DimensionRecordId")),
            PlateId = reader.GetInt32(reader.GetOrdinal("PlateId")),
            HighestToGround = GetNullableDecimal(reader, "HighestToGround"),
            LengthCm = GetNullableDecimal(reader, "LengthCm"),
            LowestToGround = GetNullableDecimal(reader, "LowestToGround")
        };
    }

    private static void AddIdParameters(DmCommand cmd, List<VehicleDimensionRecord> records)
    {
        for (var index = 0; index < records.Count; index++)
        {
            cmd.Parameters.Add($"@Id{index}", DmDbType.Int64).Value = records[index].Id;
        }
    }

    private static PlateAcquisitionJob MapPlateAcquisitionJob(DbDataReader reader) => new()
    {
        Id = reader.GetInt64(0),
        WorkflowId = reader.GetString(1),
        InteractionId = reader.GetString(2),
        CameraIdentifier = reader.GetString(3),
        Status = reader.GetString(4),
        AttemptCount = reader.GetInt32(5),
        NextAttemptAt = reader.GetDateTime(6),
        LastError = reader.IsDBNull(7) ? null : reader.GetString(7),
        CreatedAt = reader.GetDateTime(8),
        UpdatedAt = reader.GetDateTime(9),
        CandidateJson = reader.IsDBNull(10) ? null : reader.GetString(10)
    };

    private static VehicleArrivalRecord MapVehicleArrival(DbDataReader reader) => new()
    {
        Id = Convert.ToInt64(reader.GetValue(0)),
        EventId = reader.GetString(1),
        DeviceId = reader.GetString(2),
        CameraIdentifier = reader.GetString(3),
        EventType = reader.GetString(4),
        SessionId = reader.GetString(5),
        LicensePlate = reader.IsDBNull(6) ? null : reader.GetString(6),
        PlateStatus = reader.GetString(7),
        PlateSource = reader.IsDBNull(8) ? null : reader.GetString(8),
        PlateCapturedAt = reader.IsDBNull(9) ? null : reader.GetDateTime(9),
        ArrivalTime = reader.GetDateTime(10),
        RadarStatus = reader.GetString(11),
        RadarError = reader.IsDBNull(12) ? null : reader.GetString(12),
        CreatedAt = reader.GetDateTime(13),
        UpdatedAt = reader.GetDateTime(14)
    };
}
