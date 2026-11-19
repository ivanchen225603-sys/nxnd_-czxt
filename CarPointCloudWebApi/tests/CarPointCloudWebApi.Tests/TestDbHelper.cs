using System.Data;
using Dm;

namespace CarPointCloudWebApi.Tests;

public class TestDbHelper : IDisposable
{
    private readonly string _connectionString;
    private readonly string _testUserAgent;

    public TestDbHelper(string connectionString, string testUserAgent)
    {
        _connectionString = connectionString;
        _testUserAgent = testUserAgent;
    }

    public long InsertTestVehicleDimensionRecord(
        string licensePlate, string cameraIdentifier, string detectStatus,
        decimal? cabToTrailerFrontDist, decimal? trailerLength, decimal? trailerWidth,
        decimal? trailerBodyHeight, decimal? trailerBottomToGroundHeight, decimal? highLowPlateHeight,
        List<(int ribId, decimal? distToPrev)>? ribs = null,
        List<(int wireId, decimal? distToPrev)>? wires = null,
        List<(int plateId, decimal? highest, decimal? lowest)>? plates = null,
        DateTime? getTime = null)
    {
        using var conn = new DmConnection(_connectionString);
        conn.Open();

        using var tx = conn.BeginTransaction();
        try
        {
            var now = getTime ?? DateTime.Now;
            using var cmd = new DmCommand(@"
                INSERT INTO VehicleDimensionRecord
                (IdempotencyKey, LicensePlate, CameraIdentifier, GetTime, DetectStatus,
                 CabToTrailerFrontDist, TrailerLength, TrailerWidth, TrailerBodyHeight,
                 TrailerBottomToGroundHeight, HighLowPlateHeight, CreatedAt, UpdatedAt)
                VALUES
                (?, ?, ?, ?, ?,
                 ?, ?, ?, ?,
                 ?, ?, ?, ?);
                SELECT SCOPE_IDENTITY()", conn, (DmTransaction)tx);

            cmd.Parameters.Add("@IdempotencyKey", DmDbType.VarChar, 64).Value = $"test-dim-{Guid.NewGuid():N}";
            cmd.Parameters.Add("@LicensePlate", DmDbType.VarChar, 32).Value = licensePlate;
            cmd.Parameters.Add("@CameraIdentifier", DmDbType.VarChar, 64).Value = cameraIdentifier;
            cmd.Parameters.Add("@GetTime", DmDbType.DateTime).Value = now;
            cmd.Parameters.Add("@DetectStatus", DmDbType.VarChar, 32).Value = detectStatus;
            AddNullableDecimal(cmd, "@CabToTrailerFrontDist", cabToTrailerFrontDist);
            AddNullableDecimal(cmd, "@TrailerLength", trailerLength);
            AddNullableDecimal(cmd, "@TrailerWidth", trailerWidth);
            AddNullableDecimal(cmd, "@TrailerBodyHeight", trailerBodyHeight);
            AddNullableDecimal(cmd, "@TrailerBottomToGroundHeight", trailerBottomToGroundHeight);
            AddNullableDecimal(cmd, "@HighLowPlateHeight", highLowPlateHeight);
            cmd.Parameters.Add("@CreatedAt", DmDbType.DateTime).Value = now;
            cmd.Parameters.Add("@UpdatedAt", DmDbType.DateTime).Value = now;

            var id = Convert.ToInt64(cmd.ExecuteScalar());

            if (ribs != null)
            {
                foreach (var (ribId, distToPrev) in ribs)
                {
                    using var ribCmd = new DmCommand(
                        "INSERT INTO VehicleDimensionRib (DimensionRecordId, RibId, DistToPrev) VALUES (?, ?, ?)",
                        conn, (DmTransaction)tx);
                    ribCmd.Parameters.Add("@DimId", DmDbType.Int64).Value = id;
                    ribCmd.Parameters.Add("@RibId", DmDbType.Int32).Value = ribId;
                    AddNullableDecimal(ribCmd, "@DistToPrev", distToPrev);
                    ribCmd.ExecuteNonQuery();
                }
            }

            if (wires != null)
            {
                foreach (var (wireId, distToPrev) in wires)
                {
                    using var wireCmd = new DmCommand(
                        "INSERT INTO VehicleDimensionWire (DimensionRecordId, WireId, DistToPrev) VALUES (?, ?, ?)",
                        conn, (DmTransaction)tx);
                    wireCmd.Parameters.Add("@DimId", DmDbType.Int64).Value = id;
                    wireCmd.Parameters.Add("@WireId", DmDbType.Int32).Value = wireId;
                    AddNullableDecimal(wireCmd, "@DistToPrev", distToPrev);
                    wireCmd.ExecuteNonQuery();
                }
            }

            if (plates != null)
            {
                foreach (var (plateId, highest, lowest) in plates)
                {
                    using var plateCmd = new DmCommand(
                        "INSERT INTO VehicleDimensionHighLowPlate (DimensionRecordId, PlateId, HighestToGround, LowestToGround) VALUES (?, ?, ?, ?)",
                        conn, (DmTransaction)tx);
                    plateCmd.Parameters.Add("@DimId", DmDbType.Int64).Value = id;
                    plateCmd.Parameters.Add("@PlateId", DmDbType.Int32).Value = plateId;
                    AddNullableDecimal(plateCmd, "@Highest", highest);
                    AddNullableDecimal(plateCmd, "@Lowest", lowest);
                    plateCmd.ExecuteNonQuery();
                }
            }

            tx.Commit();
            return id;
        }
        catch
        {
            tx.Rollback();
            throw;
        }
    }

    public long InsertTestCoalVolumeRecord(string licensePlate, string cameraIdentifier, decimal volume, string unit, DateTime measureTime, string status, decimal? confidence)
    {
        using var conn = new DmConnection(_connectionString);
        conn.Open();

        var idempotencyKey = $"test-coal-{Guid.NewGuid():N}";
        var businessKey = $"test-business-{Guid.NewGuid():N}";
        var payloadHash = Guid.NewGuid().ToString("N").PadRight(64, '0');
        using var transaction = conn.BeginTransaction();
        using var plate = new DmCommand(@"
            INSERT INTO LicensePlateRecord
            (InteractionId, CameraIdentifier, CameraId, LicensePlate, CaptureTime, EnterId,
             RequestTime, ResponseCode, ResponseMessage, IsSuccess, CreatedAt)
            VALUES (?, ?, ?, ?, ?, ?, ?, 200, 'test-fixture', 1, ?);
            SELECT SCOPE_IDENTITY()", conn, (DmTransaction)transaction);
        plate.Parameters.Add("@Interaction", DmDbType.VarChar, 64).Value = "test-coal-identity-" + Guid.NewGuid().ToString("N");
        plate.Parameters.Add("@Camera", DmDbType.VarChar, 64).Value = cameraIdentifier;
        plate.Parameters.Add("@CameraId", DmDbType.VarChar, 64).Value = cameraIdentifier;
        plate.Parameters.Add("@Plate", DmDbType.VarChar, 32).Value = licensePlate;
        plate.Parameters.Add("@Capture", DmDbType.DateTime).Value = measureTime.AddSeconds(-2);
        plate.Parameters.Add("@Enter", DmDbType.VarChar, 64).Value = "TEST-" + Guid.NewGuid().ToString("N");
        plate.Parameters.Add("@Request", DmDbType.DateTime).Value = measureTime;
        plate.Parameters.Add("@Created", DmDbType.DateTime).Value = measureTime;
        var plateId = Convert.ToInt64(plate.ExecuteScalar());

        using var cmd = new DmCommand(@"
            INSERT INTO CoalVolumeRecord
            (IdempotencyKey, BusinessKey, PayloadHash, LicensePlate, CameraIdentifier,
             Volume, Unit, MeasureTime, Status, Confidence, CreatedAt,
             LicensePlateRecordId, MeasurementStartedAt, MeasurementEndedAt)
            VALUES
            (?, ?, ?, ?, ?,
             ?, ?, ?, ?, ?, ?, ?, ?, ?);
            SELECT SCOPE_IDENTITY()", conn, (DmTransaction)transaction);

        cmd.Parameters.Add("@IdempotencyKey", DmDbType.VarChar, 64).Value = idempotencyKey;
        cmd.Parameters.Add("@BusinessKey", DmDbType.VarChar, 64).Value = businessKey;
        cmd.Parameters.Add("@PayloadHash", DmDbType.VarChar, 64).Value = payloadHash;
        cmd.Parameters.Add("@LicensePlate", DmDbType.VarChar, 32).Value = licensePlate;
        cmd.Parameters.Add("@CameraIdentifier", DmDbType.VarChar, 64).Value = cameraIdentifier;
        cmd.Parameters.Add("@Volume", DmDbType.Decimal).Value = volume;
        cmd.Parameters["@Volume"].Precision = 10;
        cmd.Parameters["@Volume"].Scale = 1;
        cmd.Parameters.Add("@Unit", DmDbType.VarChar, 16).Value = unit;
        cmd.Parameters.Add("@MeasureTime", DmDbType.DateTime).Value = measureTime;
        cmd.Parameters.Add("@Status", DmDbType.VarChar, 32).Value = status?.ToLowerInvariant() ?? "success";
        AddNullableDecimal(cmd, "@Confidence", confidence, 5, 2);
        cmd.Parameters.Add("@CreatedAt", DmDbType.DateTime).Value = DateTime.Now;
        cmd.Parameters.Add("@PlateRecord", DmDbType.Int64).Value = plateId;
        cmd.Parameters.Add("@ActualStart", DmDbType.DateTime).Value = measureTime.AddSeconds(-1);
        cmd.Parameters.Add("@ActualEnd", DmDbType.DateTime).Value = measureTime;

        var result = Convert.ToInt64(cmd.ExecuteScalar());
        transaction.Commit();
        return result;
    }

    public void CleanupTestData()
    {
        using var conn = new DmConnection(_connectionString);
        conn.Open();

        // Delete child tables first
        using (var cmd = new DmCommand(@"
            DELETE FROM VehicleDimensionRib WHERE DimensionRecordId IN
            (SELECT Id FROM VehicleDimensionRecord WHERE LicensePlate LIKE 'TEST%' OR IdempotencyKey LIKE 'test-dim-%')", conn))
        {
            cmd.ExecuteNonQuery();
        }

        using (var cmd = new DmCommand(@"
            DELETE FROM VehicleDimensionWire WHERE DimensionRecordId IN
            (SELECT Id FROM VehicleDimensionRecord WHERE LicensePlate LIKE 'TEST%' OR IdempotencyKey LIKE 'test-dim-%')", conn))
        {
            cmd.ExecuteNonQuery();
        }

        using (var cmd = new DmCommand(@"
            DELETE FROM VehicleDimensionHighLowPlate WHERE DimensionRecordId IN
            (SELECT Id FROM VehicleDimensionRecord WHERE LicensePlate LIKE 'TEST%' OR IdempotencyKey LIKE 'test-dim-%')", conn))
        {
            cmd.ExecuteNonQuery();
        }

        // Delete main tables
        using (var cmd = new DmCommand("DELETE FROM VehicleDimensionRecord WHERE LicensePlate LIKE 'TEST%' OR IdempotencyKey LIKE 'test-dim-%'", conn))
        {
            cmd.ExecuteNonQuery();
        }

        using (var cmd = new DmCommand("DELETE FROM CoalVolumeRecord WHERE LicensePlate LIKE 'TEST%' OR IdempotencyKey LIKE 'test-coal-%'", conn))
        {
            cmd.ExecuteNonQuery();
        }

        using (var cmd = new DmCommand("DELETE FROM VehicleArrival WHERE DeviceId LIKE 'TEST%'", conn))
        {
            cmd.ExecuteNonQuery();
        }

        using (var cmd = new DmCommand("DELETE FROM LicensePlateRecord WHERE CameraIdentifier LIKE 'TEST%' OR InteractionId LIKE 'test-coal-identity-%'", conn))
        {
            cmd.ExecuteNonQuery();
        }

        using (var cmd = new DmCommand("DELETE FROM ApiAccessLog WHERE UserAgent = ?", conn))
        {
            cmd.Parameters.Add("@UserAgent", DmDbType.VarChar, 512).Value = _testUserAgent;
            cmd.ExecuteNonQuery();
        }
    }

    public int CountVehicleDimensionsByIdempotencyKey(string idempotencyKey)
    {
        return ExecuteCount(
            "SELECT COUNT(*) FROM VehicleDimensionRecord WHERE IdempotencyKey = ?",
            idempotencyKey);
    }

    public int CountCoalVolumesByIdempotencyKey(string idempotencyKey)
    {
        return ExecuteCount(
            "SELECT COUNT(*) FROM CoalVolumeRecord WHERE IdempotencyKey = ?",
            idempotencyKey);
    }

    private int ExecuteCount(string sql, string idempotencyKey)
    {
        using var conn = new DmConnection(_connectionString);
        conn.Open();
        using var cmd = new DmCommand(sql, conn);
        cmd.Parameters.Add("@IdempotencyKey", DmDbType.VarChar, 64).Value = idempotencyKey;
        return Convert.ToInt32(cmd.ExecuteScalar());
    }

    private static void AddNullableDecimal(DmCommand cmd, string name, decimal? value, byte precision = 10, byte scale = 1)
    {
        cmd.Parameters.Add(name, DmDbType.Decimal).Value = (object?)value ?? DBNull.Value;
        cmd.Parameters[name].Precision = precision;
        cmd.Parameters[name].Scale = scale;
    }

    public void Dispose()
    {
        CleanupTestData();
    }
}
