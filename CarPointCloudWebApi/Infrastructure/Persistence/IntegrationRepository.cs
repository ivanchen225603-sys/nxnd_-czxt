using System.Data;
using System.Data.Common;
using Dm;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>
/// 数据访问层仓储类。
/// </summary>
/// <remarks>
/// 职责：封装所有与 CarPointCloud 数据库的交互操作。
/// 数据库：达梦 DM8
/// 连接管理：每次操作创建独立连接，使用完毕后自动释放
/// 幂等性支持：车辆尺寸和煤炭体积入库支持幂等键，防止重复插入
/// </remarks>
public partial class IntegrationRepository : IIntegrationRepository
{
    private const string VehicleDimensionColumns = @"
        Id, IdempotencyKey, WorkflowId, PairId, PayloadHash, LicensePlate, CameraIdentifier, GetTime, DetectStatus,
        CabToTrailerFrontDist, TrailerLength, TrailerWidth, TrailerBodyHeight,
        TrailerBottomToGroundHeight, HighLowPlateHeight,
        SourceFile, ResultJsonPath, ResultImagePath,
        CreatedAt, UpdatedAt, ActualRibCount, MaximumWidthCm";

    private const string CoalVolumeColumns = @"
        Id, IdempotencyKey, BusinessKey, WorkflowId, PayloadHash, LicensePlate, CameraIdentifier, Volume, Unit, MeasureTime,
        LicensePlateRecordId, Status, Confidence, SourceFile, BaselineFile, CreatedAt,
        MeasurementStartedAt, MeasurementEndedAt,
        (SELECT TOP 1 p.EnterId FROM LicensePlateRecord p
         WHERE p.IsSuccess = 1 AND p.CameraIdentifier = CoalVolumeRecord.CameraIdentifier
           AND p.LicensePlate = CoalVolumeRecord.LicensePlate
           AND p.Id = CoalVolumeRecord.LicensePlateRecordId) AS EnterId,
        (SELECT TOP 1 p.CaptureTime FROM LicensePlateRecord p
         WHERE p.IsSuccess = 1 AND p.CameraIdentifier = CoalVolumeRecord.CameraIdentifier
           AND p.LicensePlate = CoalVolumeRecord.LicensePlate
           AND p.Id = CoalVolumeRecord.LicensePlateRecordId) AS CaptureTime";

    private readonly string _connectionString;
    private readonly int _commandTimeoutSeconds;
    private readonly int _captureLeadSeconds;
    private readonly int _captureWindowSeconds;
    private readonly int _captureSkewSeconds;
    private readonly ILogger<IntegrationRepository>? _logger;

    /// <summary>创建承载第三方集成、点云结果和来车流程持久化的达梦仓储。</summary>
    /// <param name="configuration">数据库连接串和 SQL 命令超时配置。</param>
    /// <param name="logger">记录已提交车牌的判断依据。</param>
    /// <exception cref="InvalidOperationException">数据库连接串缺失或为空。</exception>
    public IntegrationRepository(IConfiguration configuration, ILogger<IntegrationRepository>? logger = null)
    {
        _logger = logger;
        _connectionString = configuration.GetConnectionString("CarPointCloudDb")
            ?? throw new InvalidOperationException("缺少数据库连接字符串 ConnectionStrings:CarPointCloudDb");
        _commandTimeoutSeconds = configuration.GetValue<int>("Database:CommandTimeoutSeconds", 30);
        _captureLeadSeconds = Math.Clamp(configuration.GetValue("PlateAcquisition:CaptureLeadToleranceSeconds", 5), 0, 60);
        _captureWindowSeconds = Math.Clamp(configuration.GetValue("PlateAcquisition:DeadlineSeconds", 30), 1, 30);
        _captureSkewSeconds = Math.Clamp(configuration.GetValue("PlateAcquisition:ClockSkewToleranceSeconds", 5), 1, 60);

        if (string.IsNullOrWhiteSpace(_connectionString))
        {
            throw new InvalidOperationException("数据库连接字符串 ConnectionStrings:CarPointCloudDb 不能为空");
        }
    }

    private DmConnection CreateConnection()
    {
        return new DmConnection(_connectionString);
    }

    private static void ValidatePageArguments(ref int take, DateTime? beforeTime, long? beforeId)
    {
        take = Math.Clamp(take, 1, 101);
        if (beforeTime.HasValue != beforeId.HasValue || beforeId is <= 0)
        {
            throw new ArgumentException("分页锚点必须同时包含有效时间和记录 ID");
        }
    }

    private static decimal? GetNullableDecimal(DbDataReader reader, string name)
    {
        var ordinal = reader.GetOrdinal(name);
        return reader.IsDBNull(ordinal) ? null : reader.GetDecimal(ordinal);
    }

    private static long? GetNullableInt64(DbDataReader reader, string name)
    {
        var ordinal = reader.GetOrdinal(name);
        return reader.IsDBNull(ordinal) ? null : reader.GetInt64(ordinal);
    }

    private static string? GetNullableString(DbDataReader reader, string name)
    {
        var ordinal = reader.GetOrdinal(name);
        return reader.IsDBNull(ordinal) ? null : reader.GetString(ordinal);
    }

    private static void AddNullableDecimal(DmCommand cmd, string name, decimal? value, byte precision = 10, byte scale = 1)
    {
        cmd.Parameters.Add(name, DmDbType.Decimal).Value = (object?)value ?? DBNull.Value;
        cmd.Parameters[name].Precision = precision;
        cmd.Parameters[name].Scale = scale;
    }

    /// <summary>
    /// 达梦 VARCHAR 原生支持 Unicode（UTF-8 字符集），无需区分 VarChar/NVarChar。
    /// </summary>
    private static void AddVarcharParam(DmCommand cmd, string name, int size, string? value, bool required = false)
    {
        cmd.Parameters.Add(name, DmDbType.VarChar, size).Value = ToSizedValue(value, size, required);
    }

    /// <summary>
    /// 添加大文本参数（对应 SQL Server NVARCHAR(MAX)，达梦使用 CLOB/TEXT 列存储）。
    /// </summary>
    private static void AddTextParam(DmCommand cmd, string name, string? value)
    {
        var p = cmd.CreateParameter();
        p.ParameterName = name;
        p.DbType = DbType.String;
        p.Value = string.IsNullOrEmpty(value) ? DBNull.Value : value;
        cmd.Parameters.Add(p);
    }

    private static object ToSizedValue(string? value, int size, bool required)
    {
        if (string.IsNullOrEmpty(value))
        {
            return required ? string.Empty : DBNull.Value;
        }

        return value.Length <= size ? value : value[..size];
    }

    /// <summary>
    /// 达梦唯一约束冲突错误码：-6602。
    /// </summary>
    private static bool IsUniqueConstraintViolation(DmException ex)
    {
        return ex.Number == -6602;
    }

    private sealed record IdempotencyIdentity(long Id, string? PayloadHash);

    private static void TryRollback(DbTransaction tx)
    {
        try
        {
            tx.Rollback();
        }
        catch
        {
            // Preserve the original database exception.
        }
    }
}
