
namespace CarPointCloudWebApi.Infrastructure;

/// <summary>定义按车牌读取最新有效空车融合点云档案的持久化契约。</summary>
public interface IEmptyVehicleProfileRepository
{
    /// <summary>查询指定车牌当前唯一的空车基准档案。</summary>
    /// <param name="licensePlate">已规范化的车牌号。</param>
    /// <returns>空车档案及尺寸明细；没有有效档案时返回 <see langword="null"/>。</returns>
    EmptyVehicleProfileRecord? GetByLicensePlate(string licensePlate);

    /// <summary>批量读取展示装载率所需的车厢内尺寸。</summary>
    IReadOnlyDictionary<string, EmptyVehicleProfileRecord> GetDimensionsByLicensePlates(IReadOnlyList<string> licensePlates) =>
        licensePlates.Select(GetByLicensePlate).OfType<EmptyVehicleProfileRecord>()
            .ToDictionary(profile => profile.LicensePlate, StringComparer.Ordinal);
}
