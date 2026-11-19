
namespace CarPointCloudWebApi.Infrastructure;

/// <summary>数据库关闭时使用的空车档案空对象实现，始终返回无数据。</summary>
public sealed class NullEmptyVehicleProfileRepository : IEmptyVehicleProfileRepository
{
    /// <inheritdoc />
    public EmptyVehicleProfileRecord? GetByLicensePlate(string licensePlate) => null;
}
