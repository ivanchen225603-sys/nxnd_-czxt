
namespace CarPointCloudWebApi.Infrastructure;

public interface ILoadRadarPassageRepository
{
    LoadPassageStartRecord RegisterPassageStart(LoadPassageStartRecord record, VehicleArrivalRecord arrival,
        int registrationMaxSeconds) => throw new LoadRadarException(503, "到车原子登记未启用");
    LoadPassageStartRecord? GetPassageStart(string key);
    LoadPassageBinding BindStartedPassage(LoadPassageStartRecord record, LoadPassageBinding binding);
}
