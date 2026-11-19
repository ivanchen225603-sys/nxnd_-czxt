using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using Xunit;

namespace CarPointCloudWebApi.Tests;

/// <summary>2026-11-02：CH4 双雷达改由载货雷达位置停稳触发，CAM004 来车永久停用（不依赖任何开关）。</summary>
public sealed class Cam004ArrivalDisabledTests
{
    [Fact]
    public void Cam004ArrivalIsAcceptedWithoutRecordPlateJobOrScan()
    {
        var repository = new CountingRepository();
        var controller = new RadarController(repository,
            Options.Create(new IntegrationOptions { EnableMockLicensePlate = true, MockLicensePlatePrefix = "粤A" }),
            NullLogger<RadarController>.Instance)
        {
            ControllerContext = new ControllerContext { HttpContext = new DefaultHttpContext() }
        };

        var result = controller.VehicleArrival(new VehicleArrivalRequest
        {
            DeviceId = "TEST-UNIT",
            CameraIdentifier = "CAM004",
            Timestamp = "2026-01-01 12:00:00",
            EventType = "ARRIVAL",
            EventId = "TEST-CAM004-1"
        });

        var ok = Assert.IsType<OkObjectResult>(result.Result);
        var payload = Assert.IsType<ApiResponse<VehicleArrivalResponse>>(ok.Value);
        Assert.Equal("handled_by_load_radar", payload.Data!.Status);
        Assert.Equal(0, repository.Inserts);
        Assert.Equal(0, repository.RadarUpdates);
    }

    private sealed class CountingRepository : NullIntegrationRepository
    {
        public int Inserts;
        public int RadarUpdates;

        public override VehicleArrivalInsertResult InsertVehicleArrivalRecord(VehicleArrivalRecord record)
        {
            Inserts++;
            return new VehicleArrivalInsertResult { Record = record, IsNew = true };
        }

        public override void UpdateVehicleArrivalRadarResult(long recordId, string radarStatus, string? radarError, DateTime updatedAt) =>
            RadarUpdates++;
    }

}
