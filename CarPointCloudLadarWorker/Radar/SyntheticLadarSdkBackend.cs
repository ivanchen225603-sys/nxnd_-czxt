using System.Runtime.InteropServices;
using CarPointCloudLadarWorker.Config;
using Ladarclr;

namespace CarPointCloudLadarWorker.Radar;

/// <summary>
/// 联调用的合成雷达：按固定间隔为每台设备产生一场点云，经与真实 SDK 相同的 cmd118 回调路径进入 Worker。
/// </summary>
internal sealed class SyntheticLadarSdkBackend(LadarCaptureOptions options) : ILadarSdkBackend
{
    private readonly List<(SyntheticDevice Device, SdkFrameHandler OnFrame)> _devices = [];
    private readonly ManualResetEventSlim _stop = new();

    /// <summary>只含命令字 118（偏移 2，小端）的假帧头。</summary>
    private readonly IntPtr _frame = CreateFrameHeader();

    public SdkDevice AddLadar(string address, ushort port, SdkFrameHandler onFrame, SdkStatusHandler onStatus)
    {
        var device = new SyntheticDevice(address, port);
        _devices.Add((device, onFrame));
        return device;
    }

    public void Run()
    {
        var interval = TimeSpan.FromSeconds(options.SyntheticFrameIntervalSeconds);
        for (long sequence = 1; !_stop.Wait(interval); sequence++)
        {
            for (var i = 0; i < _devices.Count; i++)
            {
                var (device, onFrame) = _devices[i];
                device.Next = CreateVehicleFrame(sequence, i, options.SyntheticPointCount);
                onFrame(device, _frame, 4);
                if (i + 1 < _devices.Count)
                    _stop.Wait(options.SyntheticInterDeviceDelayMilliseconds);
            }
        }
    }

    public void Stop() => _stop.Set();

    public void Dispose() => Marshal.FreeHGlobal(_frame);

    private static IntPtr CreateFrameHeader()
    {
        var frame = Marshal.AllocHGlobal(4);
        Marshal.WriteInt16(frame, 0, 0);
        Marshal.WriteInt16(frame, 2, 118);
        return frame;
    }

    /// <summary>生成一场车辆点云；点数 ≥ 50000 时生成带底板、四壁与车头的完整车厢场景。</summary>
    internal static float3[] CreateVehicleFrame(long sequence, int deviceIndex, int pointCount)
    {
        var points = new float3[pointCount];
        if (pointCount >= 50_000)
        {
            FillTruckScene(points, sequence, deviceIndex);
            return points;
        }

        var longitudinalOffset = (sequence % 31) * 0.03f;
        var lateralOffset = deviceIndex == 0 ? -0.8f : 0.8f;
        for (var i = 0; i < pointCount; i++)
        {
            var angle = (float)(i * 0.61803398875 + deviceIndex * 0.25) * MathF.PI * 2f;
            var isVehicle = i % 5 != 0;
            points[i] = new float3
            {
                x = longitudinalOffset + (isVehicle ? 1f : 4f) * MathF.Cos(angle),
                y = lateralOffset + (isVehicle ? 1.25f : 3.5f) * MathF.Sin(angle),
                z = isVehicle ? 1.1f + 0.65f * MathF.Sin(angle * 2f) : 0.02f * MathF.Sin(angle * 3f)
            };
        }
        return points;
    }

    private static void FillTruckScene(float3[] points, long sequence, int deviceIndex)
    {
        // 两台雷达共用场景随机数，只有噪声不同，保证 B_to_A 外参下两份点云描述同一辆车。
        var seed = unchecked((int)sequence) ^ 0x2a6f31c5;
        var random = new Random(seed);
        var noise = new Random(seed ^ (deviceIndex + 1) * 7919);
        float Between(Random r, double min, double max) => (float)(min + (max - min) * r.NextDouble());

        // 依次为：地面、左右侧壁、前后端壁、底板、车头、底盘，按点数比例分配。
        (double Until, double X0, double X1, double Y0, double Y1, double Z0, double Z1)[] parts =
        [
            (0.25, -18, 18, -8, 8, -0.025, 0.025),
            (0.33, -6, 6, -1.25, -1.20, 1.45, 3.35),
            (0.41, -6, 6, 1.20, 1.25, 1.45, 3.35),
            (0.56, -6.03, -5.97, -1.25, 1.25, 1.45, 3.35),
            (0.71, 5.97, 6.03, -1.25, 1.25, 1.45, 3.35),
            (0.89, -5.9, 5.9, -1.15, 1.15, 1.48, 1.52),
            (0.98, 6.1, 8.7, -1.05, 1.05, 0.65, 2.35),
            (1.00, -6, 8.7, -0.9, 0.9, 0.4, 0.8)
        ];
        for (var i = 0; i < points.Length; i++)
        {
            var fraction = (double)i / points.Length;
            var part = parts.First(p => fraction < p.Until);
            points[i] = new float3
            {
                x = Between(random, part.X0, part.X1) + Between(noise, -0.003, 0.003),
                y = Between(random, part.Y0, part.Y1) + Between(noise, -0.003, 0.003),
                z = Between(random, part.Z0, part.Z1) + Between(noise, -0.003, 0.003)
            };
        }
    }

    private sealed class SyntheticDevice(string address, ushort port) : SdkDevice(address, port)
    {
        public float3[] Next { get; set; } = [];

        public override bool IsConnected() => true;

        public override int Standby(ushort mode) => 1;

        public override int Parse118(IntPtr frame, int frameLength, out float3[] points)
        {
            points = Next;
            return 1;
        }
    }
}
