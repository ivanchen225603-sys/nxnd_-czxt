#if WINDOWS_LADAR_SDK
using Ladarclr;

namespace CarPointCloudLadarWorker.Radar;

/// <summary>Adapter for the existing Windows CLR wrapper.</summary>
internal sealed unsafe class WindowsLadarSdkBackend : ILadarSdkBackend
{
    private readonly LadarPoller _poller = new();

    /// <summary>
    /// 持有设备及其专属的原生回调委托，避免委托被 GC，也避免依赖
    /// SDK 回调传回的 CLR wrapper 引用与 AddLadar 返回引用完全相同。
    /// </summary>
    private readonly List<WindowsSdkDevice> _devices = [];
    private volatile bool _running;

    public SdkDevice AddLadar(
        string address,
        ushort port,
        SdkFrameHandler onFrame,
        SdkStatusHandler onStatus)
    {
        ArgumentNullException.ThrowIfNull(onFrame);
        ArgumentNullException.ThrowIfNull(onStatus);
        if (_running)
        {
            throw new InvalidOperationException(
                "Run() 之后不能再 AddLadar：设备字典此时正被 SDK 回调线程并发读取。" +
                "如需支持运行期增删设备，必须先把 _devices 换成并发安全的容器。");
        }

        WindowsSdkDevice? device = null;
        LadarFrameHandlerDelegate frameHandler = (_, frame, frameLength) =>
            device?.FrameHandler((IntPtr)frame, frameLength);
        LadarStatusHandlerDelegate statusHandler = (_, oldStatus, status) =>
            device?.StatusHandler((uint)oldStatus, (uint)status);

        var raw = _poller.AddLadar(address, port, frameHandler, statusHandler);
        if (raw is null)
        {
            throw new InvalidOperationException($"AddLadar failed: {address}:{port}");
        }

        // AddLadar 只注册设备，Run 之后才会投递回调，因此闭包在首个回调前必然已绑定。
        device = new WindowsSdkDevice(
            raw,
            address,
            port,
            onFrame,
            onStatus,
            frameHandler,
            statusHandler);
        _devices.Add(device);
        return device;
    }

    public void Run()
    {
        // 必须在进入 SDK 循环之前置位：Run 一旦开始投递回调，_devices 就只能读不能写。
        _running = true;
        _poller.Run();
    }

    public void Stop() => _poller.Stop();

    public void Dispose() => _poller.Dispose();

    private sealed class WindowsSdkDevice : SdkDevice
    {
        private readonly Ladar _raw;
        private readonly SdkFrameHandler _onFrame;
        private readonly SdkStatusHandler _onStatus;
        // SDK 只保存函数指针；托管侧必须持有每台设备的委托直至 poller Dispose。
        private readonly LadarFrameHandlerDelegate _nativeFrameHandler;
        private readonly LadarStatusHandlerDelegate _nativeStatusHandler;

        public WindowsSdkDevice(
            Ladar raw,
            string address,
            ushort port,
            SdkFrameHandler onFrame,
            SdkStatusHandler onStatus,
            LadarFrameHandlerDelegate nativeFrameHandler,
            LadarStatusHandlerDelegate nativeStatusHandler)
            : base(address, port)
        {
            _raw = raw;
            _onFrame = onFrame;
            _onStatus = onStatus;
            _nativeFrameHandler = nativeFrameHandler;
            _nativeStatusHandler = nativeStatusHandler;
        }

        public void FrameHandler(IntPtr frame, int frameLength) =>
            _onFrame(this, frame, frameLength);

        public void StatusHandler(uint oldStatus, uint status) =>
            _onStatus(this, oldStatus, status);

        public override bool IsConnected() => _raw.IsConnected();


        public override int Standby(ushort mode) => _raw.Standby(mode);

        public override unsafe int Parse118(
            IntPtr frame,
            int frameLength,
            out float3[] points)
        {
            points = [];
            var result = _raw.Parse118(
                out var data,
                (byte*)frame,
                frameLength);
            if (result <= 0 || data.points is null || data.points.Count == 0)
            {
                return result;
            }
            points = data.points
                .Select(point => new float3 { x = point.x, y = point.y, z = point.z })
                .ToArray();
            return result;
        }
    }
}
#endif
