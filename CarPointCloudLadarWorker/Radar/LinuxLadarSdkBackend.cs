#if !WINDOWS_LADAR_SDK
using System.Runtime.InteropServices;
using Ladarclr;

namespace CarPointCloudLadarWorker.Radar;

/// <summary>Adapter for the vendor's Linux C++ SDK through libladar_bridge.so.</summary>
internal sealed class LinuxLadarSdkBackend : ILadarSdkBackend
{
    private readonly LinuxLadarBridge.FrameCallback _frameCallback;
    private readonly LinuxLadarBridge.StatusCallback _statusCallback;

    /// <remarks>
    /// 只在 <see cref="Run"/> 之前由单线程写入，之后仅由原生回调线程读取。
    /// <see cref="Dictionary{TKey,TValue}"/> 不支持并发读写，这个"写完再读"的时序
    /// 是它唯一的安全保障，因此 <see cref="AddLadar"/> 在 Run 之后会直接抛异常，
    /// 而不是让将来的热插拔改动悄悄制造出难以复现的字典损坏。
    /// </remarks>
    private readonly Dictionary<IntPtr, LinuxSdkDevice> _devices = [];
    private readonly GCHandle _selfHandle;
    private IntPtr _poller;
    private volatile bool _running;
    private bool _disposed;

    public LinuxLadarSdkBackend()
    {
        _frameCallback = OnFrame;
        _statusCallback = OnStatus;
        _selfHandle = GCHandle.Alloc(this);
        try
        {
            _poller = LinuxLadarBridge.bridge_create(
                _frameCallback,
                _statusCallback,
                GCHandle.ToIntPtr(_selfHandle));
            if (_poller == IntPtr.Zero)
            {
                throw new InvalidOperationException(
                    "无法创建 Linux 雷达 SDK poller。请确认 libladar_bridge.so 和 libladarsdk_gd.so 已安装。\n" +
                    "Cannot create Linux radar SDK poller; check the native SDK files.");
            }
        }
        catch
        {
            _selfHandle.Free();
            throw;
        }
    }

    public SdkDevice AddLadar(
        string address,
        ushort port,
        SdkFrameHandler onFrame,
        SdkStatusHandler onStatus)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        ArgumentNullException.ThrowIfNull(onFrame);
        ArgumentNullException.ThrowIfNull(onStatus);
        if (_running)
        {
            throw new InvalidOperationException(
                "Run() 之后不能再 AddLadar：设备字典此时正被原生回调线程并发读取。" +
                "如需支持运行期增删设备，必须先把 _devices 换成并发安全的容器。");
        }

        var handle = LinuxLadarBridge.bridge_add_ladar(_poller, address, port);
        if (handle == IntPtr.Zero)
        {
            throw new InvalidOperationException($"Linux AddLadar failed: {address}:{port}");
        }

        var device = new LinuxSdkDevice(handle, address, port, onFrame, onStatus);
        _devices[handle] = device;
        return device;
    }

    public void Run()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        // 必须在进入原生循环之前置位：bridge_run 一旦开始投递回调，
        // _devices 就只能读不能写了。
        _running = true;
        LinuxLadarBridge.bridge_run(_poller);
    }

    public void Stop()
    {
        if (!_disposed && _poller != IntPtr.Zero)
        {
            LinuxLadarBridge.bridge_stop(_poller);
        }
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _disposed = true;
        if (_poller != IntPtr.Zero)
        {
            LinuxLadarBridge.bridge_destroy(_poller);
            _poller = IntPtr.Zero;
        }
        _selfHandle.Free();
        _devices.Clear();
    }

    private static LinuxLadarSdkBackend? FromContext(IntPtr context)
    {
        if (context == IntPtr.Zero)
        {
            return null;
        }
        return GCHandle.FromIntPtr(context).Target as LinuxLadarSdkBackend;
    }

    private static void OnFrame(
        IntPtr context,
        IntPtr nativeDevice,
        IntPtr frame,
        int frameLength)
    {
        var backend = FromContext(context);
        if (backend is not null && backend._devices.TryGetValue(nativeDevice, out var device))
        {
            device.FrameHandler(frame, frameLength);
        }
    }

    private static void OnStatus(
        IntPtr context,
        IntPtr nativeDevice,
        uint oldStatus,
        uint status)
    {
        var backend = FromContext(context);
        if (backend is not null && backend._devices.TryGetValue(nativeDevice, out var device))
        {
            device.StatusHandler(oldStatus, status);
        }
    }

    private sealed class LinuxSdkDevice : SdkDevice
    {
        private readonly IntPtr _handle;
        private readonly SdkFrameHandler _onFrame;
        private readonly SdkStatusHandler _onStatus;

        public LinuxSdkDevice(
            IntPtr handle,
            string address,
            ushort port,
            SdkFrameHandler onFrame,
            SdkStatusHandler onStatus)
            : base(address, port)
        {
            _handle = handle;
            _onFrame = onFrame;
            _onStatus = onStatus;
        }

        public void FrameHandler(IntPtr frame, int frameLength) =>
            _onFrame(this, frame, frameLength);

        public void StatusHandler(uint oldStatus, uint status) =>
            _onStatus(this, oldStatus, status);

        public override bool IsConnected() =>
            LinuxLadarBridge.bridge_is_connected(_handle);


        public override int Standby(ushort mode) =>
            LinuxLadarBridge.bridge_standby(_handle, mode);

        public override unsafe int Parse118(
            IntPtr frame,
            int frameLength,
            out float3[] points)
        {
            points = [];
            var result = LinuxLadarBridge.bridge_parse118(
                _handle,
                frame,
                frameLength,
                out var nativePoints,
                out var pointCount);
            if (result <= 0 || nativePoints == IntPtr.Zero || pointCount <= 0)
            {
                return result;
            }

            try
            {
                var source = (LinuxLadarBridge.NativePoint*)nativePoints;
                points = new float3[pointCount];
                for (var i = 0; i < pointCount; i++)
                {
                    points[i] = new float3
                    {
                        x = source[i].X,
                        y = source[i].Y,
                        z = source[i].Z
                    };
                }
            }
            finally
            {
                LinuxLadarBridge.bridge_free_points(nativePoints);
            }

            return result;
        }
    }
}
#endif
