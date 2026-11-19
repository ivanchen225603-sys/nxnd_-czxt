using System.Runtime.InteropServices;
using Ladarclr;

namespace CarPointCloudLadarWorker.Radar;

/// <summary>
/// The small, platform-neutral surface used by the worker.  The Windows CLR
/// SDK and the Linux native SDK are deliberately hidden behind this boundary.
/// </summary>
internal abstract class SdkDevice
{
    protected SdkDevice(string address, ushort port)
    {
        Address = address;
        Port = port;
    }

    public string Address { get; }
    public ushort Port { get; }

    public abstract bool IsConnected();
    public abstract int Standby(ushort mode);
    public abstract int Parse118(IntPtr frame, int frameLength, out float3[] points);
}

internal delegate void SdkFrameHandler(SdkDevice device, IntPtr frame, int frameLength);
internal delegate void SdkStatusHandler(SdkDevice device, uint oldStatus, uint status);

internal interface ILadarSdkBackend : IDisposable
{
    SdkDevice AddLadar(string address, ushort port, SdkFrameHandler onFrame, SdkStatusHandler onStatus);
    void Run();
    void Stop();
}

#if !WINDOWS_LADAR_SDK
internal static class LinuxLadarBridge
{
    [StructLayout(LayoutKind.Sequential)]
    internal struct NativePoint
    {
        public float X;
        public float Y;
        public float Z;
    }

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void FrameCallback(
        IntPtr context,
        IntPtr device,
        IntPtr frame,
        int frameLength);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void StatusCallback(
        IntPtr context,
        IntPtr device,
        uint oldStatus,
        uint status);

    [DllImport("libladar_bridge.so", CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr bridge_create(
        FrameCallback frameCallback,
        StatusCallback statusCallback,
        IntPtr context);

    [DllImport("libladar_bridge.so", CallingConvention = CallingConvention.Cdecl)]
    internal static extern void bridge_destroy(IntPtr poller);

    [DllImport("libladar_bridge.so", CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr bridge_add_ladar(
        IntPtr poller,
        [MarshalAs(UnmanagedType.LPUTF8Str)] string address,
        ushort port);

    [DllImport("libladar_bridge.so", CallingConvention = CallingConvention.Cdecl)]
    internal static extern void bridge_run(IntPtr poller);

    [DllImport("libladar_bridge.so", CallingConvention = CallingConvention.Cdecl)]
    internal static extern void bridge_stop(IntPtr poller);

    [DllImport("libladar_bridge.so", CallingConvention = CallingConvention.Cdecl)]
    [return: MarshalAs(UnmanagedType.I1)]
    internal static extern bool bridge_is_connected(IntPtr device);

    [DllImport("libladar_bridge.so", CallingConvention = CallingConvention.Cdecl)]
    internal static extern int bridge_standby(IntPtr device, ushort mode);

    [DllImport("libladar_bridge.so", CallingConvention = CallingConvention.Cdecl)]
    internal static extern int bridge_parse118(
        IntPtr device,
        IntPtr frame,
        int frameLength,
        out IntPtr points,
        out int pointCount);

    [DllImport("libladar_bridge.so", CallingConvention = CallingConvention.Cdecl)]
    internal static extern void bridge_free_points(IntPtr points);
}
#endif
