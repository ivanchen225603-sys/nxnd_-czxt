#if !WINDOWS_LADAR_SDK
namespace Ladarclr;

/// <summary>
/// Linux build compatibility type matching the vendor CLR wrapper's float3.
/// The Linux native bridge converts vendor float3_t values into this shape.
/// </summary>
public struct float3
{
    public float x;
    public float y;
    public float z;
}
#endif
