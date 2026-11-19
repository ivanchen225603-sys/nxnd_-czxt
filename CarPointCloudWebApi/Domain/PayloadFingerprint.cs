using System.Security.Cryptography;
using System.Text.Json;

namespace CarPointCloudWebApi.Domain;

/// <summary>
/// 生成规范化业务载荷的内部 SHA-256 指纹，用于检测幂等键误复用。
/// </summary>
public static class PayloadFingerprint
{
    private static readonly JsonSerializerOptions SerializerOptions = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase
    };

    /// <summary>
    /// 将业务对象按统一 JSON 命名规则序列化并计算 SHA-256 指纹。
    /// </summary>
    /// <typeparam name="T">业务载荷类型。</typeparam>
    /// <param name="value">需要绑定到幂等键的业务载荷。</param>
    /// <returns>小写十六进制 SHA-256 指纹。</returns>
    /// <remarks>指纹仅用于判断载荷是否一致，不能替代第三方 MD5 鉴权签名。</remarks>
    public static string Compute<T>(T value)
    {
        var bytes = JsonSerializer.SerializeToUtf8Bytes(value, SerializerOptions);
        return Convert.ToHexString(SHA256.HashData(bytes)).ToLowerInvariant();
    }
}
