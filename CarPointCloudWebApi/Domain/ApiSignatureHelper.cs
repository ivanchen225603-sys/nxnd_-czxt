using System.Security.Cryptography;
using System.Text;

namespace CarPointCloudWebApi.Domain;

/// <summary>
/// API 签名工具类。
/// </summary>
/// <remarks>
/// 签名规则来自接口需求：Signature = MD5(Timestamp + AppSecret)，MD5 输出必须为小写 32 位十六进制字符串。
/// Timestamp 使用 Unix 毫秒时间戳，调用方和服务端需保持时钟同步。
/// </remarks>
public static class ApiSignatureHelper
{
    /// <summary>
    /// 根据毫秒级时间戳和应用密钥生成接口签名。
    /// </summary>
    /// <param name="timestamp">Unix 毫秒级时间戳字符串。</param>
    /// <param name="appSecret">双方约定的应用密钥。</param>
    /// <returns>小写 MD5 签名字符串。</returns>
    /// <exception cref="ArgumentNullException">当入参为 null 且底层字符串拼接或编码不可处理时抛出。</exception>
    public static string GenerateSignature(string timestamp, string appSecret)
    {
        var plainText = timestamp + appSecret;
        var hash = MD5.HashData(Encoding.UTF8.GetBytes(plainText));
        return Convert.ToHexString(hash).ToLowerInvariant();
    }
}
