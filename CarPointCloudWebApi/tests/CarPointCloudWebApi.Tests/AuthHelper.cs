using System.Security.Cryptography;
using System.Text;

namespace CarPointCloudWebApi.Tests;

public static class AuthHelper
{
    public static (string Timestamp, string Signature) GenerateAuthHeaders()
    {
        var timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds().ToString();
        var signature = GenerateSignature(timestamp, TestConfiguration.AppSecret);
        return (timestamp, signature);
    }

    public static string GenerateSignature(string timestamp, string appSecret)
    {
        var plainText = timestamp + appSecret;
        var hash = MD5.HashData(Encoding.UTF8.GetBytes(plainText));
        return Convert.ToHexString(hash).ToLowerInvariant();
    }
}
