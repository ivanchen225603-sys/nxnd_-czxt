using System.Security.Cryptography;
using System.Text.RegularExpressions;

namespace CarPointCloudWebApi.Application;

/// <summary>
/// 为未接入真实车牌系统的演示环境生成格式受控的随机模拟车牌。
/// </summary>
/// <remarks>仅在显式启用模拟车牌配置时使用，生成值不得作为真实识别结果对外对账。</remarks>
public static partial class MockLicensePlateGenerator
{
    private const string Alphabet = "ABCDEFGHJKLMNPQRSTUVWXYZ0123456789";

    /// <summary>使用指定地域或业务前缀生成五位随机后缀的模拟车牌。</summary>
    /// <param name="prefix">一至四个不含控制字符的车牌前缀。</param>
    /// <returns>前缀与密码学随机后缀拼接的模拟车牌。</returns>
    /// <exception cref="ArgumentException">前缀为空、超长或包含控制字符。</exception>
    public static string Generate(string prefix)
    {
        var normalizedPrefix = prefix?.Trim() ?? string.Empty;
        if (!PrefixRegex().IsMatch(normalizedPrefix))
        {
            throw new ArgumentException("模拟车牌前缀必须是 1-4 个非控制字符", nameof(prefix));
        }

        Span<char> suffix = stackalloc char[5];
        for (var index = 0; index < suffix.Length; index++)
        {
            suffix[index] = Alphabet[RandomNumberGenerator.GetInt32(Alphabet.Length)];
        }

        return normalizedPrefix + new string(suffix);
    }

    [GeneratedRegex(@"^[^\p{Cc}\p{Cs}]{1,4}$")]
    private static partial Regex PrefixRegex();
}
