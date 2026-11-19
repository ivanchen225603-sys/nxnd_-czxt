using System.Security.Cryptography;

namespace CarPointCloudWebApi.Application;

/// <summary>
/// 密码哈希工具（PBKDF2-SHA256 加盐）。
/// </summary>
/// <remarks>
/// 安全说明：
/// - 使用 100000 次 PBKDF2 迭代 + 16 字节随机盐，抗暴力破解；
/// - 哈希与盐均为 Base64 存储，数据库中即使泄露也无法还原明文；
/// - 校验使用固定时间比较，防止时序攻击。
/// </remarks>
public static class PasswordHasher
{
    /// <summary>PBKDF2 迭代次数。</summary>
    private const int Iterations = 100_000;

    /// <summary>盐字节数。</summary>
    private const int SaltSize = 16;

    /// <summary>哈希字节数。</summary>
    private const int HashSize = 32;

    /// <summary>
    /// 生成密码哈希。
    /// </summary>
    /// <param name="password">明文密码。</param>
    /// <returns>哈希与盐的元组；Hash 与 Salt 均为 Base64 字符串。</returns>
    /// <exception cref="ArgumentException">密码为空时抛出。</exception>
    public static (string Hash, string Salt) HashPassword(string password)
    {
        if (string.IsNullOrEmpty(password))
        {
            throw new ArgumentException("密码不能为空", nameof(password));
        }

        var salt = RandomNumberGenerator.GetBytes(SaltSize);
        var hash = Rfc2898DeriveBytes.Pbkdf2(
            password,
            salt,
            Iterations,
            HashAlgorithmName.SHA256,
            HashSize);

        return (Convert.ToBase64String(hash), Convert.ToBase64String(salt));
    }

    /// <summary>
    /// 校验密码。
    /// </summary>
    /// <param name="password">待校验的明文密码。</param>
    /// <param name="storedHash">数据库中的哈希（Base64）。</param>
    /// <param name="storedSalt">数据库中的盐（Base64）。</param>
    /// <returns>密码匹配返回 true，否则 false。任一参数无效返回 false。</returns>
    public static bool VerifyPassword(string password, string storedHash, string storedSalt)
    {
        if (string.IsNullOrEmpty(password) ||
            string.IsNullOrEmpty(storedHash) ||
            string.IsNullOrEmpty(storedSalt))
        {
            return false;
        }

        try
        {
            var salt = Convert.FromBase64String(storedSalt);
            var expectedHash = Convert.FromBase64String(storedHash);
            var actualHash = Rfc2898DeriveBytes.Pbkdf2(
                password,
                salt,
                Iterations,
                HashAlgorithmName.SHA256,
                expectedHash.Length);

            return CryptographicOperations.FixedTimeEquals(actualHash, expectedHash);
        }
        catch (FormatException)
        {
            // 数据库中存储的哈希/盐格式非法，按不匹配处理
            return false;
        }
    }
}
