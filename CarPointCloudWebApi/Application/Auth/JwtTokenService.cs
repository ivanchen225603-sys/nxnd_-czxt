using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Application;

/// <summary>appsettings.json 的 <c>Auth</c> 段。</summary>
public class JwtOptions
{
    public const string SectionName = "Auth";
    public string JwtSecret { get; set; } = string.Empty;
    public int TokenExpireMinutes { get; set; } = 480;
    public string Issuer { get; set; } = "CarPointCloudWebApi";
    public string Audience { get; set; } = "CarPointCloudViewer";
}

/// <summary>令牌中携带的登录用户。</summary>
public class TokenUserInfo
{
    public long UserId { get; set; }
    public string Username { get; set; } = string.Empty;
    public string RoleCode { get; set; } = string.Empty;
}

/// <summary>HS256 JWT 的签发与校验。</summary>
public class JwtTokenService(IOptions<JwtOptions> options)
{
    private readonly JwtOptions _options = options.Value;

    public int TokenExpireMinutes => _options.TokenExpireMinutes;

    public string IssueToken(TokenUserInfo user, DateTime nowUtc)
    {
        var header = Base64UrlEncode(JsonSerializer.SerializeToUtf8Bytes(new { alg = "HS256", typ = "JWT" }));
        var payload = Base64UrlEncode(JsonSerializer.SerializeToUtf8Bytes(new
        {
            sub = user.UserId.ToString(),
            name = user.Username,
            role = user.RoleCode,
            iss = _options.Issuer,
            aud = _options.Audience,
            iat = new DateTimeOffset(nowUtc).ToUnixTimeSeconds(),
            exp = new DateTimeOffset(nowUtc.AddMinutes(_options.TokenExpireMinutes)).ToUnixTimeSeconds()
        }));
        return $"{header}.{payload}.{Sign($"{header}.{payload}")}";
    }

    /// <summary>签名正确、未过期、签发方与受众一致时返回用户，否则返回 null。</summary>
    public TokenUserInfo? ValidateToken(string token, DateTime nowUtc)
    {
        var parts = token.Split('.');
        if (parts.Length != 3 || Sign($"{parts[0]}.{parts[1]}") != parts[2])
            return null;

        using var document = JsonDocument.Parse(Base64UrlDecode(parts[1]));
        var root = document.RootElement;
        if (root.GetProperty("exp").GetInt64() <= new DateTimeOffset(nowUtc).ToUnixTimeSeconds() ||
            root.GetProperty("iss").GetString() != _options.Issuer ||
            root.GetProperty("aud").GetString() != _options.Audience)
            return null;

        return new TokenUserInfo
        {
            UserId = long.Parse(root.GetProperty("sub").GetString()!),
            Username = root.GetProperty("name").GetString() ?? string.Empty,
            RoleCode = root.GetProperty("role").GetString() ?? string.Empty
        };
    }

    private string Sign(string input) =>
        Base64UrlEncode(HMACSHA256.HashData(Encoding.UTF8.GetBytes(_options.JwtSecret), Encoding.UTF8.GetBytes(input)));

    private static string Base64UrlEncode(byte[] bytes) =>
        Convert.ToBase64String(bytes).TrimEnd('=').Replace('+', '-').Replace('/', '_');

    private static byte[] Base64UrlDecode(string value)
    {
        var padded = value.Replace('-', '+').Replace('_', '/');
        return Convert.FromBase64String(padded.PadRight(padded.Length + (4 - padded.Length % 4) % 4, '='));
    }
}
