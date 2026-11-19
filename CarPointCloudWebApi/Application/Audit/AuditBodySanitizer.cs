using System.Text.Json;
using System.Text.Json.Nodes;

namespace CarPointCloudWebApi.Application;

/// <summary>正文在截断前递归脱敏；非 JSON 或损坏正文不原样写入审计库。</summary>
public static class AuditBodySanitizer
{
    /// <summary>移除凭据字段，包括嵌套对象和数组，保留业务诊断结构。</summary>
    public static string? Sanitize(string? text)
    {
        if (string.IsNullOrWhiteSpace(text)) return text;
        try
        {
            var node = JsonNode.Parse(text);
            Redact(node);
            return node is JsonObject or JsonArray ? node.ToJsonString(new JsonSerializerOptions
            {
                Encoder = System.Text.Encodings.Web.JavaScriptEncoder.UnsafeRelaxedJsonEscaping
            }) : "[non-object body omitted]";
        }
        catch (Exception error) when (error is JsonException or ArgumentException) { return "[unparseable body omitted]"; }
    }

    internal static bool IsSensitiveName(string key)
    {
        var normalized = new string(key.Where(char.IsLetterOrDigit).ToArray()).ToLowerInvariant();
        return new[] { "password", "passwd", "pwd", "secret", "token", "signature", "authorization",
            "cookie", "connectionstring", "credential", "apikey", "privatekey" }.Any(normalized.Contains);
    }

    private static void Redact(JsonNode? node)
    {
        if (node is JsonObject obj)
        {
            foreach (var key in obj.Select(entry => entry.Key).ToArray())
            {
                if (IsSensitiveName(key)) obj[key] = "***REDACTED***";
                else Redact(obj[key]);
            }
        }
        else if (node is JsonArray array)
            foreach (var item in array) Redact(item);
    }
}
