namespace CarPointCloudWebApi.Domain;

/// <summary>
/// 人工输入车牌的规范化与校验。
/// </summary>
/// <remarks>
/// 刻意比 <c>IntegrationQueryService</c> 的车牌规则更严。那条规则服务于合作方系统之间的
/// 机器对接，只拒绝控制字符和超长；这里面对的是磅房操作员的键盘输入，必须额外挡住占位词
/// 和内部空白——Analyzer 落库时对车牌只检查长度和控制字符，一个输入进来的 <c>UNKNOWN</c>
/// 会被当成真实车牌写进业务表，并随合作方查询接口发出去。
/// </remarks>
public static class ManualLicensePlate
{
    /// <summary>车牌最大长度，与 DM8 的 VARCHAR(32) 一致。</summary>
    public const int MaxLength = 32;

    /// <summary>被拒绝的占位车牌。</summary>
    private static readonly HashSet<string> RejectedPlates = new(StringComparer.OrdinalIgnoreCase)
    {
        "UNKNOWN", "NULL", "N/A", "NA", "NONE", "无车牌", "未知", "未知车牌"
    };

    /// <summary>
    /// 规范化并校验人工输入的车牌。
    /// </summary>
    /// <param name="raw">操作员输入的原始文本。</param>
    /// <param name="plate">规范化后的车牌（去掉全部空白并转为大写）。</param>
    /// <param name="error">校验失败原因；成功时为空字符串。</param>
    /// <returns>校验通过返回 <see langword="true"/>。</returns>
    /// <remarks>
    /// 去掉的是<b>全部</b>空白而不只是首尾：车牌中间的空格来自输入习惯，不是车牌的一部分，
    /// 留着会让同一辆车在数据库里出现两种写法，空车档案也会因此分裂成两条。
    /// </remarks>
    public static bool TryNormalize(string? raw, out string plate, out string error)
    {
        plate = string.Empty;
        error = string.Empty;

        if (string.IsNullOrWhiteSpace(raw))
        {
            error = "licensePlate 不能为空";
            return false;
        }

        var normalized = string.Concat(raw.Where(character => !char.IsWhiteSpace(character)))
            .ToUpperInvariant();

        if (normalized.Length == 0)
        {
            error = "licensePlate 不能只包含空白字符";
            return false;
        }

        if (normalized.Length > MaxLength)
        {
            error = $"licensePlate 长度不能超过 {MaxLength}";
            return false;
        }

        if (normalized.Any(char.IsControl))
        {
            error = "licensePlate 不能包含控制字符";
            return false;
        }

        if (RejectedPlates.Contains(normalized))
        {
            error = "licensePlate 不能是占位值，请输入真实车牌";
            return false;
        }

        plate = normalized;
        return true;
    }
}
