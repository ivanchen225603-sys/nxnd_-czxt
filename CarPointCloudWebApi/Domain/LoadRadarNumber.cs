using System.Globalization;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace CarPointCloudWebApi.Domain;

/// <summary>机器测量数值：兼容旧 decimal 字节形式，并保留 Python 有限 double 的完整精度。</summary>
[JsonConverter(typeof(LoadRadarNumberConverter))]
public readonly struct LoadRadarNumber
{
    private readonly decimal decimalValue;
    private readonly double doubleValue;
    private readonly bool usesDouble;

    public LoadRadarNumber(decimal value) => decimalValue = value;

    private LoadRadarNumber(double value)
    {
        if (!double.IsFinite(value)) throw new ArgumentOutOfRangeException(nameof(value), "测量数值必须有限");
        doubleValue = value;
        usesDouble = true;
    }

    public static implicit operator LoadRadarNumber(decimal value) => new(value);

    public static LoadRadarNumber FromDouble(double value)
    {
        if (!double.IsFinite(value)) throw new ArgumentOutOfRangeException(nameof(value), "测量数值必须有限");
        if (decimal.TryParse(value.ToString("R", CultureInfo.InvariantCulture), NumberStyles.Float,
            CultureInfo.InvariantCulture, out var exact) && DecimalRoundTrips(exact, value))
            return new LoadRadarNumber(exact);
        return new LoadRadarNumber(value);
    }

    internal static bool DecimalRoundTrips(decimal value, double expected) =>
        double.Parse(value.ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture) == expected;

    public double ToDouble() => usesDouble ? doubleValue :
        double.Parse(decimalValue.ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);

    // 保留 decimal 分支的精确边界；例如 0.030...001 不得先转 double 后放行。
    // double 分支只与业务使用的 0、1、0.03、0.15、1000 等明确门槛比较。
    private int CompareThreshold(decimal threshold) => usesDouble
        ? doubleValue.CompareTo((double)threshold) : decimalValue.CompareTo(threshold);

    public static bool InRange(LoadRadarNumber? value, decimal minimum, decimal? maximum = null) =>
        value is { } number && number.CompareThreshold(minimum) >= 0 &&
        (maximum is null || number.CompareThreshold(maximum.Value) <= 0);

    public decimal RoundCoalVolume()
    {
        // 仅对通过体积范围校验的数值调用；不能用 (decimal)double 的 15 位隐式舍入。
        var value = usesDouble
            ? decimal.Parse(doubleValue.ToString("R", CultureInfo.InvariantCulture), NumberStyles.Float, CultureInfo.InvariantCulture)
            : decimalValue;
        return decimal.Round(value, 1, MidpointRounding.AwayFromZero);
    }

    internal void Write(Utf8JsonWriter writer)
    {
        if (usesDouble) writer.WriteNumberValue(doubleValue);
        else writer.WriteNumberValue(decimalValue);
    }
}

public sealed class LoadRadarNumberConverter : JsonConverter<LoadRadarNumber>
{
    public override LoadRadarNumber Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options)
    {
        if (reader.TokenType != JsonTokenType.Number || !reader.TryGetDouble(out var value) || !double.IsFinite(value))
            throw new JsonException("测量数值必须为有限 JSON number");
        if (reader.TryGetDecimal(out var exact) && LoadRadarNumber.DecimalRoundTrips(exact, value))
            return new LoadRadarNumber(exact);
        return LoadRadarNumber.FromDouble(value);
    }

    public override void Write(Utf8JsonWriter writer, LoadRadarNumber value, JsonSerializerOptions options) => value.Write(writer);
}
