using Xunit;

namespace CarPointCloudWebApi.Tests;

/// <summary>
/// 需要真实 DM8 的集成用例。未配置连接串时在发现阶段就标记为 skip。
/// </summary>
public sealed class DatabaseFactAttribute : FactAttribute
{
    public DatabaseFactAttribute()
    {
        if (!TestConfiguration.HasDatabaseConnectionString)
        {
            Skip = TestConfiguration.DatabaseSkipReason;
        }
    }
}

/// <summary>
/// <see cref="DatabaseFactAttribute"/> 的数据驱动版本。
/// </summary>
public sealed class DatabaseTheoryAttribute : TheoryAttribute
{
    public DatabaseTheoryAttribute()
    {
        if (!TestConfiguration.HasDatabaseConnectionString)
        {
            Skip = TestConfiguration.DatabaseSkipReason;
        }
    }
}
