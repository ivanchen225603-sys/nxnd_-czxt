namespace CarPointCloudWebApi.Tests;

internal static class TestConfiguration
{
    internal const string AppSecret = "integration-test-secret-not-for-production";

    internal const string DatabaseEnvironmentVariable = "CARPOINTCLOUD_TEST_DB_CONNECTION_STRING";

    internal const string DatabaseSkipReason =
        "未设置 " + DatabaseEnvironmentVariable + "：跳过需要真实 DM8 的集成测试。";

    /// <summary>
    /// 返回 DM8 集成测试连接串；未配置时返回 null 而不是抛异常。
    /// 没有数据库的机器（开发笔记本、CI 构建节点）应当把这些用例报成 skip 而非 fail，
    /// 否则真实回归失败会淹没在几十条环境噪音里。
    /// </summary>
    internal static string? GetDatabaseConnectionString()
    {
        var connectionString = Environment.GetEnvironmentVariable(DatabaseEnvironmentVariable);
        return string.IsNullOrWhiteSpace(connectionString) ? null : connectionString;
    }

    internal static bool HasDatabaseConnectionString => GetDatabaseConnectionString() is not null;
}
