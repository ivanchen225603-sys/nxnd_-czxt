namespace CarPointCloudWebApi.Tests;

/// <summary>
/// 可手动推进的确定性时间源。
/// </summary>
/// <remarks>
/// 账号锁定按 15 分钟计、令牌回查缓存按 30 秒计，用真实时钟验证意味着一条用例真等十几分钟。
/// 这里自己实现而不引入测试用 NuGet 包，是为了不给现场部署的项目增加额外依赖。
/// </remarks>
internal sealed class TestTimeProvider : TimeProvider
{
    private DateTimeOffset _utcNow;

    /// <summary>创建从指定时刻起步的虚拟时钟。</summary>
    /// <param name="startUtc">初始 UTC 时刻。</param>
    public TestTimeProvider(DateTimeOffset? startUtc = null)
    {
        _utcNow = startUtc ?? new DateTimeOffset(2026, 9, 6, 0, 0, 0, TimeSpan.Zero);
    }

    /// <inheritdoc />
    public override DateTimeOffset GetUtcNow() => _utcNow;

    /// <inheritdoc />
    public override TimeZoneInfo LocalTimeZone => TimeZoneInfo.Utc;

    /// <summary>向前推进虚拟时钟。</summary>
    /// <param name="delta">推进量，必须为非负。</param>
    public void Advance(TimeSpan delta)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(delta, TimeSpan.Zero);
        _utcNow = _utcNow.Add(delta);
    }
}
