namespace CarPointCloudLadarWorker.Tests;

/// <summary>
/// 可手动推进的确定性时间源。
/// </summary>
/// <remarks>
/// 周期估计、完整场最小间隔和会话超时全部使用单调时间，而这些判据的量级是"秒"到
/// "几十秒"（7 秒周期下一次会话最坏要跨两个周期）。若用真实时钟验证，
/// 一条用例就要真等十几秒，整套时序测试会慢到没人愿意跑。
/// 这里自己实现而不引入测试用 NuGet 包，是为了不给现场部署的项目增加额外依赖。
/// </remarks>
internal sealed class TestTimeProvider : TimeProvider
{
    /// <summary>使用纳秒刻度，保证毫秒级推进不产生取整误差。</summary>
    private const long Frequency = 1_000_000_000;

    private long _timestamp;
    private DateTimeOffset _utcNow;

    /// <summary>创建从指定挂钟时刻起步的虚拟时钟。</summary>
    /// <param name="startUtc">虚拟时钟的初始 UTC 时刻。</param>
    public TestTimeProvider(DateTimeOffset? startUtc = null)
    {
        _utcNow = startUtc ?? new DateTimeOffset(2026, 9, 5, 0, 0, 0, TimeSpan.Zero);
    }

    /// <inheritdoc />
    public override long TimestampFrequency => Frequency;

    /// <inheritdoc />
    public override long GetTimestamp() => Interlocked.Read(ref _timestamp);

    /// <inheritdoc />
    public override DateTimeOffset GetUtcNow() => _utcNow;

    /// <summary>只调整业务挂钟，模拟 NTP；不改变单调计时。</summary>
    public void ShiftWallClock(TimeSpan delta) => _utcNow = _utcNow.Add(delta);

    /// <inheritdoc />
    public override TimeZoneInfo LocalTimeZone => TimeZoneInfo.Utc;

    /// <summary>同步推进单调时间戳与挂钟时间。</summary>
    /// <param name="delta">推进量，必须为非负。</param>
    public void Advance(TimeSpan delta)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(delta, TimeSpan.Zero);
        Interlocked.Add(ref _timestamp, (long)(delta.TotalSeconds * Frequency));
        _utcNow = _utcNow.Add(delta);
    }

    /// <summary>推进指定毫秒数。</summary>
    /// <param name="milliseconds">推进的毫秒数。</param>
    public void AdvanceMilliseconds(double milliseconds) =>
        Advance(TimeSpan.FromMilliseconds(milliseconds));

    /// <summary>推进指定秒数。</summary>
    /// <param name="seconds">推进的秒数。</param>
    public void AdvanceSeconds(double seconds) =>
        Advance(TimeSpan.FromSeconds(seconds));
}
