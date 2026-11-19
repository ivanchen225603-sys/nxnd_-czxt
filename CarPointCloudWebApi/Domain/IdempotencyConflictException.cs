namespace CarPointCloudWebApi.Domain;

/// <summary>
/// 表示同一个幂等键被用于不同的业务载荷。
/// </summary>
public sealed class IdempotencyConflictException : Exception
{
    /// <summary>
    /// 使用指定业务冲突说明创建幂等键冲突异常。
    /// </summary>
    /// <param name="message">同一幂等键对应不同载荷时返回给上层的冲突原因。</param>
    public IdempotencyConflictException(string message)
        : base(message)
    {
    }
}
