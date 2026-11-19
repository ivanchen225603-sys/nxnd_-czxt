namespace CarPointCloudWebApi.Api;

/// <summary>
/// 统一接口响应包装对象。
/// </summary>
/// <typeparam name="T">接口业务数据类型；失败响应时通常为空。</typeparam>
/// <remarks>
/// 业务关键点：所有对外接口均使用 code/message/data 三段式返回，便于乙方系统统一解析。
/// </remarks>
public class ApiResponse<T>
{
    /// <summary>业务状态码；成功固定为 200，异常时与 HTTP 状态码保持一致。</summary>
    public int Code { get; set; }

    /// <summary>业务提示信息；成功固定为 success，失败返回可读错误原因。</summary>
    public string Message { get; set; } = "success";

    /// <summary>
    /// 业务数据载荷；失败响应为默认值。
    /// </summary>
    public T? Data { get; set; }

    /// <summary>
    /// 是否成功（Code == 200）。
    /// </summary>
    /// <remarks>
    /// 兼容前端查看器（logs.html/login.html）中 result.success 的取值习惯；
    /// 序列化后输出为小写 success 字段，与 code/message/data 并存，不影响既有乙方接口解析。
    /// </remarks>
    [System.Text.Json.Serialization.JsonPropertyName("success")]
    public bool IsSuccess => Code == 200;

    /// <summary>
    /// 创建成功响应。
    /// </summary>
    /// <param name="data">需要返回给调用方的业务数据。</param>
    /// <returns>code=200 的标准响应对象。</returns>
    public static ApiResponse<T> Success(T data)
    {
        return new ApiResponse<T>
        {
            Code = 200,
            Message = "success",
            Data = data
        };
    }

    /// <summary>
    /// 创建失败响应。
    /// </summary>
    /// <param name="code">业务错误码，建议与 HTTP 状态码一致。</param>
    /// <param name="message">错误说明，需能直接定位调用方参数或系统问题。</param>
    /// <returns>标准失败响应对象。</returns>
    public static ApiResponse<T> Fail(int code, string message)
    {
        return new ApiResponse<T>
        {
            Code = code,
            Message = message,
            Data = default
        };
    }
}
