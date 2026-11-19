using CarPointCloudLadarWorker.Config;
using CarPointCloudLadarWorker.Hosting;
using Serilog;

// 组合根：读取唯一配置文件 appsettings.json（不按环境叠加），校验一次后注册服务并运行。
var builder = Host.CreateApplicationBuilder(args);
builder.Configuration.Sources.Clear();
builder.Configuration.AddJsonFile(
    Path.Combine(builder.Environment.ContentRootPath, "appsettings.json"), optional: false, reloadOnChange: false);

try
{
    var options = builder.Configuration.GetSection("LadarCapture").Get<LadarCaptureOptions>() ?? new LadarCaptureOptions();
    options.Validate();

    builder.Services.AddWindowsService(o => o.ServiceName = "CarPointCloudLadarWorker");
    builder.Services.AddSerilog((sp, lc) => lc
        .ReadFrom.Configuration(builder.Configuration)
        .ReadFrom.Services(sp)
        .Enrich.FromLogContext());
    builder.Services.AddWorker(options);

    builder.Build().Run();
}
catch (Exception ex)
{
    // 以 Windows 服务或 systemd 运行时 stderr 无人接收，启动失败必须写进日志文件才能排查。
    using var logger = new LoggerConfiguration().ReadFrom.Configuration(builder.Configuration).CreateLogger();
    logger.Fatal(ex, "Worker 进程因未处理异常终止。若为配置校验失败，请按异常内容修正 LadarCapture 配置。");
    throw;
}
