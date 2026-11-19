global using Xunit;
global using CarPointCloudLadarWorker.Config;
global using CarPointCloudLadarWorker.Domain;
global using Microsoft.Extensions.Logging.Abstractions;
global using Microsoft.Extensions.Options;

namespace CarPointCloudLadarWorker.Tests;

internal static class TestSupport
{
    public static LadarCaptureOptions Options(Action<LadarCaptureOptions>? configure = null)
    {
        var options = new LadarCaptureOptions
        {
            Devices =
            [
                new LadarDeviceOptions { CameraIdentifier = "CAM_004", Address = "10.0.0.4", MinValidFramePointCount = 100 },
                new LadarDeviceOptions { CameraIdentifier = "CAM_005", Address = "10.0.0.5", MinValidFramePointCount = 100 }
            ],
            DiscardCompleteCyclesAfterCommand = 1,
            MaximumFrameDeliveryDelayMilliseconds = 2000,
            ScanOnceTimeoutSeconds = 40
        };
        configure?.Invoke(options);
        return options;
    }

    public static string TempDirectory()
    {
        var path = Path.Combine(Path.GetTempPath(), "worker-tests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(path);
        return path;
    }

    /// <summary>以 B_to_A 平移 (dx,0,0) 写一份外参。</summary>
    public static string WriteExtrinsic(string directory, double dx = 1.5)
    {
        var path = Path.Combine(directory, "extrinsic.json");
        File.WriteAllText(path, $$"""
            {"frame":"B_to_A","cameraA":"CAM_004","cameraB":"CAM_005",
             "matrix":[[1,0,0,{{dx}}],[0,1,0,0],[0,0,1,0],[0,0,0,1]]}
            """);
        return path;
    }

    /// <summary>返回固定响应并记下请求的 HTTP 处理器。</summary>
    public sealed class StubHandler(Func<HttpRequestMessage, HttpResponseMessage> respond) : HttpMessageHandler
    {
        public List<(HttpRequestMessage Request, string Body)> Requests { get; } = [];

        protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
        {
            var body = request.Content is null ? string.Empty : await request.Content.ReadAsStringAsync(cancellationToken);
            lock (Requests)
                Requests.Add((request, body));
            return respond(request);
        }
    }

    public static HttpResponseMessage Json(string json, int status = 200) =>
        new((System.Net.HttpStatusCode)status) { Content = new StringContent(json, System.Text.Encoding.UTF8, "application/json") };
}
