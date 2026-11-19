using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Text.Json;

namespace CarPointCloudWebApi.Tests;

public class MockPartnerServer : IDisposable
{
    private readonly HttpListener _listener;
    private readonly Thread _listenerThread;
    private bool _running = true;
    private string _mode = "NonEmptyArray";
    private int _requestCount;
    private CapturedPartnerRequest? _lastRequest;

    public string Mode
    {
        get => Volatile.Read(ref _mode);
        set => Volatile.Write(ref _mode, value);
    }

    public int RequestCount => Volatile.Read(ref _requestCount);

    public CapturedPartnerRequest? LastRequest => Volatile.Read(ref _lastRequest);

    public string BaseUrl { get; }

    public MockPartnerServer()
    {
        var port = GetFreePort();
        BaseUrl = $"http://localhost:{port}/";
        _listener = new HttpListener();
        _listener.Prefixes.Add(BaseUrl);
        _listener.Start();

        _listenerThread = new Thread(HandleRequests)
        {
            IsBackground = true
        };
        _listenerThread.Start();
    }

    private static int GetFreePort()
    {
        using var l = new TcpListener(IPAddress.Loopback, 0);
        l.Start();
        var port = ((IPEndPoint)l.LocalEndpoint).Port;
        l.Stop();
        return port;
    }

    private void HandleRequests()
    {
        while (_running)
        {
            try
            {
                var context = _listener.GetContext();
                var request = context.Request;
                var response = context.Response;
                Interlocked.Increment(ref _requestCount);
                Volatile.Write(ref _lastRequest, new CapturedPartnerRequest(
                    request.HttpMethod,
                    request.Url?.AbsolutePath ?? string.Empty,
                    request.Url?.Query ?? string.Empty,
                    request.Headers["Timestamp"],
                    request.Headers["Signature"]));

                var cameraId = ParseCameraIdentifier(request.Url);

                var now = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
                object responseData = Mode switch
                {
                    "EmptyArray" => new
                    {
                        code = 200,
                        message = "success",
                        data = Array.Empty<object>()
                    },
                    "BusinessError" => new
                    {
                        code = 500,
                        message = "partner-internal-detail",
                        data = Array.Empty<object>()
                    },
                    _ => new
                    {
                        code = 200,
                        message = "success",
                        data = new[]
                        {
                            new { cameraId, licensePlate = "京A07550", captureTime = now, enterID = "business-1" },
                            new { cameraId, licensePlate = "京B08811", captureTime = now, enterID = "business-2" }
                        }
                    }
                };

                var responseText = Mode == "InvalidJson"
                    ? "{not-json"
                    : JsonSerializer.Serialize(responseData);
                if (Mode == "Http500")
                {
                    response.StatusCode = (int)HttpStatusCode.InternalServerError;
                    responseText = "{\"detail\":\"partner-sensitive-debug-data\"}";
                }

                response.ContentType = Mode == "NonJson"
                    ? "text/plain; charset=utf-8"
                    : "application/json; charset=utf-8";
                if (Mode == "NonJson")
                {
                    responseText = "partner-sensitive-plain-text";
                }

                var buffer = Encoding.UTF8.GetBytes(responseText);
                response.ContentLength64 = buffer.Length;
                response.OutputStream.Write(buffer, 0, buffer.Length);
                response.OutputStream.Close();
            }
            catch
            {
                if (!_running) break;
            }
        }
    }

    public void ResetCapture()
    {
        Interlocked.Exchange(ref _requestCount, 0);
        Volatile.Write(ref _lastRequest, null);
    }

    private static string ParseCameraIdentifier(Uri? url)
    {
        var cameraId = "CAM001";
        if (url == null || string.IsNullOrEmpty(url.Query))
        {
            return cameraId;
        }

        var query = url.Query.TrimStart('?');
        foreach (var pair in query.Split('&', StringSplitOptions.RemoveEmptyEntries))
        {
            var kv = pair.Split('=', 2);
            if (kv.Length == 2 && kv[0] == "cameraIdentifier")
            {
                cameraId = Uri.UnescapeDataString(kv[1]);
                break;
            }
        }

        return cameraId;
    }

    public void Dispose()
    {
        _running = false;
        try { _listener.Stop(); } catch { }
        try { _listener.Close(); } catch { }
    }
}

public sealed record CapturedPartnerRequest(
    string Method,
    string Path,
    string Query,
    string? Timestamp,
    string? Signature);
