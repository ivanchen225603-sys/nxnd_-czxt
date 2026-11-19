using System.Security.Cryptography;
using System.Text.RegularExpressions;
using Microsoft.Extensions.Options;

namespace CarPointCloudWebApi.Infrastructure;

/// <summary>CH4 空车基线 NPZ 文件库：{StorageRoot}/baselines/{sha256}.npz，文件名即内容摘要。</summary>
public sealed class LoadRadarAssetStore(IOptions<LoadRadarOptions> options)
{
    /// <summary>上传接口的请求体上限。</summary>
    public const long MaxBytes = 32 * 1024 * 1024;

    private string Root => Path.GetFullPath(Path.Combine(options.Value.StorageRoot, "baselines"));

    public string PathFor(string sha256) => Regex.IsMatch(sha256, "\\A[a-f0-9]{64}\\z")
        ? Path.Combine(Root, sha256 + ".npz")
        : throw new LoadRadarException(400, "assetSha256 必须为 64 位小写 SHA-256");

    /// <summary>保存上传的基线；内容摘要必须与文件名一致。</summary>
    public async Task Upload(string sha256, Stream body, CancellationToken cancellationToken)
    {
        var finalPath = PathFor(sha256);
        Directory.CreateDirectory(Root);
        var temporary = Path.Combine(Root, Guid.NewGuid().ToString("N") + ".upload");
        await using (var output = File.Create(temporary))
            await body.CopyToAsync(output, cancellationToken);
        if (Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(temporary))).ToLowerInvariant() != sha256)
        {
            File.Delete(temporary);
            throw new LoadRadarException(400, "文件摘要不一致，未接收基线");
        }
        File.Move(temporary, finalPath, overwrite: true);
    }

    /// <summary>基线文件由采集端在测量登记后单独上传；未到达时相关操作稍后重试。</summary>
    public void Verify(string sha256)
    {
        if (!File.Exists(PathFor(sha256)))
            throw new LoadRadarException(409, "基线文件尚未完整上传", retryable: true);
    }

    public FileStream Open(string sha256) => File.OpenRead(PathFor(sha256));
}
