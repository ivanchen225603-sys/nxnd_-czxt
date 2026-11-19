# 按各项目当前 appsettings.json 启动；不设置环境变量或覆盖业务参数。
[CmdletBinding()]
param(
    [ValidateSet('All', 'Worker', 'Analyzer', 'WebApi')][string]$Component = 'All',
    [switch]$Start
)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$components = if ($Component -eq 'All') { @('Analyzer', 'WebApi', 'Worker') } else { @($Component) }
foreach ($name in $components) {
    $folder = switch ($name) { 'Worker' { 'CarPointCloudLadarWorker' }; 'Analyzer' { 'CarPointCloud_Analyzer_fixed' }; 'WebApi' { 'CarPointCloudWebApi' } }
    $project = Join-Path $repoRoot $folder
    $configPath = Join-Path $project 'appsettings.json'
    try { $config = Get-Content -LiteralPath $configPath -Raw -Encoding UTF8 | ConvertFrom-Json }
    catch { throw "$name 配置文件缺失或格式错误（不回显内容）：$configPath" }
    if ($name -eq 'Analyzer') {
        $executable = Join-Path $project '.venv\Scripts\python.exe'
        $arguments = @('-m', 'carpointcloud_analyzer.service')
        $port = [int]$config.CARPOINT_SERVICE_PORT
        $healthPath = '/health'
    } else {
        $executable = (Get-Command dotnet -ErrorAction Stop).Source
        $dll = Join-Path $project "bin\Release\net8.0\$folder.dll"
        if (-not (Test-Path -LiteralPath $dll -PathType Leaf)) { throw "$name 尚未构建：$dll" }
        $arguments = @(('"' + $dll + '"'))
        $port = if ($name -eq 'Worker') { [int]$config.LadarCapture.HttpApiPort } else { ([uri]$config.Kestrel.Endpoints.Http.Url).Port }
        $healthPath = if ($name -eq 'Worker') { '/api/health' } else { '/healthz' }
    }
    if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) { throw "$name 运行时不存在：$executable" }
    if ($port -lt 1 -or $port -gt 65535) { throw "$name 配置文件中的端口无效。" }
    if (@(Get-NetTCPConnection -LocalPort $port -State Listen -ErrorAction SilentlyContinue).Count) {
        [pscustomobject]@{ Component=$name; Status='port_in_use_not_restarted'; Port=$port; Config=$configPath }
        continue
    }
    if (-not $Start) {
        [pscustomobject]@{ Component=$name; Status='files_found'; Port=$port; Config=$configPath; DatabaseConnectionVerified=$false }
        continue
    }
    $logDir = Join-Path $project 'logs'
    New-Item -ItemType Directory -Force -Path $logDir | Out-Null
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
    $process = Start-Process -FilePath $executable -ArgumentList $arguments -WorkingDirectory $project -WindowStyle Hidden -RedirectStandardOutput (Join-Path $logDir "$name-$stamp.out.log") -RedirectStandardError (Join-Path $logDir "$name-$stamp.err.log") -PassThru
    $available = $false
    $client = New-Object System.Net.WebClient
    $client.Proxy = $null
    try {
        for ($attempt = 0; $attempt -lt 40; $attempt++) {
            $process.Refresh()
            if ($process.HasExited) { throw "$name 启动失败，退出码 $($process.ExitCode)。请检查 $logDir 中本次日志。" }
            try { $null = $client.DownloadString("http://127.0.0.1:$port$healthPath"); $available = $true; break } catch { }
            Start-Sleep -Milliseconds 500
        }
    } finally { $client.Dispose() }
    [pscustomobject]@{ Component=$name; Status=$(if ($available) {'http_running'} else {'process_started_health_pending'}); Pid=$process.Id; Port=$port; Config=$configPath; DatabaseConnectionVerified=$false }
}
