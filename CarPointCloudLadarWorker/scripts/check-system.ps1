param(
    [string]$AnalyzerHealthUrl = "http://127.0.0.1:8010/health",
    [string]$WorkerProcessName = "CarPointCloudLadarWorker",
    [string]$CameraIdentifier = "CAM_001",
    [string]$CaptureDirectory = "D:\data\captures",
    [string]$HealthSnapshotDirectory = "D:\data\worker-health",
    [string]$AnalysisQueueDirectory = "D:\data\analysis-queue",
    [string]$UploadQueueDirectory = "D:\data\upload-queue",
    [string]$DataDrive = "D:"
)

$ErrorActionPreference = "Stop"

function Count-JsonFiles([string]$Path) {
    if (-not (Test-Path $Path)) {
        return 0
    }

    return @(Get-ChildItem -Path $Path -Filter "*.json" -File -ErrorAction SilentlyContinue).Count
}

function Get-QueueStats([string]$Root) {
    [ordered]@{
        pending = Count-JsonFiles (Join-Path $Root "pending")
        sent = Count-JsonFiles (Join-Path $Root "sent")
        dead = Count-JsonFiles (Join-Path $Root "dead")
    }
}

function Get-LatestPcd([string]$Path) {
    if (-not (Test-Path $Path)) {
        return $null
    }

    $file = Get-ChildItem -Path $Path -Filter "*.pcd" -File -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTimeUtc -Descending |
        Select-Object -First 1

    if ($null -eq $file) {
        return $null
    }

    [ordered]@{
        path = $file.FullName
        lastWriteUtc = $file.LastWriteTimeUtc.ToString("o")
        ageSeconds = [math]::Round(((Get-Date).ToUniversalTime() - $file.LastWriteTimeUtc).TotalSeconds, 1)
        sizeBytes = $file.Length
    }
}

$analyzer = [ordered]@{
    ok = $false
    url = $AnalyzerHealthUrl
    body = $null
    error = $null
}

try {
    $analyzer.body = Invoke-RestMethod -Uri $AnalyzerHealthUrl -TimeoutSec 3
    $analyzer.ok = $true
} catch {
    $analyzer.error = $_.Exception.Message
}

$workerProcesses = @(Get-CimInstance Win32_Process |
    Where-Object {
        ($_.Name -eq "$WorkerProcessName.exe") -or
        ($_.Name -eq "dotnet.exe" -and $_.CommandLine -match "\brun\b" -and $_.CommandLine -like "*$WorkerProcessName.csproj*") -or
        ($_.Name -eq "dotnet.exe" -and $_.CommandLine -like "*$WorkerProcessName.dll*")
    } |
    Select-Object ProcessId, Name, CommandLine)

$healthSnapshotPath = Join-Path $HealthSnapshotDirectory "$CameraIdentifier.json"
$healthSnapshot = $null
if (Test-Path $healthSnapshotPath) {
    try {
        $healthSnapshot = Get-Content -Raw $healthSnapshotPath | ConvertFrom-Json
    } catch {
        $healthSnapshot = [ordered]@{ error = $_.Exception.Message; path = $healthSnapshotPath }
    }
}

$driveName = $DataDrive.TrimEnd(":")
$drive = Get-PSDrive -Name $driveName -ErrorAction SilentlyContinue
$disk = if ($null -eq $drive) {
    [ordered]@{ drive = $DataDrive; error = "drive not found" }
} else {
    [ordered]@{
        drive = $DataDrive
        freeGb = [math]::Round($drive.Free / 1GB, 2)
        usedGb = [math]::Round($drive.Used / 1GB, 2)
    }
}

[ordered]@{
    checkedAtUtc = (Get-Date).ToUniversalTime().ToString("o")
    analyzer = $analyzer
    worker = [ordered]@{
        ok = $workerProcesses.Count -gt 0
        processes = $workerProcesses
        healthSnapshotPath = $healthSnapshotPath
        healthSnapshot = $healthSnapshot
    }
    latestPcd = Get-LatestPcd $CaptureDirectory
    analysisQueue = Get-QueueStats $AnalysisQueueDirectory
    uploadQueue = Get-QueueStats $UploadQueueDirectory
    disk = $disk
} | ConvertTo-Json -Depth 10
