param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$PackageVersion = "1.2.1",
    [string]$SdkRoot = "",
    [string]$OutputRoot = ""
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($SdkRoot)) {
    $sdkParent = Split-Path -Parent $ProjectRoot
    $sdkContainer = Get-ChildItem -LiteralPath $sdkParent -Directory |
        Where-Object { $_.Name -like "*SDK*" -and $_.Name -like "*1.1.4*" } |
        Select-Object -First 1
    if ($null -eq $sdkContainer) {
        throw "Cannot locate the Linux SDK. Pass -SdkRoot explicitly."
    }
    $SdkRoot = Join-Path $sdkContainer.FullName "galaxy_ladar_sdk_V1.1.4"
}
if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    $OutputRoot = Join-Path $ProjectRoot "artifacts\linux-x64-package"
}

$ProjectRoot = (Resolve-Path $ProjectRoot).Path
$SdkRoot = (Resolve-Path $SdkRoot).Path
$OutputRoot = [System.IO.Path]::GetFullPath($OutputRoot)
$StageRoot = Join-Path $OutputRoot "CarPointCloudLadarWorker-linux-x64-$PackageVersion"
$AppOutput = Join-Path $StageRoot "app"
$NativeOutput = Join-Path $StageRoot "native"

if (-not (Test-Path (Join-Path $SdkRoot "linux\libladarsdk_gd.so"))) {
    throw "Linux SDK library not found: $SdkRoot\linux\libladarsdk_gd.so"
}
if (-not (Test-Path (Join-Path $SdkRoot "include\ladar_api.h"))) {
    throw "Linux SDK header not found: $SdkRoot\include\ladar_api.h"
}

if (Test-Path $OutputRoot) {
    $resolvedOutput = [System.IO.Path]::GetFullPath((Resolve-Path $OutputRoot).Path)
    $resolvedProject = [System.IO.Path]::GetFullPath($ProjectRoot)
    if (-not $resolvedOutput.StartsWith($resolvedProject + [System.IO.Path]::DirectorySeparatorChar, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove output outside the project artifacts directory: $resolvedOutput"
    }
    Remove-Item -LiteralPath $OutputRoot -Recurse -Force
}

New-Item -ItemType Directory -Path $AppOutput, $NativeOutput, (Join-Path $NativeOutput "vendor") -Force | Out-Null

Write-Host "Publishing Linux x64 managed application..."
dotnet publish (Join-Path $ProjectRoot "CarPointCloudLadarWorker.csproj") `
    -c Release `
    -r linux-x64 `
    --self-contained true `
    -p:PublishSingleFile=false `
    -p:PublishTrimmed=false `
    -o $AppOutput
if ($LASTEXITCODE -ne 0) {
    throw "dotnet publish failed with exit code $LASTEXITCODE"
}

Copy-Item (Join-Path $ProjectRoot "packaging\linux-x64\appsettings.template.json") (Join-Path $AppOutput "appsettings.json") -Force
Copy-Item (Join-Path $ProjectRoot "packaging\linux-x64\appsettings.Synthetic.json") (Join-Path $AppOutput "appsettings.Synthetic.json") -Force
Copy-Item (Join-Path $SdkRoot "linux\libladarsdk_gd.so") (Join-Path $NativeOutput "libladarsdk_gd.so") -Force
Copy-Item (Join-Path $SdkRoot "include\ladar_api.h") (Join-Path $NativeOutput "vendor\ladar_api.h") -Force
Copy-Item (Join-Path $ProjectRoot "native\linux\ladar_bridge.cpp") $NativeOutput -Force
Copy-Item (Join-Path $ProjectRoot "native\linux\ladar_bridge.h") $NativeOutput -Force
Copy-Item (Join-Path $ProjectRoot "packaging\linux-x64\install.sh") $StageRoot -Force
Copy-Item (Join-Path $ProjectRoot "packaging\linux-x64\uninstall.sh") $StageRoot -Force
Copy-Item (Join-Path $ProjectRoot "packaging\linux-x64\self-test.sh") $StageRoot -Force
Copy-Item (Join-Path $ProjectRoot "packaging\linux-x64\check-analyzer-integration.sh") $StageRoot -Force
Copy-Item (Join-Path $ProjectRoot "packaging\linux-x64\README.md") $StageRoot -Force
Copy-Item (Join-Path $ProjectRoot "packaging\linux-x64\dual-radar-extrinsic.example.json") $StageRoot -Force
Copy-Item (Join-Path $ProjectRoot "packaging\linux-x64\car-pointcloud-ladar-worker.service") $StageRoot -Force
Copy-Item (Join-Path $ProjectRoot "scripts\workflow-v6-server-test.py") $StageRoot -Force

$manifest = @{
    package = "CarPointCloudLadarWorker"
    version = $PackageVersion
    target = "Kylin V10 SP1 desktop / linux-x64"
    serverOs = "Kylin V10 SP1"
    architecture = "x86_64"
    cpu = "Zhaoxin KaiXian KX-U6780A, 8 cores"
    serverAddress = "plant=203.0.113.19; radar=198.51.100.101/24"
    sdk = "Galaxy LiDAR SDK V1.1.4 Linux"
    buildUtc = [DateTime]::UtcNow.ToString("o")
    install = "sudo bash ./install.sh --app-secret APP_SECRET"
    analyzerInstall = "sudo bash ./install.sh --app-secret APP_SECRET --enable-analyzer --extrinsic /data/carpointcloud/extrinsics/dual-radar-extrinsic.json"
    syntheticInstall = "sudo bash ./install.sh --synthetic --allow-system-disk"
    selfTest = "bash ./self-test.sh"
    analyzerIntegrationCheck = "sudo bash ./check-analyzer-integration.sh"
    workflowIntegrationTest = "sudo env LD_LIBRARY_PATH=/dm8/dmdbms/bin:/usr/lib64:/usr/lib /opt/carpointcloud-analyzer/venv/bin/python ./workflow-v6-server-test.py"
}
$manifest | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $StageRoot "package-manifest.json") -Encoding UTF8

Write-Host "Creating tar.gz package..."
$archive = Join-Path $OutputRoot "CarPointCloudLadarWorker-linux-x64-$PackageVersion.tar.gz"
tar -czf $archive -C $OutputRoot (Split-Path $StageRoot -Leaf)
if ($LASTEXITCODE -ne 0) {
    throw "tar failed with exit code $LASTEXITCODE"
}

Write-Host "Package created: $archive"
