param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [switch]$Published
)

$ErrorActionPreference = "Stop"

Set-Location -LiteralPath $ProjectRoot

if ($Published) {
    $exe = Join-Path $ProjectRoot "bin\Release\net8.0\win-x64\publish\CarPointCloudLadarWorker.exe"
    if (-not (Test-Path $exe)) {
        throw "Published worker executable not found: $exe"
    }

    & $exe
    exit $LASTEXITCODE
}

$projectFile = Join-Path $ProjectRoot "CarPointCloudLadarWorker.csproj"
if (-not (Test-Path $projectFile)) {
    throw "Worker project file not found: $projectFile"
}

# 不加载 IDE 的启动配置；业务参数由当前 appsettings.json 提供。
dotnet run --project $projectFile --configuration Release --no-launch-profile
exit $LASTEXITCODE
