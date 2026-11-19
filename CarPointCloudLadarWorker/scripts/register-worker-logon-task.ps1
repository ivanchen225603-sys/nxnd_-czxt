param(
    [string]$TaskName = "CarPointCloudLadarWorker",
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [switch]$Published,
    [ValidateRange(0, 999)]
    [int]$RestartCount = 10,
    [ValidateRange(1, 1440)]
    [int]$RestartIntervalMinutes = 1
)

$ErrorActionPreference = "Stop"

$scriptPath = Join-Path $PSScriptRoot "start-worker.ps1"
if (-not (Test-Path $scriptPath)) {
    throw "Worker start script not found: $scriptPath"
}

$arguments = @(
    "-NoProfile",
    "-ExecutionPolicy", "Bypass",
    "-File", "`"$scriptPath`"",
    "-ProjectRoot", "`"$ProjectRoot`"",
    "-Environment", "Production"
)

if ($Published) {
    $arguments += "-Published"
}

$action = New-ScheduledTaskAction -Execute "powershell.exe" -Argument ($arguments -join " ")
$trigger = New-ScheduledTaskTrigger -AtLogOn
$principal = New-ScheduledTaskPrincipal -UserId $env:USERNAME -LogonType Interactive -RunLevel Highest
$settings = New-ScheduledTaskSettingsSet `
    -RestartCount $RestartCount `
    -RestartInterval (New-TimeSpan -Minutes $RestartIntervalMinutes) `
    -ExecutionTimeLimit ([TimeSpan]::Zero) `
    -MultipleInstances IgnoreNew

Register-ScheduledTask -TaskName $TaskName -Action $action -Trigger $trigger -Principal $principal -Settings $settings -Force | Out-Null
Write-Host "Registered scheduled task: $TaskName (restartCount=$RestartCount, restartIntervalMinutes=$RestartIntervalMinutes)"
