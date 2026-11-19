# 当前 appsettings.json 决定监听地址、路径、数据库和算法参数。
param([string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot))
$ErrorActionPreference = 'Stop'
$python = Join-Path $ProjectRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python -PathType Leaf)) { throw "Python runtime not found: $python" }
Push-Location -LiteralPath $ProjectRoot
try { & $python -m carpointcloud_analyzer.service; exit $LASTEXITCODE }
finally { Pop-Location }
