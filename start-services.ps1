# 一键启动当前文件配置的三个服务。
& (Join-Path $PSScriptRoot 'scripts\windows-internal-chain.ps1') -Component All -Start
