$ErrorActionPreference = "Stop"

$steamVrPath = Join-Path ${env:ProgramFiles(x86)} "Steam\steamapps\common\SteamVR"
$vrPathReg = Join-Path $steamVrPath "bin\win64\vrpathreg.exe"
$driverRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$builtDriverPath = Join-Path $driverRoot "build\output\aitrackinghmd"

if (-not (Test-Path $vrPathReg)) {
  throw "vrpathreg.exe not found. Check SteamVR installation."
}

if (-not (Test-Path $builtDriverPath)) {
  throw "Built driver folder not found: $builtDriverPath"
}

& $vrPathReg adddriver $builtDriverPath
Write-Host "Registered AI Tracking Headless HMD driver:"
Write-Host $builtDriverPath

