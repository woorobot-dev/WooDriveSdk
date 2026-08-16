# Fails if arduino/WooDriveSdk.{h,cpp} has drifted from core/WooDriveSdk.{h,cpp}.
# Run this in CI / pre-commit to catch a forgotten scripts\sync-arduino.ps1.
#
# Usage: powershell -File scripts\check-arduino-sync.ps1

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$coreDir = Join-Path $repoRoot "core"
$arduinoDir = Join-Path $repoRoot "arduino"

$ok = $true
foreach ($name in "WooDriveSdk.h", "WooDriveSdk.cpp") {
    $corePath = Join-Path $coreDir $name
    $arduinoPath = Join-Path $arduinoDir $name
    $diff = Compare-Object -ReferenceObject (Get-Content $corePath) -DifferenceObject (Get-Content $arduinoPath)
    if ($diff) {
        Write-Host "OUT OF SYNC: arduino\$name differs from core\$name"
        Write-Host "  Run scripts\sync-arduino.ps1 and commit the result."
        $ok = $false
    }
}

if ($ok) {
    Write-Host "arduino/ is in sync with core/."
    exit 0
} else {
    exit 1
}
