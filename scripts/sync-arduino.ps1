# Copies the platform-agnostic core SDK (core/WooDriveSdk.{h,cpp}) into the
# Arduino library folder (arduino/WooDriveSdk.{h,cpp}).
#
# The Arduino Library Manager requires a self-contained library folder, so
# arduino/ cannot simply #include core/ by relative path the way windows/,
# raspberrypi/ and ros2/ do. Instead, arduino/WooDriveSdk.{h,cpp} is a
# generated copy of core/ -- always edit core/, then run this script before
# committing.
#
# Usage: powershell -File scripts\sync-arduino.ps1

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$coreDir = Join-Path $repoRoot "core"
$arduinoDir = Join-Path $repoRoot "arduino"

foreach ($name in "WooDriveSdk.h", "WooDriveSdk.cpp") {
    Copy-Item -Path (Join-Path $coreDir $name) -Destination (Join-Path $arduinoDir $name) -Force
    Write-Host "synced $name -> arduino\$name"
}

Write-Host "Done. Review 'git diff arduino/' and commit alongside your core/ changes."
