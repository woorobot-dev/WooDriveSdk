# One-time setup: point git at the repo-tracked hooks in .githooks/ (git
# does not use .githooks/ automatically -- each clone must opt in).
#
# Usage: powershell -File scripts\install-hooks.ps1

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
git -C $repoRoot config core.hooksPath .githooks

Write-Host "Installed: git now runs hooks from .githooks/ (core.hooksPath set)."
Write-Host "pre-commit will block commits where arduino/ has drifted from core/."
Write-Host "(Requires Git for Windows' bash to actually execute the hook script.)"
