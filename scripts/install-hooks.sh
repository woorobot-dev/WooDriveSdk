#!/usr/bin/env bash
# One-time setup: point git at the repo-tracked hooks in .githooks/ (git
# does not use .githooks/ automatically -- each clone must opt in).
#
# Usage: scripts/install-hooks.sh

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
git -C "$repo_root" config core.hooksPath .githooks
chmod +x "$repo_root"/.githooks/* 2>/dev/null || true

echo "Installed: git now runs hooks from .githooks/ (core.hooksPath set)."
echo "pre-commit will block commits where arduino/ has drifted from core/."
