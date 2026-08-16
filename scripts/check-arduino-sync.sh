#!/usr/bin/env bash
# Fails if arduino/WooDriveSdk.{h,cpp} has drifted from core/WooDriveSdk.{h,cpp}.
# Run this in CI / pre-commit to catch a forgotten `scripts/sync-arduino.sh`.
#
# Usage: scripts/check-arduino-sync.sh

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
core_dir="$repo_root/core"
arduino_dir="$repo_root/arduino"

status=0
for name in WooDriveSdk.h WooDriveSdk.cpp; do
  if ! diff -q "$core_dir/$name" "$arduino_dir/$name" > /dev/null 2>&1; then
    echo "OUT OF SYNC: arduino/$name differs from core/$name"
    echo "  Run scripts/sync-arduino.sh and commit the result."
    status=1
  fi
done

if [ "$status" -eq 0 ]; then
  echo "arduino/ is in sync with core/."
fi
exit "$status"
