#!/usr/bin/env bash
# Copies the platform-agnostic core SDK (core/WooDriveSdk.{h,cpp}) into the
# Arduino library folder (arduino/WooDriveSdk.{h,cpp}).
#
# The Arduino Library Manager requires a self-contained library folder, so
# arduino/ cannot simply #include core/ by relative path the way windows/,
# raspberrypi/ and ros2/ do. Instead, arduino/WooDriveSdk.{h,cpp} is a
# generated copy of core/ -- always edit core/, then run this script before
# committing.
#
# Usage: scripts/sync-arduino.sh

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
core_dir="$repo_root/core"
arduino_dir="$repo_root/arduino"

for name in WooDriveSdk.h WooDriveSdk.cpp; do
  cp "$core_dir/$name" "$arduino_dir/$name"
  echo "synced $name -> arduino/$name"
done

echo "Done. Review 'git diff arduino/' and commit alongside your core/ changes."
