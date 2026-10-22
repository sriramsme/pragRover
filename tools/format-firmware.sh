#!/usr/bin/env bash
# Format only our application/test code, never the SDK or managed components.
set -euo pipefail

rover_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
shopt -s nullglob
sources=(
    "$rover_root"/firmware/motion_controller/core/*.[ch]
    "$rover_root"/firmware/motion_controller/main/*.[ch]
    "$rover_root"/firmware/motion_controller/native/*.c
    "$rover_root"/firmware/camera_node/main/*.c
)

case "${1:-}" in
    --check) exec clang-format --dry-run --Werror "${sources[@]}" ;;
    '') exec clang-format -i "${sources[@]}" ;;
    *) printf 'Usage: tools/format-firmware.sh [--check]\n' >&2; exit 2 ;;
esac
