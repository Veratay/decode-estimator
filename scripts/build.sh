#!/usr/bin/env bash
set -euo pipefail

build_dir="${BUILD_DIR:-build}"
build_type="${BUILD_TYPE:-Release}"
enable_rerun="${DECODE_ENABLE_RERUN:-OFF}"
use_local_deps="${DECODE_USE_LOCAL_DEPS:-OFF}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir)
      build_dir="$2"
      shift 2
      ;;
    --debug)
      build_type="Debug"
      shift
      ;;
    --release)
      build_type="Release"
      shift
      ;;
    --rerun)
      enable_rerun="ON"
      shift
      ;;
    --no-rerun)
      enable_rerun="OFF"
      shift
      ;;
    --system-deps)
      use_local_deps="OFF"
      shift
      ;;
    --local-deps)
      use_local_deps="ON"
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if command -v nproc >/dev/null 2>&1; then
  jobs="$(nproc)"
else
  jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu)"
fi

cmake -S . -B "$build_dir" \
  -DCMAKE_BUILD_TYPE="$build_type" \
  -DDECODE_ENABLE_RERUN="$enable_rerun" \
  -DDECODE_USE_LOCAL_DEPS="$use_local_deps"

cmake --build "$build_dir" -j "$jobs"

# Disable TBB malloc proxy to prevent memory corruption
export LD_PRELOAD=""
export TBBMALLOC_PROXY_DISABLE=1
