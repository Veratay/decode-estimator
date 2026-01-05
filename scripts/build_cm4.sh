#!/usr/bin/env bash
set -euo pipefail

build_type="Release"
enable_rerun="OFF"
build_dir="build-cm4"
clean_build="OFF"
toolchain_file="cmake/toolchains/cm4.cmake"

usage() {
    cat <<'EOF'
Usage: scripts/build_cm4.sh [options]

Options:
  --debug           Debug build
  --release         Release build (default)
  --rerun           Enable Rerun visualization
  --build-dir DIR   Override build directory (default: build-cm4)
  --toolchain FILE  Override toolchain file
  --clean           Remove build directory before configuring
  -h, --help        Show this help

Environment:
  CM4_TOOLCHAIN_PREFIX  Toolchain triplet (default: aarch64-linux-gnu)
  CM4_SYSROOT           Sysroot for target
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --debug)
            build_type="Debug"
            ;;
        --release)
            build_type="Release"
            ;;
        --rerun)
            enable_rerun="ON"
            ;;
        --build-dir)
            build_dir="$2"
            shift
            ;;
        --toolchain)
            toolchain_file="$2"
            shift
            ;;
        --clean)
            clean_build="ON"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage
            exit 1
            ;;
    esac
    shift
done

if [[ "$clean_build" == "ON" ]]; then
    rm -rf "$build_dir"
fi

jobs="${JOBS:-}"
if [[ -z "$jobs" ]]; then
    if command -v nproc >/dev/null 2>&1; then
        jobs="$(nproc)"
    else
        jobs="$(getconf _NPROCESSORS_ONLN)"
    fi
fi

cmake -S . -B "$build_dir" \
    -DCMAKE_BUILD_TYPE="$build_type" \
    -DCMAKE_TOOLCHAIN_FILE="$toolchain_file" \
    -DDECODE_ENABLE_RERUN="$enable_rerun" \
    -DDECODE_BUILD_TESTS=OFF

cmake --build "$build_dir" -j"$jobs"
