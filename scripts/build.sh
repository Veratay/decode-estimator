#!/usr/bin/env bash
set -euo pipefail

build_type="Release"
enable_rerun="OFF"
build_tests="OFF"
build_dir="build"
build_dir_set="OFF"
clean_build="OFF"
build_android="OFF"
android_abi="arm64-v8a"
android_api_level="21"
android_ndk=""

usage() {
    cat <<'EOF'
Usage: scripts/build.sh [options]

Options:
  --debug               Debug build
  --release             Release build (default)
  --rerun               Enable Rerun visualization
  --tests               Build unit tests (host only)
  --build-dir DIR       Override build directory (default: build, or build-android with --android)
  --clean               Remove build directory before configuring
  --android             Build for Android using NDK
  --android-abi ABI     Android ABI (default: arm64-v8a)
                        Options: armeabi-v7a, arm64-v8a, x86, x86_64
  --android-api LEVEL   Android API level (default: 21)
  --android-ndk PATH    Path to Android NDK (default: $ANDROID_NDK_HOME)
  -h, --help            Show this help
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
        --tests)
            build_tests="ON"
            ;;
        --build-dir)
            build_dir="$2"
            build_dir_set="ON"
            shift
            ;;
        --clean)
            clean_build="ON"
            ;;
        --android)
            build_android="ON"
            if [[ "$build_dir_set" == "OFF" ]]; then
                build_dir="build-android"
            fi
            ;;
        --android-abi)
            android_abi="$2"
            shift
            ;;
        --android-api)
            android_api_level="$2"
            shift
            ;;
        --android-ndk)
            android_ndk="$2"
            shift
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

# Android NDK configuration
android_cmake_args=()
if [[ "$build_android" == "ON" ]]; then
    # Determine Android NDK path
    if [[ -z "$android_ndk" ]]; then
        if [[ -n "${ANDROID_NDK_HOME:-}" ]]; then
            android_ndk="$ANDROID_NDK_HOME"
        elif [[ -n "${ANDROID_NDK:-}" ]]; then
            android_ndk="$ANDROID_NDK"
        else
            echo "Error: Android NDK not found. Please set ANDROID_NDK_HOME or use --android-ndk" >&2
            exit 1
        fi
    fi

    if [[ ! -d "$android_ndk" ]]; then
        echo "Error: Android NDK directory not found: $android_ndk" >&2
        exit 1
    fi

    # Locate the toolchain file
    toolchain_file="$android_ndk/build/cmake/android.toolchain.cmake"
    if [[ ! -f "$toolchain_file" ]]; then
        echo "Error: Android toolchain file not found: $toolchain_file" >&2
        exit 1
    fi

    echo "Building for Android:"
    echo "  NDK: $android_ndk"
    echo "  ABI: $android_abi"
    echo "  API Level: $android_api_level"
    echo "  Toolchain: $toolchain_file"
    echo ""

    android_cmake_args=(
        -DCMAKE_TOOLCHAIN_FILE="$toolchain_file"
        -DANDROID_ABI="$android_abi"
        -DANDROID_PLATFORM="android-$android_api_level"
        -DANDROID_STL=c++_shared
        -DDECODE_BUILD_FOR_ANDROID=ON
    )

    # Disable tests for Android builds
    build_tests="OFF"
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
    -DDECODE_ENABLE_RERUN="$enable_rerun" \
    -DDECODE_BUILD_TESTS="$build_tests" \
    "${android_cmake_args[@]}"

cmake --build "$build_dir" -j"$jobs"
