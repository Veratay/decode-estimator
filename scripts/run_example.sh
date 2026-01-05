#!/usr/bin/env bash
set -euo pipefail

build_dir="build"

usage() {
    cat <<'EOF'
Usage: scripts/run_example.sh [options]

Options:
  --build-dir DIR   Override build directory (default: build)
  -h, --help        Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build-dir)
            build_dir="$2"
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

scripts/build.sh --rerun --build-dir "$build_dir"
"${build_dir}/examples/simple_localization"
