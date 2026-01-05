#!/usr/bin/env bash
set -euo pipefail

build_dir="${BUILD_DIR:-build}"
build_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir)
      build_dir="$2"
      build_args+=("$1" "$2")
      shift 2
      ;;
    --debug|--release|--rerun|--no-rerun)
      build_args+=("$1")
      shift
      ;;
    --)
      shift
      break
      ;;
    *)
      break
      ;;
  esac
done

example_path="$build_dir/examples/simple_localization"

if [[ ! -x "$example_path" ]]; then
  "$(dirname "$0")/build.sh" "${build_args[@]}"
fi

exec "$example_path" "$@"
