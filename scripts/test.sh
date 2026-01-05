#!/usr/bin/env bash
set -euo pipefail

build_dir="${1:-build}"

if [[ ! -d "$build_dir" ]]; then
    echo "Build directory not found: $build_dir" >&2
    exit 1
fi

if [[ ! -x "$build_dir/tests/test_bearing_factor" ]] || [[ ! -x "$build_dir/tests/test_pose_estimator" ]]; then
    echo "Tests are not built. Rebuild with --tests." >&2
    exit 1
fi

cd "$build_dir"
./tests/test_bearing_factor
./tests/test_pose_estimator
