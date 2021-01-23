#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

WS_CLEAN=0
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --clean)
            WS_CLEAN=1; shift ;;
        *)
            echo "WARNING: Unknown parameter passed: $1"; shift ;;
    esac
done

pushd "$SCRIPT_DIR" >/dev/null 2>&1
[[ WS_CLEAN -eq 1 ]] && { echo "Cleaning out build products..."; rm -rf install/ build/ log/; }
colcon build --symlink-install
popd >/dev/null 2>&1
