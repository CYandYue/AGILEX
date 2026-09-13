#!/bin/bash
set -eo pipefail
capture_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
capture_args=("$@")
set --
source /opt/ros/noetic/setup.bash
exec /usr/bin/python3 "$capture_dir/capture.py" "${capture_args[@]}"
