#!/bin/bash
set -eo pipefail
capture_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
capture_args=("$@")
set --
source "$capture_dir/../lio_ws/devel/setup.bash"
exec roslaunch "$capture_dir/launch/sensors.launch" "${capture_args[@]}"
