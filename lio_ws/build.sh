#!/bin/bash
set -eo pipefail
lio_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$lio_dir/../devel/setup.bash"
cd "$lio_dir"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release "$@"
