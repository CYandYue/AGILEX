#!/bin/bash
set -eo pipefail
lio_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$lio_dir/devel/setup.bash" ]]; then
  echo "Build first: $lio_dir/build.sh" >&2
  exit 1
fi
source "$lio_dir/devel/setup.bash"
exec roslaunch agilex_lio mapping.launch "$@"
