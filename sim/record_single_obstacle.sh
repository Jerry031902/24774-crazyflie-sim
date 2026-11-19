#!/usr/bin/env bash
# Host: sudo docker exec -it cf_sim_dev bash /workspace/sim/record_single_obstacle.sh on
set -euo pipefail
safety_mode=${1:-on}
if [[ $# -gt 1 || ( "$safety_mode" != on && "$safety_mode" != off ) ]]; then
    printf 'Usage: bash /workspace/sim/record_single_obstacle.sh [on|off]\n' >&2
    exit 2
fi
cd /workspace
umask 022
mkdir -p build/single_obstacle_video
run_dir=$(mktemp -d "/workspace/build/single_obstacle_video/${safety_mode}-XXXXXX")
chmod a+rx "$run_dir"
trap 'chmod -R a+rX "$run_dir"' EXIT
# The recorder requires its output directory to be empty. Keep console outside it.
printf 'Recording output: %s\n' "$run_dir"
/isaac-sim/python.sh sim/record_mission.py --mission single_obstacle \
    --safety "$safety_mode" --output "$run_dir" 2>&1 | tee "${run_dir}.console.log"
