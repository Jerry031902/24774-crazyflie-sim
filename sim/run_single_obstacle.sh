#!/usr/bin/env bash
# Host: sudo docker exec -it cf_sim_dev bash /workspace/sim/run_single_obstacle.sh on
set -euo pipefail
safety_mode=${1:-on}
if [[ $# -gt 1 || ( "$safety_mode" != on && "$safety_mode" != off ) ]]; then
    printf 'Usage: bash /workspace/sim/run_single_obstacle.sh [on|off]\n' >&2
    exit 2
fi
cd /workspace
umask 022
mkdir -p build/single_obstacle
run_dir=$(mktemp -d "/workspace/build/single_obstacle/${safety_mode}-XXXXXX")
chmod a+rx "$run_dir"
trap 'chmod -R a+rX "$run_dir"' EXIT
exec > >(tee "$run_dir/console.log") 2>&1
printf 'Safety: %s; results: %s\n' "$safety_mode" "$run_dir"
/isaac-sim/python.sh sim/build_mission_scene.py --mission single_obstacle
printf 'Opening the flight window. Close it or press Ctrl+C to finish and save logs.\n'
/isaac-sim/python.sh sim/run_sim.py \
    --mode firmware_pid --navigator firmware --mission single_obstacle \
    --safety "$safety_mode" --ranger-noise off --seed 7 \
    --csv "$run_dir/flight.csv"
