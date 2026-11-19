#!/usr/bin/env bash
# Host: sudo docker exec -it cf_sim_dev bash /workspace/sim/run_validation_suite.sh
set -euo pipefail
cd /workspace
umask 022
mkdir -p build/validation_suite
suite_dir=$(mktemp -d /workspace/build/validation_suite/run-XXXXXX)
chmod a+rx "$suite_dir"
trap 'chmod -R a+rX "$suite_dir"' EXIT
exec > >(tee "$suite_dir/console.log") 2>&1
printf 'SUITE_RESULTS %s\n' "$suite_dir"
make bindings_python PYTHON=/isaac-sim/python.sh
failed=0
for experiment in multi_on multi_off popup_on popup_off; do
    case "$experiment" in
        multi_*) mission=matlab_main; limit=320; extra=() ;;
        popup_*) mission=single_obstacle; limit=120; extra=(--popup-at 3) ;;
    esac
    safety_mode=${experiment##*_}
    printf '\nSTART_EXPERIMENT %s\n' "$experiment"
    if ! /isaac-sim/python.sh sim/record_mission.py \
        --mission "$mission" --safety "$safety_mode" \
        --output "$suite_dir/$experiment" --max-seconds "$limit" "${extra[@]}"; then
        printf 'RECORDING_FAILED %s\n' "$experiment"
        failed=1
        continue
    fi
    if ! /isaac-sim/python.sh sim/check_experiment.py "$suite_dir/$experiment"; then
        printf 'VALIDATION_FAILED %s\n' "$experiment"
        failed=1
    fi
done
if [[ "$failed" == 0 ]]; then
    printf '\nSUITE_PASSED %s\n' "$suite_dir"
else
    printf '\nSUITE_NEEDS_REVIEW %s\n' "$suite_dir"
fi
exit "$failed"
