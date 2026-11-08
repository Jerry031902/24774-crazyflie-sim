# Horizontal Safety supervisor for Isaac Sim

Python/Isaac integration of the MATLAB Primary/Backup, command arbiter and
BRAKE/HOLD/RECOVER architecture. Enable it with
`run_sim.py --mission matlab_main --safety on`. Default is **off**, retaining
the existing navigation/flight-controller path.

**Validation status:** CPU tests and the native C navigation harness run without
Isaac. Live PID/PhysX validation has **not** been run on this Windows development
machine (Isaac Sim/cffirmware are not installed). Braking bounds are assumptions
until measured in the team's Isaac environment. This is a rule-based supervisor,
not a CBF-QP or certified collision-avoidance guarantee.

## Integrated behavior

```text
Multi-ranger + estimated state -> WedgeBug candidate -> Primary --+
RAW packet + health clocks -> independent Backup ----------------+-> arbiter
                                                                 -> body vx/vy, yaw rate
                                                                 -> existing PID -> motors -> PhysX
```

- Primary checks current/requested horizontal closing motion in the sampled
  3-D cones, uncertainty, observed travel, and a configurable stopping envelope.
- Backup bypasses the Primary median filter. It checks raw validity, sequence/
  timestamp monotonicity, age, completion heartbeats, hard distance, overspeed
  and tilt. It uses its own braking parameters.
- BRAKE sends zero horizontal target velocity and zero yaw rate. Actual velocity
  remains governed by the original PID and PhysX.
- HOLD adds bounded XY position feedback: gain 1/s, maximum 0.1 m/s. A correction
  is suppressed during faults or if raw clearance/cone coverage cannot support
  it; `safety_hold_limited` records that suppression.
- Three distinct healthy packets release HOLD; RECOVER ramps speed from
  0.04 m/s to cruise over 2 s.
- The altitude target stays at `--target-z`; altitude/attitude PID remains active.
  This does not cut motor thrust, implement vertical obstacle avoidance, or
  supervise takeoff/landing. Safety starts after settled takeoff and continues
  through navigation and goal hold; `safety_active=0` marks takeoff rows.
- C and Python navigators support checkpoint/rollback: rejected candidates do
  not advance waypoints, scan buffers or events. Recovery after interrupted
  translation preserves target/episode history and clears the stale partial
  plan. A stationary scan pause is preserved.
- Scans count each raw frame once, use sample orientation/time, and await
  fresh endpoint measurements, and the unchecked final diagonal slide is
  disabled, whenever the navigator is given timestamped frames. Missions and
  the 2-D/MATLAB simulators now always pass them, with Safety on or off.

Primary/Backup run in the same Python process. Heartbeat checks detect missing
routine completions while the loop runs; they cannot actuate a brake if the
whole process/physics loop hangs.

## Run on the team's Isaac machine

Use the repository's existing Ubuntu/NVIDIA/Docker/Isaac 5.1 setup from
[README](README.md#setup-one-time). Start the existing container and rebuild:

```bash
docker exec -it cf_sim_dev bash
cd /workspace
make bindings_python
```

Rebuilding is required for `wedgebugCopyState`, `wedgebugResume` and the extended
timestamped-range input. Stale bindings give an explicit error for Safety.
Original PID/EKF gains and `cf2_defconfig` are not changed by this port.

Start with truth feedback, then test estimated-state feedback separately:

```bash
/isaac-sim/python.sh sim/run_sim.py \
  --mode firmware_pid --mission matlab_main --safety on

/isaac-sim/python.sh sim/run_sim.py \
  --mode ekf_closed_loop --mission matlab_main --safety on --noise off
```

`--navigator python` selects the reference navigator; both backends retain
the firmware PID. Use `--safety off` as baseline. Safety requires `--mission`
and `--ranger on`; the existing `--demo` hover/move path stays separate.

## Braking assumptions and thresholds

MATLAB's discrete first-order stopping formula is not used for PID/PhysX.
Primary assumes the parameterized distance envelope:

```text
D(V) = V * brake_response + V^2 / (2 * brake_accel)
```

`V` conservatively includes actual and requested horizontal speed. The required
directional clearance adds worst cone closure, sample age, observed
sample-to-now translation, one navigation tick, and the reserve below. Backup
uses a separate raw-data envelope and independent response/deceleration values.

| Setting | Default |
|---|---:|
| `--safety-brake-accel` | 0.5 m/s^2 |
| `--safety-brake-response` | 0.20 s |
| `--safety-backup-brake-accel` | 0.5 m/s^2 |
| `--safety-backup-brake-response` | 0.25 s |
| Primary reserve | 0.04 m + 2 sigma |
| Backup reserve | 0.015 m + 1 sigma |
| Hard distance | 0.01 m body gap (about 0.0642 m raw ranger reading) |
| Maximum data age / completion timeout | 0.35 / 0.08 s |
| Horizontal speed / tilt limit | 1 m/s / 20 deg |

These parameters do not assert that the PID produces constant deceleration.
Identify an upper envelope across speeds, headings, attitudes, heights,
saturation and noise; update both monitors. Passing one live smoke run does
not establish a global bound. Reports record `braking_bounds_calibrated: false`.

## Sensing and state source

`MultiRanger.snapshot()` adds a frame sequence and per-direction status:
0 unavailable, 1 hit, 2 valid no-return, 3 invalid. A 32767-mm sentinel becomes
free space only for an explicitly completed no-return observation. Corrupt or
out-of-range packets fail closed. CSV metadata is additive; timing is unchanged.

PhysX truth still generates sensor readings. Safety receives no hit position,
mesh, or true sample pose. It associates each packet with the historical
estimate available at sample time. In `ekf_closed_loop` this is the same
estimated-state source used by navigation/PID; truth is separately used for
metrics. Sensor cones rotate with the full sampled quaternion, not yaw alone.
Monitor timestamps are absolute simulation time; navigator scans/events use
navigation-relative time.

`--noise` affects IMU/Flow/ToF. Independent `--ranger-noise on` uses the existing
Isaac std **0.003 m**, not MATLAB main's **0.1 m**. Safety uses the enabled ranger
std. These are simulation assumptions, not measured hardware accuracy.

## Inspect results

Default Safety outputs have a separate suffix, for example:

```text
build/missions/matlab_main_firmware_pid_safety.csv
build/missions/matlab_main_firmware_pid_safety.safety.json
build/missions/matlab_main_firmware_pid_safety.ranger.csv
```

CSV adds state/reason/source, accepted flag, monitor triggers/margins, age,
sequence, nominal/candidate/selected speeds and caught errors. JSON stores
config, events and summary. Stop/Play resets Safety/filter/pose history.

```bash
/isaac-sim/python.sh sim/plot_safety.py \
  build/missions/matlab_main_firmware_pid_safety.csv
```

Plotting also works on a laptop with normal Python + NumPy + Matplotlib.

## Verify

CPU-only tests:

```bash
python -m unittest discover -s sim/tests -p test_safety.py -v
python -m unittest discover -s sim/tests -v
```

The native C harness compiles with host GCC when available. Firmware PID/EKF/
SWIG-specific tests skip if `cffirmware` is missing; rerun in the container
after rebuilding bindings.

LIVE checks on the Isaac machine:

```bash
/isaac-sim/python.sh sim/tests/smoke_safety.py --mode firmware_pid
/isaac-sim/python.sh sim/tests/smoke_safety.py --mode ekf_closed_loop
```

The runner writes a separate empty-room USD under `build/isaac_safety_live`,
then tests nominal flight and temporary ranger/Primary/navigation dropouts.
It records CSVs/events/braking travel and checks completion, backup intervention,
stop/resume, preserved altitude targets and the observed stopping envelope.
`--case primary_timeout` selects one case. Original USD scenes are not saved.

## Pending live work

- Run the full rebuilt C bindings/PID/EKF/PhysX checks on the team's GPU machine.
- Identify braking distance including drift after the stopped-speed threshold;
  test realistic estimator/sensor uncertainty.
- Port MATLAB's scheduled popup scene/benchmarks to live USD colliders. This
  change ports Safety, not the MATLAB 24 cm collision boundary into 3-D.
- LQR, CBF-QP, full firmware watchdog/arming and hardware flight remain separate.

Sharing this port requires both `sim/` and `apps/wedgebug/src/` in the commit;
adding only `matlab-trajectory-sim/` omits the Isaac integration.
