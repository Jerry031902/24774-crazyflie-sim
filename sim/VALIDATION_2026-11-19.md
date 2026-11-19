# Safety recovery validation — 2026-11-19

The stopped-heading recovery fix completed both the multi-obstacle mission and
the scheduled popup experiment. All four release checks passed. This is a
deterministic simulator baseline for subsequent CBF work, not a hardware safety
claim or evidence that Safety OFF necessarily collides.

## Change under test

A nearly stationary vehicle can request translation outside the delayed Ranger
frame's observed cones just after turning. The Primary still rejects that
command. Recovery now preserves the rolled-back navigator state for this
specific stopped `unobserved_direction` condition instead of launching another
avoidance scan. Moving rejections still request replanning. Primary/Backup
thresholds, PID gains, cruise speed and recovery ramp remain unchanged.

## Setup and results

- Isaac Sim 5.1, firmware C WedgeBug navigator and stock firmware PID.
- Ground-truth feedback; target altitude 1 m; Ranger noise off; seed 7.
- 1 ms physics tick, 50 Hz navigation, nominal 10 Hz Ranger / 100 ms delay.
- Real viewport recordings: 1280 x 720, 20 fps, 1x simulation time.
- Local evidence: `build/validation_suite/run-b7xmWg/` (ignored by Git).

| Experiment | Targets reached | Navigation time | Min. body clearance | Collision | Safety brakes |
|---|---:|---:|---:|---|---:|
| Multi-obstacle, ON | 4/4 | 95.58 s | 21.21 cm | No | 9 |
| Multi-obstacle, OFF | 4/4 | 85.60 s | 13.33 cm | No | — |
| Popup, ON | 1/1 | 32.04 s | 34.47 cm | No | 3 |
| Popup, OFF | 1/1 | 30.26 s | 33.13 cm | No | — |

Both ON runs had zero Backup brakes and no recorded Safety exception. The
multi-obstacle ON run completed seven scans, versus six with Safety OFF.
Recordings include takeoff and three seconds of goal hold, so their durations
(105.80, 95.80, 42.25 and 40.45 seconds respectively) exceed navigation times.
All four MP4s were fully decoded successfully; PNG, frame CSV and capture
metadata counts matched (2116, 1916, 845 and 809 frames).

## Popup verification

The box starts invisible with collision disabled, then both are enabled at
navigation time 3.000 s (simulation time 10.137 s). It remains stationary.
Ranger readings before activation were checked for unintended box hits.

| Measurement at appearance | Safety ON | Safety OFF |
|---|---:|---:|
| Horizontal speed | 0.505 m/s | 0.503 m/s |
| Body clearance | 1.588 m | 1.032 m |
| First front Ranger sample | 10.200 s | 10.200 s |
| First delivery | 10.300 s | 10.300 s |

The first detected collider was `/World/Obstacles/Box` in both runs. Insertion
did not overlap the drone. The timing/world schedule was identical, but the
closed-loop vehicle positions differed at appearance; these runs do not isolate
reaction performance at identical initial clearance. Both OFF and ON avoided
the box, so this case does not demonstrate an OFF collision prevented by ON.

## Reproduction

Follow `sim/README.md` for the Ubuntu/NVIDIA/container setup and build bindings.
Install FFmpeg with libx264 in the container, or provide a compatible executable
at `/workspace/build/video_tools/ffmpeg` (not committed). On the host:

```bash
xhost '+si:localuser:#1234'
sudo docker exec -it cf_sim_dev bash /workspace/sim/run_validation_suite.sh
```

The suite prints `SUITE_PASSED` only if all runs pass their recorded checks.
For each run, inspect `capture.json`, `validation.json`, CSVs and `flight.mp4`.
Noise-free CPU regression checks on the development host: 89 tests discovered,
73 passed, 16 skipped because host Python lacks Isaac/cffirmware. The native C
navigation harness was included. Run the skipped bindings tests in the team's
Isaac environment for coverage beyond these flight experiments.

## Remaining work

- Repeated seeds, Ranger/estimator noise, estimated-state feedback and timing faults.
- Measured/calibrated PID/PhysX braking envelopes; current bounds remain assumptions.
- An explicitly labelled fault-injection case to evaluate independent Backup
  intervention and, if physically feasible, contrast OFF collision with ON avoidance.
- Agility tuning and Isaac-compatible CBF design, separately from this recovery fix.
