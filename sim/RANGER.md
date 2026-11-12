# Multi-ranger simulation

The Multi-ranger model provides front/back/left/right/up distances from live
PhysX colliders, following the real deck and its stock firmware driver. It is separate from the stock EKF and PID. It does not
implement obstacle avoidance, emergency braking, or a NIS gate. The only
consumer is `run_sim.py --mission`, whose WedgeBug navigator reads the held
front/left/back/right values (see the README's WedgeBug section).

## 1. Sources and model boundaries

Verified by repository and installed Isaac Sim 5.1 source inspection:

| Behavior | Source / implementation |
|---|---|
| Five direction names, mm log units | `src/modules/src/range.c:38-43,66-86` |
| 100 ms nominal task interval (10 Hz) | `src/deck/drivers/src/multiranger.c:134-143`; hardware also waits for data-ready sequentially |
| Invalid reading = 32767 mm | `multiranger.c:99-105`; default accepts only RANGE_VALID status |
| Nominal range up to 4 m, accuracy "within a few millimeters" | [Bitcraze Multi-ranger specification](https://www.bitcraze.io/products/multi-ranger-deck/); real range depends on surface and light |
| 27° full field of view | ST VL53L1X datasheet (DS12385), default 16x16 SPAD ROI; `src/drivers/src/vl53l1x.c` never sets an ROI |
| Delivery one period after measurement start | `multiranger.c:84-112,136-143` reads each result and restarts the sensor every 100 ms; ST defaults left by `vl53l1xInit` are long mode and a 41 ms timing budget (`src/lib/vl53l1/core/src/vl53l1_api.c`, `VL53L1_StaticInit`/`VL53L1_SetPresetMode`) |
| Sensor positions | [Multi-ranger datasheet](https://www.bitcraze.io/documentation/hardware/multi_ranger_deck/multi_ranger_deck-datasheet.pdf) drawing: side sensor faces 32.5 mm apart front-back, 32.6 mm left-right; up sensor at the deck centre |
| Body frame | X forward, Y left, Z up, same as the existing sim |
| Scene queries | `omni.physx.get_physx_scene_query_interface().raycast_all`, installed PhysX 107.3.26 in Isaac Sim 5.1; [107.3 Python API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/extensions/runtime/source/omni.physx/docs/api/python.html) |
| Headless query enable | `SimulationContext.get_physics_context().set_enable_scene_query_support(True)`, installed `isaacsim.core.api/physics_context/physics_context.py:695-709` |

`omni.physx` is the PhysX extension, not a deprecated `omni.isaac.*` API.
The backend traverses all hits, excludes the drone root and descendants, then
chooses the nearest remaining collider. It does not assume hit ordering or
stop after a self hit. World-aligned bounding boxes are printed by the smoke test.

The following are explicit approximations, not hardware-calibrated claims:

- Each sensor casts a grid of rays clipped to its 27° cone (49 rays with the
  default `RANGER_CONE_RAYS = 9`) and reports the nearest hit. The real sensor
  reports one distance from a signal histogram over its whole field of view;
  nearest-hit is a conservative approximation. Because the cone is 3-D, a tilted
  drone can see the floor within 4 m (below about 0.93 m altitude even when level).
  The cone's horizontal row is exactly the 2-D fan in `read_rangers.m`.
  `fov_rad=0` gives the previous single centre ray.
- No reflectivity, transparent materials, ambient-light sensitivity, multipath,
  sensor crosstalk, or random invalid-status model. Rays intersect collision
  geometry, not render-only meshes.
- Default origins are the datasheet sensor faces, `RANGER_ORIGINS_BODY_M`. Their
  heights above the body origin are unmeasured and left at 0. Both origin and
  direction rotate with the full body quaternion, including tilt.
- Zero-mean Gaussian noise std 0.002 m, the upper end of the team's static
  bench test (1.4-2.0 mm, front/back/left/right at 0.2 and 0.5 m). The up
  sensor and longer ranges were not measured. Bias defaults to zero.
  Integer-mm quantization is modeled.
- All five axes are measured simultaneously at 0, 0.1, 0.2 s, ... and delivered
  one period later (`RANGER_DELAY_MS = 100`), so the first values arrive at 0.1 s.
  The geometry is taken at the start of each 41 ms measurement. No I2C/RTOS
  jitter, sequential sensor reads, or hardware startup is emulated.
- No visual deck mesh or extra mass is added; `MASS_KG` remains the total vehicle
  mass and should be measured with all installed decks and battery.

This model reads scene geometry. The existing downward ToF in `sensors.py` still
uses its flat-floor analytical model and remains the only range EKF input.

## 2. Run and configure

Run from the **host** after starting `cf_sim_dev`:

```bash
docker exec -it cf_sim_dev /isaac-sim/python.sh /workspace/sim/run_sim.py \
  --mode ekf_closed_loop --demo --noise off --hover-seconds 5 \
  --ranger on --ranger-noise off \
  --csv /workspace/build/demo_live/ranger_demo.csv
```

Takeoff/settle occupies 0-10 s, hover 10-15 s, then the position target changes
to `(0.5,0,1)` m at 15 s. The GUI keeps holding the final target. Add
`--headless` for automatic completion at 25 s.

- `--ranger on` is the `run_sim.py` default in all modes, including altitude PID.
- `--ranger off` restores the previous sensing configuration.
- `--noise` affects IMU/Flow/ToF only. Multi-ranger noise (bench-measured, 2 mm)
  is on by default with `--ranger on`; `--ranger-noise off` gives ideal ranging.
  Both use `--seed`, but independent RNG streams.
- `--ranger-csv PATH` selects the standalone range log, even without `--demo`.
- Edit the top-level constants in `ranger.py` for noise std, bias, extra delay,
  nominal maximum range, and optical origins. Restart the script after editing.
- Delays postpone delivery of the original sampled ray, without recomputing it
  using the later pose. Pending measurements at shutdown are not flushed early.

`setup_simulation()` defaults Ranger off for existing programmatic runners.
The historical EKF comparison and video scripts are unchanged. New callers use
`setup_simulation(..., ranger=True)` and call `sample_ranger(truth)` once per
1 ms step before logging. `reset_runtime()` also resets Ranger timing and RNG.

## 3. Outputs and validity

For the command above:

| Artifact | Content |
|---|---|
| `ranger_demo.csv` | Existing flight state/control log plus held range values and validity/sample-time fields |
| `ranger_demo.measurements.csv` | Existing Flow/ToF innovations, unchanged |
| `ranger_demo.ranger.csv` | One row per delivered sensor, five rows per 100 ms frame |
| `ranger_demo.ranger.json` | Model configuration, generated/delivered frame counts |

The range CSV contains sample/delivery time, direction, integer `range_mm`,
`valid`, measured `distance_m`, noiseless `ideal_distance_m`, `hit_prim`, and
world ray origin/direction. No-return and out-of-range readings have
`range_mm=32767`, `valid=0`, `distance_m=NaN`. Never interpret the sentinel as an
obstacle 32.767 m away. With no ceiling, the arena's upward ray normally has no return.

Before the first measurement, held flight-CSV registers are zero, validity is
false, and sample time is NaN. At other times, repeated held values do not imply
new samples; inspect the timestamp. These ranges are telemetry, not EKF inputs.

Plot using the container's Python:

```bash
docker exec cf_sim_dev /isaac-sim/python.sh /workspace/sim/plot_ranger.py \
  /workspace/build/demo_live/ranger_demo.ranger.csv
```

The sibling `.png` shows ideal ray distance and measurement; red crosses mark
invalid returns at a fixed visual marker height, not a physical distance.

## 4. Validation

```bash
docker exec cf_sim_dev /isaac-sim/python.sh -m pytest \
  /workspace/sim/tests/test_ranger.py -q
docker exec cf_sim_dev /isaac-sim/python.sh /workspace/sim/tests/smoke_ranger.py \
  --output-dir /workspace/build/ranger_validation
```

Offline tests cover scheduling, delayed historical samples, reset, frames and
offsets, invalid/maximum range, unsorted/self hits, noise reproducibility and logs.
The live smoke runner checks actual walls, nearest pillar, yaw/pitch transforms,
offsets, open ceiling, maximum range and self exclusion. It then runs the 25 s
EKF demo with noise off and on, plus a noisy-EKF reference with Ranger disabled.
It checks identical true states/motor commands for the noisy reference, and
saves CSVs, range/flight plots, timing statistics, and `summary.json`.

The geometry-only option is `--geometry-only`. Existing scene USDs and firmware
sources are not rewritten by these tests. Use a new output directory to keep old results.

Verified on 2026-11-01 in `cf_sim_dev` (Isaac Sim 5.1), with the previous
single-ray, zero-delay, body-origin model. The hardware cone/delay/position model
has passed the offline tests; rerun the smoke runner in the container to
re-verify it live:

- 23 sim unit tests passed, including 7 Ranger tests and the existing sensor/PID tests.
- 23 offline real-logger tests passed, including optional Ranger block/rate/invalid handling.
- All live geometric assertions passed: center-to-wall 2.95 m, test-to-pillar
  0.35 m, translation, yaw/pitch, optical offset, open ceiling, maximum range,
  and continuing past the live drone's own collider.
- Each 25 s flight delivered 250 frames / 1250 direction readings. Noise off/on
  passed the existing bounded-flight checks. Noise-on still has the baseline's
  position error; this does not claim precise real flight or obstacle avoidance.
- With EKF noise on, Ranger enabled/disabled produced bit-identical logged true
  position, quaternion, linear velocity, and four motor commands.
- User-facing `run_sim.py` CLI, independent Ranger-noise flag, standalone Ranger
  CSV, and held flight-CSV timestamps were checked in bounded headless runs.

Artifacts: `build/ranger_validation_20261101/` (ignored by Git), including
`summary.json`, `noise_off.ranger.png`, `noise_on.ranger.png`, full flight/range
CSVs and runner logs. The noisy Ranger measurement residual std was about 3.01 mm
over 1000 valid readings; this checks the configured simulation noise, not hardware accuracy.

## 5. Hardware comparison

`real/log_flight.py --ranger` optionally adds a 10-byte, 100 ms log block for
`range.front/back/left/right/up`, and checks `deck.bcMultiranger` is present.
The original logging and flight behavior is unchanged when the flag is omitted.
See [real/README.md](../real/README.md) for a motors-off wall-distance collection.
Real log timestamps are packet times, not acquisition timestamps; keep invalid
32767 readings and report their fraction separately. Noise and optical offsets
remain uncalibrated until measured on the team's hardware.
