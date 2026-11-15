# Stock EKF baseline

Verified against firmware HEAD `774e72256714d26f8383455ab9f3afe293b37af7` and the
existing Python 3.11 `cffirmware` build. Firmware source and `cf2_defconfig` are
unchanged. Hardware selection: Crazyflie 2.1 board, 47-17 propellers, Flow deck v2
(VL53L1X), confirmed by the owner. No custom NIS gate is present in this baseline.

## 1. Stock order and rates

Verified by source inspection. Rates below distinguish nominal scheduling from
actual sensor arrival times on hardware.

| Operation | Stock behavior | Source |
|---|---|---|
| IMU acquisition | 1000 Hz; gyro in body deg/s, acceleration in body Gs | `src/hal/src/sensors_bmi088_bmp3xx.c:61,325-346`; `src/modules/interface/stabilizer_types.h:165-174` |
| IMU filtering | Gyro 80 Hz, accelerometer 30 Hz, 2-pole LPF at 1000 Hz | `sensors_bmi088_bmp3xx.c:140-141,546-547`; `src/utils/src/filter.c:74-125` |
| Stabilizer / estimator wakeup | Stabilizer runs on 1 kHz IMU data-ready; copies the latest completed estimator output, then wakes the worker | `src/modules/src/stabilizer.c:317-329`; `src/modules/src/estimator/estimator_kalman.c:289-299` |
| Worker scheduling | Lower priority than stabilizer: 2 versus 5. Binary semaphore may coalesce wakeups under load | `src/config/config.h:65,93`; `estimator_kalman.c:200-202,225-226` |
| Prediction | Nominal 100 Hz, `nowMs >= nextPredictionMs`, then `nextPredictionMs = nowMs + 10` | `estimator_kalman.c:119-120,220,244-249` |
| IMU subsampling | Finalize accumulated **previous** samples before predict; arithmetic mean, multiply acceleration by 9.81 and gyro by pi/180 | `estimator_kalman.c:245-248,350-356,382-383`; `src/modules/src/axis3fSubSampler.c:29-53` |
| Process noise | Every worker iteration, after prediction and before queued measurements, even on non-prediction iterations | `estimator_kalman.c:258-261`; `src/modules/src/kalman_core/kalman_core.c:642-648` |
| Flow / ToF updates | Drain measurement queue in FIFO order. No universal Flow-before-ToF ordering. Flow uses `gyroLatest` in deg/s; gyro/acc queue entries accumulate subsamples and update latest values | `estimator_kalman.c:301-366` |
| Finalize | Called after draining queue on every worker iteration. Performs work only if `isUpdated` | `estimator_kalman.c:263-266`; `kalman_core.c:651-657` |
| Bounds check | Position +/-100 m, body velocity +/-10 m/s; request reinitialization at the next iteration | `estimator_kalman.c:228-231,268-269`; `src/modules/src/kalman_supervisor.c:34-56` |
| Externalize | Every iteration, after finalize/bounds check. World position/velocity, quaternion wxyz, legacy Euler pitch negated, acceleration from latest IMU in Gs | `estimator_kalman.c:277-283`; `kalman_core.c:774-818` |

Thus one nominal worker tick is:

```text
pending reset
→ if predict due: finalize IMU subsamples → kalmanCorePredict
→ kalmanCoreAddProcessNoise
→ drain FIFO: IMU accumulate/latest, Flow update, ToF update, as received
→ kalmanCoreFinalize
→ stock bounds check (schedule reset if necessary)
→ kalmanCoreExternalizeState
```

`quadIsFlying` normally comes from `supervisorIsFlying()` (`estimator_kalman.c:233-237`).
The stock supervisor samples motor-over-idle activity at 25 Hz and uses 2000 ms
low-thrust hysteresis (`supervisor.c:56,276-299,588-610`,
`stabilizer_types.h:375`). This simulation assumes an already calibrated, armed,
healthy vehicle and mirrors that flight-condition path using ideal motor activity.
Full arming, watchdog, and emergency state machines are outside this estimator
baseline. The estimator bounds/reset behavior is retained.

Unlike the old `bindings/util/estimator_kalman_emulator.py`, the new wrapper uses
the source's `>=` deadline, starts its first prediction at the initial timestamp
(dt=0), schedules from `now`, retains `accLatest` for externalization, handles
Flow/ToF, and does not assume `quadIsFlying=True` from startup.

## 2. Deck scheduling and stock measurement checks

Flow v2 initializes `bcZRanger2` (`src/deck/drivers/src/flowdeck_v1v2.c:210-217`).

| Sensor | Nominal interval | Additional stock behavior |
|---|---|---|
| Flow | `vTaskDelay(10)` with 1 ms RTOS ticks, about 100 Hz | SPI reading/task execution adds time. `flow.dt` uses actual microsecond timestamps, not a hard-coded 0.01 (`flowdeck_v1v2.c:88-95,121-125`; `src/config/FreeRTOSConfig.h:77-78`) |
| ToF v2 | 25 ms timing budget and task period, nominal 40 Hz | Driver waits for data-ready and restarts measurement; I2C/task timing can make actual arrivals slower (`zranger2.c:69-81,124-143`) |

Flow applies the mounting axis swap/sign convention before constructing
`flowMeasurement_t`: dpixelx=-raw deltaY, dpixely=-raw deltaX. It rejects counts
with absolute value >=100, uses fixed stdDev 2.0 by default, and enqueues only a
valid motion/status flag 0xB0. Adaptive stdDev and optional LP/MA smoothing are
off by default (`flowdeck_v1v2.c:44-48,73-77,92-125,150-158`). The synthetic model
generates counts already in that firmware frame; it must not swap/sign them twice.

ToF rejects ranges >=5000 mm, converts distance to meters, and sets its stock
distance-dependent stdDev:

```text
stdDev = 0.0025 * (1 + exp(k * (distance - 2.5)))
k = log(0.2 / 0.0025) / (4.0 - 2.5)
```

Source: `src/deck/drivers/src/zranger2.c:49-55,102-103,137-143`.
`mm_tof.c:35-59` additionally skips an update unless R[2][2] > 0.1. These stock
checks remain. No residual-based rejection or new NIS logic was added.

## 3. Measurement models and simulation boundaries

`sensors.py` uses PhysX truth only on the measurement-generation side:

- IMU specific force is R-transpose * (world acceleration - gravity), divided by
  9.81 to get Gs; gyro is R-transpose * world angular velocity, converted to deg/s.
  Acceleration uses successive PhysX velocities. Its first sample assumes zero
  inertial acceleration. C firmware LPFs are reused; their first values are primed
  to represent an already booted/calibrated IMU. Sensor mounting is body-aligned
  at the body reference point; bias calibration and IMU lever-arm dynamics are omitted.
- Flow uses `mm_flow.c:31,47-97`: height floor 0.1 m, Npix=35, thetapix=0.71674,
  rotation terms, camera lever arm, R[2][2], and the 0.1 measurement scaling. CF2
  camera offset defaults to zero (`platform_defaults.h:239-247`). The truth
  generator uses true angular rate; the EKF uses latest filtered measured gyro,
  exactly as the stock update does.
- ToF uses the stock flat-floor cone model (`mm_tof.c:35-40`):
  height / cos(max(acos(R[2][2])-7.5 degrees, 0)). It does not raycast obstacles.
  The demo route has a clear flat floor. Horizontal Ranger is not an EKF input.

This is a continuous-valued measurement-interface model. ADC/pixel quantization,
PMW image formation, surface-dependent quality/status, bias/random walk, and
dropouts are not simulated. The floor is assumed textured and valid PMW status is
assumed 0xB0. Existing numeric range/attitude checks are still applied. The C EKF
receives no pose, position or velocity ground-truth measurements.

All noise/delay settings are constants at the top of `sensors.py`. Noise-on
draws independent Gaussian samples. IMU and ToF values come from the team's
static bench tests (motors off): the filtered std was gyro 0.114 / 0.169 / 0.126
deg/s and accelerometer 0.00133 / 0.00101 / 0.00138 G (x / y / z), and ToF
1.65-2.06 mm at 0.5-0.7 m (the sim uses 2.06 mm). IMU noise is added before
its software LPFs, so it is injected at the bench std divided by each filter's
white-noise gain (0.416 at 80 Hz, 0.258 at 30 Hz): 0.274 / 0.406 / 0.303 deg/s
and 0.0052 / 0.0039 / 0.0054 G. Flow keeps an assumed 2.0 firmware counts per
axis until it is measured on a consistent floor and lighting. All additional delays default to zero; their integer
millisecond queue implementation is tested separately. Noise-off leaves firmware
filters, process noise, and stock measurement stdDev untouched.

The simulator schedules ideal nominal sensor intervals and a 1 kHz worker with no
RTOS/SPI/I2C jitter or missed wakeups. At coincident delivery times the deterministic
enqueue order is gyro, acc, Flow, ToF. This is a documented simulation ordering,
not a claim that hardware always processes the deck packets in that order.
Packets retain sample times and Flow sample intervals when delayed. The stock
core updates at arrival time; no extra delay compensation or history rewind is added.

`kalmanCoreDefaultParams()` uses the firmware's shared parameter defaults
(`kalman_core_params_defaults.h:63-98`). Initial XYZ and yaw remain zero; initial
covariances, process noise, and default attitude reversion 0.001 remain unchanged.
The Flow v2 descriptor requests Kalman but not attitude-reversion disable
(`flowdeck_v1v2.c:242-253`). The initial true Z is 0.1 m, so startup estimation
error is deliberately visible until ToF arrives. No truth initialization hides it.

## 4. Modes and matched demo

- `ekf_shadow`: existing noise-free ground-truth C PID controls the vehicle.
  Synthetic measurements drive a parallel EKF. Sensor noise cannot affect this
  controller, allowing estimation errors to be isolated.
- `ekf_closed_loop`: C PID's `state_t` comes directly from the previous completed
  EKF iteration; `sensorData_t` comes from the simulated IMU. No additional gyro
  filtering, Euler conversion, or ground-truth velocity differentiation occurs
  inside this controller path.

The worker runs after the controller in each 1 ms step, reflecting the stock
copy-before-wakeup and task priorities. Both current worker estimates and the
older estimates used by the controller are logged.

The four-case comparison runner uses the same clock-based targets in both modes: takeoff/settle to (0,0,1) during
t=0..10 s, hover during t=10..20 s, then move/settle at (0.5,0,1) until t=30 s.
The GUI keeps holding afterward. This avoids using true position to decide when
closed-loop waypoints switch, and avoids noisy estimates continuously resetting
a settling timer. `run_sim.py --demo` now defaults to a shorter 5 s hover; add
`--hover-seconds 10` to reproduce this comparison schedule. The comparison runner
and recorder keep their original 10 s hover default.

## 5. Running and evaluating

Host commands, using the existing development container:

```bash
docker exec -it cf_sim_dev /isaac-sim/python.sh /workspace/sim/run_sim.py --mode ekf_shadow --demo --noise off --hover-seconds 10
docker exec -it cf_sim_dev /isaac-sim/python.sh /workspace/sim/run_sim.py --mode ekf_closed_loop --demo --noise on --hover-seconds 10
docker exec cf_sim_dev /isaac-sim/python.sh -m unittest discover -s /workspace/sim/tests -v
docker exec cf_sim_dev /isaac-sim/python.sh /workspace/sim/tests/smoke_firmware_ekf.py
```

The last command runs all four combinations sequentially on the same arena,
resetting physics, EKF, PID, filters and RNG each time. Options `--mode`, `--noise`,
`--seed` and `--output-dir` select subsets or separate experiments.

Artifacts are ignored by Git under `build/ekf_baseline/`. Each run has a 1 kHz
state/motor CSV, an event-rate `.measurements.csv`, a plot, and a JSON report.
Flow innovations are in measurement-space pixels (interface counts * 0.1); ToF
innovations are in meters. The C measurement files keep prediction log variables static, so diagnostics
reconstruct the same model from the pre-update C state in Python. They may differ
by floating-point roundoff and never influence measurement acceptance.

Reports distinguish estimation error (estimate minus truth) from tracking error
(truth minus target). Max Z estimation error includes startup, with an additional
metric after 1 s. Hover XY estimation drift is the change in estimate-minus-truth
error between the first/last samples marked `hover` (t=10..20 s in these original runs); true hover drift and maximum XY
error are also recorded. Attitude error is quaternion angular distance, with both
max and RMS, avoiding Euler wrapping artifacts.

The predeclared smoke-test stability criteria are: complete the demo (30 s for
this comparison; `phase=complete` in the CSV), finite states,
no estimator resets or ground contact, |roll/pitch| <45 degrees, hover height
within 0.2 m, hover XY within 0.5 m, final-three-second mean target error <0.5 m,
and mean speed <0.25 m/s. Passing means bounded/stable baseline flight, not
centimeter tracking or validated real-world sensor fidelity. Actual errors are
reported separately. Flow/ToF supply no absolute XY or yaw reference, so drift
under noise is expected even when flight remains stable.

The unit suite also checks C model consistency, schedule/rate/units, queue delays,
repeatable noise, existing stock ToF rejection, acceptance of a large ungated
innovation, and the absence of ground-truth fields in the closed-loop PID input.

## 6. Observed results

Verified in Isaac Sim 5.1 with seed 7 and the noise/delay settings above.
All four runs completed 30 s without estimator reset; all driver rejection counts
were zero. Each run executed 3001 predict calls (including initial dt=0),
30001 process-noise/finalize/externalize calls, 3000 Flow and 1200 ToF updates.

| Mode | Noise | Max Z estimation error, all / after 1 s (cm) | Hover XY estimation drift (cm) | Max / RMS attitude error (deg) | Bounded-flight checks |
|---|---|---|---|---|---|
| ekf_shadow | off | 10.124 / 0.389 | (0.000, 0.000) | 1.305 / 0.092 | PASS |
| ekf_shadow | on | 17.874 / 0.497 | (16.453, 22.964) | 1.208 / 0.185 | PASS |
| ekf_closed_loop | off | 10.127 / 0.378 | (0.000, 0.000) | 1.365 / 0.100 | PASS |
| ekf_closed_loop | on | 17.793 / 0.502 | (16.599, 23.103) | 1.237 / 0.230 | PASS |

Startup errors are included rather than hidden: stock initial Z is zero while
true Z starts at 0.1 m, and the first ToF arrives at 25 ms. Early Flow updates
with noisy measurements can also alter the height estimate before that return.
The after-1-s column describes the subsequent portion of the same run.

The noise-on closed-loop run remained airborne and bounded, but exhibited
substantial horizontal drift and small attitude oscillations. Its final true
position was (0.4173, -0.0766, 1.0005) m for target (0.5, 0, 1) m. The noise-off
closed-loop final position was (0.5009, -0.000001, 0.9997) m. No sensor-noise
parameters, EKF covariances, or PID gains were tuned to obtain these results.
This single seed is a reproducible baseline example, not a statistical robustness
study or an identification of real hardware noise.

Artifacts: `build/ekf_baseline/summary.json` and the four named CSV/PNG/JSON
sets. Re-run the smoke script to reproduce the reports or choose a new seed.

Additional verified checks: all 16 unit tests passed; the existing 14 PhysX
motor/sign tests and ground-truth C PID flight passed; Python altitude PID
step/disturbance, hover feedforward and free-fall regressions passed. The GUI
entry point ran `ekf_closed_loop --noise on --demo --steps 2000`, produced state
and measurement CSVs, and exited with status 0. Regression JSONs and unit-test
output are retained in `build/ekf_baseline/`.

## 7. Flow NIS diagnostics (logging only)

The historical results above predate the measured 33 g stock reference in
`build/step0_stock_33g_seed7/ekf`. Preserve that reference when evaluating changes.
Flow updates now record dimensionless `nis_x` and `nis_y` in the measurement CSV;
ToF rows contain NaN in those columns. No measurement gating, covariance tuning,
or controller change is included.

`kalmanCoreScalarUpdateWithNis` records `error * error / HPHR` using the exact
innovation variance of the C scalar update. The original scalar-update entry
point calls the same implementation without requesting a diagnostic. Flow X
and Y request separate outputs; Y uses covariance after X, retaining the stock
cached velocity/height prediction and original correction order.

`kalmanCoreGetFlowNisX/Y` expose the latest values through `cffirmware`.
The Python adapter reads them immediately after each flow call. Like the existing
`kalman_pred` values, they are module-wide diagnostics, not per-core storage;
they are NaN before the first flow update and remain held across core resets
until another flow update. Firmware LOG names are `kalman_pred.nisX` and
`kalman_pred.nisY`. The native bindings build uses `UNIT_TEST_MODE`, so it does
not verify the embedded LOG registration or STM32 build. Hardware validation
remains unperformed.

After rebuilding bindings (including forced `build_ext`), run and analyze:

```bash
/isaac-sim/python.sh sim/tests/smoke_firmware_ekf.py --seed 7 \
  --output-dir build/step1_nis_seed7/ekf
/isaac-sim/python.sh sim/plot_flow_nis.py \
  --input-dir build/step1_nis_seed7/ekf \
  --reference-dir build/step0_stock_33g_seed7/ekf
```

Use new output directories for later experiments. The analysis checks flight
CSV byte equality and exact equality of every original measurement CSV field.
It saves one histogram per case and CSV/JSON statistics: count, mean, median,
95th/99th/99.9th percentiles, and fractions strictly above 6.63 and 9.0. Phase
comes from the flight CSV at delivery time; the final `complete` sample is
assigned to move, so phase counts total all 3000 flow updates. No startup or
tail samples are discarded. Empirical single-seed percentiles do not establish
a universal chi-square calibration or independent-sample confidence bound.

## Step 2: synthetic flow faults and matched shuttles

The fault model is default-off, simulation-only, and never makes an EKF update
decision. The stock C correction is unchanged. Read-only C getters additionally
expose each scalar update's exact `HPHR` (pixels squared) and cached prediction
(pixels); the Y variance includes the preceding X correction. This avoids
inferring variance by dividing a reconstructed innovation by a small NIS.
These getters are exposed through the existing `mm_flow.h` SWIG include. Rebuild
bindings with forced `build_ext` after this change. STM32 remains unverified.

### Driver interpretation and limits

`src/deck/drivers/src/flowdeck_v1v2.c` first requires both aligned counts to
satisfy `abs(count) < 100`, updates its `lastTime`, then requires the **whole**
motion byte to equal `0xB0`. Zero counts with `0xB0` are enqueued. A different
motion byte is not enqueued. In particular, status dropout still updates the
driver clock, whereas a count-limit failure does not. Recovery after a status
dropout therefore normally has `dt=0.01 s`, not the duration of the dropout.
The fixed default standard deviation is 2 counts, with adaptive standard
deviation disabled. The default driver path does not reject by SQUAL.

Neither this driver nor the publicly available PMW3901 register summary proves
which values the device returns on a featureless floor. `zero` and `dropout`
are separate hypotheses. The synthetic dropout byte `0x30` only represents
`0xB0` with bit 7 cleared; it is not a claimed measured hardware response.
See the [Bitcraze Flow tutorial](https://www.bitcraze.io/documentation/tutorials/getting-started-with-flow-deck/)
and [PixArt PMW3901 datasheet](https://wiki.bitcraze.io/_media/projects:crazyflie2:expansionboards:pot0189-pmw3901mb-txqt-ds-r1.40-280119.pdf).

### Parameters and ordering

Both `run_sim.py` and `tests/smoke_firmware_ekf.py` accept:

| Flag | Default | Meaning |
|---|---|---|
| `--trajectory` | `hover-move` | Also `flow-shuttle-x` / `flow-shuttle-y` |
| `--flow-fault` | `off` | `spike`, `zero`, `dropout`, `scale` |
| `--flow-fault-seed` | Same number as `--seed` | Separate RNG with domain `0x464C4F57` |
| `--flow-fault-phase` | `move` | Clock-scheduled `takeoff`, `hover`, or `move` |
| `--flow-fault-offset` | 0.2 s | Nonnegative time after phase start |
| `--flow-fault-duration` | 1 s | Active window is `[start,end)`, at sample time |
| `--flow-spike-amplitude` | 20 counts | Each axis gets an independent random sign |
| `--flow-spike-probability` | 0.1 | Bernoulli per 10 ms read; both axes fire together |
| `--flow-scale-factor` | 0.5 | Positive finite multiplier on ideal flow |
| `--flow-audit` | off | Opt in to separate sensor-read and C-update CSVs |

The smoke runner also accepts `--flow-spike-amplitudes 5 10 20 50`, creating
four separate runs with the same seed and spike schedule. Values are in the
driver's aligned counts, before the EKF's multiplication by 0.1; no additional
axis swap or quantization is introduced.

For ideal counts `c` and the ordinary noise draw `e`, the candidates are:

- `off`: `c+e`.
- `spike`: `clip(c+e ± A, -99, 99)` when triggered, otherwise `c+e`. This
  synthetic clipping ensures both components pass the count limit; clipping
  and actual deltas are recorded. It is not a PMW3901 saturation model.
- `zero`: overwrite both components with exactly zero, motion `0xB0`.
- `dropout`: keep `c+e`, but set synthetic motion `0x30`.
- `scale`: `factor*c+e`; the noise and EKF R are not scaled. Large factors can
  reach the ordinary count limit and cause driver rejection.

The existing count limit and motion-byte checks run **after** these operations.
Every case still draws ordinary sensor noise, including zero/dropout samples.
Fault RNG draws never touch the sensor RNG. On identical truth sequences the
noise samples remain identical for all faults and after reset. In closed loop,
the trajectory can change; the recorded noise draws are compared separately.
ToF also consumes its scheduled noise draw when no downward floor return is
available; the invalid measurement remains unqueued. This fixes a pre-existing
conditional draw that shifted later IMU/flow noise after fault-driven extreme
attitudes. Every saved stock reference has valid ToF returns, so its draw order
and bytes are unchanged. The old RNG sequence on an invalid-ToF trajectory is
intentionally not preserved. Audit comparisons verify the matched noise draws.

### Trajectories and reference runs

Both shuttles hold altitude 1 m, yaw 0, and the other horizontal target at 0.
They take off during 0–10 s and hover during 10–20 s. Position follows a quintic
smoothstep on every leg, with zero velocity and acceleration at endpoints:

| Time [s] | 20 | 24 | 32 | 40 | 48 | 52 | 62 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Selected axis target [m] | 0 | +1 | -1 | +1 | -1 | 0 | 0 |

52–62 s is a separate recovery phase. Peak reference speed is 0.46875 m/s.
For level flight at 1 m, ideal counts are `4.8832213*v/z`, peaking at about
2.289 counts. Actual ideal flow also contains body rotation and tracking error;
it must be read from the sensor audit. X and Y follow the clear central axes of
the existing arena. No scene, navigation, or safety changes are made.

Use `--flow-fault-offset 6 --flow-fault-duration 4` for the campaign window
26–30 s, across the sustained middle portion of the +1 to -1 m leg. These are
explicit campaign overrides, not changed defaults. A new trajectory requires
its own faults-off reference. Run X first; Y flight runs require the user's
later instruction. Y schedule unit tests do not constitute flight validation.

Commands inside `cf_sim_dev` (use a fresh output directory each experiment):

```bash
/isaac-sim/python.sh sim/tests/smoke_firmware_ekf.py --seed 7 \
  --output-dir build/step2_flow_faults_seed7/compatibility
# Compare all eight existing CSVs byte-for-byte with Step 1 before continuing.
/isaac-sim/python.sh sim/tests/smoke_firmware_ekf.py --seed 7 \
  --trajectory flow-shuttle-x --flow-audit \
  --output-dir build/step2_flow_faults_seed7/shuttle_x_reference
/isaac-sim/python.sh sim/analyze_flow_faults.py detectability \
  build/step2_flow_faults_seed7/shuttle_x_reference/flow-shuttle-x_ekf_closed_loop_noise_on.csv \
  --output-dir build/step2_flow_faults_seed7/detectability
# Review the detectability table before any fault flights.
/isaac-sim/python.sh sim/tests/smoke_firmware_ekf.py --seed 7 \
  --mode ekf_closed_loop --noise on --trajectory flow-shuttle-x \
  --flow-fault spike --flow-spike-amplitudes 5 10 20 50 \
  --flow-fault-offset 6 --flow-fault-duration 4 \
  --output-dir build/step2_flow_faults_seed7/spikes
# Repeat separately for --flow-fault zero, dropout, scale, with identical
# trajectory, mode, noise, seed, offset, duration and separate output directories.
```

### Audit and interpretation

Faults off preserves the original flight/measurement CSV schema and bytes;
the default produces no extra audit CSVs. Explicit `--flow-audit` adds sidecars
without changing those original CSVs. Faults on enables audit automatically
and adds `flow_sample_id`, `fault_type`, `fault_window_active`, `fault_active`,
`fault_changed_x/y` to **all** measurement rows, including clean intervals.
ToF has `not_applicable` labels. `fault_active` means the operator fired;
it is distinct from being inside the window or having nonzero effect.
Labels are fixed at sample time and copied to CSV after the C update.

`*.flow_samples.csv` records every flow read, including drops: ideal/noise/
clean/final counts, actual fault deltas, motion byte, count/status checks,
enqueue status, drop reason, read/driver dt, clipping, and scheduled delivery.
`*.flow_updates.csv` records only actual C updates, joined by `flow_sample_id`,
with exact C HPHR, prediction and NIS. Dropout has **no update or NIS** and
must never be counted as gate rejection. A scheduled delivery is not proof of
consumption; join against the update CSV, especially with nonzero delays.

`analyze_flow_faults.py detectability` saves ideal counts versus time and a
pre-fault table for both the default and campaign windows. It uses exact stock
HPHR and prediction, includes the noise variance for spikes/scale, and omits
that term for zero (noise is overwritten). Predictions condition on the stock
reference, not on a fault-adapted filter. `summarize` reports actual stock-run
errors and the fraction of NIS exceeding 9. This is a **hypothetical threshold
exceedance**, never an implemented gate rejection rate. Clean reference rates
are separate from inactive/recovery samples of fault runs.

Tests cover RNG separation/reset, bounds, probability extremes, half-open
windows, delayed labels, zero/scale arithmetic, dropout clock/update behavior,
audit joins, invalid-ToF RNG scheduling, unchanged default CSVs, C HPHR, and both shuttle schedules. Keep
the Step 0 failing-test baseline; report only changes. None of this proves a
real floor's texture response, fault frequency, a universal threshold, hardware
performance, or that a future gate can detect scale/zero or recover dropout.
