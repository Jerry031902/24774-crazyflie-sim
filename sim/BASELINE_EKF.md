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

All noise/delay settings are constants at the top of `sensors.py`. Default
noise-on assumptions are independent Gaussian samples: accelerometer 0.005 G per
axis, gyro 0.1 deg/s per axis, Flow 2.0 firmware counts per axis, ToF 0.003 m.
These have not been identified from the team's hardware. IMU noise is added
before its software LPFs. All additional delays default to zero; their integer
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

Both modes use the same clock-based targets: takeoff/settle to (0,0,1) during
t=0..10 s, hover during t=10..20 s, then move/settle at (0.5,0,1) until t=30 s.
The GUI keeps holding afterward. This avoids using true position to decide when
closed-loop waypoints switch, and avoids noisy estimates continuously resetting
a settling timer. Existing modes retain their original behavior/demo.

## 5. Running and evaluating

Host commands, using the existing development container:

```bash
docker exec -it cf_sim_dev /isaac-sim/python.sh /workspace/sim/run_sim.py --mode ekf_shadow --demo --noise off
docker exec -it cf_sim_dev /isaac-sim/python.sh /workspace/sim/run_sim.py --mode ekf_closed_loop --demo --noise on
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
error between the beginning/end of t=10..20 s; true hover drift and maximum XY
error are also recorded. Attitude error is quaternion angular distance, with both
max and RMS, avoiding Euler wrapping artifacts.

The predeclared smoke-test stability criteria are: complete 30 s, finite states,
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
