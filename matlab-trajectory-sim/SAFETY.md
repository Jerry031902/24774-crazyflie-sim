# Safety supervision in the MATLAB simulator

An optional Python/Isaac integration of this architecture is now available in
[`sim/SAFETY.md`](../sim/SAFETY.md). Its PID/PhysX braking bounds require separate
live validation; the MATLAB stopping-distance results do not transfer directly.

The default run enables a rule-based safety supervisor between WedgeBug and
`dynamics_step`. Navigation still proposes body-frame velocity/yaw-rate commands.
Two parallel monitors can require a fallback command; the command arbiter is the
only command owner. This implements the Safety layer of the project architecture.
The existing first-order velocity model approximates the Control layer and plant;
no EKF, altitude, attitude controller or rotor model has been added to MATLAB.

```matlab
p = default_params();          % p.safety.enabled is true
p.safety.enabled = false;     % original navigation and sensor timing
```

## Sensor and state inputs

`sim_log.ranges` remains the held raw front/left/back/right ranger readings. Each
delivered frame additionally has its measurement/delivery times, sequence number,
measurement position/yaw, and per-direction status: 0 not received, 1 hit,
2 valid no return, 3 invalid. Positive Inf in a received frame means no return
within `range_max`; it does not mean that all space is clear. Startup waits for
three distinct healthy frames. Reading a held value repeatedly cannot satisfy
the recovery counter. Backwards sequence numbers/timestamps are rejected.

Range Processing uses a short median window of distinct samples. Translation
reduces older ranges conservatively, and turning resets the history. Closing
rate is diagnostic; the static-world monitor uses observed vehicle velocity.
Scanning records raw readings once per measurement with their sample yaw/time,
and waits for measurements at both ends of the sweep. Near-target motion remains
nose-first while safety is enabled, avoiding the original unchecked diagonal slide.

Navigation and safety get the model state, not a simulated EKF. Ground velocity
is obtained from successive observed positions. Environmental edges are used
only by the sensor generator and collision/metric code, never by the monitors.

## Primary and backup

The Primary evaluates one candidate-command step, followed by braking. Its
stopping distance exactly matches the semi-implicit discrete first-order model,
including acceleration saturation and the exponential tail. The clearance budget
includes sample age, a control step, body radius, sensor-face offset, cone angle,
and range-noise margin. Significant commands outside the observed cones are
rejected until the vehicle turns and receives a suitable measurement.

The Backup reads the raw frame and separate completion timestamps. It checks
frame validity/age, sequence ordering, estimator finiteness, a very close hit in
any direction, a conservative stopping envelope for actual motion, and primary /
navigator heartbeats. It bypasses the median filter, candidate assessment and
primary stopping-distance function. It shares the physical sensor and vehicle
model assumptions: it is processing diversity, not independent hardware redundancy.

Both monitors are braking overrides in this version; there is no CBF/QP/MPC
optimization or formal all-future-time safety claim. Margins are statistical,
not bounds on unbounded Gaussian noise. The nine-ray sensor approximation has
blind space between cones; real optical/material effects and moving obstacles
are not modelled. The parameters have not been identified on the team's aircraft.

## Arbitration and recovery

`safety_state` is separate from navigation `mode`:

| Code | State | Action |
|---|---|---|
| 0 | DISABLED | Original navigator output |
| 1 | NORMAL | Assess and accept/reject the navigation candidate |
| 2 | BRAKE | Override translation and yaw; wait for low ground speed |
| 3 | HOLD | Hold the stopped position and await healthy new frames |
| 4 | RECOVER | Reassess commands with a gradual speed ramp |

Backup intervention cannot be cleared by a primary approval. Faults must become
healthy and three fresh frames must pass before recovery. Candidate navigation
state is committed only when its command is accepted. An interrupted translation
invalidates the pass point/partial scan and restarts a sweep, preserving the
current waypoint, chosen side and episode/loop history. A stationary timestamped
scan is paused and continued, preventing noisy close returns from restarting it
indefinitely. Sensor and monitor updates continue during every override.

With no wind, BRAKE commands zero body velocity and yaw rate. For the model's
additive wind drift, the fallback estimates drift from displacement minus internal
velocity, compensates it, and adds bounded position feedback during HOLD. It
does not read `p.wind` or the true gust. Invalid state still triggers an override,
but successful physical holding cannot be claimed without a usable estimate.

## Diagnostics and validation

`main` prints brake/override metrics and opens `plot_safety` after playback.
The map/playback marks override motion in red and reports safety state separately.
The diagnostics show nominal, assessed and selected commands, ground speed,
predicted stopping distance, monitor margins, measurement age and trigger states.
`safety_events` records brake, stopped and resume events, including the override
reason/source and measured brake time/path length. `safety_last_error` retains
exceptions caught by the fail-safe paths.

```matlab
setup_paths;
r = runtests('tests'); assertSuccess(r);
[comparison, faults] = safety_validation(); % 60 on/off runs: 3 noises x 10 seeds
```

The validation function writes CSVs, MAT logs and PNG figures to `build/safety`.
Time-triggered static obstacles have a separate `demo_popup` entry point and
`popup_validation` benchmark under `build/popups`; see [POPUP.md](POPUP.md).
They use the same bottom-level velocity response. The schedule never informs
either safety monitor; reaction is limited by the ordinary ranger delivery.
Measured main-mission comparison, seeds 1-10 at each noise level:

| Noise std dev | Safety | Completed | Collisions | Worst body gap | Median finish | Mean brakes |
|---|---|---|---|---|---|---|
| 0 cm | off | 10/10 | 0 | 0.107 m | 70.90 s | 0 |
| 0 cm | on | 10/10 | 0 | 0.152 m | 77.70 s | 0 |
| 5 cm | off | 10/10 | 0 | 0.028 m | 79.32 s | 0 |
| 5 cm | on | 10/10 | 0 | 0.081 m | 98.89 s | 7.1 |
| 10 cm | off | 10/10 | 0 | 0.045 m | 85.14 s | 0 |
| 10 cm | on | 10/10 | 0 | 0.115 m | 173.69 s | 34.6 |

The conservative settings increase clearance at a substantial time cost under
10 cm noise, including noise-induced stops and recovery scans. These finite
simulations do not establish a collision probability or hardware performance.
All three one-second dropout/heartbeat faults stopped and subsequently completed
the open-room mission: braking took 0.68 s and approximately 0.108 m.

Legacy tests explicitly disable supervision; safety-enabled missions, commands,
startup, timestamped sweeps, hold under wind and faults have separate tests.
The old sweep-distance assertion failed before this change: its 5 cm tolerance
ignored delayed sensing and velocity lag. It now checks a model-based delay and
braking envelope without changing the disabled navigator's behavior.

The optional sixth argument of `run_simulation` injects deterministic simulation
faults. Time windows are half-open `[start end)`:

```matlab
faults = struct('primary_timeout', [3 4]);
L = run_simulation(start, waypoints, goal, env, p, faults);
```

`ranger_dropout`, `primary_timeout`, and `navigator_timeout` take N-by-2 windows.
`processed_bias` takes N-by-3 `[start end metres]` rows, corrupting only the
processed path so the raw Backup path can be verified independently.
Heartbeat tests simulate missed completions. A single MATLAB loop cannot detect
its own total hang; an independently scheduled watchdog/task is required when
porting this design to firmware. Python/C/Isaac have not yet been changed.
