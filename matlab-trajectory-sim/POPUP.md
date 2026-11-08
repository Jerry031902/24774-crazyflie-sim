# Time-triggered static obstacles

Run `demo_popup` in the MATLAB project directory. A box appears during steady
flight, remains fixed, and is first discovered through the existing Multi-ranger.
The default demonstration uses a 0.24 m **body-to-obstacle** gap at 4.02 s,
0.5 m/s cruise, 10 Hz ranging and 0.1 s measurement latency.

In `demo_popup.m`, change `gap`, `appear_time`, `p.ranger.noise`, or
`p.safety.enabled`. Useful gaps are 0.55 m (ample), 0.20/0.24 m (around the
current zero-target braking boundary), and 0.05 m (too close). Both modes use
the same `dynamics_step`, acceleration limit, time constant and ranger timing.

## Scheduling your own obstacle

```matlab
setup_paths;
p = default_params();
env = make_environment(p, {});
env = schedule_obstacle(env, obstacle_rect([3 4], [0.12 1.2]), 4.02);
L = run_simulation([1 4], [], [9 4], env, p);
plot_popup(L, env, p);
animate_run(L, env, p, struct('speed', 1, 'start_time', 3.6));
```

`schedule_obstacle` takes fixed world geometry and an appearance time. Multiple
events can be supplied in any order. A non-grid time activates on the next
integration tick. The geometry changes before that tick's sensor sample and
collision check. An obstacle that appears around the body counts as an immediate
collision, including when its edges do not touch the body centre's radius.

The schedule is only used for sensor generation, collision checking and scoring.
It is never passed to navigation, range processing, either monitor, or the arbiter.
An already queued sample retains the old geometry; no update is forced when an
obstacle appears. `obstacle_distance(env, positions, times)` evaluates only the
geometry existing at each time. Summary maps show future boxes dashed and labelled;
playback actually hides them until their activation time.

## Same control stack, different command selection

The nominal WedgeBug output and the fallback output feed the same velocity-response
model through the arbiter. Safety adds sensing checks, prediction, override priority,
holding feedback and recovery. In no wind, its BRAKE command is zero horizontal
target velocity and zero yaw rate. Actual velocity decays under the existing
first-order response and acceleration saturation; it is not reset instantly.
HOLD adds bounded position feedback and inferred-drift compensation.

This is an added supervisory control stack. There is no new attitude/PID controller
in MATLAB and no increase to `a_max` or change to `tau`. The nominal navigator
already commands zero velocity when it detects a blocking obstacle. Consequently,
Safety on/off can have the same first braking time and collision boundary.
The Backup still provides an independently checked raw-data/timeout override.

For steady 0.5 m/s and default settings, a zero target travels another 0.115 m.
An appearance at 4.02 s misses the 4.00 s sample; the 4.10 s measurement arrives
at 4.20 s. The 0.18 s reaction interval adds about 0.09 m, giving approximately
0.205 m required body gap for this particular zero-target response.

`popup_summary` also reports a best-case straight-line maximum-deceleration
reference for the same discrete model. A collision below the zero-target boundary
does not establish that every possible controller must collide. A new braking
control law could use the available acceleration differently; this change tests
the current controller and does not add that alternative law. Moving/crossing
obstacles and real hardware dynamics are outside this static-popup test.

## Validation and outputs

```matlab
r = runtests('tests'); assertSuccess(r);
report = popup_validation();
```

Default validation runs 126 encounters:

- 66 noiseless encounters: 11 initial body gaps, 3 sensor phases (4.00, 4.02,
  4.08 s), and Safety off/on.
- 60 noise encounters: 3 gaps, 5/10 cm noise, seeds 1-5, and Safety off/on.

`popup_scenario` first runs an obstacle-free reference offline to set the box's
fixed coordinates. This matches appearance gap, cruise speed and sensor phase
between the modes, whose startup trajectories differ. The actual trial is then
rerun from the start using the fixed box schedule. This benchmark calibration
does not give the controllers any geometry or advance notice.

Results are written under `build/popups`:

| File | Contents |
|---|---|
| `encounters.csv` | Every encounter, reaction/braking times, stopping references and outcome |
| `distance_sweep.csv` / `noise_stress.csv` | Separate noiseless boundary and noisy encounter data |
| `summary.csv` | Grouped collision/arrival fractions |
| `distance_boundary.png` | Clearance against initial body gap for each sensor phase |
| `gap_050_on.png` | Too-close encounter |
| `gap_180_on.png` | Insufficient for the current delayed zero-target response |
| `gap_240_on.png` | Stop with a small positive gap |
| `gap_550_on.png` | Ample gap, stopping and subsequent navigation |
| `gap_*_on.mat` | Full logs, scene and settings for replay |
| `all_tests.mat` / `validation.log` | MATLAB test results and validation console output |
| `before_popups.zip` | Source snapshot before this popup implementation |

Times in the CSV are absolute simulation times. `scheduled_reaction_s` is the
sensor-scheduling delay. `zero_target_required_gap_m` adds that reaction travel
to the current model's stopping distance. NaN for a fresh delivery or override
means it never happened before the run ended (for example, contact occurred first).
The minimum gap is signed: a negative value means body penetration/contact.
Runs intentionally below the stopping boundary are recorded as collisions rather
than being counted as successful avoidance. A stopped run that cannot complete
the route is distinguished from an arrived run.

## Observed results (default model)

The full regression suite passed 50 tests; all 126 benchmark encounters were
recorded, including the intentionally too-close collisions. Noiseless on/off
encounters had the same collision classification at every tested gap and phase.
The zero-target body-gap boundary was approximately 0.165 m at 4.00 s,
0.205 m at 4.02 s, and 0.175 m at 4.08 s. The different times change when the
next ranger sample can be delivered, while controller parameters stay fixed.

At the 4.02 s appearance, Safety-enabled noiseless examples were:

| Initial body gap | First fresh delivery | First zero command | Result |
|---|---|---|---|
| 0.05 m | Not delivered before contact | Not issued | Collision before perception |
| 0.18 m | 4.20 s | 4.20 s | Collision during braking |
| 0.24 m | 4.20 s | 4.20 s | Speed below 0.03 m/s at 4.88 s; route completed |
| 0.55 m | 4.20 s | 4.30 s | Stopped and completed the route |

The 0.24 m example retained about 0.042 m body clearance at the stopped-speed
threshold (a small remaining velocity tail subsequently reduces that gap).
The 0.55 m case brakes through nominal WedgeBug after its filtered observation;
the monitors do not override it. Independent raw Backup intervention is visible
in the closer examples.

With 5/10 cm noise, all five seeds at 0.55 m completed in both modes. All five
0.05 m cases collided in both modes. At 0.22 m all cases stopped without contact;
the original navigator completed, while Safety on remained stopped or replanning
without reaching the goal within the 90 s cap. This records the conservative
near-obstacle recovery limitation; a successful stop is not a successful mission.
There is no special post-brake escape controller in this implementation.

To replay a saved example:

```matlab
S = load('../build/popups/gap_240_on.mat');
plot_popup(S.L, S.env, S.p);
animate_run(S.L, S.env, S.p, struct('speed', 1, 'start_time', 3.6));
```
