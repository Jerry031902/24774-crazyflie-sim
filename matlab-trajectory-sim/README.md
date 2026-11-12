# Crazyflie Reactive Navigation Simulation

This is a top-down 2-D simulation of a Crazyflie with a Multi-ranger deck flying through waypoints in a room. It uses only the deck's four rangers (front, left, back, right) to avoid obstacles it has no map of. In free space it flies a straight line to each waypoint. When an obstacle blocks the way it uses WedgeBug: it stops, sweeps its rangers across ±45° to see where the obstacle ends on each side, and goes round the shorter way. It only ever moves in a direction one of its sensors faces. It runs in plain MATLAB with no toolboxes.

## Running it

```matlab
main               % fly the mission in main.m: metrics, plots (Figure 1), playback (Figure 2)
runtests('tests')  % unit and closed-loop tests
demo_popup        % obstacle appears during flight; delayed sensing and braking
```

Figure 2 has a **Replay** button (it reads **Stop** while playing) and a speed menu, so you can watch the run again at any time.

`demo_popup` tests a static obstacle appearing at a specified time in front of
the cruising vehicle. Change its body gap, appearance time, noise or Safety flag
in the script. `popup_validation` compares stopping boundaries across distance,
sensor phase and noise; see [POPUP.md](POPUP.md) for results, replay and controller
interpretation. Normal `main` retains the original pre-existing obstacle mission.

To change the mission, edit the **Mission** section of `main.m`. Positions are in metres inside the 10 × 8 m room:

```matlab
start     = [0.8 0.8];
waypoints = [3.4 4.2; 6.6 2.0; 9.4 3.8];   % or [] to fly straight to the goal
goal      = [6.4 6.4];
obstacles = {obstacle_square([2.5 3], 1), obstacle_rect([5.5 4], [0.2 3.4]), ...
             obstacle_circle([8.0 2.9], 0.3), obstacle_rect([7.6 5.3], [1.6 0.5])};
```

Obstacles are squares (`obstacle_square(center, side, angle_deg)`), rectangles (`obstacle_rect(center, [w h], angle_deg)`) and circles (`obstacle_circle(center, r)`). Overlapping rectangles can be combined into larger shapes such as an L or a U.

Any number of waypoints works, including none. `run_simulation(start, waypoints, goal, env, p)` accepts `[]`, `{}` or `zeros(0, 2)` for no waypoints, and the drone then flies straight to the goal.

Before flying, `run_simulation` checks the start, waypoints and goal. It stops with an error naming any point that is inside an obstacle or closer than `p.nav.d_wall` to one, because the drone could never reach it.

## Each tick

Safety supervision is enabled by default. A primary stopping-distance monitor
and a separate raw-range/timeout backup arbitrate navigation and braking/holding
commands before the model update. Set `p.safety.enabled = false` to reproduce the
original navigator. See [SAFETY.md](SAFETY.md) for the architecture, diagnostics,
fault injection and the on/off validation runner.

```
read_rangers -> range_processing -> navigator candidate -> primary monitor
      |                                                     |
      +----------> raw backup monitor ----------------> command arbiter
                                                            |
                                                      dynamics_step
```

The selected command is the same as a Crazyflie hover setpoint (`vx, vy, yawrate`),
without height. The disabled-safety path is the original `read_rangers ->
navigator_step -> dynamics_step` loop.

## Navigator (`src/nav/navigator_step.m`)

The rangers turn with the body, so the drone points a sensor along every move. Before changing direction it brakes almost to a stop, and it only yaws while stopped or, slightly, while flying nose first.

**GO_TO_GOAL**

The drone turns on the spot to face the current target, then flies straight at it nose first, so the front ranger watches the whole line. If the heading drifts more than `yaw_tol` off the target, it stops and turns again.

It switches to AVOID if either of these happens:

- An obstacle is ahead and short of the target. The drone slows and comes to rest `d_sweep` (1 m) from it, then sweeps.
- A side ranger reads less than `d_side`. This means a wall is closing in at a shallow angle, so the drone goes straight to wall-following.

**AVOID (WedgeBug)**

1. **Sweep.** The drone hovers and yaws from -45° to +45° about the blocked heading, logging the front ranger at each angle. The left and right rangers sit 90° off the nose, so the same sweep also scans +45° to +135° and -135° to -45°. That gives a 270° view.
2. **Find the edges.** On each side, it walks outward from the middle of the scan to where the obstacle's outline ends: either the range jumps up by more than `edge_jump`, or there is no return. Each reading is the nearest hit anywhere in the sensor's 27° cone, so the drone places each point half a cone inward. That puts an edge on the corner rather than 13.5° past it.
3. **Pick the shorter way.** Each side costs `|drone → edge| + |edge → target|`. The first sweep of an episode picks the cheaper side, and the drone keeps that side until it has got past the obstacle, so it can't dither between the two.
4. **Go round.** What happens depends on whether the chosen edge was seen:
   - **Edge seen:** the drone flies nose first to a pass point outside the edge, then flies straight at the target again. The pass point starts `pass_margin` out from the edge, square to the line of sight. It then moves further out (up to `pass_max`) until the next leg, from the pass point to the target, stays `tangent_clearance` from the obstacle outline the sweep saw. It only moves through space the sweep saw as empty. A reading `r` means the whole of that ranger's cone is clear out to `r`, so the flight to the pass point is clear too. If more of the obstacle blocks the way, for example a far face the first sweep couldn't see, the drone sweeps again from wherever it is.
   - **Edge out of view** (the obstacle runs past ±135°): it follows the obstacle instead, along body axes, with the wall on the right when going round the left and vice versa.
     - Forward blocked: turn away from the wall.
     - Wall reading jumps long: fly `corner_margin` further, then turn back toward the wall.
     - Wall closer than `d_close`: step away.

     It leaves the wall once it is closer to the target than its hit point, and the free space it can see toward the target reaches closer than any point of the wall followed so far. That second condition is TangentBug's rule, which WedgeBug builds on. It stops the drone leaving inside a pocket whose back wall stands between it and the target.

**Episodes and unreachable targets**

An episode runs from first contact with an obstacle until the drone is closer to the target than where that contact happened. So each new episode starts closer to the target than the last.

The target is reported as unreachable and skipped if any of these happens without the drone getting closer:

- it circles the target by more than `max_winding` (1¼ turns)
- it comes back to an earlier sweep spot
- wall-following returns to where it started
- it reaches `max_sweeps` sweeps

**Reached**

Within `wp_tol` of a target, the drone moves on to the next one. With supervision
disabled, the last `yaw_free_radius` is a direct slide without turning. Safety-enabled
motion keeps a ranger facing the move through the final approach. After the goal,
the mode is DONE.

**Hysteresis and unknown readings**

Every rule above that compares a reading with a distance (`d_side`, `d_sweep`, `d_detect`, `d_wall`, `d_close`, `d_open`, and the obstacle to head for after squaring up) switches on at its threshold and back only once the reading is `p.nav.hysteresis` (0.05 m) past it. A reading wavering at a threshold therefore can't flip a decision back and forth, and no safety distance is reduced.

A reading of `NaN` is unknown (an invalid reading). It changes no rule's state, never counts as free space (not an edge in a sweep, not a clear way, not a reason to leave a wall) or as an obstacle, and the drone does not fly toward it until that ranger reads again. If every reading straight ahead in a sweep is unknown, the way counts as blocked.

**Readings paired with their heading, wall fit, and passing speed**

`run_simulation` gives the navigator each ranger frame (`ranger_frame`: sequence number, measurement time and the yaw it was measured at), with Safety on or off.

- **Sweeps** log each frame once, at the yaw it was measured at, not the yaw when it arrived 100 ms later. They wait at both ends of the wedge for a frame measured there. The final slide onto a target without turning is replaced by flying nose first.
- **Wall-following** starts square to the nearest room axis, then fits the wall's direction from successive wall-side readings. Each frame gives a wall point; once the drone has moved `wall_fit_travel` (0.1 m), the line between two points is the wall. `psi_hold` turns `wall_fit_gain` (half) of the way to parallel, ignoring fits more than `wall_fit_max` (30°) off, such as at corners. Rotated walls are followed along their length, not along a room axis.
- **Passing speed:** flying to a point, the speed is at most `k_stop` × (nearest side reading − `d_close`), the same rule as for obstacles ahead, so the drone slows down passing close to an obstacle beside it. Wall-following applies it to the side away from the wall, never below `v_turn`.

## Model

State `x = [px; py; vx; vy]`, yaw `psi`, and a command of `u_body = [forward; left]` plus a yaw rate `r`:

```
v_cmd = R(psi) * u_body          |v_cmd| <= v_max
dv/dt = (v_cmd - v) / tau        |dv/dt| <= a_max
dp/dt = v + wind
dpsi/dt = r                      |r| <= yaw_rate_max
```

**Rangers.** The model follows the real [Multi-ranger deck](https://www.bitcraze.io/products/multi-ranger-deck/) and its stock firmware driver (`src/deck/drivers/src/multiranger.c`). The Isaac Sim deck (`sim/ranger.py`) uses the same model in 3-D.

- **Field of view:** each VL53L1x is a 27° cone of rays (`p.ranger.fov`, `n_rays`), the sensor's full field of view, which the driver never narrows. It reports the nearest hit within `range_max = 4 m`, and `Inf` when nothing is in range.
- **Sensor position:** rays start at the sensor face, `p.ranger.offset` ahead of the body centre (16.25 mm front/back, 16.3 mm left/right, from the deck drawing), so readings are that much shorter than the centre's distance.
- **Timing:** the driver reads every sensor every 100 ms (`p.ranger.period`), one period after starting that measurement (`p.ranger.latency`). Each reading is therefore 100 ms old when the navigator gets it and is held until the next one; before the first, the navigator sees `Inf`.
- **Resolution:** readings are whole millimetres (`p.ranger.resolution`).
- **Invalid readings:** with probability `p.ranger.p_invalid` (0 by default) a reading is `NaN`, unknown, as when the real sensor reports an invalid status on a dark, shiny or steeply angled surface. The stock driver reports invalid readings and "nothing within 4 m" alike as 32767 mm, so on the drone they can only be told apart if the driver also passes on the reading's status.

The deck's fifth (up) sensor is not modelled in 2-D. The room walls are visible to the sensors. With `fov = 0` each sensor is a single centre ray, which can miss a corner that is within the drone's own width.

**Noise.** Each reading that hits something gets zero-mean Gaussian noise with standard deviation `p.ranger.noise + p.ranger.noise_rel × range`, drawn independently for each sensor and each update. By default `p.ranger.noise` is the team's bench measurement, `p.ranger.noise_measured` = 0.002 m (1.4-2.0 mm std, front/back/left/right at 0.2 and 0.5 m), and `noise_rel` is 0. Set `p.ranger.noise = 0` for ideal ranging. `main.m` keeps a commented-out 0.1 m line as a stress test.

## Ranger noise study

`noise_study(start, waypoints, goal, obstacles, p)` flies one mission 10 times at each noise level, 0, 1, 2, 5, 10 and 20 cm std dev, with a different random seed for each run. It prints a summary and plots outcome, closest approach, time to goal and sweeps against the noise level. The last section of `main.m` runs it on the mission defined there.

With safety supervision disabled, the hardware ranger timing and sensor
positions, 0.05 m hysteresis, readings paired with their heading, the wall
fit and the passing-speed limit, the run takes 75.4 s with a closest gap of
0.13 m with the default 2 mm ranger noise, and 73.3 s / 0.13 m without noise
(2-D Python port; it matches MATLAB exactly without noise, while with noise
the two draw different random numbers).
The table below is historical: it was measured with the original navigator
and the previous ranger model (25 Hz, no delay, sensors at the body centre,
no hysteresis). Run `safety_validation` for a current on/off comparison, or
`noise_study` for the settings in the supplied parameter struct.

| Noise (std dev) | Succeeded | Collided | Closest gap, worst run | Time to goal, median |
|---|---|---|---|---|
| 0 cm | 10/10 | 0 | 0.21 m | 65.8 s |
| 1 cm | 10/10 | 0 | 0.20 m | 66.6 s |
| 2 cm | 10/10 | 0 | 0.19 m | 67.6 s |
| 5 cm | 10/10 | 0 | 0.11 m | 73.6 s |
| 10 cm | 10/10 | 0 | 0.09 m | 82.7 s |
| 20 cm | 4/10 | 0 | 0.09 m | 174 s |

At 20 cm, 4 runs were still re-sweeping when they hit `t_max`, and 2 gave up on a reachable waypoint. The noise starts to look like obstacle edges, because `edge_jump` is 0.5 m. None of the runs collided at any level.

## Layout

| Folder | Contents |
|---|---|
| `main.m` | Entry script; the mission is defined here |
| `src/config` | `default_params`: every setting in one struct (vehicle, rangers, navigator) |
| `src/env` | `make_environment`, `obstacle_square`, `obstacle_rect`, `obstacle_circle`, `obstacle_distance` |
| `src/sensors` | `read_rangers` |
| `src/nav` | `navigator_init`, `navigator_step`, `validate_waypoints`, `target_names` |
| `src/model` | `dynamics_step`, `sat_norm` |
| `src/sim` | `run_simulation` |
| `src/metrics` | `compute_metrics`, `print_metrics` |
| `src/analysis` | `noise_study`: the mission flown repeatedly with noisy rangers |
| `src/viz` | `plot_results` (map coloured by mode, hit and leave points, sensor/speed/clearance plots), `animate_run` (rays, leave zone, replay, MP4 export) |
| `tests` | `test_drone_sim`: limits, sensors, straight-line flight, square/rectangle/circle and L/U-of-rectangles runs (every move sensed), ranger noise, unreachable and invalid waypoints, no waypoints |

## Metrics

`compute_metrics` reports:

- whether the drone reached the goal, and its finish time
- collisions
- targets reached, and the names of any that were unreachable
- flown path length, and its ratio to straight lines through the route
- number of AVOID episodes and time spent in AVOID
- minimum clearance and near misses
- peak speed

The result is a scalar struct, so a parameter sweep can stack the runs into a struct array and view them with `struct2table`.
