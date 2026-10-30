# Crazyflie Reactive Navigation Simulation

This is a top-down 2-D simulation of a Crazyflie with a Multi-ranger deck flying through waypoints in a room. It uses only the deck's four rangers (front, left, back, right) to avoid obstacles it has no map of. In free space it flies a straight line to each waypoint. When an obstacle blocks the way it uses WedgeBug: it stops, sweeps its rangers across ±45° to see where the obstacle ends on each side, and goes round the shorter way. It only ever moves in a direction one of its sensors faces. It runs in plain MATLAB with no toolboxes.

## Running it

```matlab
main               % fly the mission in main.m: metrics, plots (Figure 1), playback (Figure 2)
runtests('tests')  % unit and closed-loop tests
```

Figure 2 has a **Replay** button (it reads **Stop** while playing) and a speed menu, so you can watch the run again at any time.

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

```
read_rangers  ->  navigator_step  ->  dynamics_step
4 distances      GO_TO_GOAL / AVOID    kinematic point, velocity lag,
(held between    -> [forward; left]    speed, acceleration and
 updates)           + yaw rate         yaw-rate limits
```

The command is the same as a Crazyflie hover setpoint (`vx, vy, yawrate`), without height.

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

Within `wp_tol` of a target, the drone moves on to the next one. The last `yaw_free_radius` is a direct slide, without turning. After the goal, the mode is DONE.

## Model

State `x = [px; py; vx; vy]`, yaw `psi`, and a command of `u_body = [forward; left]` plus a yaw rate `r`:

```
v_cmd = R(psi) * u_body          |v_cmd| <= v_max
dv/dt = (v_cmd - v) / tau        |dv/dt| <= a_max
dp/dt = v + wind
dpsi/dt = r                      |r| <= yaw_rate_max
```

**Rangers.** Each VL53L1x is modelled as a 27° cone of rays (`p.ranger.fov`, `n_rays`). It reports the nearest hit within `range_max = 4 m`, and `Inf` when nothing is in range, refreshed every `p.ranger.period`. The room walls are visible to the sensors. With `fov = 0` each sensor is a single centre ray, which can miss a corner that is within the drone's own width.

**Noise.** Each reading that hits something gets zero-mean Gaussian noise with standard deviation `p.ranger.noise + p.ranger.noise_rel × range`, drawn independently for each sensor and each update. Both terms are 0 by default. To fly the main run with noise, uncomment the `p.ranger.noise` line at the top of `main.m`.

## Ranger noise study

`noise_study(start, waypoints, goal, obstacles, p)` flies one mission 10 times at each noise level, 0, 1, 2, 5, 10 and 20 cm std dev, with a different random seed for each run. It prints a summary and plots outcome, closest approach, time to goal and sweeps against the noise level. The last section of `main.m` runs it on the mission defined there.

Results for the current mission:

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
