#!/usr/bin/env python3
"""2-D kinematic test simulator for the WedgeBug navigator (no Isaac Sim).

Port of matlab-trajectory-sim/src/sim/run_simulation.m, dynamics_step.m and
compute_metrics.m. Use it to check the navigator quickly before flying it in
Isaac Sim, and to check that the C core matches the Python reference:

    python3 sim/navigation/sim2d.py                         # Python reference
    PYTHONPATH=build python3 sim/navigation/sim2d.py --backend firmware
    python3 sim/navigation/sim2d.py --plot build/missions/matlab_main_2d.png
"""

import argparse
import math
from pathlib import Path
import sys

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from navigation.firmware_nav import make_navigator
    from navigation.geometry import obstacle_distance, read_rangers, rot2d, sat_norm, target_names, \
        validate_waypoints, wrap_to_pi
    from navigation.missions import MISSIONS
    from navigation.params import default_params
    from navigation.wedgebug import MODES, PHASES
else:
    from .firmware_nav import make_navigator
    from .geometry import obstacle_distance, read_rangers, rot2d, sat_norm, target_names, \
        validate_waypoints, wrap_to_pi
    from .missions import MISSIONS
    from .params import default_params
    from .wedgebug import MODES, PHASES


def dynamics_step(x, psi, u_body, yaw_rate, wind, p):
    """Kinematic point: velocity lag tau, speed/acceleration/yaw-rate limits."""
    v_cmd = sat_norm(rot2d(psi) @ u_body, p.v_max)
    vel = x[2:4]
    acc = sat_norm((v_cmd - vel) / max(p.tau, p.dt), p.a_max)
    vel = sat_norm(vel + acc * p.dt, p.v_max)
    pos = x[0:2] + (vel + wind) * p.dt
    psi = float(wrap_to_pi(psi + min(max(yaw_rate, -p.yaw_rate_max), p.yaw_rate_max) * p.dt))
    return np.concatenate([pos, vel]), psi


def _touches(pos, edges, r):
    A = edges[:, 0:2]
    AB = edges[:, 2:4] - A
    AP = pos - A
    s = np.clip(np.sum(AP * AB, axis=1) / np.sum(AB ** 2, axis=1), 0, 1)
    return np.min(np.linalg.norm(AP - s[:, None] * AB, axis=1)) < r


def run_simulation(start, waypoints, goal, env, p, backend="python"):
    """Fly start -> waypoints -> goal; returns a log dict (one row per tick)."""
    waypoints = np.zeros((0, 2)) if waypoints is None or len(waypoints) == 0 else np.asarray(waypoints, float)
    start = np.asarray(start, dtype=float)
    targets = np.vstack([waypoints, np.asarray(goal, dtype=float)[None, :]])
    validate_waypoints(start, targets, env, p.nav.d_wall)
    rng = np.random.default_rng(p.seed)

    n = int(math.floor(p.t_max / p.dt + 1e-9)) + 1
    t = np.arange(n) * p.dt
    cols = {"x": np.zeros((n, 4)), "psi": np.zeros(n), "ranges": np.zeros((n, 4)), "mode": np.zeros(n, int),
            "phase": np.zeros(n, int), "wp": np.zeros(n, int), "dir": np.zeros(n, int),
            "u_body": np.zeros((n, 2)), "yaw_rate": np.zeros(n)}

    x = np.array([start[0], start[1], 0.0, 0.0])
    psi = p.psi0
    nav = make_navigator(backend, start, targets, p)
    gust = np.zeros(2)
    tau_g = 2.0
    next_read = 0.0
    ranges = np.full(4, np.inf)
    arrived = collided = False
    t_arrive = t_collide = math.nan
    for k in range(n):
        if t[k] >= next_read - 1e-9:
            ranges = read_rangers(x[0:2], psi, env, p, rng)
            next_read += p.ranger.period
        u_body, yaw_rate = nav.step(x, psi, ranges, t[k])
        wind = np.asarray(p.wind, dtype=float) + gust
        cols["x"][k], cols["psi"][k], cols["ranges"][k] = x, psi, ranges
        cols["mode"][k] = MODES.index(nav.mode) + 1
        cols["phase"][k] = PHASES.index(nav.phase) + 1 if nav.phase else 0
        cols["wp"][k], cols["dir"][k] = nav.wp, nav.exec_dir
        cols["u_body"][k], cols["yaw_rate"][k] = u_body, yaw_rate

        if p.stop_on_collision and _touches(x[0:2], env.edges, p.radius):
            collided, t_collide = True, t[k]
            break
        if nav.mode == "DONE" and np.linalg.norm(x[2:4]) < p.goal_vtol:
            arrived = len(targets) - 1 not in nav.unreachable
            t_arrive = t[k]
            break
        if k == n - 1:
            break
        x, psi = dynamics_step(x, psi, u_body, yaw_rate, wind, p)
        if p.wind_gust > 0:   # Ornstein-Uhlenbeck gusts with standard deviation p.wind_gust
            gust = gust - gust * (p.dt / tau_g) + p.wind_gust * math.sqrt(2 * p.dt / tau_g) * rng.standard_normal(2)

    log = {key: value[:k + 1] for key, value in cols.items()}
    log.update(t=t[:k + 1], start=start, targets=targets, target_names=target_names(len(targets)),
               events=nav.events, sweeps=nav.sweeps, unreachable=list(nav.unreachable),
               arrived=arrived, t_arrive=t_arrive, collided=collided, t_collide=t_collide, dt=p.dt)
    return log


def compute_metrics(log, env, p):
    pos = log["x"][:, 0:2]
    gap = obstacle_distance(env, pos) - p.radius
    route = np.vstack([log["start"], log["targets"]])
    types = [e["type"] for e in log["events"]]
    path_length = float(np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1)))
    direct = float(np.sum(np.linalg.norm(np.diff(route, axis=0), axis=1)))
    below = gap < p.near_miss
    return {
        "arrived": log["arrived"], "t_finish": log["t_arrive"],
        "collided": bool(log["collided"] or np.any(gap < 0)),
        "targets_reached": types.count("reached"), "n_targets": len(log["targets"]),
        "unreachable": ", ".join(log["target_names"][i] for i in log["unreachable"]),
        "path_length": path_length, "direct_length": direct, "path_ratio": path_length / direct,
        "avoid_episodes": types.count("hit"), "sweeps": len(log["sweeps"]),
        "avoid_time": float(np.count_nonzero(log["mode"] == 2) * log["dt"]),
        "min_clearance": float(gap.min()),
        "near_misses": int(np.count_nonzero(np.diff(np.concatenate([[False], below]).astype(int)) == 1)),
        "max_speed": float(np.max(np.linalg.norm(log["x"][:, 2:4], axis=1))),
    }


def print_metrics(m):
    status = "arrived" if m["arrived"] else ("COLLIDED" if m["collided"] else "did not arrive")
    print(f"Result: {status}" + (f" at t = {m['t_finish']:.2f} s" if m["arrived"] else ""))
    print(f"Targets reached: {m['targets_reached']} / {m['n_targets']}"
          + (f"; unreachable: {m['unreachable']}" if m["unreachable"] else ""))
    print(f"Path: {m['path_length']:.2f} m flown, {m['direct_length']:.2f} m direct (ratio {m['path_ratio']:.2f})")
    print(f"Avoidance: {m['avoid_episodes']} episodes, {m['sweeps']} sweeps, {m['avoid_time']:.1f} s in AVOID")
    print(f"Clearance: min {m['min_clearance']:.3f} m, {m['near_misses']} near misses; max speed {m['max_speed']:.2f} m/s")


def plot_log(log, env, path, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 6.6))
    for obs in env.obstacles:
        ax.fill(*obs["vertices"].T, color="0.6")
    ax.plot([env.xlim[0], env.xlim[1], env.xlim[1], env.xlim[0], env.xlim[0]],
            [env.ylim[0], env.ylim[0], env.ylim[1], env.ylim[1], env.ylim[0]], color="0.2")
    pos = log["x"][:, 0:2]
    colors = {1: "tab:blue", 2: "tab:orange", 3: "tab:green"}
    for mode, color in colors.items():
        seg = np.where(log["mode"] == mode, pos[:, 0], np.nan)
        ax.plot(seg, pos[:, 1], color=color, lw=2, label=MODES[mode - 1])
    ax.plot(*log["start"], "ko", label="start")
    ax.plot(*log["targets"][:-1].T, "ks", mfc="none", label="waypoints")
    ax.plot(*log["targets"][-1], "k*", ms=12, label="goal")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.legend(loc="upper left", fontsize=8)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mission", choices=sorted(MISSIONS), default="matlab_main")
    parser.add_argument("--backend", choices=("python", "firmware"), default="python")
    parser.add_argument("--ranger-noise", type=float, default=0.0, help="Gaussian ranger noise std dev [m]")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--plot", type=Path, help="Save a top-down plot (needs matplotlib)")
    args = parser.parse_args()

    mission = MISSIONS[args.mission]
    p = default_params()
    p.ranger.noise, p.seed = args.ranger_noise, args.seed
    env = mission.environment()
    log = run_simulation(mission.start, mission.waypoints, mission.goal, env, p, backend=args.backend)
    print(f"Mission {mission.name} ({args.backend} navigator, ranger noise {args.ranger_noise} m)")
    print_metrics(compute_metrics(log, env, p))
    if args.plot:
        plot_log(log, env, args.plot, f"{mission.name}, {args.backend} navigator (2-D)")
        print(f"Plot: {args.plot}")


if __name__ == "__main__":
    main()
