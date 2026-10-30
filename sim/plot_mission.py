#!/usr/bin/env python3
"""Plot an Isaac Sim WedgeBug mission CSV (run_sim.py --mission) against the 2-D reference.

    /isaac-sim/python.sh /workspace/sim/plot_mission.py /workspace/build/missions/matlab_main_firmware_pid.csv

Writes a sibling .png: top-down path coloured by navigator mode (with the
2-D kinematic simulator's path for the same mission dashed), height, and the
four ranger readings. Prints the closest approach to any obstacle or wall.
"""

import argparse
import csv
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from navigation.geometry import obstacle_distance  # noqa: E402
from navigation.missions import MISSIONS  # noqa: E402
from navigation.params import default_params  # noqa: E402
from navigation.sim2d import run_simulation  # noqa: E402
from navigation.wedgebug import MODES  # noqa: E402


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows or "nav_mode" not in rows[0]:
        raise SystemExit(f"{path} is not a run_sim.py --mission CSV")
    data = {key: np.array([float(r[key]) if r[key] not in ("", "nan") else np.nan for r in rows])
            for key in rows[0] if key != "phase"}
    data["phase"] = np.array([r["phase"] for r in rows])
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--mission", choices=sorted(MISSIONS), help="Default: from the file name")
    args = parser.parse_args()
    name = args.mission or next((m for m in MISSIONS if args.csv.name.startswith(m)), None)
    if name is None:
        raise SystemExit("Give --mission; it could not be inferred from the file name")
    mission = MISSIONS[name]
    d = load(args.csv)
    env = mission.environment()
    p = default_params()

    xy = np.column_stack([d["x_m"], d["y_m"]])
    nav = d["nav_mode"] > 0
    gap = obstacle_distance(env, xy[nav]) - p.radius
    print(f"Closest approach while navigating: {gap.min():.3f} m (body radius {p.radius} m)")
    print(f"Flight time: {d['time_s'][-1]:.1f} s; final phase: {d['phase'][-1]}")
    reference = run_simulation(mission.start, mission.waypoints, mission.goal, env, p)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(13, 7))
    ax = fig.add_subplot(1, 2, 1)
    for obs in env.obstacles:
        ax.fill(*obs["vertices"].T, color="0.6")
    (x0, x1), (y0, y1) = env.xlim, env.ylim
    ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], color="0.2")
    ax.plot(*reference["x"][:, 0:2].T, "k--", lw=1, label="2-D reference")
    ax.plot(*xy[~nav].T, color="0.5", lw=1, label="takeoff")
    for mode, color in ((1, "tab:blue"), (2, "tab:orange"), (3, "tab:green")):
        ax.plot(np.where(d["nav_mode"] == mode, xy[:, 0], np.nan), xy[:, 1], color=color, lw=2,
                label=f"Isaac {MODES[mode - 1]}")
    targets = np.array(mission.targets)
    ax.plot(*mission.start, "ko")
    ax.plot(*targets[:-1].T, "ks", mfc="none")
    ax.plot(*targets[-1], "k*", ms=12)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{mission.name}: Isaac Sim (true path) vs 2-D reference")
    ax.legend(loc="upper left", fontsize=8)

    t = d["time_s"]
    ax = fig.add_subplot(3, 2, 2)
    ax.plot(t, d["z_m"], label="z")
    ax.plot(t, d["target_z_m"], "k--", lw=1, label="target z")
    ax.set_ylabel("height [m]")
    ax.legend(fontsize=8)
    ax = fig.add_subplot(3, 2, 4, sharex=ax)
    for key in ("front", "left", "back", "right"):
        ax.plot(t, d[f"range_{key}_m"], lw=1, label=key)
    ax.set_ylabel("ranger [m]")
    ax.set_ylim(0, p.ranger.range_max + 0.2)
    ax.legend(fontsize=8, ncol=4)
    ax = fig.add_subplot(3, 2, 6, sharex=ax)
    ax.plot(t, np.hypot(d["vx_m_s"], d["vy_m_s"]), label="speed (true)")
    ax.plot(t, np.hypot(d["cmd_forward_m_s"], d["cmd_left_m_s"]), lw=1, label="commanded")
    ax.set_ylabel("speed [m/s]")
    ax.set_xlabel("time [s]")
    ax.legend(fontsize=8)

    out = args.csv.with_suffix(".png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"Plot: {out}")


if __name__ == "__main__":
    main()
