#!/usr/bin/env python3
"""Plot a WedgeBug hardware run recorded by wedgebug_mission.py.

    python real/plot_wedgebug.py real/data/<run folder>      -> <run folder>/wedgebug.png

Left: the path in the mission frame (Flow-deck estimate, not ground truth),
coloured by navigator mode, with start, targets, fence and the mission's
obstacles. Right: app state and navigator mode, the four Multi-ranger
readings, and the commanded body velocity and yaw rate. Needs matplotlib.
"""

import csv
import json
import math
from pathlib import Path
import sys

STATES = ("IDLE", "PREFLIGHT", "TAKEOFF", "SETTLE", "NAVIGATE", "ARRIVE", "HOLD", "LAND")
MODE_COLORS = {0: "0.6", 1: "tab:blue", 2: "tab:red", 3: "tab:green"}  # -, GO_TO_GOAL, AVOID, DONE


def read_csv(path):
    with open(path, newline="") as file:
        rows = list(csv.reader(file))
    header, data = rows[0], rows[1:]
    return {name: [float(r[i]) for r in data] for i, name in enumerate(header)}


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    run = Path(argv[1])
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon, Rectangle
    except ImportError:
        print("Needs matplotlib: python -m pip install matplotlib", file=sys.stderr)
        return 1
    mission = json.loads((run / "meta.json").read_text())["mission"]
    s, c = read_csv(run / "wb_state.csv"), read_csv(run / "wb_cmd.csv")
    t0 = s["timestamp_ms"][0]
    ts = [(v - t0) / 1000 for v in s["timestamp_ms"]]
    tc = [(v - t0) / 1000 for v in c["timestamp_ms"]]

    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(1, 2, 1)
    flying = [i for i, st in enumerate(s["wedgebug.state"]) if st >= 2]
    for a, b in zip(flying, flying[1:]):
        if b == a + 1:
            ax.plot(s["wedgebug.x"][a:b + 1], s["wedgebug.y"][a:b + 1],
                    color=MODE_COLORS.get(int(s["wedgebug.mode"][b]), "k"), lw=1.5)
    x0, x1, y0, y1 = mission["fence"]
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ls="--", color="0.5", label="fence"))
    for o in mission.get("obstacles", []):
        if o["kind"] == "box":
            (cx, cy), (w, h) = o["center"], o["size"]
            a = math.radians(o.get("angle_deg", 0.0))
            corners = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
            pts = [(cx + u * math.cos(a) - v * math.sin(a), cy + u * math.sin(a) + v * math.cos(a)) for u, v in corners]
            ax.add_patch(Polygon(pts, color="0.75"))
        elif o["kind"] == "cylinder":
            ax.add_patch(Circle(o["center"], o["radius"], color="0.75"))
    ax.plot(*mission["start"], "ks", label="start")
    for i, (x, y) in enumerate(mission["targets"]):
        ax.plot(x, y, "k*" if i == len(mission["targets"]) - 1 else "ko", ms=10)
        ax.annotate(str(i), (x, y), textcoords="offset points", xytext=(6, 6))
    for mode, name in ((1, "GO_TO_GOAL"), (2, "AVOID"), (3, "DONE")):
        ax.plot([], [], color=MODE_COLORS[mode], label=name)
    ax.set(aspect="equal", xlabel="x [m]", ylabel="y [m]", title=f"{mission['name']}: estimated path")
    ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.3)

    a1 = fig.add_subplot(3, 2, 2)
    a1.step(ts, s["wedgebug.state"], where="post", label="app state")
    a1.step(ts, s["wedgebug.mode"], where="post", label="nav mode")
    a1.step(ts, s["wedgebug.phase"], where="post", label="nav phase", alpha=0.6)
    a1.set_yticks(range(len(STATES)))
    a1.set_yticklabels([f"{i} {n}" for i, n in enumerate(STATES)], fontsize=7)
    a1.legend(fontsize=7, loc="upper left")
    a1.grid(alpha=0.3)

    a2 = fig.add_subplot(3, 2, 4, sharex=a1)
    for d in ("front", "left", "back", "right"):
        a2.plot(tc, [math.nan if v >= 32767 else v / 1000 for v in c[f"range.{d}"]], label=d)
    for level, name in ((1.0, "dSweep"), (0.3, "dWall")):
        a2.axhline(level, ls=":", color="0.4")
        a2.annotate(name, (tc[0], level), fontsize=7, color="0.4")
    a2.set(ylabel="range [m]", ylim=(0, 4.2))
    a2.legend(fontsize=7, loc="upper left", ncol=4)
    a2.grid(alpha=0.3)

    a3 = fig.add_subplot(3, 2, 6, sharex=a1)
    a3.plot(tc, c["wedgebug.cmdVx"], label="forward [m/s]")
    a3.plot(tc, c["wedgebug.cmdVy"], label="left [m/s]")
    a3.plot(tc, c["wedgebug.cmdYawRate"], label="yaw rate [rad/s]")
    a3.set(xlabel="time [s]")
    a3.legend(fontsize=7, loc="upper left")
    a3.grid(alpha=0.3)

    fig.tight_layout()
    out = run / "wedgebug.png"
    fig.savefig(out, dpi=120)
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
