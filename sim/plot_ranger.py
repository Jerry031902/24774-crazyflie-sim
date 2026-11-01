"""Plot a *.ranger.csv using Isaac Sim's Python (matplotlib is bundled)."""

import argparse
import csv
from pathlib import Path

import numpy as np


def plot_ranger(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path = Path(path)
    with path.open(newline="") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise ValueError("No Ranger measurements; run for at least 0.1 simulation seconds")
    fig, axes = plt.subplots(5, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
    for axis, name in zip(axes, ("front", "back", "left", "right", "up")):
        subset = [r for r in rows if r["direction"] == name]
        time = np.array([float(r["sample_time_s"]) for r in subset])
        ideal = np.array([float(r["ideal_distance_m"]) for r in subset])
        measured = np.array([float(r["distance_m"]) for r in subset])
        valid = np.array([r["valid"] == "1" for r in subset])
        if np.any(np.isfinite(ideal)):
            axis.plot(time, ideal, label="Geometric ray distance", lw=1.5)
        if np.any(valid):
            axis.plot(time, measured, ".", label="Measurement", markersize=2)
        if np.any(~valid):
            axis.plot(time[~valid], np.full(np.sum(~valid), .06), "x", color="tab:red",
                      transform=axis.get_xaxis_transform(), label="Invalid / no return", markersize=3)
        axis.set_ylabel(f"{name}\n[m]")
        if not np.any(np.isfinite(ideal)):
            axis.set_ylim(0, 1)
            axis.set_yticks([])
            axis.set_ylabel(f"{name}\n[no return]")
            axis.text(.5, .5, "No surface within range", transform=axis.transAxes,
                      ha="center", va="center", color="tab:red")
        axis.grid(alpha=.25)
        axis.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("Simulation sample time [s]")
    fig.suptitle("Multi-ranger: five body-fixed rays (no obstacle avoidance)")
    output = path.with_suffix(".png")
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    print(plot_ranger(parser.parse_args().csv))
