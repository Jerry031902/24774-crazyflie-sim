"""Plot a firmware flight CSV. Run with /isaac-sim/python.sh sim/plot_flight.py CSV."""

import argparse
from pathlib import Path


def plot_flight(csv_path, output_path=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    csv_path = Path(csv_path)
    output_path = Path(output_path) if output_path else csv_path.with_suffix(".png")
    data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    t = data["time_s"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True, constrained_layout=True)
    colors = ["#1670b7", "#db8730", "#279463", "#986ab6"]
    for axis, color in zip("xyz", colors):
        axes[0].plot(t, data[f"{axis}_m"], color=color, label=axis.upper())
        axes[0].plot(t, data[f"target_{axis}_m"], "--", color=color, alpha=.65, linewidth=1)
    axes[0].set(ylabel="Position [m]", title="Crazyflie C firmware PID | PhysX ground truth | 47-17 propellers")
    for name, label, color in zip(("roll_deg", "pitch_legacy_deg", "yaw_deg"), ("Roll", "Pitch (legacy)", "Yaw"), colors):
        axes[1].plot(t, data[name], color=color, label=label)
    axes[1].set_ylabel("Attitude [deg]")
    for i, color in enumerate(colors, 1):
        axes[2].plot(t, data[f"m{i}_command"], color=color, label=f"M{i}", linewidth=.8)
        axes[3].plot(t, data[f"m{i}_force_N"], color=color, label=f"M{i}", linewidth=.8)
    axes[2].set_ylabel("Motor command [0..65535]")
    axes[3].set(ylabel="Motor force [N]", xlabel="Simulation time [s]")
    moving = np.flatnonzero(data["target_x_m"] > .25)
    if len(moving):
        move_t = t[moving[0]]
        for ax in axes:
            ax.axvline(move_t, color="0.3", linestyle=":", linewidth=1)
        axes[0].annotate("Move to X=0.5 m", xy=(move_t, .5), xytext=(move_t+.3, .7),
                         arrowprops={"arrowstyle": "->", "color": "0.3"}, fontsize=9)
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(loc="upper right", ncol=4, fontsize=8)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    print(plot_flight(args.csv, args.output))
