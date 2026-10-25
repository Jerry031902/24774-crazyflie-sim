"""Plot/score an EKF CSV; all errors are evaluated against held-out PhysX truth."""

import argparse
from pathlib import Path
import numpy as np


def load(csv_path):
    return np.atleast_1d(np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8"))


def analyze(csv_path):
    d = load(csv_path)
    true = np.column_stack([d[f"{a}_m"] for a in "xyz"])
    est = np.column_stack([d[f"est_{a}_m"] for a in "xyz"])
    target = np.column_stack([d[f"target_{a}_m"] for a in "xyz"])
    velocity = np.column_stack([d[f"v{a}_m_s"] for a in "xyz"])
    error = est - true
    hover = d["phase"] == "hover"
    after_startup = d["time_s"] >= 1
    final_window = d["time_s"] >= d["time_s"][-1]-3
    tracking = true - target
    angles = np.column_stack([d["roll_deg"], d["pitch_legacy_deg"]])
    complete = d["time_s"][-1] >= 30
    finite = bool(np.isfinite(true).all() and np.isfinite(est).all() and np.isfinite(d["attitude_error_deg"]).all())
    hover_z = float(np.max(np.abs(tracking[hover, 2]))) if np.any(hover) else float("inf")
    hover_xy = float(np.max(np.linalg.norm(tracking[hover, :2], axis=1))) if np.any(hover) else float("inf")
    final_error = float(np.linalg.norm(np.mean(tracking[final_window], axis=0)))
    criteria = {"completed_30_s": bool(complete), "finite_states": finite,
                "no_EKF_resets": bool(np.max(d["ekf_reset_count"]) == 0),
                "no_ground_contact": bool(np.min(true[:, 2]) > .03),
                "tilt_below_45_deg": bool(np.max(np.abs(angles)) < 45),
                "hover_z_within_20_cm": hover_z < .2,
                "hover_xy_within_50_cm": hover_xy < .5,
                "final_mean_target_error_below_50_cm": final_error < .5,
                "final_mean_speed_below_25_cm_s": bool(np.mean(np.linalg.norm(velocity[final_window], axis=1)) < .25)}
    drift = error[hover][-1, :2] - error[hover][0, :2] if np.any(hover) else [float("nan")]*2
    true_drift = true[hover][-1, :2]-true[hover][0, :2] if np.any(hover) else [float("nan")]*2
    return {"stable": all(criteria.values()), "stability_checks": criteria,
            "duration_s": float(d["time_s"][-1]),
            "max_z_estimation_error_m": float(np.max(np.abs(error[:, 2]))),
            "max_z_estimation_error_after_1s_m": float(np.max(np.abs(error[after_startup, 2]))) if np.any(after_startup) else None,
            "hover_xy_estimation_drift_m": np.asarray(drift).tolist(),
            "hover_xy_true_drift_m": np.asarray(true_drift).tolist(),
            "max_hover_xy_estimation_error_m": float(np.max(np.linalg.norm(error[hover, :2], axis=1))) if np.any(hover) else None,
            "max_attitude_error_deg": float(np.max(d["attitude_error_deg"])),
            "rms_attitude_error_deg": float(np.sqrt(np.mean(d["attitude_error_deg"]**2))),
            "max_hover_z_tracking_error_m": hover_z,
            "max_hover_xy_tracking_error_m": hover_xy,
            "takeoff_peak_z_m": float(np.max(true[d["time_s"] < 10, 2])),
            "final_true_position_m": true[-1].tolist(), "final_estimated_position_m": est[-1].tolist(),
            "final_mean_tracking_error_m": final_error,
            "motor_saturated_steps": int(np.sum(d["saturated"])),
            "ekf_resets": int(np.max(d["ekf_reset_count"]))}


def plot_ekf(csv_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    csv_path = Path(csv_path)
    d = load(csv_path)
    t = d["time_s"]
    fig, axes = plt.subplots(4, 3, figsize=(15, 13), sharex=True, constrained_layout=True)
    for j, axis in enumerate("xyz"):
        axes[0, j].plot(t, d[f"{axis}_m"], label="PhysX true", linewidth=1.2)
        axes[0, j].plot(t, d[f"est_{axis}_m"], label="Stock EKF", linewidth=.8)
        axes[0, j].plot(t, d[f"target_{axis}_m"], "k--", label="Target", alpha=.45, linewidth=.8)
        axes[0, j].set(title=f"{axis.upper()} position", ylabel="m")
        axes[1, j].plot(t, d[f"v{axis}_m_s"], label="PhysX true", linewidth=1.2)
        axes[1, j].plot(t, d[f"est_v{axis}_m_s"], label="Stock EKF", linewidth=.8)
        axes[1, j].set(title=f"{axis.upper()} world velocity", ylabel="m/s")
    for j, angle in enumerate(("roll", "pitch_legacy", "yaw")):
        axes[2, j].plot(t, d[f"{angle}_deg"], label="PhysX true", linewidth=1.2)
        axes[2, j].plot(t, d[f"est_{angle}_deg"], label="Stock EKF", linewidth=.8)
        axes[2, j].set(title=angle.replace("_", " "), ylabel="deg")
    packets = load(csv_path.with_suffix(".measurements.csv"))
    for j, (sensor, component, label) in enumerate((("flow", "x", "Flow X innovation"), ("flow", "y", "Flow Y innovation"), ("tof", "x", "ToF innovation"))):
        selected = packets[packets["sensor"] == sensor]
        axes[3, j].plot(selected["time_s"], selected[f"innovation_{component}"], linewidth=.55, label="measurement - prediction")
        axes[3, j].set(title=label, ylabel="pixels" if sensor == "flow" else "m", xlabel="Simulation time [s]")
    for ax in axes.flat:
        ax.axvspan(10, 20, color="#34a853", alpha=.07)
        ax.axvline(20, color="0.4", linestyle=":", linewidth=.8)
        ax.grid(alpha=.2)
        ax.legend(fontsize=7, loc="best")
    fig.suptitle(f"{csv_path.stem} | stock EKF | shaded: 10 s hover | dotted: move to X=0.5 m")
    output = csv_path.with_suffix(".png")
    fig.savefig(output, dpi=140)
    plt.close(fig)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    import json
    print(json.dumps(analyze(args.csv), indent=2))
    print(plot_ekf(args.csv))
