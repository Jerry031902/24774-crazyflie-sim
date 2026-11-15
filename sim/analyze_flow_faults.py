"""Read-only detectability estimates and stock fault scoring, never a gate.

Run `detectability` on a completed faults-off shuttle before running fault flights.
All analytic estimates condition on the stock state/covariance; the estimator
and controller will react during a real fault run, so these are not forecasts.
"""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from plot_ekf import load


def write_csv(path, rows):
    with Path(path).open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def truth_bool(values):
    return np.asarray(values).astype(str) == "True"


def joined_updates(path):
    samples = load(path.with_suffix(".flow_samples.csv"))
    updates = load(path.with_suffix(".flow_updates.csv"))
    lookup = {int(i): n for n, i in enumerate(samples["flow_sample_id"])}
    indices = [lookup[int(i)] for i in updates["flow_sample_id"]]
    selected = samples[indices]
    if not truth_bool(selected["enqueued"]).all():
        raise AssertionError("A dropped sensor read incorrectly has an EKF update")
    return samples, selected, updates


def detectability(reference, output, start=26., duration=4.):
    output.mkdir(parents=True, exist_ok=True)
    metadata = json.loads(reference.with_suffix(".json").read_text())
    if metadata["settings"]["flow_fault"]["kind"] != "off":
        raise ValueError("Detectability requires a faults-off stock reference")
    _, samples, updates = joined_updates(reference)
    flight = load(reference)
    # Flight uses step*0.001; sensor timestamps use ms/1000. Join integer
    # milliseconds, not binary-float search order (e.g. 0.35 vs 0.35000000000000003).
    flight_ms = np.rint(flight["time_s"]*1000).astype(np.int64)
    sample_ms = np.rint(samples["sample_time_s"]*1000).astype(np.int64)
    indices = np.searchsorted(flight_ms, sample_ms)
    if np.any(indices >= len(flight_ms)) or not np.array_equal(flight_ms[indices], sample_ms):
        raise AssertionError("Audit/truth sample times do not match")
    # This import only supplies the deterministic trajectory, no simulator app.
    from flight_demo import FlowShuttleDemo
    demo = FlowShuttleDemo(metadata["trajectory"][-1])
    factor = .01*35/.71674/.1
    traces = []
    for sample, update, state in zip(samples, updates, flight[indices]):
        _, v, _ = demo.reference(float(sample["sample_time_s"]))
        row = {"time_s": float(sample["sample_time_s"]), "target_velocity_m_s": v,
               "level_reference_counts": factor*v, "true_vx_m_s": float(state["vx_m_s"]),
               "true_vy_m_s": float(state["vy_m_s"]), "true_z_m": float(state["z_m"])}
        for axis in "xy":
            row[f"ideal_{axis}_counts"] = float(sample[f"ideal_{axis}_counts"])
            row[f"prediction_{axis}_counts"] = float(update[f"prediction_{axis}_counts"])
            row[f"sqrt_hphr_{axis}_counts"] = float(np.sqrt(update[f"hphr_{axis}_pixels2"])/.1)
        traces.append(row)
    write_csv(output/"ideal_flow_vs_time.csv", traces)
    noise_variance = metadata["settings"]["flow_noise_std_counts"]**2 if metadata["settings"]["noise"] else 0.
    rows = []
    for window, lo, hi in (("defaults", 20.2, 21.2), ("campaign", start, start+duration)):
        mask = (samples["sample_time_s"] >= lo) & (samples["sample_time_s"] < hi)
        if not mask.any():
            raise ValueError("No reference measurements in fault window")
        for axis in "xy":
            c = samples[f"ideal_{axis}_counts"][mask]
            p = updates[f"prediction_{axis}_counts"][mask]
            variance = updates[f"hphr_{axis}_pixels2"][mask]/.1**2
            sigma = np.sqrt(variance)
            for fault in ("off", "spike_5", "spike_10", "spike_20", "spike_50", "zero", "dropout", "scale_0.5"):
                base = dict(window=window, start_s=lo, end_s=hi, axis=axis, fault=fault,
                            samples=len(c), ideal_counts_rms=float(np.sqrt(np.mean(c*c))),
                            ideal_counts_abs_max=float(np.max(np.abs(c))),
                            sqrt_hphr_counts_median=float(np.median(sigma)))
                if fault == "dropout":
                    base.update(fault_signal_error_counts_rms=None, fault_signal_error_sigma_rms=None,
                                innovation_counts_rms=None, normalized_innovation_rms=None,
                                expected_nis_mean=None, expected_nis_max=None)
                else:
                    if fault.startswith("spike_"):
                        amp = float(fault.split("_")[1])
                        # Expectation over independent random sign and fresh noise,
                        # conditional on an injected spike; valid absent clipping.
                        if np.max(np.abs(samples[f"clean_{axis}_counts"][mask]))+amp >= 99:
                            raise ValueError("Unclipped analytic spike estimate is invalid here")
                        bias2 = np.full_like(c, amp*amp)
                        expected = ((c-p)**2 + noise_variance + bias2)/variance
                    elif fault == "zero":
                        bias2 = c*c
                        expected = p*p/variance  # Noise overwritten, no +1 term.
                    elif fault == "scale_0.5":
                        bias2 = (.5*c)**2
                        expected = ((.5*c-p)**2 + noise_variance)/variance
                    else:
                        bias2 = np.zeros_like(c)
                        expected = ((c-p)**2 + noise_variance)/variance
                    base.update(fault_signal_error_counts_rms=float(np.sqrt(np.mean(bias2))),
                                fault_signal_error_sigma_rms=float(np.sqrt(np.mean(bias2/variance))),
                                innovation_counts_rms=float(np.sqrt(np.mean(expected*variance))),
                                normalized_innovation_rms=float(np.sqrt(np.mean(expected))),
                                expected_nis_mean=float(np.mean(expected)), expected_nis_max=float(np.max(expected)))
                rows.append(base)
    write_csv(output/"detectability.csv", rows)
    info = {"reference": str(reference), "factor_counts_per_velocity_over_height": factor,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "reference_peak_speed_m_s": .46875, "reference_peak_level_counts": factor*.46875,
            "formula": "S_counts=HPHR_pixels2/0.1^2; spike E[NIS]=((c-p)^2+noise_variance+A^2)/S; scale=((f*c-p)^2+noise_variance)/S; zero=p^2/S",
            "scope": "Stock-conditioned one-sample estimates, not closed-loop forecasts. Spike conditional on firing; both axes independent signs. Zero overwrites noise. Dropout has no NIS. No gate exists.",
            "rows": rows}
    (output/"detectability.json").write_text(json.dumps(info, indent=2))
    lines = ["# Pre-fault detectability", "", info["scope"], "", info["formula"], "",
             "Signal error is the RMS injected bias. Innovation RMS includes retained noise; normalized RMS squared equals mean E[NIS].", "",
             "| Window | Axis | Fault | Signal error counts RMS | Innovation counts RMS | Innovation / sqrt(HPHR) RMS | E[NIS] mean | E[NIS] max |",
             "|---|---|---|---:|---:|---:|---:|---:|"]
    for row in rows:
        values = [row[k] for k in ("fault_signal_error_counts_rms", "innovation_counts_rms", "normalized_innovation_rms", "expected_nis_mean", "expected_nis_max")]
        values = ["N/A" if v is None else f"{v:.5g}" for v in values]
        lines.append(f"| {row['window']} | {row['axis']} | {row['fault']} | " + " | ".join(values) + " |")
    (output/"detectability.md").write_text("\n".join(lines)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
    t = samples["sample_time_s"]
    ax[0].plot(t, [r["level_reference_counts"] for r in traces], label="Level reference 4.883 v/z")
    for axis in "xy":
        ax[0].plot(t, samples[f"ideal_{axis}_counts"], label=f"True ideal flow {axis}", linewidth=.7)
        ax[1].plot(t, np.sqrt(updates[f"hphr_{axis}_pixels2"])/.1, label=f"sqrt(HPHR) {axis}", linewidth=.7)
    for a in ax:
        a.axvspan(start, start+duration, alpha=.15, color="red", label="Proposed fault window")
        a.grid(alpha=.2)
        a.legend()
    ax[0].set(ylabel="Firmware-aligned counts", title=reference.stem)
    ax[1].set(xlabel="Simulation time [s]", ylabel="Counts (log scale)", yscale="log")
    fig.savefig(output/"detectability.png", dpi=150)
    plt.close(fig)
    return info


def summarize(reference, cases, output):
    output.mkdir(parents=True, exist_ok=True)
    clean_samples, _, _ = joined_updates(reference)
    clean_lookup = {int(row["flow_sample_id"]): row for row in clean_samples}
    reports = []
    for path in [reference]+cases:
        metadata = json.loads(path.with_suffix(".json").read_text())
        samples, selected, updates = joined_updates(path)
        flight = load(path)
        active = truth_bool(selected["fault_active"])
        fault_reads = truth_bool(samples["fault_active"])
        report = {"case": path.stem, "stable": metadata["stable"],
                  "duration_s": metadata["duration_s"], "flight_error": metadata.get("error"),
                  "failed_checks": [k for k, v in metadata["stability_checks"].items() if not v],
                  "sensor_reads": len(samples), "active_fault_reads": int(fault_reads.sum()),
                  "active_fault_updates": int(active.sum()), "ekf_flow_updates": len(updates),
                  "driver_drops": int((~truth_bool(samples["enqueued"])).sum()),
                  "clipped_reads": int(truth_bool(samples["clipped"]).sum()),
                  "gate_implemented": False,
                  "noise_samples_identical_to_reference": all(
                      row[f"noise_{axis}_counts"] == clean_lookup[int(row["flow_sample_id"])][f"noise_{axis}_counts"]
                      for row in samples for axis in "xy")}
        xy_error = np.sqrt(sum((flight[f"est_{axis}_m"]-flight[f"{axis}_m"])**2 for axis in "xy"))
        xy_tracking = np.sqrt(sum((flight[f"target_{axis}_m"]-flight[f"{axis}_m"])**2 for axis in "xy"))
        for phase, mask in (("all", flight["time_s"] >= 1), ("move", flight["phase"] == "move"),
                            ("recovery", flight["phase"] == "recovery")):
            report[f"{phase}_xy_estimate_error_max_m"] = float(np.max(xy_error[mask])) if mask.any() else None
            report[f"{phase}_xy_estimate_error_rms_m"] = float(np.sqrt(np.mean(xy_error[mask]**2))) if mask.any() else None
            report[f"{phase}_xy_tracking_error_max_m"] = float(np.max(xy_tracking[mask])) if mask.any() else None
        for axis in "xy":
            for label, mask in (("active", active), ("inactive", ~active)):
                nis = updates[f"nis_{axis}"][mask]
                report[f"{label}_{axis}_nis_nonfinite_count"] = int((~np.isfinite(nis)).sum())
                nis = nis[np.isfinite(nis)]
                report[f"{label}_{axis}_nis_mean"] = float(nis.mean()) if len(nis) else None
                report[f"{label}_{axis}_nis_above_9_fraction"] = float(np.mean(nis > 9)) if len(nis) else None
        reports.append(report)
    (output/"fault_summary.json").write_text(json.dumps(reports, indent=2))
    write_csv(output/"fault_summary.csv", reports)
    return reports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    det = sub.add_parser("detectability")
    det.add_argument("reference", type=Path)
    det.add_argument("--output-dir", type=Path, required=True)
    det.add_argument("--start", type=float, default=26.)
    det.add_argument("--duration", type=float, default=4.)
    score = sub.add_parser("summarize")
    score.add_argument("reference", type=Path)
    score.add_argument("cases", type=Path, nargs="+")
    score.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "detectability":
        print(json.dumps(detectability(args.reference, args.output_dir, args.start, args.duration), indent=2))
    else:
        print(json.dumps(summarize(args.reference, args.cases, args.output_dir), indent=2))


if __name__ == "__main__":
    main()
