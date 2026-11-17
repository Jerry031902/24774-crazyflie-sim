"""Write exhaustive tables and static figures from a completed threshold-9 matrix."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_flow_campaign import CASES, SEEDS, ERROR_METRICS, FLIGHT_COLUMNS, errors, load, write_csv

CASE_LABELS = {"off": "off", "spike5": "spike 5", "spike10": "spike 10",
               "spike20": "spike 20", "spike50": "spike 50", "zero": "zero",
               "dropout": "dropout", "scale05": "scale 0.5"}
LABELS = {
    "during_estimate_rms_m": "During: XY estimate RMS",
    "during_tracking_rms_m": "During: XY tracking RMS",
    "post_5s_estimate_rms_m": "Post 5 s: XY estimate RMS",
    "post_5s_tracking_rms_m": "Post 5 s: XY tracking RMS",
    "full_estimate_max_m": "Full run: XY estimate max",
    "full_tracking_max_m": "Full run: XY tracking max",
}


def number(value, metric, signed=False):
    if value is None:
        return "NA"
    scale = 100 if metric.endswith("_rejection_rate") else 1
    return format(value * scale, "+.5g" if signed else ".5g")


def mean_sd(row, prefix, metric):
    value = number(row[prefix + "_mean"], metric)
    sd = number(row[prefix + "_std"], metric)
    result = f"{value} ± {sd} (n={row[prefix + '_n']})"
    if metric.startswith("full_") and prefix in ("stock", "gated"):
        result += f"; complete={row[prefix + '_complete_runs']}/5"
    return result


def unit(metric):
    if metric.endswith("_rejection_rate"):
        return "% / Δ percentage points"
    if metric.endswith("_m"):
        return "m"
    if metric.endswith("_s"):
        return "s"
    return "scalar updates"


def markdown_table(header, rows):
    return ["| " + " | ".join(header) + " |",
            "| " + " | ".join("---" for _ in header) + " |"] + [
                "| " + " | ".join(str(x) for x in row) + " |" for row in rows]


def make_tables(out, rows, pairs, stats):
    intro = [
        "# Threshold 9.0: 完整 metrics", "",
        "固定 maxReject=10；seeds=7,17,27,37,47；noise seed=fault seed。",
        "During 为 [26,30) s，post 为 [30,35) s；faults-off 使用同一时间段。",
        "所有 ± 为 seed 间 sample SD（ddof=1）。Δ=gated−stock。error 越小越好。",
        "recovery 使用同 mode、同 seed 的 faults-off reference：estimate error ≤ reference+0.05 m，"
        "tracking error ≤ reference+0.10 m，连续 1 s；分别报告两个条件以及 joint。",
        "recovery 的 NA 是未观察到 recovery 或 off 不适用；mean/SD 只含已观察到的 recovery，n 明列。",
        "fault-active rejection 的分母是实际到达 EKF 的 scalar updates；dropout 没有 update，rate 为 NA。",
        "clean rejection 包含所有 fault-inactive updates；post_5s_clean 和 post_all_clean 单列。",
        "rate 仅基于实际观察到的 updates；截短 run 的 post_5s_clean 不代表观察满 5 s。"
        "各分母、duration 和 window coverage 都保留在 per_seed.csv。",
        "forced accept 不算 rejected；forced count 的增加是发生次数增加，不单独证明 flight 变差。",
        "fault-active rejection 越高表示检测到更多 fault，但不单独证明 flight 更好。",
        "full-run max 在未跑满 62 s 时只是 observed lower bound，不能用较小的 partial max 宣称改善。",
        "Observed Δ 栏保留截短 run 的 max difference；其 SD 描述已观察到的 differences，不是完整 62 s difference 的 SD。",
        "对于 censored recovery 或 partial max，better/worse 仅在已有观察足以确定方向时计入；其余为 indeterminate。",
        "", "[逐个 run 的全部数据](per_seed.csv) · [每个 paired difference](paired.csv) · "
        "[全部变差的 seed/metric](worse_per_seed.csv) · [mean 变差的 metrics](worse_mean.csv)", "",
    ]
    content = list(intro)
    for case in CASES:
        content += [f"## {CASE_LABELS[case]}", "", f"[该 case 的 per-seed CSV](per_seed_{case}.csv)", ""]
        table = []
        for r in stats:
            if r["case"] != case:
                continue
            m = r["metric"]
            table.append([m, unit(m), mean_sd(r, "stock", m), mean_sd(r, "gated", m),
                          mean_sd(r, "delta", m),
                          mean_sd(r, "observed_delta", m) if m.startswith("full_") and r["delta_n"]<5 else "same",
                          f"{r['better']} / {r['worse']} / {r['ties']} / {r['indeterminate']}"])
        content += markdown_table(["Metric", "Unit", "Stock mean ± SD", "Gated mean ± SD",
                                   "Comparable paired Δ mean ± SD", "Observed Δ mean ± SD (partial max)",
                                   "Better / worse / tie / indeterminate (of 5)"], table)
        content += ["", "### 每个 seed", ""]
        table = []
        for seed in SEEDS:
            for p in pairs:
                if p["case"] == case and p["seed"] == seed:
                    m = p["metric"]
                    table.append([seed, m, number(p["stock"], m), number(p["gated"], m),
                                  number(p["delta"], m, True), p["comparable"], p["outcome"]])
        content += markdown_table(["Seed", "Metric", "Stock", "Gated", "Δ", "Comparable Δ", "Outcome"], table)
        content += ["", "### Run 状态", ""]
        table = [[r["seed"], r["mode"], r["duration_s"], r["stable"], r["contact_events"],
                  r["ekf_resets"], r["recovery_joint_status"], r["failed_checks"], r["termination_error"]]
                 for r in rows if r["case"] == case]
        content += markdown_table(["Seed", "Mode", "Duration s", "Flight checks pass", "Contacts",
                                   "EKF resets", "Joint recovery", "Failed checks", "Termination error"], table)
        content += [""]
    (out / "TABLES.md").write_text("\n".join(content) + "\n")


def make_regressions(out, rows, pairs, stats):
    lines = ["# Gated 变差的完整清单", "",
             "以下逐项列出所有能确定变差的 seed/metric。Δ=gated−stock；error 单位 m，"
             "recovery 单位 s，rate 的 Δ 单位 percentage points。Forced count 增加只表示发生次数增加。",
             "mean/SD 和完整对照见 [TABLES.md](TABLES.md)；机器可读数据见 [worse_per_seed.csv](worse_per_seed.csv)。",
             ""]
    for case in CASES:
        lines += [f"## {CASE_LABELS[case]}", ""]
        table = []
        for r in stats:
            if r["case"] != case:
                continue
            selected = [p for p in pairs if p["case"] == case and p["metric"] == r["metric"]
                        and p["outcome"] == "worse"]
            if not selected:
                continue
            details = []
            for p in selected:
                value = number(p["delta"], p["metric"], True)
                if not p["comparable"]:
                    value = ("gated censored" if p["delta"] is None else "observed Δ " + value)
                details.append(f"{p['seed']} ({value})")
            table.append([r["metric"], unit(r["metric"]), ", ".join(details)])
        lines += markdown_table(["Metric", "Unit", "变差的 seeds (Δ)"], table) + [""]
    lines += ["## 截短 run 的 observed max", "",
              "这些是已观察到的 max differences；没有覆盖 62 s 的 run 不代表完整 62 s 的表现。"
              "indeterminate 的两侧都已截短，不能据此断言完整 horizon 的差异。", ""]
    partial = [p for p in pairs if p["metric"].startswith("full_") and not p["comparable"]]
    lines += markdown_table(["Case", "Seed", "Metric", "Observed Δ m", "Complete-horizon outcome"],
                            [[p["case"], p["seed"], p["metric"], number(p["delta"], p["metric"], True),
                              p["outcome"]] for p in partial]) + [""]
    lookup = {(r["case"], r["seed"], r["mode"]): r for r in rows}
    checks = []
    for case in CASES:
        for seed in SEEDS:
            a, z = [json.loads(Path(lookup[(case, seed, mode)]["path"]).with_suffix(".json").read_text())
                    for mode in ("stock", "gated")]
            for name, passed in a["stability_checks"].items():
                if passed and not z["stability_checks"][name]:
                    checks.append(dict(case=case, seed=seed, check=name, stock=True, gated=False))
    (out / "flight_check_regressions.json").write_text(json.dumps(checks, indent=2))
    write_csv(out / "flight_check_regressions.csv", checks)
    lines += ["## 原有 flight checks 的 regression", ""]
    lines += markdown_table(["Case", "Seed", "Check", "Stock", "Gated"],
                            [[r["case"], r["seed"], r["check"], "pass", "fail"] for r in checks])
    (out / "REGRESSIONS.md").write_text("\n".join(lines) + "\n")


def plot_paired(out, pairs, stats, cases, filename):
    fig, axes = plt.subplots(3, 2, figsize=(12, 11), constrained_layout=True)
    colors = plt.get_cmap("tab10")(np.arange(5))
    lookup = {(p["case"], p["seed"], p["metric"]): p for p in pairs}
    slookup = {(r["case"], r["metric"]): r for r in stats}
    for ax, metric in zip(axes.flat, ERROR_METRICS):
        ax.axhline(0, color="0.35", lw=1)
        for i, case in enumerate(cases):
            for j, seed in enumerate(SEEDS):
                p = lookup[(case, seed, metric)]
                if p["comparable"]:
                    ax.scatter(i + (j - 2) * .045, p["delta"] * 100,
                               color=colors[j], s=27, label=f"seed {seed}" if i == 0 else None)
                elif p["delta"] is not None:
                    ax.scatter(i + (j - 2) * .045, p["delta"] * 100, facecolors="none",
                               edgecolors=colors[j], marker="s", s=40)
            r = slookup[(case, metric)]
            if r["delta_mean"] is not None:
                ax.errorbar(i, r["delta_mean"] * 100,
                            yerr=(r["delta_std"] or 0) * 100, color="black", marker="D",
                            ms=4, capsize=4, lw=1.1, zorder=5)
            elif metric.startswith("post_5s") and r["delta_n"] == 0:
                ax.text(i, .94, "Incomplete\nwindow", transform=ax.get_xaxis_transform(),
                        ha="center", va="top", fontsize=9, color="0.35")
        ax.set_xticks(range(len(cases)), [CASE_LABELS[c] for c in cases])
        ax.set_title(LABELS[metric])
        ax.set_ylabel("Gated - stock (cm)")
        if "zero" in cases and metric.startswith("full_"):
            ax.set_yscale("symlog", linthresh=5)
            ax.set_ylabel("Gated - stock (cm; symlog, linear +/-5)")
        ax.grid(axis="y", alpha=.2)
    axes.flat[0].legend(fontsize=8, ncol=3)
    fig.suptitle("Threshold 9, maxReject 10 | dots: paired seeds; black: mean +/- sample SD\n"
                 "Negative = lower error; hollow squares = incomplete-run observed delta", fontsize=11)
    fig.savefig(out / filename, dpi=180)
    plt.close(fig)


def plot_clean(out, rows):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    colors = plt.get_cmap("tab10")(np.arange(5))
    lookup = {(r["seed"], r["mode"]): r for r in rows if r["case"] == "off"}
    for ax, metric in zip(axes, ERROR_METRICS[-2:]):
        for i, seed in enumerate(SEEDS):
            values = [lookup[(seed, mode)][metric] * 100 for mode in ("stock", "gated")]
            ax.plot([0, 1], values, "o-", color=colors[i], label=f"seed {seed}")
        ax.set_xticks([0, 1], ["Stock", "Gated (9.0)"])
        ax.set_title(LABELS[metric])
        ax.set_ylabel("cm")
        ax.grid(axis="y", alpha=.2)
    axes[0].legend(fontsize=8)
    fig.suptitle("Faults off: same-seed pairs; all runs 62 s")
    fig.savefig(out / "clean_pairs.png", dpi=180)
    plt.close(fig)


def plot_zero_detail(out, rows):
    """Show the recorded return of clean flow and the ensuing gate decisions."""
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
    for mode, color in (("stock", "tab:blue"), ("gated", "tab:orange")):
        r = next(r for r in rows if r["case"] == "zero" and r["seed"] == 7 and r["mode"] == mode)
        path = Path(r["path"])
        d = load(path, FLIGHT_COLUMNS)
        est, _ = errors(d)
        axes[0].plot(d["time_s"], est, color=color, label=mode)
        gates = load(path.with_suffix(".flow_gate.csv"))
        x = gates[gates["axis"] == "x"]
        axes[1].plot(x["sample_time_s"], x["nis"], color=color, lw=.9, label=mode)
        if mode == "gated":
            for j, (decision, marker) in enumerate((("accepted", "."), ("rejected", "x"), ("forced", "D"))):
                selected = x[x["decision"] == decision]
                axes[2].scatter(selected["sample_time_s"], np.full(len(selected), j), s=15,
                                marker=marker, label=decision)
            for ax in axes:
                ax.axvline(r["duration_s"], color="red", ls=":", lw=1)
    for ax in axes:
        ax.axvspan(26, 30, color="0.5", alpha=.15)
        ax.axvline(30, color="0.35", ls="--", lw=1)
        ax.set_xlim(25, 35)
        ax.grid(alpha=.2)
    axes[0].set_ylabel("XY estimate error (m)")
    axes[0].legend()
    axes[1].set_yscale("symlog", linthresh=1)
    axes[1].set_ylim(bottom=0)
    axes[1].axhline(9, color="black", ls="--", label="threshold 9")
    axes[1].set_ylabel("X NIS (symlog)")
    axes[1].legend()
    axes[2].set_yticks([0, 1, 2], ["accepted", "rejected", "forced"])
    axes[2].set_ylabel("Gated X decision")
    axes[2].set_xlabel("Time (s)")
    fig.suptitle("Zero fault, seed 7 | shaded: zero counts enqueued; red: gated run reaches position bound\n"
                 "Normal flow resumes at 30 s; missing data after termination is not extrapolated", fontsize=11)
    fig.savefig(out / "zero_seed7_reacquisition.png", dpi=180)
    plt.close(fig)


def main(root):
    out = root / "analysis"
    rows, pairs, stats = [json.loads((out / f"{name}.json").read_text())
                          for name in ("per_seed", "paired", "summary")]
    assert len(rows) == 80 and len({(r["case"], r["seed"], r["mode"]) for r in rows}) == 80
    make_tables(out, rows, pairs, stats)
    make_regressions(out, rows, pairs, stats)
    plot_paired(out, pairs, stats, CASES[:5], "paired_spikes.png")
    plot_paired(out, pairs, stats, CASES[5:], "paired_zero_dropout_scale.png")
    plot_clean(out, rows)
    plot_zero_detail(out, rows)
    criterion = []
    for case in CASES[1:5]:
        endpoints = [r for r in stats if r["case"] == case and r["metric"] in ERROR_METRICS[:4]]
        criterion.append(dict(case=case, clear_paired_rms_improvement=all(
            r["delta_n"] == 5 and r["delta_mean"] < 0 and r["better"] >= 4 for r in endpoints),
            endpoints=[{k: r[k] for k in ("metric", "delta_mean", "delta_std", "delta_n", "better", "worse")}
                       for r in endpoints]))
    (out / "spike_improvement_criterion.json").write_text(json.dumps(criterion, indent=2))
    print(f"Wrote exhaustive tables and four figures to {out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", type=Path)
    main(parser.parse_args().campaign)
