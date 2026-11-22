"""Stock recovery tails and quantitative matched gate metrics, from saved CSVs."""

import argparse
import json
from pathlib import Path

import numpy as np

from analyze_flow_faults import write_csv, truth_bool
from plot_ekf import load


def bursts(mask, times):
    padded = np.r_[False, mask, False].astype(int)
    starts, ends = np.flatnonzero(np.diff(padded) == 1), np.flatnonzero(np.diff(padded) == -1)
    return [{"count": int(b-a), "start_s": float(times[a]), "end_s": float(times[b-1]),
             "span_s": float(times[b-1]-times[a]+.01)} for a, b in zip(starts, ends)]


def tails(cases, output):
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for path in cases:
        meta = json.loads(path.with_suffix(".json").read_text())
        end = meta["settings"]["flow_fault"]["end_s"]
        contact = meta.get("contacts", {}).get("first_contact_s")
        d = load(path.with_suffix(".flow_updates.csv"))
        for threshold in (6.63, 9.):
            for axis in "xy":
                for scope in ("all_post_fault", "before_first_contact"):
                    keep = d["sample_time_s"] >= end
                    if scope == "before_first_contact" and contact is not None:
                        keep &= d["sample_time_s"] < contact
                    times, nis = d["sample_time_s"][keep], d[f"nis_{axis}"][keep]
                    above = nis > threshold
                    runs = bursts(above, times)
                    first_good = np.flatnonzero(~above)
                    leading = int(first_good[0]) if len(first_good) else len(times)
                    longest = max(runs, key=lambda r: r["count"]) if runs else None
                    rows.append(dict(case=path.stem, threshold=threshold, axis=axis, scope=scope,
                                     post_samples=len(times), leading_high_samples=leading,
                                     first_below_threshold_delay_s=float(times[first_good[0]]-end) if len(first_good) else None,
                                     max_high_run_samples=longest["count"] if longest else 0,
                                     max_high_run_span_s=longest["span_s"] if longest else 0.,
                                     longest_start_s=longest["start_s"] if longest else None,
                                     first_contact_s=contact, all_runs=runs))
    (output/"stock_recovery_tails.json").write_text(json.dumps(rows, indent=2))
    write_csv(output/"stock_recovery_tails.csv", [{k:v for k,v in r.items() if k != "all_runs"} for r in rows])
    return rows


def recovery_time(ok, times, end, hold=1.):
    """First start of a continuous hold window; None means right-censored."""
    eligible = (times >= end) & ok
    for run in bursts(eligible, times):
        # Flight data is 1 kHz; burst's nominal 10 ms span is not used here.
        if run["end_s"]-run["start_s"] >= hold-1e-9:
            return run["start_s"]-end
    return None


def metrics(clean_path, cases, output, start=26., end=30., est_margin=.05, track_margin=.1, hold=1.):
    output.mkdir(parents=True, exist_ok=True)
    clean = load(clean_path)
    clean_meta = json.loads(clean_path.with_suffix(".json").read_text())
    if clean_meta["settings"].get("flow_fault", {}).get("kind", "off") != "off":
        raise ValueError("Recovery reference must have faults off")
    clean_t = np.rint(clean["time_s"]*1000).astype(int)
    clean_est = np.sqrt(sum((clean[f"est_{a}_m"]-clean[f"{a}_m"])**2 for a in "xy"))
    clean_track = np.sqrt(sum((clean[f"target_{a}_m"]-clean[f"{a}_m"])**2 for a in "xy"))
    reports = []
    for path in cases:
        meta = json.loads(path.with_suffix(".json").read_text())
        for key in ("mode", "trajectory"):
            if meta.get(key) != clean_meta.get(key):
                raise ValueError(f"Unmatched clean reference: {key}")
        for key in ("noise", "seed"):
            if meta["settings"][key] != clean_meta["settings"][key]:
                raise ValueError(f"Unmatched clean reference: sensor {key}")
        if meta.get("scene", {}).get("scene", "arena") != clean_meta.get("scene", {}).get("scene", "arena"):
            raise ValueError("Unmatched clean scene")
        gate, clean_gate = meta.get("flow_gate") or {}, clean_meta.get("flow_gate") or {}
        if gate.get("enabled",0) != clean_gate.get("enabled",0):
            raise ValueError("Recovery must use the same gate mode's faults-off reference")
        if gate.get("enabled",0) and any(gate.get(k) != clean_gate.get(k) for k in ("threshold","max_reject")):
            raise ValueError("Unmatched clean gate parameters")
        d = load(path)
        t = d["time_s"]
        indices = np.searchsorted(clean_t, np.rint(t*1000).astype(int))
        if np.any(indices >= len(clean_t)) or not np.array_equal(clean_t[indices], np.rint(t*1000).astype(int)):
            raise ValueError("Matched clean trajectory lacks timestamps")
        err = np.sqrt(sum((d[f"est_{a}_m"]-d[f"{a}_m"])**2 for a in "xy"))
        track = np.sqrt(sum((d[f"target_{a}_m"]-d[f"{a}_m"])**2 for a in "xy"))
        contact = meta.get("contacts", {}).get("first_contact_s")
        fault_config = meta["settings"].get("flow_fault", {})
        case_start, case_end = ((fault_config["start_s"],fault_config["end_s"])
                               if fault_config.get("kind","off") != "off" else (start,end))
        r = dict(case=path.stem, clean_reference=str(clean_path), duration_s=float(t[-1]),
                 stable=meta["stable"], first_contact_s=contact,
                 sensor_seed=meta["settings"]["seed"],
                 fault_seed=meta["settings"].get("flow_fault", {}).get("resolved_seed"),
                 flow_gate=meta.get("flow_gate"), fault_window=[case_start,case_end])
        for phase, mask in (("full", np.ones(len(t),dtype=bool)),
                            ("hover", d["phase"]=="hover"), ("move", d["phase"]=="move"),
                            ("during", (t>=case_start)&(t<case_end)), ("post_5s", (t>=case_end)&(t<case_end+5)),
                            ("post_all", t>=case_end)):
            for kind, values in (("estimate", err), ("tracking", track)):
                a = values[mask]
                r[f"{phase}_{kind}_rms_m"] = float(np.sqrt(np.mean(a*a))) if len(a) else None
                r[f"{phase}_{kind}_p95_m"] = float(np.percentile(a,95)) if len(a) else None
                r[f"{phase}_{kind}_max_m"] = float(np.max(a)) if len(a) else None
        valid = np.isfinite(err) & np.isfinite(track)
        if contact is not None:
            valid &= t < contact
        est_ok = valid & (err <= clean_est[indices]+est_margin)
        track_ok = valid & (track <= clean_track[indices]+track_margin)
        fault = meta["settings"].get("flow_fault", {}).get("kind", "off")
        for kind, ok in (("estimate", est_ok), ("tracking", track_ok), ("joint", est_ok & track_ok)):
            r[f"recovery_{kind}_s"] = recovery_time(ok,t,case_end,hold) if fault != "off" else None
        r["recovery_definition"] = dict(estimate_margin_m=est_margin, tracking_margin_m=track_margin,
                                         continuous_hold_s=hold, censored_at_contact=True,
                                         note="relative to matched clean error at each timestamp; None is censored or no fault")
        gate_path = path.with_suffix(".flow_gate.csv")
        if gate_path.exists():
            gates = load(gate_path)
            for axis in "xy":
                g = gates[gates["axis"]==axis]
                for scope, mask in (("all", np.ones(len(g),dtype=bool)),
                                    ("fault_active", truth_bool(g["fault_active"]))):
                    selected = g[mask]
                    for decision in ("accepted", "rejected", "forced"):
                        n = int(np.sum(selected["decision"]==decision))
                        r[f"{axis}_{scope}_{decision}"] = n
                        r[f"{axis}_{scope}_{decision}_rate"] = n/len(selected) if len(selected) else None
            r["false_rejection_rate_defined"] = fault == "off"
        reports.append(r)
    (output/"metrics.json").write_text(json.dumps(reports,indent=2))
    flat = [{k:v for k,v in r.items() if not isinstance(v,dict)} for r in reports]
    keys = list(dict.fromkeys(k for r in flat for k in r))
    write_csv(output/"metrics.csv", [{k:r.get(k) for k in keys} for r in flat])
    return reports


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest="command",required=True)
    tail=sub.add_parser("tails")
    tail.add_argument("cases",type=Path,nargs="+")
    tail.add_argument("--output-dir",type=Path,required=True)
    score=sub.add_parser("metrics")
    score.add_argument("clean_reference",type=Path)
    score.add_argument("cases",type=Path,nargs="+")
    score.add_argument("--output-dir",type=Path,required=True)
    args=parser.parse_args()
    result=(tails(args.cases,args.output_dir) if args.command=="tails" else
            metrics(args.clean_reference,args.cases,args.output_dir))
    print(json.dumps(result,indent=2))


if __name__=="__main__":
    main()
