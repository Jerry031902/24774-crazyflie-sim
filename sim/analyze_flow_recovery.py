"""Fixed A/B analysis for stock, v1 and v2; preserves Step 4 metric definitions."""
import argparse
import json
from pathlib import Path
import numpy as np

from analyze_flow_campaign import (CASES, FLIGHT_COLUMNS, SAMPLE_COLUMNS, RECOVERY_METRICS,
    compare, errors, load, metric_specs, observed_window, summary, write_csv)
from analyze_flow_faults import truth_bool
from analyze_flow_gate import recovery_time

MODES = {"stock": 0, "v1": 1, "v2": 2}


def audit_axis(gates, mode):
    """Independently reconstruct every decision and counter from recorded NIS."""
    epoch = None
    for r in gates:
        if epoch != int(r["reset_epoch"]):
            epoch = int(r["reset_epoch"])
            totals = dict(accepted=0, rejected=0, forced=0)
            streak = good = entries = samples = 0
            state = "armed"
        assert r["gate_enabled"] == mode and r["threshold"] == 9 and r["max_reject"] == 10
        assert np.isfinite(r["nis"]) and r["hphr_pixels2"] > 0
        before, used, run, good_run = state, state, 0, 0
        old_streak = streak
        if mode == 2 and state == "recovery":
            decision = "accepted"
            samples += 1
            good_run = good + 1 if r["nis"] <= 9 else 0
            good = good_run
            if good >= 10:
                state, good = "armed", 0
        elif mode and r["nis"] > 9:
            run = streak + 1
            decision = "forced" if run >= 10 else "rejected"
            if mode == 2 and decision == "forced":
                state = used = "recovery"
                entries += 1
                samples += 1
                good = 0
        else:
            decision = "accepted"
        streak = run if decision == "rejected" else 0
        totals[decision] += 1
        assert r["decision"] == decision and r["streak_before"] == old_streak
        assert r["run_at_decision"] == run and r["consecutive_rejections"] == streak
        for k, v in totals.items():
            assert r[k + "_total"] == v
        if mode == 2:
            assert r["recover_k"] == 10
            assert (r["state_before"], r["state_at_update"], r["state_after"]) == (before, used, state)
            assert (r["recovery_good_run"], r["recovery_good_streak"]) == (good_run, good)
            assert (r["recovery_entries_total"], r["recovery_samples_total"]) == (entries, samples)


def score(entry, clean_entry, cache):
    path = Path(entry["path"])
    meta = json.loads(path.with_suffix(".json").read_text())
    clean_path = Path(clean_entry["path"])
    cm = json.loads(clean_path.with_suffix(".json").read_text())
    def flight(p):
        key = str(p)
        if key not in cache:
            cache[key] = load(p, FLIGHT_COLUMNS)
        return cache[key]
    d, cd = flight(path), flight(clean_path)
    t = d["time_s"]
    est, track = errors(d)
    assert np.isfinite(est).all() and np.isfinite(track).all()
    assert np.array_equal(np.rint(t*1000).astype(int), np.arange(len(t)))
    assert meta["flow_gate"] == cm["flow_gate"]
    assert meta["flow_gate"]["enabled"] == MODES[entry["mode"]]
    for k in ("noise", "seed", "gyro_noise_std_deg_s", "accel_noise_std_G", "flow_noise_std_counts", "tof_noise_std_m"):
        assert meta["settings"][k] == cm["settings"][k]
    assert meta["mass_kg"] == cm["mass_kg"] == .033
    assert meta["trajectory"] == cm["trajectory"] == "flow-shuttle-x"
    assert meta["scene"] == cm["scene"]
    assert meta["scene"]["scene"] == "flow-empty" and meta["scene"]["position_bound_m"] == 20
    prefix = min(len(d), len(cd), 26000)
    assert np.array_equal(d[:prefix], cd[:prefix]), (entry, "pre-fault prefix")
    contact = meta["contacts"]["first_contact_s"]
    r = dict(dataset=entry["dataset"], case=entry["case"], seed=entry["seed"], mode=entry["mode"], path=str(path),
             duration_s=float(t[-1]), complete_run=bool(t[-1]>=62), stable=meta["stable"],
             contact_events=meta["contacts"]["events"], first_contact_s=contact,
             ekf_resets=int(d["ekf_reset_count"].max()), termination_error=meta.get("error"),
             out_of_bounds=meta.get("error")=="AssertionError('Flight left the test arena')",
             failed_checks=", ".join(k for k,v in meta["stability_checks"].items() if not v),
             full_estimate_max_m=float(est.max()), full_tracking_max_m=float(track.max()))
    for name, start, end in (("during",26.,30.), ("post_5s",30.,35.)):
        for kind, values in (("estimate",est), ("tracking",track)):
            w = observed_window(values,t,start,end)
            r[f"{name}_{kind}_rms_m"] = w["rms"]
            r[f"{name}_{kind}_observed_rms_m"] = w["observed_rms"]
            r[f"{name}_coverage"], r[f"{name}_complete"] = w["coverage"], w["complete"]
    n = min(len(d),len(cd))
    ce, ct = errors(cd[:n]); times = t[:n]
    valid = np.ones(n,dtype=bool)
    for hit in (contact, cm["contacts"]["first_contact_s"]):
        if hit is not None: valid &= times < hit
    end = float(times[valid][-1]) if valid.any() else 0.
    r.update(clean_reference=str(clean_path), recovery_observation_end_s=end,
             recovery_censor_lower_bound_s=max(0.,end-31.))
    eok, tok = valid&(est[:n]<=ce+.05), valid&(track[:n]<=ct+.10)
    for kind, ok in (("estimate",eok), ("tracking",tok), ("joint",eok&tok)):
        v = recovery_time(ok,times,30.,1.) if entry["case"] != "off" else None
        r[f"recovery_{kind}_s"] = v
        r[f"recovery_{kind}_status"] = "not_applicable" if entry["case"]=="off" else "recovered" if v is not None else "censored"
    sensors = load(path.with_suffix(".flow_samples.csv"),SAMPLE_COLUMNS)
    gates = load(path.with_suffix(".flow_gate.csv"))
    ids = sensors["flow_sample_id"].astype(int)
    active, enqueued = truth_bool(sensors["fault_active"]), truth_bool(sensors["enqueued"])
    labels = {int(s["flow_sample_id"]):s for s in sensors}
    r.update(raw_fault_active_samples=int(active.sum()), driver_dropped_fault_active_samples=int((active&~enqueued).sum()),
             driver_dropped_clean_samples=int((~active&~enqueued).sum()))
    if entry["case"]=="dropout": assert not np.any(active&enqueued)
    for axis in ("x","y","pooled"):
        g = gates if axis=="pooled" else gates[gates["axis"]==axis]
        if axis != "pooled":
            assert np.array_equal(g["flow_sample_id"].astype(int),ids[enqueued])
            audit_axis(g,MODES[entry["mode"]])
            for row in g:
                sample = labels[int(row["flow_sample_id"])]
                assert str(sample["fault_active"])==str(row["fault_active"])
                assert sample["fault_type"]==row["fault_type"]
        ga, gt = truth_bool(g["fault_active"]), g["sample_time_s"]
        scopes = dict(fault_active=ga, clean=~ga, during_clean=(~ga)&(gt>=26)&(gt<30),
                      post_5s_clean=(~ga)&(gt>=30)&(gt<35), post_all_clean=(~ga)&(gt>=30), all=np.ones(len(g),bool))
        for scope, keep in scopes.items():
            sub = g[keep]; key = f"{axis}_{scope}"
            r[key+"_attempted"] = len(sub)
            for dec in ("accepted","rejected","forced"):
                r[key+"_"+dec] = int((sub["decision"]==dec).sum())
            r[key+"_rejection_rate"] = r[key+"_rejected"]/len(sub) if len(sub) else None
        entry_mask = (g["state_before"]=="armed")&(g["state_at_update"]=="recovery") if entry["mode"]=="v2" else np.zeros(len(g),bool)
        sample_mask = g["state_at_update"]=="recovery" if entry["mode"]=="v2" else np.zeros(len(g),bool)
        r[f"{axis}_gate_recovery_entries"] = int(entry_mask.sum())
        r[f"{axis}_gate_recovery_samples"] = int(sample_mask.sum())
        r[f"{axis}_post_gate_recovery_entries"] = int((entry_mask&(gt>=30)).sum())
        r[f"{axis}_active_gate_recovery_entries"] = int((entry_mask&ga).sum())
    if entry["case"] != "off": cache.pop(str(path),None)
    return r,sensors


def specs():
    result = metric_specs()
    for a in ("x","y","pooled"):
        for k in ("entries","samples"):
            result[f"{a}_gate_recovery_{k}"] = "lower_frequency"
    return result


def aggregate(rows, seeds, comparisons):
    lookup = {(r["case"],r["seed"],r["mode"]):r for r in rows}
    pairs, stats = [], []
    for left,right in comparisons:
        name = f"{right}_minus_{left}"
        for case in CASES:
            for metric,direction in specs().items():
                group = []
                for seed in seeds:
                    a,z = lookup[(case,seed,left)],lookup[(case,seed,right)]
                    c = compare(metric,a,z,direction)
                    group.append(dict(comparison=name,case=case,seed=seed,metric=metric,direction=direction,
                                      reference_mode=left,candidate_mode=right,reference=c.pop("stock"),candidate=c.pop("gated"),**c))
                pairs.extend(group)
                ss = {"reference":summary([p["reference"] for p in group]),
                      "candidate":summary([p["candidate"] for p in group]),
                      "delta":summary([p["delta"] for p in group if p["comparable"]]),
                      "observed_delta":summary([p["delta"] for p in group])}
                r = dict(comparison=name,case=case,metric=metric,direction=direction,reference_mode=left,candidate_mode=right,
                         **{f"{prefix}_{k}":v for prefix,s in ss.items() for k,v in s.items()})
                for outcome in ("better","worse","tie","indeterminate"):
                    r[outcome] = sum(p["outcome"]==outcome for p in group)
                r["reference_complete_runs"] = sum(lookup[(case,s,left)]["complete_run"] for s in seeds)
                r["candidate_complete_runs"] = sum(lookup[(case,s,right)]["complete_run"] for s in seeds)
                r["mean_worse_proven_from_lower_bound"] = bool(metric.startswith("full_") and
                    r["reference_complete_runs"]==len(seeds) and r["candidate_complete_runs"]<len(seeds)
                    and r["observed_delta_n"]==len(seeds) and r["observed_delta_mean"]>1e-10)
                stats.append(r)
    return pairs,stats


def analyze(root,dataset):
    plan = json.loads((root/"pre_results_plan.json").read_text())
    seeds = plan["development_seeds" if dataset=="A" else "validation_seeds"]
    entries = json.loads((root/dataset/"runs.json").read_text())
    assert len(entries)==(40 if dataset=="A" else 80),"Incomplete dataset"
    if dataset=="A":
        old = json.loads(Path("build/step4_threshold9/runs.json").read_text())
        entries += [dict(e,dataset="A",mode="v1" if e["mode"]=="gated" else "stock") for e in old]
    clean = {(e["seed"],e["mode"]):e for e in entries if e["case"]=="off"}
    rows,cache,samples = [],{},{}
    for e in entries:
        r,s = score(e,clean[(e["seed"],e["mode"])],cache)
        rows.append(r);samples[(e["case"],e["seed"],e["mode"])]=s
    comparisons = [("stock","v1"),("stock","v2"),("v1","v2")] if dataset=="A" else [("stock","v2")]
    checks = []
    for left,right in comparisons:
        for case in CASES:
            for seed in seeds:
                a,z = samples[(case,seed,left)],samples[(case,seed,right)]
                n = min(len(a),len(z))
                for field in ("flow_sample_id","sample_time_s","fault_active","noise_x_counts","noise_y_counts"):
                    assert np.array_equal(a[field][:n],z[field][:n]),(dataset,case,seed,left,right,field)
                checks.append(dict(case=case,seed=seed,comparison=f"{right}_minus_{left}",matched_reads=n))
    paired,stats = aggregate(rows,seeds,comparisons)
    out = root/dataset/"analysis";out.mkdir(exist_ok=True)
    worse = [p for p in paired if p["comparison"]=="v2_minus_stock" and p["outcome"]=="worse"]
    worse_mean = [s for s in stats if s["comparison"]=="v2_minus_stock" and
                  (s["delta_mean"] is not None and (s["delta_mean"] if s["direction"]!="higher" else -s["delta_mean"])>1e-10
                   or s["mean_worse_proven_from_lower_bound"])]
    for name,data in (("per_seed",rows),("paired",paired),("summary",stats),("v2_worse_than_stock",worse),
                      ("v2_worse_mean",worse_mean),("pairing_checks",checks)):
        (out/f"{name}.json").write_text(json.dumps(data,indent=2,allow_nan=False));write_csv(out/f"{name}.csv",data)
    print(f"Dataset {dataset}: scored {len(rows)} runs; {len(checks)} paired input checks passed",flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root",type=Path)
    parser.add_argument("dataset",choices=("A","B"))
    args=parser.parse_args();analyze(args.root,args.dataset)
