"""Paired multi-seed flow-gate metrics, with explicit censoring and denominators."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from analyze_flow_gate import recovery_time
from analyze_flow_faults import truth_bool

SEEDS=(7,17,27,37,47)
CASES=("off","spike5","spike10","spike20","spike50","zero","dropout","scale05")
ERROR_METRICS=("during_estimate_rms_m","during_tracking_rms_m","post_5s_estimate_rms_m",
               "post_5s_tracking_rms_m","full_estimate_max_m","full_tracking_max_m")
RECOVERY_METRICS=tuple(f"recovery_{kind}_s" for kind in ("estimate","tracking","joint"))
FLIGHT_COLUMNS=("time_s","x_m","y_m","est_x_m","est_y_m","target_x_m","target_y_m","ekf_reset_count")
SAMPLE_COLUMNS=("flow_sample_id","sample_time_s","fault_active","enqueued","fault_type",
                "noise_x_counts","noise_y_counts")


def load(path, columns=None):
    return np.atleast_1d(np.genfromtxt(path,delimiter=",",names=True,dtype=None,encoding="utf-8",usecols=columns))


def write_csv(path, rows):
    if not rows:return
    keys=list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w",newline="") as stream:
        out=csv.DictWriter(stream,fieldnames=keys);out.writeheader();out.writerows(rows)


def summary(values):
    a=np.array([x for x in values if x is not None and math.isfinite(x)],dtype=float)
    return dict(n=len(a),mean=float(a.mean()) if len(a) else None,
                std=float(a.std(ddof=1)) if len(a)>1 else None)


def errors(data):
    est=np.hypot(data["est_x_m"]-data["x_m"],data["est_y_m"]-data["y_m"])
    track=np.hypot(data["target_x_m"]-data["x_m"],data["target_y_m"]-data["y_m"])
    return est,track


def observed_window(values, times, start, end):
    """Partial-window RMS is retained, but not reported as full-window RMS."""
    ms=np.rint(times*1000).astype(np.int64)
    keep=(ms>=round(start*1000))&(ms<round(end*1000))
    selected=values[keep]
    complete=np.array_equal(ms[keep],np.arange(round(start*1000),round(end*1000)))
    finite=bool(np.isfinite(selected).all())
    rms=float(np.sqrt(np.mean(selected*selected))) if len(selected) and finite else None
    return dict(rms=rms if complete else None,observed_rms=rms,
                coverage=len(selected)/round((end-start)*1000),complete=bool(complete and finite))


def per_run(entry, clean, cache):
    path=Path(entry["path"]);meta=json.loads(path.with_suffix(".json").read_text())
    data=cache.setdefault(str(path),None)
    if data is None:data=load(path,FLIGHT_COLUMNS);cache[str(path)]=data
    times=data["time_s"];err,track=errors(data)
    assert np.isfinite(err).all() and np.isfinite(track).all(),path
    assert np.array_equal(np.rint(times*1000).astype(int),np.arange(len(times))),path
    contact=meta["contacts"]["first_contact_s"]
    r=dict(case=entry["case"],seed=entry["seed"],mode=entry["mode"],path=str(path),
           duration_s=float(times[-1]),complete_run=bool(times[-1]>=62),stable=meta["stable"],
           first_contact_s=contact,contact_events=meta["contacts"]["events"],
           ekf_resets=int(np.max(data["ekf_reset_count"])),termination_error=meta.get("error"),
           failed_checks=", ".join(k for k,v in meta["stability_checks"].items() if not v),
           full_estimate_max_m=float(err.max()),full_tracking_max_m=float(track.max()))
    for phase,start,end in (("during",26.,30.),("post_5s",30.,35.)):
        for name,a in (("estimate",err),("tracking",track)):
            w=observed_window(a,times,start,end)
            r[f"{phase}_{name}_rms_m"]=w["rms"]
            r[f"{phase}_{name}_observed_rms_m"]=w["observed_rms"]
            r[f"{phase}_coverage"]=w["coverage"]
            r[f"{phase}_complete"]=w["complete"]
    clean_path=Path(clean["path"])
    cm=json.loads(clean_path.with_suffix(".json").read_text())
    cd=cache.setdefault(str(clean_path),None)
    if cd is None:cd=load(clean_path,FLIGHT_COLUMNS);cache[str(clean_path)]=cd
    assert meta["flow_gate"]==cm["flow_gate"],"Recovery must use the same gate configuration"
    for key in ("seed","noise"):
        assert meta["settings"][key]==cm["settings"][key]
    for key in ("accel_noise_std_G","gyro_noise_std_deg_s","accel_filtered_std_G",
                "gyro_filtered_std_deg_s","flow_noise_std_counts","tof_noise_std_m",
                "delays_ms","imu_hz","flow_nominal_hz","tof_nominal_hz"):
        assert meta["settings"][key]==cm["settings"][key],(path,key)
    assert meta["mass_kg"]==cm["mass_kg"]==.033
    assert meta["scene"]["scene"]==cm["scene"]["scene"]=="flow-empty"
    assert meta["trajectory"]==cm["trajectory"]=="flow-shuttle-x"
    # Physics/controller prefixes must match each mode's independent clean run.
    prefix=min(len(data),len(cd),26000)
    assert np.array_equal(data[:prefix],cd[:prefix]),f"Unmatched pre-fault prefix: {path}"
    shared=min(len(data),len(cd));ce,ct=errors(cd[:shared])
    st=times[:shared]
    valid=np.ones(shared,dtype=bool)
    for hit in (contact,cm["contacts"]["first_contact_s"]):
        if hit is not None:valid &= st<hit
    last_valid=float(st[valid][-1]) if valid.any() else 0.
    r["recovery_observation_end_s"]=last_valid
    r["recovery_censor_lower_bound_s"]=max(0.,last_valid-30.-1.)
    r["clean_reference"]=str(clean_path)
    est_ok=valid&(err[:shared]<=ce+.05)
    track_ok=valid&(track[:shared]<=ct+.1)
    for name,ok in (("estimate",est_ok),("tracking",track_ok),("joint",est_ok&track_ok)):
        v=recovery_time(ok,st,30.,1.) if entry["case"]!="off" else None
        r[f"recovery_{name}_s"]=v
        r[f"recovery_{name}_status"]=("not_applicable" if entry["case"]=="off" else
                                      "recovered" if v is not None else "censored")
    samples=load(path.with_suffix(".flow_samples.csv"),SAMPLE_COLUMNS)
    gates=load(path.with_suffix(".flow_gate.csv"))
    sample_ids=samples["flow_sample_id"].astype(int)
    enqueued=truth_bool(samples["enqueued"]);active=truth_bool(samples["fault_active"])
    lookup={int(row["flow_sample_id"]):row for row in samples}
    r.update(raw_fault_active_samples=int(active.sum()),
             driver_dropped_fault_active_samples=int(np.sum(active&~enqueued)),
             driver_dropped_clean_samples=int(np.sum(~active&~enqueued)))
    if entry["case"]=="dropout":assert not np.any(active&enqueued),path
    for axis in ("x","y","pooled"):
        g=gates if axis=="pooled" else gates[gates["axis"]==axis]
        if axis!="pooled":
            assert np.array_equal(g["flow_sample_id"].astype(int),sample_ids[enqueued]),path
            epoch=None;totals=dict(accepted=0,rejected=0,forced=0);streak=0
            for row in g:
                if epoch!=int(row["reset_epoch"]):
                    epoch=int(row["reset_epoch"]);totals=dict(accepted=0,rejected=0,forced=0);streak=0
                assert row["threshold"]==9 and row["max_reject"]==10
                assert row["gate_enabled"]==int(entry["mode"]=="gated")
                high=bool(row["gate_enabled"] and row["nis"]>9)
                run=streak+1 if high else 0
                decision=("forced" if run>=10 else "rejected") if high else "accepted"
                assert row["decision"]==decision and row["streak_before"]==streak
                assert row["run_at_decision"]==run
                streak=run if decision=="rejected" else 0
                assert row["consecutive_rejections"]==streak
                totals[decision]+=1
                for k,v in totals.items():assert row[k+"_total"]==v
                sample=lookup[int(row["flow_sample_id"])]
                assert str(sample["fault_active"])==str(row["fault_active"])
                assert sample["fault_type"]==row["fault_type"]
        ga=truth_bool(g["fault_active"]);gt=g["sample_time_s"]
        scopes={"fault_active":ga,"clean":~ga,
                "during_clean":(~ga)&(gt>=26)&(gt<30),
                "post_5s_clean":(~ga)&(gt>=30)&(gt<35),
                "post_all_clean":(~ga)&(gt>=30),"all":np.ones(len(g),dtype=bool)}
        for scope,mask in scopes.items():
            subset=g[mask];prefix=f"{axis}_{scope}"
            r[prefix+"_attempted"]=len(subset)
            for decision in ("accepted","rejected","forced"):
                r[prefix+"_"+decision]=int(np.sum(subset["decision"]==decision))
            r[prefix+"_rejection_rate"]=r[prefix+"_rejected"]/len(subset) if len(subset) else None
    # Keep only clean traces cached across runs; each full array is otherwise released.
    if entry["case"]!="off":cache.pop(str(path),None)
    return r,samples


def metric_specs():
    specs={m:"lower" for m in ERROR_METRICS+RECOVERY_METRICS}
    for axis in ("x","y","pooled"):
        for scope in ("fault_active","clean","post_5s_clean","post_all_clean"):
            specs[f"{axis}_{scope}_rejection_rate"]="higher" if scope=="fault_active" else "lower"
        for scope in ("all","post_5s_clean","post_all_clean"):
            specs[f"{axis}_{scope}_forced"]="lower_frequency"
    return specs


def compare(metric,a,z,direction):
    av,zv=a.get(metric),z.get(metric)
    delta=zv-av if av is not None and zv is not None else None
    comparable=delta is not None
    if metric.startswith("full_") and not (a["complete_run"] and z["complete_run"]):comparable=False
    result="indeterminate"
    if comparable:
        sign=delta if direction!="higher" else -delta
        result="better" if sign < -1e-10 else "worse" if sign > 1e-10 else "tie"
    elif metric.startswith("full_") and delta is not None:
        # A truncated maximum is a lower bound. It cannot win by being small,
        # but exceeding a complete counterpart already proves it is worse.
        if a["complete_run"] and zv>av+1e-10:result="worse"
        if z["complete_run"] and av>zv+1e-10:result="better"
    elif metric in RECOVERY_METRICS:
        # A confirmed recovery can beat a censored counterpart only when it is
        # earlier than the counterpart's last fully evaluable hold-window start.
        if av is None and zv is not None and zv<a["recovery_censor_lower_bound_s"]-1e-9:result="better"
        if zv is None and av is not None and av<z["recovery_censor_lower_bound_s"]-1e-9:result="worse"
    return dict(stock=av,gated=zv,delta=delta,comparable=comparable,outcome=result)


def aggregate(rows):
    lookup={(r["case"],r["seed"],r["mode"]):r for r in rows}
    paired=[];summaries=[]
    for case in CASES:
        for metric,direction in metric_specs().items():
            pairs=[]
            for seed in SEEDS:
                if (case,seed,"stock") not in lookup or (case,seed,"gated") not in lookup:continue
                a,z=lookup[(case,seed,"stock")],lookup[(case,seed,"gated")]
                comp=compare(metric,a,z,direction)
                pairs.append(dict(case=case,seed=seed,metric=metric,direction=direction,**comp))
            if not pairs:continue
            paired+=pairs
            s=summary([p["stock"] for p in pairs]);g=summary([p["gated"] for p in pairs])
            d=summary([p["delta"] for p in pairs if p["comparable"]])
            observed=summary([p["delta"] for p in pairs])
            r=dict(case=case,metric=metric,direction=direction,pairs=len(pairs),
                   **{f"stock_{k}":v for k,v in s.items()},**{f"gated_{k}":v for k,v in g.items()},
                   **{f"delta_{k}":v for k,v in d.items()},
                   **{f"observed_delta_{k}":v for k,v in observed.items()},
                   better=sum(p["outcome"]=="better" for p in pairs),
                   worse=sum(p["outcome"]=="worse" for p in pairs),
                   ties=sum(p["outcome"]=="tie" for p in pairs),
                   indeterminate=sum(p["outcome"]=="indeterminate" for p in pairs))
            for mode in ("stock","gated"):
                r[f"{mode}_complete_runs"]=sum(
                    lookup[(case,p["seed"],mode)]["complete_run"] for p in pairs)
            r["mean_worse_proven_from_lower_bound"]=bool(
                metric.startswith("full_") and r["stock_complete_runs"]==len(pairs)
                and r["gated_complete_runs"]<len(pairs) and observed["n"]==len(pairs)
                and observed["mean"]>1e-10)
            # Descriptive 95% paired t interval for five complete pairs only.
            half=2.7764451051977987*d["std"]/math.sqrt(5) if d["n"]==5 else None
            r["delta_t95_low"]=d["mean"]-half if half is not None else None
            r["delta_t95_high"]=d["mean"]+half if half is not None else None
            summaries.append(r)
    return paired,summaries


def analyze(root, partial=False):
    index=json.loads((root/"runs.json").read_text())
    if not partial:assert len(index)==80,"Refuse a final report from an incomplete matrix"
    clean={(r["seed"],r["mode"]):r for r in index if r["case"]=="off"}
    rows=[];sample_cache={};cache={}
    for entry in index:
        if (entry["seed"],entry["mode"]) not in clean:continue
        row,samples=per_run(entry,clean[(entry["seed"],entry["mode"])],cache)
        rows.append(row);sample_cache[(row["case"],row["seed"],row["mode"])]=samples
    checks=[]
    for case in CASES:
        for seed in SEEDS:
            keys=[(case,seed,m) for m in ("stock","gated")]
            if not all(k in sample_cache for k in keys):continue
            a,z=(sample_cache[k] for k in keys);n=min(len(a),len(z))
            for field in ("flow_sample_id","sample_time_s","fault_active","noise_x_counts","noise_y_counts"):
                assert np.array_equal(a[field][:n],z[field][:n]),(case,seed,field)
            checks.append(dict(case=case,seed=seed,matched_flow_reads=n,same_noise_and_fault_schedule=True))
    amplitude_checks=[]
    for seed in SEEDS:
        for mode in ("stock","gated"):
            base=sample_cache.get(("spike5",seed,mode))
            if base is None:continue
            for case in ("spike10","spike20","spike50"):
                other=sample_cache.get((case,seed,mode))
                if other is None:continue
                n=min(len(base),len(other))
                for field in ("flow_sample_id","sample_time_s","fault_active","noise_x_counts","noise_y_counts"):
                    assert np.array_equal(base[field][:n],other[field][:n]),("amplitude pairing",seed,mode,case,field)
                amplitude_checks.append(dict(seed=seed,mode=mode,reference="spike5",case=case,
                                             matched_flow_reads=n,same_noise_and_fault_schedule=True))
    paired,stats=aggregate(rows)
    out=root/("analysis_partial" if partial else "analysis");out.mkdir(exist_ok=True)
    for name,data in (("per_seed",rows),("paired",paired),("summary",stats),
                      ("worse_per_seed",[r for r in paired if r["outcome"]=="worse"]),
                      ("worse_mean",[r for r in stats if r["delta_mean"] is not None and
                       (r["delta_mean"] if r["direction"]!="higher" else -r["delta_mean"])>1e-10
                       or r["mean_worse_proven_from_lower_bound"]])):
        (out/f"{name}.json").write_text(json.dumps(data,indent=2,allow_nan=False))
        write_csv(out/f"{name}.csv",data)
    for case in CASES:write_csv(out/f"per_seed_{case}.csv",[r for r in rows if r["case"]==case])
    (out/"pairing_checks.json").write_text(json.dumps(checks,indent=2))
    (out/"amplitude_pairing_checks.json").write_text(json.dumps(amplitude_checks,indent=2))
    print(f"Scored {len(rows)}/80 runs, {len(checks)}/40 matched pairs; no threshold 6.63 runs",flush=True)
    return rows,paired,stats


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("campaign",type=Path)
    p.add_argument("--partial",action="store_true")
    args=p.parse_args();analyze(args.campaign,args.partial)
