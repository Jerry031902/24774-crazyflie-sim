"""Resume a fixed, sequential 80-flight threshold-9 campaign inside cf_sim_dev.

No tuning, no automatic threshold sweep. Every run uses fresh SimulationApp
via the existing smoke runner. Flight-check failures are results; infrastructure
errors stop the campaign. Seed-7 visual-update compatibility is checked first.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

SEEDS = (7,17,27,37,47)
CASES = ("off","spike5","spike10","spike20","spike50","zero","dropout","scale05")
STEM = "flow-shuttle-x_ekf_closed_loop_noise_on"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def case_name(case):
    if case=="off":return STEM
    if case.startswith("spike"):return STEM+"_spike_"+case[5:]
    return STEM+"_"+("scale" if case=="scale05" else case)


def command(root, seed, mode, case):
    out=root/f"seed{seed}"/mode/case
    fault="spike" if case.startswith("spike") else "scale" if case=="scale05" else case
    args=[sys.executable,"sim/tests/smoke_firmware_ekf.py","--mode","ekf_closed_loop",
          "--noise","on","--seed",str(seed),"--scene","flow-empty","--trajectory","flow-shuttle-x",
          "--flow-fault",fault,"--flow-fault-seed",str(seed),"--flow-fault-offset","6",
          "--flow-fault-duration","4","--flow-gate",str(int(mode=="gated")),
          "--flow-gate-threshold","9.0","--flow-gate-max-reject","10",
          "--flow-audit","--flow-gate-log","--no-render","--no-plots","--output-dir",str(out)]
    if fault=="spike":args += ["--flow-spike-amplitude",case[5:]]
    if fault=="scale":args += ["--flow-scale-factor","0.5"]
    return args,out/case_name(case)


def verify_meta(path,seed,mode,case,native_hash):
    meta=json.loads(path.read_text())
    assert meta["mode"]=="ekf_closed_loop" and meta["trajectory"]=="flow-shuttle-x"
    assert meta["scene"]["scene"]=="flow-empty" and meta["mass_kg"]==.033
    assert meta["settings"]["noise"] and meta["settings"]["seed"]==seed
    f=meta["settings"]["flow_fault"]
    expected="spike" if case.startswith("spike") else "scale" if case=="scale05" else case
    assert f["kind"]==expected and f["resolved_seed"]==seed and f["start_s"]==26 and f["end_s"]==30
    if expected=="spike":assert f["spike_amplitude"]==float(case[5:]) and f["spike_probability"]==.1
    if expected=="scale":assert f["scale_factor"]==.5
    g=meta["flow_gate"]
    assert (g["enabled"],g["threshold"],g["max_reject"],g["logging"])==(int(mode=="gated"),9.,10,True)
    assert meta["bindings_sha256"]==native_hash and meta["render_every_steps"]==0
    assert "analysis_error" not in meta,meta.get("analysis_error")
    if "error" in meta:assert meta["error"]=="AssertionError('Flight left the test arena')",meta["error"]
    return meta


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output-dir",type=Path,default=Path("build/step4_threshold9"))
    p.add_argument("--source-commit",required=True,help="Recorded on the host; no git operations inside the container")
    args=p.parse_args()
    root=args.output_dir.resolve();root.mkdir(parents=True,exist_ok=True)
    import _cffirmware
    native_hash=digest(Path(_cffirmware.__file__))
    accepted=Path("build/step3_flow_gate_seed7")
    source_paths=json.loads((accepted/"source_files.json").read_text())["sha256"]
    frozen={name:digest(Path(name)) for name in source_paths}
    frozen["sim/tests/run_flow_gate_campaign.py"]=digest(Path(__file__))
    plan=dict(seeds=SEEDS,cases=CASES,modes=["stock","gated"],threshold=9.,max_reject=10,
              fault_window_s=[26.,30.],noise=True,scene="flow-empty",trajectory="flow-shuttle-x",
              native_sha256=native_hash,source_sha256=frozen,
              commit=args.source_commit)
    plan_path=root/"plan.json"
    if plan_path.exists():assert json.loads(plan_path.read_text())==json.loads(json.dumps(plan)),"Campaign source/config changed"
    else:plan_path.write_text(json.dumps(plan,indent=2))
    index_path=root/"runs.json"
    rows=json.loads(index_path.read_text()) if index_path.exists() else []
    indexed={(r["seed"],r["mode"],r["case"]):r for r in rows}
    probes=[(7,"stock","off"),(7,"gated","off"),(7,"gated","spike20")]
    jobs=probes+[(s,m,c) for c in CASES for s in SEEDS for m in ("stock","gated") if (s,m,c) not in probes]
    references={(7,"stock","off"):accepted/"empty_stock_reference"/STEM,
                (7,"gated","off"):accepted/"gate_on_clean"/STEM,
                (7,"gated","spike20"):accepted/"gate_on_spike20"/(STEM+"_spike_20")}
    for seed,mode,case in jobs:
        assert all(digest(Path(name))==value for name,value in frozen.items()),"Frozen source changed"
        assert digest(Path(_cffirmware.__file__))==native_hash,"Bindings changed"
        cmd,base=command(root,seed,mode,case)
        if not base.with_suffix(".json").exists():
            if base.with_suffix(".csv").exists():raise RuntimeError(f"Partial run requires review: {base}")
            base.parent.mkdir(parents=True,exist_ok=True)
            started=datetime.now(timezone.utc).isoformat();tick=time.monotonic()
            print(f"BEGIN {seed} {mode} {case}",flush=True)
            with (base.parent/"run.log").open("w") as log:
                code=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT).returncode
            if code not in (0,1):raise RuntimeError(f"Infrastructure exit {code}: {base}")
            execution=dict(command=cmd,exit_code=code,started_utc=started,wall_seconds=time.monotonic()-tick)
            (base.parent/"execution.json").write_text(json.dumps(execution,indent=2))
        meta=verify_meta(base.with_suffix(".json"),seed,mode,case,native_hash)
        assert base.with_suffix(".flow_gate.csv").exists(),base
        key=(seed,mode,case)
        if key in references:
            ref=references[key];comparisons=[]
            for suffix in (".csv",".measurements.csv",".flow_samples.csv",".flow_updates.csv",".contacts.csv",".flow_gate.csv"):
                a,z=ref.with_suffix(suffix),base.with_suffix(suffix)
                if not a.exists():continue
                comparisons.append(dict(reference=str(a),actual=str(z),identical=digest(a)==digest(z)))
            (base.parent/"render_compatibility.json").write_text(json.dumps(comparisons,indent=2))
            assert all(r["identical"] for r in comparisons),"No-render changed a reference CSV; stop"
        indexed[key]=dict(seed=seed,mode=mode,case=case,path=str(base.with_suffix(".csv")),
                          stable=meta["stable"],duration_s=meta["duration_s"],contacts=meta["contacts"],
                          error=meta.get("error"))
        rows=sorted(indexed.values(),key=lambda r:(CASES.index(r["case"]),r["seed"],r["mode"]))
        index_path.write_text(json.dumps(rows,indent=2))
        print(f"DONE {len(rows)}/80 {seed} {mode} {case}: stable={meta['stable']} duration={meta['duration_s']} contacts={meta['contacts']['events']}",flush=True)
    assert len(rows)==80
    print("THRESHOLD_9_CAMPAIGN_COMPLETE; no threshold 6.63 runs",flush=True)


if __name__=="__main__":main()
