"""Sequential, frozen A/B experiment for flow gate v2; no automatic A-to-B run."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from run_flow_gate_campaign import CASES, case_name


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def snapshot(native):
    paths = [p for directory in ("sim", "src", "bindings") for p in Path(directory).rglob("*")
             if p.is_file() and p.suffix in (".py", ".c", ".h", ".i")]
    paths += list(Path("sim/tests").glob("*golden.json"))
    return dict(native_sha256=digest(native), source_sha256={str(p): digest(p) for p in sorted(paths)})


def gpu_free():
    raw = subprocess.check_output(["nvidia-smi", "--query-gpu=memory.free", "--format=csv,noheader,nounits"], text=True)
    free = int(raw.strip().splitlines()[0])
    if free < 6144:
        raise RuntimeError(f"GPU free memory {free} MiB < 6144 MiB; no flight started")
    return free


def command(root, seed, mode, case):
    out = root/f"seed{seed}"/mode/case
    fault = "spike" if case.startswith("spike") else "scale" if case=="scale05" else case
    args = [sys.executable,"sim/tests/smoke_firmware_ekf.py","--mode","ekf_closed_loop",
            "--noise","on","--seed",str(seed),"--scene","flow-empty","--trajectory","flow-shuttle-x",
            "--flow-fault",fault,"--flow-fault-seed",str(seed),"--flow-fault-offset","6",
            "--flow-fault-duration","4","--flow-gate","2" if mode=="v2" else "0",
            "--flow-gate-threshold","9.0","--flow-gate-max-reject","10","--flow-gate-recover-k","10",
            "--flow-audit","--flow-gate-log","--no-render","--no-plots","--output-dir",str(out)]
    if fault=="spike": args += ["--flow-spike-amplitude",case[5:]]
    if fault=="scale": args += ["--flow-scale-factor","0.5"]
    return args,out/case_name(case)


def verify_meta(path,seed,mode,case,native_hash):
    meta=json.loads(path.read_text())
    assert meta["mode"]=="ekf_closed_loop" and meta["trajectory"]=="flow-shuttle-x"
    assert meta["scene"]["scene"]=="flow-empty" and meta["mass_kg"]==.033
    assert meta["settings"]["noise"] and meta["settings"]["seed"]==seed
    f=meta["settings"]["flow_fault"]
    expected="spike" if case.startswith("spike") else "scale" if case=="scale05" else case
    assert f["kind"]==expected and f["resolved_seed"]==seed and f["start_s"]==26 and f["end_s"]==30
    if expected=="spike": assert f["spike_amplitude"]==float(case[5:]) and f["spike_probability"]==.1
    if expected=="scale": assert f["scale_factor"]==.5
    g=meta["flow_gate"]
    assert (g["enabled"],g["threshold"],g["max_reject"],g["logging"])==(2 if mode=="v2" else 0,9.,10,True)
    if mode=="v2": assert g["recover_k"]==10
    assert meta["bindings_sha256"]==native_hash and meta["render_every_steps"]==0
    assert "analysis_error" not in meta,meta.get("analysis_error")
    if "error" in meta: assert meta["error"]=="AssertionError('Flight left the test arena')",meta["error"]
    return meta


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("dataset",choices=("A","B"))
    p.add_argument("--output-dir",type=Path,default=Path("build/step5_flow_gate_v2"))
    args=p.parse_args();root=args.output_dir.resolve()
    plan_path=root/"pre_results_plan.json";plan=json.loads(plan_path.read_text())
    assert plan["parameters"]==dict(threshold=9.,maxReject=10,recoverK=10)
    assert tuple(plan["cases"])==CASES
    legacy=json.loads((root/"legacy_replay/summary.json").read_text())
    assert len(legacy)==80 and all(r["identical"] for r in legacy)
    import _cffirmware
    native=Path(_cffirmware.__file__)
    frozen=snapshot(native);frozen["plan_sha256"]=digest(plan_path)
    freeze_path=root/"frozen.json"
    if freeze_path.exists(): assert json.loads(freeze_path.read_text())==frozen,"Frozen source/config changed"
    else:
        assert args.dataset=="A"
        freeze_path.write_text(json.dumps(frozen,indent=2))
    if args.dataset=="B":
        assert len(json.loads((root/"A/runs.json").read_text()))==40
        review=json.loads((root/"a_review.json").read_text())
        assert review["proceed_to_B"] is True and review["frozen_sha256"]==digest(freeze_path)
    seeds=plan["development_seeds" if args.dataset=="A" else "validation_seeds"]
    modes=("v2",) if args.dataset=="A" else ("stock","v2")
    target=root/args.dataset;target.mkdir(exist_ok=True)
    index=target/"runs.json"
    indexed={(r["seed"],r["mode"],r["case"]):r for r in json.loads(index.read_text())} if index.exists() else {}
    total=len(seeds)*len(modes)*len(CASES)
    for case in CASES:
        for seed in seeds:
            for mode in modes:
                assert snapshot(native)=={k:v for k,v in frozen.items() if k!="plan_sha256"},"Frozen code/native changed"
                assert digest(plan_path)==frozen["plan_sha256"]
                cmd,base=command(target,seed,mode,case)
                if not base.with_suffix(".json").exists():
                    if base.with_suffix(".csv").exists(): raise RuntimeError(f"Partial run requires review: {base}")
                    free=gpu_free();base.parent.mkdir(parents=True,exist_ok=True)
                    started=datetime.now(timezone.utc).isoformat();tick=time.monotonic()
                    print(f"BEGIN {args.dataset} {seed} {mode} {case}; GPU free {free} MiB",flush=True)
                    with (base.parent/"run.log").open("w") as log:
                        code=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT).returncode
                    execution=dict(command=cmd,exit_code=code,started_utc=started,wall_seconds=time.monotonic()-tick,gpu_free_mib=free)
                    (base.parent/"execution.json").write_text(json.dumps(execution,indent=2))
                    if code not in (0,1): raise RuntimeError(f"Infrastructure exit {code}: {base}")
                meta=verify_meta(base.with_suffix(".json"),seed,mode,case,frozen["native_sha256"])
                for suffix in (".csv",".measurements.csv",".flow_samples.csv",".flow_updates.csv",".flow_gate.csv",".contacts.csv"):
                    assert base.with_suffix(suffix).exists(),(base,suffix)
                indexed[(seed,mode,case)]=dict(dataset=args.dataset,seed=seed,mode=mode,case=case,path=str(base.with_suffix(".csv")),
                    stable=meta["stable"],duration_s=meta["duration_s"],contacts=meta["contacts"],error=meta.get("error"))
                rows=sorted(indexed.values(),key=lambda r:(CASES.index(r["case"]),r["seed"],r["mode"]))
                index.write_text(json.dumps(rows,indent=2))
                print(f"DONE {args.dataset} {len(rows)}/{total} {seed} {mode} {case}: stable={meta['stable']} duration={meta['duration_s']}",flush=True)
    assert len(indexed)==total
    print(f"DATASET_{args.dataset}_COMPLETE; no automatic next dataset or threshold sweep",flush=True)


if __name__=="__main__": main()
