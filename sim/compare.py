"""Run matched Isaac/PhysX PID vs LQR experiments, then summarize REAL CSVs.

Use --run on the team's Isaac computer. Each controller gets a fresh process
because firmware controllers have static global state. No invented results.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SCENARIOS = {
    "truth": ("firmware_pid", "firmware_lqr", "off", 0),
    "ekf_clean": ("ekf_closed_loop", "ekf_lqr", "off", 0),
    "ekf_noise": ("ekf_closed_loop", "ekf_lqr", "on", 0),
    "ekf_noise_delay": ("ekf_closed_loop", "ekf_lqr", "on", 20),
}

def summarize(path):
    with Path(path).open(newline="") as file:
        rows=list(csv.DictReader(file))
    required=["time_s","x_m","y_m","z_m","target_x_m","target_y_m","target_z_m",
              "vx_m_s","vy_m_s","roll_ref_deg","pitch_ref_deg"]
    if not rows or any(k not in rows[0] for k in required):
        raise ValueError(f"{path}: missing finite flight/control columns; rebuild bindings")
    d={k:np.array([float(r[k]) for r in rows]) for k in required}
    if not all(np.isfinite(v).all() for v in d.values()):
        raise ValueError(f"{path}: nonfinite data")
    t=d["time_s"]
    if len(t)<2 or np.any(np.diff(t)<=0):
        raise ValueError("Need strictly increasing simulation time")
    # Fixed schedule: 0..10 takeoff, 10..20 hold, 20..30 +0.5 m step.
    if t[-1]<29.99:
        raise ValueError(f"{path}: incomplete comparison ({t[-1]:g} s)")
    move=t>=20; hover=(t>=10)&(t<20); evaluation=t>=10
    ex=d["x_m"]-d["target_x_m"]; ey=d["y_m"]-d["target_y_m"]
    horiz=np.hypot(ex,ey)
    speed=np.hypot(d["vx_m_s"],d["vy_m_s"])
    ok=(horiz[move]<=.03)&(speed[move]<=.05)
    # Earliest time after which the trajectory STAYS within both tolerances.
    remaining=np.logical_and.accumulate(ok[::-1])[::-1]
    candidates=np.flatnonzero(remaining)
    settled=None
    if len(candidates):
        candidate=t[move][candidates[0]]
        if t[-1]-candidate >= 1:
            settled=float(candidate-20)
    tilt=np.radians(np.column_stack((d["roll_ref_deg"],d["pitch_ref_deg"])))
    dt=np.diff(t)
    effort=float(np.sum(np.sum(tilt[:-1]**2,axis=1)[evaluation[:-1]]*dt[evaluation[:-1]]))
    result=dict(horizontal_rmse_m=float(np.sqrt(np.mean(horiz[evaluation]**2))),
                hover_rmse_m=float(np.sqrt(np.mean(horiz[hover]**2))),
                move_rmse_m=float(np.sqrt(np.mean(horiz[move]**2))),
                x_step_overshoot_m=float(max(0,np.max(ex[move]))),
                x_step_overshoot_percent=float(max(0,np.max(ex[move]))/.5*100),
                settling_time_s=settled,
                settling_definition="within 0.03 m XY and 0.05 m/s, stays through end for >=1 s",
                altitude_rmse_m=float(np.sqrt(np.mean((d["z_m"][evaluation]-d["target_z_m"][evaluation])**2))),
                angle_effort_rad2_s=effort,
                peak_axis_command_deg=float(np.max(abs(np.degrees(tilt)))),
                duration_s=float(t[-1]),
                final_horizontal_error_m=float(horiz[-1]))
    if "controller_est_x_m" in rows[0]:
        est=np.array([[float(r["controller_est_x_m"]),float(r["controller_est_y_m"])] for r in rows])
        true=np.column_stack((d["x_m"],d["y_m"]))
        result["estimator_xy_rmse_m"]=float(np.sqrt(np.mean(np.sum((est[evaluation]-true[evaluation])**2,axis=1))))
    return result,d

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--python", default="/isaac-sim/python.sh", help="Isaac Python launcher")
    parser.add_argument("--out", type=Path, default=ROOT/"build/lqr/physx")
    parser.add_argument("--scenario", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    parser.add_argument("--seeds", nargs="+", type=int, default=[7,11,23])
    args=parser.parse_args()
    if any(s<0 for s in args.seeds): parser.error("Seeds must be nonnegative")
    args.out.mkdir(parents=True,exist_ok=True)
    results=[]; curves=[]
    for scenario in args.scenario:
        pid,lqr,noise,delay=SCENARIOS[scenario]
        for seed in args.seeds:
            for label,mode in (("PID",pid),("LQR",lqr)):
                name=f"{scenario}_seed{seed}_{label}"
                path=args.out/(name+".csv")
                command=[args.python,str(ROOT/"sim/run_sim.py"),"--mode",mode,
                         "--comparison-demo","--headless","--hover-seconds","10",
                         "--target-z","1","--ranger","off","--noise",noise,
                         "--seed",str(seed),"--delay-ms",str(delay),"--csv",str(path)]
                if args.run:
                    if path.exists():
                        raise FileExistsError(f"Refusing to overwrite results: {path}; choose a new --out")
                    metadata=dict(command=command,scenario=scenario,seed=seed,controller=label,
                                  tilt_limit_deg_per_axis=10,backend="Isaac/PhysX",
                                  plant="repository actuator/rigid-body model; no added aerodynamic drag")
                    path.with_suffix(".run.json").write_text(json.dumps(metadata,indent=2)+"\n",encoding="utf-8")
                    with path.with_suffix(".stdout.txt").open("w",encoding="utf-8") as log:
                        subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
                if not path.exists():
                    raise FileNotFoundError(f"No measured CSV: {path}. Run with --run on the Isaac computer.")
                metrics,data=summarize(path)
                results.append(dict(scenario=scenario,seed=seed,controller=label,**metrics))
                if seed==args.seeds[0]: curves.append((scenario,label,data))
    (args.out/"metrics.json").write_text(json.dumps(results,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    with (args.out/"metrics.csv").open("w",newline="") as file:
        keys=list(dict.fromkeys(k for row in results for k in row))
        writer=csv.DictWriter(file,fieldnames=keys); writer.writeheader(); writer.writerows(results)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(len(args.scenario),2,squeeze=False,figsize=(11,3*len(args.scenario)))
    for scenario,label,d in curves:
        ax=axes[args.scenario.index(scenario)]
        ax[0].plot(d["time_s"],d["x_m"],label=label)
        ax[1].plot(d["time_s"],np.hypot(d["x_m"]-d["target_x_m"],d["y_m"]-d["target_y_m"]),label=label)
        ax[0].set_title(scenario+": world x (m)")
        ax[1].set_title("True XY tracking error (m)")
    for row in axes:
        for ax in row: ax.grid(); ax.legend(); ax.set_xlabel("Time (s)")
    fig.tight_layout(); fig.savefig(args.out/"comparison.png",dpi=160)
    print(f"Wrote actual-run metrics and comparison.png to {args.out}")

if __name__=="__main__": main()

