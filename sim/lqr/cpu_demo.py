"""CPU-only nonlinear smoke comparison using REAL C PID/LQR/attitude/mixer.

Not Isaac/PhysX, not the stock EKF, not flight evidence. Quaternion rigid body,
ideal motor force mapping, assumed inertia. Optional synthetic XY feedback
noise/delay is deliberately labelled separately from a sensor/EKF experiment.
"""
import csv
import json
import math
from pathlib import Path
import sys
from collections import deque
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from controllers.firmware_pid import rotation_and_rpy
from actuators import MOTOR_POSITIONS_BODY_M, BODY_YAW_SIGNS, THRUST_TO_TORQUE_M, THRUST_MIN_N
from native import NativeController, ROOT
from compare import summarize

def simulate(lqr, path, beta=0., noisy=False, delay_steps=0, seed=7):
    dt=.001; mass=.030
    inertia=np.array([1.6572e-5,1.6656e-5,2.9262e-5])
    p=np.array([0.,0.,1.]); v=np.zeros(3); w=np.zeros(3); q=np.array([1.,0.,0.,0.])
    c=NativeController(lqr)
    rng=np.random.default_rng(seed); history=deque(maxlen=delay_steps+1)
    with path.open("w",newline="") as file:
        writer=None
        for tick in range(30001):
            t=tick*dt
            rotation,rpy=rotation_and_rpy(q)
            measured=np.r_[p,v,np.degrees(rpy)*[1,-1,1],np.degrees(w)]
            history.append(measured.copy())
            feedback=measured.copy()
            feedback[[0,1,3,4]]=history[0][[0,1,3,4]]
            if noisy: feedback[[0,1,3,4]]+=rng.normal(0,[.005,.005,.01,.01])
            target=np.array([.5 if t>=20 else 0.,0.,1.])
            out=c.step(feedback,[*target,0,0,0],tick=tick)
            forces=out[13:17].astype(float)*(.12/65535)
            forces[forces<THRUST_MIN_N]=0
            torque=np.array([MOTOR_POSITIONS_BODY_M[:,1]@forces,
                             -MOTOR_POSITIONS_BODY_M[:,0]@forces,
                             BODY_YAW_SIGNS@forces*THRUST_TO_TORQUE_M])
            row=dict(time_s=t,roll_ref_deg=float(out[0]),pitch_ref_deg=float(out[1]))
            for i,a in enumerate("xyz"):
                row.update({a+"_m":p[i], "v"+a+"_m_s":v[i], "target_"+a+"_m":target[i]})
            if writer is None:
                writer=csv.DictWriter(file,fieldnames=list(row)); writer.writeheader()
            writer.writerow(row)
            if not np.isfinite(measured).all() or np.linalg.norm(p)>20 or abs(rpy[:2]).max()>1:
                raise RuntimeError(f"CPU dynamics diverged at {t:g}s; results are incomplete")
            if tick==30000: break
            acceleration=rotation[:,2]*sum(forces)/mass-[0,0,9.81]-np.r_[beta*v[:2],0.]
            wdot=(torque-np.cross(w,inertia*w))/inertia
            # Quaternion exponential with midpoint angular rate, body -> world.
            mid=w+.5*dt*wdot; angle=np.linalg.norm(mid)*dt
            dq=np.r_[math.cos(angle/2), mid*dt*.5*np.sinc(angle/(2*np.pi))]
            qw,qv=q[0],q[1:]; dw,dv=dq[0],dq[1:]
            q=np.r_[qw*dw-qv@dv, qw*dv+dw*qv+np.cross(qv,dv)]
            q/=np.linalg.norm(q)
            w+=dt*wdot
            p+=dt*v+.5*dt*dt*acceleration; v+=dt*acceleration

def main():
    out=ROOT/"build/lqr/cpu"; out.mkdir(parents=True,exist_ok=True)
    scenarios={"nominal_no_drag":(0.,False,0), "drag_mismatch":(.5,False,0),
               "synthetic_noise_delay":(.5,True,20)}
    rows=[]; curves=[]
    for scenario,conditions in scenarios.items():
        for label,lqr in (("PID",False),("LQR",True)):
            path=out/f"{scenario}_{label}.csv"
            simulate(lqr,path,*conditions)
            metrics,data=summarize(path)
            rows.append(dict(scenario=scenario,controller=label,**metrics))
            curves.append((scenario,label,data))
            print(scenario,label,json.dumps(metrics),flush=True)
    (out/"metrics.json").write_text(json.dumps(dict(
        backend="CPU quaternion rigid body + compiled repository PID/LQR/mixer",
        not_validated=["Isaac/PhysX","stock EKF","STM32 timing","real flight"],
        assumptions=dict(mass_kg=.030,inertia_kgm2=[1.6572e-5,1.6656e-5,2.9262e-5],
                         dt_s=.001,motors="instantaneous ideal force",seed=7),
        results=rows),indent=2)+"\n",encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(11,10))
    for scenario,label,d in curves:
        ax=axes[list(scenarios).index(scenario)]
        ax[0].plot(d["time_s"],d["x_m"],label=label)
        ax[1].plot(d["time_s"],d["z_m"],label=label)
        ax[0].set_title(scenario+": x (m)"); ax[1].set_title("Height (m)")
    for row in axes:
        for ax in row: ax.grid(); ax.legend(); ax.set_xlabel("Time (s)")
    fig.suptitle("CPU smoke test: real C controllers; assumed rigid-body plant\nNot Isaac/PhysX, not hardware validation")
    fig.tight_layout(); fig.savefig(out/"comparison.png",dpi=150)
    print(out)

if __name__=="__main__": main()

