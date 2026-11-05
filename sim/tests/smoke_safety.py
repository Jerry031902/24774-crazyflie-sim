#!/usr/bin/env python3
"""LIVE PhysX/PID safety validation; run on the team's Isaac Sim machine.

/isaac-sim/python.sh /workspace/sim/tests/smoke_safety.py --mode firmware_pid
This runner is not a CPU surrogate and requires SimulationApp and cffirmware.
"""

import argparse
import json
from pathlib import Path
import sys
import numpy as np

SIM=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SIM))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=('firmware_pid','ekf_closed_loop'),default='firmware_pid')
    parser.add_argument('--navigator',choices=('firmware','python'),default='firmware')
    parser.add_argument('--case',choices=('all','nominal','ranger_dropout','primary_timeout','navigator_timeout'),default='all')
    parser.add_argument('--output-dir',type=Path,default=SIM.parent/'build/isaac_safety_live')
    parser.add_argument('--seconds',type=float,default=45)
    args=parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds<=0:
        parser.error('--seconds must be positive')
    args.output_dir=args.output_dir.resolve()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True})
    reports=[]
    try:
        import run_sim as runtime
        from build_mission_scene import build_mission_scene
        from navigation.missions import Mission
        from navigation.params import default_params
        from mission_flight import WedgeBugMission
        from flight_demo import FlightLog
        from safety import SafetyConfig
        from ranger import RangerLog
        mission=Mission('safety_smoke','Bounded flight and temporary safety faults',
                        (0,10),(0,8),(),(1,4),(),(4,4),'safety_smoke.usd')
        scene=args.output_dir/'safety_smoke.usd'
        build_mission_scene(mission,scene)
        sim,controller=runtime.setup_simulation(args.mode,target_z_m=1.,noise=False,seed=7,ranger=True,
            ranger_noise=False,scene_path=scene,initial_position=(1,4,runtime.INITIAL_POSITION[2]),
            ekf_initial_xy_yaw=(1,4,0))
        cases=('nominal','ranger_dropout','primary_timeout','navigator_timeout') if args.case=='all' else (args.case,)
        for case in cases:
            runtime.reset_runtime(sim,controller)
            cfg=SafetyConfig()
            mission_run=WedgeBugMission(mission,default_params(),1.,backend=args.navigator,
                                       use_estimate=args.mode=='ekf_closed_loop',safety_config=cfg)
            if case!='nominal': mission_run.safety_faults={case:[(3.,4.)]}
            name=f'{args.mode}_{args.navigator}_{case}'
            csv_path=args.output_dir/f'{name}.csv'
            flight=FlightLog(csv_path)
            ranges=RangerLog(csv_path.with_suffix('.ranger.csv'),runtime._ranger)
            observed_events=0
            braking=[]
            active_brake=None
            max_tilt=0.
            max_nav_altitude_error=0.
            finite_states=True
            try:
                for tick in range(round(args.seconds/runtime.PHYSICS_DT)):
                    now=tick*runtime.PHYSICS_DT
                    state=runtime.read_state(now)
                    finite_states &= bool(np.isfinite(np.r_[state['position_m'],state['linear_velocity_m_s'],
                                                             state['orientation_wxyz']]).all())
                    ranges.write(runtime.sample_ranger(state))
                    target=mission_run.target(state)
                    command=controller.compute(state,target)
                    runtime.apply_command(command)
                    if 'estimate' in command: mission_run.observe_estimate(command['estimate'])
                    flight.write(state,target,command,mission_run.phase,mission_run.log_fields())
                    if mission_run.phase!='takeoff':
                        max_nav_altitude_error=max(max_nav_altitude_error,abs(float(state['position_m'][2])-1.))
                        assert abs(target['position_m'][2]-1.)<1e-9,'Safety changed the altitude target'
                        assert 'velocity_body_m_s' in target
                    events=mission_run.safety.events
                    for event in events[observed_events:]:
                        if event['type']=='brake':
                            v=np.asarray(state['linear_velocity_m_s'])[:2]
                            speed=float(np.linalg.norm(v))
                            active_brake={'start_s':now,'speed_m_s':speed,'direction':v/max(speed,1e-12),
                                          'origin':np.asarray(state['position_m'])[:2].copy(),
                                          'max_forward_travel_m':0.,'bound_m':cfg.braking_distance(speed)}
                            braking.append(active_brake)
                    observed_events=len(events)
                    if active_brake is not None:
                        delta=np.asarray(state['position_m'])[:2]-active_brake['origin']
                        active_brake['max_forward_travel_m']=max(active_brake['max_forward_travel_m'],
                                                                 float(delta@active_brake['direction']))
                        if np.linalg.norm(state['linear_velocity_m_s'][:2])<=cfg.stop_speed:
                            active_brake['stopped_s']=now
                            active_brake=None
                    from controllers.firmware_pid import rotation_and_rpy
                    R,_=rotation_and_rpy(state['orientation_wxyz'])
                    max_tilt=max(max_tilt,float(np.arccos(np.clip(R[2,2],-1,1))))
                    sim.step(render=False)
                    if mission_run.done: break
            finally:
                flight.close(); ranges.close()
                mission_run.write_safety_report(csv_path.with_suffix('.safety.json'))
            result=mission_run.summary()
            fault_brakes=[e for e in mission_run.safety.events if e['type']=='brake' and e['source']=='backup']
            checks={'arrived':bool(result['arrived']),'no_horizontal_collision':not result['collided'],
                    'no_controller_exception':not mission_run.safety.last_error,
                    'finite_states':finite_states,'navigation_altitude_within_30cm':max_nav_altitude_error<.3,
                    'tilt_below_45_deg':max_tilt<np.pi/4}
            if case!='nominal':
                expected='ranger_timeout' if case=='ranger_dropout' else case
                checks['backup_intervened']=any(e['reason']==expected for e in fault_brakes)
                checks['stopped_and_resumed']=bool(fault_brakes) and any(
                    e['type']=='stopped' for e in mission_run.safety.events) and any(
                    e['type']=='resume' and e['t']>fault_brakes[0]['t'] for e in mission_run.safety.events)
            meaningful=[b for b in braking if b['speed_m_s']>.05]
            if case!='nominal': checks['braking_test_exercised']=bool(meaningful)
            checks['observed_braking_within_assumed_envelope']=all(
                'stopped_s' in b and b['max_forward_travel_m']<=b['bound_m']+.01 for b in meaningful)
            clean_braking=[{k:v for k,v in b.items() if k not in ('direction','origin')} for b in braking]
            report={'case':case,'mode':args.mode,'navigator':args.navigator,'passed':all(checks.values()),
                    'checks':checks,'summary':result,'braking_observations':clean_braking,
                    'scope':'One noise-free live run; does not certify the envelope for all states/noise'}
            reports.append(report)
            (args.output_dir/f'{name}.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2),flush=True)
        (args.output_dir/'summary.json').write_text(json.dumps(reports,indent=2)+'\n')
        if not all(r['passed'] for r in reports):
            raise AssertionError('Live safety checks failed; inspect CSVs and recalibrate braking assumptions')
        print('ISAAC_SAFETY_LIVE_PASSED',flush=True)
    finally:
        app.close()


if __name__=='__main__': main()
