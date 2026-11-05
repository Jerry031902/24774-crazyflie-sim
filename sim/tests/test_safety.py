"""CPU-only safety contracts and mission integration; no Isaac/GPU required."""

import copy
from dataclasses import replace
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import numpy as np

SIM=Path(__file__).resolve().parents[1]
ROOT=SIM.parent
sys.path.insert(0,str(SIM))
from safety import SafetyConfig, Observation, RangeFrame, SafetySupervisor, EstimatedRangeTransport
from safety.core import (RangeProcessor, primary_monitor, backup_monitor, HIT, NO_RETURN, INVALID)
from navigation.params import default_params
from navigation.wedgebug import WedgeBugNavigator
from navigation.missions import Mission
from mission_flight import WedgeBugMission
from ranger import MultiRanger, RayHit


def observe(position=(1.,4.,1.), velocity=(0.,0.,0.), yaw=0.):
    a,b=math.cos(yaw),math.sin(yaw)
    return Observation(np.array(position,dtype=float),np.array(velocity,dtype=float),
                       np.array([[a,-b,0],[b,a,0],[0,0,1.]]),yaw)


def frame(now=.2, seq=2, ranges=None, yaw=0., position=(1.,4.,1.)):
    values=np.full(4,np.inf) if ranges is None else np.asarray(ranges,dtype=float)
    return RangeFrame(seq,max(0.,now-.1),now,values,np.where(np.isfinite(values),HIT,NO_RETURN),
                      np.array(position,dtype=float),observe(yaw=yaw).rotation)


class FakeNavigator:
    def __init__(self, u=(.5,0), yaw=0):
        self.u=np.array(u,dtype=float); self.yaw=yaw
        self.calls=0; self.resumes=0
    def checkpoint(self): return self.calls
    def restore(self, state): self.calls=state
    def resume(self,*args): self.resumes+=1
    def step(self,*args,**kwargs):
        self.calls+=1
        return self.u.copy(),self.yaw


class MonitorTests(unittest.TestCase):
    def setUp(self): self.c=SafetyConfig()

    def test_braking_envelope_is_not_the_matlab_tau_model(self):
        self.assertAlmostEqual(self.c.braking_distance(.5),.35)
        self.assertAlmostEqual(replace(self.c,brake_accel=1,brake_response=.1).braking_distance(.5),.175)

    def test_no_packet_invalid_packet_and_valid_no_return_are_distinct(self):
        args=(observe(),np.zeros(2),.2,self.c)
        self.assertTrue(primary_monitor(RangeFrame.unavailable(),np.full(4,np.inf),*args).trip)
        f=frame(); self.assertFalse(primary_monitor(f,f.ranges,*args).trip)
        f.status[0]=INVALID
        self.assertEqual(primary_monitor(f,f.ranges,*args).reason,'invalid_raw_frame')

    def test_stationary_dangerous_candidate_is_rejected_before_motion(self):
        f=frame(ranges=[.20,np.inf,np.inf,np.inf])
        self.assertFalse(primary_monitor(f,f.ranges,observe(),np.zeros(2),.2,self.c).trip)
        self.assertTrue(primary_monitor(f,f.ranges,observe(),np.array([.5,0]),.2,self.c).trip)

    def test_side_rear_and_unobserved_diagonal_commands(self):
        for i,u in [(1,(0,.5)),(2,(-.5,0)),(3,(0,-.5))]:
            f=frame(); f.ranges[i]=.2; f.status[i]=HIT
            self.assertTrue(primary_monitor(f,f.ranges,observe(),np.array(u),.2,self.c).trip)
        f=frame()
        self.assertEqual(primary_monitor(f,f.ranges,observe(),np.array([.3,.3]),.2,self.c).reason,
                         'unobserved_direction')

    def test_pure_yaw_does_not_create_physical_closing(self):
        f=frame(ranges=[.3,.3,.3,.3],yaw=.5)
        self.assertFalse(primary_monitor(f,f.ranges,observe(yaw=.8),np.zeros(2),.2,self.c).trip)

    def test_backup_ignores_corrupt_processed_data(self):
        f=frame(ranges=[.15,np.inf,np.inf,np.inf]); o=observe(velocity=(.5,0,0))
        self.assertFalse(primary_monitor(f,np.full(4,4.),o,np.zeros(2),.2,self.c).trip)
        result=backup_monitor(f,o,.2,.2,.2,True,0,-math.inf,self.c)
        self.assertEqual(result.reason,'backup_distance')

    def test_hard_gap_trips_at_rest(self):
        f=frame(ranges=[.05,np.inf,np.inf,np.inf])
        result=backup_monitor(f,observe(),.2,.2,.2,False,0,-math.inf,self.c)
        self.assertEqual(result.reason,'hard_distance')

    def test_stale_future_invalid_and_rollback_frames(self):
        for f,reason in [(frame(now=0), 'ranger_timeout'),
                         (replace(frame(),delivery_time=.3),'invalid_raw_frame'),
                         (replace(frame(),ranges=np.array([np.nan,1,1,1])),'invalid_raw_frame')]:
            self.assertEqual(backup_monitor(f,observe(),.5 if reason=='ranger_timeout' else .2,
                                           .2,.2,False,0,-math.inf,self.c).reason,reason)
        self.assertEqual(backup_monitor(frame(),observe(),.2,.2,.2,False,3,.15,self.c).reason,
                         'nonmonotonic_frame')

    def test_separate_primary_and_navigation_heartbeats(self):
        f=frame()
        self.assertEqual(backup_monitor(f,observe(),.2,0,.2,True,0,-math.inf,self.c).reason,'primary_timeout')
        self.assertEqual(backup_monitor(f,observe(),.2,.2,0,True,0,-math.inf,self.c).reason,'navigator_timeout')

    def test_tilt_is_checked_and_cones_use_full_sample_rotation(self):
        o=observe(); a=math.radians(30)
        o.rotation=np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])
        self.assertEqual(backup_monitor(frame(),o,.2,.2,.2,False,0,-math.inf,self.c).reason,'tilt_limit')
        f=frame(); f.rotation=o.rotation
        self.assertEqual(primary_monitor(f,f.ranges,observe(),np.array([.5,0]),.2,self.c).reason,
                         'unobserved_direction')

    def test_overspeed_triggers_raw_backup(self):
        result=backup_monitor(frame(),observe(velocity=(1.2,0,0)),.2,.2,.2,False,0,-math.inf,self.c)
        self.assertEqual(result.reason,'overspeed')

    def test_filter_counts_distinct_packets_and_resets_on_rotation(self):
        processor=RangeProcessor(self.c)
        f=frame(ranges=[1,1,1,1]); processor.update(f,.2)
        processor.update(f,.22); self.assertEqual(len(processor.frames),1)
        processor.update(frame(.3,3,[2,2,2,2]),.3)
        self.assertEqual(len(processor.frames),2)
        processor.update(frame(.4,4,[3,3,3,3],yaw=.2),.4)
        self.assertEqual(len(processor.frames),1)
        np.testing.assert_array_equal(processor.closing_rate,np.zeros(4))

    def test_configuration_rejects_invalid_braking_bounds(self):
        for kw in ({'brake_accel':0},{'backup_brake_accel':-1},{'brake_response':math.nan},
                   {'noise_std':-.1},{'clear_frames':0}):
            with self.subTest(kw=kw),self.assertRaises(ValueError): SafetyConfig(**kw)


class ArbiterTests(unittest.TestCase):
    def ready(self,u=(.5,0),yaw=0):
        s=SafetySupervisor(); nav=FakeNavigator(u,yaw)
        for k in range(1,4): s.step(observe(),frame(k*.1,k),k*.1,nav)
        return s,nav

    def test_recovery_requires_distinct_healthy_frames(self):
        s=SafetySupervisor(); n=FakeNavigator()
        for now in [.1,.12,.14,.16]: s.step(observe(),frame(.1,1),now,n)
        self.assertEqual(s.state,'HOLD'); self.assertEqual(n.calls,0)
        s.step(observe(),frame(.2,2),.2,n)
        s.step(observe(),frame(.3,3),.3,n)
        self.assertEqual(s.state,'RECOVER')
        u,_=s.step(observe(),frame(.3,3),.32,n)
        self.assertGreater(u[0],0); self.assertLess(u[0],.05)

    def test_rejected_navigation_rolls_back_and_yaw_stops(self):
        s,n=self.ready(yaw=1)
        s.state='NORMAL'
        u,yaw=s.step(observe(velocity=(.5,0,0)),frame(.4,4,[.1,4,4,4]),.4,n)
        self.assertEqual(n.calls,0)
        np.testing.assert_array_equal(u,[0,0]); self.assertEqual(yaw,0)
        self.assertEqual(s.state,'BRAKE')

    def test_stopping_holding_and_replanning(self):
        s,n=self.ready(); s.state='NORMAL'
        s.step(observe(velocity=(.5,0,0)),frame(.4,4,[.1,4,4,4]),.4,n)
        s.step(observe(),frame(.5,5),.5,n)
        self.assertEqual(s.state,'HOLD')
        s.step(observe(),frame(.6,6),.6,n)
        s.step(observe(),frame(.7,7),.7,n)
        self.assertEqual(s.state,'RECOVER'); self.assertEqual(n.resumes,1)

    def test_hold_feedback_is_bounded_and_points_back(self):
        s,n=self.ready(); s.state='HOLD'; s.hold_position=np.array([1.,4.])
        s.clear_count=0
        u,yaw=s.step(observe(position=(1.5,4,1)),frame(.4,4),.4,n)
        self.assertAlmostEqual(u[0],-.1); self.assertEqual(yaw,0)

    def test_holding_correction_cannot_bypass_distance_or_blind_direction(self):
        for target,raw in [([2,4],[.1,np.inf,np.inf,np.inf]),([2,5],None)]:
            s,n=self.ready(); s.state='HOLD'; s.hold_position=np.array(target,dtype=float)
            s.clear_count=0
            u,yaw=s.step(observe(),frame(.4,4,raw),.4,n)
            np.testing.assert_allclose(u,[0,0]); self.assertTrue(s.hold_limited)

    def test_invalid_processed_data_and_primary_failure_fall_back(self):
        s,n=self.ready(); s.state='NORMAL'
        s.step(observe(),frame(.4,4),.4,n,processed_bias=math.nan)
        self.assertEqual(s.reason,'invalid_processed_ranges')
        s,n=self.ready(); s.state='NORMAL'
        s.step(observe(),frame(.5,5),.5,n,primary_available=False)
        self.assertEqual(s.reason,'primary_timeout')

    def test_invalid_estimate_never_produces_nan_commands(self):
        s,n=self.ready(); o=observe(); o.velocity[0]=math.nan
        u,yaw=s.step(o,frame(.4,4),.4,n)
        self.assertTrue(np.isfinite(np.r_[u,yaw]).all()); self.assertFalse(s.accepted)

    def test_recovery_from_invalid_initial_position_does_not_poison_hold(self):
        s=SafetySupervisor(); n=FakeNavigator(); bad=observe()
        bad.position[0]=math.nan
        u,yaw=s.step(bad,frame(.1,1),.1,n)
        self.assertTrue(np.isfinite(np.r_[u,yaw]).all()); self.assertIsNone(s.hold_position)
        for i in range(2,6):
            u,yaw=s.step(observe(),frame(i*.1,i),i*.1,n)
            self.assertTrue(np.isfinite(np.r_[u,yaw]).all())
        np.testing.assert_allclose(s.hold_position,[1,4])


class TransportAndNavigationTests(unittest.TestCase):
    def test_malformed_transport_and_mutated_held_packet_fail_closed(self):
        tr=EstimatedRangeTransport()
        f=tr.frame({'ranger_frame_seq':1,'ranger_sample_time_s':'broken'})
        self.assertTrue((f.status==INVALID).all())
        src={'position_m':(1,4,1),'linear_velocity_m_s':(0,0,0),'orientation_wxyz':(1,0,0,0)}
        tr.observe(0,src)
        fields={'ranger_frame_seq':1,'ranger_sample_time_s':0.,'ranger_delivery_time_s':.1}
        for d in ('front','left','back','right'):
            fields.update({f'range.{d}':1000,f'range_{d}_valid':1,f'range_{d}_status':HIT})
        self.assertTrue((tr.frame(fields).status==HIT).all())
        fields['range.front']=900
        self.assertTrue((tr.frame(fields).status==INVALID).all())

    def test_navigation_frame_times_use_navigation_epoch(self):
        class Capture(FakeNavigator):
            def step(self,*args,**kw):
                self.sample_time=kw['frame'].sample_time
                return super().step(*args,**kw)
        s=SafetySupervisor(); s.state='NORMAL'; n=Capture()
        s.step(observe(),frame(10.2,102),10.2,n,nav_time=.2)
        self.assertAlmostEqual(n.sample_time,.1)
        self.assertAlmostEqual(s.processor.last.sample_time,10.1)

    def test_transport_uses_sample_time_estimates_not_current_pose(self):
        sensor=MultiRanger(lambda *args:None); transport=EstimatedRangeTransport()
        for ms in range(101):
            truth={'time_s':ms/1000,'position_m':(99,99,1),'orientation_wxyz':(1,0,0,0)}
            source={'position_m':(1+ms/1000,4,1),'linear_velocity_m_s':(1,0,0),
                    'orientation_wxyz':(1,0,0,0)}
            transport.observe(ms/1000,source); sensor.sample(truth)
        f=transport.frame(sensor.snapshot())
        np.testing.assert_allclose(f.position,[1,4,1]); self.assertEqual(f.sample_time,0)
        self.assertTrue(np.isposinf(f.ranges).all()); self.assertTrue((f.status==NO_RETURN).all())
        self.assertEqual(f.seq,1)

    def test_invalid_sensor_status_never_becomes_valid_free_space(self):
        sensor=MultiRanger(lambda *args:RayHit(.5,'/Wall'),bias_m=-1)
        tr=EstimatedRangeTransport()
        for ms in range(101):
            source={'time_s':ms/1000,'position_m':(1,4,1),'linear_velocity_m_s':(0,0,0),
                    'orientation_wxyz':(1,0,0,0)}
            tr.observe(ms/1000,source); sensor.sample(source)
        f=tr.frame(sensor.snapshot())
        self.assertTrue((f.status==INVALID).all())

    def test_python_resume_preserves_waypoint_side_and_episode_history(self):
        n=WedgeBugNavigator((1,4),[(3,4),(9,4)],default_params()); n.prepare_safety()
        n._start_avoid(np.array([1.,4.]),0); n.side=1; n.sweep_spots=[[1,4,0]]
        n.scan={'t':[.1],'ang':[0],'rng':[1]}; n.subgoal=np.array([4,5])
        n.resume(np.array([2,4,0,0]),.2,1)
        self.assertEqual(n.wp,0); self.assertEqual(n.side,1); self.assertEqual(len(n.sweep_spots),1)
        self.assertEqual(n.phase,'sweep'); self.assertEqual(n.scan['t'],[])
        self.assertTrue(np.isnan(n.subgoal).all())

    def test_timestamped_scan_uses_sample_yaw_and_no_duplicate_packets(self):
        n=WedgeBugNavigator((1,4),[(9,4)],default_params()); n.prepare_safety()
        n._start_avoid(np.array([1.,4.]),0); n._start_sweep(0); n.sweep_stage='scan'
        f=frame(.2,1,[2,3,4,3],yaw=0)
        n.step([1,4,0,0],.2,[4,4,4,4],.2,frame=f)
        self.assertEqual(n.scan['ang'][0],0); self.assertEqual(n.scan['rng'][0],2)
        n.step([1,4,0,0],.3,[4,4,4,4],.22,frame=f)
        self.assertEqual(len(n.scan['t']),3)


class MissionIntegrationTests(unittest.TestCase):
    def make(self,enabled=True,estimate=False):
        m=Mission('safety_test','CPU integration',(0,10),(0,8),(),(1,4),(),(9,4),'unused.usd')
        demo=WedgeBugMission(m,default_params(),1.,backend='python',use_estimate=estimate,
                             safety_config=SafetyConfig() if enabled else None)
        demo.phase='navigate'; demo.nav_start_s=0.; demo.next_nav_s=0.
        return demo

    def feed(self,demo,hit=None,poison_truth=False,end_ms=500):
        sensor=MultiRanger(lambda *args:hit)
        keys=set(demo.log_fields())
        for ms in range(end_ms+1):
            state={'time_s':ms/1000,'position_m':(8,7,1) if poison_truth else (1,4,1),
                   'linear_velocity_m_s':(0,0,0),'orientation_wxyz':(1,0,0,0)}
            sensor.sample(state); state['ranger']=sensor.snapshot()
            if demo.use_estimate:
                demo.observe_estimate({'position_m':(1,4,1),'linear_velocity_m_s':(0,0,0),
                                       'orientation_wxyz':(1,0,0,0)})
            target=demo.target(state)
            self.assertEqual(set(demo.log_fields()),keys)
            self.assertEqual(target['position_m'][2],1.)
        return target

    def test_real_sensor_transport_and_mission_keep_pid_height(self):
        demo=self.make(); target=self.feed(demo)
        self.assertIn('velocity_body_m_s',target); self.assertGreater(target['velocity_body_m_s'][0],0)
        self.assertEqual(demo.safety.state,'RECOVER')
        demo=self.make(); target=self.feed(demo,RayHit(.04,'/Wall'))
        self.assertEqual(demo.safety.reason,'hard_distance'); self.assertEqual(target['yaw_rate_rad_s'],0)
        np.testing.assert_allclose(target['velocity_body_m_s'],[0,0])

    def test_closed_loop_safety_does_not_read_truth_pose(self):
        a,b=self.make(estimate=True),self.make(estimate=True)
        ta,tb=self.feed(a),self.feed(b,poison_truth=True)
        np.testing.assert_allclose(ta['velocity_body_m_s'],tb['velocity_body_m_s'])
        np.testing.assert_allclose(b.safety_frame.position,[1,4,1])

    def test_off_path_has_no_safety_fields_and_reset_clears_histories(self):
        off=self.make(False); self.feed(off)
        self.assertNotIn('safety_state',off.log_fields()); self.assertIsNone(off.safety)
        on=self.make(); self.feed(on); on.reset()
        self.assertEqual(on.safety.state,'HOLD'); self.assertEqual(len(on.safety_transport.poses),0)
        self.assertEqual(on.safety_frame.seq,0)

    def test_navigation_fault_injection_triggers_backup(self):
        demo=self.make(); demo.safety_faults={'navigator_timeout':[(.4,.8)]}
        self.feed(demo,end_ms=600)
        self.assertEqual(demo.safety.reason,'navigator_timeout')

    def test_report_contains_config_events_and_finite_json(self):
        demo=self.make(); self.feed(demo)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'report.json'; demo.write_safety_report(path)
            import json
            report=json.loads(path.read_text())
            self.assertFalse(report['braking_bounds_calibrated'])
            self.assertEqual(report['summary']['safety_enabled'],True)
            self.assertIsNone(report['summary']['t_finish_s'])

    def test_real_flight_csv_schema_and_safety_plot(self):
        from flight_demo import FlightLog
        from plot_safety import plot_safety
        import importlib.util
        if not importlib.util.find_spec('matplotlib'):
            self.skipTest('Matplotlib needed for plot verification')
        import os
        os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'build/isaac_safety/mpl_config'))
        import matplotlib
        matplotlib.use('Agg')
        demo=self.make(); sensor=MultiRanger(lambda *args:None)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'cpu_contract.csv'; logger=FlightLog(path)
            try:
                for ms in range(401):
                    state={'time_s':ms/1000,'position_m':(1,4,1),'linear_velocity_m_s':(0,0,0),
                           'angular_velocity_rad_s':(0,0,0),'orientation_wxyz':(1,0,0,0)}
                    sensor.sample(state); state['ranger']=sensor.snapshot()
                    target=demo.target(state)
                    # A logger-interface fixture, not PID/PhysX flight data.
                    cmd={'tick':ms,'control_legacy':(0,0,0,0),'gyro_body_deg_s':(0,0,0),
                         'motor_commands':[0]*4,'motor_commands_applied':[0]*4,
                         'motor_forces_N':[0]*4,'motor_yaw_torques_Nm':[0]*4,
                         'total_thrust_N':0.,'saturated':False}
                    logger.write(state,target,cmd,demo.phase,demo.log_fields())
            finally:
                logger.close()
            image=plot_safety(path)
            self.assertTrue(image.is_file()); self.assertGreater(image.stat().st_size,10000)


@unittest.skipUnless(shutil.which('gcc'),'host GCC is optional; run C harness on the Isaac machine')
class NativeNavigationTests(unittest.TestCase):
    def test_c_transactions_delayed_scans_and_recovery(self):
        with tempfile.TemporaryDirectory() as folder:
            executable=Path(folder)/('test_safety_nav.exe' if sys.platform=='win32' else 'test_safety_nav')
            command=[shutil.which('gcc'),'-std=c99','-Wall','-Wextra','-Werror',
                     '-I'+str(ROOT/'apps/wedgebug/src'),str(SIM/'tests/test_safety_nav.c'),
                     str(ROOT/'apps/wedgebug/src/wedgebug.c'),'-lm','-o',str(executable)]
            subprocess.run(command,check=True,capture_output=True,text=True)
            output=subprocess.run([str(executable)],check=True,capture_output=True,text=True,timeout=10)
            self.assertIn('C_SAFETY_NAV_PASSED',output.stdout)


if __name__=='__main__': unittest.main()
