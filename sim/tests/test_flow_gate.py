"""Exercise the real scalar C gate and a pre-change stock binary fixture."""

import ctypes
from dataclasses import replace
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import cffirmware as cf
except ImportError:
    cf = None
if cf is not None:
    import numpy as np
    from estimators.firmware_ekf import FirmwareEKF, read_float_array
    from flow_gate import FlowGateConfig
    from flow_faults import FlowFaultConfig
    from sensors import SensorSuite


@unittest.skipIf(cf is None, "Use Isaac Sim Python with built cffirmware")
class FlowGateTests(unittest.TestCase):
    def setUp(self):
        self.configure()

    def tearDown(self):
        self.configure()

    def configure(self, enabled=0, threshold=9., limit=None):
        self.assertTrue(cf.kalmanCoreSetFlowGateConfig(enabled, threshold,
            cf.kalmanCoreGetFlowGateDefaultMaxReject() if limit is None else limit))

    def core(self):
        core, params = cf.kalmanCoreData_t(), cf.kalmanCoreParams_t()
        cf.kalmanCoreDefaultParams(params)
        cf.kalmanCoreInit(core, params, 0)
        self.array(core.S, 9)[:] = [0,0,1,0,0,0,0,0,0]
        self.array(core.P, 81).reshape(9,9)[:] = np.eye(9)*1e-6
        return core

    @staticmethod
    def array(pointer, length):
        return np.ctypeslib.as_array((ctypes.c_float*length).from_address(int(pointer)))

    @staticmethod
    def update(core, x=20., y=20., gyro=None):
        flow = cf.flowMeasurement_t()
        flow.dt, flow.stdDevX, flow.stdDevY = .01, 2., 2.
        flow.dpixelx, flow.dpixely = x, y
        cf.kalmanCoreUpdateWithFlow(core, flow, gyro or cf.Axis3f())
        return cf.kalmanCoreGetFlowGateX(core), cf.kalmanCoreGetFlowGateY(core)

    @staticmethod
    def state_bits(core, nis=False):
        arrays = [read_float_array(getattr(core,k),shape).ravel()
                  for k,shape in (("S",(9,)),("P",(9,9)),("q",(4,)),("R",(3,3)))]
        if nis:
            arrays.append(np.array([cf.kalmanCoreGetFlowNisX(),cf.kalmanCoreGetFlowNisY()],dtype=np.float32))
        return np.concatenate(arrays).view(np.uint32).tolist()

    def test_off_bit_identical_to_independent_step2_stock_binary(self):
        golden = json.loads(Path(__file__).with_name("flow_gate_stock_golden.json").read_text())
        self.assertEqual(golden["source_commit"], "2c30d973")
        self.configure(0, .000001, 1)  # Disabled even when every sample exceeds threshold.
        previous_height = None
        for row in golden["rows"]:
            if row["height"] != previous_height:
                core = self.core()
                self.array(core.S,9)[:] = [.2,-.1,row["height"],.3,-.2,.01,0,0,0]
                p = self.array(core.P,81).reshape(9,9)
                p[:] = np.diag([.02,.03,.004,.02,.03,.01,.001,.001,.001])
                p[3,4]=p[4,3]=.005
                p[2,3]=p[3,2]=.0002
                previous_height = row["height"]
            gyro = cf.Axis3f()
            gyro.x,gyro.y,gyro.z = 1.,-2.,.3
            gx,gy = self.update(core,*row["measurement"],gyro)
            self.assertEqual(self.state_bits(core,nis=True),row["bits"])
            self.assertEqual((gx.decision,gy.decision),(cf.KC_FLOW_ACCEPTED,)*2)

    def test_reject_leaves_state_covariance_and_update_flag_unchanged(self):
        self.configure(1)
        core = self.core()
        before = self.state_bits(core)
        gx,gy = self.update(core)
        self.assertGreater(cf.kalmanCoreGetFlowNisX(),9)
        self.assertEqual(self.state_bits(core),before)
        self.assertFalse(core.isUpdated)
        for g in (gx,gy):
            self.assertEqual((g.decision,g.consecutive,g.runAtDecision,g.rejected),(cf.KC_FLOW_REJECTED,1,1,1))
            self.assertEqual((g.accepted,g.forced),(0,0))

    def test_axes_decide_independently(self):
        self.configure(1)
        core = self.core()
        gx,gy = self.update(core,20,1)
        self.assertEqual((gx.decision,gy.decision),(cf.KC_FLOW_REJECTED,cf.KC_FLOW_ACCEPTED))
        self.assertEqual(read_float_array(core.S,(9,))[3],0.)
        self.assertGreater(read_float_array(core.S,(9,))[4],0.)
        self.assertTrue(core.isUpdated)

    def test_below_threshold_and_exact_threshold_are_accepted(self):
        core = self.core()
        self.update(core,2,0)
        threshold = cf.kalmanCoreGetFlowNisX()
        core = self.core()
        self.configure(1,threshold)
        gx,gy = self.update(core,2,0)
        self.assertEqual(cf.kalmanCoreGetFlowNisX(),threshold)
        self.assertEqual((gx.decision,gy.decision),(cf.KC_FLOW_ACCEPTED,)*2)
        self.assertEqual((gx.accepted,gy.accepted),(1,1))

    def test_forced_accept_at_limit_and_disjoint_counters(self):
        self.configure(1,9,3)
        core = self.core()
        for expected in (1,2):
            gx,gy = self.update(core)
            self.assertEqual((gx.consecutive,gy.consecutive),(expected,expected))
        before = self.state_bits(core)
        gx,gy = self.update(core)
        self.assertNotEqual(self.state_bits(core),before)
        for g in (gx,gy):
            self.assertEqual((g.decision,g.streakBefore,g.runAtDecision,g.consecutive),(cf.KC_FLOW_FORCED,2,3,0))
            self.assertEqual((g.accepted,g.rejected,g.forced),(0,2,1))
        gx,gy = self.update(core,0,0)
        self.assertEqual((gx.accepted,gx.rejected,gx.forced),(1,2,1))
        self.assertEqual((gy.accepted,gy.rejected,gy.forced),(1,2,1))

    def test_accepted_measurement_breaks_streak(self):
        self.configure(1)
        core = self.core()
        self.update(core)
        gx,_ = self.update(core,0,0)
        self.assertEqual((gx.streakBefore,gx.consecutive,gx.runAtDecision),(1,0,0))
        gx,_ = self.update(core)
        self.assertEqual((gx.streakBefore,gx.consecutive),(0,1))

    def test_runtime_switch_resets_streak_but_keeps_totals(self):
        self.configure(1)
        core = self.core()
        self.update(core)
        self.configure(0)
        gx,_ = self.update(core)
        self.assertEqual((gx.decision,gx.streakBefore,gx.consecutive,gx.accepted,gx.rejected),(0,0,0,1,1))
        self.configure(1,6.63,3)
        gx,_ = self.update(core)
        self.assertEqual((gx.consecutive,gx.rejected),(1,2))

    def test_limit_one_and_uint16_max(self):
        self.configure(1,9,1)
        gx,_ = self.update(self.core())
        self.assertEqual((gx.decision,gx.runAtDecision,gx.forced),(cf.KC_FLOW_FORCED,1,1))
        self.configure(1,9,65535)
        core = self.core()
        self.update(core)  # Cache config before preparing the boundary fixture.
        core.flowGateX.consecutive = 65534
        gx,_ = self.update(core)
        self.assertEqual((gx.decision,gx.runAtDecision,gx.consecutive),(cf.KC_FLOW_FORCED,65535,0))

    def test_core_reset_and_core_independence(self):
        self.configure(1)
        a,b = self.core(),self.core()
        self.update(a)
        self.assertEqual(cf.kalmanCoreGetFlowGateX(b).rejected,0)
        params=cf.kalmanCoreParams_t()
        cf.kalmanCoreDefaultParams(params)
        cf.kalmanCoreInit(a,params,100)
        g=cf.kalmanCoreGetFlowGateX(a)
        self.assertEqual((g.accepted,g.rejected,g.forced,g.consecutive),(0,0,0,0))

    def test_invalid_configuration_does_not_replace_valid_config(self):
        self.configure(1,9,3)
        for enabled,threshold,limit in ((2,9,3),(1,0,3),(1,float("nan"),3),(1,float("inf"),3),(1,9,0)):
            self.assertFalse(cf.kalmanCoreSetFlowGateConfig(enabled,threshold,limit))
        self.assertEqual((cf.kalmanCoreGetFlowGateEnabled(),cf.kalmanCoreGetFlowGateThreshold(),cf.kalmanCoreGetFlowGateMaxReject()),(1,9,3))
        for args in ((1,0,3),(1,9,65536),(1,9,1.5),(2,9,3)):
            with self.assertRaises(ValueError):
                FlowGateConfig(*args)

    def test_counter_saturates_instead_of_wrapping(self):
        core=self.core()
        core.flowGateX.accepted=2**32-1
        gx,_=self.update(core,0,0)
        self.assertEqual(gx.accepted,2**32-1)

    def test_fault_labels_cannot_influence_enabled_gate(self):
        suite=SensorSuite(noise=True,seed=7,flow_audit=True)
        cores=[FirmwareEKF(flow_gate=FlowGateConfig(1),flow_gate_log=True) for _ in range(2)]
        for ms in range(101):
            truth=dict(time_s=ms/1000,position_m=(0,0,1),orientation_wxyz=(1,0,0,0),
                       linear_velocity_m_s=(.2,0,0),angular_velocity_rad_s=(0,0,0))
            packets=suite.sample(truth)
            changed=[replace(p,flow_labels=dict(p.flow_labels,fault_type="spike",fault_active=True))
                     if p.kind=="flow" else p for p in packets]
            cores[0].step(ms,packets,True)
            cores[1].step(ms,changed,True)
            self.assertEqual(self.state_bits(cores[0].core),self.state_bits(cores[1].core))
            for a,b in zip(cores[0].flow_gate_rows,cores[1].flow_gate_rows):
                self.assertEqual((a["nis"],a["decision"],a["consecutive_rejections"]),
                                 (b["nis"],b["decision"],b["consecutive_rejections"]))

    def test_logs_actual_axis_updates_and_excludes_dropout(self):
        suite=SensorSuite(noise=True,seed=7,flow_fault=FlowFaultConfig("dropout",start_s=.02,duration_s=.02),flow_audit=True)
        ekf=FirmwareEKF(flow_gate=FlowGateConfig(1),flow_gate_log=True)
        logged=[]
        for ms in range(51):
            truth=dict(time_s=ms/1000,position_m=(0,0,1),orientation_wxyz=(1,0,0,0),
                       linear_velocity_m_s=(.2,0,0),angular_velocity_rad_s=(0,0,0))
            ekf.step(ms,suite.sample(truth),True)
            logged.extend(ekf.flow_gate_rows)
        self.assertEqual([(r["flow_sample_id"],r["axis"]) for r in logged],[(n,a) for n in (1,4,5) for a in "xy"])
        self.assertTrue(all(not r["fault_active"] for r in logged))
        for axis in "xy":
            g=[r for r in logged if r["axis"]==axis]
            for decision in ("accepted","rejected","forced"):
                self.assertEqual(g[-1][decision+"_total"],sum(r["decision"]==decision for r in g))


if __name__ == "__main__":
    unittest.main()
