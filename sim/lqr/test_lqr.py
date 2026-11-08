"""Numerical and actual C integration tests; no drone, radio or GPU."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from design import design
from native import NativeController

class DesignTests(unittest.TestCase):
    def test_zoh_matches_independent_matrix_exponential(self):
        from scipy.linalg import expm, solve_discrete_are
        for beta in (0., .5, 1.):
            d=design(beta)
            ac=np.zeros((4,4)); ac[0,2]=ac[1,3]=1; ac[2,2]=ac[3,3]=-beta
            bc=np.array([[0,0],[0,0],[0,-9.81],[-9.81,0]])
            aug=np.zeros((6,6)); aug[:4,:4]=ac; aug[:4,4:]=bc
            exact=expm(aug*.01)
            np.testing.assert_allclose(d["Ad"],exact[:4,:4],atol=1e-12)
            np.testing.assert_allclose(d["Bd"],exact[:4,4:],atol=1e-12)
            a,b,q,r=map(np.array,(d["Ad"],d["Bd"],d["Q"],d["R"]))
            p=solve_discrete_are(a,b,q,r)
            np.testing.assert_allclose(d["K"],np.linalg.solve(r+b.T@p@b,b.T@p@a),atol=1e-9)

class FirmwareTests(unittest.TestCase):
    def s(self, **kwargs):
        fields=dict(x=0,y=0,z=1,vx=0,vy=0,vz=0,roll=0,pitch=0,yaw=0,gx=0,gy=0,gz=0)
        fields.update(kwargs); return list(fields.values())
    def test_position_signs_and_yaw_rotation(self):
        c=NativeController(True)
        a=c.step(self.s(),[.1,0,1,0,0,0])
        self.assertLess(a[1],0); self.assertAlmostEqual(a[0],0)
        c=NativeController(True)
        b=c.step(self.s(yaw=90),[.1,0,1,0,0,90])
        self.assertGreater(b[0],0); self.assertAlmostEqual(b[1],0,places=5)
        self.assertAlmostEqual(a[1],-b[0],places=5)
    def test_matches_design_gain_at_zero_yaw(self):
        d=design(); c=NativeController(True)
        s=self.s(x=.1,y=-.2,vx=.03,vy=-.05)
        out=c.step(s,[0,0,1,0,0,0])
        expected=-np.array(d["K"])@np.array([s[0],s[1],s[3],s[4]])
        np.testing.assert_allclose(out[:2],np.degrees(expected),atol=1e-5)
    def test_altitude_response_is_identical_to_pid(self):
        outputs=[]
        for lqr in (False,True):
            c=NativeController(lqr)
            outputs.append([c.step(self.s(z=.8),[.1,0,1,0,0,0],tick=i)[2] for i in range(31)])
        np.testing.assert_array_equal(*outputs)
        self.assertGreater(outputs[1][0],0)
    def test_velocity_ignores_stale_waypoint_and_handles_body_yaw(self):
        outputs=[]
        for p in (0,100):
            c=NativeController(True)
            outputs.append(c.step(self.s(yaw=90),[p,p,1,.1,0,90],velocity=True,body=True))
        np.testing.assert_array_equal(*outputs)
        self.assertLess(outputs[0][1],0)
        self.assertAlmostEqual(outputs[0][0],0,places=5)
        self.assertEqual(outputs[0][6],2)
    def test_brake_overrides_position_target(self):
        c=NativeController(True)
        out=c.step(self.s(vx=.3),[100,100,1,0,0,0],brake=True)
        self.assertGreater(out[1],0); self.assertEqual(out[6],3)
        self.assertAlmostEqual(out[0],0)
    def test_stop_is_not_overridden(self):
        c=NativeController(True)
        out=c.step(self.s(),[100,100,1,0,0,0],stop=True,brake=True)
        np.testing.assert_array_equal(out[2:6],0)
        np.testing.assert_array_equal(out[9:],0)
        self.assertEqual(out[6],0)
    def test_limits_rate_gate_and_mode_selection(self):
        c=NativeController(True)
        out=c.step(self.s(),[100,-100,1,0,0,0])
        np.testing.assert_allclose(abs(out[:2]),10,atol=1e-5)
        self.assertEqual(out[7],1); self.assertEqual(out[8],1)
        for tick in range(1,10):
            out=c.step(self.s(),[0,0,1,0,0,0],tick=tick)
            self.assertEqual(out[8],1)
        out=c.step(self.s(),[0,0,1,0,0,0],tick=10)
        self.assertEqual(out[8],2)
        np.testing.assert_allclose(out[:2],0,atol=1e-6)
        c.lib.bridgeEnable(0)
        out=c.step(self.s(),[.1,0,1,0,0,0],tick=20)
        self.assertEqual(out[6],0); self.assertLess(out[1],0)
    def test_invalid_horizontal_reference_neutralizes_angles(self):
        c=NativeController(True)
        out=c.step(self.s(),[float("nan"),0,1,0,0,0])
        self.assertEqual(out[6],4)
        np.testing.assert_allclose(out[:2],0)

if __name__ == "__main__":
    unittest.main()

