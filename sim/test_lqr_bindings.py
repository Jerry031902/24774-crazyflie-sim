"""Actual cffirmware boundary tests; run after rebuilding the bindings."""
import math
from pathlib import Path
import sys
import tempfile
import unittest
import csv
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
try:
    import cffirmware as cf
except ImportError:
    cf=None

@unittest.skipIf(cf is None,"Rebuild cffirmware first; these tests require real C bindings")
class LQRBindingTests(unittest.TestCase):
    @staticmethod
    def state(t=0,yaw=0):
        return dict(time_s=t,position_m=(0.,0.,1.),linear_velocity_m_s=(0.,0.,0.),
                    angular_velocity_rad_s=(0.,0.,0.),
                    orientation_wxyz=(math.cos(yaw/2),0.,0.,math.sin(yaw/2)))
    def test_factory_selects_lqr_and_retains_z_and_yaw(self):
        from run_sim import make_controller
        c=make_controller("firmware_lqr")
        out=c.compute(self.state(),dict(position_m=(.1,0,1),yaw_rad=.2))
        self.assertTrue(cf.horizontalLqrGetEnabled())
        self.assertEqual(c.setpoint.mode.z,cf.modeAbs)
        self.assertAlmostEqual(c.setpoint.attitude.yaw,math.degrees(.2),places=5)
        self.assertLess(out["horizontal_control"]["pitch_ref_deg"],0)
        self.assertGreater(out["control_legacy"][0],0)
    def test_body_velocity_interface_and_brake_priority(self):
        from controllers.firmware_pid import FirmwarePIDController
        c=FirmwarePIDController(horizontal_lqr=True)
        target=dict(position_m=(100,100,1),velocity_body_m_s=(.2,0),yaw_rate_rad_s=.1)
        out=c.compute(self.state(yaw=math.pi/2),target)
        self.assertEqual(out["horizontal_control"]["lqr_mode"],2)
        self.assertLess(out["horizontal_control"]["pitch_ref_deg"],0)
        self.assertAlmostEqual(out["horizontal_control"]["roll_ref_deg"],0,places=5)
        for i in range(1,11):
            out=c.compute(self.state(i*.001,yaw=math.pi/2),dict(target,brake=True))
        self.assertEqual(out["horizontal_control"]["lqr_mode"],3)
        self.assertEqual(c.setpoint.mode.z,cf.modeAbs)
        self.assertEqual(c.setpoint.attitudeRate.yaw,0)
        self.assertEqual(c.setpoint.velocity.x,0)
        self.assertAlmostEqual(out["horizontal_control"]["pitch_ref_deg"],0,places=5)
    def test_reset_and_baseline_selection(self):
        from controllers.firmware_pid import FirmwarePIDController
        c=FirmwarePIDController(horizontal_lqr=True)
        c.compute(self.state(),dict(position_m=(0,0,1),brake=True))
        c.reset()
        out=c.compute(self.state(),dict(position_m=(.1,0,1)))
        self.assertEqual(out["horizontal_control"]["lqr_mode"],1)
        c=FirmwarePIDController(horizontal_lqr=False)
        out=c.compute(self.state(),dict(position_m=(.1,0,1)))
        self.assertFalse(cf.horizontalLqrGetEnabled())
        self.assertEqual(out["horizontal_control"]["lqr_mode"],0)
    def test_ekf_lqr_uses_estimate_and_configured_delay(self):
        from run_sim import make_controller
        c=make_controller("ekf_lqr",noise=True,seed=7,delays_ms={"flow":20,"tof":20})
        self.assertEqual(c.mode,"ekf_closed_loop")
        self.assertEqual(c.suite.delays["flow"],20)
        # Ground truth XYZ is far from the initial estimator state.
        truth=self.state();truth["position_m"]=(8.,9.,1.)
        out=c.compute(truth,dict(position_m=(.1,0,1)))
        np.testing.assert_allclose(out["estimate_used"]["position_m"][:2],[0,0])
        self.assertEqual(out["horizontal_control"]["lqr_mode"],1)
        self.assertLess(abs(out["horizontal_control"]["pitch_ref_deg"]),2)
        self.assertEqual(out["horizontal_control"]["roll_ref_deg"],0)
    def test_flight_csv_contains_control_commands(self):
        from controllers.firmware_pid import FirmwarePIDController
        from actuators import MotorActuator
        from flight_demo import FlightLog
        c=FirmwarePIDController(horizontal_lqr=True)
        target=dict(position_m=(.1,0,1));state=self.state()
        out=c.compute(state,target)
        out.update(MotorActuator(None).convert(out["motor_commands"]))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"flight.csv";log=FlightLog(path)
            try: log.write(state,target,out,"hold")
            finally: log.close()
            with path.open() as file: row=next(csv.DictReader(file))
            self.assertEqual(int(row["lqr_enabled"]),1)
            self.assertEqual(int(row["lqr_mode"]),1)
            self.assertLess(float(row["pitch_ref_deg"]),0)

if __name__=="__main__":
    unittest.main()

