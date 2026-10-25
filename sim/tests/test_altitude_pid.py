"""Run without Isaac Sim: python3 -m unittest discover -s sim/tests -v."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controllers.altitude_pid import AltitudePIDController


class AltitudePIDTests(unittest.TestCase):
    def make_controller(self):
        return AltitudePIDController(mass_kg=.03, gravity_m_s2=9.81, dt_s=.001,
                                     kp=6, ki=.2, kd=4, max_thrust_N=.5886)

    @staticmethod
    def state(z=1.0, vz=0.0):
        return {"position_m": (0, 0, z), "linear_velocity_m_s": (0, 0, vz)}

    def test_long_saturation_does_not_leave_integral_windup(self):
        # A blocked/infeasible target must not leave a thrust bias when released.
        for target_z, limit in ((100.0, .5886), (-100.0, 0.0)):
            with self.subTest(target_z=target_z):
                controller = self.make_controller()
                for _ in range(5000):
                    command = controller.compute(self.state(), {"position_m": (0, 0, target_z)})
                    self.assertEqual(command["total_thrust_N"], limit)
                released = controller.compute(self.state(), {"position_m": (0, 0, 1.0)})
                self.assertAlmostEqual(released["total_thrust_N"], .03 * 9.81)

    def test_reset_clears_accumulated_error(self):
        controller = self.make_controller()
        for _ in range(1000):
            controller.compute(self.state(), {"position_m": (0, 0, 1.1)})
        biased = controller.compute(self.state(), {"position_m": (0, 0, 1.0)})
        self.assertGreater(biased["total_thrust_N"], .03 * 9.81)
        controller.reset()
        fresh = self.make_controller().compute(self.state(), {"position_m": (0, 0, 1.0)})
        self.assertEqual(controller.compute(self.state(), {"position_m": (0, 0, 1.0)}), fresh)

    def test_nonfinite_state_cannot_reach_force_interface(self):
        controller = self.make_controller()
        for state, target in (
            (self.state(z=float("nan")), 1.0),
            (self.state(vz=float("inf")), 1.0),
            (self.state(), float("nan")),
        ):
            with self.assertRaises(ValueError):
                controller.compute(state, {"position_m": (0, 0, target)})


if __name__ == "__main__":
    unittest.main()
