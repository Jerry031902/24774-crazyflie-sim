"""Boundary tests: run with Isaac Sim Python, no SimulationApp/GPU needed."""

import math
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
    from actuators import MotorActuator, THRUST_MIN_N
    from controllers.firmware_pid import FirmwarePIDController, FirmwareGyroFilter


@unittest.skipIf(cf is None, "Build cffirmware and use Isaac Sim Python 3.11")
class FirmwarePIDTests(unittest.TestCase):
    @staticmethod
    def state(t=0, z=1, q=(1, 0, 0, 0), omega=(0, 0, 0), velocity=(0, 0, 0)):
        return {"time_s": t, "position_m": (0, 0, z), "orientation_wxyz": q,
                "angular_velocity_rad_s": omega, "linear_velocity_m_s": velocity}

    def test_world_gyro_to_body_and_specific_force(self):
        c = FirmwarePIDController()
        q = (math.sqrt(.5), 0, 0, math.sqrt(.5))  # +90 deg yaw.
        c.compute(self.state(q=q, omega=(1, 2, 3)), {"position_m": (0, 0, 1)})
        np.testing.assert_allclose([c.sensors.gyro.x, c.sensors.gyro.y, c.sensors.gyro.z],
                                   np.degrees([2, -1, 3]), rtol=1e-6)
        np.testing.assert_allclose([c.sensors.acc.x, c.sensors.acc.y, c.sensors.acc.z], [0, 0, 1])
        c.compute(self.state(t=.001, q=q, velocity=(0, 0, -.00981)), {"position_m": (0, 0, 1)})
        self.assertAlmostEqual(c.sensors.acc.z, 0, places=5)  # Free fall: zero specific force.
        self.assertAlmostEqual(c.firmware_state.acc.z, -1, places=5)

    def test_legacy_pitch_inverted_but_quaternion_and_gyro_are_not(self):
        c = FirmwarePIDController()
        q = (math.cos(math.pi/12), 0, math.sin(math.pi/12), 0)
        c.compute(self.state(q=q, omega=(0, 1, 0)), {"position_m": (0, 0, 1)})
        self.assertAlmostEqual(c.firmware_state.attitude.pitch, -30, places=4)
        self.assertGreater(c.firmware_state.attitudeQuaternion.y, 0)
        self.assertAlmostEqual(c.sensors.gyro.y, math.degrees(1), places=4)

    def test_c_rate_gates_and_persistent_control(self):
        c = FirmwarePIDController()
        target = {"position_m": (0, 0, 1)}
        first = c.compute(self.state(), target)
        for i in range(1, 10):
            value = c.compute(self.state(i*.001, z=.8), target)
            self.assertEqual(value["control_legacy"][0], first["control_legacy"][0])
        value = c.compute(self.state(.010, z=.8), target)
        self.assertGreater(value["control_legacy"][0], first["control_legacy"][0])
        c.reset()
        manual = {"attitude_legacy_deg": (0, 0, 0), "collective_command": 40000}
        c.compute(self.state(), manual)
        manual["attitude_legacy_deg"] = (2, 0, 0)
        odd = c.compute(self.state(.001), manual)
        even = c.compute(self.state(.002), manual)
        self.assertEqual(odd["control_legacy"][1], 0)
        self.assertGreater(even["control_legacy"][1], 0)
        with self.assertRaises(ValueError):
            c.compute(self.state(.005), manual)  # Missed physics ticks must not silently change gains.

    def test_reset_clears_c_history(self):
        c = FirmwarePIDController()
        target = {"position_m": (0, 0, 1)}
        initial = c.compute(self.state(z=.1), target)["motor_commands"].copy()
        for i in range(1, 100):
            c.compute(self.state(i*.001, z=.3, omega=(1, 2, 3)), target)
        c.reset()
        np.testing.assert_array_equal(c.compute(self.state(z=.1), target)["motor_commands"], initial)

    def test_firmware_gyro_filter_dc_and_250_hz_rejection(self):
        f = FirmwareGyroFilter(1000, 80)
        np.testing.assert_allclose(f.apply([10, -20, 30]), [10, -20, 30], rtol=2e-6)
        for _ in range(200):
            value = f.apply([20, -40, 60])
        np.testing.assert_allclose(value, [20, -40, 60], rtol=2e-6)
        outputs = []
        for i in range(1000):
            outputs.append(f.apply([math.sin(2*math.pi*250*i/1000), 0, 0])[0])
        # The observed alternating 500 Hz rate-loop output is a 250 Hz signal.
        # The firmware's 80 Hz 2-pole filter must attenuate it by >90%.
        self.assertLess(np.sqrt(np.mean(np.square(outputs[200:])))/math.sqrt(.5), .1)


    def test_actuator_desaturation_cutoff_and_invalid_input(self):
        actuator = MotorActuator(None)
        result = actuator.convert([65540, 65545, 65550, 65535])
        np.testing.assert_array_equal(result["motor_commands_applied"], [65525, 65530, 65535, 65520])
        self.assertTrue(result["saturated"])
        result = actuator.convert([0, -100, 1000, 65535])
        np.testing.assert_allclose(result["motor_forces_N"], [0, 0, 0, .12])
        threshold = THRUST_MIN_N / .12 * 65535
        result = actuator.convert([math.ceil(threshold)-1, math.ceil(threshold), 0, 0])
        self.assertEqual(result["motor_forces_N"][0], 0)
        self.assertGreaterEqual(result["motor_forces_N"][1], THRUST_MIN_N)
        for commands in ([1, 2, 3], [1, 2, 3, float("nan")], [1, 2, 3, .5]):
            with self.assertRaises(ValueError):
                actuator.convert(commands)


if __name__ == "__main__":
    unittest.main()
