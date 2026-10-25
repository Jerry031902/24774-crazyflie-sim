"""Pure-Python altitude PID, with gravity feed-forward and conditional integration."""

import math

from .base import Controller


class AltitudePIDController(Controller):
    """Control only world Z for a near-level drone, using body +Z thrust.

    T = m * (g + kp * (z_target - z) + ki * integral(error) - kd * vz).
    Gains are in acceleration form: kp [1/s^2], ki [1/s^3], kd [1/s].
    Mass is in kg, dt_s is the interval between compute() calls, and the thrust
    limit is in N. Target position_m supplies a fixed altitude; X/Y are ignored.
    This controller does not stabilize attitude or compensate for body tilt.
    """

    def __init__(self, *, mass_kg, gravity_m_s2, dt_s, kp, ki, kd, max_thrust_N):
        for name, value in (("mass_kg", mass_kg), ("gravity_m_s2", gravity_m_s2),
                            ("dt_s", dt_s), ("max_thrust_N", max_thrust_N)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if any(not math.isfinite(gain) or gain < 0 for gain in (kp, ki, kd)):
            raise ValueError("PID gains must be finite and nonnegative")
        self.mass_kg = mass_kg
        self.gravity_m_s2 = gravity_m_s2
        self.dt_s = dt_s
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_thrust_N = max_thrust_N
        self.reset()

    def reset(self) -> None:
        self._integral_error_m_s = 0.0

    def compute(self, state: dict, target: dict) -> dict:
        z = float(state["position_m"][2])
        vz = float(state["linear_velocity_m_s"][2])
        target_z = float(target["position_m"][2])
        if not all(math.isfinite(value) for value in (z, vz, target_z)):
            raise ValueError("Altitude, vertical velocity and target must be finite")

        error = target_z - z
        candidate = self._integral_error_m_s + error * self.dt_s if self.ki else 0.0
        # Derivative on measurement avoids a D-term kick on target changes.
        base_acceleration = self.gravity_m_s2 + self.kp * error - self.kd * vz
        raw_thrust = self.mass_kg * (base_acceleration + self.ki * candidate)
        # Freeze integration only when the error would drive further into a limit.
        # Integration in the opposite direction remains enabled to unwind it.
        saturating_high = raw_thrust > self.max_thrust_N and error > 0
        saturating_low = raw_thrust < 0.0 and error < 0
        if not (saturating_high or saturating_low):
            self._integral_error_m_s = candidate
        raw_thrust = self.mass_kg * (base_acceleration + self.ki * self._integral_error_m_s)
        thrust = max(0.0, min(self.max_thrust_N, raw_thrust))
        return {"total_thrust_N": thrust}
