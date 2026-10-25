"""47-17 propeller actuator model for the firmware's default cf2_defconfig.

Source references are repository-relative, for the inspected firmware version.
This is an ideal, instantaneous force-command model: no battery, RPM lag or drag.
The default firmware command is linear in N, NOT raw PWM. Applying a second
PWM-to-RPM curve here would be incorrect (docs/functional-areas/crtp/
crtp_commander.md:29-36; src/drivers/src/motors.c:180-202).
"""

import math
from pathlib import Path
import re

import numpy as np

# src/platform/interface/platform_defaults_cf2.h:47,68-75, 2.1+ / 47-17 branch.
ARM_LENGTH_M = 0.046
THRUST_MIN_N = 0.012817578393224994
THRUST_MAX_N = 0.12  # Per motor, not the sum of four motors.
THRUST_TO_TORQUE_M = 0.0069928948992470565
COMMAND_MAX = 65535

# M1..M4 == indices 0..3: src/drivers/interface/motors.h:154-159.
# Derivation from the SI mixer, power_distribution_quadrotor.c:98-107:
# tau_x = y*Fz, tau_y = -x*Fz; matching its column signs gives these positions.
# Its 0.707106781*ARM_LENGTH gives each X/Y offset. Body frame is FLU.
_A = ARM_LENGTH_M / math.sqrt(2)
MOTOR_POSITIONS_BODY_M = np.array([[_A, -_A, 0], [-_A, -_A, 0],
                                  [-_A, _A, 0], [_A, _A, 0]], dtype=float)
# Planar z=0 represents the same vertical line of action as the propeller plane:
# translating a +Z force along Z does not change its moment. The visual Z is kept.
# Positive spin means CCW viewed from above. Reaction on body is opposite.
# Firmware SI mixer yaw coefficients are [-,+,-,+] (same source:102-107),
# implying spin [+,-,+,-]. Also confirmed by Bitcraze's 47-17 assembly guide:
# https://www.bitcraze.io/products/propellers-47-17/ (M1/M3 CCW, M2/M4 CW).
MOTOR_SPIN_DIRECTIONS = np.array([1, -1, 1, -1], dtype=int)
BODY_YAW_SIGNS = -MOTOR_SPIN_DIRECTIONS


def validate_firmware_config(cf):
    """Refuse a mismatched build instead of silently using the wrong force curve."""
    config = Path(__file__).resolve().parents[1] / "build/include/generated/autoconf.h"
    if not config.is_file():
        raise RuntimeError("Run make cf2_defconfig and make bindings_python first")
    text = config.read_text()
    for flag in ("CONFIG_PLATFORM_CF2", "CONFIG_CRAZYFLIE_21_PLUS", "CONFIG_ENABLE_THRUST_BAT_COMPENSATED"):
        if not re.search(rf"^#define {flag} 1$", text, flags=re.MULTILINE):
            raise RuntimeError(f"47-17 actuator model requires {flag}=1; rebuild bindings")
    if not math.isclose(cf.powerDistributionGetMaxThrust(), 4 * THRUST_MAX_N, rel_tol=1e-6):
        raise RuntimeError("Compiled firmware THRUST_MAX differs from the actuator model")


class MotorActuator:
    """Apply four point forces and four yaw reaction torques to ONE rigid body."""

    def __init__(self, body):
        import cffirmware
        self.cf, self.body = cffirmware, body
        validate_firmware_config(cffirmware)
        self._raw = cffirmware.motors_thrust_uncapped_t()
        self._capped = cffirmware.motors_thrust_pwm_t()

    def convert(self, motor_commands):
        commands = np.asarray(motor_commands, dtype=float)
        if commands.shape != (4,) or not np.isfinite(commands).all():
            raise ValueError("Expected four finite motor commands in M1..M4 order")
        if np.any(commands != np.trunc(commands)) or np.any(np.abs(commands) > 2**31 - 1):
            raise ValueError("Firmware motor commands must be int32 integers")
        for i, command in enumerate(commands, 1):
            setattr(self._raw.motors, f"m{i}", int(command))
        # Reuse firmware desaturation: subtract common excess, then floor at idle.
        # In hardware this operates after battery compensation; here it bounds
        # ideal FORCE commands, with no voltage/PWM stage (source:160-198).
        capped = self.cf.powerDistributionCap(self._raw, self._capped)
        applied = np.array([getattr(self._capped.motors, f"m{i}") for i in range(1, 5)], dtype=np.int32)
        forces = applied.astype(float) * (THRUST_MAX_N / COMMAND_MAX)
        # motors.c:198-202 rejects thrust below THRUST_MIN in the compensated path.
        forces[forces < THRUST_MIN_N] = 0
        yaw_torques = BODY_YAW_SIGNS * THRUST_TO_TORQUE_M * forces
        return {"motor_commands_applied": applied, "motor_forces_N": forces,
                "motor_yaw_torques_Nm": yaw_torques,
                "total_thrust_N": float(forces.sum()),
                "saturated": bool(capped or np.any(applied != commands))}

    def apply(self, motor_commands):
        if self.body is None or not self.body.is_physics_handle_valid():
            raise RuntimeError("Initialize the rigid body before applying motor forces")
        result = self.convert(motor_commands)
        for position, force, torque in zip(MOTOR_POSITIONS_BODY_M, result["motor_forces_N"], result["motor_yaw_torques_Nm"]):
            # These calls accumulate until the next physics step, checked by the
            # equal-motor hover test. PhysX produces r x F; never add it here.
            self.body.apply_forces_and_torques_at_pos(
                forces=np.array([[0, 0, force]], dtype=np.float32),
                torques=np.array([[0, 0, torque]], dtype=np.float32),
                positions=np.array([position], dtype=np.float32),
                is_global=False,
            )
        return result
