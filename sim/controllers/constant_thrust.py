"""Constant-thrust controller for free-fall and feed-forward hover checks."""

import math

from .base import Controller


class ConstantThrustController(Controller):
    """No position or attitude feedback; return the configured thrust in N."""

    def __init__(self, total_thrust_N: float):
        if not math.isfinite(total_thrust_N) or total_thrust_N < 0:
            raise ValueError("total_thrust_N must be finite and nonnegative")
        self.total_thrust_N = float(total_thrust_N)

    def reset(self) -> None:
        # This controller has no history or integrator to reset.
        pass

    def compute(self, state: dict, target: dict) -> dict:
        return {"total_thrust_N": self.total_thrust_N}
