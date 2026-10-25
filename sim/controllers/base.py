"""Controller interface independent of Isaac Sim and the rendering loop."""

from abc import ABC, abstractmethod


class Controller(ABC):
    """Consume SI-unit state/target dictionaries and return actuator commands.

    State keys: time_s, position_m (world XYZ), orientation_wxyz (body-to-world
    quaternion), linear_velocity_m_s (world XYZ), angular_velocity_rad_s
    (world XYZ). The current actuator command is total_thrust_N, along body +Z.
    """

    @abstractmethod
    def reset(self) -> None:
        """Clear controller history when a simulation starts or resets."""

    @abstractmethod
    def compute(self, state: dict, target: dict) -> dict:
        """Return a command for the next physics step."""
