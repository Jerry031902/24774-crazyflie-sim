"""Target sequence and CSV recording shared by GUI and headless firmware runs."""

import csv
from pathlib import Path

import numpy as np

from controllers.firmware_pid import rotation_and_rpy

HOVER_HOLD_S = 10.0
MOVE_HOLD_S = 3.0
POSITION_TOLERANCE_M = 0.03
SPEED_TOLERANCE_M_S = 0.05

# Matched schedule for all EKF comparisons. No ground-truth-dependent waypoint
# transitions and no reset of the hold timer by individual noisy estimates.
EKF_TAKEOFF_SETTLE_S = 10.0
EKF_MOVE_SETTLE_S = 10.0


class TimedHoverMoveDemo:
    """10 s takeoff/settle, 10 s hover, 10 s move/settle, then hold final target."""

    def __init__(self, target_z_m=1.0):
        self.target_z_m = target_z_m
        self.reset()

    def reset(self):
        self.phase = "takeoff"
        self.done = False
        self.hover_start_s = EKF_TAKEOFF_SETTLE_S
        self.move_time_s = self.hover_start_s + HOVER_HOLD_S
        self.end_time_s = self.move_time_s + EKF_MOVE_SETTLE_S

    def target(self, state):
        # Only the clock is used; neither true nor estimated position controls
        # the trajectory schedule. Actual stability is evaluated separately.
        now = state["time_s"]
        self.done = now >= self.end_time_s
        self.phase = ("complete" if self.done else "move" if now >= self.move_time_s
                      else "hover" if now >= self.hover_start_s else "takeoff")
        return {"position_m": (0.5 if now >= self.move_time_s else 0., 0., self.target_z_m), "yaw_rad": 0.}


class HoverMoveDemo:
    """Reach (0,0,z), hold continuously for 10 s, then reach (0.5,0,z)."""

    def __init__(self, target_z_m=1.0):
        self.target_z_m = target_z_m
        self.reset()

    def reset(self):
        self.phase = "takeoff"
        self.stable_since = None
        self.move_time_s = None
        self.hover_start_s = None
        self.done = False

    def target(self, state):
        now = state["time_s"]
        position = (0.5 if self.phase in ("move", "complete") else 0.0, 0.0, self.target_z_m)
        settled = (np.linalg.norm(np.asarray(state["position_m"]) - position) < POSITION_TOLERANCE_M
                   and np.linalg.norm(state["linear_velocity_m_s"]) < SPEED_TOLERANCE_M_S)
        if self.phase in ("takeoff", "hover"):
            if settled:
                if self.stable_since is None:
                    self.stable_since = now
                    self.hover_start_s = now
                self.phase = "hover"
                if now - self.stable_since >= HOVER_HOLD_S:
                    self.phase = "move"
                    self.move_time_s = now
                    self.stable_since = None
                    position = (0.5, 0.0, self.target_z_m)
            else:
                self.stable_since = None
                self.phase = "takeoff"
        elif self.phase == "move":
            if settled:
                if self.stable_since is None:
                    self.stable_since = now
                if now - self.stable_since >= MOVE_HOLD_S:
                    self.phase, self.done = "complete", True
            else:
                self.stable_since = None
        return {"position_m": position, "yaw_rad": 0.0}


class FlightLog:
    """Record every 1 ms pre-step state and the command applied for that interval."""

    def __init__(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("w", newline="")
        self.writer = None
        self.measurement_path = path.with_suffix(".measurements.csv")
        self.measurements = None
        self.measurement_writer = None

    def write(self, state, target, command, phase, extra=None):
        """extra: optional dict of additional columns, same keys on every row."""
        _, rpy = rotation_and_rpy(state["orientation_wxyz"])
        attitude = np.degrees(rpy) * [1, -1, 1]
        row = {"time_s": state["time_s"], "phase": phase, "tick": command["tick"]}
        for axis, value, goal, velocity, omega in zip("xyz", state["position_m"], target["position_m"], state["linear_velocity_m_s"], state["angular_velocity_rad_s"]):
            row[f"{axis}_m"], row[f"target_{axis}_m"] = value, goal
            row[f"v{axis}_m_s"], row[f"omega_world_{axis}_rad_s"] = velocity, omega
        for name, value in zip(("qw", "qx", "qy", "qz"), state["orientation_wxyz"]):
            row[name] = value
        for name, value in zip(("roll_deg", "pitch_legacy_deg", "yaw_deg"), attitude):
            row[name] = value
        for name, value in zip(("control_thrust", "control_roll", "control_pitch", "control_yaw"), command["control_legacy"]):
            row[name] = value
        for axis, value in zip("xyz", command["gyro_body_deg_s"]):
            row[f"gyro_filtered_body_{axis}_deg_s"] = value
        for i in range(4):
            row[f"m{i+1}_command"] = command["motor_commands"][i]
            row[f"m{i+1}_applied_command"] = command["motor_commands_applied"][i]
            row[f"m{i+1}_force_N"] = command["motor_forces_N"][i]
        row["total_thrust_N"] = command["total_thrust_N"]
        row["yaw_torque_Nm"] = sum(command["motor_yaw_torques_Nm"])
        row["saturated"] = int(command["saturated"])
        if "estimate" in command:
            est = command["estimate"]
            for prefix, data in (("est", est), ("controller_est", command["estimate_used"])):
                for axis, p, v, angle in zip("xyz", data["position_m"], data["linear_velocity_m_s"], data["attitude_legacy_deg"]):
                    row[f"{prefix}_{axis}_m"] = p
                    row[f"{prefix}_v{axis}_m_s"] = v
                    row[f"{prefix}_{dict(x='roll', y='pitch_legacy', z='yaw')[axis]}_deg"] = angle
                for axis, value in zip("wxyz", data["orientation_wxyz"]):
                    row[f"{prefix}_q{axis}"] = value
            q1, q2 = np.asarray(state["orientation_wxyz"], dtype=float), est["orientation_wxyz"]
            cosine = abs(np.dot(q1, q2)/(np.linalg.norm(q1)*np.linalg.norm(q2)))
            row["attitude_error_deg"] = np.degrees(2*np.arccos(np.clip(cosine, 0, 1)))
            for axis, acc, gyro in zip("xyz", command["imu_acc_G"], command["imu_gyro_deg_s"]):
                row[f"imu_acc_{axis}_G"], row[f"imu_gyro_{axis}_deg_s"] = acc, gyro
            row["ekf_reset_count"] = command["ekf_reset_count"]
            row["quad_is_flying"] = int(command["quad_is_flying"])
            for innovation in command["ekf_innovations"]:
                if self.measurements is None:
                    self.measurements = self.measurement_path.open("w", newline="")
                    self.measurement_writer = csv.DictWriter(self.measurements, fieldnames=list(innovation))
                    self.measurement_writer.writeheader()
                self.measurement_writer.writerow(innovation)
        if extra:
            row.update(extra)
        if self.writer is None:
            self.writer = csv.DictWriter(self.file, fieldnames=list(row))
            self.writer.writeheader()
        self.writer.writerow(row)

    def close(self):
        self.file.close()
        if self.measurements is not None:
            self.measurements.close()
