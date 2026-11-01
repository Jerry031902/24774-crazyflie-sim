"""GO_TO_GOAL / AVOID state machine using WedgeBug sweeps.

Line-by-line port of matlab-trajectory-sim/src/nav/navigator_init.m and
navigator_step.m; see that folder's README for the algorithm. Inputs and
outputs are the Crazyflie hover setpoint without height:

    u_body, yaw_rate = nav.step(x, psi, ranges, t)

    x        [px, py, vx, vy], world frame [m, m/s]
    psi      yaw [rad], counter-clockwise from +x
    ranges   [front, left, back, right] Multi-ranger readings [m], inf = no return
    u_body   commanded velocity [forward, left] in the body frame [m/s]
    yaw_rate commanded yaw rate [rad/s], counter-clockwise positive

Differences from MATLAB: target indices (wp, unreachable, events) are
0-based, and directions keep MATLAB's 1 forward, 2 left, 3 back, 4 right,
0 straight at the point.
"""

from dataclasses import dataclass, field
import math

import numpy as np

from .geometry import rot2d, wrap_to_pi

MODES = ("GO_TO_GOAL", "AVOID", "DONE")
PHASES = ("sweep", "pass", "align", "follow", "overshoot")
# Unit vectors of directions 1..4 (forward, left, back, right) in the body frame.
DIRS = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]])


@dataclass
class _Command:
    dir: int
    speed: float
    yaw_rate: float
    u: np.ndarray = field(default_factory=lambda: np.zeros(2))


def _toward(d, hand):
    """Direction of the wall side: left for hand = +1, right for hand = -1."""
    return (d + (2 if hand < 0 else 0)) % 4 + 1


def _away(d, hand):
    """Direction away from the wall: right for hand = +1, left for hand = -1."""
    return (d + (2 if hand > 0 else 0)) % 4 + 1


def _round_half_away(x):
    return math.copysign(math.floor(abs(x) + 0.5), x)


def _segment_distance(pts, a, b):
    """Distance from each row of pts to the segment a-b."""
    ab = b - a
    denom = float(ab @ ab)
    s = np.zeros(len(pts)) if denom == 0 else np.clip(((pts - a) @ ab) / denom, 0, 1)
    return np.linalg.norm(pts - (a + s[:, None] * ab), axis=1)


class WedgeBugNavigator:
    """Reactive navigator: fly from start through targets (waypoints, then the goal)."""

    def __init__(self, start, targets, p):
        self.p = p
        self.targets = np.atleast_2d(np.asarray(targets, dtype=float))
        self.wp = 0                    # index of the current target
        self.mode = "GO_TO_GOAL"       # 'GO_TO_GOAL' | 'AVOID' | 'DONE'
        self.phase = ""                # AVOID: 'sweep' | 'pass' | 'align' | 'follow' | 'overshoot'

        # Motion
        self.facing = False            # lined up with the point being flown to
        self.exec_dir = 1              # direction being flown
        self.last_pos = np.asarray(start, dtype=float).copy()

        # Avoidance episode
        self.episode = False
        self.hit_point = np.full(2, np.nan)
        self.hit_dist = math.inf
        self.side = 0                  # way round chosen by the first sweep: +1 left, -1 right
        self.ep_travel = 0.0
        self.sweep_spots = []          # [x, y, ep_travel] at each sweep of the episode
        self.winding = 0.0
        self.last_bearing = 0.0

        # Sweep
        self.sweep_stage = ""          # 'stop' | 'to_start' | 'scan'
        self.sweep_center = 0.0
        self.scan = {"t": [], "ang": [], "rng": []}
        self.subgoal = np.full(2, np.nan)

        # Wall-following
        self.hand = 1                  # +1 wall on the left, -1 wall on the right
        self.travel_dir = 1
        self.psi_hold = 0.0
        self.following = False
        self.loop_point = np.full(2, np.nan)
        self.travel = 0.0
        self.since_turn = 0.0
        self.wall_seen = False
        self.overshoot_left = 0.0
        self.d_followed = math.inf

        self.unreachable = []
        self.events = []               # dicts: t, type, pos, wp
        self.sweeps = []               # dicts: t, ang, rng, pos, center, edge, seen, cost, side, pass

    # ------------------------------------------------------------------
    def target(self):
        return self.targets[self.wp]

    def step(self, x, psi, ranges, t):
        p = self.p
        x = np.asarray(x, dtype=float)
        ranges = np.asarray(ranges, dtype=float)
        pos = x[0:2].copy()
        speed = float(np.linalg.norm(x[2:4]))
        moved = float(np.linalg.norm(pos - self.last_pos))
        self.last_pos = pos
        if self.episode:
            self.ep_travel += moved
            # Circling the target more than max_winding without getting closer: walled in.
            d = pos - self.target()
            bearing = math.atan2(d[1], d[0])
            self.winding += float(wrap_to_pi(bearing - self.last_bearing))
            self.last_bearing = bearing
            if abs(self.winding) > p.nav.max_winding:
                self._log_event(t, "unreachable", pos)
                self.unreachable.append(self.wp)
                self._next_target()

        if self.mode != "DONE" and np.linalg.norm(self.target() - pos) < p.nav.wp_tol:
            self._log_event(t, "reached", pos)
            self._next_target()

        # A mode or phase switch is handled within the same tick.
        cmd = _Command(self.exec_dir, 0.0, 0.0)
        for _ in range(4):
            if self.mode == "DONE":
                return np.zeros(2), 0.0
            if self.mode == "GO_TO_GOAL":
                cmd, switched = self._go_to_goal(pos, psi, speed, ranges, t)
            else:
                cmd, switched = self._avoid(pos, psi, speed, ranges, moved, t)
            if not switched:
                break

        # Fly the command, braking first if it changes direction.
        if cmd.dir == 0:
            self.exec_dir = 0
            return cmd.u, cmd.yaw_rate
        if cmd.dir != self.exec_dir:
            if speed > p.nav.v_turn:
                return np.zeros(2), 0.0
            self.exec_dir = cmd.dir
        return cmd.speed * DIRS[cmd.dir - 1], cmd.yaw_rate

    # ==================================================================
    def _go_to_goal(self, pos, psi, speed, ranges, t):
        target = self.target()
        if self.episode and np.linalg.norm(target - pos) < self.hit_dist - self.p.nav.leave_margin:
            self._end_episode()   # past the obstacle: the next one is a fresh start
        cmd, event = self._fly_to(pos, psi, speed, ranges, target, self.p.nav.d_sweep)
        if event == "blocked":
            self._start_avoid(pos, t)
            self._start_sweep(psi)
        elif event == "side_left":
            self._start_avoid(pos, t)
            self._start_follow(psi, +1)
        elif event == "side_right":
            self._start_avoid(pos, t)
            self._start_follow(psi, -1)
        return cmd, bool(event)

    def _fly_to(self, pos, psi, speed, ranges, point, d_block):
        """Face point, then fly straight at it nose first.

        event: 'blocked' (obstacle ahead, short of the point, within d_block),
        'side_left' / 'side_right' (a side ranger below d_side), or ''.
        """
        nav = self.p.nav
        d = point - pos
        dist = float(np.linalg.norm(d))

        if dist <= nav.yaw_free_radius:
            # The last few centimetres: slide straight onto the point, no turning.
            cmd = _Command(0, 0.0, 0.0)
            if dist > 0:
                cmd.u = min(nav.v_cruise, nav.k_stop * dist) * (rot2d(psi).T @ d) / dist
            return cmd, ""

        err = float(wrap_to_pi(math.atan2(d[1], d[0]) - psi))
        if abs(err) > nav.yaw_tol:
            self.facing = False
        elif abs(err) < math.radians(2):
            self.facing = True
        if not self.facing:
            # Not facing the point: stop, then turn on the spot until lined up.
            return _Command(1, 0.0, (speed < nav.v_turn) * self._yaw_command(err)), ""

        front = ranges[0]
        short_of_point = front - nav.d_wall < dist - 0.02
        if short_of_point and front <= d_block + 0.02:
            return _Command(1, 0.0, 0.0), "blocked"
        if ranges[1] < nav.d_side:
            return _Command(1, 0.0, 0.0), "side_left"
        if ranges[3] < nav.d_side:
            return _Command(1, 0.0, 0.0), "side_right"

        stop_at = d_block if short_of_point else nav.d_wall   # come to rest where it will stop and look
        speed_cmd = min(nav.v_cruise, nav.k_stop * dist, nav.k_stop * max(front - stop_at, 0.0))
        return _Command(1, speed_cmd, self._yaw_command(err)), ""   # keep the nose on the point

    # ==================================================================
    def _avoid(self, pos, psi, speed, ranges, moved, t):
        if self.phase == "sweep":
            return self._sweep(pos, psi, speed, ranges, t)
        if self.phase == "pass":
            return self._pass_edge(pos, psi, speed, ranges, t)
        return self._follow(pos, psi, speed, ranges, moved, t)

    # ----- sweep: hover and scan the wedge ------------------------------
    def _sweep(self, pos, psi, speed, ranges, t):
        nav = self.p.nav
        half = nav.sweep_half
        cmd = _Command(self.exec_dir, 0.0, 0.0)

        if self.sweep_stage == "stop":
            if speed >= nav.v_turn:
                return cmd, False                      # hover first
            # Back at an earlier sweep spot without having got closer, or
            # swept too often: the target cannot be reached.
            if self.sweep_spots:
                spots = np.asarray(self.sweep_spots)
                again = ((np.linalg.norm(spots[:, 0:2] - pos, axis=1) < 2 * nav.loop_tol)
                         & (self.ep_travel - spots[:, 2] > nav.loop_min_travel))
                if again.any() or len(spots) >= nav.max_sweeps:
                    return cmd, self._give_up(pos, t)
            self.sweep_stage = "to_start"

        elif self.sweep_stage == "to_start":           # turn to the right end of the wedge
            err = float(wrap_to_pi(self.sweep_center - half - psi))
            cmd = _Command(self.exec_dir, 0.0, self._yaw_command(err))
            if abs(err) < math.radians(1):
                self.sweep_stage = "scan"
                self.scan = {"t": [], "ang": [], "rng": []}

        elif self.sweep_stage == "scan":
            # Sweep to the left end, logging front, left and right: the sides
            # sit 90 deg off the nose, so a +/-45 deg sweep sees +/-135 deg.
            self.scan["t"] += [t, t, t]
            self.scan["ang"] += [psi, psi + math.pi / 2, psi - math.pi / 2]
            self.scan["rng"] += [ranges[0], ranges[1], ranges[3]]
            if wrap_to_pi(psi - (self.sweep_center - half)) < 2 * half - 1e-3:
                return _Command(self.exec_dir, 0.0, nav.sweep_rate), False
            return cmd, self._decide(pos, t)
        return cmd, False

    def _decide(self, pos, t):
        """Turn the wedge into a way round the obstacle."""
        target = self.target()
        w = self._read_wedge(pos, target)
        if not w["blocked"]:
            self.mode = "GO_TO_GOAL"                   # nothing in the way after all
            self.facing = False
            return True
        if self.side == 0:
            self.side = 1 - 2 * int(np.argmin(w["cost"]))   # +1 = round the left, -1 = round the right
        i = (1 - self.side) // 2                       # row of w.edge for that side

        pass_pt = np.full(2, np.nan)
        if w["seen"][i]:
            pass_pt = self._choose_pass_point(w["edge"][i], pos, target, self.side, w["outline"])
        self.sweeps.append({"t": list(self.scan["t"]), "ang": list(self.scan["ang"]),
                            "rng": list(self.scan["rng"]), "pos": pos.copy(),
                            "center": self.sweep_center, "edge": w["edge"].copy(),
                            "seen": list(w["seen"]), "cost": list(w["cost"]),
                            "side": self.side, "pass": pass_pt})
        self.sweep_spots.append([pos[0], pos[1], self.ep_travel])

        if w["seen"][i]:
            self.subgoal = pass_pt
            self.phase = "pass"
            self.facing = False
        else:
            # Round the left = wall on the right. Square up from the heading
            # the sweep looked along, not from where the sweep ended.
            self._start_follow(self.sweep_center, -self.side)
        return True

    def _choose_pass_point(self, E, pos, target, side, outline):
        """Point outside edge E, moved out until the next leg clears the outline."""
        nav = self.p.nav
        ray = (E - pos) / np.linalg.norm(E - pos)
        out = side * np.array([-ray[1], ray[0]])
        P = E + nav.pass_margin * out
        n = int(math.floor((nav.pass_max - nav.pass_margin) / 0.05 + 1e-9)) + 1
        for k in range(n):
            Q = E + (nav.pass_margin + k * 0.05) * out
            if not self._seen_free(Q, pos):
                break                                  # beyond what the sweep can vouch for
            P = Q
            if len(outline) and _segment_distance(outline, P, target).min() >= nav.tangent_clearance:
                break                                  # the next leg clears the obstacle
        return P

    def _seen_free(self, Q, pos):
        """Q and the line to it are clear if some reading whose cone covers it reaches d_wall past it."""
        v = Q - pos
        ang = np.asarray(self.scan["ang"])
        rng = np.asarray(self.scan["rng"])
        on = np.abs(wrap_to_pi(math.atan2(v[1], v[0]) - ang)) <= self.p.ranger.fov / 2
        r = np.minimum(rng[on], self.p.ranger.range_max)
        return bool(np.any(np.linalg.norm(v) <= r - self.p.nav.d_wall))

    def _read_wedge(self, pos, target):
        """Edges of the obstacle in a sweep, one per side (row 0 left, row 1 right).

        Walk out from the middle of the scan; an edge is the last point of the
        outline before the range jumps up by more than edge_jump or returns
        nothing. Points sit half a cone inward of each reading, which puts an
        edge on the obstacle's corner rather than half a cone past it.
        """
        p = self.p
        center = self.sweep_center
        rel_all = wrap_to_pi(np.asarray(self.scan["ang"]) - center)
        order = np.argsort(rel_all, kind="stable")
        rel = rel_all[order]
        r = np.asarray(self.scan["rng"])[order]
        w = {"blocked": False, "edge": np.full((2, 2), np.nan), "seen": [False, False],
             "cost": [math.inf, math.inf], "outline": np.zeros((0, 2))}
        core = np.flatnonzero(np.abs(rel) <= math.radians(10))
        if core.size == 0:
            return w
        k = int(np.argmin(r[core]))
        w["blocked"] = bool(r[core[k]] <= p.nav.d_sweep + 0.3)
        if not w["blocked"]:
            return w
        c = int(core[k])
        h = p.ranger.fov / 2
        ends = [c, c]
        for i in (0, 1):
            step = 1 - 2 * i                           # +1 walks left (increasing angle), -1 right
            j = c
            while True:
                jn = j + step
                if jn < 0 or jn >= len(r):
                    break                              # reached the end of the scan
                if math.isinf(r[jn]) or r[jn] - r[j] > p.nav.edge_jump:
                    w["seen"][i] = True
                    break
                j = jn
            ends[i] = j
            th = center + rel[j] - step * h
            E = pos + r[j] * np.array([math.cos(th), math.sin(th)])
            w["edge"][i] = E
            w["cost"][i] = float(np.linalg.norm(E - pos) + np.linalg.norm(target - E))

        idx = np.arange(len(r))
        on = (idx >= ends[1]) & (idx <= ends[0]) & np.isfinite(r)
        th = center + rel[on] - np.sign(idx[on] - c) * h   # half a cone inward, as for the edges
        w["outline"] = pos + r[on][:, None] * np.column_stack([np.cos(th), np.sin(th)])
        return w

    # ----- pass: fly to the point outside the chosen edge ----------------
    def _pass_edge(self, pos, psi, speed, ranges, t):
        nav = self.p.nav
        if np.linalg.norm(self.subgoal - pos) < nav.wp_tol + 0.05:
            self.mode = "GO_TO_GOAL"                   # round the edge: head for the target
            self.subgoal = np.full(2, np.nan)
            self.facing = False
            return _Command(self.exec_dir, 0.0, 0.0), True
        # Only something close ahead counts: the corner being rounded sits
        # just beside this path and shows in the front cone from further out.
        cmd, event = self._fly_to(pos, psi, speed, ranges, self.subgoal, nav.d_detect)
        if event == "blocked":
            self._start_sweep(psi)                     # more of the obstacle: look again from here
        elif event == "side_left":
            self._start_follow(psi, +1)
        elif event == "side_right":
            self._start_follow(psi, -1)
        return cmd, bool(event)

    # ----- follow: wall-following with the wall on side 'hand' -----------
    def _follow(self, pos, psi, speed, ranges, moved, t):
        nav = self.p.nav
        hold_rate = self._yaw_command(float(wrap_to_pi(self.psi_hold - psi)))

        if self.phase == "align":
            squared = abs(float(wrap_to_pi(self.psi_hold - psi))) < math.radians(0.5)
            if not (squared and speed < nav.v_turn):
                # Stop, then turn on the spot to the nearest room axis.
                return _Command(self.exec_dir, 0.0, (speed < nav.v_turn) * hold_rate), False
            # Head for the obstacle along whichever of front, left, right sees it nearest.
            sides = (1, 2, 4)
            readings = ranges[[0, 1, 3]]
            i = int(np.argmin(readings))
            self.travel_dir = sides[i]
            if readings[i] > nav.d_detect + nav.corner_margin:
                self.travel_dir = 1
            self.phase = "follow"
            self.since_turn = 0.0

        k = self.travel_dir
        flying = self.exec_dir == k                    # false while braking for a turn
        if flying:
            self.since_turn += moved
            if self.following:
                self.travel += moved

        # Exit: closer to the target than the hit point, and (TangentBug's rule)
        # the free space seen toward the target reaches closer to it than any
        # point of the wall followed so far.
        target = self.target()
        R = rot2d(psi)
        rel = R.T @ (target - pos)
        for s in {k, _toward(k, self.hand)}:          # front and wall-side readings are wall points
            if math.isfinite(ranges[s - 1]):
                wall_pt = pos + R @ DIRS[s - 1] * ranges[s - 1]
                self.d_followed = min(self.d_followed, float(np.linalg.norm(target - wall_pt)))
        reach = max(float(np.linalg.norm(rel)) - self._free_toward(rel, ranges), 0.0)
        if (self.following and np.linalg.norm(rel) < self.hit_dist - nav.leave_margin
                and reach < self.d_followed - nav.leave_margin):
            self._log_event(t, "leave", pos)
            self._end_episode()
            self.mode = "GO_TO_GOAL"
            self.facing = False
            return _Command(k, 0.0, 0.0), True

        # Full loop around the obstacle without an exit: the target is unreachable.
        if (self.following and self.travel > nav.loop_min_travel
                and np.linalg.norm(pos - self.loop_point) < nav.loop_tol):
            return _Command(k, 0.0, 0.0), self._give_up(pos, t)

        # Nothing blocked the way in after squaring up: a false alarm, carry on.
        if not self.following and flying and self.since_turn > nav.d_detect + nav.corner_margin:
            self.mode = "GO_TO_GOAL"
            self.facing = False
            return _Command(k, 0.0, 0.0), True

        # Wall-following rules, only while actually flying the travel direction.
        front = ranges[k - 1]
        wall = ranges[_toward(k, self.hand) - 1]
        if flying:
            if wall <= nav.d_open:
                self.wall_seen = True
            if front <= nav.d_wall + 0.02 or wall < nav.d_close:
                self._turn(_away(k, self.hand), pos, True)        # blocked ahead, or too close to the wall
            elif self.phase == "follow":
                if self.wall_seen and wall > nav.d_open:
                    self.phase = "overshoot"                       # the wall has ended
                    self.overshoot_left = nav.corner_margin
                elif (self.following and not self.wall_seen
                      and self.since_turn > 2 * (nav.d_wall + nav.corner_margin)):
                    self._turn(_toward(k, self.hand), pos, False)  # no wall came back: wrap again
            else:
                self.overshoot_left -= moved
                if self.overshoot_left <= 0.02:
                    self._turn(_toward(k, self.hand), pos, False)  # clear of the corner: turn back

        k = self.travel_dir
        speed_cmd = min(nav.v_follow, nav.k_stop * max(ranges[k - 1] - nav.d_wall, 0.0))
        if self.phase == "overshoot":
            speed_cmd = min(speed_cmd, nav.k_stop * max(self.overshoot_left, 0.0))
        return _Command(k, speed_cmd, hold_rate), False

    def _free_toward(self, rel, ranges):
        """How far the drone can see it could fly toward the target (rel, body frame)."""
        p = self.p
        dist = float(np.linalg.norm(rel))
        if dist == 0:
            return math.inf
        ang = np.arccos(np.clip(DIRS @ (rel / dist), -1, 1))   # sensor axis to target direction
        a = np.maximum(ang - p.ranger.fov / 2, 0)
        look = a < math.radians(80)
        if not look.any() or np.all(np.isinf(ranges[look])):
            return math.inf
        r = np.minimum(ranges[look], p.ranger.range_max)
        return max(float(np.min(r / np.cos(a[look]))) - p.nav.d_wall, 0.0)

    # ==================================================================
    def _start_avoid(self, pos, t):
        self.mode = "AVOID"
        if not self.episode:                           # first contact: this is the hit point
            self.episode = True
            self.hit_point = pos.copy()
            self.hit_dist = float(np.linalg.norm(self.target() - pos))
            self.side = 0
            self.ep_travel = 0.0
            self.sweep_spots = []
            d = pos - self.target()
            self.last_bearing = math.atan2(d[1], d[0])
            self.winding = 0.0
            self._log_event(t, "hit", pos)

    def _start_sweep(self, psi):
        self.phase = "sweep"
        self.sweep_stage = "stop"
        self.sweep_center = psi

    def _start_follow(self, psi, hand):
        self.phase = "align"
        self.hand = hand
        self.psi_hold = _round_half_away(psi / (math.pi / 2)) * (math.pi / 2)   # nearest room axis
        self.d_followed = math.inf
        self.following = False
        self.wall_seen = False
        self.since_turn = 0.0
        self.travel = 0.0

    def _turn(self, new_dir, pos, wall_seen):
        self.travel_dir = new_dir
        self.phase = "follow"
        self.wall_seen = wall_seen
        self.since_turn = 0.0
        if not self.following:                         # first turn: the wall is now beside the drone
            self.following = True
            self.loop_point = pos.copy()
            self.travel = 0.0

    def _give_up(self, pos, t):
        self._log_event(t, "unreachable", pos)
        self.unreachable.append(self.wp)
        self._next_target()
        return True

    def _end_episode(self):
        self.episode = False
        self.side = 0
        self.subgoal = np.full(2, np.nan)
        self.hit_point = np.full(2, np.nan)
        self.hit_dist = math.inf

    def _next_target(self):
        self._end_episode()
        self.following = False
        self.facing = False
        self.phase = ""
        if self.wp < len(self.targets) - 1:
            self.wp += 1
            self.mode = "GO_TO_GOAL"
        else:
            self.mode = "DONE"

    def _log_event(self, t, kind, pos):
        self.events.append({"t": t, "type": kind, "pos": pos.copy(), "wp": self.wp})

    def _yaw_command(self, err):
        return float(np.clip(self.p.nav.k_yaw * err, -self.p.yaw_rate_max, self.p.yaw_rate_max))
