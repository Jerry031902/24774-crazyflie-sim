"""The C WedgeBug core (apps/wedgebug/src/wedgebug.c) behind the Python navigator interface.

This is the code that runs on the drone; the simulator calls it through the
cffirmware bindings. Build them with `make bindings_python` after changing
the C code.
"""

import math

import numpy as np

from .wedgebug import MODES, PHASES

# Python parameter name (navigation.params) -> C field (wedgebugParams_t).
_NAV_FIELDS = {
    "v_cruise": "vCruise", "v_follow": "vFollow", "k_stop": "kStop", "k_yaw": "kYaw",
    "yaw_tol": "yawTol", "yaw_free_radius": "yawFreeRadius", "d_sweep": "dSweep",
    "sweep_half": "sweepHalf", "sweep_rate": "sweepRate", "edge_jump": "edgeJump",
    "pass_margin": "passMargin", "tangent_clearance": "tangentClearance", "pass_max": "passMax",
    "max_sweeps": "maxSweeps", "max_winding": "maxWinding", "d_side": "dSide",
    "d_detect": "dDetect", "d_wall": "dWall", "d_open": "dOpen", "d_close": "dClose",
    "corner_margin": "cornerMargin", "v_turn": "vTurn", "wp_tol": "wpTol",
    "leave_margin": "leaveMargin", "loop_tol": "loopTol", "loop_min_travel": "loopMinTravel",
}
EVENT_TYPES = {1: "hit", 2: "leave", 3: "reached", 4: "unreachable"}


def _cf():
    try:
        import cffirmware
    except ImportError as exc:
        raise RuntimeError("Build cffirmware (make bindings_python) and add build/ to PYTHONPATH") from exc
    if not hasattr(cffirmware, "wedgebugStep"):
        raise RuntimeError("cffirmware has no WedgeBug: rebuild the bindings with make bindings_python")
    return cffirmware


def params_to_c(p):
    """Fill a wedgebugParams_t from a navigation.params namespace."""
    cf = _cf()
    c = cf.wedgebugParams_t()
    cf.wedgebugDefaultParams(c)
    c.yawRateMax = p.yaw_rate_max
    c.rangerFov = p.ranger.fov
    c.rangerRangeMax = p.ranger.range_max
    for py_name, c_name in _NAV_FIELDS.items():
        value = getattr(p.nav, py_name)
        setattr(c, c_name, int(value) if c_name == "maxSweeps" else float(value))
    return c


def c_default_params():
    """The C defaults as a {python name: value} dict, for consistency checks."""
    cf = _cf()
    c = cf.wedgebugParams_t()
    cf.wedgebugDefaultParams(c)
    values = {"yaw_rate_max": c.yawRateMax, "ranger.fov": c.rangerFov, "ranger.range_max": c.rangerRangeMax}
    values.update({name: getattr(c, field) for name, field in _NAV_FIELDS.items()})
    return values


class FirmwareWedgeBug:
    """Same step()/state interface as navigation.wedgebug.WedgeBugNavigator."""

    def __init__(self, start, targets, p):
        cf = self.cf = _cf()
        self.p = p
        self.targets = np.atleast_2d(np.asarray(targets, dtype=float))
        self.params = params_to_c(p)
        if p.nav.max_sweeps > cf.WEDGEBUG_MAX_SWEEP_SPOTS:
            raise ValueError(f"max_sweeps must be <= {cf.WEDGEBUG_MAX_SWEEP_SPOTS} in the C core")
        self.c = cf.wedgebug_t()
        cf.wedgebugInit(self.c, self.params, float(start[0]), float(start[1]))
        for x, y in self.targets:
            if not cf.wedgebugAddTarget(self.c, float(x), float(y)):
                raise ValueError(f"At most {cf.WEDGEBUG_MAX_TARGETS} targets (waypoints + goal)")
        self._in = cf.wedgebugInput_t()
        self._out = cf.wedgebugOutput_t()
        self._event = cf.wedgebugEvent_t()
        self.events = []
        self.unreachable = []
        self.sweeps = []
        self.subgoal = np.full(2, np.nan)

    # State mirrored from the C struct, named as in the Python navigator.
    @property
    def mode(self):
        return MODES[self.c.mode - 1]

    @property
    def phase(self):
        return PHASES[self.c.phase - 1] if self.c.phase else ""

    @property
    def wp(self):
        return self.c.wp

    @property
    def exec_dir(self):
        return self.c.execDir

    @property
    def hand(self):
        return self.c.hand

    @property
    def episode(self):
        return bool(self.c.episode)

    def target(self):
        return self.targets[self.c.wp]

    def step(self, x, psi, ranges, t):
        cf, i = self.cf, self._in
        i.t = float(t)
        i.x, i.y, i.vx, i.vy = map(float, x[:4])
        i.yaw = float(psi)
        i.rangeFront, i.rangeLeft, i.rangeBack, i.rangeRight = map(float, ranges)
        sweeps_before = self.c.sweepCount
        cf.wedgebugStep(self.c, i, self._out)

        for k in range(self.c.stepEventCount):
            cf.wedgebugGetStepEvent(self.c, k, self._event)
            e = self._event
            kind = EVENT_TYPES[e.type]
            self.events.append({"t": e.t, "type": kind, "pos": np.array([e.x, e.y]), "wp": e.wp})
            if kind == "unreachable":
                self.unreachable.append(e.wp)
        if self.c.sweepCount != sweeps_before:
            s = self.c.lastSweep
            record = {"pos": np.array([s.x, s.y]), "center": s.center,
                      "edge": np.array([[s.edgeLeftX, s.edgeLeftY], [s.edgeRightX, s.edgeRightY]]),
                      "seen": [bool(s.seenLeft), bool(s.seenRight)], "cost": [s.costLeft, s.costRight],
                      "side": s.side, "pass": np.array([s.passX, s.passY]), "t": float(t)}
            self.sweeps.append(record)
            if self.phase == "pass":
                self.subgoal = record["pass"]
        if self.phase != "pass":
            self.subgoal = np.full(2, np.nan)
        return np.array([self._out.vxBody, self._out.vyBody]), self._out.yawRate


def make_navigator(backend, start, targets, p):
    """backend: 'firmware' (the C code that flies) or 'python' (reference port)."""
    if backend == "firmware":
        return FirmwareWedgeBug(start, targets, p)
    if backend == "python":
        from .wedgebug import WedgeBugNavigator
        return WedgeBugNavigator(start, targets, p)
    raise ValueError(f"Unknown navigator backend: {backend}")


def is_finite_command(u_body, yaw_rate):
    return bool(np.all(np.isfinite(u_body)) and math.isfinite(yaw_rate))
