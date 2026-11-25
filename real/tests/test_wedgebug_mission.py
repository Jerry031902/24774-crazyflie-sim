"""Offline tests of wedgebug_mission.py. No CRTP init, radio or USB access."""

import math
from pathlib import Path
import struct
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import wedgebug_mission as app

MISSIONS = Path(__file__).resolve().parents[1] / "missions"


def args(*argv):
    return app.parse_args([str(a) for a in argv])


class Clock:
    def __init__(self, on_sleep=None):
        self.now, self.on_sleep = 0.0, on_sleep

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        if self.on_sleep:
            self.on_sleep(self.now)


class MissionTest(unittest.TestCase):
    def test_example_missions_load(self):
        for name in ("straight", "one_box"):
            m = app.load_mission(args(MISSIONS / f"{name}.json"))
            self.assertEqual(m["name"], name)
            self.assertEqual(m["targets"], [m["goal"]])
            params = app.mission_params(m, args(MISSIONS / f"{name}.json"))
            self.assertEqual(params["wedgebug.nTargets"], 1)
            self.assertEqual((params["wedgebug.x0"], params["wedgebug.y0"]), tuple(m["goal"]))
            self.assertEqual(params["wedgebug.tsRanges"], 1)

    def test_overrides(self):
        a = args(MISSIONS / "one_box.json", "--target", 1, 0.5, "--target", 2, 0, "--psi0-deg", 90,
                 "--nav", "vCruise=0.3", "--nav", "maxSweeps=8", "--baseline")
        m = app.load_mission(a)
        self.assertEqual(m["targets"], [[1.0, 0.5], [2.0, 0.0]])
        params = app.mission_params(m, a)
        self.assertEqual(params["wedgebug.nTargets"], 2)
        self.assertAlmostEqual(params["wedgebug.psi0"], math.pi / 2)
        self.assertEqual(params["wbnav.vCruise"], 0.3)
        self.assertEqual(params["wedgebug.tsRanges"], 0)

    def test_fence_from_room(self):
        m = app.load_mission(app.parse_args(["--target", "3", "2", "--fence", "0", "4", "0", "3"]))
        self.assertEqual(m["fence"], [0.0, 4.0, 0.0, 3.0])

    def test_rejects(self):
        for argv in (["--target", "5", "0", "--fence", "-1", "3", "-1", "1"],          # outside the fence
                     ["--target", "1", "0", "--fence", "-1", "3", "-1", "1", "--height", "1.5"],
                     ["--target", "1", "0"],                                              # no fence
                     ["--start", "4", "0", "--target", "1", "0", "--fence", "-1", "3", "-1", "1"],
                     ["--fence", "-1", "3", "-1", "1"] + ["--target", "1", "0"] * 9):    # 9 targets
            with self.assertRaises(ValueError, msg=argv):
                app.load_mission(app.parse_args(argv))


def fake_param(types):
    """cf.param that acknowledges at float32 precision, as the drone does."""
    callbacks, sent = {}, {}

    def element(name):
        return SimpleNamespace(pytype=types[name]) if name in types else None

    def set_value(name, text):
        pytype = types[name]
        value = float(text) if pytype == "<f" else int(text)  # what cflib does
        sent[name] = text
        echoed = struct.unpack(pytype, struct.pack(pytype, value))[0]
        group, key = name.split(".")
        callbacks[(group, key)](name, str(echoed))

    param = SimpleNamespace(toc=SimpleNamespace(get_element_by_complete_name=element), set_value=set_value,
                            add_update_callback=lambda group, name, cb: callbacks.__setitem__((group, name), cb),
                            remove_update_callback=lambda group, name, cb: callbacks.pop((group, name)))
    return param, sent


class SetParamTest(unittest.TestCase):
    def test_float32_and_integers(self):
        param, sent = fake_param({"wbnav.wpTol": "<f", "wbnav.maxSweeps": "<i", "wedgebug.nTargets": "<B"})
        cf = SimpleNamespace(param=param)
        app.set_param(cf, "wbnav.wpTol", 0.1)           # 0.1 is not exact in float32
        app.set_param(cf, "wbnav.maxSweeps", 8.0)       # from --nav maxSweeps=8
        app.set_param(cf, "wedgebug.nTargets", 2)
        self.assertEqual(sent, {"wbnav.wpTol": "0.1", "wbnav.maxSweeps": "8", "wedgebug.nTargets": "2"})
        with self.assertRaises(ValueError):
            app.set_param(cf, "wbnav.maxSweeps", 8.5)
        with self.assertRaises(KeyError):
            app.set_param(cf, "wbnav.noSuch", 1)


class FlyTest(unittest.TestCase):
    """fly() against a scripted app: telemetry changes with simulated time."""

    def run_fly(self, script, interrupts=(), connected=lambda t: True):
        values = {"wedgebug.count": 4, "wedgebug.state": 0, "wedgebug.reason": 1}  # an earlier mission
        started = []
        cf = SimpleNamespace(param=SimpleNamespace(set_value=Mock(side_effect=lambda n, v: started.append(clock.now)
                                                                   if (n, v) == ("wedgebug.start", "1") else None)),
                             supervisor=Mock(), is_connected=lambda: connected(clock.now))

        def advance(now):
            if started:
                values.update(script(now - started[0]))
            if interrupts and now >= interrupts[0]:
                interrupts.pop(0)
                raise KeyboardInterrupt

        clock = Clock(advance)
        telemetry = SimpleNamespace(get=lambda name, default=math.nan: values.get(name, default))
        interrupts = list(interrupts)
        with patch.object(app.time, "monotonic", clock.monotonic), patch.object(app.time, "sleep", clock.sleep), \
                patch("builtins.print"):
            result = app.fly(cf, telemetry, lambda what: None, timeout_s=120)
        return result, cf

    @staticmethod
    def mission(end_reason, end_at=20.0):
        def script(t):
            if t < 0.1:
                return {}
            state = 1 if t < 2 else 2 if t < 4 else 4 if t < end_at else 7 if t < end_at + 3 else 0
            reason = end_reason if state in (0, 7) else 0
            return {"wedgebug.count": 5, "wedgebug.state": state, "wedgebug.reason": reason}
        return script

    def test_completes(self):
        result, cf = self.run_fly(self.mission(1))
        self.assertEqual(result, 1)
        cf.param.set_value.assert_called_once_with("wedgebug.start", "1")

    def test_old_reason_is_not_taken_as_the_end(self):
        # count unchanged for a while: still the previous mission's state/reason
        def slow(t):
            return {} if t < 2.5 else self.mission(1)(t - 2.5)
        result, _ = self.run_fly(slow)
        self.assertEqual(result, 1)

    def test_new_count_with_idle_state_and_no_reason_is_not_the_end(self):
        def script(t):
            if t < 1.0:
                return {"wedgebug.count": 5, "wedgebug.state": 0, "wedgebug.reason": 0}
            return self.mission(1)(t)
        result, _ = self.run_fly(script)
        self.assertEqual(result, 1)

    def test_bad_mission_ends_without_leaving_idle(self):
        result, _ = self.run_fly(lambda t: {"wedgebug.count": 5, "wedgebug.state": 0, "wedgebug.reason": 3})
        self.assertEqual(result, 3)

    def test_start_not_taken_up(self):
        with self.assertRaisesRegex(RuntimeError, "did not take up"):
            self.run_fly(lambda t: {})

    def test_ctrl_c_lands_then_emergency_stop(self):
        def script(t):  # flies until told to stop, never lands in this script
            return {} if t <= 0.1 else {"wedgebug.count": 5, "wedgebug.state": 4, "wedgebug.reason": 0}
        result, cf = self.run_fly(script, interrupts=(10.0, 12.0))
        self.assertIsNone(result)
        cf.param.set_value.assert_any_call("wedgebug.start", "0")
        cf.supervisor.send_emergency_stop.assert_called_once()

    def test_ctrl_c_once_lands(self):
        landed = {}
        def script(t):
            if landed.get("at") is None and t >= 10.0:
                landed["at"] = t
            if landed:
                return {"wedgebug.count": 5, "wedgebug.state": 7 if t < landed["at"] + 3 else 0, "wedgebug.reason": 2}
            return {} if t <= 0.1 else {"wedgebug.count": 5, "wedgebug.state": 4, "wedgebug.reason": 0}
        result, cf = self.run_fly(script, interrupts=(10.0,))
        self.assertEqual(result, 2)
        cf.supervisor.send_emergency_stop.assert_not_called()

    def test_link_lost(self):
        with self.assertRaises(ConnectionError):
            self.run_fly(self.mission(1), connected=lambda t: t < 5)


class StatusLineTest(unittest.TestCase):
    def test_partial_telemetry(self):
        values = {"wedgebug.state": 4, "wedgebug.mode": 2, "wedgebug.phase": 1, "range.front": 32767, "range.left": 850}
        line = app.status_line(SimpleNamespace(get=lambda n, d=math.nan: values.get(n, d)))
        self.assertIn("NAVIGATE", line)
        self.assertIn("AVOID/sweep", line)
        self.assertIn("F  inf", line)
        self.assertIn("L 0.85", line)


if __name__ == "__main__":
    unittest.main()
