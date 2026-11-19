"""Native gate-v2 state machine and independent pre-v2 binary compatibility."""
import hashlib
import json
from pathlib import Path
import struct
import unittest

import test_flow_gate as legacy
cf = legacy.cf


@unittest.skipIf(cf is None, "Use Isaac Sim Python with built cffirmware")
class FlowRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.h = legacy.FlowGateTests()
        self.assertTrue(cf.kalmanCoreSetFlowGateRecoverK(10))
        self.h.configure()

    def tearDown(self):
        cf.kalmanCoreSetFlowGateRecoverK(10)
        self.h.configure()

    def configure(self, limit=3, k=3):
        self.assertTrue(cf.kalmanCoreSetFlowGateRecoverK(k))
        self.h.configure(2, 9., limit)

    def enter(self, core, limit=3):
        for _ in range(limit):
            result = self.h.update(core, 20, 0)
        self.assertEqual(result[0].state, cf.KC_FLOW_RECOVERY)
        return result

    def test_modes_zero_and_one_match_independent_accepted_binary(self):
        fixture = json.loads(Path(__file__).with_name("flow_gate_v1_golden.json").read_text())
        self.assertEqual(fixture["source_commit"], "15c6e48f")
        for case in fixture["rows"]:
            self.h.configure(case["mode"], 9., case["limit"])
            core = self.h.core()
            digest = hashlib.sha256()
            for x, y in fixture["trace"]:
                gx, gy = self.h.update(core, x, y)
                bits = self.h.state_bits(core, nis=True)
                digest.update(struct.pack("<" + "I" * len(bits), *bits))
                digest.update(bytes([core.isUpdated]))
                digest.update(struct.pack("<14I", *[getattr(g, k) for g in (gx, gy) for k in
                    ("accepted", "rejected", "forced", "consecutive", "streakBefore", "runAtDecision", "decision")]))
            self.assertEqual(digest.hexdigest(), case["sha256"], case)

    def test_defaults_and_nth_sample_enters_recovery(self):
        self.assertEqual(cf.kalmanCoreGetFlowGateDefaultRecoverK(), 10)
        self.assertEqual(cf.kalmanCoreGetFlowGateDefaultMaxReject(), 10)
        self.configure(10, 10)
        core = self.h.core()
        before = self.h.state_bits(core)
        for n in range(1, 10):
            x, y = self.h.update(core, 20, 0)
            self.assertEqual((x.state, x.consecutive, x.rejected), (0, n, n))
        # Y's zero updates can alter covariance, so compare X update flag/state separately.
        self.assertEqual(self.h.array(core.S, 9)[3], 0.)
        x, y = self.h.update(core, 20, 0)
        self.assertEqual((x.stateBefore, x.stateAtUpdate, x.state), (0, 1, 1))
        self.assertEqual((x.streakBefore, x.runAtDecision, x.consecutive), (9, 10, 0))
        self.assertEqual((x.accepted, x.rejected, x.forced, x.recoveryEntries, x.samplesInRecovery), (0, 9, 1, 1, 1))
        self.assertEqual((y.state, y.recoveryEntries, y.samplesInRecovery), (0, 0, 0))
        self.assertNotEqual(self.h.state_bits(core), before)

    def test_recovery_high_nis_updates_use_original_r(self):
        self.configure()
        a, stock = self.h.core(), self.h.core()
        # Reject both axes, leaving state/covariance exactly equal to stock before entry.
        for _ in range(2):
            self.h.update(a, 20, 20)
        for i in range(6):
            self.h.configure(2, 9, 3)
            gx, gy = self.h.update(a, 20, 20)
            actual = self.h.state_bits(a, nis=True)
            self.assertGreater(cf.kalmanCoreGetFlowNisX(), 9.)
            self.h.configure(0, 9, 3)
            self.h.update(stock, 20, 20)
            self.assertEqual(actual, self.h.state_bits(stock, nis=True))
            self.assertEqual((gx.samplesInRecovery, gx.recoveryEntries), (i + 1, 1))
            self.assertEqual(gx.decision, cf.KC_FLOW_FORCED if i == 0 else cf.KC_FLOW_ACCEPTED)

    def test_consecutive_low_nis_rearms_only_after_k_and_high_breaks_run(self):
        self.configure()
        core = self.h.core()
        self.enter(core)
        for n in (1, 2):
            x, _ = self.h.update(core, 0, 0)
            self.assertEqual((x.state, x.recoveryConsecutive), (1, n))
        x, _ = self.h.update(core, 20, 0)
        self.assertEqual((x.state, x.recoveryConsecutive, x.decision), (1, 0, 0))
        for _ in range(3):
            x, _ = self.h.update(core, 0, 0)
        self.assertEqual((x.stateBefore, x.stateAtUpdate, x.state), (1, 1, 0))
        self.assertEqual((x.recoveryRunAtDecision, x.recoveryConsecutive, x.samplesInRecovery), (3, 0, 7))
        x, _ = self.h.update(core, 20, 0)
        self.assertEqual((x.decision, x.consecutive, x.samplesInRecovery), (1, 1, 7))
        self.h.update(core, 20, 0)
        x, _ = self.h.update(core, 20, 0)
        self.assertEqual((x.recoveryEntries, x.samplesInRecovery, x.forced), (2, 8, 2))

    def test_exact_threshold_counts_as_recovery_good_sample(self):
        self.configure(k=1)
        a, b = self.h.core(), self.h.core()
        self.enter(a)
        self.enter(b)
        self.h.update(b, 2, 0)
        threshold = cf.kalmanCoreGetFlowNisX()
        self.h.configure(2, threshold, 3)
        # Prepare the same pre-update state, with an already cached config, so
        # this tests equality rather than the separate parameter-reset rule.
        a.flowGateThreshold = threshold
        x, _ = self.h.update(a, 2, 0)
        self.assertEqual(cf.kalmanCoreGetFlowNisX(), threshold)
        self.assertEqual((x.stateAtUpdate, x.state, x.recoveryRunAtDecision), (1, 0, 1))

    def test_runtime_change_resets_state_but_not_totals(self):
        self.configure()
        core = self.h.core()
        self.enter(core)
        cf.kalmanCoreSetFlowGateRecoverK(4)
        x, _ = self.h.update(core, 20, 0)
        self.assertEqual((x.stateBefore, x.state, x.consecutive, x.recoveryEntries, x.samplesInRecovery), (0, 0, 1, 1, 1))
        self.h.configure(1, 9, 10)
        self.h.update(core, 20, 0)
        cf.kalmanCoreSetFlowGateRecoverK(8)  # Irrelevant to v1; do not reset its streak.
        x, _ = self.h.update(core, 20, 0)
        self.assertEqual((x.consecutive, x.recoveryEntries), (2, 1))

    def test_core_reset_and_independence(self):
        self.configure()
        a, b = self.h.core(), self.h.core()
        self.enter(a)
        self.assertEqual((b.flowGateX.state, b.flowGateX.recoveryEntries), (0, 0))
        params = cf.kalmanCoreParams_t()
        cf.kalmanCoreDefaultParams(params)
        cf.kalmanCoreInit(a, params, 100)
        self.assertEqual((a.flowGateX.state, a.flowGateX.recoveryEntries, a.flowGateX.samplesInRecovery), (0, 0, 0))

    def test_recovery_counters_saturate_and_k_uint16_boundary(self):
        self.configure(limit=1, k=65535)
        core = self.h.core()
        core.flowGateX.recoveryEntries = core.flowGateX.samplesInRecovery = 2**32-1
        self.enter(core, 1)
        core.flowGateX.recoveryConsecutive = 65534
        x, _ = self.h.update(core, 0, 0)
        self.assertEqual((x.state, x.recoveryRunAtDecision), (0, 65535))
        self.assertEqual((x.recoveryEntries, x.samplesInRecovery), (2**32-1, 2**32-1))

    def test_recover_k_validation(self):
        from flow_gate import FlowGateConfig
        self.assertFalse(cf.kalmanCoreSetFlowGateRecoverK(0))
        self.assertEqual(cf.kalmanCoreGetFlowGateRecoverK(), 10)
        for k in (0, -1, 65536, 1.5):
            with self.assertRaises(ValueError):
                FlowGateConfig(2, recover_k=k)
        self.assertEqual(FlowGateConfig(2).apply(cf)["recover_k"], 10)

    def test_logging_labels_and_dropout_do_not_drive_state_machine(self):
        from dataclasses import replace
        import numpy as np
        from estimators.firmware_ekf import FirmwareEKF
        from flow_faults import FlowFaultConfig
        from flow_gate import FlowGateConfig
        from sensors import SensorSuite
        suite = SensorSuite(noise=True, seed=7, flow_fault=FlowFaultConfig("dropout", start_s=.02, duration_s=.02), flow_audit=True)
        ekfs = [FirmwareEKF(flow_gate=FlowGateConfig(2, max_reject=1, recover_k=10), flow_gate_log=True) for _ in range(2)]
        logged = []
        for ms in range(51):
            truth = dict(time_s=ms/1000, position_m=(0,0,1), orientation_wxyz=(1,0,0,0),
                         linear_velocity_m_s=(.2,0,0), angular_velocity_rad_s=(0,0,0))
            packets = [replace(p, value=np.array([50.,50.])) if p.kind == "flow" else p for p in suite.sample(truth)]
            changed = [replace(p, flow_labels=dict(p.flow_labels, fault_type="zero", fault_active=True))
                       if p.kind == "flow" else p for p in packets]
            for ekf, ps in zip(ekfs, (packets, changed)):
                ekf.step(ms, ps, True)
            self.assertEqual(self.h.state_bits(ekfs[0].core), self.h.state_bits(ekfs[1].core))
            for a, b in zip(ekfs[0].flow_gate_rows, ekfs[1].flow_gate_rows):
                for key in ("decision", "state_after", "recovery_entries_total", "recovery_samples_total"):
                    self.assertEqual(a[key], b[key])
            logged.extend(ekfs[0].flow_gate_rows)
            if 10 <= ms < 40:
                self.assertEqual(ekfs[0].core.flowGateX.samplesInRecovery, 1)
        self.assertEqual([(r["flow_sample_id"], r["axis"]) for r in logged], [(n,a) for n in (1,4,5) for a in "xy"])
        self.assertTrue(all("state_at_update" in r and r["recover_k"] == 10 for r in logged))


if __name__ == "__main__":
    unittest.main()
