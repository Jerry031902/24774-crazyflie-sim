"""Metric boundaries and censoring, independent of flight pass/fail."""

from pathlib import Path
import copy
import json
import sys
import tempfile
import unittest

import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze_flow_gate import bursts, metrics, recovery_time


class GateMetricTests(unittest.TestCase):
    def test_above_threshold_runs_include_single_and_terminal_samples(self):
        t=np.arange(6)*.01+30
        runs=bursts(np.array([True,False,True,True,False,True]),t)
        self.assertEqual([r["count"] for r in runs],[1,2,1])
        self.assertAlmostEqual(runs[1]["span_s"],.02)
        self.assertEqual(bursts(np.zeros(6,dtype=bool),t),[])

    def test_recovery_requires_full_continuous_hold_after_window(self):
        t=np.arange(0,3001)*.001+29
        ok=np.ones(len(t),dtype=bool)
        ok[(t>=30.7)&(t<30.8)]=False
        self.assertAlmostEqual(recovery_time(ok,t,30,1),.8)
        self.assertEqual(recovery_time(np.ones(len(t),dtype=bool),t,30,1),0.)

    def test_contact_censor_or_insufficient_tail_is_not_success(self):
        t=np.arange(1000)*.001+30
        self.assertIsNone(recovery_time(np.ones(len(t),dtype=bool),t,30,1))
        t=np.arange(3001)*.001+30
        self.assertIsNone(recovery_time(t<30.5,t,30,1))

    def test_recovery_rejects_unmatched_clean_counterfactual(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            clean,case=root/"clean.csv",root/"fault.csv"
            clean.write_text("time_s,est_x_m,x_m,est_y_m,y_m,target_x_m,target_y_m\n0,0,0,0,0,0,0\n")
            baseline=dict(mode="ekf_closed_loop",trajectory="flow-shuttle-x",scene={"scene":"flow-empty"},
                          settings={"seed":7,"noise":True},
                          flow_gate={"enabled":1,"threshold":9.,"max_reject":10})
            clean.with_suffix(".json").write_text(json.dumps(baseline))
            for section,key,value in (("settings","seed",8),("scene","scene","arena"),
                                      ("flow_gate","enabled",0),("flow_gate","threshold",6.63)):
                changed=copy.deepcopy(baseline)
                changed[section][key]=value
                case.with_suffix(".json").write_text(json.dumps(changed))
                with self.assertRaises(ValueError):
                    metrics(clean,[case],root/"out")


if __name__=="__main__":
    unittest.main()
