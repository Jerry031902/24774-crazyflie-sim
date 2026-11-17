"""Check paired statistics and incomplete-window/censored comparisons."""
from pathlib import Path
import sys
import unittest

import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze_flow_campaign import aggregate, compare, observed_window, summary, SEEDS, ERROR_METRICS


class CampaignStatisticsTests(unittest.TestCase):
    def test_sample_std_and_missing_values_are_explicit(self):
        self.assertEqual(summary([1.,2.,3.,None]),dict(n=3,mean=2.,std=1.))
        self.assertEqual(summary([None]),dict(n=0,mean=None,std=None))
        self.assertEqual(summary([4.]),dict(n=1,mean=4.,std=None))

    def test_half_open_window_and_incomplete_rms(self):
        t=np.arange(25000,36001)*.001
        values=np.full(len(t),2.)
        values[(t<26)|(t>=30)]=1000.
        r=observed_window(values,t,26.,30.)
        self.assertEqual(r,dict(rms=2.,observed_rms=2.,coverage=1.,complete=True))
        keep=t<28
        r=observed_window(values[keep],t[keep],26.,30.)
        self.assertIsNone(r["rms"])
        self.assertEqual(r["observed_rms"],2.)
        self.assertEqual(r["coverage"],.5)
        self.assertFalse(r["complete"])

    def test_early_termination_is_not_a_max_error_win(self):
        a=dict(full_tracking_max_m=12.,complete_run=True)
        z=dict(full_tracking_max_m=1.,complete_run=False)
        r=compare("full_tracking_max_m",a,z,"lower")
        self.assertEqual(r["delta"],-11.)
        self.assertFalse(r["comparable"])
        self.assertEqual(r["outcome"],"indeterminate")
        z["full_tracking_max_m"]=13.
        self.assertEqual(compare("full_tracking_max_m",a,z,"lower")["outcome"],"worse")
        self.assertEqual(compare("full_tracking_max_m",z,a,"lower")["outcome"],"better")

    def test_recovery_censoring_does_not_become_zero_or_a_false_win(self):
        a=dict(recovery_joint_s=None,recovery_censor_lower_bound_s=31.)
        z=dict(recovery_joint_s=2.,recovery_censor_lower_bound_s=31.)
        r=compare("recovery_joint_s",a,z,"lower")
        self.assertEqual(r["outcome"],"better")
        self.assertIsNone(r["delta"])
        self.assertFalse(r["comparable"])
        a["recovery_censor_lower_bound_s"]=1.
        self.assertEqual(compare("recovery_joint_s",a,z,"lower")["outcome"],"indeterminate")
        z["recovery_joint_s"]=None
        self.assertEqual(compare("recovery_joint_s",a,z,"lower")["outcome"],"indeterminate")

    def test_partial_max_group_reports_observed_delta_not_a_complete_delta(self):
        rows=[]
        for seed in SEEDS:
            rows.extend([
                dict(case="zero",seed=seed,mode="stock",complete_run=True,full_estimate_max_m=float(seed)),
                dict(case="zero",seed=seed,mode="gated",complete_run=False,full_estimate_max_m=float(seed+2)),
            ])
        _,stats=aggregate(rows)
        r=next(r for r in stats if r["metric"]=="full_estimate_max_m")
        self.assertEqual((r["delta_n"],r["delta_mean"]),(0,None))
        self.assertEqual((r["observed_delta_n"],r["observed_delta_mean"],r["observed_delta_std"]),(5,2.,0.))
        self.assertEqual((r["stock_complete_runs"],r["gated_complete_runs"]),(5,0))
        self.assertTrue(r["mean_worse_proven_from_lower_bound"])
        self.assertEqual(r["worse"],5)
        for row in rows:
            if row["mode"]=="gated":row["full_estimate_max_m"]-=4
        _,stats=aggregate(rows)
        r=next(r for r in stats if r["metric"]=="full_estimate_max_m")
        self.assertFalse(r["mean_worse_proven_from_lower_bound"])
        self.assertEqual((r["better"],r["indeterminate"]),(0,5))

    def test_detection_and_clean_rejection_have_opposite_directions(self):
        a,z={"rate":0.},{"rate":.5}
        self.assertEqual(compare("rate",a,z,"higher")["outcome"],"better")
        self.assertEqual(compare("rate",a,z,"lower")["outcome"],"worse")

    def test_paired_delta_sd_is_not_the_difference_of_group_sds(self):
        deltas=(1.,-2.,0.,3.,-4.)
        rows=[]
        for seed,delta in zip(SEEDS,deltas):
            for mode,offset in (("stock",0.),("gated",delta)):
                row=dict(case="off",seed=seed,mode=mode,complete_run=True)
                row.update({k:10.+seed+offset for k in ERROR_METRICS})
                rows.append(row)
        _,stats=aggregate(rows)
        r=next(r for r in stats if r["metric"]=="during_estimate_rms_m")
        self.assertAlmostEqual(r["delta_mean"],-.4)
        self.assertAlmostEqual(r["delta_std"],np.std(deltas,ddof=1))
        self.assertEqual((r["better"],r["worse"],r["ties"],r["indeterminate"]),(2,2,1,0))
        self.assertEqual(r["delta_n"],5)


if __name__=="__main__":unittest.main()
