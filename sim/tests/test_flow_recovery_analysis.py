"""Keep paired analysis compatible with Step 4 and independent of seed values."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze_flow_campaign import aggregate as previous_aggregate, CASES, SEEDS, ERROR_METRICS
from analyze_flow_recovery import aggregate


class RecoveryAnalysisTests(unittest.TestCase):
    def rows(self,seeds):
        rows=[]
        for case in CASES:
            for j,seed in enumerate(seeds):
                for mode,offset in (("stock",0.),("gated",float(j-2))):
                    r=dict(case=case,seed=seed,mode=mode,complete_run=True)
                    r.update({k:10.+j+offset for k in ERROR_METRICS})
                    rows.append(r)
        return rows

    def test_existing_pairs_and_statistics_are_preserved(self):
        old=self.rows(SEEDS);op,os=previous_aggregate(old)
        new=[dict(r,mode="v1" if r["mode"]=="gated" else "stock") for r in old]
        pairs,stats=aggregate(new,SEEDS,[("stock","v1")])
        pl={(r["case"],r["seed"],r["metric"]):r for r in pairs}
        sl={(r["case"],r["metric"]):r for r in stats}
        for r in op:
            n=pl[(r["case"],r["seed"],r["metric"])]
            for k in ("delta","comparable","outcome"):self.assertEqual(n[k],r[k])
            self.assertEqual((n["reference"],n["candidate"]),(r["stock"],r["gated"]))
        for r in os:
            n=sl[(r["case"],r["metric"])]
            for k in ("delta_n","delta_mean","delta_std","observed_delta_n","observed_delta_mean","observed_delta_std",
                      "better","worse","indeterminate","mean_worse_proven_from_lower_bound"):
                self.assertEqual(n[k],r[k])
            self.assertEqual(n["tie"],r["ties"])

    def test_holdout_seeds_are_not_replaced_with_development_seeds(self):
        seeds=(2026111701,2026111702,2026111703,2026111704,2026111705)
        rows=[dict(r,mode="v2" if r["mode"]=="gated" else "stock") for r in self.rows(seeds)]
        pairs,stats=aggregate(rows,seeds,[("stock","v2")])
        self.assertEqual({r["seed"] for r in pairs},set(seeds))
        s=next(r for r in stats if r["case"]=="off" and r["metric"]==ERROR_METRICS[0])
        self.assertEqual((s["delta_n"],s["delta_mean"],s["better"],s["worse"],s["tie"]),(5,0.,2,2,1))


if __name__=="__main__":unittest.main()
