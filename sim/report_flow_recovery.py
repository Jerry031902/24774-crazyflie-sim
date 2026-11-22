"""Exhaustive A/B tables, regression lists and static paired-error plots."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from analyze_flow_campaign import CASES, ERROR_METRICS
from report_flow_campaign import number, mean_sd, unit, markdown_table


def report(root, dataset):
    out=root/dataset/"analysis"
    read=lambda name: json.loads((out/(name+".json")).read_text())
    rows,pairs,stats,worse,means=(read(n) for n in ("per_seed","paired","summary","v2_worse_than_stock","v2_worse_mean"))
    modes=("stock","v1","v2") if dataset=="A" else ("stock","v2")
    seeds=sorted({r["seed"] for r in rows})
    intro=[f"# Gate v2 dataset {dataset}","",
        f"Seeds: {seeds}. flow-empty / flow-shuttle-x / closed loop / noise on / mass 0.033 kg.",
        "threshold=9.0, maxReject=10, recoverK=10，均在 A 之前固定；A/B 之间 code 和参数不变。",
        "During=[26,30) s；post=[30,35) s。± 是 seed 间 sample SD（ddof=1），不是 standard error。",
        "Δ=candidate−reference；每项给出各自 n。A 包含 v1−stock、v2−stock、v2−v1；B 包含 v2−stock。",
        "recovery time 使用同 mode、同 seed 的 clean reference：estimate ≤ clean+0.05 m、tracking ≤ clean+0.10 m，连续 1 s；joint 同时满足。",
        "NA recovery 保留 censoring；只有观察到 recovery 才进入 mean/SD。两边都未恢复不能视为相等。",
        "未跑满 62 s 的 full-run max 是 observed lower bound；较小的 partial max 不算 improvement。未观察完整 post window 的 RMS 为 NA，observed RMS/coverage 另存。",
        "fault-active rate 的分母只包含实际进入 EKF 的 scalar updates；dropout 的 rate 为 NA，绝不算 gate rejection。",
        "forced accept 是进入 RECOVERY 的那一个 update；其余 RECOVERY updates 为 accepted，始终使用原来的 R。",
        "RECOVERY samples 包含进入和第 K 个 low-NIS update；state_after 是下一次 update 使用的 state。",
        "rejection rate 和 counters 是 diagnostics。更多 recovery entries/samples 或 forced updates 不单独证明 flight 变差。",
        "所有较差的 paired metric（包括 diagnostics）均列入 REGRESSIONS.md；缺失/截短不能用 0 填补。",
        "", "[全部 per-seed metrics](per_seed.csv) · [全部 paired differences](paired.csv) · [全部 mean/SD](summary.csv) · [全部变差项目](REGRESSIONS.md)",""]
    lines=list(intro)
    lines += ["## 完成情况与 RECOVERY",""]
    table=[]
    for case in CASES:
        for mode in modes:
            g=[r for r in rows if r["case"]==case and r["mode"]==mode]
            table.append([case,mode,sum(r["out_of_bounds"] for r in g),sum(r["stable"] for r in g),
                sum(r["complete_run"] for r in g),sum(r["pooled_gate_recovery_entries"]>0 for r in g),
                ", ".join(f"{r['seed']}:{r['pooled_gate_recovery_entries']}/{r['pooled_gate_recovery_samples']}" for r in g),
                sum(r["recovery_joint_s"] is not None for r in g),sum(r["contact_events"] for r in g)])
    lines+=markdown_table(["Case","Mode","OOB /5","All checks pass /5","62 s complete /5","Entered RECOVERY /5","Seed: entries/samples (X+Y)","Joint recovered /5","Contacts"],table)+[""]
    for comparison in dict.fromkeys(s["comparison"] for s in stats):
        lines += [f"## {comparison}",""]
        for case in CASES:
            lines += [f"### {case}",""]
            table=[]
            for r in stats:
                if r["comparison"]!=comparison or r["case"]!=case: continue
                m=r["metric"]
                table.append([m,unit(m),mean_sd(r,"reference",m),mean_sd(r,"candidate",m),
                    mean_sd(r,"delta",m),mean_sd(r,"observed_delta",m) if m.startswith("full_") and r["delta_n"]<5 else "same",
                    f"{r['better']} / {r['worse']} / {r['tie']} / {r['indeterminate']}"])
            lines += markdown_table(["Metric","Unit","Reference mean ± SD","Candidate mean ± SD","Comparable Δ mean ± SD","Observed Δ (partial max)","Better / worse / tie / unknown"],table)+[""]
    (out/"REPORT.md").write_text("\n".join(lines)+"\n")
    per=[f"# Dataset {dataset}: 每个 seed 的 paired differences","",*intro[2:]]
    for case in CASES:
        per += [f"## {case}",""]
        table=[]
        for p in pairs:
            if p["case"]!=case:continue
            m=p["metric"]
            table.append([p["comparison"],p["seed"],m,number(p["reference"],m),number(p["candidate"],m),number(p["delta"],m,True),p["comparable"],p["outcome"]])
        per += markdown_table(["Comparison","Seed","Metric","Reference","Candidate","Observed Δ","Comparable","Outcome"],table)+[""]
    (out/"PER_SEED.md").write_text("\n".join(per)+"\n")
    lines=[f"# Dataset {dataset}: v2 相对 stock 的所有变差项目","",
           "diagnostic counts/rates 的数值方向依 Step 4 定义；增加次数不等于 flight quality 变差。",
           "只列已知较差的 paired outcomes；censored/partial 无法判定的结果保留在 paired.csv。","","## Mean 较差"," "]
    table=[]
    for r in means:
        m=r["metric"]
        table.append([r["case"],m,mean_sd(r,"delta",m),mean_sd(r,"observed_delta",m),r["worse"],r["mean_worse_proven_from_lower_bound"]])
    lines+=markdown_table(["Case","Metric","Comparable Δ mean ± SD","Observed Δ mean ± SD","Worse /5","Mean worse from lower bound"],table)+["","## 每个 seed 较差",""]
    table=[]
    for p in worse:
        m=p["metric"]
        table.append([p["case"],p["seed"],m,number(p["reference"],m),number(p["candidate"],m),number(p["delta"],m,True),p["comparable"]])
    lines+=markdown_table(["Case","Seed","Metric","Stock","v2","Observed Δ","Comparable"],table)+[""]
    (out/"REGRESSIONS.md").write_text("\n".join(lines))
    fig,axs=plt.subplots(3,2,figsize=(14,11),layout="constrained")
    for ax,metric in zip(axs.flat,ERROR_METRICS):
        for i,case in enumerate(CASES):
            g=[p for p in pairs if p["comparison"]=="v2_minus_stock" and p["case"]==case and p["metric"]==metric]
            for j,p in enumerate(g):
                if p["delta"] is None:continue
                ax.scatter(i+(j-2)*.055,p["delta"]*100,marker="o" if p["comparable"] else "x",color="tab:blue" if p["comparable"] else "tab:orange")
        ax.axhline(0,color="black",lw=.7);ax.set_xticks(range(8),CASES,rotation=25)
        ax.set_title(metric);ax.set_ylabel("v2 - stock (cm)");ax.grid(alpha=.2)
    fig.suptitle(f"Dataset {dataset}: paired errors; orange x = partial observed max (not a complete-run difference)")
    fig.savefig(out/"paired_errors.png",dpi=160);plt.close(fig)
    print(f"Dataset {dataset}: complete tables; {len(worse)} worse paired metrics, {len(means)} worse means",flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("root",type=Path);p.add_argument("dataset",choices=("A","B"))
    args=p.parse_args();report(args.root,args.dataset)
