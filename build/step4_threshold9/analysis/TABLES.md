# Threshold 9.0: 完整 metrics

固定 maxReject=10；seeds=7,17,27,37,47；noise seed=fault seed。
During 为 [26,30) s，post 为 [30,35) s；faults-off 使用同一时间段。
所有 ± 为 seed 间 sample SD（ddof=1）。Δ=gated−stock。error 越小越好。
recovery 使用同 mode、同 seed 的 faults-off reference：estimate error ≤ reference+0.05 m，tracking error ≤ reference+0.10 m，连续 1 s；分别报告两个条件以及 joint。
recovery 的 NA 是未观察到 recovery 或 off 不适用；mean/SD 只含已观察到的 recovery，n 明列。
fault-active rejection 的分母是实际到达 EKF 的 scalar updates；dropout 没有 update，rate 为 NA。
clean rejection 包含所有 fault-inactive updates；post_5s_clean 和 post_all_clean 单列。
rate 仅基于实际观察到的 updates；截短 run 的 post_5s_clean 不代表观察满 5 s。各分母、duration 和 window coverage 都保留在 per_seed.csv。
forced accept 不算 rejected；forced count 的增加是发生次数增加，不单独证明 flight 变差。
fault-active rejection 越高表示检测到更多 fault，但不单独证明 flight 更好。
full-run max 在未跑满 62 s 时只是 observed lower bound，不能用较小的 partial max 宣称改善。
Observed Δ 栏保留截短 run 的 max difference；其 SD 描述已观察到的 differences，不是完整 62 s difference 的 SD。
对于 censored recovery 或 partial max，better/worse 仅在已有观察足以确定方向时计入；其余为 indeterminate。

[逐个 run 的全部数据](per_seed.csv) · [每个 paired difference](paired.csv) · [全部变差的 seed/metric](worse_per_seed.csv) · [mean 变差的 metrics](worse_mean.csv)

## off

[该 case 的 per-seed CSV](per_seed_off.csv)

| Metric | Unit | Stock mean ± SD | Gated mean ± SD | Comparable paired Δ mean ± SD | Observed Δ mean ± SD (partial max) | Better / worse / tie / indeterminate (of 5) |
| --- | --- | --- | --- | --- | --- | --- |
| during_estimate_rms_m | m | 0.35945 ± 0.20684 (n=5) | 0.334 ± 0.1997 (n=5) | -0.025458 ± 0.045143 (n=5) | same | 3 / 2 / 0 / 0 |
| during_tracking_rms_m | m | 0.44751 ± 0.28001 (n=5) | 0.42494 ± 0.27189 (n=5) | -0.022568 ± 0.046732 (n=5) | same | 3 / 2 / 0 / 0 |
| post_5s_estimate_rms_m | m | 0.34807 ± 0.24243 (n=5) | 0.34402 ± 0.20608 (n=5) | -0.0040464 ± 0.051157 (n=5) | same | 2 / 3 / 0 / 0 |
| post_5s_tracking_rms_m | m | 0.35803 ± 0.22029 (n=5) | 0.353 ± 0.18456 (n=5) | -0.0050357 ± 0.049667 (n=5) | same | 2 / 3 / 0 / 0 |
| full_estimate_max_m | m | 0.58901 ± 0.24242 (n=5); complete=5/5 | 0.59949 ± 0.237 (n=5); complete=5/5 | 0.010482 ± 0.078684 (n=5) | same | 3 / 2 / 0 / 0 |
| full_tracking_max_m | m | 0.68076 ± 0.17614 (n=5); complete=5/5 | 0.66633 ± 0.20348 (n=5); complete=5/5 | -0.014429 ± 0.080212 (n=5) | same | 3 / 2 / 0 / 0 |
| recovery_estimate_s | s | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| recovery_tracking_s | s | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| recovery_joint_s | s | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| x_fault_active_rejection_rate | % / Δ percentage points | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| x_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25806 ± 0.055873 (n=5) | 0.25806 ± 0.055873 (n=5) | same | 0 / 5 / 0 / 0 |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.32 ± 0.43818 (n=5) | 0.32 ± 0.43818 (n=5) | same | 0 / 2 / 3 / 0 |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.21243 ± 0.097297 (n=5) | 0.21243 ± 0.097297 (n=5) | same | 0 / 5 / 0 / 0 |
| x_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_fault_active_rejection_rate | % / Δ percentage points | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| y_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.24839 ± 0.06205 (n=5) | 0.24839 ± 0.06205 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.44 ± 0.16733 (n=5) | 0.44 ± 0.16733 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.23118 ± 0.092674 (n=5) | 0.23118 ± 0.092674 (n=5) | same | 0 / 5 / 0 / 0 |
| y_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_fault_active_rejection_rate | % / Δ percentage points | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| pooled_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25323 ± 0.053978 (n=5) | 0.25323 ± 0.053978 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.38 ± 0.21679 (n=5) | 0.38 ± 0.21679 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.22181 ± 0.072764 (n=5) | 0.22181 ± 0.072764 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |

### 每个 seed

| Seed | Metric | Stock | Gated | Δ | Comparable Δ | Outcome |
| --- | --- | --- | --- | --- | --- | --- |
| 7 | during_estimate_rms_m | 0.13549 | 0.16903 | +0.033541 | True | worse |
| 7 | during_tracking_rms_m | 0.11221 | 0.099852 | -0.012361 | True | better |
| 7 | post_5s_estimate_rms_m | 0.06641 | 0.11776 | +0.051354 | True | worse |
| 7 | post_5s_tracking_rms_m | 0.088195 | 0.13441 | +0.046214 | True | worse |
| 7 | full_estimate_max_m | 0.4141 | 0.47663 | +0.062534 | True | worse |
| 7 | full_tracking_max_m | 0.51998 | 0.49117 | -0.028816 | True | better |
| 7 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 7 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 7 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 7 | x_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 7 | x_clean_rejection_rate | 0 | 0.27419 | +0.27419 | True | worse |
| 7 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 7 | x_all_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 7 | y_clean_rejection_rate | 0 | 0.19355 | +0.19355 | True | worse |
| 7 | y_post_5s_clean_rejection_rate | 0 | 0.2 | +0.2 | True | worse |
| 7 | y_post_all_clean_rejection_rate | 0 | 0.093721 | +0.093721 | True | worse |
| 7 | y_all_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 7 | pooled_clean_rejection_rate | 0 | 0.23387 | +0.23387 | True | worse |
| 7 | pooled_post_5s_clean_rejection_rate | 0 | 0.1 | +0.1 | True | worse |
| 7 | pooled_post_all_clean_rejection_rate | 0 | 0.14058 | +0.14058 | True | worse |
| 7 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | during_estimate_rms_m | 0.17671 | 0.11403 | -0.062681 | True | better |
| 17 | during_tracking_rms_m | 0.3475 | 0.27237 | -0.075126 | True | better |
| 17 | post_5s_estimate_rms_m | 0.15301 | 0.16456 | +0.01155 | True | worse |
| 17 | post_5s_tracking_rms_m | 0.1966 | 0.21229 | +0.015699 | True | worse |
| 17 | full_estimate_max_m | 0.26209 | 0.23911 | -0.02298 | True | better |
| 17 | full_tracking_max_m | 0.48555 | 0.40121 | -0.084334 | True | better |
| 17 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 17 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 17 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 17 | x_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 17 | x_clean_rejection_rate | 0 | 0.27419 | +0.27419 | True | worse |
| 17 | x_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 17 | x_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | x_all_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 17 | y_clean_rejection_rate | 0 | 0.32258 | +0.32258 | True | worse |
| 17 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 17 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | y_all_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 17 | pooled_clean_rejection_rate | 0 | 0.29839 | +0.29839 | True | worse |
| 17 | pooled_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 17 | pooled_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | during_estimate_rms_m | 0.35719 | 0.31076 | -0.046431 | True | better |
| 27 | during_tracking_rms_m | 0.30997 | 0.34 | +0.030035 | True | worse |
| 27 | post_5s_estimate_rms_m | 0.33986 | 0.34557 | +0.0057027 | True | worse |
| 27 | post_5s_tracking_rms_m | 0.35301 | 0.35401 | +0.001004 | True | worse |
| 27 | full_estimate_max_m | 0.68596 | 0.79453 | +0.10857 | True | worse |
| 27 | full_tracking_max_m | 0.69733 | 0.81259 | +0.11526 | True | worse |
| 27 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 27 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 27 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 27 | x_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 27 | x_clean_rejection_rate | 0 | 0.30645 | +0.30645 | True | worse |
| 27 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_rejection_rate | 0 | 0.21868 | +0.21868 | True | worse |
| 27 | x_all_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 27 | y_clean_rejection_rate | 0 | 0.29032 | +0.29032 | True | worse |
| 27 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 27 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 27 | y_all_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 27 | pooled_clean_rejection_rate | 0 | 0.29839 | +0.29839 | True | worse |
| 27 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 27 | pooled_post_all_clean_rejection_rate | 0 | 0.26554 | +0.26554 | True | worse |
| 27 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | during_estimate_rms_m | 0.6084 | 0.54483 | -0.063575 | True | better |
| 37 | during_tracking_rms_m | 0.79797 | 0.73123 | -0.066742 | True | better |
| 37 | post_5s_estimate_rms_m | 0.58523 | 0.49721 | -0.088013 | True | better |
| 37 | post_5s_tracking_rms_m | 0.57201 | 0.48477 | -0.087235 | True | better |
| 37 | full_estimate_max_m | 0.84728 | 0.75195 | -0.095334 | True | better |
| 37 | full_tracking_max_m | 0.88413 | 0.80963 | -0.074498 | True | better |
| 37 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 37 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 37 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 37 | x_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 37 | x_clean_rejection_rate | 0 | 0.27419 | +0.27419 | True | worse |
| 37 | x_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 37 | x_post_all_clean_rejection_rate | 0 | 0.28116 | +0.28116 | True | worse |
| 37 | x_all_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 37 | y_clean_rejection_rate | 0 | 0.25806 | +0.25806 | True | worse |
| 37 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 37 | y_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 37 | y_all_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 37 | pooled_clean_rejection_rate | 0 | 0.26613 | +0.26613 | True | worse |
| 37 | pooled_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 37 | pooled_post_all_clean_rejection_rate | 0 | 0.2343 | +0.2343 | True | worse |
| 37 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | during_estimate_rms_m | 0.51947 | 0.53133 | +0.011858 | True | worse |
| 47 | during_tracking_rms_m | 0.66991 | 0.68126 | +0.011353 | True | worse |
| 47 | post_5s_estimate_rms_m | 0.59584 | 0.59502 | -0.00082504 | True | better |
| 47 | post_5s_tracking_rms_m | 0.58035 | 0.57949 | -0.00086079 | True | better |
| 47 | full_estimate_max_m | 0.73563 | 0.73526 | -0.00037458 | True | better |
| 47 | full_tracking_max_m | 0.81681 | 0.81705 | +0.00024334 | True | worse |
| 47 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 47 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 47 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 47 | x_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 47 | x_clean_rejection_rate | 0 | 0.16129 | +0.16129 | True | worse |
| 47 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_rejection_rate | 0 | 0.06248 | +0.06248 | True | worse |
| 47 | x_all_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 47 | y_clean_rejection_rate | 0 | 0.17742 | +0.17742 | True | worse |
| 47 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 47 | y_post_all_clean_rejection_rate | 0 | 0.24992 | +0.24992 | True | worse |
| 47 | y_all_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 47 | pooled_clean_rejection_rate | 0 | 0.16935 | +0.16935 | True | worse |
| 47 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 47 | pooled_post_all_clean_rejection_rate | 0 | 0.1562 | +0.1562 | True | worse |
| 47 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |

### Run 状态

| Seed | Mode | Duration s | Flight checks pass | Contacts | EKF resets | Joint recovery | Failed checks | Termination error |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 7 | gated | 62.0 | True | 0 | 0 | not_applicable |  | None |
| 7 | stock | 62.0 | True | 0 | 0 | not_applicable |  | None |
| 17 | gated | 62.0 | True | 0 | 0 | not_applicable |  | None |
| 17 | stock | 62.0 | True | 0 | 0 | not_applicable |  | None |
| 27 | gated | 62.0 | False | 0 | 0 | not_applicable | final_mean_target_error_below_50_cm | None |
| 27 | stock | 62.0 | False | 0 | 0 | not_applicable | final_mean_target_error_below_50_cm | None |
| 37 | gated | 62.0 | False | 0 | 0 | not_applicable | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 37 | stock | 62.0 | False | 0 | 0 | not_applicable | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 47 | gated | 62.0 | True | 0 | 0 | not_applicable |  | None |
| 47 | stock | 62.0 | True | 0 | 0 | not_applicable |  | None |

## spike 5

[该 case 的 per-seed CSV](per_seed_spike5.csv)

| Metric | Unit | Stock mean ± SD | Gated mean ± SD | Comparable paired Δ mean ± SD | Observed Δ mean ± SD (partial max) | Better / worse / tie / indeterminate (of 5) |
| --- | --- | --- | --- | --- | --- | --- |
| during_estimate_rms_m | m | 0.3761 ± 0.2074 (n=5) | 0.35298 ± 0.17649 (n=5) | -0.023118 ± 0.058946 (n=5) | same | 4 / 1 / 0 / 0 |
| during_tracking_rms_m | m | 0.45134 ± 0.29702 (n=5) | 0.42389 ± 0.27629 (n=5) | -0.027447 ± 0.032744 (n=5) | same | 3 / 2 / 0 / 0 |
| post_5s_estimate_rms_m | m | 0.38258 ± 0.23272 (n=5) | 0.37332 ± 0.16849 (n=5) | -0.0092633 ± 0.071109 (n=5) | same | 3 / 2 / 0 / 0 |
| post_5s_tracking_rms_m | m | 0.38906 ± 0.21502 (n=5) | 0.37892 ± 0.15301 (n=5) | -0.010134 ± 0.070383 (n=5) | same | 3 / 2 / 0 / 0 |
| full_estimate_max_m | m | 0.60261 ± 0.22787 (n=5); complete=5/5 | 0.60776 ± 0.18117 (n=5); complete=5/5 | 0.0051487 ± 0.075742 (n=5) | same | 2 / 3 / 0 / 0 |
| full_tracking_max_m | m | 0.69152 ± 0.19566 (n=5); complete=5/5 | 0.66088 ± 0.1911 (n=5); complete=5/5 | -0.030641 ± 0.071336 (n=5) | same | 4 / 1 / 0 / 0 |
| recovery_estimate_s | s | 6.0128 ± 8.555 (n=5) | 5.568 ± 9.9904 (n=5) | -0.4448 ± 3.7077 (n=5) | same | 1 / 2 / 2 / 0 |
| recovery_tracking_s | s | 0 ± 0 (n=5) | 1.7074 ± 3.8179 (n=5) | 1.7074 ± 3.8179 (n=5) | same | 0 / 1 / 4 / 0 |
| recovery_joint_s | s | 6.0128 ± 8.555 (n=5) | 5.568 ± 9.9904 (n=5) | -0.4448 ± 3.7077 (n=5) | same | 1 / 2 / 2 / 0 |
| x_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 25.682 ± 11.019 (n=5) | 25.682 ± 11.019 (n=5) | same | 5 / 0 / 0 / 0 |
| x_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25974 ± 0.05744 (n=5) | 0.25974 ± 0.05744 (n=5) | same | 0 / 5 / 0 / 0 |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.32 ± 0.43818 (n=5) | 0.32 ± 0.43818 (n=5) | same | 0 / 2 / 3 / 0 |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.21243 ± 0.097297 (n=5) | 0.21243 ± 0.097297 (n=5) | same | 0 / 5 / 0 / 0 |
| x_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 29.576 ± 5.1553 (n=5) | 29.576 ± 5.1553 (n=5) | same | 5 / 0 / 0 / 0 |
| y_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25976 ± 0.062009 (n=5) | 0.25976 ± 0.062009 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.44 ± 0.16733 (n=5) | 0.44 ± 0.16733 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.23118 ± 0.092674 (n=5) | 0.23118 ± 0.092674 (n=5) | same | 0 / 5 / 0 / 0 |
| y_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 27.629 ± 7.4885 (n=5) | 27.629 ± 7.4885 (n=5) | same | 5 / 0 / 0 / 0 |
| pooled_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25975 ± 0.056939 (n=5) | 0.25975 ± 0.056939 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.38 ± 0.21679 (n=5) | 0.38 ± 0.21679 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.22181 ± 0.072764 (n=5) | 0.22181 ± 0.072764 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |

### 每个 seed

| Seed | Metric | Stock | Gated | Δ | Comparable Δ | Outcome |
| --- | --- | --- | --- | --- | --- | --- |
| 7 | during_estimate_rms_m | 0.13665 | 0.21357 | +0.076926 | True | worse |
| 7 | during_tracking_rms_m | 0.10862 | 0.11132 | +0.0027049 | True | worse |
| 7 | post_5s_estimate_rms_m | 0.1004 | 0.17285 | +0.07245 | True | worse |
| 7 | post_5s_tracking_rms_m | 0.11653 | 0.18458 | +0.068047 | True | worse |
| 7 | full_estimate_max_m | 0.44732 | 0.49752 | +0.050197 | True | worse |
| 7 | full_tracking_max_m | 0.51921 | 0.48804 | -0.031172 | True | better |
| 7 | recovery_estimate_s | 2.82 | 4.79 | +1.97 | True | worse |
| 7 | recovery_tracking_s | 0 | 0 | +0 | True | tie |
| 7 | recovery_joint_s | 2.82 | 4.79 | +1.97 | True | worse |
| 7 | x_fault_active_rejection_rate | 0 | 44.186 | +44.186 | True | better |
| 7 | x_clean_rejection_rate | 0 | 0.27611 | +0.27611 | True | worse |
| 7 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 7 | x_all_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_fault_active_rejection_rate | 0 | 34.884 | +34.884 | True | better |
| 7 | y_clean_rejection_rate | 0 | 0.22738 | +0.22738 | True | worse |
| 7 | y_post_5s_clean_rejection_rate | 0 | 0.2 | +0.2 | True | worse |
| 7 | y_post_all_clean_rejection_rate | 0 | 0.093721 | +0.093721 | True | worse |
| 7 | y_all_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_fault_active_rejection_rate | 0 | 39.535 | +39.535 | True | better |
| 7 | pooled_clean_rejection_rate | 0 | 0.25175 | +0.25175 | True | worse |
| 7 | pooled_post_5s_clean_rejection_rate | 0 | 0.1 | +0.1 | True | worse |
| 7 | pooled_post_all_clean_rejection_rate | 0 | 0.14058 | +0.14058 | True | worse |
| 7 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | during_estimate_rms_m | 0.22785 | 0.19893 | -0.028925 | True | better |
| 17 | during_tracking_rms_m | 0.37264 | 0.31036 | -0.06228 | True | better |
| 17 | post_5s_estimate_rms_m | 0.24734 | 0.30733 | +0.059991 | True | worse |
| 17 | post_5s_tracking_rms_m | 0.27716 | 0.33893 | +0.061767 | True | worse |
| 17 | full_estimate_max_m | 0.30481 | 0.34407 | +0.039259 | True | worse |
| 17 | full_tracking_max_m | 0.5087 | 0.43073 | -0.077961 | True | better |
| 17 | recovery_estimate_s | 20.494 | 23.05 | +2.556 | True | worse |
| 17 | recovery_tracking_s | 0 | 8.537 | +8.537 | True | worse |
| 17 | recovery_joint_s | 20.494 | 23.05 | +2.556 | True | worse |
| 17 | x_fault_active_rejection_rate | 0 | 18.868 | +18.868 | True | better |
| 17 | x_clean_rejection_rate | 0 | 0.29283 | +0.29283 | True | worse |
| 17 | x_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 17 | x_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | x_all_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_fault_active_rejection_rate | 0 | 22.642 | +22.642 | True | better |
| 17 | y_clean_rejection_rate | 0 | 0.34163 | +0.34163 | True | worse |
| 17 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 17 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | y_all_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_fault_active_rejection_rate | 0 | 20.755 | +20.755 | True | better |
| 17 | pooled_clean_rejection_rate | 0 | 0.31723 | +0.31723 | True | worse |
| 17 | pooled_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 17 | pooled_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | during_estimate_rms_m | 0.34424 | 0.26755 | -0.076692 | True | better |
| 27 | during_tracking_rms_m | 0.26377 | 0.27334 | +0.0095695 | True | worse |
| 27 | post_5s_estimate_rms_m | 0.32372 | 0.29007 | -0.033656 | True | better |
| 27 | post_5s_tracking_rms_m | 0.34007 | 0.30427 | -0.035797 | True | better |
| 27 | full_estimate_max_m | 0.62401 | 0.71083 | +0.086826 | True | worse |
| 27 | full_tracking_max_m | 0.63601 | 0.72887 | +0.092862 | True | worse |
| 27 | recovery_estimate_s | 0 | 0 | +0 | True | tie |
| 27 | recovery_tracking_s | 0 | 0 | +0 | True | tie |
| 27 | recovery_joint_s | 0 | 0 | +0 | True | tie |
| 27 | x_fault_active_rejection_rate | 0 | 20 | +20 | True | better |
| 27 | x_clean_rejection_rate | 0 | 0.30819 | +0.30819 | True | worse |
| 27 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_rejection_rate | 0 | 0.21868 | +0.21868 | True | worse |
| 27 | x_all_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_fault_active_rejection_rate | 0 | 25.714 | +25.714 | True | better |
| 27 | y_clean_rejection_rate | 0 | 0.29197 | +0.29197 | True | worse |
| 27 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 27 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 27 | y_all_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_fault_active_rejection_rate | 0 | 22.857 | +22.857 | True | better |
| 27 | pooled_clean_rejection_rate | 0 | 0.30008 | +0.30008 | True | worse |
| 27 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 27 | pooled_post_all_clean_rejection_rate | 0 | 0.26554 | +0.26554 | True | worse |
| 27 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | during_estimate_rms_m | 0.62906 | 0.5767 | -0.052363 | True | better |
| 37 | during_tracking_rms_m | 0.82151 | 0.7662 | -0.055304 | True | better |
| 37 | post_5s_estimate_rms_m | 0.59065 | 0.52668 | -0.063968 | True | better |
| 37 | post_5s_tracking_rms_m | 0.5755 | 0.51137 | -0.064132 | True | better |
| 37 | full_estimate_max_m | 0.84367 | 0.77705 | -0.066619 | True | better |
| 37 | full_tracking_max_m | 0.93301 | 0.86378 | -0.069236 | True | better |
| 37 | recovery_estimate_s | 0 | 0 | +0 | True | tie |
| 37 | recovery_tracking_s | 0 | 0 | +0 | True | tie |
| 37 | recovery_joint_s | 0 | 0 | +0 | True | tie |
| 37 | x_fault_active_rejection_rate | 0 | 17.857 | +17.857 | True | better |
| 37 | x_clean_rejection_rate | 0 | 0.25924 | +0.25924 | True | worse |
| 37 | x_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 37 | x_post_all_clean_rejection_rate | 0 | 0.28116 | +0.28116 | True | worse |
| 37 | x_all_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_fault_active_rejection_rate | 0 | 32.143 | +32.143 | True | better |
| 37 | y_clean_rejection_rate | 0 | 0.25924 | +0.25924 | True | worse |
| 37 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 37 | y_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 37 | y_all_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_fault_active_rejection_rate | 0 | 25 | +25 | True | better |
| 37 | pooled_clean_rejection_rate | 0 | 0.25924 | +0.25924 | True | worse |
| 37 | pooled_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 37 | pooled_post_all_clean_rejection_rate | 0 | 0.2343 | +0.2343 | True | worse |
| 37 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | during_estimate_rms_m | 0.54268 | 0.50814 | -0.034538 | True | better |
| 47 | during_tracking_rms_m | 0.69015 | 0.65822 | -0.031927 | True | better |
| 47 | post_5s_estimate_rms_m | 0.65082 | 0.56969 | -0.081134 | True | better |
| 47 | post_5s_tracking_rms_m | 0.63601 | 0.55546 | -0.080555 | True | better |
| 47 | full_estimate_max_m | 0.79324 | 0.70932 | -0.083919 | True | better |
| 47 | full_tracking_max_m | 0.86067 | 0.79298 | -0.067696 | True | better |
| 47 | recovery_estimate_s | 6.75 | 0 | -6.75 | True | better |
| 47 | recovery_tracking_s | 0 | 0 | +0 | True | tie |
| 47 | recovery_joint_s | 6.75 | 0 | -6.75 | True | better |
| 47 | x_fault_active_rejection_rate | 0 | 27.5 | +27.5 | True | better |
| 47 | x_clean_rejection_rate | 0 | 0.16234 | +0.16234 | True | worse |
| 47 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_rejection_rate | 0 | 0.06248 | +0.06248 | True | worse |
| 47 | x_all_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_fault_active_rejection_rate | 0 | 32.5 | +32.5 | True | better |
| 47 | y_clean_rejection_rate | 0 | 0.17857 | +0.17857 | True | worse |
| 47 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 47 | y_post_all_clean_rejection_rate | 0 | 0.24992 | +0.24992 | True | worse |
| 47 | y_all_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_fault_active_rejection_rate | 0 | 30 | +30 | True | better |
| 47 | pooled_clean_rejection_rate | 0 | 0.17045 | +0.17045 | True | worse |
| 47 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 47 | pooled_post_all_clean_rejection_rate | 0 | 0.1562 | +0.1562 | True | worse |
| 47 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |

### Run 状态

| Seed | Mode | Duration s | Flight checks pass | Contacts | EKF resets | Joint recovery | Failed checks | Termination error |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 7 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 7 | stock | 62.0 | True | 0 | 0 | recovered |  | None |
| 17 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 17 | stock | 62.0 | True | 0 | 0 | recovered |  | None |
| 27 | gated | 62.0 | False | 0 | 0 | recovered | final_mean_target_error_below_50_cm | None |
| 27 | stock | 62.0 | False | 0 | 0 | recovered | final_mean_target_error_below_50_cm | None |
| 37 | gated | 62.0 | False | 0 | 0 | recovered | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 37 | stock | 62.0 | False | 0 | 0 | recovered | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 47 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 47 | stock | 62.0 | True | 0 | 0 | recovered |  | None |

## spike 10

[该 case 的 per-seed CSV](per_seed_spike10.csv)

| Metric | Unit | Stock mean ± SD | Gated mean ± SD | Comparable paired Δ mean ± SD | Observed Δ mean ± SD (partial max) | Better / worse / tie / indeterminate (of 5) |
| --- | --- | --- | --- | --- | --- | --- |
| during_estimate_rms_m | m | 0.40169 ± 0.20396 (n=5) | 0.33894 ± 0.18197 (n=5) | -0.062747 ± 0.059448 (n=5) | same | 4 / 1 / 0 / 0 |
| during_tracking_rms_m | m | 0.46738 ± 0.30584 (n=5) | 0.4239 ± 0.26572 (n=5) | -0.043477 ± 0.08591 (n=5) | same | 4 / 1 / 0 / 0 |
| post_5s_estimate_rms_m | m | 0.42866 ± 0.22151 (n=5) | 0.34509 ± 0.1805 (n=5) | -0.083566 ± 0.060467 (n=5) | same | 5 / 0 / 0 / 0 |
| post_5s_tracking_rms_m | m | 0.43275 ± 0.20702 (n=5) | 0.35119 ± 0.16336 (n=5) | -0.081554 ± 0.05629 (n=5) | same | 5 / 0 / 0 / 0 |
| full_estimate_max_m | m | 0.6303 ± 0.20712 (n=5); complete=5/5 | 0.59459 ± 0.20974 (n=5); complete=5/5 | -0.035712 ± 0.15215 (n=5) | same | 3 / 2 / 0 / 0 |
| full_tracking_max_m | m | 0.71059 ± 0.21796 (n=5); complete=5/5 | 0.66335 ± 0.18034 (n=5); complete=5/5 | -0.047235 ± 0.15983 (n=5) | same | 4 / 1 / 0 / 0 |
| recovery_estimate_s | s | 7.1075 ± 11.112 (n=4) | 1.41 ± 3.1529 (n=5) | -5.345 ± 7.7054 (n=4) | same | 3 / 0 / 2 / 0 |
| recovery_tracking_s | s | 5.2254 ± 6.0204 (n=5) | 0 ± 0 (n=5) | -5.2254 ± 6.0204 (n=5) | same | 3 / 0 / 2 / 0 |
| recovery_joint_s | s | 7.1075 ± 11.112 (n=4) | 1.41 ± 3.1529 (n=5) | -5.345 ± 7.7054 (n=4) | same | 3 / 0 / 2 / 0 |
| x_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 98.929 ± 1.4725 (n=5) | 98.929 ± 1.4725 (n=5) | same | 5 / 0 / 0 / 0 |
| x_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25648 ± 0.055477 (n=5) | 0.25648 ± 0.055477 (n=5) | same | 0 / 5 / 0 / 0 |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.32 ± 0.43818 (n=5) | 0.32 ± 0.43818 (n=5) | same | 0 / 2 / 3 / 0 |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.21243 ± 0.097297 (n=5) | 0.21243 ± 0.097297 (n=5) | same | 0 / 5 / 0 / 0 |
| x_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 97.943 ± 1.306 (n=5) | 97.943 ± 1.306 (n=5) | same | 5 / 0 / 0 / 0 |
| y_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25326 ± 0.06769 (n=5) | 0.25326 ± 0.06769 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.44 ± 0.16733 (n=5) | 0.44 ± 0.16733 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.23118 ± 0.092674 (n=5) | 0.23118 ± 0.092674 (n=5) | same | 0 / 5 / 0 / 0 |
| y_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 98.436 ± 0.61027 (n=5) | 98.436 ± 0.61027 (n=5) | same | 5 / 0 / 0 / 0 |
| pooled_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25487 ± 0.057055 (n=5) | 0.25487 ± 0.057055 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.38 ± 0.21679 (n=5) | 0.38 ± 0.21679 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.22181 ± 0.072764 (n=5) | 0.22181 ± 0.072764 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |

### 每个 seed

| Seed | Metric | Stock | Gated | Δ | Comparable Δ | Outcome |
| --- | --- | --- | --- | --- | --- | --- |
| 7 | during_estimate_rms_m | 0.15419 | 0.17451 | +0.020315 | True | worse |
| 7 | during_tracking_rms_m | 0.13095 | 0.092064 | -0.038888 | True | better |
| 7 | post_5s_estimate_rms_m | 0.16404 | 0.13044 | -0.033604 | True | better |
| 7 | post_5s_tracking_rms_m | 0.17476 | 0.14496 | -0.029793 | True | better |
| 7 | full_estimate_max_m | 0.48888 | 0.50035 | +0.011471 | True | worse |
| 7 | full_tracking_max_m | 0.52827 | 0.50633 | -0.021936 | True | better |
| 7 | recovery_estimate_s | 5.04 | 0 | -5.04 | True | better |
| 7 | recovery_tracking_s | 2.765 | 0 | -2.765 | True | better |
| 7 | recovery_joint_s | 5.04 | 0 | -5.04 | True | better |
| 7 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 7 | x_clean_rejection_rate | 0 | 0.25987 | +0.25987 | True | worse |
| 7 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 7 | x_all_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_fault_active_rejection_rate | 0 | 97.674 | +97.674 | True | better |
| 7 | y_clean_rejection_rate | 0 | 0.1949 | +0.1949 | True | worse |
| 7 | y_post_5s_clean_rejection_rate | 0 | 0.2 | +0.2 | True | worse |
| 7 | y_post_all_clean_rejection_rate | 0 | 0.093721 | +0.093721 | True | worse |
| 7 | y_all_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_fault_active_rejection_rate | 0 | 98.837 | +98.837 | True | better |
| 7 | pooled_clean_rejection_rate | 0 | 0.22738 | +0.22738 | True | worse |
| 7 | pooled_post_5s_clean_rejection_rate | 0 | 0.1 | +0.1 | True | worse |
| 7 | pooled_post_all_clean_rejection_rate | 0 | 0.14058 | +0.14058 | True | worse |
| 7 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | during_estimate_rms_m | 0.29072 | 0.16722 | -0.1235 | True | better |
| 17 | during_tracking_rms_m | 0.41405 | 0.30836 | -0.10569 | True | better |
| 17 | post_5s_estimate_rms_m | 0.3468 | 0.22714 | -0.11967 | True | better |
| 17 | post_5s_tracking_rms_m | 0.36788 | 0.26182 | -0.10607 | True | better |
| 17 | full_estimate_max_m | 0.40196 | 0.26938 | -0.13258 | True | better |
| 17 | full_tracking_max_m | 0.55377 | 0.42977 | -0.124 | True | better |
| 17 | recovery_estimate_s | 23.39 | 7.05 | -16.34 | True | better |
| 17 | recovery_tracking_s | 12.376 | 0 | -12.376 | True | better |
| 17 | recovery_joint_s | 23.39 | 7.05 | -16.34 | True | better |
| 17 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 17 | x_clean_rejection_rate | 0 | 0.27656 | +0.27656 | True | worse |
| 17 | x_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 17 | x_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | x_all_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_fault_active_rejection_rate | 0 | 98.113 | +98.113 | True | better |
| 17 | y_clean_rejection_rate | 0 | 0.34163 | +0.34163 | True | worse |
| 17 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 17 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | y_all_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_fault_active_rejection_rate | 0 | 99.057 | +99.057 | True | better |
| 17 | pooled_clean_rejection_rate | 0 | 0.30909 | +0.30909 | True | worse |
| 17 | pooled_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 17 | pooled_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | during_estimate_rms_m | 0.34491 | 0.29134 | -0.053572 | True | better |
| 27 | during_tracking_rms_m | 0.23345 | 0.3264 | +0.09296 | True | worse |
| 27 | post_5s_estimate_rms_m | 0.32368 | 0.32007 | -0.0036064 | True | better |
| 27 | post_5s_tracking_rms_m | 0.34233 | 0.32924 | -0.013094 | True | better |
| 27 | full_estimate_max_m | 0.56424 | 0.77626 | +0.21202 | True | worse |
| 27 | full_tracking_max_m | 0.57697 | 0.79432 | +0.21736 | True | worse |
| 27 | recovery_estimate_s | 0 | 0 | +0 | True | tie |
| 27 | recovery_tracking_s | 0 | 0 | +0 | True | tie |
| 27 | recovery_joint_s | 0 | 0 | +0 | True | tie |
| 27 | x_fault_active_rejection_rate | 0 | 97.143 | +97.143 | True | better |
| 27 | x_clean_rejection_rate | 0 | 0.30819 | +0.30819 | True | worse |
| 27 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_rejection_rate | 0 | 0.21868 | +0.21868 | True | worse |
| 27 | x_all_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 27 | y_clean_rejection_rate | 0 | 0.29197 | +0.29197 | True | worse |
| 27 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 27 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 27 | y_all_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_fault_active_rejection_rate | 0 | 98.571 | +98.571 | True | better |
| 27 | pooled_clean_rejection_rate | 0 | 0.30008 | +0.30008 | True | worse |
| 27 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 27 | pooled_post_all_clean_rejection_rate | 0 | 0.26554 | +0.26554 | True | worse |
| 27 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | during_estimate_rms_m | 0.6505 | 0.5335 | -0.117 | True | better |
| 37 | during_tracking_rms_m | 0.84599 | 0.71889 | -0.1271 | True | better |
| 37 | post_5s_estimate_rms_m | 0.59682 | 0.47024 | -0.12658 | True | better |
| 37 | post_5s_tracking_rms_m | 0.57985 | 0.45842 | -0.12143 | True | better |
| 37 | full_estimate_max_m | 0.84045 | 0.70839 | -0.13206 | True | better |
| 37 | full_tracking_max_m | 0.98295 | 0.79089 | -0.19206 | True | better |
| 37 | recovery_estimate_s | 0 | 0 | +0 | True | tie |
| 37 | recovery_tracking_s | 0 | 0 | +0 | True | tie |
| 37 | recovery_joint_s | 0 | 0 | +0 | True | tie |
| 37 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 37 | x_clean_rejection_rate | 0 | 0.27544 | +0.27544 | True | worse |
| 37 | x_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 37 | x_post_all_clean_rejection_rate | 0 | 0.28116 | +0.28116 | True | worse |
| 37 | x_all_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_fault_active_rejection_rate | 0 | 96.429 | +96.429 | True | better |
| 37 | y_clean_rejection_rate | 0 | 0.25924 | +0.25924 | True | worse |
| 37 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 37 | y_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 37 | y_all_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_fault_active_rejection_rate | 0 | 98.214 | +98.214 | True | better |
| 37 | pooled_clean_rejection_rate | 0 | 0.26734 | +0.26734 | True | worse |
| 37 | pooled_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 37 | pooled_post_all_clean_rejection_rate | 0 | 0.2343 | +0.2343 | True | worse |
| 37 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | during_estimate_rms_m | 0.56814 | 0.52816 | -0.03998 | True | better |
| 47 | during_tracking_rms_m | 0.71246 | 0.67379 | -0.038672 | True | better |
| 47 | post_5s_estimate_rms_m | 0.71195 | 0.57757 | -0.13438 | True | better |
| 47 | post_5s_tracking_rms_m | 0.69891 | 0.56153 | -0.13738 | True | better |
| 47 | full_estimate_max_m | 0.85598 | 0.71857 | -0.13741 | True | better |
| 47 | full_tracking_max_m | 0.91099 | 0.79546 | -0.11554 | True | better |
| 47 | recovery_estimate_s | NA | 0 | NA | False | better |
| 47 | recovery_tracking_s | 10.986 | 0 | -10.986 | True | better |
| 47 | recovery_joint_s | NA | 0 | NA | False | better |
| 47 | x_fault_active_rejection_rate | 0 | 97.5 | +97.5 | True | better |
| 47 | x_clean_rejection_rate | 0 | 0.16234 | +0.16234 | True | worse |
| 47 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_rejection_rate | 0 | 0.06248 | +0.06248 | True | worse |
| 47 | x_all_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_fault_active_rejection_rate | 0 | 97.5 | +97.5 | True | better |
| 47 | y_clean_rejection_rate | 0 | 0.17857 | +0.17857 | True | worse |
| 47 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 47 | y_post_all_clean_rejection_rate | 0 | 0.24992 | +0.24992 | True | worse |
| 47 | y_all_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_fault_active_rejection_rate | 0 | 97.5 | +97.5 | True | better |
| 47 | pooled_clean_rejection_rate | 0 | 0.17045 | +0.17045 | True | worse |
| 47 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 47 | pooled_post_all_clean_rejection_rate | 0 | 0.1562 | +0.1562 | True | worse |
| 47 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |

### Run 状态

| Seed | Mode | Duration s | Flight checks pass | Contacts | EKF resets | Joint recovery | Failed checks | Termination error |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 7 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 7 | stock | 62.0 | True | 0 | 0 | recovered |  | None |
| 17 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 17 | stock | 62.0 | True | 0 | 0 | recovered |  | None |
| 27 | gated | 62.0 | False | 0 | 0 | recovered | final_mean_target_error_below_50_cm | None |
| 27 | stock | 62.0 | False | 0 | 0 | recovered | final_mean_target_error_below_50_cm | None |
| 37 | gated | 62.0 | False | 0 | 0 | recovered | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 37 | stock | 62.0 | False | 0 | 0 | recovered | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 47 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 47 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |

## spike 20

[该 case 的 per-seed CSV](per_seed_spike20.csv)

| Metric | Unit | Stock mean ± SD | Gated mean ± SD | Comparable paired Δ mean ± SD | Observed Δ mean ± SD (partial max) | Better / worse / tie / indeterminate (of 5) |
| --- | --- | --- | --- | --- | --- | --- |
| during_estimate_rms_m | m | 0.47211 ± 0.19038 (n=5) | 0.33644 ± 0.18454 (n=5) | -0.13568 ± 0.088776 (n=5) | same | 5 / 0 / 0 / 0 |
| during_tracking_rms_m | m | 0.52901 ± 0.30518 (n=5) | 0.42168 ± 0.26724 (n=5) | -0.10733 ± 0.11911 (n=5) | same | 4 / 1 / 0 / 0 |
| post_5s_estimate_rms_m | m | 0.53877 ± 0.21346 (n=5) | 0.34165 ± 0.1867 (n=5) | -0.19712 ± 0.11108 (n=5) | same | 5 / 0 / 0 / 0 |
| post_5s_tracking_rms_m | m | 0.54059 ± 0.20347 (n=5) | 0.3485 ± 0.16893 (n=5) | -0.19209 ± 0.10207 (n=5) | same | 5 / 0 / 0 / 0 |
| full_estimate_max_m | m | 0.70318 ± 0.20736 (n=5); complete=5/5 | 0.5915 ± 0.21656 (n=5); complete=5/5 | -0.11167 ± 0.24564 (n=5) | same | 4 / 1 / 0 / 0 |
| full_tracking_max_m | m | 0.79612 ± 0.24214 (n=5); complete=5/5 | 0.66257 ± 0.18459 (n=5); complete=5/5 | -0.13354 ± 0.19804 (n=5) | same | 4 / 1 / 0 / 0 |
| recovery_estimate_s | s | 0 ± 0 (n=2) | 0.3362 ± 0.75177 (n=5) | 0 ± 0 (n=2) | same | 3 / 0 / 2 / 0 |
| recovery_tracking_s | s | 2.9097 ± 5.0397 (n=3) | 0 ± 0 (n=5) | -2.9097 ± 5.0397 (n=3) | same | 3 / 0 / 2 / 0 |
| recovery_joint_s | s | 0 ± 0 (n=2) | 0.3362 ± 0.75177 (n=5) | 0 ± 0 (n=2) | same | 3 / 0 / 2 / 0 |
| x_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 100 ± 0 (n=5) | 100 ± 0 (n=5) | same | 5 / 0 / 0 / 0 |
| x_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25648 ± 0.055477 (n=5) | 0.25648 ± 0.055477 (n=5) | same | 0 / 5 / 0 / 0 |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.32 ± 0.43818 (n=5) | 0.32 ± 0.43818 (n=5) | same | 0 / 2 / 3 / 0 |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.21243 ± 0.097297 (n=5) | 0.21243 ± 0.097297 (n=5) | same | 0 / 5 / 0 / 0 |
| x_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 100 ± 0 (n=5) | 100 ± 0 (n=5) | same | 5 / 0 / 0 / 0 |
| y_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25326 ± 0.06769 (n=5) | 0.25326 ± 0.06769 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.44 ± 0.16733 (n=5) | 0.44 ± 0.16733 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.23118 ± 0.092674 (n=5) | 0.23118 ± 0.092674 (n=5) | same | 0 / 5 / 0 / 0 |
| y_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 100 ± 0 (n=5) | 100 ± 0 (n=5) | same | 5 / 0 / 0 / 0 |
| pooled_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25487 ± 0.057055 (n=5) | 0.25487 ± 0.057055 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.38 ± 0.21679 (n=5) | 0.38 ± 0.21679 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.22181 ± 0.072764 (n=5) | 0.22181 ± 0.072764 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |

### 每个 seed

| Seed | Metric | Stock | Gated | Δ | Comparable Δ | Outcome |
| --- | --- | --- | --- | --- | --- | --- |
| 7 | during_estimate_rms_m | 0.2216 | 0.17361 | -0.04799 | True | better |
| 7 | during_tracking_rms_m | 0.21549 | 0.090762 | -0.12473 | True | better |
| 7 | post_5s_estimate_rms_m | 0.31075 | 0.12214 | -0.18861 | True | better |
| 7 | post_5s_tracking_rms_m | 0.31757 | 0.13787 | -0.1797 | True | better |
| 7 | full_estimate_max_m | 0.59104 | 0.49279 | -0.098247 | True | better |
| 7 | full_tracking_max_m | 0.58292 | 0.5035 | -0.079422 | True | better |
| 7 | recovery_estimate_s | NA | 0 | NA | False | better |
| 7 | recovery_tracking_s | 8.729 | 0 | -8.729 | True | better |
| 7 | recovery_joint_s | NA | 0 | NA | False | better |
| 7 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 7 | x_clean_rejection_rate | 0 | 0.25987 | +0.25987 | True | worse |
| 7 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 7 | x_all_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 7 | y_clean_rejection_rate | 0 | 0.1949 | +0.1949 | True | worse |
| 7 | y_post_5s_clean_rejection_rate | 0 | 0.2 | +0.2 | True | worse |
| 7 | y_post_all_clean_rejection_rate | 0 | 0.093721 | +0.093721 | True | worse |
| 7 | y_all_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 7 | pooled_clean_rejection_rate | 0 | 0.22738 | +0.22738 | True | worse |
| 7 | pooled_post_5s_clean_rejection_rate | 0 | 0.1 | +0.1 | True | worse |
| 7 | pooled_post_all_clean_rejection_rate | 0 | 0.14058 | +0.14058 | True | worse |
| 7 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | during_estimate_rms_m | 0.43246 | 0.15569 | -0.27677 | True | better |
| 17 | during_tracking_rms_m | 0.53 | 0.30181 | -0.22818 | True | better |
| 17 | post_5s_estimate_rms_m | 0.55366 | 0.21456 | -0.3391 | True | better |
| 17 | post_5s_tracking_rms_m | 0.56432 | 0.25105 | -0.31327 | True | better |
| 17 | full_estimate_max_m | 0.60747 | 0.25552 | -0.35196 | True | better |
| 17 | full_tracking_max_m | 0.68228 | 0.42239 | -0.25989 | True | better |
| 17 | recovery_estimate_s | NA | 1.681 | NA | False | better |
| 17 | recovery_tracking_s | NA | 0 | NA | False | better |
| 17 | recovery_joint_s | NA | 1.681 | NA | False | better |
| 17 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 17 | x_clean_rejection_rate | 0 | 0.27656 | +0.27656 | True | worse |
| 17 | x_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 17 | x_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | x_all_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 17 | y_clean_rejection_rate | 0 | 0.34163 | +0.34163 | True | worse |
| 17 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 17 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | y_all_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 17 | pooled_clean_rejection_rate | 0 | 0.30909 | +0.30909 | True | worse |
| 17 | pooled_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 17 | pooled_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | during_estimate_rms_m | 0.38605 | 0.29247 | -0.093576 | True | better |
| 27 | during_tracking_rms_m | 0.23939 | 0.32285 | +0.083465 | True | worse |
| 27 | post_5s_estimate_rms_m | 0.36942 | 0.32083 | -0.048596 | True | better |
| 27 | post_5s_tracking_rms_m | 0.38981 | 0.3305 | -0.059314 | True | better |
| 27 | full_estimate_max_m | 0.48583 | 0.77187 | +0.28605 | True | worse |
| 27 | full_tracking_max_m | 0.60078 | 0.78994 | +0.18916 | True | worse |
| 27 | recovery_estimate_s | 0 | 0 | +0 | True | tie |
| 27 | recovery_tracking_s | 0 | 0 | +0 | True | tie |
| 27 | recovery_joint_s | 0 | 0 | +0 | True | tie |
| 27 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 27 | x_clean_rejection_rate | 0 | 0.30819 | +0.30819 | True | worse |
| 27 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_rejection_rate | 0 | 0.21868 | +0.21868 | True | worse |
| 27 | x_all_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 27 | y_clean_rejection_rate | 0 | 0.29197 | +0.29197 | True | worse |
| 27 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 27 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 27 | y_all_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 27 | pooled_clean_rejection_rate | 0 | 0.30008 | +0.30008 | True | worse |
| 27 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 27 | pooled_post_all_clean_rejection_rate | 0 | 0.26554 | +0.26554 | True | worse |
| 27 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | during_estimate_rms_m | 0.69534 | 0.53297 | -0.16237 | True | better |
| 37 | during_tracking_rms_m | 0.89748 | 0.71884 | -0.17863 | True | better |
| 37 | post_5s_estimate_rms_m | 0.61123 | 0.46554 | -0.14569 | True | better |
| 37 | post_5s_tracking_rms_m | 0.59052 | 0.45322 | -0.13731 | True | better |
| 37 | full_estimate_max_m | 0.83633 | 0.71224 | -0.12409 | True | better |
| 37 | full_tracking_max_m | 1.0849 | 0.79089 | -0.29396 | True | better |
| 37 | recovery_estimate_s | 0 | 0 | +0 | True | tie |
| 37 | recovery_tracking_s | 0 | 0 | +0 | True | tie |
| 37 | recovery_joint_s | 0 | 0 | +0 | True | tie |
| 37 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 37 | x_clean_rejection_rate | 0 | 0.27544 | +0.27544 | True | worse |
| 37 | x_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 37 | x_post_all_clean_rejection_rate | 0 | 0.28116 | +0.28116 | True | worse |
| 37 | x_all_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 37 | y_clean_rejection_rate | 0 | 0.25924 | +0.25924 | True | worse |
| 37 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 37 | y_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 37 | y_all_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 37 | pooled_clean_rejection_rate | 0 | 0.26734 | +0.26734 | True | worse |
| 37 | pooled_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 37 | pooled_post_all_clean_rejection_rate | 0 | 0.2343 | +0.2343 | True | worse |
| 37 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | during_estimate_rms_m | 0.62511 | 0.52743 | -0.097679 | True | better |
| 47 | during_tracking_rms_m | 0.76268 | 0.67412 | -0.088558 | True | better |
| 47 | post_5s_estimate_rms_m | 0.84879 | 0.58519 | -0.2636 | True | better |
| 47 | post_5s_tracking_rms_m | 0.84072 | 0.56986 | -0.27087 | True | better |
| 47 | full_estimate_max_m | 0.99521 | 0.72511 | -0.27011 | True | better |
| 47 | full_tracking_max_m | 1.0298 | 0.80616 | -0.22361 | True | better |
| 47 | recovery_estimate_s | NA | 0 | NA | False | better |
| 47 | recovery_tracking_s | NA | 0 | NA | False | better |
| 47 | recovery_joint_s | NA | 0 | NA | False | better |
| 47 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 47 | x_clean_rejection_rate | 0 | 0.16234 | +0.16234 | True | worse |
| 47 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_rejection_rate | 0 | 0.06248 | +0.06248 | True | worse |
| 47 | x_all_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 47 | y_clean_rejection_rate | 0 | 0.17857 | +0.17857 | True | worse |
| 47 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 47 | y_post_all_clean_rejection_rate | 0 | 0.24992 | +0.24992 | True | worse |
| 47 | y_all_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 47 | pooled_clean_rejection_rate | 0 | 0.17045 | +0.17045 | True | worse |
| 47 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 47 | pooled_post_all_clean_rejection_rate | 0 | 0.1562 | +0.1562 | True | worse |
| 47 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |

### Run 状态

| Seed | Mode | Duration s | Flight checks pass | Contacts | EKF resets | Joint recovery | Failed checks | Termination error |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 7 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 7 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 17 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 17 | stock | 62.0 | True | 0 | 0 | censored |  | None |
| 27 | gated | 62.0 | False | 0 | 0 | recovered | final_mean_target_error_below_50_cm | None |
| 27 | stock | 62.0 | True | 0 | 0 | recovered |  | None |
| 37 | gated | 62.0 | False | 0 | 0 | recovered | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 37 | stock | 62.0 | False | 0 | 0 | recovered | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 47 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 47 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |

## spike 50

[该 case 的 per-seed CSV](per_seed_spike50.csv)

| Metric | Unit | Stock mean ± SD | Gated mean ± SD | Comparable paired Δ mean ± SD | Observed Δ mean ± SD (partial max) | Better / worse / tie / indeterminate (of 5) |
| --- | --- | --- | --- | --- | --- | --- |
| during_estimate_rms_m | m | 0.76268 ± 0.16325 (n=5) | 0.33644 ± 0.18454 (n=5) | -0.42624 ± 0.19736 (n=5) | same | 5 / 0 / 0 / 0 |
| during_tracking_rms_m | m | 0.84482 ± 0.23535 (n=5) | 0.42168 ± 0.26724 (n=5) | -0.42314 ± 0.17737 (n=5) | same | 5 / 0 / 0 / 0 |
| post_5s_estimate_rms_m | m | 0.94992 ± 0.32458 (n=5) | 0.34165 ± 0.1867 (n=5) | -0.60827 ± 0.33106 (n=5) | same | 5 / 0 / 0 / 0 |
| post_5s_tracking_rms_m | m | 0.95602 ± 0.33351 (n=5) | 0.3485 ± 0.16893 (n=5) | -0.60752 ± 0.33045 (n=5) | same | 5 / 0 / 0 / 0 |
| full_estimate_max_m | m | 1.2006 ± 0.2453 (n=5); complete=5/5 | 0.5915 ± 0.21656 (n=5); complete=5/5 | -0.60909 ± 0.37201 (n=5) | same | 5 / 0 / 0 / 0 |
| full_tracking_max_m | m | 1.3132 ± 0.31803 (n=5); complete=5/5 | 0.66257 ± 0.18459 (n=5); complete=5/5 | -0.65064 ± 0.37643 (n=5) | same | 5 / 0 / 0 / 0 |
| recovery_estimate_s | s | 15.97 ± 2.1213 (n=2) | 0.3362 ± 0.75177 (n=5) | -15.97 ± 2.1213 (n=2) | same | 5 / 0 / 0 / 0 |
| recovery_tracking_s | s | 6.9555 ± 8.3615 (n=2) | 0 ± 0 (n=5) | -6.9555 ± 8.3615 (n=2) | same | 5 / 0 / 0 / 0 |
| recovery_joint_s | s | 15.97 ± 2.1213 (n=2) | 0.3362 ± 0.75177 (n=5) | -15.97 ± 2.1213 (n=2) | same | 5 / 0 / 0 / 0 |
| x_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 100 ± 0 (n=5) | 100 ± 0 (n=5) | same | 5 / 0 / 0 / 0 |
| x_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25648 ± 0.055477 (n=5) | 0.25648 ± 0.055477 (n=5) | same | 0 / 5 / 0 / 0 |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.32 ± 0.43818 (n=5) | 0.32 ± 0.43818 (n=5) | same | 0 / 2 / 3 / 0 |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.21243 ± 0.097297 (n=5) | 0.21243 ± 0.097297 (n=5) | same | 0 / 5 / 0 / 0 |
| x_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 100 ± 0 (n=5) | 100 ± 0 (n=5) | same | 5 / 0 / 0 / 0 |
| y_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25326 ± 0.06769 (n=5) | 0.25326 ± 0.06769 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.44 ± 0.16733 (n=5) | 0.44 ± 0.16733 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.23118 ± 0.092674 (n=5) | 0.23118 ± 0.092674 (n=5) | same | 0 / 5 / 0 / 0 |
| y_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 100 ± 0 (n=5) | 100 ± 0 (n=5) | same | 5 / 0 / 0 / 0 |
| pooled_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25487 ± 0.057055 (n=5) | 0.25487 ± 0.057055 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.38 ± 0.21679 (n=5) | 0.38 ± 0.21679 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.22181 ± 0.072764 (n=5) | 0.22181 ± 0.072764 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |

### 每个 seed

| Seed | Metric | Stock | Gated | Δ | Comparable Δ | Outcome |
| --- | --- | --- | --- | --- | --- | --- |
| 7 | during_estimate_rms_m | 0.50632 | 0.17361 | -0.33271 | True | better |
| 7 | during_tracking_rms_m | 0.59104 | 0.090762 | -0.50028 | True | better |
| 7 | post_5s_estimate_rms_m | 0.78282 | 0.12214 | -0.66068 | True | better |
| 7 | post_5s_tracking_rms_m | 0.7943 | 0.13787 | -0.65643 | True | better |
| 7 | full_estimate_max_m | 0.96198 | 0.49279 | -0.46919 | True | better |
| 7 | full_tracking_max_m | 1.0055 | 0.5035 | -0.50203 | True | better |
| 7 | recovery_estimate_s | NA | 0 | NA | False | better |
| 7 | recovery_tracking_s | NA | 0 | NA | False | better |
| 7 | recovery_joint_s | NA | 0 | NA | False | better |
| 7 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 7 | x_clean_rejection_rate | 0 | 0.25987 | +0.25987 | True | worse |
| 7 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 7 | x_all_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 7 | y_clean_rejection_rate | 0 | 0.1949 | +0.1949 | True | worse |
| 7 | y_post_5s_clean_rejection_rate | 0 | 0.2 | +0.2 | True | worse |
| 7 | y_post_all_clean_rejection_rate | 0 | 0.093721 | +0.093721 | True | worse |
| 7 | y_all_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 7 | pooled_clean_rejection_rate | 0 | 0.22738 | +0.22738 | True | worse |
| 7 | pooled_post_5s_clean_rejection_rate | 0 | 0.1 | +0.1 | True | worse |
| 7 | pooled_post_all_clean_rejection_rate | 0 | 0.14058 | +0.14058 | True | worse |
| 7 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | during_estimate_rms_m | 0.92606 | 0.15569 | -0.77036 | True | better |
| 17 | during_tracking_rms_m | 0.99828 | 0.30181 | -0.69647 | True | better |
| 17 | post_5s_estimate_rms_m | 1.261 | 0.21456 | -1.0464 | True | better |
| 17 | post_5s_tracking_rms_m | 1.2772 | 0.25105 | -1.0261 | True | better |
| 17 | full_estimate_max_m | 1.4335 | 0.25552 | -1.1779 | True | better |
| 17 | full_tracking_max_m | 1.6016 | 0.42239 | -1.1792 | True | better |
| 17 | recovery_estimate_s | NA | 1.681 | NA | False | better |
| 17 | recovery_tracking_s | NA | 0 | NA | False | better |
| 17 | recovery_joint_s | NA | 1.681 | NA | False | better |
| 17 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 17 | x_clean_rejection_rate | 0 | 0.27656 | +0.27656 | True | worse |
| 17 | x_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 17 | x_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | x_all_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 17 | y_clean_rejection_rate | 0 | 0.34163 | +0.34163 | True | worse |
| 17 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 17 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | y_all_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 17 | pooled_clean_rejection_rate | 0 | 0.30909 | +0.30909 | True | worse |
| 17 | pooled_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 17 | pooled_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | during_estimate_rms_m | 0.70687 | 0.29247 | -0.41439 | True | better |
| 27 | during_tracking_rms_m | 0.59016 | 0.32285 | -0.2673 | True | better |
| 27 | post_5s_estimate_rms_m | 0.70698 | 0.32083 | -0.38615 | True | better |
| 27 | post_5s_tracking_rms_m | 0.72126 | 0.3305 | -0.39076 | True | better |
| 27 | full_estimate_max_m | 0.99386 | 0.77187 | -0.22198 | True | better |
| 27 | full_tracking_max_m | 0.95068 | 0.78994 | -0.16074 | True | better |
| 27 | recovery_estimate_s | 17.47 | 0 | -17.47 | True | better |
| 27 | recovery_tracking_s | 12.868 | 0 | -12.868 | True | better |
| 27 | recovery_joint_s | 17.47 | 0 | -17.47 | True | better |
| 27 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 27 | x_clean_rejection_rate | 0 | 0.30819 | +0.30819 | True | worse |
| 27 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_rejection_rate | 0 | 0.21868 | +0.21868 | True | worse |
| 27 | x_all_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 27 | y_clean_rejection_rate | 0 | 0.29197 | +0.29197 | True | worse |
| 27 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 27 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 27 | y_all_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 27 | pooled_clean_rejection_rate | 0 | 0.30008 | +0.30008 | True | worse |
| 27 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 27 | pooled_post_all_clean_rejection_rate | 0 | 0.26554 | +0.26554 | True | worse |
| 27 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | during_estimate_rms_m | 0.83793 | 0.53297 | -0.30496 | True | better |
| 37 | during_tracking_rms_m | 1.0759 | 0.71884 | -0.35709 | True | better |
| 37 | post_5s_estimate_rms_m | 0.65823 | 0.46554 | -0.1927 | True | better |
| 37 | post_5s_tracking_rms_m | 0.63474 | 0.45322 | -0.18153 | True | better |
| 37 | full_estimate_max_m | 1.1276 | 0.71224 | -0.41541 | True | better |
| 37 | full_tracking_max_m | 1.3983 | 0.79089 | -0.60744 | True | better |
| 37 | recovery_estimate_s | 14.47 | 0 | -14.47 | True | better |
| 37 | recovery_tracking_s | 1.043 | 0 | -1.043 | True | better |
| 37 | recovery_joint_s | 14.47 | 0 | -14.47 | True | better |
| 37 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 37 | x_clean_rejection_rate | 0 | 0.27544 | +0.27544 | True | worse |
| 37 | x_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 37 | x_post_all_clean_rejection_rate | 0 | 0.28116 | +0.28116 | True | worse |
| 37 | x_all_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 37 | y_clean_rejection_rate | 0 | 0.25924 | +0.25924 | True | worse |
| 37 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 37 | y_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 37 | y_all_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 37 | pooled_clean_rejection_rate | 0 | 0.26734 | +0.26734 | True | worse |
| 37 | pooled_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 37 | pooled_post_all_clean_rejection_rate | 0 | 0.2343 | +0.2343 | True | worse |
| 37 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | during_estimate_rms_m | 0.83623 | 0.52743 | -0.3088 | True | better |
| 47 | during_tracking_rms_m | 0.9687 | 0.67412 | -0.29458 | True | better |
| 47 | post_5s_estimate_rms_m | 1.3406 | 0.58519 | -0.7554 | True | better |
| 47 | post_5s_tracking_rms_m | 1.3526 | 0.56986 | -0.78273 | True | better |
| 47 | full_estimate_max_m | 1.486 | 0.72511 | -0.76091 | True | better |
| 47 | full_tracking_max_m | 1.6099 | 0.80616 | -0.80377 | True | better |
| 47 | recovery_estimate_s | NA | 0 | NA | False | better |
| 47 | recovery_tracking_s | NA | 0 | NA | False | better |
| 47 | recovery_joint_s | NA | 0 | NA | False | better |
| 47 | x_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 47 | x_clean_rejection_rate | 0 | 0.16234 | +0.16234 | True | worse |
| 47 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_rejection_rate | 0 | 0.06248 | +0.06248 | True | worse |
| 47 | x_all_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 47 | y_clean_rejection_rate | 0 | 0.17857 | +0.17857 | True | worse |
| 47 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 47 | y_post_all_clean_rejection_rate | 0 | 0.24992 | +0.24992 | True | worse |
| 47 | y_all_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_fault_active_rejection_rate | 0 | 100 | +100 | True | better |
| 47 | pooled_clean_rejection_rate | 0 | 0.17045 | +0.17045 | True | worse |
| 47 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 47 | pooled_post_all_clean_rejection_rate | 0 | 0.1562 | +0.1562 | True | worse |
| 47 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |

### Run 状态

| Seed | Mode | Duration s | Flight checks pass | Contacts | EKF resets | Joint recovery | Failed checks | Termination error |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 7 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 7 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 17 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 17 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 27 | gated | 62.0 | False | 0 | 0 | recovered | final_mean_target_error_below_50_cm | None |
| 27 | stock | 62.0 | True | 0 | 0 | recovered |  | None |
| 37 | gated | 62.0 | False | 0 | 0 | recovered | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 37 | stock | 62.0 | False | 0 | 0 | recovered | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 47 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 47 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |

## zero

[该 case 的 per-seed CSV](per_seed_zero.csv)

| Metric | Unit | Stock mean ± SD | Gated mean ± SD | Comparable paired Δ mean ± SD | Observed Δ mean ± SD (partial max) | Better / worse / tie / indeterminate (of 5) |
| --- | --- | --- | --- | --- | --- | --- |
| during_estimate_rms_m | m | 4.3867 ± 0.46755 (n=5) | 4.4664 ± 0.49878 (n=5) | 0.079718 ± 0.12667 (n=5) | same | 1 / 4 / 0 / 0 |
| during_tracking_rms_m | m | 3.6172 ± 0.44111 (n=5) | 3.69 ± 0.46755 (n=5) | 0.07281 ± 0.11669 (n=5) | same | 1 / 4 / 0 / 0 |
| post_5s_estimate_rms_m | m | 11.123 ± 0.63387 (n=3) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| post_5s_tracking_rms_m | m | 15.055 ± 0.79985 (n=3) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| full_estimate_max_m | m | 11.993 ± 1.1949 (n=5); complete=3/5 | 17.638 ± 0.52829 (n=5); complete=0/5 | NA ± NA (n=0) | 5.6456 ± 0.68981 (n=5) | 0 / 3 / 0 / 2 |
| full_tracking_max_m | m | 18.018 ± 1.0756 (n=5); complete=3/5 | 19.01 ± 0.028851 (n=5); complete=0/5 | NA ± NA (n=0) | 0.99174 ± 1.065 (n=5) | 0 / 3 / 0 / 2 |
| recovery_estimate_s | s | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| recovery_tracking_s | s | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| recovery_joint_s | s | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| x_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 4.4901 ± 0.89144 (n=5) | 4.4901 ± 0.89144 (n=5) | same | 0 / 5 / 0 / 0 |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 90.098 ± 0.069571 (n=5) | 90.098 ± 0.069571 (n=5) | same | 0 / 5 / 0 / 0 |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 90.098 ± 0.069571 (n=5) | 90.098 ± 0.069571 (n=5) | same | 0 / 5 / 0 / 0 |
| x_all_forced | scalar updates | 0 ± 0 (n=5) | 12.6 ± 2.7019 (n=5) | 12.6 ± 2.7019 (n=5) | same | 0 / 5 / 0 / 0 |
| x_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 12.6 ± 2.7019 (n=5) | 12.6 ± 2.7019 (n=5) | same | 0 / 5 / 0 / 0 |
| x_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 12.6 ± 2.7019 (n=5) | 12.6 ± 2.7019 (n=5) | same | 0 / 5 / 0 / 0 |
| y_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 2.0117 ± 1.8226 (n=5) | 2.0117 ± 1.8226 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 40.677 ± 39.419 (n=5) | 40.677 ± 39.419 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 40.677 ± 39.419 (n=5) | 40.677 ± 39.419 (n=5) | same | 0 / 5 / 0 / 0 |
| y_all_forced | scalar updates | 0 ± 0 (n=5) | 1.8 ± 2.1679 (n=5) | 1.8 ± 2.1679 (n=5) | same | 0 / 3 / 2 / 0 |
| y_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 1.8 ± 2.1679 (n=5) | 1.8 ± 2.1679 (n=5) | same | 0 / 3 / 2 / 0 |
| y_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 1.8 ± 2.1679 (n=5) | 1.8 ± 2.1679 (n=5) | same | 0 / 3 / 2 / 0 |
| pooled_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 3.2509 ± 0.9544 (n=5) | 3.2509 ± 0.9544 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 65.388 ± 19.71 (n=5) | 65.388 ± 19.71 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 65.388 ± 19.71 (n=5) | 65.388 ± 19.71 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_all_forced | scalar updates | 0 ± 0 (n=5) | 14.4 ± 3.2094 (n=5) | 14.4 ± 3.2094 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 14.4 ± 3.2094 (n=5) | 14.4 ± 3.2094 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 14.4 ± 3.2094 (n=5) | 14.4 ± 3.2094 (n=5) | same | 0 / 5 / 0 / 0 |

### 每个 seed

| Seed | Metric | Stock | Gated | Δ | Comparable Δ | Outcome |
| --- | --- | --- | --- | --- | --- | --- |
| 7 | during_estimate_rms_m | 4.4596 | 4.5051 | +0.045555 | True | worse |
| 7 | during_tracking_rms_m | 3.6899 | 3.7344 | +0.044503 | True | worse |
| 7 | post_5s_estimate_rms_m | 11.745 | NA | NA | False | indeterminate |
| 7 | post_5s_tracking_rms_m | 15.935 | NA | NA | False | indeterminate |
| 7 | full_estimate_max_m | 11.957 | 17.623 | +5.6663 | False | worse |
| 7 | full_tracking_max_m | 18.31 | 19.01 | +0.69981 | False | worse |
| 7 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 7 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 7 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 7 | x_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 7 | x_clean_rejection_rate | 0 | 4.6171 | +4.6171 | True | worse |
| 7 | x_post_5s_clean_rejection_rate | 0 | 90 | +90 | True | worse |
| 7 | x_post_all_clean_rejection_rate | 0 | 90 | +90 | True | worse |
| 7 | x_all_forced | 0 | 13 | +13 | True | worse |
| 7 | x_post_5s_clean_forced | 0 | 13 | +13 | True | worse |
| 7 | x_post_all_clean_forced | 0 | 13 | +13 | True | worse |
| 7 | y_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 7 | y_clean_rejection_rate | 0 | 2.1986 | +2.1986 | True | worse |
| 7 | y_post_5s_clean_rejection_rate | 0 | 41.538 | +41.538 | True | worse |
| 7 | y_post_all_clean_rejection_rate | 0 | 41.538 | +41.538 | True | worse |
| 7 | y_all_forced | 0 | 1 | +1 | True | worse |
| 7 | y_post_5s_clean_forced | 0 | 1 | +1 | True | worse |
| 7 | y_post_all_clean_forced | 0 | 1 | +1 | True | worse |
| 7 | pooled_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 7 | pooled_clean_rejection_rate | 0 | 3.4078 | +3.4078 | True | worse |
| 7 | pooled_post_5s_clean_rejection_rate | 0 | 65.769 | +65.769 | True | worse |
| 7 | pooled_post_all_clean_rejection_rate | 0 | 65.769 | +65.769 | True | worse |
| 7 | pooled_all_forced | 0 | 14 | +14 | True | worse |
| 7 | pooled_post_5s_clean_forced | 0 | 14 | +14 | True | worse |
| 7 | pooled_post_all_clean_forced | 0 | 14 | +14 | True | worse |
| 17 | during_estimate_rms_m | 4.1121 | 4.1728 | +0.060756 | True | worse |
| 17 | during_tracking_rms_m | 3.4132 | 3.4627 | +0.049515 | True | worse |
| 17 | post_5s_estimate_rms_m | 11.145 | NA | NA | False | indeterminate |
| 17 | post_5s_tracking_rms_m | 14.856 | NA | NA | False | indeterminate |
| 17 | full_estimate_max_m | 11.403 | 17.326 | +5.9228 | False | worse |
| 17 | full_tracking_max_m | 17.054 | 19.028 | +1.9741 | False | worse |
| 17 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 17 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 17 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 17 | x_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 17 | x_clean_rejection_rate | 0 | 4.9252 | +4.9252 | True | worse |
| 17 | x_post_5s_clean_rejection_rate | 0 | 90.141 | +90.141 | True | worse |
| 17 | x_post_all_clean_rejection_rate | 0 | 90.141 | +90.141 | True | worse |
| 17 | x_all_forced | 0 | 14 | +14 | True | worse |
| 17 | x_post_5s_clean_forced | 0 | 14 | +14 | True | worse |
| 17 | x_post_all_clean_forced | 0 | 14 | +14 | True | worse |
| 17 | y_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 17 | y_clean_rejection_rate | 0 | 4.4874 | +4.4874 | True | worse |
| 17 | y_post_5s_clean_rejection_rate | 0 | 81.69 | +81.69 | True | worse |
| 17 | y_post_all_clean_rejection_rate | 0 | 81.69 | +81.69 | True | worse |
| 17 | y_all_forced | 0 | 5 | +5 | True | worse |
| 17 | y_post_5s_clean_forced | 0 | 5 | +5 | True | worse |
| 17 | y_post_all_clean_forced | 0 | 5 | +5 | True | worse |
| 17 | pooled_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 17 | pooled_clean_rejection_rate | 0 | 4.7063 | +4.7063 | True | worse |
| 17 | pooled_post_5s_clean_rejection_rate | 0 | 85.915 | +85.915 | True | worse |
| 17 | pooled_post_all_clean_rejection_rate | 0 | 85.915 | +85.915 | True | worse |
| 17 | pooled_all_forced | 0 | 19 | +19 | True | worse |
| 17 | pooled_post_5s_clean_forced | 0 | 19 | +19 | True | worse |
| 17 | pooled_post_all_clean_forced | 0 | 19 | +19 | True | worse |
| 27 | during_estimate_rms_m | 3.9354 | 3.878 | -0.05747 | True | better |
| 27 | during_tracking_rms_m | 3.1685 | 3.1164 | -0.052068 | True | better |
| 27 | post_5s_estimate_rms_m | 10.478 | NA | NA | False | indeterminate |
| 27 | post_5s_tracking_rms_m | 14.373 | NA | NA | False | indeterminate |
| 27 | full_estimate_max_m | 10.695 | 17.003 | +6.3085 | False | worse |
| 27 | full_tracking_max_m | 16.723 | 18.975 | +2.2511 | False | worse |
| 27 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 27 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 27 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 27 | x_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 27 | x_clean_rejection_rate | 0 | 5.6159 | +5.6159 | True | worse |
| 27 | x_post_5s_clean_rejection_rate | 0 | 90.062 | +90.062 | True | worse |
| 27 | x_post_all_clean_rejection_rate | 0 | 90.062 | +90.062 | True | worse |
| 27 | x_all_forced | 0 | 16 | +16 | True | worse |
| 27 | x_post_5s_clean_forced | 0 | 16 | +16 | True | worse |
| 27 | x_post_all_clean_forced | 0 | 16 | +16 | True | worse |
| 27 | y_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 27 | y_clean_rejection_rate | 0 | 0.36232 | +0.36232 | True | worse |
| 27 | y_post_5s_clean_rejection_rate | 0 | 1.2422 | +1.2422 | True | worse |
| 27 | y_post_all_clean_rejection_rate | 0 | 1.2422 | +1.2422 | True | worse |
| 27 | y_all_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 27 | pooled_clean_rejection_rate | 0 | 2.9891 | +2.9891 | True | worse |
| 27 | pooled_post_5s_clean_rejection_rate | 0 | 45.652 | +45.652 | True | worse |
| 27 | pooled_post_all_clean_rejection_rate | 0 | 45.652 | +45.652 | True | worse |
| 27 | pooled_all_forced | 0 | 16 | +16 | True | worse |
| 27 | pooled_post_5s_clean_forced | 0 | 16 | +16 | True | worse |
| 27 | pooled_post_all_clean_forced | 0 | 16 | +16 | True | worse |
| 37 | during_estimate_rms_m | 5.1472 | 5.2087 | +0.061467 | True | worse |
| 37 | during_tracking_rms_m | 4.3326 | 4.3889 | +0.056267 | True | worse |
| 37 | post_5s_estimate_rms_m | NA | NA | NA | False | indeterminate |
| 37 | post_5s_tracking_rms_m | NA | NA | NA | False | indeterminate |
| 37 | full_estimate_max_m | 13.911 | 18.396 | +4.4851 | False | indeterminate |
| 37 | full_tracking_max_m | 19.024 | 19.047 | +0.022509 | False | indeterminate |
| 37 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 37 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 37 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 37 | x_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 37 | x_clean_rejection_rate | 0 | 3.2714 | +3.2714 | True | worse |
| 37 | x_post_5s_clean_rejection_rate | 0 | 90.11 | +90.11 | True | worse |
| 37 | x_post_all_clean_rejection_rate | 0 | 90.11 | +90.11 | True | worse |
| 37 | x_all_forced | 0 | 9 | +9 | True | worse |
| 37 | x_post_5s_clean_forced | 0 | 9 | +9 | True | worse |
| 37 | x_post_all_clean_forced | 0 | 9 | +9 | True | worse |
| 37 | y_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 37 | y_clean_rejection_rate | 0 | 2.8996 | +2.8996 | True | worse |
| 37 | y_post_5s_clean_rejection_rate | 0 | 78.022 | +78.022 | True | worse |
| 37 | y_post_all_clean_rejection_rate | 0 | 78.022 | +78.022 | True | worse |
| 37 | y_all_forced | 0 | 3 | +3 | True | worse |
| 37 | y_post_5s_clean_forced | 0 | 3 | +3 | True | worse |
| 37 | y_post_all_clean_forced | 0 | 3 | +3 | True | worse |
| 37 | pooled_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 37 | pooled_clean_rejection_rate | 0 | 3.0855 | +3.0855 | True | worse |
| 37 | pooled_post_5s_clean_rejection_rate | 0 | 84.066 | +84.066 | True | worse |
| 37 | pooled_post_all_clean_rejection_rate | 0 | 84.066 | +84.066 | True | worse |
| 37 | pooled_all_forced | 0 | 12 | +12 | True | worse |
| 37 | pooled_post_5s_clean_forced | 0 | 12 | +12 | True | worse |
| 37 | pooled_post_all_clean_forced | 0 | 12 | +12 | True | worse |
| 47 | during_estimate_rms_m | 4.279 | 4.5673 | +0.28828 | True | worse |
| 47 | during_tracking_rms_m | 3.482 | 3.7479 | +0.26584 | True | worse |
| 47 | post_5s_estimate_rms_m | NA | NA | NA | False | indeterminate |
| 47 | post_5s_tracking_rms_m | NA | NA | NA | False | indeterminate |
| 47 | full_estimate_max_m | 11.997 | 17.843 | +5.8453 | False | indeterminate |
| 47 | full_tracking_max_m | 18.979 | 18.99 | +0.011225 | False | indeterminate |
| 47 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 47 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 47 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 47 | x_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 47 | x_clean_rejection_rate | 0 | 4.0207 | +4.0207 | True | worse |
| 47 | x_post_5s_clean_rejection_rate | 0 | 90.179 | +90.179 | True | worse |
| 47 | x_post_all_clean_rejection_rate | 0 | 90.179 | +90.179 | True | worse |
| 47 | x_all_forced | 0 | 11 | +11 | True | worse |
| 47 | x_post_5s_clean_forced | 0 | 11 | +11 | True | worse |
| 47 | x_post_all_clean_forced | 0 | 11 | +11 | True | worse |
| 47 | y_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 47 | y_clean_rejection_rate | 0 | 0.11066 | +0.11066 | True | worse |
| 47 | y_post_5s_clean_rejection_rate | 0 | 0.89286 | +0.89286 | True | worse |
| 47 | y_post_all_clean_rejection_rate | 0 | 0.89286 | +0.89286 | True | worse |
| 47 | y_all_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 47 | pooled_clean_rejection_rate | 0 | 2.0657 | +2.0657 | True | worse |
| 47 | pooled_post_5s_clean_rejection_rate | 0 | 45.536 | +45.536 | True | worse |
| 47 | pooled_post_all_clean_rejection_rate | 0 | 45.536 | +45.536 | True | worse |
| 47 | pooled_all_forced | 0 | 11 | +11 | True | worse |
| 47 | pooled_post_5s_clean_forced | 0 | 11 | +11 | True | worse |
| 47 | pooled_post_all_clean_forced | 0 | 11 | +11 | True | worse |

### Run 状态

| Seed | Mode | Duration s | Flight checks pass | Contacts | EKF resets | Joint recovery | Failed checks | Termination error |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 7 | gated | 31.295 | False | 0 | 0 | censored | completed_demo, final_mean_target_error_below_50_cm, final_mean_speed_below_25_cm_s | AssertionError('Flight left the test arena') |
| 7 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 17 | gated | 31.418 | False | 0 | 0 | censored | completed_demo, final_mean_target_error_below_50_cm, final_mean_speed_below_25_cm_s | AssertionError('Flight left the test arena') |
| 17 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 27 | gated | 31.609 | False | 0 | 0 | censored | completed_demo, final_mean_target_error_below_50_cm, final_mean_speed_below_25_cm_s | AssertionError('Flight left the test arena') |
| 27 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 37 | gated | 30.903000000000002 | False | 0 | 0 | censored | completed_demo, hover_xy_within_50_cm, final_mean_target_error_below_50_cm, final_mean_speed_below_25_cm_s | AssertionError('Flight left the test arena') |
| 37 | stock | 31.101 | False | 0 | 0 | censored | completed_demo, hover_xy_within_50_cm, final_mean_target_error_below_50_cm, final_mean_speed_below_25_cm_s | AssertionError('Flight left the test arena') |
| 47 | gated | 31.119 | False | 0 | 0 | censored | completed_demo, final_mean_target_error_below_50_cm, final_mean_speed_below_25_cm_s | AssertionError('Flight left the test arena') |
| 47 | stock | 31.753 | False | 0 | 0 | censored | completed_demo, final_mean_target_error_below_50_cm, final_mean_speed_below_25_cm_s | AssertionError('Flight left the test arena') |

## dropout

[该 case 的 per-seed CSV](per_seed_dropout.csv)

| Metric | Unit | Stock mean ± SD | Gated mean ± SD | Comparable paired Δ mean ± SD | Observed Δ mean ± SD (partial max) | Better / worse / tie / indeterminate (of 5) |
| --- | --- | --- | --- | --- | --- | --- |
| during_estimate_rms_m | m | 0.50606 ± 0.20826 (n=5) | 0.48077 ± 0.18585 (n=5) | -0.025292 ± 0.057656 (n=5) | same | 4 / 1 / 0 / 0 |
| during_tracking_rms_m | m | 0.59793 ± 0.25506 (n=5) | 0.56913 ± 0.23471 (n=5) | -0.028801 ± 0.054917 (n=5) | same | 3 / 2 / 0 / 0 |
| post_5s_estimate_rms_m | m | 0.50776 ± 0.15477 (n=5) | 0.49554 ± 0.12443 (n=5) | -0.012228 ± 0.056551 (n=5) | same | 3 / 2 / 0 / 0 |
| post_5s_tracking_rms_m | m | 0.54611 ± 0.14046 (n=5) | 0.53112 ± 0.10727 (n=5) | -0.014993 ± 0.056263 (n=5) | same | 3 / 2 / 0 / 0 |
| full_estimate_max_m | m | 0.98864 ± 0.2637 (n=5); complete=5/5 | 0.95572 ± 0.23356 (n=5); complete=5/5 | -0.032918 ± 0.0759 (n=5) | same | 4 / 1 / 0 / 0 |
| full_tracking_max_m | m | 0.88382 ± 0.19586 (n=5); complete=5/5 | 0.87914 ± 0.18328 (n=5); complete=5/5 | -0.0046761 ± 0.10601 (n=5) | same | 2 / 3 / 0 / 0 |
| recovery_estimate_s | s | 1.2 ± NA (n=1) | 9.17 ± 11.218 (n=3) | 0.99 ± NA (n=1) | same | 2 / 1 / 0 / 2 |
| recovery_tracking_s | s | 3.7278 ± 3.6195 (n=4) | 2.196 ± 1.3218 (n=3) | 0.134 ± 0.74573 (n=3) | same | 2 / 2 / 0 / 1 |
| recovery_joint_s | s | 1.2 ± NA (n=1) | 9.17 ± 11.218 (n=3) | 0.99 ± NA (n=1) | same | 2 / 1 / 0 / 2 |
| x_fault_active_rejection_rate | % / Δ percentage points | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| x_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.25517 ± 0.049372 (n=5) | 0.25517 ± 0.049372 (n=5) | same | 0 / 5 / 0 / 0 |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.32 ± 0.43818 (n=5) | 0.32 ± 0.43818 (n=5) | same | 0 / 2 / 3 / 0 |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.21243 ± 0.097297 (n=5) | 0.21243 ± 0.097297 (n=5) | same | 0 / 5 / 0 / 0 |
| x_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_fault_active_rejection_rate | % / Δ percentage points | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| y_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.23103 ± 0.069609 (n=5) | 0.23103 ± 0.069609 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.44 ± 0.16733 (n=5) | 0.44 ± 0.16733 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.23118 ± 0.092674 (n=5) | 0.23118 ± 0.092674 (n=5) | same | 0 / 5 / 0 / 0 |
| y_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_fault_active_rejection_rate | % / Δ percentage points | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| pooled_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.2431 ± 0.055334 (n=5) | 0.2431 ± 0.055334 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.38 ± 0.21679 (n=5) | 0.38 ± 0.21679 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.22181 ± 0.072764 (n=5) | 0.22181 ± 0.072764 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |

### 每个 seed

| Seed | Metric | Stock | Gated | Δ | Comparable Δ | Outcome |
| --- | --- | --- | --- | --- | --- | --- |
| 7 | during_estimate_rms_m | 0.19415 | 0.24889 | +0.05474 | True | worse |
| 7 | during_tracking_rms_m | 0.2301 | 0.25183 | +0.021733 | True | worse |
| 7 | post_5s_estimate_rms_m | 0.29009 | 0.36092 | +0.070827 | True | worse |
| 7 | post_5s_tracking_rms_m | 0.31888 | 0.38574 | +0.066857 | True | worse |
| 7 | full_estimate_max_m | 0.59928 | 0.68436 | +0.085073 | True | worse |
| 7 | full_tracking_max_m | 0.60956 | 0.66865 | +0.059097 | True | worse |
| 7 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 7 | recovery_tracking_s | 8.725 | NA | NA | False | worse |
| 7 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 7 | x_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 7 | x_clean_rejection_rate | 0 | 0.25862 | +0.25862 | True | worse |
| 7 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 7 | x_all_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 7 | y_clean_rejection_rate | 0 | 0.15517 | +0.15517 | True | worse |
| 7 | y_post_5s_clean_rejection_rate | 0 | 0.2 | +0.2 | True | worse |
| 7 | y_post_all_clean_rejection_rate | 0 | 0.093721 | +0.093721 | True | worse |
| 7 | y_all_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 7 | pooled_clean_rejection_rate | 0 | 0.2069 | +0.2069 | True | worse |
| 7 | pooled_post_5s_clean_rejection_rate | 0 | 0.1 | +0.1 | True | worse |
| 7 | pooled_post_all_clean_rejection_rate | 0 | 0.14058 | +0.14058 | True | worse |
| 7 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | during_estimate_rms_m | 0.49395 | 0.38573 | -0.10822 | True | better |
| 17 | during_tracking_rms_m | 0.62487 | 0.5089 | -0.11597 | True | better |
| 17 | post_5s_estimate_rms_m | 0.42902 | 0.37151 | -0.057515 | True | better |
| 17 | post_5s_tracking_rms_m | 0.51735 | 0.44744 | -0.069913 | True | better |
| 17 | full_estimate_max_m | 0.9534 | 0.83406 | -0.11934 | True | better |
| 17 | full_tracking_max_m | 0.95686 | 0.78849 | -0.16837 | True | better |
| 17 | recovery_estimate_s | NA | 22.11 | NA | False | better |
| 17 | recovery_tracking_s | 4.042 | 3.667 | -0.375 | True | better |
| 17 | recovery_joint_s | NA | 22.11 | NA | False | better |
| 17 | x_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 17 | x_clean_rejection_rate | 0 | 0.2931 | +0.2931 | True | worse |
| 17 | x_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 17 | x_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | x_all_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 17 | y_clean_rejection_rate | 0 | 0.2931 | +0.2931 | True | worse |
| 17 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 17 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | y_all_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 17 | pooled_clean_rejection_rate | 0 | 0.2931 | +0.2931 | True | worse |
| 17 | pooled_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 17 | pooled_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | during_estimate_rms_m | 0.56294 | 0.54025 | -0.022692 | True | better |
| 27 | during_tracking_rms_m | 0.53048 | 0.54497 | +0.014493 | True | worse |
| 27 | post_5s_estimate_rms_m | 0.52271 | 0.54387 | +0.021163 | True | worse |
| 27 | post_5s_tracking_rms_m | 0.57638 | 0.59472 | +0.018345 | True | worse |
| 27 | full_estimate_max_m | 0.96947 | 0.92626 | -0.043209 | True | better |
| 27 | full_tracking_max_m | 0.82997 | 0.94005 | +0.11009 | True | worse |
| 27 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 27 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 27 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 27 | x_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 27 | x_clean_rejection_rate | 0 | 0.2931 | +0.2931 | True | worse |
| 27 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_rejection_rate | 0 | 0.21868 | +0.21868 | True | worse |
| 27 | x_all_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 27 | y_clean_rejection_rate | 0 | 0.31034 | +0.31034 | True | worse |
| 27 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 27 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 27 | y_all_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 27 | pooled_clean_rejection_rate | 0 | 0.30172 | +0.30172 | True | worse |
| 27 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 27 | pooled_post_all_clean_rejection_rate | 0 | 0.26554 | +0.26554 | True | worse |
| 27 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | during_estimate_rms_m | 0.7759 | 0.74877 | -0.027134 | True | better |
| 37 | during_tracking_rms_m | 0.93677 | 0.90185 | -0.034923 | True | better |
| 37 | post_5s_estimate_rms_m | 0.61386 | 0.55614 | -0.057716 | True | better |
| 37 | post_5s_tracking_rms_m | 0.66468 | 0.62263 | -0.04205 | True | better |
| 37 | full_estimate_max_m | 1.3256 | 1.3077 | -0.01786 | True | better |
| 37 | full_tracking_max_m | 1.1482 | 1.1562 | +0.0080162 | True | worse |
| 37 | recovery_estimate_s | 1.2 | 2.19 | +0.99 | True | worse |
| 37 | recovery_tracking_s | 0.823 | 1.813 | +0.99 | True | worse |
| 37 | recovery_joint_s | 1.2 | 2.19 | +0.99 | True | worse |
| 37 | x_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 37 | x_clean_rejection_rate | 0 | 0.25862 | +0.25862 | True | worse |
| 37 | x_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 37 | x_post_all_clean_rejection_rate | 0 | 0.28116 | +0.28116 | True | worse |
| 37 | x_all_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 37 | y_clean_rejection_rate | 0 | 0.22414 | +0.22414 | True | worse |
| 37 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 37 | y_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 37 | y_all_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 37 | pooled_clean_rejection_rate | 0 | 0.24138 | +0.24138 | True | worse |
| 37 | pooled_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 37 | pooled_post_all_clean_rejection_rate | 0 | 0.2343 | +0.2343 | True | worse |
| 37 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | during_estimate_rms_m | 0.50338 | 0.48022 | -0.023154 | True | better |
| 47 | during_tracking_rms_m | 0.66745 | 0.63811 | -0.029341 | True | better |
| 47 | post_5s_estimate_rms_m | 0.68315 | 0.64525 | -0.037899 | True | better |
| 47 | post_5s_tracking_rms_m | 0.65325 | 0.60505 | -0.048202 | True | better |
| 47 | full_estimate_max_m | 1.0954 | 1.0262 | -0.069257 | True | better |
| 47 | full_tracking_max_m | 0.87452 | 0.8423 | -0.032213 | True | better |
| 47 | recovery_estimate_s | NA | 3.21 | NA | False | better |
| 47 | recovery_tracking_s | 1.321 | 1.108 | -0.213 | True | better |
| 47 | recovery_joint_s | NA | 3.21 | NA | False | better |
| 47 | x_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 47 | x_clean_rejection_rate | 0 | 0.17241 | +0.17241 | True | worse |
| 47 | x_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_rejection_rate | 0 | 0.06248 | +0.06248 | True | worse |
| 47 | x_all_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 47 | y_clean_rejection_rate | 0 | 0.17241 | +0.17241 | True | worse |
| 47 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 47 | y_post_all_clean_rejection_rate | 0 | 0.24992 | +0.24992 | True | worse |
| 47 | y_all_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_fault_active_rejection_rate | NA | NA | NA | False | indeterminate |
| 47 | pooled_clean_rejection_rate | 0 | 0.17241 | +0.17241 | True | worse |
| 47 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 47 | pooled_post_all_clean_rejection_rate | 0 | 0.1562 | +0.1562 | True | worse |
| 47 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |

### Run 状态

| Seed | Mode | Duration s | Flight checks pass | Contacts | EKF resets | Joint recovery | Failed checks | Termination error |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 7 | gated | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 7 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 17 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 17 | stock | 62.0 | True | 0 | 0 | censored |  | None |
| 27 | gated | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 27 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 37 | gated | 62.0 | False | 0 | 0 | recovered | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 37 | stock | 62.0 | False | 0 | 0 | recovered | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 47 | gated | 62.0 | True | 0 | 0 | recovered |  | None |
| 47 | stock | 62.0 | True | 0 | 0 | censored |  | None |

## scale 0.5

[该 case 的 per-seed CSV](per_seed_scale05.csv)

| Metric | Unit | Stock mean ± SD | Gated mean ± SD | Comparable paired Δ mean ± SD | Observed Δ mean ± SD (partial max) | Better / worse / tie / indeterminate (of 5) |
| --- | --- | --- | --- | --- | --- | --- |
| during_estimate_rms_m | m | 0.79629 ± 0.10704 (n=5) | 0.795 ± 0.081125 (n=5) | -0.0012838 ± 0.060612 (n=5) | same | 1 / 4 / 0 / 0 |
| during_tracking_rms_m | m | 0.71284 ± 0.032988 (n=5) | 0.70681 ± 0.03508 (n=5) | -0.006022 ± 0.050612 (n=5) | same | 3 / 2 / 0 / 0 |
| post_5s_estimate_rms_m | m | 1.5009 ± 0.30203 (n=5) | 1.5436 ± 0.28973 (n=5) | 0.042772 ± 0.072464 (n=5) | same | 1 / 4 / 0 / 0 |
| post_5s_tracking_rms_m | m | 1.5451 ± 0.31343 (n=5) | 1.5894 ± 0.30367 (n=5) | 0.044284 ± 0.071893 (n=5) | same | 1 / 4 / 0 / 0 |
| full_estimate_max_m | m | 1.5754 ± 0.29452 (n=5); complete=5/5 | 1.6227 ± 0.27715 (n=5); complete=5/5 | 0.047374 ± 0.073391 (n=5) | same | 1 / 4 / 0 / 0 |
| full_tracking_max_m | m | 1.7187 ± 0.32259 (n=5); complete=5/5 | 1.7645 ± 0.29202 (n=5); complete=5/5 | 0.045787 ± 0.064238 (n=5) | same | 1 / 4 / 0 / 0 |
| recovery_estimate_s | s | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| recovery_tracking_s | s | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| recovery_joint_s | s | NA ± NA (n=0) | NA ± NA (n=0) | NA ± NA (n=0) | same | 0 / 0 / 0 / 5 |
| x_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.5 ± 0.17678 (n=5) | 0.5 ± 0.17678 (n=5) | same | 5 / 0 / 0 / 0 |
| x_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.3069 ± 0.068316 (n=5) | 0.3069 ± 0.068316 (n=5) | same | 0 / 5 / 0 / 0 |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.92 ± 0.38987 (n=5) | 0.92 ± 0.38987 (n=5) | same | 0 / 5 / 0 / 0 |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.30615 ± 0.11133 (n=5) | 0.30615 ± 0.11133 (n=5) | same | 0 / 5 / 0 / 0 |
| x_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| x_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.85 ± 0.60208 (n=5) | 0.85 ± 0.60208 (n=5) | same | 4 / 0 / 1 / 0 |
| y_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.22759 ± 0.074558 (n=5) | 0.22759 ± 0.074558 (n=5) | same | 0 / 5 / 0 / 0 |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.4 ± 0.24495 (n=5) | 0.4 ± 0.24495 (n=5) | same | 0 / 4 / 1 / 0 |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.22493 ± 0.10455 (n=5) | 0.22493 ± 0.10455 (n=5) | same | 0 / 5 / 0 / 0 |
| y_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| y_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_fault_active_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.675 ± 0.36012 (n=5) | 0.675 ± 0.36012 (n=5) | same | 5 / 0 / 0 / 0 |
| pooled_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.26724 ± 0.065653 (n=5) | 0.26724 ± 0.065653 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.66 ± 0.251 (n=5) | 0.66 ± 0.251 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 0 ± 0 (n=5) | 0.26554 ± 0.086265 (n=5) | 0.26554 ± 0.086265 (n=5) | same | 0 / 5 / 0 / 0 |
| pooled_all_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_5s_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |
| pooled_post_all_clean_forced | scalar updates | 0 ± 0 (n=5) | 0 ± 0 (n=5) | 0 ± 0 (n=5) | same | 0 / 0 / 5 / 0 |

### 每个 seed

| Seed | Metric | Stock | Gated | Δ | Comparable Δ | Outcome |
| --- | --- | --- | --- | --- | --- | --- |
| 7 | during_estimate_rms_m | 0.85702 | 0.89556 | +0.038539 | True | worse |
| 7 | during_tracking_rms_m | 0.69831 | 0.73623 | +0.037916 | True | worse |
| 7 | post_5s_estimate_rms_m | 1.5954 | 1.6683 | +0.072947 | True | worse |
| 7 | post_5s_tracking_rms_m | 1.6416 | 1.7152 | +0.073623 | True | worse |
| 7 | full_estimate_max_m | 1.6674 | 1.744 | +0.076542 | True | worse |
| 7 | full_tracking_max_m | 1.8153 | 1.886 | +0.070657 | True | worse |
| 7 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 7 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 7 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 7 | x_fault_active_rejection_rate | 0 | 0.75 | +0.75 | True | better |
| 7 | x_clean_rejection_rate | 0 | 0.31034 | +0.31034 | True | worse |
| 7 | x_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 7 | x_post_all_clean_rejection_rate | 0 | 0.28116 | +0.28116 | True | worse |
| 7 | x_all_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_fault_active_rejection_rate | 0 | 1.5 | +1.5 | True | better |
| 7 | y_clean_rejection_rate | 0 | 0.13793 | +0.13793 | True | worse |
| 7 | y_post_5s_clean_rejection_rate | 0 | 0 | +0 | True | tie |
| 7 | y_post_all_clean_rejection_rate | 0 | 0.06248 | +0.06248 | True | worse |
| 7 | y_all_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_fault_active_rejection_rate | 0 | 1.125 | +1.125 | True | better |
| 7 | pooled_clean_rejection_rate | 0 | 0.22414 | +0.22414 | True | worse |
| 7 | pooled_post_5s_clean_rejection_rate | 0 | 0.3 | +0.3 | True | worse |
| 7 | pooled_post_all_clean_rejection_rate | 0 | 0.17182 | +0.17182 | True | worse |
| 7 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 7 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | during_estimate_rms_m | 0.83325 | 0.84077 | +0.0075237 | True | worse |
| 17 | during_tracking_rms_m | 0.74766 | 0.73088 | -0.016779 | True | better |
| 17 | post_5s_estimate_rms_m | 1.8249 | 1.8953 | +0.070305 | True | worse |
| 17 | post_5s_tracking_rms_m | 1.8854 | 1.9588 | +0.073452 | True | worse |
| 17 | full_estimate_max_m | 1.8959 | 1.9673 | +0.071354 | True | worse |
| 17 | full_tracking_max_m | 2.0697 | 2.1158 | +0.046097 | True | worse |
| 17 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 17 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 17 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 17 | x_fault_active_rejection_rate | 0 | 0.5 | +0.5 | True | better |
| 17 | x_clean_rejection_rate | 0 | 0.32759 | +0.32759 | True | worse |
| 17 | x_post_5s_clean_rejection_rate | 0 | 1.2 | +1.2 | True | worse |
| 17 | x_post_all_clean_rejection_rate | 0 | 0.37488 | +0.37488 | True | worse |
| 17 | x_all_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_fault_active_rejection_rate | 0 | 1.25 | +1.25 | True | better |
| 17 | y_clean_rejection_rate | 0 | 0.2931 | +0.2931 | True | worse |
| 17 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 17 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 17 | y_all_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_fault_active_rejection_rate | 0 | 0.875 | +0.875 | True | better |
| 17 | pooled_clean_rejection_rate | 0 | 0.31034 | +0.31034 | True | worse |
| 17 | pooled_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 17 | pooled_post_all_clean_rejection_rate | 0 | 0.34364 | +0.34364 | True | worse |
| 17 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 17 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | during_estimate_rms_m | 0.9114 | 0.8081 | -0.1033 | True | better |
| 27 | during_tracking_rms_m | 0.7356 | 0.64934 | -0.086255 | True | better |
| 27 | post_5s_estimate_rms_m | 1.7194 | 1.6572 | -0.062183 | True | better |
| 27 | post_5s_tracking_rms_m | 1.7679 | 1.7095 | -0.058382 | True | better |
| 27 | full_estimate_max_m | 1.7811 | 1.7253 | -0.055807 | True | better |
| 27 | full_tracking_max_m | 1.9461 | 1.8995 | -0.046556 | True | better |
| 27 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 27 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 27 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 27 | x_fault_active_rejection_rate | 0 | 0.5 | +0.5 | True | better |
| 27 | x_clean_rejection_rate | 0 | 0.39655 | +0.39655 | True | worse |
| 27 | x_post_5s_clean_rejection_rate | 0 | 1.2 | +1.2 | True | worse |
| 27 | x_post_all_clean_rejection_rate | 0 | 0.40612 | +0.40612 | True | worse |
| 27 | x_all_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_fault_active_rejection_rate | 0 | 0 | +0 | True | tie |
| 27 | y_clean_rejection_rate | 0 | 0.31034 | +0.31034 | True | worse |
| 27 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 27 | y_post_all_clean_rejection_rate | 0 | 0.3124 | +0.3124 | True | worse |
| 27 | y_all_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_fault_active_rejection_rate | 0 | 0.25 | +0.25 | True | better |
| 27 | pooled_clean_rejection_rate | 0 | 0.35345 | +0.35345 | True | worse |
| 27 | pooled_post_5s_clean_rejection_rate | 0 | 0.9 | +0.9 | True | worse |
| 27 | pooled_post_all_clean_rejection_rate | 0 | 0.35926 | +0.35926 | True | worse |
| 27 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 27 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | during_estimate_rms_m | 0.6405 | 0.69039 | +0.049895 | True | worse |
| 37 | during_tracking_rms_m | 0.66407 | 0.69968 | +0.03561 | True | worse |
| 37 | post_5s_estimate_rms_m | 1.1919 | 1.3183 | +0.12641 | True | worse |
| 37 | post_5s_tracking_rms_m | 1.2341 | 1.3615 | +0.12739 | True | worse |
| 37 | full_estimate_max_m | 1.2326 | 1.3691 | +0.1365 | True | worse |
| 37 | full_tracking_max_m | 1.335 | 1.4649 | +0.12989 | True | worse |
| 37 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 37 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 37 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 37 | x_fault_active_rejection_rate | 0 | 0.5 | +0.5 | True | better |
| 37 | x_clean_rejection_rate | 0 | 0.2931 | +0.2931 | True | worse |
| 37 | x_post_5s_clean_rejection_rate | 0 | 1.2 | +1.2 | True | worse |
| 37 | x_post_all_clean_rejection_rate | 0 | 0.34364 | +0.34364 | True | worse |
| 37 | x_all_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_fault_active_rejection_rate | 0 | 1 | +1 | True | better |
| 37 | y_clean_rejection_rate | 0 | 0.22414 | +0.22414 | True | worse |
| 37 | y_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 37 | y_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 37 | y_all_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_fault_active_rejection_rate | 0 | 0.75 | +0.75 | True | better |
| 37 | pooled_clean_rejection_rate | 0 | 0.25862 | +0.25862 | True | worse |
| 37 | pooled_post_5s_clean_rejection_rate | 0 | 0.8 | +0.8 | True | worse |
| 37 | pooled_post_all_clean_rejection_rate | 0 | 0.26554 | +0.26554 | True | worse |
| 37 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 37 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | during_estimate_rms_m | 0.73927 | 0.7402 | +0.0009267 | True | worse |
| 47 | during_tracking_rms_m | 0.71854 | 0.71794 | -0.00060154 | True | better |
| 47 | post_5s_estimate_rms_m | 1.1727 | 1.179 | +0.0063779 | True | worse |
| 47 | post_5s_tracking_rms_m | 1.1965 | 1.2019 | +0.0053404 | True | worse |
| 47 | full_estimate_max_m | 1.2998 | 1.3081 | +0.0082784 | True | worse |
| 47 | full_tracking_max_m | 1.4275 | 1.4564 | +0.028846 | True | worse |
| 47 | recovery_estimate_s | NA | NA | NA | False | indeterminate |
| 47 | recovery_tracking_s | NA | NA | NA | False | indeterminate |
| 47 | recovery_joint_s | NA | NA | NA | False | indeterminate |
| 47 | x_fault_active_rejection_rate | 0 | 0.25 | +0.25 | True | better |
| 47 | x_clean_rejection_rate | 0 | 0.2069 | +0.2069 | True | worse |
| 47 | x_post_5s_clean_rejection_rate | 0 | 0.4 | +0.4 | True | worse |
| 47 | x_post_all_clean_rejection_rate | 0 | 0.12496 | +0.12496 | True | worse |
| 47 | x_all_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | x_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_fault_active_rejection_rate | 0 | 0.5 | +0.5 | True | better |
| 47 | y_clean_rejection_rate | 0 | 0.17241 | +0.17241 | True | worse |
| 47 | y_post_5s_clean_rejection_rate | 0 | 0.6 | +0.6 | True | worse |
| 47 | y_post_all_clean_rejection_rate | 0 | 0.24992 | +0.24992 | True | worse |
| 47 | y_all_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | y_post_all_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_fault_active_rejection_rate | 0 | 0.375 | +0.375 | True | better |
| 47 | pooled_clean_rejection_rate | 0 | 0.18966 | +0.18966 | True | worse |
| 47 | pooled_post_5s_clean_rejection_rate | 0 | 0.5 | +0.5 | True | worse |
| 47 | pooled_post_all_clean_rejection_rate | 0 | 0.18744 | +0.18744 | True | worse |
| 47 | pooled_all_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_5s_clean_forced | 0 | 0 | +0 | True | tie |
| 47 | pooled_post_all_clean_forced | 0 | 0 | +0 | True | tie |

### Run 状态

| Seed | Mode | Duration s | Flight checks pass | Contacts | EKF resets | Joint recovery | Failed checks | Termination error |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 7 | gated | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 7 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 17 | gated | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 17 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 27 | gated | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 27 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 37 | gated | 62.0 | False | 0 | 0 | censored | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 37 | stock | 62.0 | False | 0 | 0 | censored | hover_xy_within_50_cm, final_mean_target_error_below_50_cm | None |
| 47 | gated | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |
| 47 | stock | 62.0 | False | 0 | 0 | censored | final_mean_target_error_below_50_cm | None |

