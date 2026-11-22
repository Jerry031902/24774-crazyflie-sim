# Gated 变差的完整清单

以下逐项列出所有能确定变差的 seed/metric。Δ=gated−stock；error 单位 m，recovery 单位 s，rate 的 Δ 单位 percentage points。Forced count 增加只表示发生次数增加。
mean/SD 和完整对照见 [TABLES.md](TABLES.md)；机器可读数据见 [worse_per_seed.csv](worse_per_seed.csv)。

## off

| Metric | Unit | 变差的 seeds (Δ) |
| --- | --- | --- |
| during_estimate_rms_m | m | 7 (+0.033541), 47 (+0.011858) |
| during_tracking_rms_m | m | 27 (+0.030035), 47 (+0.011353) |
| post_5s_estimate_rms_m | m | 7 (+0.051354), 17 (+0.01155), 27 (+0.0057027) |
| post_5s_tracking_rms_m | m | 7 (+0.046214), 17 (+0.015699), 27 (+0.001004) |
| full_estimate_max_m | m | 7 (+0.062534), 27 (+0.10857) |
| full_tracking_max_m | m | 27 (+0.11526), 47 (+0.00024334) |
| x_clean_rejection_rate | % / Δ percentage points | 7 (+0.27419), 17 (+0.27419), 27 (+0.30645), 37 (+0.27419), 47 (+0.16129) |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 17 (+0.8), 37 (+0.8) |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.18744), 17 (+0.3124), 27 (+0.21868), 37 (+0.28116), 47 (+0.06248) |
| y_clean_rejection_rate | % / Δ percentage points | 7 (+0.19355), 17 (+0.32258), 27 (+0.29032), 37 (+0.25806), 47 (+0.17742) |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.2), 17 (+0.4), 27 (+0.6), 37 (+0.4), 47 (+0.6) |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.093721), 17 (+0.3124), 27 (+0.3124), 37 (+0.18744), 47 (+0.24992) |
| pooled_clean_rejection_rate | % / Δ percentage points | 7 (+0.23387), 17 (+0.29839), 27 (+0.29839), 37 (+0.26613), 47 (+0.16935) |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.1), 17 (+0.6), 27 (+0.3), 37 (+0.6), 47 (+0.3) |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.14058), 17 (+0.3124), 27 (+0.26554), 37 (+0.2343), 47 (+0.1562) |

## spike 5

| Metric | Unit | 变差的 seeds (Δ) |
| --- | --- | --- |
| during_estimate_rms_m | m | 7 (+0.076926) |
| during_tracking_rms_m | m | 7 (+0.0027049), 27 (+0.0095695) |
| post_5s_estimate_rms_m | m | 7 (+0.07245), 17 (+0.059991) |
| post_5s_tracking_rms_m | m | 7 (+0.068047), 17 (+0.061767) |
| full_estimate_max_m | m | 7 (+0.050197), 17 (+0.039259), 27 (+0.086826) |
| full_tracking_max_m | m | 27 (+0.092862) |
| recovery_estimate_s | s | 7 (+1.97), 17 (+2.556) |
| recovery_tracking_s | s | 17 (+8.537) |
| recovery_joint_s | s | 7 (+1.97), 17 (+2.556) |
| x_clean_rejection_rate | % / Δ percentage points | 7 (+0.27611), 17 (+0.29283), 27 (+0.30819), 37 (+0.25924), 47 (+0.16234) |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 17 (+0.8), 37 (+0.8) |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.18744), 17 (+0.3124), 27 (+0.21868), 37 (+0.28116), 47 (+0.06248) |
| y_clean_rejection_rate | % / Δ percentage points | 7 (+0.22738), 17 (+0.34163), 27 (+0.29197), 37 (+0.25924), 47 (+0.17857) |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.2), 17 (+0.4), 27 (+0.6), 37 (+0.4), 47 (+0.6) |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.093721), 17 (+0.3124), 27 (+0.3124), 37 (+0.18744), 47 (+0.24992) |
| pooled_clean_rejection_rate | % / Δ percentage points | 7 (+0.25175), 17 (+0.31723), 27 (+0.30008), 37 (+0.25924), 47 (+0.17045) |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.1), 17 (+0.6), 27 (+0.3), 37 (+0.6), 47 (+0.3) |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.14058), 17 (+0.3124), 27 (+0.26554), 37 (+0.2343), 47 (+0.1562) |

## spike 10

| Metric | Unit | 变差的 seeds (Δ) |
| --- | --- | --- |
| during_estimate_rms_m | m | 7 (+0.020315) |
| during_tracking_rms_m | m | 27 (+0.09296) |
| full_estimate_max_m | m | 7 (+0.011471), 27 (+0.21202) |
| full_tracking_max_m | m | 27 (+0.21736) |
| x_clean_rejection_rate | % / Δ percentage points | 7 (+0.25987), 17 (+0.27656), 27 (+0.30819), 37 (+0.27544), 47 (+0.16234) |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 17 (+0.8), 37 (+0.8) |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.18744), 17 (+0.3124), 27 (+0.21868), 37 (+0.28116), 47 (+0.06248) |
| y_clean_rejection_rate | % / Δ percentage points | 7 (+0.1949), 17 (+0.34163), 27 (+0.29197), 37 (+0.25924), 47 (+0.17857) |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.2), 17 (+0.4), 27 (+0.6), 37 (+0.4), 47 (+0.6) |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.093721), 17 (+0.3124), 27 (+0.3124), 37 (+0.18744), 47 (+0.24992) |
| pooled_clean_rejection_rate | % / Δ percentage points | 7 (+0.22738), 17 (+0.30909), 27 (+0.30008), 37 (+0.26734), 47 (+0.17045) |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.1), 17 (+0.6), 27 (+0.3), 37 (+0.6), 47 (+0.3) |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.14058), 17 (+0.3124), 27 (+0.26554), 37 (+0.2343), 47 (+0.1562) |

## spike 20

| Metric | Unit | 变差的 seeds (Δ) |
| --- | --- | --- |
| during_tracking_rms_m | m | 27 (+0.083465) |
| full_estimate_max_m | m | 27 (+0.28605) |
| full_tracking_max_m | m | 27 (+0.18916) |
| x_clean_rejection_rate | % / Δ percentage points | 7 (+0.25987), 17 (+0.27656), 27 (+0.30819), 37 (+0.27544), 47 (+0.16234) |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 17 (+0.8), 37 (+0.8) |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.18744), 17 (+0.3124), 27 (+0.21868), 37 (+0.28116), 47 (+0.06248) |
| y_clean_rejection_rate | % / Δ percentage points | 7 (+0.1949), 17 (+0.34163), 27 (+0.29197), 37 (+0.25924), 47 (+0.17857) |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.2), 17 (+0.4), 27 (+0.6), 37 (+0.4), 47 (+0.6) |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.093721), 17 (+0.3124), 27 (+0.3124), 37 (+0.18744), 47 (+0.24992) |
| pooled_clean_rejection_rate | % / Δ percentage points | 7 (+0.22738), 17 (+0.30909), 27 (+0.30008), 37 (+0.26734), 47 (+0.17045) |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.1), 17 (+0.6), 27 (+0.3), 37 (+0.6), 47 (+0.3) |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.14058), 17 (+0.3124), 27 (+0.26554), 37 (+0.2343), 47 (+0.1562) |

## spike 50

| Metric | Unit | 变差的 seeds (Δ) |
| --- | --- | --- |
| x_clean_rejection_rate | % / Δ percentage points | 7 (+0.25987), 17 (+0.27656), 27 (+0.30819), 37 (+0.27544), 47 (+0.16234) |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 17 (+0.8), 37 (+0.8) |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.18744), 17 (+0.3124), 27 (+0.21868), 37 (+0.28116), 47 (+0.06248) |
| y_clean_rejection_rate | % / Δ percentage points | 7 (+0.1949), 17 (+0.34163), 27 (+0.29197), 37 (+0.25924), 47 (+0.17857) |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.2), 17 (+0.4), 27 (+0.6), 37 (+0.4), 47 (+0.6) |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.093721), 17 (+0.3124), 27 (+0.3124), 37 (+0.18744), 47 (+0.24992) |
| pooled_clean_rejection_rate | % / Δ percentage points | 7 (+0.22738), 17 (+0.30909), 27 (+0.30008), 37 (+0.26734), 47 (+0.17045) |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.1), 17 (+0.6), 27 (+0.3), 37 (+0.6), 47 (+0.3) |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.14058), 17 (+0.3124), 27 (+0.26554), 37 (+0.2343), 47 (+0.1562) |

## zero

| Metric | Unit | 变差的 seeds (Δ) |
| --- | --- | --- |
| during_estimate_rms_m | m | 7 (+0.045555), 17 (+0.060756), 37 (+0.061467), 47 (+0.28828) |
| during_tracking_rms_m | m | 7 (+0.044503), 17 (+0.049515), 37 (+0.056267), 47 (+0.26584) |
| full_estimate_max_m | m | 7 (observed Δ +5.6663), 17 (observed Δ +5.9228), 27 (observed Δ +6.3085) |
| full_tracking_max_m | m | 7 (observed Δ +0.69981), 17 (observed Δ +1.9741), 27 (observed Δ +2.2511) |
| x_clean_rejection_rate | % / Δ percentage points | 7 (+4.6171), 17 (+4.9252), 27 (+5.6159), 37 (+3.2714), 47 (+4.0207) |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+90), 17 (+90.141), 27 (+90.062), 37 (+90.11), 47 (+90.179) |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+90), 17 (+90.141), 27 (+90.062), 37 (+90.11), 47 (+90.179) |
| x_all_forced | scalar updates | 7 (+13), 17 (+14), 27 (+16), 37 (+9), 47 (+11) |
| x_post_5s_clean_forced | scalar updates | 7 (+13), 17 (+14), 27 (+16), 37 (+9), 47 (+11) |
| x_post_all_clean_forced | scalar updates | 7 (+13), 17 (+14), 27 (+16), 37 (+9), 47 (+11) |
| y_clean_rejection_rate | % / Δ percentage points | 7 (+2.1986), 17 (+4.4874), 27 (+0.36232), 37 (+2.8996), 47 (+0.11066) |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+41.538), 17 (+81.69), 27 (+1.2422), 37 (+78.022), 47 (+0.89286) |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+41.538), 17 (+81.69), 27 (+1.2422), 37 (+78.022), 47 (+0.89286) |
| y_all_forced | scalar updates | 7 (+1), 17 (+5), 37 (+3) |
| y_post_5s_clean_forced | scalar updates | 7 (+1), 17 (+5), 37 (+3) |
| y_post_all_clean_forced | scalar updates | 7 (+1), 17 (+5), 37 (+3) |
| pooled_clean_rejection_rate | % / Δ percentage points | 7 (+3.4078), 17 (+4.7063), 27 (+2.9891), 37 (+3.0855), 47 (+2.0657) |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+65.769), 17 (+85.915), 27 (+45.652), 37 (+84.066), 47 (+45.536) |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+65.769), 17 (+85.915), 27 (+45.652), 37 (+84.066), 47 (+45.536) |
| pooled_all_forced | scalar updates | 7 (+14), 17 (+19), 27 (+16), 37 (+12), 47 (+11) |
| pooled_post_5s_clean_forced | scalar updates | 7 (+14), 17 (+19), 27 (+16), 37 (+12), 47 (+11) |
| pooled_post_all_clean_forced | scalar updates | 7 (+14), 17 (+19), 27 (+16), 37 (+12), 47 (+11) |

## dropout

| Metric | Unit | 变差的 seeds (Δ) |
| --- | --- | --- |
| during_estimate_rms_m | m | 7 (+0.05474) |
| during_tracking_rms_m | m | 7 (+0.021733), 27 (+0.014493) |
| post_5s_estimate_rms_m | m | 7 (+0.070827), 27 (+0.021163) |
| post_5s_tracking_rms_m | m | 7 (+0.066857), 27 (+0.018345) |
| full_estimate_max_m | m | 7 (+0.085073) |
| full_tracking_max_m | m | 7 (+0.059097), 27 (+0.11009), 37 (+0.0080162) |
| recovery_estimate_s | s | 37 (+0.99) |
| recovery_tracking_s | s | 7 (gated censored), 37 (+0.99) |
| recovery_joint_s | s | 37 (+0.99) |
| x_clean_rejection_rate | % / Δ percentage points | 7 (+0.25862), 17 (+0.2931), 27 (+0.2931), 37 (+0.25862), 47 (+0.17241) |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 17 (+0.8), 37 (+0.8) |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.18744), 17 (+0.3124), 27 (+0.21868), 37 (+0.28116), 47 (+0.06248) |
| y_clean_rejection_rate | % / Δ percentage points | 7 (+0.15517), 17 (+0.2931), 27 (+0.31034), 37 (+0.22414), 47 (+0.17241) |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.2), 17 (+0.4), 27 (+0.6), 37 (+0.4), 47 (+0.6) |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.093721), 17 (+0.3124), 27 (+0.3124), 37 (+0.18744), 47 (+0.24992) |
| pooled_clean_rejection_rate | % / Δ percentage points | 7 (+0.2069), 17 (+0.2931), 27 (+0.30172), 37 (+0.24138), 47 (+0.17241) |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.1), 17 (+0.6), 27 (+0.3), 37 (+0.6), 47 (+0.3) |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.14058), 17 (+0.3124), 27 (+0.26554), 37 (+0.2343), 47 (+0.1562) |

## scale 0.5

| Metric | Unit | 变差的 seeds (Δ) |
| --- | --- | --- |
| during_estimate_rms_m | m | 7 (+0.038539), 17 (+0.0075237), 37 (+0.049895), 47 (+0.0009267) |
| during_tracking_rms_m | m | 7 (+0.037916), 37 (+0.03561) |
| post_5s_estimate_rms_m | m | 7 (+0.072947), 17 (+0.070305), 37 (+0.12641), 47 (+0.0063779) |
| post_5s_tracking_rms_m | m | 7 (+0.073623), 17 (+0.073452), 37 (+0.12739), 47 (+0.0053404) |
| full_estimate_max_m | m | 7 (+0.076542), 17 (+0.071354), 37 (+0.1365), 47 (+0.0082784) |
| full_tracking_max_m | m | 7 (+0.070657), 17 (+0.046097), 37 (+0.12989), 47 (+0.028846) |
| x_clean_rejection_rate | % / Δ percentage points | 7 (+0.31034), 17 (+0.32759), 27 (+0.39655), 37 (+0.2931), 47 (+0.2069) |
| x_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.6), 17 (+1.2), 27 (+1.2), 37 (+1.2), 47 (+0.4) |
| x_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.28116), 17 (+0.37488), 27 (+0.40612), 37 (+0.34364), 47 (+0.12496) |
| y_clean_rejection_rate | % / Δ percentage points | 7 (+0.13793), 17 (+0.2931), 27 (+0.31034), 37 (+0.22414), 47 (+0.17241) |
| y_post_5s_clean_rejection_rate | % / Δ percentage points | 17 (+0.4), 27 (+0.6), 37 (+0.4), 47 (+0.6) |
| y_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.06248), 17 (+0.3124), 27 (+0.3124), 37 (+0.18744), 47 (+0.24992) |
| pooled_clean_rejection_rate | % / Δ percentage points | 7 (+0.22414), 17 (+0.31034), 27 (+0.35345), 37 (+0.25862), 47 (+0.18966) |
| pooled_post_5s_clean_rejection_rate | % / Δ percentage points | 7 (+0.3), 17 (+0.8), 27 (+0.9), 37 (+0.8), 47 (+0.5) |
| pooled_post_all_clean_rejection_rate | % / Δ percentage points | 7 (+0.17182), 17 (+0.34364), 27 (+0.35926), 37 (+0.26554), 47 (+0.18744) |

## 截短 run 的 observed max

这些是已观察到的 max differences；没有覆盖 62 s 的 run 不代表完整 62 s 的表现。indeterminate 的两侧都已截短，不能据此断言完整 horizon 的差异。

| Case | Seed | Metric | Observed Δ m | Complete-horizon outcome |
| --- | --- | --- | --- | --- |
| zero | 7 | full_estimate_max_m | +5.6663 | worse |
| zero | 17 | full_estimate_max_m | +5.9228 | worse |
| zero | 27 | full_estimate_max_m | +6.3085 | worse |
| zero | 37 | full_estimate_max_m | +4.4851 | indeterminate |
| zero | 47 | full_estimate_max_m | +5.8453 | indeterminate |
| zero | 7 | full_tracking_max_m | +0.69981 | worse |
| zero | 17 | full_tracking_max_m | +1.9741 | worse |
| zero | 27 | full_tracking_max_m | +2.2511 | worse |
| zero | 37 | full_tracking_max_m | +0.022509 | indeterminate |
| zero | 47 | full_tracking_max_m | +0.011225 | indeterminate |

## 原有 flight checks 的 regression

| Case | Seed | Check | Stock | Gated |
| --- | --- | --- | --- | --- |
| spike20 | 27 | final_mean_target_error_below_50_cm | pass | fail |
| spike50 | 27 | final_mean_target_error_below_50_cm | pass | fail |
| zero | 7 | completed_demo | pass | fail |
| zero | 7 | final_mean_speed_below_25_cm_s | pass | fail |
| zero | 17 | completed_demo | pass | fail |
| zero | 17 | final_mean_speed_below_25_cm_s | pass | fail |
| zero | 27 | completed_demo | pass | fail |
| zero | 27 | final_mean_speed_below_25_cm_s | pass | fail |
